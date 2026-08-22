#!/usr/bin/env python3
"""비상 제동 (AEB) — 플래너와 독립된 최후 안전 계층.

플래너·회피 로직이 죽거나 헛돌아도 살아 있어야 하므로, 의도적으로 아주 단순하게
짰다. `/scan` 과 실측 속도만 보고 충돌까지 남은 시간(iTTC)을 재서 `/emergency_brake`
를 올린다.

  iTTC_i = r_i / max(closing_rate_i, eps),   closing_rate_i = v · cos(θ_i)

주행 코리도 밖(옆 벽 등)의 빔은 버린다. 이게 없으면 코너 진입마다 정면 벽 때문에
계속 오작동한다. 조향각을 알면 코리도를 그 곡률로 휘고, 모르면 직선으로 둔다
(직선 가정이 더 보수적 = 더 잘 멈춤 → fail-safe).

**맵 필터**: 코리도만으로는 부족하다. 레이싱라인은 일부러 벽에 붙어 달리므로
(이 트랙은 섬 벽에서 0.36 m) 코너를 앞두면 직선 코리도가 반드시 벽을 문다.
그래서 빔 끝점을 맵에 대조해 "이미 아는 벽" 이면 버린다. AEB 가 잡아야 하는 건
맵에 없는 것 — 갑자기 나타난 차·사람·콘 이다. 아는 벽을 피하는 건 경로계획의
일이고, 그쪽은 이미 맵 기반으로 검사한다.
맵이나 TF 가 없으면 필터를 끄고 전부 위험으로 본다 (fail-safe).

control_node 가 `/emergency_brake` 를 받아 실제 제동을 건다. 이 노드는 판단만 한다.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String

from path_following import vehicle_geometry as vg
from path_following.avoidance_safety import InflatedMap
from path_following.viz_gate import has_listener


# ============================================================
# USER TUNING — 비상 제동 (여기만 수정)
# ============================================================
CFG = {
    "scan_topic": "/scan",
    "speed_topic": "/vehicle/speed_mps",
    "drive_topic": "/drive",            # 조향각만 참고 (코리도를 휘는 용도)
    "brake_topic": "/emergency_brake",
    "ttc_topic": "/emergency_brake/ttc",  # 튜닝용. 현재 최소 TTC [s]
    # ---- 트리거 조건 ----
    # 이 시간 안에 부딪힐 것 같으면 제동. 낮출수록 늦게 개입(오작동↓, 위험↑).
    # 제동거리 = v²/(2a) 를 시간으로 보면 v/(2a). 3m/s·7m/s² 면 약 0.21 s 이므로
    # 그보다 여유를 둔 값이어야 실제로 멈출 수 있다.
    "ttc_threshold_s": 0.55,
    # 이 속도 미만이면 TTC 판정을 끈다 (정지 직전 덜덜거림 방지).
    # 단 아래 min_standoff_m 는 속도와 무관하게 살아 있다 — 안 그러면 감속할 때마다
    # TTC 판정이 꺼져서 조금씩 전진하다 결국 장애물을 들이받는다.
    "arm_speed_mps": 0.4,
    # 코리도 안 장애물과 최소 이격 [m]. 속도 무관. 기어가듯 다가가는 걸 막는다.
    #
    # 주의: 이 값은 **라이다 원점 기준** 이다. _evaluate 가 스캔 range 를 그대로
    # closest 로 쓰기 때문이다(closest = r.min()). 라이다는 base_link 에서
    # 0.31 m 앞이고 차 앞끝은 0.50 m 이므로, 앞끝은 라이다보다 0.19 m 더 앞에
    # 있다. 즉 여기 적은 값에서 0.19 를 빼야 실제 범퍼 여유다.
    #
    # 20260816 재산정 — 앞끝 기준으로 원하는 여유를 정하고 0.19 를 더한다.
    #   평상시 트리거: 범퍼 여유 0.12  -> 0.19+0.12 = 0.31 (이전 0.30, 거의 동일)
    #   해제:          범퍼 여유 0.22  -> 0.19+0.22 = 0.41 (이전 0.40)
    # 평상시 값이 거의 안 바뀌는 게 중요하다 — AEB 는 좀처럼 안 걸려야 한다.
    "min_standoff_m": round(vg.LASER_TO_FRONT_M + 0.12, 3),   # 이전 0.30
    "standoff_release_m": round(vg.LASER_TO_FRONT_M + 0.22, 3),  # 이전 0.40
    # ---- 데드락 탈출구 ----
    # 여유 안쪽에 서 버렸을 때의 탈출구.
    # 정지 상태가 이만큼 이어지면 해제한다. 선 차는 아무것도 못 받으므로
    # 제동을 붙들고 있을 안전상 이유가 없고, 다시 다가가면 standoff 가
    # 재트리거한다. 이게 없으면 위 계산이 빗나간 순간 영구 정지가 된다.
    "stuck_release_sec": 1.5,
    "stuck_speed_mps": 0.05,
    # ---- 탈출 창 (재발동 방지) ----
    # stuck 으로 풀어 줘도 차가 아직 그 자리면 다음 틱에 standoff 가 다시 물어서
    # "1.9초 제동 → 0.02초 해제 → 재제동" 루프가 된다. 제동이 풀린 시간이
    # 20 ms 뿐이라 차는 영원히 못 빠져나온다.
    #
    # 그래서 stuck 으로 풀린 직후에는 일정 시간/거리 동안 재발동을 막는다.
    # 그동안 FGM+플래너가 탈출 경로를 찾아 실제로 빠져나가야 한다.
    # 무한정 막지 않도록 네 가지로 창을 닫는다:
    #   (1) escape_min_travel_m 만큼 실제로 이동   (2) escape_max_sec 경과
    #   (3) escape_speed_end_mps 이상으로 가속     (4) 아래 hard_stop 침범
    # (4) 는 창 안에서도 살아 있는 절대 방어선이다 — 이것까지 없으면
    # 탈출한답시고 벽으로 그대로 들어간다.
    "escape_enable": True,
    "escape_min_travel_m": 0.35,
    "escape_max_sec": 3.0,
    "escape_speed_end_mps": 1.0,
    # 탈출 창 안에서도 살아 있는 절대 방어선. 이것도 라이다 기준이라,
    # 0.12 는 범퍼가 장애물을 7 cm 지나간 지점이었다 (0.12 - 0.19 < 0).
    # 범퍼 여유 5 cm 를 남긴다.
    "escape_hard_stop_m": round(vg.LASER_TO_FRONT_M + 0.05, 3),  # 이전 0.12
    # ---- 후진 탈출 ----
    # 장애물 코앞에 서면 전진으로는 못 나간다. 버블 반각이
    # asin((장애물반경+버블+차반폭)/거리) 라 거리가 그 반경(약 0.55) 안이면 90° —
    # 최대 조향을 줘도 갈 데가 없다. 0.4 m 만 물러나도 반각이 90°→50° 로
    # 떨어져 전진 탈출이 그때부터 가능해진다.
    #
    # 그래서 전진 탈출 창이 **아무 진전 없이 시간 초과** 로 닫히면 (= 최대
    # 조향으로도 못 나갔다는 뜻) 곧게 뒤로 물러난 뒤 다시 전진 탈출을 시킨다.
    # 조향은 집행하는 control_node 가 중립으로 잡는다 — 꺾인 채 후진하면
    # 뒤가 어디로 갈지 예측이 안 된다.
    "reverse_escape_enable": True,
    # 탈출 창이 시간 초과까지 가 주면 위 판정으로 충분한데, 실차에서는 거기까지
    # 가지도 못했다. `closest` 가 `escape_hard_stop_m` 과 같은 값에 걸터앉으면
    # (둘 다 0.24) 창이 열린 다음 틱에 hard_stop 침범으로 닫힌다:
    #
    #   탈출 창 시작 #105 → 0.02 s 뒤 EMERGENCY BRAKE [STANDOFF] → 반복
    #
    # 실제로 #105 까지 돌았다. 제동이 풀린 시간이 한 틱뿐이라 차는 조향만
    # 파르르 떨며 그 자리에 서 있고, 시간 초과 분기는 영영 안 온다.
    #
    # 그래서 창과 무관하게 본다: **서 있고 앞이 막혔으면** 물러난다. 창이 어떤
    # 사유로 닫히든, AEB 가 켜져 있든 꺼져 있든 상관없다. 서 있다는 것 자체가
    # 전진으로 못 나간다는 증거다.
    "reverse_stuck_sec": 1.0,
    # 이 안에 뭔가 있어야 "앞이 막혀서" 선 것으로 본다 (라이다 기준).
    # 범퍼 기준 0.6 m — 그 밖이면 못 가는 이유가 장애물이 아니다.
    "reverse_stuck_obstacle_m": round(vg.LASER_TO_FRONT_M + 0.60, 3),
    # **코앞** 이면 위 1.0 초를 안 기다린다. 범퍼 앞 0.25 m 안에 뭐가 있는데
    # 차가 서 있으면 더 볼 것이 없다 — 전진으로 못 나가는 게 이미 확정이다.
    # (버블 반각이 asin(0.55/거리) 라 이 거리에서는 90°, 즉 갈 데가 없다.)
    # 다만 0 은 아니다. VESC 속도가 순간 0 을 찍는 것과 진짜 정지를 가르려면
    # 몇 틱은 봐야 한다.
    "reverse_close_obstacle_m": round(vg.LASER_TO_FRONT_M + 0.25, 3),
    "reverse_close_stuck_sec": 0.3,
    # 물러난 직후엔 이만큼 다시 안 건다. 플래너가 나갈 기회를 줘야 하고,
    # 안 그러면 뒤가 빌 때까지 계속 뒷걸음질한다.
    # 한 번에 0.2 m 씩 끊어 물러나므로(아래) 한 걸음으로 안 열리는 경우가 있다.
    # 그때 3 초를 기다리면 상자 앞에서 오래 굳어 보인다.
    "reverse_cooldown_sec": 1.5,
    # 한 걸음에 이만큼 물러난다.
    #
    # 예전엔 0.40 이었다. 전진 탈출이 열리는 기하학적 최소치가 그쯤이라
    # (반각 90°→50°) 한 번에 끝내려던 값이다. 그런데 뒤 여유를 0.60 m 나
    # 요구하게 되고, 트랙 폭을 생각하면 그 조건이 자주 안 맞아서 "그대로
    # 선다" 로 빠졌다.
    #
    # 지금은 짧게 끊는다. 한 걸음으로 안 열리면 쿨다운 뒤 또 한 걸음 물러난다.
    # 필요한 만큼 가되, 매번 뒤 여유를 다시 확인하고 가는 셈이라 뒤가 좁은
    # 자리에서도 할 수 있는 만큼은 한다.
    "reverse_travel_m": 0.20,
    "reverse_max_sec": 2.0,
    # 뒤가 이만큼 안 비어 있으면 시작하지 않는다 (범퍼 기준).
    # 한 걸음(0.20) + 중단 임계(0.25) 는 있어야 도중에 안 끊긴다.
    "reverse_min_clearance_m": 0.45,
    # 후진 중 뒤 여유가 여기까지 줄면 즉시 중단. 벽에 대고 밀면 안 된다.
    "reverse_abort_clearance_m": 0.25,
    # 맵에서 "차가 여기 들어가나" 를 볼 때 반폭에 더할 여유. 뒤는 안 보고
    # 가는 거라 조향이 조금만 남아 있어도 실제 궤적이 옆으로 샌다.
    "reverse_map_margin_m": 0.10,
    "reverse_topic": "/aeb/escape_reverse",
    # 노이즈 빔 하나로 급정거하지 않도록, 조건을 만족하는 빔이 이 개수 이상일 때만
    "min_hit_beams": 3,
    # ---- 주행 코리도 (오작동의 대부분이 여기서 갈린다) ----
    # 실측 반폭 0.15. 이전 0.17 은 어림값이었다. 유효 코리도 반폭
    # (half_width + margin) 은 0.22 로 그대로 유지해서 오작동률을 안 건드린다.
    "ego_half_width_m": vg.HALF_WIDTH_M,  # 이전 0.17
    "corridor_margin_m": 0.07,            # 이전 0.05 (합계 0.22 유지)
    "fov_deg": 100.0,              # 전방 ±50°만 검사
    "max_range_m": 6.0,
    "min_range_m": 0.05,
    # 조향각으로 코리도를 휘어 코너 진입 오작동을 줄인다. /drive 가 끊기면
    # 자동으로 직선 코리도(더 보수적)로 되돌아간다.
    "use_steering_corridor": True,
    "wheelbase_m": vg.WHEELBASE_M,
    "drive_stale_sec": 0.3,
    # ---- 맵 필터 (아는 벽은 AEB 대상이 아니다) ----
    # 빔 끝점이 맵의 점유 셀에서 이 거리 안이면 "아는 벽" 으로 보고 버린다.
    # 측위 오차 + 맵 해상도(0.05m) + LiDAR 오차를 덮을 만큼은 돼야 하지만,
    # 벽에 바짝 붙은 진짜 장애물까지 지워버릴 만큼 크면 안 된다.
    "map_filter_enable": True,
    "map_topic": "/map",
    "map_frame": "map",
    "map_match_tol_m": 0.22,
    "map_tf_timeout_sec": 0.05,
    # 이 거리보다 가까우면 맵에 뭐라고 적혀 있든 위험으로 본다. "마지막 한 뼘"
    # 방지선이지 안전여유가 아니다 — 레이싱라인이 벽에서 0.36 m 로 달리고 LiDAR 가
    # 축보다 0.275 m 앞에 있어서, 헤어핀에서 빔이 0.25 m 까지 내려간다. 이 값을
    # 그보다 크게 잡으면 코너마다 맵 필터가 무력화돼 급정거한다.
    "map_filter_bypass_m": 0.10,
    # ---- 회피 계층과의 간섭 조정 ----
    # 회피 중에는 일부러 장애물 옆을 스치듯 지난다. AEB 를 평상시 기준 그대로
    # 두면 회피를 시작하는 순간(조향이 아직 안 실려 코리도가 장애물을 물고 있을
    # 때) 제동이 걸려 회피가 죽는다. 그래서 AVOID/REJOIN 동안만 임계를 낮춘다.
    #
    # 단 "끄지" 는 않는다. AEB 는 최후 방어선이라 플래너 버그로 무력화되면
    # 의미가 없다. scale 을 아무리 낮춰도 아래 floor 밑으로는 안 내려가고,
    # /planner/mode 가 끊기면 자동으로 평상시(엄격) 기준으로 돌아간다.
    "planner_mode_topic": "/planner/mode",
    "mode_stale_sec": 0.5,
    "avoid_modes": ["AVOID", "REJOIN"],
    "avoid_ttc_scale": 0.6,
    "avoid_ttc_floor_s": 0.25,      # 이 아래로는 절대 완화 안 됨
    "avoid_standoff_scale": 0.6,
    # 이 아래로는 절대 완화 안 됨. 라이다 기준이라 이전 0.18 은 범퍼가 장애물에
    # 닿은 지점(0.18 - 0.19 < 0)이었다. 회피 중에는 여유를 줄여도 되지만
    # 음수는 안 된다. 범퍼 여유 6 cm 를 남긴다.
    "avoid_standoff_floor_m": round(vg.LASER_TO_FRONT_M + 0.06, 3),  # 이전 0.18
    # ---- 히스테리시스 ----
    # 한 번 걸리면 최소 이만큼 유지 (채터링 방지)
    "min_hold_sec": 0.4,
    # 해제 조건을 트리거보다 느슨하게 (TTC 가 이 배율 이상 회복돼야 풀림)
    "release_ttc_factor": 1.5,
    "timer_period_sec": 0.02,
    "status_log_hz": 1.0,          # 트리거/해제 로그 (0=끔)
}


class EmergencyBrakeNode(Node):
    # 후방 검사 간격. 맵 해상도(보통 0.05)보다 잘게 볼 이유가 없다.
    _REVERSE_PROBE_STEP_M = 0.05

    def __init__(self) -> None:
        super().__init__("emergency_brake_node")
        for key, value in CFG.items():
            self.declare_parameter(key, value)

        g = self.get_parameter
        self.ttc_threshold = max(1e-3, float(g("ttc_threshold_s").value))
        self.arm_speed = max(0.0, float(g("arm_speed_mps").value))
        self.min_standoff = max(0.0, float(g("min_standoff_m").value))
        self.standoff_release = max(
            self.min_standoff, float(g("standoff_release_m").value)
        )
        self.min_hit_beams = max(1, int(g("min_hit_beams").value))
        self.half_width = max(0.0, float(g("ego_half_width_m").value))
        self.corridor_margin = max(0.0, float(g("corridor_margin_m").value))
        self.fov_rad = math.radians(max(1.0, float(g("fov_deg").value)))
        self.max_range = float(g("max_range_m").value)
        self.min_range = float(g("min_range_m").value)
        self.use_steering_corridor = bool(g("use_steering_corridor").value)
        self.wheelbase = max(1e-3, float(g("wheelbase_m").value))
        self.drive_stale_ns = int(float(g("drive_stale_sec").value) * 1e9)
        self.min_hold_sec = max(0.0, float(g("min_hold_sec").value))
        self.release_factor = max(1.0, float(g("release_ttc_factor").value))
        self.stuck_release_sec = max(0.0, float(g("stuck_release_sec").value))
        self.stuck_speed = max(0.0, float(g("stuck_speed_mps").value))
        self._stopped_since = 0.0

        self.escape_enable = bool(g("escape_enable").value)
        self.escape_min_travel = max(0.0, float(g("escape_min_travel_m").value))
        self.escape_max_sec = max(0.0, float(g("escape_max_sec").value))
        self.escape_speed_end = max(0.0, float(g("escape_speed_end_mps").value))
        self.escape_hard_stop = max(0.0, float(g("escape_hard_stop_m").value))
        self._escape_until = 0.0     # 0 = 창 닫힘
        self._escape_travel = 0.0
        self._escape_count = 0
        self._last_tick = 0.0

        self.reverse_enable = bool(g("reverse_escape_enable").value)
        self.reverse_travel = max(0.05, float(g("reverse_travel_m").value))
        self.reverse_max_sec = max(0.1, float(g("reverse_max_sec").value))
        self.reverse_min_clearance = max(0.0, float(g("reverse_min_clearance_m").value))
        self.reverse_abort_clearance = max(
            0.0, float(g("reverse_abort_clearance_m").value)
        )
        self.reverse_map_margin = max(0.0, float(g("reverse_map_margin_m").value))
        self.reverse_stuck_sec = max(0.0, float(g("reverse_stuck_sec").value))
        self.reverse_close_obstacle = max(
            0.0, float(g("reverse_close_obstacle_m").value)
        )
        self.reverse_close_stuck_sec = max(
            0.0, float(g("reverse_close_stuck_sec").value)
        )
        self.reverse_stuck_obstacle = max(
            0.0, float(g("reverse_stuck_obstacle_m").value)
        )
        self.reverse_cooldown_sec = max(0.0, float(g("reverse_cooldown_sec").value))
        self._reverse_until = 0.0    # 0 = 후진 안 함
        self._reverse_travel = 0.0
        self._reverse_count = 0
        self._reverse_ready_at = 0.0  # 이 시각 전에는 다시 안 건다
        self._idle_since = 0.0        # 속도가 0 으로 떨어진 시각

        self._speed = 0.0
        self._steering = 0.0
        self._drive_recv_ns = 0
        self._scan: LaserScan | None = None
        self._beam_angles: np.ndarray | None = None

        # 히스테리시스 폭은 유지한 채로 임계만 옮긴다
        self.standoff_gap = max(0.0, self.standoff_release - self.min_standoff)
        self.avoid_modes = {
            str(m).strip().upper() for m in (g("avoid_modes").value or [])
        }
        self.mode_stale_ns = int(max(0.0, float(g("mode_stale_sec").value)) * 1e9)
        self.avoid_ttc_scale = min(1.0, max(0.0, float(g("avoid_ttc_scale").value)))
        self.avoid_ttc_floor = max(1e-3, float(g("avoid_ttc_floor_s").value))
        self.avoid_standoff_scale = min(
            1.0, max(0.0, float(g("avoid_standoff_scale").value))
        )
        self.avoid_standoff_floor = max(0.0, float(g("avoid_standoff_floor_m").value))

        self._active = False
        self._trigger_time = 0.0
        self._last_ttc = float("inf")
        self._mode = ""
        self._mode_recv_ns = 0

        self.map_filter_enable = bool(g("map_filter_enable").value)
        self.map_frame = str(g("map_frame").value)
        self.map_match_tol = max(0.0, float(g("map_match_tol_m").value))
        self.map_tf_timeout = max(0.0, float(g("map_tf_timeout_sec").value))
        self.map_filter_bypass = max(0.0, float(g("map_filter_bypass_m").value))
        self._map: InflatedMap | None = None
        self._tf_buffer = None
        self._tf_listener = None
        self._warned_no_map = False
        # 후진도 맵과 TF 로 뒤를 판단한다 (라이다가 뒤를 못 본다). 그래서
        # map_filter 를 꺼도 후진을 쓰려면 맵이 있어야 한다.
        if self.map_filter_enable or self.reverse_enable:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
            self.create_subscription(
                OccupancyGrid,
                str(g("map_topic").value),
                self._map_cb,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )

        self.brake_pub = self.create_publisher(
            Bool, str(g("brake_topic").value), 10
        )
        self.ttc_pub = self.create_publisher(Float64, str(g("ttc_topic").value), 10)
        self.reverse_pub = self.create_publisher(
            Bool, str(g("reverse_topic").value), 10
        )

        self.create_subscription(
            LaserScan, str(g("scan_topic").value), self._scan_cb, 10
        )
        self.create_subscription(
            Float64, str(g("speed_topic").value), self._speed_cb, 10
        )
        self.create_subscription(
            String, str(g("planner_mode_topic").value), self._mode_cb, 10
        )
        if self.use_steering_corridor:
            self.create_subscription(
                AckermannDriveStamped,
                str(g("drive_topic").value),
                self._drive_cb,
                10,
            )

        self.create_timer(float(g("timer_period_sec").value), self._timer_cb)

        self.get_logger().info(
            f"Emergency brake (AEB) | ttc<{self.ttc_threshold:.2f}s "
            f"(arm>{self.arm_speed:.2f}m/s) "
            # 라이다 기준값과 범퍼 기준 실제 여유를 같이 찍는다. 앞끝이 라이다보다
            # 0.19 m 앞이라, 이 둘을 헷갈리면 여유를 그만큼 착각한다.
            f"standoff<{self.min_standoff:.2f}m(라이다)"
            f"={self.min_standoff - vg.LASER_TO_FRONT_M:+.2f}m(범퍼) "
            f"beams>={self.min_hit_beams} "
            f"corridor=±{self.half_width + self.corridor_margin:.2f}m "
            f"fov=±{math.degrees(self.fov_rad) / 2:.0f}° "
            f"steer_corridor={self.use_steering_corridor} "
            f"hold={self.min_hold_sec:.2f}s | "
            f"map_filter={self.map_filter_enable}"
            f"(tol {self.map_match_tol:.2f}m, bypass<{self.map_filter_bypass:.2f}m) | "
            f"{'/'.join(sorted(self.avoid_modes)) or '-'} 중 완화 "
            f"→ ttc>{self.avoid_ttc_floor:.2f}s standoff>{self.avoid_standoff_floor:.2f}m "
            f"→ {g('brake_topic').value}"
        )

    # ------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg

    def _speed_cb(self, msg: Float64) -> None:
        value = float(msg.data)
        if math.isfinite(value):
            self._speed = abs(value)

    def _mode_cb(self, msg: String) -> None:
        self._mode = str(msg.data).strip().upper()
        self._mode_recv_ns = self.get_clock().now().nanoseconds

    def _avoiding(self) -> bool:
        """플래너가 회피 중인가. 토픽이 끊기면 False = 평상시 엄격 기준."""
        if not self.avoid_modes or self._mode_recv_ns == 0:
            return False
        age = self.get_clock().now().nanoseconds - self._mode_recv_ns
        if age > self.mode_stale_ns:
            return False
        return self._mode in self.avoid_modes

    def _thresholds(self) -> tuple[float, float, bool]:
        """(TTC 임계, 최소 이격, 완화중 여부).

        회피 중이면 완화하되 floor 아래로는 내려가지 않는다.
        """
        if not self._avoiding():
            return self.ttc_threshold, self.min_standoff, False
        return (
            max(self.avoid_ttc_floor, self.ttc_threshold * self.avoid_ttc_scale),
            max(
                self.avoid_standoff_floor,
                self.min_standoff * self.avoid_standoff_scale,
            ),
            True,
        )

    def _drive_cb(self, msg: AckermannDriveStamped) -> None:
        value = float(msg.drive.steering_angle)
        if math.isfinite(value):
            self._steering = value
            self._drive_recv_ns = self.get_clock().now().nanoseconds

    # ------------------------------------------------------------
    def _current_steering(self) -> float:
        """조향각. /drive 가 끊기면 0(직선) — 코리도가 넓어져 더 보수적."""
        if not self.use_steering_corridor:
            return 0.0
        age = self.get_clock().now().nanoseconds - self._drive_recv_ns
        if self._drive_recv_ns == 0 or age > self.drive_stale_ns:
            return 0.0
        return self._steering

    def _beam_angle_array(self, scan: LaserScan) -> np.ndarray:
        n = len(scan.ranges)
        if self._beam_angles is None or len(self._beam_angles) != n:
            self._beam_angles = scan.angle_min + np.arange(n) * scan.angle_increment
        return self._beam_angles

    def _evaluate(self, ttc_threshold: float) -> tuple[float, int, float]:
        """(최소 iTTC, TTC 임계 미만 빔 수, 코리도 내 최근접 거리).

        TTC 는 속도가 arm_speed 이상일 때만 의미가 있지만, 최근접 거리는
        속도와 무관하게 항상 낸다.
        """
        none = (float("inf"), 0, float("inf"))
        scan = self._scan
        if scan is None:
            return none
        if self.map_filter_enable and self._map is None and not self._warned_no_map:
            self._warned_no_map = True
            self.get_logger().warn(
                "map_filter_enable=True 인데 맵이 아직 없다 — 아는 벽도 위험으로 "
                "보므로 코너에서 오제동 가능. 맵이 오면 자동 해제된다."
            )

        ranges = np.asarray(scan.ranges, dtype=float)
        angles = self._beam_angle_array(scan)
        if len(ranges) != len(angles):
            return none

        ok = (
            np.isfinite(ranges)
            & (ranges >= self.min_range)
            & (ranges <= self.max_range)
            & (np.abs(angles) <= self.fov_rad * 0.5)
        )
        if not np.any(ok):
            return none

        r = ranges[ok]
        th = angles[ok]
        x = r * np.cos(th)
        y = r * np.sin(th)

        # 주행 코리도: 차가 지금 조향각으로 계속 간다고 보고 그 궤적에서
        # 반폭 이내인 점만 위험으로 본다.
        limit = self.half_width + self.corridor_margin
        delta = self._current_steering()
        if abs(delta) < 1e-3:
            in_corridor = np.abs(y) <= limit
        else:
            radius = self.wheelbase / math.tan(delta)  # +좌회전 / -우회전
            # 회전 중심 (0, R) 에서의 거리와 |R| 의 차이가 곧 궤적으로부터의 이탈
            in_corridor = (
                np.abs(np.hypot(x, y - radius) - abs(radius)) <= limit
            )
        # 뒤쪽으로 향한 빔은 제외 (전방만)
        in_corridor &= x > 0.0
        if not np.any(in_corridor):
            return none

        r = r[in_corridor]
        th = th[in_corridor]
        r, th = self._drop_known_walls(r, th, x[in_corridor], y[in_corridor])
        if r.size == 0:
            return none
        closest = float(r.min())

        if self._speed < self.arm_speed:
            return float("inf"), 0, closest

        closing = self._speed * np.cos(th)
        approaching = closing > 1e-3
        if not np.any(approaching):
            return float("inf"), 0, closest

        ttc = r[approaching] / closing[approaching]
        return (
            float(ttc.min()),
            int(np.count_nonzero(ttc < ttc_threshold)),
            closest,
        )

    def _rear_clearance(self) -> float:
        """뒤 범퍼가 벽에 닿기 전까지 물러날 수 있는 거리 [m]. 모르면 0.

        **라이다는 뒤를 못 본다.** 처음엔 스캔의 후방 섹터를 쟀는데, 뒤 빔이
        아예 없으니 대부분 inf(비었다)가 나오고 FOV 가장자리 잡음이 하나
        들어오면 0.00 이 나왔다. 그래서 실차에서 이렇게 됐다:

            후진 탈출 시작 #23 — 뒤 여유 inf m
            후진 탈출 종료 — 뒤가 막혔다 (0.00m)   ← 0.02 s 뒤

        20 ms 만에 끝나니 차는 찔끔 움직이고 만다.

        그래서 맵과 TF 로 본다. 지금 위치에서 뒤로 훑으며 차폭이 들어가는
        곳까지의 거리를 잰다. `clearance_at` 은 그 점에서 가장 가까운 벽까지
        거리라, 차 반폭+여유보다 크면 그 지점은 차가 지나갈 수 있다.

        **못 재면 0 이다 (전진 검사와 반대).** 앞은 안 보이면 라이다가 어떻게든
        보지만 뒤는 볼 수단이 이것뿐이라, 낙관하면 눈 감고 후진하는 셈이 된다.
        맵이나 TF 가 없으면 아예 물러나지 않는다.

        한계가 하나 있다. 맵에 없는 물건(사람, 새로 놓인 상자)은 뒤에 있어도
        모른다. 그래서 후진 거리를 0.4 m 로 짧게, 속도도 기는 수준으로 묶어
        둔다.
        """
        gmap = self._map
        if gmap is None:
            return 0.0
        tf = self._lookup_laser_to_map()
        if tf is None:
            return 0.0

        cos_t, sin_t, tx, ty = tf
        need = vg.HALF_WIDTH_M + self.reverse_map_margin
        # 필요한 만큼만 본다 — 더 멀리 재 봐야 쓰지도 않는다.
        limit = self.reverse_travel + self.reverse_min_clearance
        steps = int(limit / self._REVERSE_PROBE_STEP_M) + 1
        for k in range(steps + 1):
            d = min(k * self._REVERSE_PROBE_STEP_M, limit)
            # 라이다 프레임에서 뒤끝보다 d 만큼 더 뒤 (차는 중심선 위에 있다)
            lx = -(vg.LASER_TO_REAR_M + d)
            if gmap.clearance_at(lx * cos_t + tx, lx * sin_t + ty) < need:
                return d
        return limit

    def _update_reverse(self, now: float, dt: float) -> bool:
        """후진 요청 상태 갱신. True 면 이번 틱에 물러난다.

        시작 조건은 `_update_escape_window` 가 잡는다 (전진 탈출이 아무 진전
        없이 시간 초과). 여기서는 끝내는 조건만 본다 — 충분히 물러났거나,
        시간이 다 됐거나, 뒤가 좁아졌거나.
        """
        if self._reverse_until <= 0.0:
            return False

        self._reverse_travel += abs(self._speed) * dt
        rear = self._rear_clearance()

        reason = ""
        backed_off = False
        if rear < self.reverse_abort_clearance:
            reason = f"뒤가 막혔다 ({rear:.2f}m)"
        elif self._reverse_travel >= self.reverse_travel:
            reason = f"{self._reverse_travel:.2f}m 물러남 — 전진 재시도"
            backed_off = True
        elif now >= self._reverse_until:
            reason = f"시간 초과 ({self._reverse_travel:.2f}m 물러남)"
            backed_off = self._reverse_travel > 0.0

        if not reason:
            return True

        self._reverse_until = 0.0
        self._reverse_travel = 0.0
        self._reverse_ready_at = now + self.reverse_cooldown_sec
        self._idle_since = 0.0
        self.get_logger().warn(f"후진 탈출 종료 — {reason}")
        if backed_off:
            # 물러난 만큼 앞이 열렸다. 그 자리에서 standoff 가 다시 물기
            # 전에 전진 탈출을 한 번 더 시켜 준다.
            self._open_escape_window(now)
        return False

    def _stuck_against_something(self, now: float, closest: float) -> str:
        """서 있고 앞이 막혔는가. 막혔으면 사유 문자열, 아니면 "".

        탈출 창의 종료 사유를 안 본다. 창이 hard_stop 으로 한 틱 만에 닫히는
        경우가 실제로 있었고 (`closest` 가 임계와 같은 값에 걸터앉을 때),
        그러면 시간 초과 판정에 영원히 도달하지 못한다. 여기서는 결과만
        본다 — 서 있으면 못 나가는 것이다.

        기다리는 시간은 얼마나 가까운지로 갈린다. 코앞(범퍼 0.25 m 안)이면
        전진 탈출이 열릴 여지가 기하학적으로 없으므로 오래 볼 이유가 없다.
        """
        if self._speed > self.stuck_speed:
            self._idle_since = 0.0
            return ""
        if self._idle_since <= 0.0:
            self._idle_since = now
            return ""

        idle = now - self._idle_since
        if closest < self.reverse_close_obstacle:
            if idle >= self.reverse_close_stuck_sec:
                return f"코앞 {closest:.2f}m 에 서 있다"
            return ""
        if idle < self.reverse_stuck_sec:
            return ""
        if closest < self.reverse_stuck_obstacle:
            return f"{idle:.1f}s 째 못 나간다 (앞 {closest:.2f}m)"
        return ""

    def _maybe_start_reverse(self, now: float, why: str = "전진으로 못 나갔다") -> None:
        """전진으로 못 나간다 — 뒤가 비었으면 물러난다."""
        if not (self.reverse_enable and self._reverse_until <= 0.0):
            return
        if now < self._reverse_ready_at:
            return
        rear = self._rear_clearance()
        if rear < self.reverse_min_clearance:
            # 앞뒤 다 막혔다. 이 상태는 매 틱 참이므로 쿨다운을 걸어야
            # 로그가 20 Hz 로 쏟아지지 않는다.
            self._reverse_ready_at = now + self.reverse_cooldown_sec
            self.get_logger().warn(
                f"전진도 후진도 막혔다 — 뒤 여유 {rear:.2f}m "
                f"< {self.reverse_min_clearance:.2f}m. 그대로 선다"
            )
            return
        self._idle_since = 0.0
        self._reverse_until = now + self.reverse_max_sec
        self._reverse_travel = 0.0
        self._reverse_count += 1
        self.get_logger().warn(
            f"후진 탈출 시작 #{self._reverse_count} — {why}. "
            f"뒤 여유 {rear:.2f}m, {self.reverse_travel:.2f}m 물러난다"
        )

    # ------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid) -> None:
        try:
            # inflation 은 안 쓴다 (clearance_at 만 필요). unknown 은 벽으로 치지
            # 않는다 — 미지 영역에 새로 놓인 장애물을 놓치면 안 되니까.
            self._map = InflatedMap(msg, 0.0, include_unknown=False)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"맵 처리 실패, 맵 필터 미적용: {exc}")
            return
        self.get_logger().info(
            f"AEB 맵 필터 준비 — {msg.info.width}x{msg.info.height} "
            f"@{msg.info.resolution:.3f}m/px, 허용오차 {self.map_match_tol:.2f}m"
        )

    def _drop_known_walls(self, r, th, x, y):
        """맵에 이미 있는 벽을 맞은 빔을 버린다.

        AEB 는 "맵에 없는 것" 만 잡아야 한다. 레이싱라인은 벽에 붙어 달리므로
        아는 벽까지 세면 코너마다 급정거한다.
        """
        gmap = self._map
        if gmap is None or not self.map_filter_enable or r.size == 0:
            return r, th
        tf = self._lookup_laser_to_map()
        if tf is None:
            return r, th

        cos_t, sin_t, tx, ty = tf
        mx = x * cos_t - y * sin_t + tx
        my = x * sin_t + y * cos_t + ty

        j = ((mx - gmap.ox) / gmap.res).astype(np.int32)
        i = ((my - gmap.oy) / gmap.res).astype(np.int32)
        inside = (i >= 0) & (j >= 0) & (i < gmap.h) & (j < gmap.w)
        clr = np.full(r.shape, np.inf, dtype=np.float32)
        if np.any(inside):
            clr[inside] = gmap.clearance[i[inside], j[inside]]
        # 맵 밖(inside=False)은 clearance=inf → 아는 벽이 아님 → 남긴다 (보수적)
        known_wall = clr <= self.map_match_tol
        keep = ~known_wall | (r <= self.map_filter_bypass)
        return r[keep], th[keep]

    def _lookup_laser_to_map(self):
        """(cos, sin, tx, ty) 형태의 map<-scan_frame 2D 변환."""
        scan = self._scan
        if scan is None or self._tf_buffer is None:
            return None
        try:
            t = self._tf_buffer.lookup_transform(
                self.map_frame,
                scan.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.map_tf_timeout),
            )
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (
            math.cos(yaw),
            math.sin(yaw),
            t.transform.translation.x,
            t.transform.translation.y,
        )

    def _open_escape_window(self, now: float) -> None:
        """stuck 해제 직후 재발동 억제 구간 시작."""
        if not self.escape_enable or self.escape_max_sec <= 0.0:
            return
        self._escape_until = now + self.escape_max_sec
        self._escape_travel = 0.0
        self._escape_count += 1
        self.get_logger().warn(
            f"AEB 탈출 창 시작 #{self._escape_count} — "
            f"{self.escape_max_sec:.1f}s / {self.escape_min_travel:.2f}m 안에 "
            f"빠져나가야 한다 (hard_stop {self.escape_hard_stop:.2f}m 는 계속 살아 있음)"
        )

    def _update_escape_window(self, now: float, dt: float, closest: float) -> bool:
        """탈출 창 갱신. True 면 이번 틱은 재발동을 억제한다.

        이동거리는 속도 적분으로 잰다. TF 가 끊겨도 동작해야 하고, 여기서
        알고 싶은 건 "그 자리에 그대로인가" 라서 경로長이면 충분하다.
        """
        if self._escape_until <= 0.0:
            return False

        self._escape_travel += self._speed * dt

        reason = ""
        stalled = False
        if closest < self.escape_hard_stop:
            reason = f"hard_stop 침범 ({closest:.2f}m)"
        elif self._escape_travel >= self.escape_min_travel > 0.0:
            reason = f"탈출 성공 ({self._escape_travel:.2f}m 이동)"
        elif self._speed >= self.escape_speed_end > 0.0:
            reason = f"정상 주행 복귀 ({self._speed:.2f}m/s)"
        elif now >= self._escape_until:
            # 시간을 다 쓰고도 거의 제자리면 최대 조향으로도 못 나간 것이다.
            # 조금이라도 나아갔으면 느릴 뿐이니 후진까지 갈 일은 아니다.
            stalled = self._escape_travel < 0.5 * self.escape_min_travel
            reason = (
                f"시간 초과 — 못 빠져나감 ({self._escape_travel:.2f}m)"
            )

        if not reason:
            return True

        self._escape_until = 0.0
        self._escape_travel = 0.0
        self.get_logger().info(f"AEB 탈출 창 종료 — {reason}")
        if stalled:
            self._maybe_start_reverse(now)
        return False

    def _timer_cb(self) -> None:
        ttc_thr, standoff, relaxed = self._thresholds()
        ttc, hits, closest = self._evaluate(ttc_thr)
        self._last_ttc = ttc
        now = self.get_clock().now().nanoseconds * 1e-9

        ttc_hit = hits >= self.min_hit_beams
        too_close = closest < standoff

        dt = now - self._last_tick if self._last_tick > 0.0 else 0.0
        if not (0.0 < dt < 0.5):
            dt = 0.0
        self._last_tick = now
        escaping = self._update_escape_window(now, dt, closest)
        # 창이 어떻게 닫히든, 서 있는데 앞이 막혔으면 물러난다. 창의 시간
        # 초과만 보던 때는 hard_stop 으로 한 틱 만에 닫히는 경우를 통째로
        # 놓쳤다 (실차에서 탈출 창 #105 까지 헛돌았다).
        if self._reverse_until <= 0.0:
            stuck_why = self._stuck_against_something(now, closest)
            if stuck_why:
                self._maybe_start_reverse(now, stuck_why)
        reversing = self._update_reverse(now, dt)
        # 후진이 끝나면서 전진 창을 새로 열었을 수 있다. 위 줄이 이미 지나간
        # 뒤라, 이걸 안 보면 그 한 틱에 standoff 가 물어 창이 헛돈다.
        if not reversing and self._escape_until > 0.0:
            escaping = True

        if reversing:
            # 물러나는 동안은 앞을 보고 제동하지 않는다. 앞이 가까운 건
            # 이미 아는 사실이고, 그래서 뒤로 가는 중이다. 여기서 제동을
            # 걸면 후진이 그대로 막힌다. 뒤쪽 안전은 `_update_reverse` 가
            # 여유를 계속 재서 지킨다.
            if self._active:
                self._active = False
                self._stopped_since = 0.0
            self.brake_pub.publish(Bool(data=False))
            self._publish_ttc(ttc)
            self.reverse_pub.publish(Bool(data=True))
            return

        if not self._active:
            # 탈출 창 안에서는 재발동을 막는다. 단 hard_stop 안쪽은 예외 —
            # 그건 탈출이 아니라 그냥 박기 직전이다.
            if (ttc_hit or too_close) and not escaping:
                self._active = True
                self._trigger_time = now
                self._stopped_since = 0.0
                cause = "TTC" if ttc_hit else "STANDOFF"
                mode = f" mode={self._mode}(완화)" if relaxed else ""
                self.get_logger().warn(
                    f"EMERGENCY BRAKE [{cause}] — ttc={ttc:.2f}s beams={hits} "
                    f"closest={closest:.2f}m v={self._speed:.2f}m/s{mode}"
                )
        else:
            held = now - self._trigger_time >= self.min_hold_sec
            # 해제는 트리거보다 느슨하게. 두 조건 모두 회복해야 푼다.
            clear = (
                ttc >= ttc_thr * self.release_factor
                and closest >= standoff + self.standoff_gap
            )
            # 정지 지속 시간. 여유 안쪽에 서 버려 clear 가 영영 안 서는 경우의
            # 탈출구다 — 선 차는 제동을 붙들 이유가 없고, 다시 다가가면
            # standoff 가 재트리거한다.
            if self._speed <= self.stuck_speed:
                if self._stopped_since <= 0.0:
                    self._stopped_since = now
            else:
                self._stopped_since = 0.0
            stuck = (
                self.stuck_release_sec > 0.0
                and self._stopped_since > 0.0
                and now - self._stopped_since >= self.stuck_release_sec
            )

            if held and (clear or stuck):
                self._active = False
                self._stopped_since = 0.0
                if stuck and not clear:
                    self._open_escape_window(now)
                why = "clear" if clear else "stuck-escape"
                log = (
                    self.get_logger().info
                    if clear
                    else self.get_logger().warn
                )
                log(
                    f"emergency brake released [{why}] — ttc={ttc:.2f}s "
                    f"closest={closest:.2f}m v={self._speed:.2f}m/s"
                )

        self.brake_pub.publish(Bool(data=self._active))
        self._publish_ttc(ttc)
        self.reverse_pub.publish(Bool(data=False))

    def _publish_ttc(self, ttc: float) -> None:
        """TTC 텔레메트리. 듣는 데가 없으면 내지 않는다 (Foxglove 전용)."""
        if not has_listener(self.ttc_pub):
            return
        self.ttc_pub.publish(Float64(data=(ttc if math.isfinite(ttc) else -1.0)))


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrakeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
