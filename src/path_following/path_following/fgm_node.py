#!/usr/bin/env python3
"""
FGM (Follow the Gap Method) 노드.

회피의 주 경로 생성기:
  - 장애 접근/AVOID 중 /planner/fgm_enable=True → 스캔 FOV 갭 추종
  - 벽–벽 갭 중심, 장애 있으면 버블로 장애–벽 갭으로 자연 전환
  - REJOIN 은 local_planner 의 CSV 복귀 보조 (여기선 enable OFF)

/scan + (/static_obstacles 버블) + /planner/fgm_enable → /fgm_target

실차: 시뮬과 동일 알고리즘. laser_frame 만 "laser".
"""
from __future__ import annotations

import math
import subprocess
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64
from visualization_msgs.msg import Marker, MarkerArray

from path_following import vehicle_geometry as vg
from path_following.viz_gate import has_listener

_FGM_BOOST_FLAG = Path("/tmp/f1tenth_fgm_boost")
_CPU_POLICY = Path("/home/nvidia/f1tenth_ajou/scripts/apply_cpu_policy.sh")


# ============================================================
# USER TUNING — FGM 파라미터 (여기만 수정)
# launch에서 같은 이름으로 넣면 launch 값이 우선.
# ============================================================
CFG = {
    # Topics
    "scan_topic": "/scan",
    "laser_frame": "laser",  # 실차 (시뮬: ego_racecar/laser)
    "obstacle_topic": "/static_obstacles",
    "dynamic_obstacle_topic": "/dynamic_obstacles",
    "fgm_enable_topic": "/planner/fgm_enable",
    "ego_speed_topic": "/vehicle/speed_mps",
    # [탈출 조준 기준] local_planner 가 AEB 정지 후 보내는 [기준각, 허용 콘]
    # (rad, 차체 기준). 안 오거나 빈 배열이면 여기 모든 "정면 선호" 는
    # 평소대로 0° 를 뜻한다.
    "prefer_angle_topic": "/planner/fgm_prefer_angle",
    "prefer_stale_sec": 0.5,
    # True면 local_planner 의 /planner/fgm_enable 이 True 일 때만 갭 계산
    "require_planner_enable": True,
    "target_topic": "/fgm_target",
    "publish_debug_scan": False,
    # Foxglove/RViz 갭 마커. enable ON일 때만 계산·발행 (OFF면 스캔 스킵 유지).
    "publish_gap_marker": True,
    # 스캔 전처리·갭 (알고리즘)
    # 정면(레이저 +x) 기준 ±fov_half_deg 만 사용. ≤0 이면 스캔 전체.
    # Slamtec 0~360° 스캔도 wrap 후 정면 기준으로 자름.
    "fov_half_deg": 90.0,
    # 속도가 붙으면 FOV 를 좁힌다 (`_fov_for_speed`).
    #
    # FGM 은 갭만 보고 각을 고르므로 저속에서는 45~60° 도 정답이다. 그런데
    # 그 각은 그대로 요구 조향이 되고, 고속에서는 낼 수 없는 값이라 차가
    # 못 따라간다. 게다가 `_avoid_target_speed` 의 maneuver 항이 "그 조향을
    # 낼 수 있는 속도" 로 답하면서 속도까지 깎는다 — 조준각이 클수록 더 선다.
    #
    # 그래서 고속에서는 애초에 낼 수 있는 각만 후보로 둔다. 조준거리
    # L=v·target_lead_time_s 의 점을 향해 도는 데 필요한 횡가속이 순수추종
    # 기준 a = 2v²·sinψ/L 이므로
    #
    #     sin ψ ≤ a·L / (2 v²)
    #
    # 이다. a=4.5, L=0.7·v 면 5 m/s 에서 18°, 6 m/s 에서 15°, 7 m/s 에서 13°.
    #
    # `fov_narrow_speed` 아래는 손대지 않는다 — 저속 FGM 은 넓은 각이 있어야
    # 코너나 막힌 곳에서 빠져나온다. 문턱에서 각이 튀면 조준도 같이 튀므로
    # `fov_narrow_blend` 구간에 걸쳐 서서히 좁힌다.
    "fov_speed_narrow_enable": True,
    "fov_narrow_speed": 4.0,
    "fov_narrow_blend": 1.0,
    "fov_narrow_a_lat": 4.5,
    "fov_half_min_deg": 12.0,  # 이보다 좁히지는 않는다
    # 고속에선 멀리까지 봐야 갭이 미리 보임 (목표점 거리보다 넉넉하게)
    # 이 거리 밖은 전부 "뚫린 것" 으로 뭉갠다. 짧으면 멀리 목표를 못 찍어
    # 고속에서 회피가 급해진다 (target_max_m 이 이 안쪽이어야 의미가 있다).
    "scan_max_range_m": 10.0,
    # [장애물 버블] 장애 반경 위에 더하는 여유 (갭이 장애에서 얼마나 떨어질지).
    "bubble_radius_m": 0.2,
    # [A6] 고정 0.2 m 는 고속에서 너무 작고 저속에선 과하다. 켜면 속도에 비례해
    # 버블을 키운다 — /vehicle/speed_mps 는 이미 구독 중이라 추가 구독은 없다.
    # bubble = clip(base + gain*v, min, max)
    "bubble_speed_scale_enable": False,
    "bubble_base_m": 0.18,
    "bubble_speed_gain_s": 0.035,
    "bubble_min_m": 0.18,
    "bubble_max_m": 0.40,
    # [차량 버블] 뒷축 기준 발자국 — 전방 길이 / 좌우 폭(전체).
    # 폭은 FGM 섹터 반폭에 half_width로 들어가고, 전방은 planner 게이트(d−front)와 공유.
    #
    # 20260816: 두 값 다 실측과 어긋나 있었다.
    #   ego_safety_width_m 은 "전체 폭" 인데 0.15 (= 실제 반폭) 가 들어가 있어서,
    #   코드가 반으로 나눈 결과 섹터 반폭이 0.075 m — 실제의 절반이었다. 버블이
    #   그만큼 좁아 갭을 실제보다 넓게 봤다. 실측 전폭 0.30 으로 바로잡는다.
    #   ego_front_safety_m 은 실제 앞끝이 0.50 인데 0.30 이었다.
    "ego_front_safety_m": vg.FRONT_M,       # 이전 0.30
    "ego_safety_width_m": vg.WIDTH_M,       # 이전 0.15 (반폭이 잘못 들어가 있었음)
    "gap_threshold_primary_m": 1.5,
    "gap_threshold_fallback_m": 0.5,
    # 빔 개수가 아니라 각폭 기준 (라이다 분해능 바뀌어도 동일 동작)
    "min_gap_width_deg": 6.0,
    "gap_hysteresis_len_ratio": 0.78,
    # 갭 안에서 목표 각도를 가장자리로부터 얼마나 안쪽에 둘지.
    # 버블이 이미 차폭+여유를 먹고 있어 크게 줄 필요 없다 (각도라서 멀수록 과해짐).
    "gap_edge_inset_deg": 3.0,
    # ---- 목표점 주행폭 검증 (회피 직후 벽 긁힘 방지) ----
    # 갭 선택은 각도 기준이라, 목표 방향으로 실제 "차폭이 들어가는지"는 따로 봐야
    # 한다. edge_inset 은 각도라 멀수록 실제 여유가 줄어든다 (3° 는 3 m 에서
    # 겨우 0.16 m). 차폭 코리도를 훑어 막히는 지점까지만 목표를 찍고, 그래도
    # 부족하면 갭 안에서 더 뚫린 각도로 목표를 옮긴다.
    "corridor_check_enable": True,
    # 차량 반폭 + 여유. 회피 경로는 급하게 휘므로 직선 반폭이 아니라 코너
    # 스윕폭을 기준으로 잡는다 (반경 1 m 에서 0.254). 이전 0.22 는 우연히
    # 근사값이었는데, 이제는 치수에서 유도한다.
    "corridor_half_width_m": vg.PATH_CHECK_HALF_WIDTH_M,  # 이전 0.22
    "corridor_stop_margin_m": 0.15,  # 막히는 지점에서 이만큼 앞에 멈춰 찍는다
    "corridor_angle_samples": 11,    # 갭 안에서 시도할 목표 각도 후보 수
    # 크게 꺾는 데 매기는 벌점 [m/rad]. 키우면 정면 고집(회피 소극적),
    # 줄이면 여유 공간을 찾아 과감히 튼다.
    "corridor_straight_bias_m_per_rad": 1.0,
    # ---- "여유가 이만큼이면 충분하다" 는 기준 [m] ----
    # 점수는 min(여유, want) − bias·|각도| 다. want 는 안전 문턱이 아니라
    # **보상이 포화되는 지점** 이다. 여기를 넘으면 더 뚫려 있어도 점수가 안
    # 올라가므로, 그때부터는 벌점이 이겨서 정면에 가까운 각도가 선택된다.
    #
    # 예전엔 여기에 목표점 거리(target_dist, 최대 5 m)를 그대로 넣었다.
    # 그러면 굽은 통로에서 정면은 몇 미터 앞 벽에 막히고 옆으로 틀수록 멀리
    # 뚫리니, 여유 1 m 더 벌자고 45° 트는 게 이득이 된다 (45°의 벌점은 겨우
    # 0.79 m). 실측 조준각이 26~54° 까지 나왔고, r=0.20 콘 하나에 라인에서
    # 1.2~1.5 m 씩 벗어났다 (필요량 0.45 m 의 2.5~3.3 배).
    #
    # 안전은 여기가 아니라 버블(장애물에서 r+0.20+차폭반폭 확보)과 AEB 가
    # 맡는다. 그래서 이 값은 "이 방향으로 잠깐 가도 되나" 수준이면 된다.
    "corridor_want_time_s": 0.5,
    "corridor_want_min_m": 1.5,
    "corridor_want_max_m": 3.0,
    # 갭 **선택** 단계에서도 차폭을 본다.
    #
    # 위 코리도 검사는 이미 고른 갭 **안에서만** 각도를 옮긴다. 그래서 고른
    # 갭 자체가 차폭이 안 들어가는 통로면 빠져나갈 길이 없다 — 실측
    # (20260822) 에서 4.0 m/s 로 4.1 m 앞 박스를 만난 차가 -30° 를 겨눴는데
    # 그 방향 여유가 0.57 m 였다. 각도로만 고르면 "넓어 보이는데 못 지나가는"
    # 갭이 이긴다.
    #
    # 그래서 후보 갭마다 최선의 여유를 재서, 안 들어가는 갭은 후보에서 뺀다.
    # 전부 안 들어가면 예전대로 둔다 — 못 지나갈 때 판단을 바꿔 봐야 나아질
    # 게 없고, 그때는 감속과 AEB 몫이다.
    "gap_fit_check_enable": True,
    "gap_fit_samples": 5,        # 갭당 훑을 각도 수 (양 끝 포함)
    "gap_fit_min_m": 1.0,        # 이만큼도 안 뚫렸으면 후보에서 뺀다
    # 목표점 거리 = clamp(ego_speed * lead_time, min, max) [m, 레이저 프레임]
    "target_lead_time_s": 0.70,
    "target_min_m": 1.0,
    # 고속 회피의 실질 병목. 목표점이 가까우면 같은 횡오프셋도 곡률이 커져
    # 횡가속도 한계에 먼저 걸린다 (3.5 m 목표로 0.5 m 틀면 4.9 m/s 가 천장,
    # 5.0 m 면 7.0 m/s). 대신 멀수록 반응이 느려지므로 scan_max_range_m 안쪽.
    "target_max_m": 5.0,
    # 목표 스무딩: EMA 1단 + 이동 속도 제한 [m/s] (고속일수록 자동 완화)
    "target_smooth_alpha": 0.70,
    "target_max_rate_mps": 3.5,
    # RViz/Foxglove V자 갭 마커 (주행과 무관, 표시만)
    "gap_marker_arm_scale": 1.5,
    "gap_marker_max_arm_m": 2.0,
    # 맵 OccupancyGrid 가 z=0 평면이라 선을 그 위에 띄운다.
    # 예전에 토픽은 나가는데 Foxglove 에서만 안 보이던 이유가 이거였다.
    "gap_marker_z_m": 0.15,
}


def _param_bool(val) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).lower() in ("1", "true", "yes")


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _wrap_pi_np(a: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a), np.cos(a))


class FGMNode(Node):
    def __init__(self):
        super().__init__("fgm_node")

        for key, value in CFG.items():
            self.declare_parameter(key, value)

        scan_t = self.get_parameter("scan_topic").value
        obs_t = self.get_parameter("obstacle_topic").value
        tgt_t = self.get_parameter("target_topic").value
        self._laser_frame = str(self.get_parameter("laser_frame").value)
        self.require_planner_enable = _param_bool(
            self.get_parameter("require_planner_enable").value
        )
        fgm_en_t = str(self.get_parameter("fgm_enable_topic").value)

        self.scan_sub = self.create_subscription(LaserScan, scan_t, self.scan_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray, obs_t, self.obstacle_callback, 10
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("dynamic_obstacle_topic").value),
            self.dynamic_obstacle_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("ego_speed_topic").value),
            self.speed_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("prefer_angle_topic").value),
            self.prefer_callback,
            10,
        )
        self._fgm_enabled = not self.require_planner_enable
        if self.require_planner_enable:
            self.fgm_enable_sub = self.create_subscription(
                Bool, fgm_en_t, self.fgm_enable_callback, 10
            )

        self.target_pub = self.create_publisher(PointStamped, tgt_t, 10)
        self.publish_debug_scan = _param_bool(self.get_parameter("publish_debug_scan").value)
        self.publish_gap_marker = _param_bool(self.get_parameter("publish_gap_marker").value)
        self.debug_scan_pub = (
            self.create_publisher(LaserScan, "/fgm_debug_scan", 10)
            if self.publish_debug_scan
            else None
        )
        self.gap_marker_pub = (
            self.create_publisher(Marker, "/fgm_gap_marker", 10)
            if self.publish_gap_marker
            else None
        )
        # Foxglove Scene 은 MarkerArray 를  Cubes(/visualization_marker_array)
        # 와 같은 경로로 그린다. 단일 Marker 토픽은 구독은 되는데 안 그리는
        # 경우가 있어서 둘 다 낸다. 레이아웃에 이미 있는 /fgm_gap_marker 는
        # 그대로 두고, /fgm_gap_markers 는 장애물 마커와 같은 타입이다.
        self.gap_markers_pub = (
            self.create_publisher(MarkerArray, "/fgm_gap_markers", 10)
            if self.publish_gap_marker
            else None
        )

        self.preprocess_dist = float(self.get_parameter("scan_max_range_m").value)
        self.bubble_radius = float(self.get_parameter("bubble_radius_m").value)
        self.bubble_speed_scale = _param_bool(
            self.get_parameter("bubble_speed_scale_enable").value
        )
        self.bubble_base_m = float(self.get_parameter("bubble_base_m").value)
        self.bubble_speed_gain_s = float(
            self.get_parameter("bubble_speed_gain_s").value
        )
        self.bubble_min_m = float(self.get_parameter("bubble_min_m").value)
        self.bubble_max_m = max(
            self.bubble_min_m, float(self.get_parameter("bubble_max_m").value)
        )
        self.ego_front_safety_m = max(
            0.0, float(self.get_parameter("ego_front_safety_m").value)
        )
        self.ego_safety_width_m = max(
            0.0, float(self.get_parameter("ego_safety_width_m").value)
        )
        self.ego_half_width_m = 0.5 * self.ego_safety_width_m
        self.gap_edge_inset_rad = math.radians(
            max(0.0, float(self.get_parameter("gap_edge_inset_deg").value))
        )

        self.fov_angle = math.radians(float(self.get_parameter("fov_half_deg").value))
        # ≤0 이면 FOV 크롭 안 함 (스캔 전방향)
        self._use_full_scan_fov = float(self.get_parameter("fov_half_deg").value) <= 0.0
        self.fov_speed_narrow = _param_bool(
            self.get_parameter("fov_speed_narrow_enable").value
        )
        self.fov_narrow_speed = max(
            0.0, float(self.get_parameter("fov_narrow_speed").value)
        )
        self.fov_narrow_blend = max(
            0.05, float(self.get_parameter("fov_narrow_blend").value)
        )
        self.fov_narrow_a_lat = max(
            0.3, float(self.get_parameter("fov_narrow_a_lat").value)
        )
        self.fov_half_min = math.radians(
            max(1.0, float(self.get_parameter("fov_half_min_deg").value))
        )
        # 이번 스캔에 실제로 쓰는 FOV. `scan_callback` 이 매 프레임 갱신한다.
        self._fov_rad = self.fov_angle
        self.gap_thr_primary = float(self.get_parameter("gap_threshold_primary_m").value)
        self.gap_thr_fallback = float(self.get_parameter("gap_threshold_fallback_m").value)
        self.target_lead_time_s = max(
            0.0, float(self.get_parameter("target_lead_time_s").value)
        )
        self.corridor_check_enable = bool(
            self.get_parameter("corridor_check_enable").value
        )
        self.corridor_half_width = max(
            0.01, float(self.get_parameter("corridor_half_width_m").value)
        )
        self.corridor_stop_margin = max(
            0.0, float(self.get_parameter("corridor_stop_margin_m").value)
        )
        self.corridor_angle_samples = max(
            3, int(self.get_parameter("corridor_angle_samples").value)
        )
        self.corridor_straight_bias = max(
            0.0, float(self.get_parameter("corridor_straight_bias_m_per_rad").value)
        )
        self.corridor_want_time_s = max(
            0.0, float(self.get_parameter("corridor_want_time_s").value)
        )
        self.corridor_want_min_m = max(
            0.2, float(self.get_parameter("corridor_want_min_m").value)
        )
        self.gap_fit_check_enable = bool(
            self.get_parameter("gap_fit_check_enable").value
        )
        self.gap_fit_samples = max(2, int(self.get_parameter("gap_fit_samples").value))
        self.gap_fit_min_m = max(0.0, float(self.get_parameter("gap_fit_min_m").value))
        self.corridor_want_max_m = max(
            self.corridor_want_min_m,
            float(self.get_parameter("corridor_want_max_m").value),
        )
        self.target_min_m = max(0.2, float(self.get_parameter("target_min_m").value))
        self.target_max_m = max(
            self.target_min_m, float(self.get_parameter("target_max_m").value)
        )
        self.min_gap_width_rad = math.radians(
            max(0.0, float(self.get_parameter("min_gap_width_deg").value))
        )
        self.min_gap_bins = 2
        self.hyst_ratio = min(
            0.999,
            max(0.3, float(self.get_parameter("gap_hysteresis_len_ratio").value)),
        )
        self.smooth_alpha = min(
            1.0, max(0.05, float(self.get_parameter("target_smooth_alpha").value))
        )
        self.target_max_rate_mps = max(
            0.0, float(self.get_parameter("target_max_rate_mps").value)
        )

        self.gap_marker_arm_scale = max(
            0.0, float(self.get_parameter("gap_marker_arm_scale").value)
        )
        _gmax = float(self.get_parameter("gap_marker_max_arm_m").value)
        self.gap_marker_max_arm_m = _gmax if _gmax > 0.0 else None
        self.gap_marker_z_m = float(self.get_parameter("gap_marker_z_m").value)
        self._last_gap_marker: Marker | None = None
        if self.publish_gap_marker:
            # 토픽을 노드 켜지는 순간부터 살려 둔다. foxglove_bridge 가
            # 우리보다 먼저 떠도 채널이 생기고, 배터리 갈고 노드만 다시
            # 켜도 브릿지가 토픽을 놓치지 않는다.
            self.create_timer(0.2, self._heartbeat_gap_marker)
            self._publish_gap_marker_delete()

        self.latest_obstacles: list = []
        self.latest_dynamic_obstacles: list = []
        self._last_gap_center_angle: float | None = None
        self._filt_x: float | None = None
        self._filt_y: float | None = None
        self._ego_speed = 0.0
        self._last_scan_ns: int | None = None
        self._last_corridor_warn_ns = 0
        self._last_gap_fit_counts = (0, 0)
        self._scan_positive = None
        self._scan_cx = None
        self._scan_cy = None
        self._xy_src_ranges = None
        self._xy_src_wrapped = None
        self._prefer_angle = 0.0
        self._prefer_cone = 0.0
        self._prefer_ns = 0
        self._prefer_stale_ns = int(
            max(0.05, float(self.get_parameter("prefer_stale_sec").value)) * 1e9
        )
        self._prefer_logged = False
        self._cpu_boost_active = False
        # 초기 enable 상태에 맞춰 CPU 우선순위 플래그 동기화
        self._set_cpu_boost(self._fgm_enabled)

        _bubble_desc = (
            f"{self.bubble_base_m}+{self.bubble_speed_gain_s}·v"
            f"∈[{self.bubble_min_m},{self.bubble_max_m}]m"
            if self.bubble_speed_scale
            else f"{self.bubble_radius}m"
        )
        self.get_logger().info(
            f"FGM started (sim algorithm) | frame={self._laser_frame}, "
            f"target=v*{self.target_lead_time_s}s "
            f"[{self.target_min_m}~{self.target_max_m}]m, "
            f"scan_max={self.preprocess_dist}m, "
            f"obs_bubble={_bubble_desc} "
            f"ego={self.ego_front_safety_m:.2f}m×{self.ego_safety_width_m:.2f}m, "
            f"edge_inset={math.degrees(self.gap_edge_inset_rad):.0f}°, "
            f"planner_enable={self.require_planner_enable}({fgm_en_t}), "
            f"fov={'FULL' if self._use_full_scan_fov else f'±{math.degrees(self.fov_angle):.0f}°'}, "
            f"marker scale={self.gap_marker_arm_scale} max={_gmax}m"
        )

    def obstacle_callback(self, msg: Float32MultiArray) -> None:
        self.latest_obstacles = list(msg.data)

    def dynamic_obstacle_callback(self, msg: Float32MultiArray) -> None:
        self.latest_dynamic_obstacles = list(msg.data)

    def _obstacle_sectors(self) -> list[tuple[float, float, float]]:
        """차단할 장애물 (x, y, radius) 목록 — 정적 [id,x,y,r], 동적 [id,x,y,vx,vy,r]."""
        out: list[tuple[float, float, float]] = []
        s = self.latest_obstacles
        for i in range(len(s) // 4):
            out.append((float(s[4 * i + 1]), float(s[4 * i + 2]), float(s[4 * i + 3])))
        d = self.latest_dynamic_obstacles
        for i in range(len(d) // 6):
            out.append((float(d[6 * i + 1]), float(d[6 * i + 2]), float(d[6 * i + 5])))
        return out

    def speed_callback(self, msg: Float64) -> None:
        self._ego_speed = abs(float(msg.data))

    def prefer_callback(self, msg: Float32MultiArray) -> None:
        """[기준각, 허용 콘] (rad). 빈 배열이면 선호 없음."""
        d = list(msg.data)
        if len(d) < 2 or not all(math.isfinite(float(v)) for v in d[:2]):
            self._prefer_ns = 0
            return
        self._prefer_angle = float(d[0])
        self._prefer_cone = max(0.0, float(d[1]))
        self._prefer_ns = self.get_clock().now().nanoseconds

    def _prefer_now(self) -> tuple[float, float] | None:
        """지금 유효한 (기준각, 콘). 없으면 None.

        기준각은 FOV 안으로 당긴다. 차가 콘을 벗어나게 돌아 버리면 기준이
        스캔 밖으로 나가는데, 그대로 두면 갭 선택이 늘 FOV 가장자리로 쏠린다.
        """
        if self._prefer_ns <= 0:
            return None
        if self.get_clock().now().nanoseconds - self._prefer_ns > self._prefer_stale_ns:
            return None
        a = self._prefer_angle
        if not self._use_full_scan_fov:
            a = max(-self.fov_angle, min(self.fov_angle, a))
        return a, self._prefer_cone

    def _log_prefer(self, active: bool, angle: float, cone: float) -> None:
        if active == self._prefer_logged:
            return
        self._prefer_logged = active
        if active:
            self.get_logger().warn(
                f"탈출 조준 제한 — 기준 {math.degrees(angle):+.0f}° "
                f"±{math.degrees(cone):.0f}°"
            )
        else:
            self.get_logger().info("탈출 조준 제한 해제 — 정면 기준 복귀")

    def _fov_for_speed(self) -> float:
        """이번 프레임에 쓸 FOV 반각 [rad].

        고속에서는 낼 수 있는 각만 후보로 둔다 — 자세한 근거는 CFG 주석 참고.
        `fov_narrow_speed` 아래는 설정값 그대로다.
        """
        if self._use_full_scan_fov or not self.fov_speed_narrow:
            return self.fov_angle
        v = abs(self._ego_speed)
        w = (v - self.fov_narrow_speed) / self.fov_narrow_blend
        w = min(1.0, max(0.0, w))
        if w <= 0.0:
            return self.fov_angle
        lead = max(self.target_min_m, v * self.target_lead_time_s)
        sin_max = self.fov_narrow_a_lat * lead / (2.0 * v * v)
        reach = math.asin(min(1.0, max(0.0, sin_max)))
        narrow = max(self.fov_half_min, min(self.fov_angle, reach))
        # 섞는 건 sin 이 아니라 각이다 — asin 은 sin→1 근처에서 수직이라
        # sin 을 섞으면 문턱 바로 위에서 각이 튄다.
        return self.fov_angle + w * (narrow - self.fov_angle)

    def _bubble_now(self) -> float:
        """[A6] 현재 속도에서의 장애물 버블 반경 [m]."""
        if not self.bubble_speed_scale:
            return self.bubble_radius
        b = self.bubble_base_m + self.bubble_speed_gain_s * self._ego_speed
        return min(self.bubble_max_m, max(self.bubble_min_m, b))

    def fgm_enable_callback(self, msg: Bool) -> None:
        was = self._fgm_enabled
        self._fgm_enabled = bool(msg.data)
        if self._fgm_enabled != was:
            self._set_cpu_boost(self._fgm_enabled)
        if was and not self._fgm_enabled:
            # 목표 스무딩만 리셋하고, Foxglove에 남은 마지막 갭 마커는
            # 투명 마커로 즉시 덮어쓴다. 캐시도 비워 heartbeat가 OFF 상태에서
            # 이전 마커를 다시 살리지 못하게 한다.
            self._reset_fgm_filter_state(keep_gap_hysteresis=True)
            self._last_gap_marker = None
            self._publish_gap_marker_delete()

    def _set_cpu_boost(self, enabled: bool) -> None:
        """FGM enable 시 패스 코어 1순위(nice -20). OFF면 기본(nice 5)으로 복귀."""
        if enabled == self._cpu_boost_active and _FGM_BOOST_FLAG.is_file():
            want = "1" if enabled else "0"
            try:
                if _FGM_BOOST_FLAG.read_text(encoding="utf-8").strip() == want:
                    return
            except OSError:
                pass
        self._cpu_boost_active = bool(enabled)
        try:
            _FGM_BOOST_FLAG.write_text("1\n" if enabled else "0\n", encoding="utf-8")
        except OSError as exc:
            self.get_logger().warning(f"FGM CPU boost flag write failed: {exc}")
            return
        # 즉시 적용 (데몬이 5초마다 같은 플래그를 존중)
        if _CPU_POLICY.is_file():
            try:
                subprocess.Popen(
                    ["sudo", "-n", "bash", str(_CPU_POLICY), "--once"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except OSError:
                pass
        self.get_logger().info(
            f"FGM CPU priority {'BOOST(-20)' if enabled else 'normal(5)'}"
        )

    def _reset_fgm_filter_state(self, *, keep_gap_hysteresis: bool = False) -> None:
        if not keep_gap_hysteresis:
            self._last_gap_center_angle = None
        self._filt_x = self._filt_y = None

    def _fill_marker_pose(self, m: Marker) -> None:
        # (0,0,0,0) 사원수는 무효. Foxglove Scene 은 이걸 드롭한다.
        # RViz 는 관대해서, 이쪽에서만 안 보이던 이유가 이거였다.
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.frame_locked = True
        # lifetime=0 = 안 지운다.
        #
        # 0.3 초를 줬더니 Foxglove 에서 안 보였다. 만료 판정은 뷰어가 **자기
        # 시계**로 하는데, 노트북과 젯슨 시계가 어긋나 있으면 도착하자마자
        # 만료된다 (젯슨은 RTC 배터리가 없어서 잘 어긋난다). 어차피 45 Hz 로
        # 덮어쓰고, FGM 이 꺼져도 하트비트가 같은 마커를 계속 낸다.
        m.lifetime = MsgDuration(sec=0, nanosec=0)

    def _publish_gap_marker_delete(self) -> None:
        """시작 때 한 번. 토픽을 살려 두는 빈 발행이지 화면에서 지우는 용이 아니다."""
        if self.gap_marker_pub is None and self.gap_markers_pub is None:
            return
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._laser_frame
        m.ns = "fgm_gap"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        self._fill_marker_pose(m)
        m.scale.x = 0.08
        m.color.a = 0.0
        if self.gap_marker_pub is not None:
            self.gap_marker_pub.publish(m)
        if self.gap_markers_pub is not None:
            arr = MarkerArray()
            arr.markers.append(m)
            self.gap_markers_pub.publish(arr)

    def _heartbeat_gap_marker(self) -> None:
        """FGM ON일 때만 마지막 V를 다시 내서 Foxglove 채널을 유지한다."""
        if not self._fgm_enabled:
            return
        if self._last_gap_marker is None:
            return
        m = self._last_gap_marker
        m.header.stamp = self.get_clock().now().to_msg()
        if has_listener(self.gap_marker_pub):
            self.gap_marker_pub.publish(m)
        if has_listener(self.gap_markers_pub):
            arr = MarkerArray()
            arr.markers.append(m)
            self.gap_markers_pub.publish(arr)

    def _select_gap(
        self,
        gaps: list,
        max_len: int,
        aim_idx: int | None = None,
        lock: bool = False,
        work_angles: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """따라갈 갭 하나. `aim_idx` 는 기준 방향에 해당하는 work 인덱스.

        폭이 `min_gap_bins` 를 넘긴 갭은 이미 버블을 통과했으므로 "차가
        지나갈 수 있다" 는 뜻이다. 그중에서는 **기준에 제일 가까운** 갭을
        고른다. 예전엔 제일 넓은 갭을 골랐는데, 트랙에서 각도상 제일 넓은
        방향은 보통 레이스라인이 아니라 트랙 건너편이다. 콘 하나 앞에서
        반대편을 잡으면 그 갭의 가장자리부터가 이미 멀어서, 뒤이은 각도
        선정이 아무리 정면을 선호해도 되돌릴 수 없다.

        직전 갭은 **각도**로 기억한다. work 배열은 FOV 안 빔만 모은 것이라,
        FOV 가 속도에 따라 변하면 같은 인덱스가 다른 각도를 가리킨다 —
        인덱스로 비교하면 속도가 흔들릴 때마다 엉뚱한 쪽을 당겨서 좌우로
        방황한다.

        `lock` 이면 직전 갭에 붙는 히스테리시스를 건너뛴다. AEB 탈출처럼
        기준 방향이 따로 주어진 상황에서는 한 번 옆 갭을 물면 계속 그쪽으로
        끌려가는데, 그게 정확히 막으려는 동작이다.
        """
        if not gaps:
            return None
        wide = [g for g in gaps if len(g) >= self.min_gap_bins]
        if not wide:
            wide = list(gaps)
        thresh_len = max(self.min_gap_bins, int(math.ceil(self.hyst_ratio * max_len)))

        def center_idx(g: np.ndarray) -> int:
            return int(g[len(g) // 2])

        def dist_to(g: np.ndarray, idx: int) -> int:
            """갭에서 idx 까지의 거리. 갭이 idx 를 품고 있으면 0."""
            lo, hi = int(g[0]), int(g[-1])
            if lo <= idx <= hi:
                return 0
            return lo - idx if idx < lo else idx - hi

        if (
            self._last_gap_center_angle is not None
            and work_angles is not None
            and not lock
        ):
            candidates = [g for g in wide if len(g) >= thresh_len]
            if not candidates:
                candidates = wide
            last = self._last_gap_center_angle
            return min(
                candidates,
                key=lambda g: abs(
                    _wrap_pi(float(work_angles[center_idx(g)]) - last)
                ),
            )

        if aim_idx is not None:
            # 동률이면 넓은 쪽 (가장자리를 스칠 위험이 적다)
            return min(wide, key=lambda g: (dist_to(g, aim_idx), -len(g)))
        return max(wide, key=lambda g: len(g))

    @staticmethod
    def _clamp_to_cone(
        lo: float, hi: float, center: float, cone: float
    ) -> tuple[float, float]:
        """갭 각도 범위 [lo, hi] 를 기준 ±콘 안으로 자른다.

        겹치면 겹치는 만큼만 남긴다. 문제는 안 겹칠 때다. 장애물 바로 앞에
        멈추면 버블 반각이 `asin((r+버블+차반폭)/거리)` 라 거리가 그 반경보다
        가까운 순간 90° — 정면이 통째로 막히고 갭은 FOV 끝에만 남는다. 그래서
        "안 겹침" 은 예외가 아니라 AEB 정지의 기본 상황이다.

        이때는 **콘 쪽 끝** 을 쓴다. 갭 끝을 쓰면 콘이 무력해져 옆으로 돌던
        예전 동작 그대로다. 검증된 갭 밖을 겨냥하는 셈이지만 탈출 한정으로
        받아들인다 — 속도가 0.8 m/s 이하고, AEB 가 그대로 살아 있고,
        `_pick_target_angle` 이 그 방향의 실제 코리도 여유를 다시 재서 막혀
        있으면 목표점을 코앞으로 당긴다. 즉 최악이 "조금 가다 다시 정지" 지,
        벽으로 도는 게 아니다.
        """
        clo, chi = center - cone, center + cone
        if hi < clo:
            return clo, clo
        if lo > chi:
            return chi, chi
        return max(lo, clo), min(hi, chi)

    def _smooth_target(self, tx: float, ty: float, dt: float) -> tuple[float, float]:
        """EMA 1단 + 이동 속도 제한.

        제한을 [m/frame] 대신 [m/s] 로 두어 스캔 Hz 변화에 영향받지 않고,
        고속 주행 시 갭이 빨리 흐르는 만큼 허용치도 함께 커진다.
        """
        if self._filt_x is None:
            self._filt_x, self._filt_y = tx, ty
            return float(tx), float(ty)

        px, py = self._filt_x, self._filt_y
        a = self.smooth_alpha
        nx = px + a * (tx - px)
        ny = py + a * (ty - py)

        max_step = (self.target_max_rate_mps + self._ego_speed) * max(dt, 1e-3)
        dx, dy = nx - px, ny - py
        dist = math.hypot(dx, dy)
        if max_step > 0.0 and dist > max_step:
            s = max_step / dist
            nx = px + dx * s
            ny = py + dy * s

        self._filt_x, self._filt_y = nx, ny
        return float(nx), float(ny)

    def _warn_corridor_blocked(self, angle: float, clear: float) -> None:
        """차폭이 안 들어가는 상황 경고 (1초에 한 번). 정지는 AEB 몫.

        후보 갭이 몇 개였고 그중 몇 개가 차폭을 통과했는지 같이 찍는다. 이게
        없으면 "고를 게 없었다" 와 "고를 수 있었는데 잘못 골랐다" 가 구분이
        안 되고, 둘은 손댈 곳이 정반대다 — 전자는 감속·AEB 문제고 후자만
        갭 선택 문제다. 적합성 필터는 후보가 하나뿐이거나 전부 떨어지면
        일부러 손대지 않으므로, 그 두 경우가 여기 그대로 올라온다.
        """
        now = self.get_clock().now().nanoseconds
        if now - self._last_corridor_warn_ns < 1_000_000_000:
            return
        self._last_corridor_warn_ns = now
        n_all, n_fit = self._last_gap_fit_counts
        if n_fit == 0:
            why = f"후보 {n_all}개 전부 안 들어감 — 지나갈 길이 없음"
        elif n_all < 2:
            why = "후보가 1개뿐이라 거를 수 없었음"
        else:
            why = f"후보 {n_all}개 중 {n_fit}개는 들어갔는데 그걸 못 고름"
        self.get_logger().warn(
            f"gap 은 열렸지만 차폭이 안 들어감 — aim={math.degrees(angle):+.0f}° "
            f"clear={clear:.2f}m < {self.target_min_m:.2f}m. 목표점을 당겨 찍음. "
            f"v={abs(self._ego_speed):.1f} 콘=±{math.degrees(self._fov_rad):.0f}° {why}"
        )

    def _corridor_clear_distance(
        self, geom_ranges: np.ndarray, wrapped: np.ndarray, angle: float
    ) -> float:
        """angle 방향으로 차폭 코리도가 뚫려 있는 거리 [m].

        목표 방향을 축으로 두고, 축에서 반폭 이내로 들어오는 점들 중 가장 가까운
        것까지의 전방거리를 낸다. 갭 판정이 각도 기준이라 놓치는 "멀리서 좁아지는
        통로"를 여기서 잡는다.
        """
        cx, cy = self._scan_xy(geom_ranges, wrapped)
        # 조준방향을 x 축으로 돌린 좌표. 빔마다 삼각함수를 부르던 걸 스칼라
        # 두 개로 바꾼다 — 회전은 스캔 전체에 공통이므로 원래 각도별로
        # 계산할 이유가 없었다.
        ca = math.cos(angle)
        sa = math.sin(angle)
        along = cx * ca + cy * sa
        perp = cy * ca - cx * sa

        blocking = (
            self._scan_positive
            & (along > 0.0)
            & (np.abs(perp) < self.corridor_half_width)
        )
        sel = along[blocking]
        if sel.size == 0:
            return self.preprocess_dist
        return max(0.0, float(sel.min()) - self.corridor_stop_margin)

    def _corridor_clear_reference(
        self, geom_ranges: np.ndarray, wrapped: np.ndarray, angle: float
    ) -> float:
        """빔마다 삼각함수를 부르는 원래 식. 회귀 테스트의 기준값이다.

        주행 경로에서는 안 쓴다 — `_corridor_clear_distance` 가 같은 답을
        훨씬 싸게 낸다. 대신 그게 정말 같은 답인지 확인할 무언가가 있어야
        해서 남겨 둔다. 지우면 최적화가 맞다는 걸 보일 방법이 없어진다.
        """
        d_ang = _wrap_pi_np(wrapped - angle)
        valid = (geom_ranges > 0.0) & (np.abs(d_ang) < math.pi * 0.5)
        if not np.any(valid):
            return self.preprocess_dist
        r = geom_ranges[valid]
        da = d_ang[valid]
        along = r * np.cos(da)
        perp = np.abs(r * np.sin(da))
        blocking = (perp < self.corridor_half_width) & (along > 0.0)
        if not np.any(blocking):
            return self.preprocess_dist
        return max(0.0, float(along[blocking].min()) - self.corridor_stop_margin)

    def _scan_xy(self, geom_ranges: np.ndarray, wrapped: np.ndarray):
        """이 스캔의 직교좌표 (x, y). 같은 배열이면 다시 안 만든다.

        코리도 검사는 스캔당 수십 번 돈다 (갭 적합성 × 갭 수, 조준 후보 11개).
        그때마다 `wrap → cos → sin` 을 전 빔에 걸고 있었다. 그런데 회전 대상은
        매번 같은 스캔이라, 한 번 직교좌표로 펴 두면 각도별로 남는 건 스칼라
        회전뿐이다.

        배열 **신원** 으로 확인한다. 스캔 콜백이 매번 새 배열을 만들므로
        이걸로 같은 스캔인지 판별이 된다.
        """
        if (
            self._xy_src_ranges is geom_ranges
            and self._xy_src_wrapped is wrapped
            and self._scan_cx is not None
        ):
            return self._scan_cx, self._scan_cy
        self._scan_cx = geom_ranges * np.cos(wrapped)
        self._scan_cy = geom_ranges * np.sin(wrapped)
        self._scan_positive = geom_ranges > 0.0
        self._xy_src_ranges = geom_ranges
        self._xy_src_wrapped = wrapped
        return self._scan_cx, self._scan_cy

    def _aim_range(
        self, gap_start_angle: float, gap_end_angle: float, prefer, aim_ref: float
    ) -> tuple[float, float]:
        """갭에서 **실제로 조준할 수 있는** 각도 범위.

        갭 그대로가 아니다. 가장자리 여유(`gap_edge_inset_rad`)를 물리고,
        탈출 기준 콘과 속도 연동 FOV 로 또 자른다. 갭 적합성 판정도 이 범위로
        해야 한다 — 원래 갭 폭으로 재면 "합격시킨 각도가 정작 못 쓰이는 각도"
        가 되어, 걸러 놓고도 같은 곳에서 막힌다.

        속도 제한은 **여기서만** 건다. 스캔을 좁히면 갭 탐색이 눈을 잃어서,
        정작 넓게 열린 쪽이 시야 밖이면 반대쪽 좁은 갭을 고른다 — 그게
        좌우로 방황하다 장애물 앞에 서는 동작이다. 갭은 넓게 보고, 그쪽으로
        트는 각만 낼 수 있는 만큼으로 줄인다.
        """
        lo = min(gap_start_angle, gap_end_angle)
        hi = max(gap_start_angle, gap_end_angle)
        inset = self.gap_edge_inset_rad
        if hi - lo > 2.0 * inset:
            lo += inset
            hi -= inset
        else:
            lo = hi = 0.5 * (lo + hi)
        if prefer is not None:
            lo, hi = self._clamp_to_cone(lo, hi, aim_ref, prefer[1])
        if self._fov_rad < self.fov_angle:
            lo, hi = self._clamp_to_cone(lo, hi, 0.0, self._fov_rad)
        return lo, hi

    def _gap_best_clear_m(
        self,
        geom_ranges: np.ndarray,
        wrapped: np.ndarray,
        lo: float,
        hi: float,
        stop_at: float,
    ) -> float:
        """이 각도 범위에서 낼 수 있는 **최선의** 코리도 여유거리 [m].

        전부 훑을 필요는 없다. `stop_at` 을 넘기는 각도가 하나라도 나오면
        그 갭은 합격이므로 거기서 끊는다. 보통 첫 한두 번에 끝난다.
        """
        best = 0.0
        for a in np.linspace(lo, hi, self.gap_fit_samples):
            best = max(
                best, self._corridor_clear_distance(geom_ranges, wrapped, float(a))
            )
            if best >= stop_at:
                break
        return best

    def _gaps_that_fit(
        self,
        gaps: list,
        geom_ranges: np.ndarray,
        wrapped: np.ndarray,
        work_angles: np.ndarray,
        prefer,
        aim_ref: float,
    ) -> list:
        """차폭이 실제로 들어가는 갭만 남긴다.

        갭 선택은 각도 폭으로만 이뤄져서 "각도는 넓은데 차폭은 안 들어가는"
        통로를 이긴 놈으로 뽑을 수 있다. 그 뒤의 코리도 검사는 이미 고른
        갭 **안에서** 각도를 옮길 뿐이라 되돌리지 못한다.

        판정 범위는 `_aim_range` 다. 갭 원래 폭으로 재면 조준 단계에서 잘려
        나갈 각도로 합격시키게 되어, 걸러 놓고도 같은 곳에서 막힌다.

        후보가 하나뿐이면 걸러 봐야 고를 게 없으므로 그냥 둔다. 전부
        떨어져도 원래 목록을 돌려준다 — 못 지나가는 상황에서 판단을 바꾼다고
        나아지지 않고, 그때는 감속과 AEB 가 받는다.
        """
        if not self.gap_fit_check_enable or len(gaps) < 2:
            self._last_gap_fit_counts = (len(gaps), len(gaps))
            return gaps
        fits = []
        for g in gaps:
            lo, hi = self._aim_range(
                float(work_angles[int(g[0])]),
                float(work_angles[int(g[-1])]),
                prefer,
                aim_ref,
            )
            if (
                self._gap_best_clear_m(
                    geom_ranges, wrapped, lo, hi, self.gap_fit_min_m
                )
                >= self.gap_fit_min_m
            ):
                fits.append(g)
        self._last_gap_fit_counts = (len(gaps), len(fits))
        return fits or gaps

    def _corridor_want_m(self) -> float:
        """보상이 포화되는 여유거리 [m]. 여기를 넘으면 정면 선호가 이긴다.

        속도에 비례시키되 상한을 낮게 둔다. 빨리 달릴수록 같은 시간에 더
        멀리 가니 조금 더 봐야 하지만, 상한이 없으면 "제일 멀리 보이는 쪽"
        을 겨냥하는 예전 동작으로 되돌아간다.
        """
        return min(
            self.corridor_want_max_m,
            max(self.corridor_want_min_m, self._ego_speed * self.corridor_want_time_s),
        )

    def _pick_target_angle(
        self,
        geom_ranges: np.ndarray,
        wrapped: np.ndarray,
        lo: float,
        hi: float,
        preferred: float,
        want: float,
        bias_ref: float = 0.0,
    ) -> tuple[float, float]:
        """(목표 각도, 그 방향 코리도 여유거리).

        preferred(갭 안에서 기준에 제일 가까운 각도)로 want 만큼 못 가면 갭
        안의 다른 각도를 뒤진다. 점수 = min(여유, want) − bias·|각도−기준|
        이라서, 여유가 충분해지는 순간부터는 기준에 가까운 쪽이 이긴다. 즉
        필요한 만큼만 틀고 불필요하게 크게 꺾지 않는다.

        `bias_ref` 가 기준이다. 평소엔 0(정면), AEB 탈출 중에는 멈춘 순간의
        헤딩 방향이 들어온다.

        **`want` 는 반드시 `_corridor_want_m()` 이어야 한다.** 목표점 거리를
        넣으면 보상이 5 m 까지 안 꺾여서 위 문장이 성립하지 않는다 —
        벌점(45° 에 0.79 m)이 보상 증가분을 못 이겨 항상 크게 튼다.

        후보를 [lo, hi] 로 가두므로 이미 검증된 갭 밖으로는 절대 안 나간다.
        """
        best_angle = preferred
        best_clear = self._corridor_clear_distance(geom_ranges, wrapped, preferred)
        if best_clear >= want:
            return best_angle, best_clear

        best_score = min(best_clear, want) - self.corridor_straight_bias * abs(
            preferred - bias_ref
        )
        for cand in np.linspace(lo, hi, self.corridor_angle_samples):
            angle = float(cand)
            clear = self._corridor_clear_distance(geom_ranges, wrapped, angle)
            score = min(clear, want) - self.corridor_straight_bias * abs(
                angle - bias_ref
            )
            if score > best_score:
                best_angle, best_clear, best_score = angle, clear, score
        return best_angle, best_clear

    def scan_callback(self, scan_msg: LaserScan) -> None:
        # enable OFF면 갭 연산/마커 모두 스킵 (CPU 절약). Foxglove 마커는 enable ON일 때만.
        publish_target = (not self.require_planner_enable) or self._fgm_enabled
        if not publish_target:
            return

        now_ns = self.get_clock().now().nanoseconds
        dt = 0.025
        if self._last_scan_ns is not None:
            d = (now_ns - self._last_scan_ns) * 1e-9
            if 0.0 < d < 1.0:
                dt = d
        self._last_scan_ns = now_ns

        self._fov_rad = self._fov_for_speed()

        ranges = np.array(scan_msg.ranges, dtype=np.float64)
        ranges = np.where(np.isinf(ranges), self.preprocess_dist, ranges)
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges[ranges > self.preprocess_dist] = self.preprocess_dist
        # 버블·FOV 마스킹 전의 실측 거리. 코리도 검증은 기하 문제라 원본을 써야 한다
        # (버블은 각도 섹터를 0 으로 만들어서 거리 정보가 사라진다).
        geom_ranges = ranges.copy()

        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        if angle_inc <= 1e-12:
            self.get_logger().warn("LaserScan angle_increment too small.")
            return

        n = len(ranges)
        beam_angles = angle_min + np.arange(n, dtype=np.float64) * angle_inc
        # 정면(+x)=0 기준 [-pi,pi]. Slamtec 0~360° 도 오른쪽이 음각으로 맞춰짐.
        wrapped = _wrap_pi_np(beam_angles)

        if self._use_full_scan_fov:
            fov_mask = np.ones(n, dtype=bool)
        else:
            fov_mask = np.abs(wrapped) <= self.fov_angle
            ranges = ranges.copy()
            ranges[~fov_mask] = 0.0

        valid_indices = np.where(ranges > 0.0)[0]
        if len(valid_indices) > 0:
            min_dist_idx = int(valid_indices[np.argmin(ranges[valid_indices])])
            min_dist = float(ranges[min_dist_idx])
            if min_dist < self.preprocess_dist:
                self._block_sector(
                    ranges, wrapped, float(wrapped[min_dist_idx]), min_dist, 0.0
                )

        # 검출된 장애물(정적+동적)은 거리와 무관하게 각도 섹터를 통째로 차단.
        # 거리 임계만 쓰면 gap_threshold 보다 먼 장애물이 "빈 공간"으로 남아
        # 갭 중심이 거의 안 밀리고 회피가 소극적으로 나온다.
        for obs_x, obs_y, obs_r in self._obstacle_sectors():
            obs_dist = math.hypot(obs_x, obs_y)
            if obs_dist < 1e-3 or obs_dist > self.preprocess_dist:
                continue
            obs_angle = math.atan2(obs_y, obs_x)
            if (not self._use_full_scan_fov) and abs(obs_angle) > self.fov_angle + 0.3:
                continue
            self._block_sector(ranges, wrapped, obs_angle, obs_dist, obs_r)

        # 인덱스 공간이 아니라 정면 기준 각도 순서로 갭 탐색
        # (Slamtec 0~360°에서 ±80°가 인덱스상 두 조각으로 갈라지는 문제 방지)
        fov_idx = np.where(fov_mask)[0]
        if fov_idx.size == 0:
            return
        order = np.argsort(wrapped[fov_idx])
        sorted_orig = fov_idx[order]
        work = ranges[sorted_orig].copy()
        work_angles = wrapped[sorted_orig]

        gap_threshold = self.gap_thr_primary
        threshold_indices = np.where(work > gap_threshold)[0]
        if len(threshold_indices) == 0:
            gap_threshold = self.gap_thr_fallback
            threshold_indices = np.where(work > gap_threshold)[0]
            if len(threshold_indices) == 0:
                return

        splits = np.where(np.diff(threshold_indices) > 1)[0] + 1
        gaps = [g for g in np.split(threshold_indices, splits) if len(g) > 0]
        if not gaps:
            return

        # 조준의 기준 방향. 평소엔 정면(0°), AEB 탈출 중에는 멈춘 순간의
        # 헤딩이 들어온다. 갭 선택부터 이걸 따라야 한다 — 각도만 나중에
        # 당겨 봐야, 이미 반대편 갭을 물었으면 되돌릴 수 없다.
        prefer = self._prefer_now()
        aim_ref = prefer[0] if prefer is not None else 0.0
        self._log_prefer(prefer is not None, aim_ref, prefer[1] if prefer else 0.0)

        gaps = self._gaps_that_fit(
            gaps, geom_ranges, wrapped, work_angles, prefer, aim_ref
        )

        self.min_gap_bins = max(2, int(self.min_gap_width_rad / abs(angle_inc)))
        max_len = max(len(g) for g in gaps)
        aim_idx = int(np.argmin(np.abs(_wrap_pi_np(work_angles - aim_ref))))
        chosen = self._select_gap(
            gaps, max_len, aim_idx, lock=prefer is not None, work_angles=work_angles
        )
        if chosen is None or len(chosen) == 0:
            return

        # chosen = work 배열 인덱스 → 원본 빔 / 정면 기준 각도
        center_work = int(chosen[len(chosen) // 2])
        gap_start_orig = int(sorted_orig[int(chosen[0])])
        gap_end_orig = int(sorted_orig[int(chosen[-1])])
        self._last_gap_center_angle = float(work_angles[center_work])

        gap_start_angle = float(wrapped[gap_start_orig])
        gap_end_angle = float(wrapped[gap_end_orig])
        viz_stamp = self.get_clock().now().to_msg()

        if self.publish_gap_marker:
            self.publish_gap_marker_angles(
                gap_start_angle,
                gap_end_angle,
                float(ranges[gap_start_orig]),
                float(ranges[gap_end_orig]),
                viz_stamp,
            )

        if has_listener(self.debug_scan_pub):
            debug_msg = LaserScan()
            debug_msg.header = scan_msg.header
            debug_msg.angle_min = scan_msg.angle_min
            debug_msg.angle_max = scan_msg.angle_max
            debug_msg.angle_increment = scan_msg.angle_increment
            debug_msg.range_min = scan_msg.range_min
            debug_msg.range_max = scan_msg.range_max
            debug_msg.time_increment = scan_msg.time_increment
            debug_msg.scan_time = scan_msg.scan_time
            debug_msg.ranges = [float(r) for r in ranges]
            debug_msg.header.stamp = viz_stamp
            self.debug_scan_pub.publish(debug_msg)

        if not publish_target:
            return

        # 갭 "중심"이 아니라 갭 안에서 정면(0°)에 가장 가까운 각도를 노린다.
        # 버블이 이미 차폭+여유를 먹고 있으므로 가장자리에서 inset 만 들어가면
        # 장애물을 확실히 비껴가면서도 필요 이상으로 크게 틀지 않는다.
        lo, hi = self._aim_range(gap_start_angle, gap_end_angle, prefer, aim_ref)
        eff_angle = min(hi, max(lo, aim_ref))

        # 목표점을 속도에 비례해 앞으로: 고속에서 0.5 m 앞은 조향이 즉시 되감긴다
        target_dist = min(
            self.target_max_m,
            max(self.target_min_m, self._ego_speed * self.target_lead_time_s),
        )
        # 그 방향으로 실제 뚫린 거리보다 멀리 찍지 않도록 제한
        aim_orig = int(sorted_orig[int(np.argmin(np.abs(work_angles - eff_angle)))])
        gap_range = float(ranges[aim_orig])
        if gap_range > 0.1:
            target_dist = min(target_dist, max(self.target_min_m, gap_range * 0.9))

        # 단일 빔이 아니라 차폭 코리도로 다시 검증. 각도 기준 갭 선택은
        # "각도는 열려 있는데 차폭은 안 들어가는" 통로를 걸러내지 못한다.
        if self.corridor_check_enable:
            # 각도 선정의 want 와 목표점 거리는 별개다. 목표점은 "얼마나 앞에
            # 찍을까"(조향 응답성), want 는 "여유가 이만큼이면 됐다"(방향 선택).
            eff_angle, clear = self._pick_target_angle(
                geom_ranges,
                wrapped,
                lo,
                hi,
                eff_angle,
                self._corridor_want_m(),
                bias_ref=aim_ref,
            )
            if clear < target_dist:
                target_dist = max(0.2, clear)
                if clear < self.target_min_m:
                    self._warn_corridor_blocked(eff_angle, clear)

        target_x = target_dist * math.cos(eff_angle)
        target_y = target_dist * math.sin(eff_angle)

        ox, oy = self._smooth_target(target_x, target_y, dt)

        point_msg = PointStamped()
        point_msg.header.stamp = viz_stamp
        point_msg.header.frame_id = self._laser_frame
        point_msg.point.x = float(ox)
        point_msg.point.y = float(oy)
        point_msg.point.z = 0.0
        self.target_pub.publish(point_msg)

    def publish_gap_marker_angles(
        self,
        start_angle: float,
        end_angle: float,
        range_start: float,
        range_end: float,
        stamp_msg,
    ) -> None:
        """선택 갭 양끝 V자 (정면 기준 wrap 각도)."""
        if not (
            has_listener(self.gap_marker_pub) or has_listener(self.gap_markers_pub)
        ):
            return
        marker = Marker()
        marker.header.stamp = stamp_msg
        marker.header.frame_id = self._laser_frame
        marker.ns = "fgm_gap"
        marker.id = 0
        # LINE_LIST 는 Foxglove 일부 버전에서 안 그린다. STRIP 3점이 V.
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.0
        marker.color.a = 1.0
        self._fill_marker_pose(marker)

        z = float(self.gap_marker_z_m)
        p_origin = Point()
        p_origin.x = 0.0
        p_origin.y = 0.0
        p_origin.z = z

        if not self._use_full_scan_fov:
            start_angle = max(-self.fov_angle, min(self.fov_angle, start_angle))
            end_angle = max(-self.fov_angle, min(self.fov_angle, end_angle))

        r_s = max(float(range_start), 1e-6)
        r_e = max(float(range_end), 1e-6)
        scale = self.gap_marker_arm_scale if self.gap_marker_arm_scale > 0.0 else 1.0
        len_s = r_s * scale
        len_e = r_e * scale
        if self.gap_marker_max_arm_m is not None:
            cap_hi = self.gap_marker_max_arm_m
        else:
            cap_hi = self.preprocess_dist
        len_s = min(len_s, cap_hi)
        len_e = min(len_e, cap_hi)

        p_start = Point()
        p_start.x = float(len_s * math.cos(start_angle))
        p_start.y = float(len_s * math.sin(start_angle))
        p_start.z = z

        p_end = Point()
        p_end.x = float(len_e * math.cos(end_angle))
        p_end.y = float(len_e * math.sin(end_angle))
        p_end.z = z

        marker.points.append(p_start)
        marker.points.append(p_origin)
        marker.points.append(p_end)
        self._last_gap_marker = marker
        if has_listener(self.gap_marker_pub):
            self.gap_marker_pub.publish(marker)
        if has_listener(self.gap_markers_pub):
            arr = MarkerArray()
            arr.markers.append(marker)
            self.gap_markers_pub.publish(arr)

    def publish_gap_marker(
        self,
        start_idx: int,
        end_idx: int,
        ranges: np.ndarray,
        angle_min: float,
        angle_inc: float,
        stamp_msg,
    ) -> None:
        """호환용: 인덱스 → wrap 각도 마커."""
        start_angle = _wrap_pi(angle_min + start_idx * angle_inc)
        end_angle = _wrap_pi(angle_min + end_idx * angle_inc)
        self.publish_gap_marker_angles(
            start_angle,
            end_angle,
            float(ranges[start_idx]),
            float(ranges[end_idx]),
            stamp_msg,
        )

    def _block_sector(
        self,
        ranges: np.ndarray,
        wrapped: np.ndarray,
        center_angle: float,
        dist: float,
        obstacle_radius: float,
    ) -> None:
        """장애물 각폭 + 차폭 여유만큼 각도 섹터를 0 으로.

        인덱스 창이 아니라 wrap 각도 마스크라 0/360° 경계에서도 잘리지 않는다.
        """
        # 장애 반경 + [장애물 버블] + [차량 버블 반폭(폭/2)]
        # 전방 길이는 planner ego_front_safety 로 타이밍 보정 (여기선 폭만).
        half_width = (
            obstacle_radius + self._bubble_now() + self.ego_half_width_m
        )
        if dist <= half_width:
            half_angle = math.pi / 2.0
        else:
            half_angle = math.asin(half_width / dist)
        blocked = np.abs(_wrap_pi_np(wrapped - center_angle)) <= half_angle
        ranges[blocked] = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = FGMNode()
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
