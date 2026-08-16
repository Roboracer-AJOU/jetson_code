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
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64
from visualization_msgs.msg import Marker

from path_following import vehicle_geometry as vg

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
    # True면 local_planner 의 /planner/fgm_enable 이 True 일 때만 갭 계산
    "require_planner_enable": True,
    "target_topic": "/fgm_target",
    "publish_debug_scan": False,
    # Foxglove/RViz 갭 마커. enable ON일 때만 계산·발행 (OFF면 스캔 스킵 유지).
    "publish_gap_marker": True,
    # 스캔 전처리·갭 (알고리즘)
    # 정면(레이저 +x) 기준 ±fov_half_deg 만 사용. ≤0 이면 스캔 전체.
    # Slamtec 0~360° 스캔도 wrap 후 정면 기준으로 자름.
    "fov_half_deg": 80.0,
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
    # RViz V자 갭 마커 (주행과 무관, 표시만)
    "gap_marker_arm_scale": 1.5,
    "gap_marker_max_arm_m": 2.0,
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

        self.latest_obstacles: list = []
        self.latest_dynamic_obstacles: list = []
        self._last_gap_center_idx: int | None = None
        self._filt_x: float | None = None
        self._filt_y: float | None = None
        self._ego_speed = 0.0
        self._last_scan_ns: int | None = None
        self._last_corridor_warn_ns = 0
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
            # 목표 스무딩만 리셋 — 갭 마커/히스테리시스는 유지
            self._reset_fgm_filter_state(keep_gap_hysteresis=True)
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
            self._last_gap_center_idx = None
        self._filt_x = self._filt_y = None

    def _publish_gap_marker_delete(self) -> None:
        if self.gap_marker_pub is None:
            return
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._laser_frame
        m.ns = "fgm_gap"
        m.id = 0
        m.action = Marker.DELETE
        self.gap_marker_pub.publish(m)

    def _select_gap(self, gaps: list, max_len: int) -> np.ndarray | None:
        if not gaps:
            return None
        wide = [g for g in gaps if len(g) >= self.min_gap_bins]
        if not wide:
            wide = list(gaps)
        thresh_len = max(self.min_gap_bins, int(math.ceil(self.hyst_ratio * max_len)))

        def center_idx(g: np.ndarray) -> int:
            return int(g[len(g) // 2])

        if self._last_gap_center_idx is not None:
            candidates = [g for g in wide if len(g) >= thresh_len]
            if not candidates:
                candidates = wide
            best = min(
                candidates,
                key=lambda g: abs(center_idx(g) - self._last_gap_center_idx),
            )
            return best

        return max(wide, key=lambda g: len(g))

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
        """차폭이 안 들어가는 상황 경고 (1초에 한 번). 정지는 AEB 몫."""
        now = self.get_clock().now().nanoseconds
        if now - self._last_corridor_warn_ns < 1_000_000_000:
            return
        self._last_corridor_warn_ns = now
        self.get_logger().warn(
            f"gap 은 열렸지만 차폭이 안 들어감 — aim={math.degrees(angle):+.0f}° "
            f"clear={clear:.2f}m < {self.target_min_m:.2f}m. 목표점을 당겨 찍음"
        )

    def _corridor_clear_distance(
        self, geom_ranges: np.ndarray, wrapped: np.ndarray, angle: float
    ) -> float:
        """angle 방향으로 차폭 코리도가 뚫려 있는 거리 [m].

        목표 방향을 축으로 두고, 축에서 반폭 이내로 들어오는 점들 중 가장 가까운
        것까지의 전방거리를 낸다. 갭 판정이 각도 기준이라 놓치는 "멀리서 좁아지는
        통로"를 여기서 잡는다.
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

    def _pick_target_angle(
        self,
        geom_ranges: np.ndarray,
        wrapped: np.ndarray,
        lo: float,
        hi: float,
        preferred: float,
        want: float,
    ) -> tuple[float, float]:
        """(목표 각도, 그 방향 코리도 여유거리).

        preferred(갭 안에서 정면에 제일 가까운 각도)로 want 만큼 못 가면 갭
        안의 다른 각도를 뒤진다. 점수 = min(여유, want) − bias·|각도| 라서,
        여유가 충분해지는 순간부터는 정면에 가까운 쪽이 이긴다. 즉 필요한
        만큼만 틀고 불필요하게 크게 꺾지 않는다.

        후보를 [lo, hi] 로 가두므로 이미 검증된 갭 밖으로는 절대 안 나간다.
        """
        best_angle = preferred
        best_clear = self._corridor_clear_distance(geom_ranges, wrapped, preferred)
        if best_clear >= want:
            return best_angle, best_clear

        best_score = min(best_clear, want) - self.corridor_straight_bias * abs(preferred)
        for cand in np.linspace(lo, hi, self.corridor_angle_samples):
            angle = float(cand)
            clear = self._corridor_clear_distance(geom_ranges, wrapped, angle)
            score = min(clear, want) - self.corridor_straight_bias * abs(angle)
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

        self.min_gap_bins = max(2, int(self.min_gap_width_rad / abs(angle_inc)))
        max_len = max(len(g) for g in gaps)
        chosen = self._select_gap(gaps, max_len)
        if chosen is None or len(chosen) == 0:
            return

        # chosen = work 배열 인덱스 → 원본 빔 / 정면 기준 각도
        center_work = int(chosen[len(chosen) // 2])
        gap_start_orig = int(sorted_orig[int(chosen[0])])
        gap_end_orig = int(sorted_orig[int(chosen[-1])])
        self._last_gap_center_idx = center_work

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

        if self.publish_debug_scan and self.debug_scan_pub is not None:
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
        lo = min(gap_start_angle, gap_end_angle)
        hi = max(gap_start_angle, gap_end_angle)
        inset = self.gap_edge_inset_rad
        if hi - lo > 2.0 * inset:
            lo += inset
            hi -= inset
        else:
            lo = hi = 0.5 * (lo + hi)
        eff_angle = min(hi, max(lo, 0.0))

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
            eff_angle, clear = self._pick_target_angle(
                geom_ranges, wrapped, lo, hi, eff_angle, target_dist
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
        if self.gap_marker_pub is None:
            return
        marker = Marker()
        marker.header.stamp = stamp_msg
        marker.header.frame_id = self._laser_frame
        marker.ns = "fgm_gap"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        p_origin = Point()
        p_origin.x = 0.0
        p_origin.y = 0.0
        p_origin.z = 0.0

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
        p_start.z = 0.0

        p_end = Point()
        p_end.x = float(len_e * math.cos(end_angle))
        p_end.y = float(len_e * math.sin(end_angle))
        p_end.z = 0.0

        marker.points.append(p_origin)
        marker.points.append(p_start)
        marker.points.append(p_origin)
        marker.points.append(p_end)
        self.gap_marker_pub.publish(marker)

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
