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

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64
from visualization_msgs.msg import Marker


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
    # 스캔 전처리·갭 (알고리즘)
    # 정면(레이저 +x) 기준 ±fov_half_deg 만 사용. ≤0 이면 스캔 전체.
    # Slamtec 0~360° 스캔도 wrap 후 정면 기준으로 자름.
    "fov_half_deg": 80.0,
    # 고속에선 멀리까지 봐야 갭이 미리 보임 (목표점 거리보다 넉넉하게)
    "scan_max_range_m": 4.0,
    # 세이프티 버블: 장애 각도 섹터를 차폭+여유만큼 통째로 막는다.
    # 차 반폭 0.15 m + 여유 → 이 값이 작으면 갭이 거의 안 갈라져 회피가 소극적.
    "bubble_radius_m": 0.30,
    "gap_threshold_primary_m": 1.5,
    "gap_threshold_fallback_m": 0.5,
    # 빔 개수가 아니라 각폭 기준 (라이다 분해능 바뀌어도 동일 동작)
    "min_gap_width_deg": 6.0,
    "gap_hysteresis_len_ratio": 0.78,
    # 갭 안에서 목표 각도를 가장자리로부터 얼마나 안쪽에 둘지.
    # 버블이 이미 차폭+여유를 먹고 있어 크게 줄 필요 없다 (각도라서 멀수록 과해짐).
    "gap_edge_inset_deg": 3.0,
    # 목표점 거리 = clamp(ego_speed * lead_time, min, max) [m, 레이저 프레임]
    "target_lead_time_s": 0.55,
    "target_min_m": 0.9,
    "target_max_m": 3.0,
    # 목표 스무딩: EMA 1단 + 이동 속도 제한 [m/s] (고속일수록 자동 완화)
    "target_smooth_alpha": 0.5,
    "target_max_rate_mps": 2.5,
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
        self.debug_scan_pub = self.create_publisher(LaserScan, "/fgm_debug_scan", 10)
        self.gap_marker_pub = self.create_publisher(Marker, "/fgm_gap_marker", 10)

        self.preprocess_dist = float(self.get_parameter("scan_max_range_m").value)
        self.bubble_radius = float(self.get_parameter("bubble_radius_m").value)
        self.gap_edge_inset_rad = math.radians(
            max(0.0, float(self.get_parameter("gap_edge_inset_deg").value))
        )
        self.publish_debug_scan = _param_bool(self.get_parameter("publish_debug_scan").value)

        self.fov_angle = math.radians(float(self.get_parameter("fov_half_deg").value))
        # ≤0 이면 FOV 크롭 안 함 (스캔 전방향)
        self._use_full_scan_fov = float(self.get_parameter("fov_half_deg").value) <= 0.0
        self.gap_thr_primary = float(self.get_parameter("gap_threshold_primary_m").value)
        self.gap_thr_fallback = float(self.get_parameter("gap_threshold_fallback_m").value)
        self.target_lead_time_s = max(
            0.0, float(self.get_parameter("target_lead_time_s").value)
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

        self.get_logger().info(
            f"FGM started (sim algorithm) | frame={self._laser_frame}, "
            f"target=v*{self.target_lead_time_s}s "
            f"[{self.target_min_m}~{self.target_max_m}]m, "
            f"scan_max={self.preprocess_dist}m, "
            f"bubble={self.bubble_radius}m, "
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

    def fgm_enable_callback(self, msg: Bool) -> None:
        was = self._fgm_enabled
        self._fgm_enabled = bool(msg.data)
        if was and not self._fgm_enabled:
            # 목표 스무딩만 리셋 — 갭 마커/히스테리시스는 유지
            self._reset_fgm_filter_state(keep_gap_hysteresis=True)

    def _reset_fgm_filter_state(self, *, keep_gap_hysteresis: bool = False) -> None:
        if not keep_gap_hysteresis:
            self._last_gap_center_idx = None
        self._filt_x = self._filt_y = None

    def _publish_gap_marker_delete(self) -> None:
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

    def scan_callback(self, scan_msg: LaserScan) -> None:
        # 갭 마커는 항상 계산·발행. /fgm_target 은 planner enable 일 때만.
        publish_target = (not self.require_planner_enable) or self._fgm_enabled

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

        self.publish_gap_marker_angles(
            gap_start_angle,
            gap_end_angle,
            float(ranges[gap_start_orig]),
            float(ranges[gap_end_orig]),
            viz_stamp,
        )

        if self.publish_debug_scan:
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
        half_width = obstacle_radius + self.bubble_radius
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
