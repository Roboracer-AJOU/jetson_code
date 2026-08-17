#!/usr/bin/env python3
"""
통합 장애물 노드: Map Residual + Cluster Tracking + Velocity 분류.

정적-only 회피 런치 → static_obstacle_node (기존)
정적+동적 회피 런치 → 본 노드 (맵 잔차 후 tracking/speed로 static/dynamic 분리)

/static_obstacles  [id, x, y, r, ...]        laser frame (local_planner 호환)
/dynamic_obstacles [id, x, y, vx, vy, r, ...] laser pos + laser-frame 상대속도
  closing = -(x*vx + y*vy) / hypot(x,y)  (+면 가까워짐/위협, -면 멀어짐)
  static/dynamic 판정은 map-frame 속력 사용
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from path_following.scan_cluster import ClusterParams, cluster_scan_xy
from path_following.static_obstacle_node import StaticMap, resolve_map_yaml
from path_following.track_kf import ConstantVelocityKF
from path_following.track_sliding import param_bool

_DEFAULT_MAP_DIR = "/home/nvidia/f1tenth_ajou/maps"

CFG = {
    # ===== 맵 바꿀 때 여기만 수정 =====
    # CSV(centerline/raceline)·로컬라이제이션 pbstream·static_obstacle_node 와
    # 반드시 같은 맵이어야 한다. 어긋나면 벽을 장애물로 보거나 그 반대가 된다.
    "map_name": "cartographer_map_20260817_003202.yaml",  # 이전 20260816_211739
    "map_dir": _DEFAULT_MAP_DIR,  # 보통 그대로
    # =================================
    "laser_frame": "laser",
    "map_frame": "map",
    "scan_topic": "/scan",
    "static_obstacles_topic": "/static_obstacles",
    "dynamic_obstacles_topic": "/dynamic_obstacles",
    "markers_topic": "/visualization_marker_array",
    # 맵 잔차 노이즈: 벽 매칭 여유 (너무 크게 하면 실장애 흡수)
    "wall_match_radius_m": 0.42,
    "tf_timeout_sec": 0.10,
    "cluster_gap_threshold_m": 0.28,
    "min_cluster_points": 10,
    "max_obstacle_size_m": 0.85,
    "min_obstacle_size_m": 0.14,
    "max_obstacle_range_m": 11.0,
    # 옆벽 잔차 컷. 회피로 비켜나는 동안에도 장애물을 계속 봐야 해서
    # 너무 좁으면 조향 도중 장애물이 사라져 경로가 되감긴다.
    "max_obstacle_lateral_m": 1.40,
    "match_dist_m": 1.00,
    "speed_threshold_mps": 0.45,
    # 확정/유지는 스캔 Hz와 무관하게 "초" 기준 (고속에서 지연이 곧 거리)
    "confirm_time_s": 0.04,          # 발행 전 최소 관측 시간 (반응↑)
    "dynamic_confirm_time_s": 0.08,  # dynamic 분류 최소 관측 시간
    "track_keep_time_s": 0.12,       # 미검출 시 트랙 유지 시간 (ema 모드)
    # kf 모드 전용 유지 시간. 40 Hz 에서 0.12 s 는 5 프레임뿐이라 그보다 조금만
    # 가려도 트랙이 삭제되고 새 ID 로 태어난다. age_s 가 리셋되니
    # dynamic_confirm_time_s 를 다시 세고, 그동안 달려오는 차가 static 으로
    # 분류된다. kf 는 미검출 중에도 predict 로 위치를 밀어 주므로 더 오래
    # 붙들고 있어도 되지만, ema 는 위치가 얼어붙은 채로 남아 오히려 나빠진다.
    # 그래서 tracker_mode="kf" 일 때만 이 값을 쓴다.
    "track_keep_time_s_kf": 0.25,
    "vel_ema_alpha": 0.35,
    "max_track_speed_mps": 12.0,
    # ---- [A1] 적응형 브레이크포인트 클러스터링 ----
    # 고정 0.28 m 는 근거리에선 관대하고 원거리에선 별개 물체 둘을 하나로 붙인다.
    # "adaptive" 는 임계를 거리에 비례시킨다. 검증 후 전환.
    "cluster_mode": "fixed",       # "fixed" | "adaptive"
    "abd_lambda_deg": 10.0,
    "abd_sigma_r_m": 0.02,
    "abd_min_gap_m": 0.05,
    "abd_max_gap_m": 0.35,
    # ---- [A2] 거리 스케일 최소 점수 ----
    # 10 m 앞 0.3 m 물체는 7점쯤 찍힌다 → 고정 10점에 걸려 통째로 사라진다.
    # floor 를 너무 낮추면 노이즈 3점이 장애물이 되니 [A5] 와 같이 켤 것.
    "adaptive_min_points": False,
    "min_cluster_points_floor": 3,
    "min_arc_m": 0.07,
    # ---- [A3] 대표점/반지름 정의 일관화 ----
    # 지금은 laser_x/y=최근접점, map_x/y=평균이라 정의가 다르다. 그 불일치가
    # 유한차분 속도에 그대로 노이즈로 들어간다. 켜면 속도/매칭은 centroid,
    # 거리 게이트는 최근접점으로 갈라서 쓴다 (발행 레이아웃은 그대로).
    "consistent_centroid": False,
    "radius_percentile": 90.0,
    "radius_min_m": 0.05,
    # ---- [A4] 등속 칼만 트래커 ----
    "tracker_mode": "ema",         # "ema" | "kf"
    "kf_sigma_accel": 3.0,         # 프로세스 노이즈 [m/s²]
    "kf_sigma_meas_m": 0.06,
    "kf_gate_mahalanobis": 0.0,    # 0 = 사용 안 함
    # ---- [A5] 벽 잔차 오검출 강건화 ----
    # 측위가 흔들리면 벽이 팽창 밴드를 벗어나 "장애물"로 샌다. 팽창 경계에
    # 붙어 있는 클러스터에는 더 높은 기준을 요구한다.
    #
    # 20260816 맵 기준으로 wall_clearance_m=0.12 에 걸리는 영역은 트랙
    # 자유공간의 7.8% 뿐이다. 나머지 92% (= 주행선이 지나는 트랙 한가운데)
    # 에서는 이 가드가 아무 일도 하지 않는다. 그래서 실장애를 놓칠 위험
    # 없이 벽 잔차만 골라 억제한다. 끄려면 False.
    "wall_residual_guard": True,
    "wall_clearance_m": 0.12,
    "near_wall_min_points": 14,
    "near_wall_min_span_m": 0.20,
    "log_detections": False,
    "log_throttle_sec": 1.0,
    # Foxglove/RViz MarkerArray
    "publish_markers": True,
}

def resolve_keep_time(mode: str, ema_s: float, kf_s: float) -> float:
    """미검출 트랙 유지 시간. kf 모드에서만 늘린 값을 쓴다.

    tracker_mode 하나만 되돌리면 유지 시간도 같이 원복되도록 묶어 둔다.
    """
    return kf_s if mode == "kf" else ema_s


@dataclass
class Detection:
    """클러스터 하나.

    laser_x/y, map_x/y 는 "발행/게이트에 쓰는 대표점" 이고 기존 정의(최근접점,
    평균)를 그대로 유지한다. center_* 는 추적 전용이다 — 속도와 매칭은
    프레임 간 정의가 같아야 해서 반드시 centroid 를 쓴다.
    """

    laser_x: float
    laser_y: float
    map_x: float
    map_y: float
    radius: float
    center_laser_x: float = 0.0
    center_laser_y: float = 0.0
    center_map_x: float = 0.0
    center_map_y: float = 0.0

@dataclass
class Track:
    track_id: int
    map_x: float
    map_y: float
    laser_x: float
    laser_y: float
    radius: float
    vx_map: float = 0.0
    vy_map: float = 0.0
    # laser-frame 위치 변화율 (ego 기준 상대) — closing = -(p·v)/|p|
    vx_laser: float = 0.0
    vy_laser: float = 0.0
    speed: float = 0.0  # map-frame 속력 (static/dynamic 판정)
    closing_mps: float = 0.0  # +면 접근(거리 감소)
    age_s: float = 0.0
    missed_s: float = 0.0
    matched: bool = False
    # 추적 기준점 (centroid). 매칭·속도는 전부 이 좌표로 한다.
    center_map_x: float = 0.0
    center_map_y: float = 0.0
    center_laser_x: float = 0.0
    center_laser_y: float = 0.0
    # tracker_mode="kf" 일 때만 채워진다. map/laser 를 각각 등속으로 본다 —
    # laser 쪽이 상대운동이라 closing_mps 가 여기서 바로 나온다.
    kf_map: ConstantVelocityKF | None = None
    kf_laser: ConstantVelocityKF | None = None

class IntegratedObstacleNode(Node):
    def __init__(self):
        super().__init__("integrated_obstacle_node")
        for key, value in CFG.items():
            self.declare_parameter(key, value)

        self._laser_frame = str(self.get_parameter("laser_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self.cluster_gap_threshold_m = float(
            self.get_parameter("cluster_gap_threshold_m").value
        )
        self.min_cluster_points = max(
            3, int(self.get_parameter("min_cluster_points").value)
        )
        self.max_obstacle_size_m = float(
            self.get_parameter("max_obstacle_size_m").value
        )
        self.min_obstacle_size_m = float(
            self.get_parameter("min_obstacle_size_m").value
        )
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.match_dist_m = max(
            0.05, float(self.get_parameter("match_dist_m").value)
        )
        self.speed_threshold_mps = max(
            0.0, float(self.get_parameter("speed_threshold_mps").value)
        )
        self.confirm_time_s = max(
            0.0, float(self.get_parameter("confirm_time_s").value)
        )
        self.dynamic_confirm_time_s = max(
            0.0, float(self.get_parameter("dynamic_confirm_time_s").value)
        )
        self.track_keep_time_s = max(
            0.0, float(self.get_parameter("track_keep_time_s").value)
        )
        self.vel_ema_alpha = float(
            max(0.05, min(1.0, float(self.get_parameter("vel_ema_alpha").value)))
        )
        self.max_track_speed_mps = max(
            1.0, float(self.get_parameter("max_track_speed_mps").value)
        )
        self.max_obstacle_range_m = max(
            1.0, float(self.get_parameter("max_obstacle_range_m").value)
        )
        self.max_obstacle_lateral_m = max(
            0.2, float(self.get_parameter("max_obstacle_lateral_m").value)
        )
        self.log_throttle_ns = int(
            max(0.1, float(self.get_parameter("log_throttle_sec").value)) * 1e9
        )
        self._log_detections = param_bool(self.get_parameter("log_detections").value)
        self._publish_markers = param_bool(self.get_parameter("publish_markers").value)
        self._last_detect_log_ns = 0
        self._last_tf_warn_ns = 0

        gp = self.get_parameter
        self._cluster_params = ClusterParams(
            mode=str(gp("cluster_mode").value).strip().lower(),
            gap_threshold_m=self.cluster_gap_threshold_m,
            lambda_deg=float(gp("abd_lambda_deg").value),
            sigma_r_m=float(gp("abd_sigma_r_m").value),
            min_gap_m=float(gp("abd_min_gap_m").value),
            max_gap_m=float(gp("abd_max_gap_m").value),
            min_points=self.min_cluster_points,
            adaptive_min_points=param_bool(gp("adaptive_min_points").value),
            min_points_floor=max(3, int(gp("min_cluster_points_floor").value)),
            min_arc_m=max(0.01, float(gp("min_arc_m").value)),
        )
        self._consistent_centroid = param_bool(gp("consistent_centroid").value)
        self._radius_percentile = min(100.0, max(1.0, float(gp("radius_percentile").value)))
        self._radius_min_m = max(0.01, float(gp("radius_min_m").value))

        self._tracker_mode = str(gp("tracker_mode").value).strip().lower()
        self._kf_sigma_accel = max(0.1, float(gp("kf_sigma_accel").value))
        self._kf_sigma_meas = max(0.005, float(gp("kf_sigma_meas_m").value))
        self._kf_gate_m2 = max(0.0, float(gp("kf_gate_mahalanobis").value)) ** 2
        self.track_keep_time_s = resolve_keep_time(
            self._tracker_mode,
            self.track_keep_time_s,
            max(0.0, float(gp("track_keep_time_s_kf").value)),
        )

        self._wall_guard = param_bool(gp("wall_residual_guard").value)
        self._wall_clearance_m = max(0.0, float(gp("wall_clearance_m").value))
        self._near_wall_min_points = max(1, int(gp("near_wall_min_points").value))
        self._near_wall_min_span_m = max(0.0, float(gp("near_wall_min_span_m").value))

        map_yaml = resolve_map_yaml(
            str(self.get_parameter("map_name").value),
            str(self.get_parameter("map_dir").value),
        )
        wall_r = max(0.0, float(self.get_parameter("wall_match_radius_m").value))
        self.static_map = StaticMap(map_yaml, wall_r)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        scan_topic = str(self.get_parameter("scan_topic").value)
        static_topic = str(self.get_parameter("static_obstacles_topic").value)
        dynamic_topic = str(self.get_parameter("dynamic_obstacles_topic").value)
        markers_topic = str(self.get_parameter("markers_topic").value)

        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.static_pub = self.create_publisher(Float32MultiArray, static_topic, 10)
        self.dynamic_pub = self.create_publisher(Float32MultiArray, dynamic_topic, 10)
        self.marker_pub = (
            self.create_publisher(MarkerArray, markers_topic, 10)
            if self._publish_markers
            else None
        )

        self._tracks: list[Track] = []
        self._next_id = 0
        self._last_scan_time_ns: int | None = None

        self.get_logger().info(
            "integrated_obstacle: map residual + tracking | "
            f"walls={self.static_map.yaml_path} "
            f"frame={self._map_frame}←{self._laser_frame} "
            f"tracker={self._tracker_mode} keep={self.track_keep_time_s:.2f}s "
            f"speed_th={self.speed_threshold_mps:.2f}m/s "
            f"confirm≥{self.confirm_time_s:.2f}s "
            f"dyn_confirm≥{self.dynamic_confirm_time_s:.2f}s"
        )

    def _lookup_laser_to_map(self):
        try:
            return self.tf_buffer.lookup_transform(
                self._map_frame,
                self._laser_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return None

    @staticmethod
    def _transform_xy(
        t, lx: np.ndarray, ly: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        c, s = math.cos(yaw), math.sin(yaw)
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        mx = c * lx - s * ly + tx
        my = s * lx + c * ly + ty
        return mx, my

    def _detections_from_scan(
        self, msg: LaserScan, tf
    ) -> list[Detection]:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            return []

        angle_min = float(msg.angle_min)
        angle_inc = float(msg.angle_increment)
        idx = np.arange(ranges.size, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < float(msg.range_max))
        if not np.any(valid):
            return []

        r = ranges[valid]
        th = angle_min + idx[valid] * angle_inc
        # 원거리 잔차 노이즈 컷 (너무 짧게 잡지 않음)
        near = r <= self.max_obstacle_range_m
        if not np.any(near):
            return []
        r = r[near]
        th = th[near]
        lx = r * np.cos(th)
        ly = r * np.sin(th)
        mx, my = self._transform_xy(tf, lx, ly)

        wall_hit = self.static_map.is_wall(mx, my)
        obs_mask = ~wall_hit
        if not np.any(obs_mask):
            return []

        ox = lx[obs_mask]
        oy = ly[obs_mask]
        omx = mx[obs_mask]
        omy = my[obs_mask]

        clusters = cluster_scan_xy(
            ox,
            oy,
            angle_increment=angle_inc,
            params=self._cluster_params,
            radius_percentile=self._radius_percentile,
            radius_min_m=self._radius_min_m,
            radius_max_m=self.max_obstacle_size_m / 2.0,
            consistent_centroid=self._consistent_centroid,
        )
        if not clusters:
            return []

        # [A5] 팽창 벽 경계까지의 거리. 켠 경우에만 계산한다.
        wall_dist = None
        if self._wall_guard:
            cx_arr = np.array([c.center_x for c in clusters], dtype=np.float64)
            cy_arr = np.array([c.center_y for c in clusters], dtype=np.float64)
            cmx, cmy = self._transform_xy(tf, cx_arr, cy_arr)
            wall_dist = self.static_map.wall_distance(cmx, cmy)

        detections: list[Detection] = []
        for ci, cl in enumerate(clusters):
            if cl.span_m > self.max_obstacle_size_m:
                continue
            if cl.span_m < self.min_obstacle_size_m:
                continue
            # 옆벽/측면 잔차 억제 (전방 경로 장애는 유지)
            if abs(cl.near_y) > self.max_obstacle_lateral_m:
                continue
            # [A5] 팽창 밴드에 붙은 잔차는 측위 흔들림일 확률이 높다.
            # 진짜 장애물이라면 점도 많고 폭도 있어야 한다.
            if wall_dist is not None and wall_dist[ci] < self._wall_clearance_m:
                if (
                    cl.n_points < self._near_wall_min_points
                    or cl.span_m < self._near_wall_min_span_m
                ):
                    continue

            cidx = cl.idx
            mx_arr = omx[cidx]
            my_arr = omy[cidx]
            cmap_x = float(np.mean(mx_arr))
            cmap_y = float(np.mean(my_arr))
            detections.append(
                Detection(
                    laser_x=cl.near_x,
                    laser_y=cl.near_y,
                    map_x=cmap_x,
                    map_y=cmap_y,
                    radius=cl.radius,
                    center_laser_x=cl.center_x,
                    center_laser_y=cl.center_y,
                    center_map_x=cmap_x,
                    center_map_y=cmap_y,
                )
            )
        return detections

    def _track_coords(self, det: Detection) -> tuple[float, float, float, float]:
        """추적에 쓸 (laser_x, laser_y, map_x, map_y).

        consistent_centroid 를 끄면 기존 정의(최근접점 / map 평균)를 그대로
        돌려줘 거동이 바뀌지 않는다.
        """
        if self._consistent_centroid:
            return (
                det.center_laser_x,
                det.center_laser_y,
                det.center_map_x,
                det.center_map_y,
            )
        return det.laser_x, det.laser_y, det.map_x, det.map_y

    def _finish_track(self, track: Track, det: Detection, dt: float) -> None:
        """매칭된 트랙의 공통 마무리 — 발행용 대표점과 closing 갱신."""
        track.map_x = det.map_x
        track.map_y = det.map_y
        track.laser_x = det.laser_x
        track.laser_y = det.laser_y
        track.radius = det.radius
        track.speed = math.hypot(track.vx_map, track.vy_map)
        rng = math.hypot(track.laser_x, track.laser_y)
        if rng > 1e-3:
            # closing = - (p·v)/|p|
            # 가까워질 때(거리↓) p·v < 0 → closing > 0 (위협 +)
            track.closing_mps = -(
                track.laser_x * track.vx_laser + track.laser_y * track.vy_laser
            ) / rng
        else:
            track.closing_mps = 0.0
        track.age_s += dt
        track.missed_s = 0.0
        track.matched = True

    def _update_tracks(self, detections: list[Detection], dt: float) -> None:
        if self._tracker_mode == "kf":
            self._update_tracks_kf(detections, dt)
        else:
            self._update_tracks_ema(detections, dt)
        self._tracks = [
            t for t in self._tracks if t.missed_s <= self.track_keep_time_s
        ]

    def _spawn_track(self, det: Detection, dt: float) -> None:
        lx, ly, mx, my = self._track_coords(det)
        track = Track(
            track_id=self._next_id,
            map_x=det.map_x,
            map_y=det.map_y,
            laser_x=det.laser_x,
            laser_y=det.laser_y,
            radius=det.radius,
            age_s=dt,
            matched=True,
            center_map_x=mx,
            center_map_y=my,
            center_laser_x=lx,
            center_laser_y=ly,
        )
        if self._tracker_mode == "kf":
            track.kf_map = ConstantVelocityKF(
                mx, my, self._kf_sigma_accel, self._kf_sigma_meas
            )
            track.kf_laser = ConstantVelocityKF(
                lx, ly, self._kf_sigma_accel, self._kf_sigma_meas
            )
        self._tracks.append(track)
        self._next_id += 1

    def _update_tracks_ema(self, detections: list[Detection], dt: float) -> None:
        for track in self._tracks:
            track.matched = False

        used_det: set[int] = set()
        alpha = self.vel_ema_alpha

        for track in self._tracks:
            best_idx = -1
            best_dist = float("inf")
            for idx, det in enumerate(detections):
                if idx in used_det:
                    continue
                _, _, dmx, dmy = self._track_coords(det)
                dist = math.hypot(dmx - track.center_map_x, dmy - track.center_map_y)
                if dist < best_dist and dist <= self.match_dist_m:
                    best_dist = dist
                    best_idx = idx

            if best_idx < 0:
                track.missed_s += dt
                continue

            det = detections[best_idx]
            used_det.add(best_idx)
            dlx, dly, dmx, dmy = self._track_coords(det)

            if dt > 1e-6:
                vx_m = (dmx - track.center_map_x) / dt
                vy_m = (dmy - track.center_map_y) / dt
                vx_l = (dlx - track.center_laser_x) / dt
                vy_l = (dly - track.center_laser_y) / dt
                raw_speed = math.hypot(vx_m, vy_m)
                # 매칭 점프/스캔 노이즈로 생긴 비현실 속도는 EMA에 넣지 않음
                if raw_speed <= self.max_track_speed_mps:
                    track.vx_map = alpha * vx_m + (1.0 - alpha) * track.vx_map
                    track.vy_map = alpha * vy_m + (1.0 - alpha) * track.vy_map
                    track.vx_laser = alpha * vx_l + (1.0 - alpha) * track.vx_laser
                    track.vy_laser = alpha * vy_l + (1.0 - alpha) * track.vy_laser

            track.center_map_x = dmx
            track.center_map_y = dmy
            track.center_laser_x = dlx
            track.center_laser_y = dly
            self._finish_track(track, det, dt)

        for idx, det in enumerate(detections):
            if idx not in used_det:
                self._spawn_track(det, dt)

    def _update_tracks_kf(self, detections: list[Detection], dt: float) -> None:
        """predict → 매칭 → update.

        미검출 프레임에도 predict 를 돌려 트랙이 얼어붙지 않게 한다. EMA 쪽은
        미검출 시 위치를 그대로 두는데, 그러면 다음 검출에서 여러 프레임치
        변위가 한 dt 에 몰려 속도가 튄다.
        """
        for track in self._tracks:
            track.matched = False
            if track.kf_map is None or track.kf_laser is None:
                continue
            track.kf_map.predict(dt)
            track.kf_laser.predict(dt)
            track.center_map_x = track.kf_map.px
            track.center_map_y = track.kf_map.py
            track.center_laser_x = track.kf_laser.px
            track.center_laser_y = track.kf_laser.py

        used_det: set[int] = set()
        for track in self._tracks:
            kfm = track.kf_map
            kfl = track.kf_laser
            if kfm is None or kfl is None:
                track.missed_s += dt
                continue

            best_idx = -1
            best_dist = float("inf")
            for idx, det in enumerate(detections):
                if idx in used_det:
                    continue
                _, _, dmx, dmy = self._track_coords(det)
                # 매칭 거리는 예측 위치 기준
                dist = math.hypot(dmx - kfm.px, dmy - kfm.py)
                if dist > self.match_dist_m or dist >= best_dist:
                    continue
                if self._kf_gate_m2 > 0.0 and kfm.mahalanobis2(dmx, dmy) > self._kf_gate_m2:
                    continue
                best_dist = dist
                best_idx = idx

            if best_idx < 0:
                track.missed_s += dt
                # 미검출이라도 예측 속도는 유지된다. 위치만 predict 로 전진.
                track.map_x = kfm.px
                track.map_y = kfm.py
                continue

            det = detections[best_idx]
            used_det.add(best_idx)
            dlx, dly, dmx, dmy = self._track_coords(det)
            kfm.update(dmx, dmy)
            kfl.update(dlx, dly)

            track.center_map_x = kfm.px
            track.center_map_y = kfm.py
            track.center_laser_x = kfl.px
            track.center_laser_y = kfl.py
            speed = kfm.speed
            if speed <= self.max_track_speed_mps:
                track.vx_map = kfm.vx
                track.vy_map = kfm.vy
                track.vx_laser = kfl.vx
                track.vy_laser = kfl.vy
            self._finish_track(track, det, dt)

        for idx, det in enumerate(detections):
            if idx not in used_det:
                self._spawn_track(det, dt)

    @staticmethod
    def _is_dynamic(track: Track, speed_threshold: float, min_age_s: float) -> bool:
        return track.age_s >= min_age_s and track.speed >= speed_threshold

    def _publish_empty(self) -> None:
        if self.marker_pub is not None:
            delete = MarkerArray()
            m = Marker()
            m.action = Marker.DELETEALL
            delete.markers.append(m)
            self.marker_pub.publish(delete)
        self.static_pub.publish(Float32MultiArray(data=[]))
        self.dynamic_pub.publish(Float32MultiArray(data=[]))

    def scan_callback(self, msg: LaserScan) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_scan_time_ns is None:
            self._last_scan_time_ns = now_ns
            return

        dt = (now_ns - self._last_scan_time_ns) * 1e-9
        self._last_scan_time_ns = now_ns
        if dt <= 0.0:
            return

        tf = self._lookup_laser_to_map()
        if tf is None:
            if now_ns - self._last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"TF {self._map_frame}←{self._laser_frame} 없음 — 장애 미발행"
                )
                self._last_tf_warn_ns = now_ns
            self._publish_empty()
            return

        detections = self._detections_from_scan(msg, tf)
        self._update_tracks(detections, dt)

        static_data: list[float] = []
        dynamic_data: list[float] = []
        marker_array = MarkerArray() if self.marker_pub is not None else None
        if marker_array is not None:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
        now_msg = self.get_clock().now().to_msg()

        static_count = 0
        dynamic_count = 0

        for track in self._tracks:
            if not track.matched:
                continue
            # 단발/짧은 트랙은 노이즈로 보고 미발행 (너무 길면 실장애 반응 늦음)
            if track.age_s < self.confirm_time_s:
                continue
            if self._is_dynamic(
                track, self.speed_threshold_mps, self.dynamic_confirm_time_s
            ):
                dynamic_data.extend(
                    [
                        float(track.track_id),
                        track.laser_x,
                        track.laser_y,
                        # laser-frame 상대속도 → planner closing = -(p·v)/|p|
                        track.vx_laser,
                        track.vy_laser,
                        track.radius,
                    ]
                )
                color = (0.0, 0.4, 1.0)
                dynamic_count += 1
            else:
                static_data.extend(
                    [
                        float(track.track_id),
                        track.laser_x,
                        track.laser_y,
                        track.radius,
                    ]
                )
                color = (1.0, 0.0, 0.0)
                static_count += 1

            if marker_array is not None:
                marker = Marker()
                marker.header.frame_id = self._laser_frame
                marker.header.stamp = now_msg
                marker.ns = "integrated_obstacles"
                marker.id = track.track_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = track.laser_x
                marker.pose.position.y = track.laser_y
                marker.pose.position.z = 0.0
                marker.scale.x = max(track.radius * 2.0, 0.1)
                marker.scale.y = max(track.radius * 2.0, 0.1)
                marker.scale.z = 0.2
                marker.color.a = 0.8
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.lifetime = MsgDuration(sec=0, nanosec=200000000)
                marker_array.markers.append(marker)

        self.static_pub.publish(Float32MultiArray(data=static_data))
        self.dynamic_pub.publish(Float32MultiArray(data=dynamic_data))
        if self.marker_pub is not None and marker_array is not None:
            self.marker_pub.publish(marker_array)

        if self._log_detections and (static_count + dynamic_count) > 0:
            if now_ns - self._last_detect_log_ns >= self.log_throttle_ns:
                dyn_parts = []
                for track in self._tracks:
                    if not track.matched:
                        continue
                    if self._is_dynamic(
                        track, self.speed_threshold_mps, self.dynamic_confirm_time_s
                    ):
                        dyn_parts.append(
                            f"id={track.track_id} "
                            f"v_map={track.speed:.2f}m/s "
                            f"close={track.closing_mps:+.2f} "
                            f"(vl={track.vx_laser:+.2f},{track.vy_laser:+.2f}) "
                            f"r={track.radius:.2f}m "
                            f"laser=({track.laser_x:.2f},{track.laser_y:.2f})"
                        )
                dyn_str = " | ".join(dyn_parts) if dyn_parts else "—"
                self.get_logger().info(
                    f"OBS_STATUS | static={static_count} dynamic={dynamic_count} "
                    f"tracks={len(self._tracks)} | dyn: {dyn_str}"
                )
                self._last_detect_log_ns = now_ns

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
