#!/usr/bin/env python3
"""
정적 장애물 노드: Map Residual (Static Map Subtraction).

시뮬과 동일 알고리즘. 실차: laser_frame / map_yaml 만 젯슨 경로.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration as MsgDuration
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from path_following.scan_cluster import ClusterParams, cluster_scan_xy
from path_following.track_sliding import param_bool


# maps/ 아래 yaml 파일명만 바꾸면 됨 (전체 경로 불필요)
_DEFAULT_MAP_DIR = "/home/nvidia/f1tenth_ajou/maps"


def resolve_map_yaml(map_name: str, map_dir: str = "") -> str:
    """CFG map_name → 절대경로. 절대경로를 넣으면 그대로 사용."""
    name = str(map_name).strip()
    if not name:
        raise ValueError("map_name is empty — CFG['map_name'] 에 yaml 파일명을 넣으세요.")
    p = Path(name).expanduser()
    if p.is_absolute():
        if not p.is_file():
            raise FileNotFoundError(f"map yaml not found: {p}")
        return str(p.resolve())
    base = Path(map_dir).expanduser() if str(map_dir).strip() else Path(_DEFAULT_MAP_DIR)
    cand = (base / name).resolve()
    if not cand.is_file():
        raise FileNotFoundError(
            f"map yaml not found: {cand}\n"
            f"  CFG map_name={name!r}, map_dir={base}"
        )
    return str(cand)


CFG = {
    # ===== 맵 바꿀 때 여기만 수정 =====
    # integrated_obstacle_node·로컬라이제이션 pbstream·CSV 와 같은 맵이어야 한다.
    # 이전 값은 이틀 전 맵이라 세 곳과 전부 어긋나 있었다.
    "map_name": "cartographer_map_20260816_234629.yaml",  # 이전 20260814_232850_rosmap
    "map_dir": _DEFAULT_MAP_DIR,  # 보통 그대로
    # =================================
    "laser_frame": "laser",  # 실차 (시뮬: ego_racecar/laser)
    "map_frame": "map",
    "scan_topic": "/scan",
    "obstacles_topic": "/static_obstacles",
    "markers_topic": "/visualization_marker_array",
    # 실차 TF/맵 어긋남 + 잔차 노이즈 (너무 키우면 실장애를 벽으로 흡수)
    "wall_match_radius_m": 0.42,
    "tf_timeout_sec": 0.10,
    "cluster_gap_threshold_m": 0.28,
    "min_cluster_points": 10,
    # ---- [A1]~[A3], [A5] : integrated_obstacle_node 와 같은 의미/기본값 ----
    # 두 노드가 같은 공용 클러스터러(scan_cluster.py)를 쓰므로 파라미터도 맞춘다.
    "cluster_mode": "fixed",       # "fixed" | "adaptive"
    "abd_lambda_deg": 10.0,
    "abd_sigma_r_m": 0.02,
    "abd_min_gap_m": 0.05,
    "abd_max_gap_m": 0.35,
    "adaptive_min_points": False,
    "min_cluster_points_floor": 3,
    "min_arc_m": 0.07,
    "consistent_centroid": False,
    "radius_percentile": 90.0,
    "radius_min_m": 0.05,
    # 벽 잔차 오검출 억제. integrated_obstacle_node 와 같은 기준으로 맞춘다.
    "wall_residual_guard": True,
    "wall_clearance_m": 0.12,
    "near_wall_min_points": 14,
    "near_wall_min_span_m": 0.20,
    "max_obstacle_size_m": 0.85,
    "min_obstacle_size_m": 0.14,
    "max_obstacle_range_m": 11.0,
    "max_obstacle_lateral_m": 1.40,
    # 단발 잔차 깜빡임 완화 — 스캔 Hz와 무관하게 "초" 기준
    "persist_match_m": 0.55,
    "confirm_time_s": 0.04,  # 발행 전 최소 관측 시간 (반응↑)
    "keep_time_s": 0.10,     # 미검출 시 유지 시간
    "log_detections": False,
    "log_throttle_sec": 2.0,
    # Foxglove/RViz MarkerArray
    "publish_markers": True,
}


class StaticMap:
    """ROS map YAML + PNG/PGM → 팽창된 벽 occupancy."""

    def __init__(self, yaml_path: str, wall_match_radius_m: float):
        path = Path(yaml_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"map yaml not found: {path}")

        with path.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        img_name = str(meta["image"])
        img_path = Path(img_name)
        if not img_path.is_absolute():
            img_path = path.parent / img_path
        if not img_path.is_file():
            raise FileNotFoundError(f"map image not found: {img_path}")

        self.resolution = float(meta["resolution"])
        origin = meta["origin"]
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.negate = int(meta.get("negate", 0))
        self.occupied_thresh = float(meta.get("occupied_thresh", 0.65))

        gray = np.asarray(Image.open(img_path).convert("L"), dtype=np.float64)
        if self.negate:
            occ_prob = gray / 255.0
        else:
            occ_prob = (255.0 - gray) / 255.0
        occupied = occ_prob >= self.occupied_thresh

        r_cells = max(0, int(math.ceil(wall_match_radius_m / self.resolution)))
        self.wall = self._dilate(occupied, r_cells)
        self.height, self.width = self.wall.shape
        self.image_path = str(img_path)
        self.yaml_path = str(path)
        self.wall_match_radius_m = wall_match_radius_m
        self.dilate_cells = r_cells
        self._wall_dist: np.ndarray | None = None

    @staticmethod
    def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
        if radius <= 0:
            return mask.astype(bool, copy=True)
        ys, xs = np.where(mask)
        out = np.zeros_like(mask, dtype=bool)
        h, w = mask.shape
        r2 = radius * radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > r2:
                    continue
                yy = ys + dy
                xx = xs + dx
                valid = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
                out[yy[valid], xx[valid]] = True
        return out

    def world_to_cell(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        col = np.floor((x - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor(
            (self.origin_y + self.height * self.resolution - y) / self.resolution
        ).astype(np.int64)
        return row, col

    def wall_distance(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """팽창 벽 경계까지의 거리 [m]. 벽 안쪽이면 0, 맵 밖이면 0.

        [A5] 용. 매 스캔 반경 검사 대신 distance transform 격자를 한 번 만들어
        조회한다 (avoidance_safety.InflatedMap 과 같은 방식). 첫 호출에서만
        계산하고 이후엔 조회만 한다 — 켜지 않으면 비용이 0 이다.
        """
        if self._wall_dist is None:
            try:
                from scipy.ndimage import distance_transform_edt

                self._wall_dist = (
                    distance_transform_edt(~self.wall).astype(np.float32)
                    * self.resolution
                )
            except Exception:
                # scipy 가 없으면 가드를 무력화한다 (전부 "경계에서 멀다").
                self._wall_dist = np.full(self.wall.shape, 1e3, dtype=np.float32)

        row, col = self.world_to_cell(x, y)
        inside = (
            (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        )
        out = np.zeros(np.shape(x), dtype=np.float32)
        out[inside] = self._wall_dist[row[inside], col[inside]]
        return out

    def is_wall(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        row, col = self.world_to_cell(x, y)
        inside = (
            (row >= 0)
            & (row < self.height)
            & (col >= 0)
            & (col < self.width)
        )
        out = np.ones(x.shape, dtype=bool)
        out[inside] = self.wall[row[inside], col[inside]]
        return out


class StaticObstacleNode(Node):
    def __init__(self):
        super().__init__("static_obstacle_node")
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
        self.max_obstacle_range_m = max(
            1.0, float(self.get_parameter("max_obstacle_range_m").value)
        )
        self.max_obstacle_lateral_m = max(
            0.2, float(self.get_parameter("max_obstacle_lateral_m").value)
        )
        self.persist_match_m = max(
            0.05, float(self.get_parameter("persist_match_m").value)
        )
        self.confirm_time_s = max(0.0, float(self.get_parameter("confirm_time_s").value))
        self.keep_time_s = max(0.0, float(self.get_parameter("keep_time_s").value))
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
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
        self._wall_guard = param_bool(gp("wall_residual_guard").value)
        self._wall_clearance_m = max(0.0, float(gp("wall_clearance_m").value))
        self._near_wall_min_points = max(1, int(gp("near_wall_min_points").value))
        self._near_wall_min_span_m = max(0.0, float(gp("near_wall_min_span_m").value))

        # [(x, y, r, age_s, missed_s), ...]
        self._persist: list[list[float]] = []
        self._last_scan_ns: int | None = None
        self._dt: float = 0.025

        map_yaml = resolve_map_yaml(
            str(self.get_parameter("map_name").value),
            str(self.get_parameter("map_dir").value),
        )
        wall_r = max(0.0, float(self.get_parameter("wall_match_radius_m").value))
        self.static_map = StaticMap(map_yaml, wall_r)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        scan_topic = self.get_parameter("scan_topic").value
        markers_topic = self.get_parameter("markers_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value

        self.subscription = self.create_subscription(
            LaserScan, scan_topic, self.listener_callback, 10
        )
        self.marker_pub = (
            self.create_publisher(MarkerArray, markers_topic, 10)
            if self._publish_markers
            else None
        )
        self.obstacle_pub = self.create_publisher(Float32MultiArray, obstacles_topic, 10)

        self.get_logger().info(
            "static_obstacle: Map Residual (sim algorithm) | "
            f"walls={self.static_map.yaml_path} "
            f"img={Path(self.static_map.image_path).name} "
            f"match_r={self.static_map.wall_match_radius_m:.2f}m "
            f"({self.static_map.dilate_cells} cells) "
            f"frame={self._map_frame}←{self._laser_frame}"
        )

    def _publish_empty_obstacles(self) -> None:
        if self.marker_pub is not None:
            marker_array = MarkerArray()
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
        obs_msg = Float32MultiArray()
        obs_msg.data = []
        self.obstacle_pub.publish(obs_msg)

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

    def _cluster_xy(self, px: np.ndarray, py: np.ndarray, angle_inc: float):
        """[A1] integrated_obstacle_node 와 같은 공용 클러스터러를 쓴다."""
        return cluster_scan_xy(
            px,
            py,
            angle_increment=angle_inc,
            params=self._cluster_params,
            radius_percentile=self._radius_percentile,
            radius_min_m=self._radius_min_m,
            radius_max_m=self.max_obstacle_size_m / 2.0,
            consistent_centroid=self._consistent_centroid,
        )

    def listener_callback(self, msg: LaserScan) -> None:
        scan_ns = self.get_clock().now().nanoseconds
        if self._last_scan_ns is not None:
            dt = (scan_ns - self._last_scan_ns) * 1e-9
            if 0.0 < dt < 1.0:
                self._dt = dt
        self._last_scan_ns = scan_ns

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            self._publish_empty_obstacles()
            return

        tf = self._lookup_laser_to_map()
        if tf is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"TF {self._map_frame}←{self._laser_frame} 없음 — 장애 미발행"
                )
                self._last_tf_warn_ns = now_ns
            self._publish_empty_obstacles()
            return

        angle_min = float(msg.angle_min)
        angle_inc = float(msg.angle_increment)
        idx = np.arange(ranges.size, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < float(msg.range_max))
        if not np.any(valid):
            self._publish_empty_obstacles()
            return

        r = ranges[valid]
        th = angle_min + idx[valid] * angle_inc
        near = r <= self.max_obstacle_range_m
        if not np.any(near):
            self._persist = []
            self._publish_empty_obstacles()
            return
        r = r[near]
        th = th[near]
        lx = r * np.cos(th)
        ly = r * np.sin(th)
        mx, my = self._transform_xy(tf, lx, ly)

        wall_hit = self.static_map.is_wall(mx, my)
        obs_mask = ~wall_hit
        if not np.any(obs_mask):
            self._persist = []
            self._publish_empty_obstacles()
            return

        ox = lx[obs_mask]
        oy = ly[obs_mask]
        clusters = self._cluster_xy(ox, oy, angle_inc)
        if self._wall_guard and clusters:
            cmx, cmy = self._transform_xy(
                tf,
                np.array([c.center_x for c in clusters], dtype=np.float64),
                np.array([c.center_y for c in clusters], dtype=np.float64),
            )
            wdist = self.static_map.wall_distance(cmx, cmy)
            clusters = [
                c
                for c, wd in zip(clusters, wdist)
                if wd >= self._wall_clearance_m
                or (
                    c.n_points >= self._near_wall_min_points
                    and c.span_m >= self._near_wall_min_span_m
                )
            ]
        if not clusters:
            # 검출 없음 — missed만 증가시키도록 빈 raw로 persistence 갱신
            for p in self._persist:
                p[4] += self._dt
            self._persist = [p for p in self._persist if p[4] <= self.keep_time_s]
            if not self._persist:
                self._publish_empty_obstacles()
                return
            # 아래 공통 publish 경로를 위해 raw_dets 비우고 계속할 수 없으므로
            # 여기서 확정 트랙만 재발행
            now_msg = self.get_clock().now().to_msg()
            marker_array = MarkerArray() if self.marker_pub is not None else None
            if marker_array is not None:
                delete_marker = Marker()
                delete_marker.action = Marker.DELETEALL
                marker_array.markers.append(delete_marker)
            obstacle_data_list: list[float] = []
            final_obstacle_count = 0
            nearest_logic = None
            for oid, p in enumerate(self._persist):
                if p[3] < self.confirm_time_s:
                    continue
                logic_x, logic_y, radius = p[0], p[1], p[2]
                obstacle_data_list.extend([float(oid), logic_x, logic_y, radius])
                d = math.hypot(logic_x, logic_y)
                if nearest_logic is None or d < nearest_logic[2]:
                    nearest_logic = (logic_x, logic_y, d)
                if marker_array is not None:
                    marker = Marker()
                    marker.header.frame_id = self._laser_frame
                    marker.header.stamp = now_msg
                    marker.ns = "obstacles"
                    marker.id = oid
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = logic_x
                    marker.pose.position.y = logic_y
                    marker.pose.position.z = 0.0
                    marker.scale.x = max(radius * 2.0, 0.1)
                    marker.scale.y = max(radius * 2.0, 0.1)
                    marker.scale.z = 0.2
                    marker.color.a = 0.8
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.lifetime = MsgDuration(sec=0, nanosec=200000000)
                    marker_array.markers.append(marker)
                final_obstacle_count += 1
            if self.marker_pub is not None and marker_array is not None:
                self.marker_pub.publish(marker_array)
            obs_msg = Float32MultiArray()
            obs_msg.data = obstacle_data_list
            self.obstacle_pub.publish(obs_msg)
            return

        now_msg = self.get_clock().now().to_msg()
        marker_array = MarkerArray() if self.marker_pub is not None else None
        if marker_array is not None:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        obstacle_data_list: list[float] = []
        final_obstacle_count = 0
        nearest_logic = None
        raw_dets: list[tuple[float, float, float]] = []

        for cl in clusters:
            # 거리 게이트는 최근접점 유지 (AEB/FGM 등 소비자 거동 보존)
            logic_x, logic_y = cl.near_x, cl.near_y
            if cl.span_m > self.max_obstacle_size_m:
                continue
            if cl.span_m < self.min_obstacle_size_m:
                continue
            if abs(logic_y) > self.max_obstacle_lateral_m:
                continue

            raw_dets.append((logic_x, logic_y, cl.radius))

        # 연속 히트 persistence — 단발 노이즈 깜빡임 억제
        for p in self._persist:
            p[4] += self._dt  # 미검출 누적 시간
        used: set[int] = set()
        for dx, dy, dr in raw_dets:
            best_i = -1
            best_d = float("inf")
            for i, p in enumerate(self._persist):
                if i in used:
                    continue
                d = math.hypot(dx - p[0], dy - p[1])
                if d < best_d and d <= self.persist_match_m:
                    best_d = d
                    best_i = i
            if best_i >= 0:
                p = self._persist[best_i]
                p[0], p[1], p[2] = dx, dy, dr
                p[3] += self._dt
                p[4] = 0.0
                used.add(best_i)
            else:
                self._persist.append([dx, dy, dr, self._dt, 0.0])

        self._persist = [p for p in self._persist if p[4] <= self.keep_time_s]

        for oid, p in enumerate(self._persist):
            if p[3] < self.confirm_time_s:
                continue
            logic_x, logic_y, radius = p[0], p[1], p[2]
            obstacle_data_list.extend([float(oid), logic_x, logic_y, radius])
            d = math.hypot(logic_x, logic_y)
            if nearest_logic is None or d < nearest_logic[2]:
                nearest_logic = (logic_x, logic_y, d)

            if marker_array is not None:
                marker = Marker()
                marker.header.frame_id = self._laser_frame
                marker.header.stamp = now_msg
                marker.ns = "obstacles"
                marker.id = oid
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = logic_x
                marker.pose.position.y = logic_y
                marker.pose.position.z = 0.0
                marker.scale.x = max(radius * 2.0, 0.1)
                marker.scale.y = max(radius * 2.0, 0.1)
                marker.scale.z = 0.2
                marker.color.a = 0.8
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.lifetime = MsgDuration(sec=0, nanosec=200000000)
                marker_array.markers.append(marker)
            final_obstacle_count += 1

        if self.marker_pub is not None and marker_array is not None:
            self.marker_pub.publish(marker_array)
        obs_msg = Float32MultiArray()
        obs_msg.data = obstacle_data_list
        self.obstacle_pub.publish(obs_msg)

        if final_obstacle_count > 0 and self._log_detections:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_detect_log_ns >= self.log_throttle_ns:
                if nearest_logic is not None:
                    nx, ny, nd = nearest_logic
                    self.get_logger().info(
                        "맵잔차 장애 "
                        f"{final_obstacle_count}개 "
                        f"(최근접: x={nx:.2f}m, y={ny:.2f}m, d={nd:.2f}m) "
                        "→ /static_obstacles"
                    )
                else:
                    self.get_logger().info(
                        f"맵잔차 장애 {final_obstacle_count}개 → /static_obstacles"
                    )
                self._last_detect_log_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
