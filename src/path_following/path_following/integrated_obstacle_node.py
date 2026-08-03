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

from path_following.static_obstacle_node import StaticMap, resolve_map_yaml
from path_following.track_sliding import param_bool

_DEFAULT_MAP_DIR = "/home/nvidia/f1tenth_ajou/maps"

CFG = {
    # ===== 맵 바꿀 때 여기만 수정 =====
    "map_name": "cartographer_map_20260803_202601.yaml",
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
    "max_obstacle_range_m": 8.0,
    # 옆벽 잔차 컷. 회피로 비켜나는 동안에도 장애물을 계속 봐야 해서
    # 너무 좁으면 조향 도중 장애물이 사라져 경로가 되감긴다.
    "max_obstacle_lateral_m": 1.40,
    "match_dist_m": 1.00,
    "speed_threshold_mps": 0.45,
    # 확정/유지는 스캔 Hz와 무관하게 "초" 기준 (고속에서 지연이 곧 거리)
    "confirm_time_s": 0.08,          # 발행 전 최소 관측 시간
    "dynamic_confirm_time_s": 0.15,  # dynamic 분류 최소 관측 시간
    "track_keep_time_s": 0.15,       # 미검출 시 트랙 유지 시간
    "vel_ema_alpha": 0.35,
    "max_track_speed_mps": 12.0,
    "log_detections": True,
    "log_throttle_sec": 1.0,
}

@dataclass
class Detection:
    laser_x: float
    laser_y: float
    map_x: float
    map_y: float
    radius: float

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
        self._last_detect_log_ns = 0
        self._last_tf_warn_ns = 0

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
        self.marker_pub = self.create_publisher(MarkerArray, markers_topic, 10)

        self._tracks: list[Track] = []
        self._next_id = 0
        self._last_scan_time_ns: int | None = None

        self.get_logger().info(
            "integrated_obstacle: map residual + tracking | "
            f"walls={self.static_map.yaml_path} "
            f"frame={self._map_frame}←{self._laser_frame} "
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

    def _cluster_indices(
        self, px: np.ndarray, py: np.ndarray
    ) -> list[np.ndarray]:
        n = int(px.size)
        if n == 0:
            return []
        order = np.argsort(np.arctan2(py, px))
        px = px[order]
        py = py[order]
        clusters: list[np.ndarray] = []
        start = 0
        for i in range(1, n):
            if math.hypot(px[i] - px[i - 1], py[i] - py[i - 1]) > self.cluster_gap_threshold_m:
                if i - start >= self.min_cluster_points:
                    clusters.append(order[start:i].copy())
                start = i
        if n - start >= self.min_cluster_points:
            clusters.append(order[start:].copy())
        return clusters

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
        cluster_indices = self._cluster_indices(ox, oy)
        if not cluster_indices:
            return []

        detections: list[Detection] = []
        for cidx in cluster_indices:
            px_arr = ox[cidx]
            py_arr = oy[cidx]
            mx_arr = omx[cidx]
            my_arr = omy[cidx]

            min_x, max_x = float(np.min(px_arr)), float(np.max(px_arr))
            min_y, max_y = float(np.min(py_arr)), float(np.max(py_arr))
            size_x = max_x - min_x
            size_y = max_y - min_y
            if size_x > self.max_obstacle_size_m or size_y > self.max_obstacle_size_m:
                continue
            span_m = max(size_x, size_y)
            if span_m < self.min_obstacle_size_m:
                continue

            d2 = px_arr * px_arr + py_arr * py_arr
            kmin = int(np.argmin(d2))
            lx_pt = float(px_arr[kmin])
            ly_pt = float(py_arr[kmin])
            # 옆벽/측면 잔차 억제 (전방 경로 장애는 유지)
            if abs(ly_pt) > self.max_obstacle_lateral_m:
                continue
            detections.append(
                Detection(
                    laser_x=lx_pt,
                    laser_y=ly_pt,
                    map_x=float(np.mean(mx_arr)),
                    map_y=float(np.mean(my_arr)),
                    radius=span_m / 2.0,
                )
            )
        return detections

    def _update_tracks(self, detections: list[Detection], dt: float) -> None:
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
                dist = math.hypot(det.map_x - track.map_x, det.map_y - track.map_y)
                if dist < best_dist and dist <= self.match_dist_m:
                    best_dist = dist
                    best_idx = idx

            if best_idx < 0:
                track.missed_s += dt
                continue

            det = detections[best_idx]
            used_det.add(best_idx)

            if dt > 1e-6:
                vx_m = (det.map_x - track.map_x) / dt
                vy_m = (det.map_y - track.map_y) / dt
                vx_l = (det.laser_x - track.laser_x) / dt
                vy_l = (det.laser_y - track.laser_y) / dt
                raw_speed = math.hypot(vx_m, vy_m)
                # 매칭 점프/스캔 노이즈로 생긴 비현실 속도는 EMA에 넣지 않음
                if raw_speed <= self.max_track_speed_mps:
                    track.vx_map = alpha * vx_m + (1.0 - alpha) * track.vx_map
                    track.vy_map = alpha * vy_m + (1.0 - alpha) * track.vy_map
                    track.vx_laser = alpha * vx_l + (1.0 - alpha) * track.vx_laser
                    track.vy_laser = alpha * vy_l + (1.0 - alpha) * track.vy_laser

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

        for idx, det in enumerate(detections):
            if idx in used_det:
                continue
            self._tracks.append(
                Track(
                    track_id=self._next_id,
                    map_x=det.map_x,
                    map_y=det.map_y,
                    laser_x=det.laser_x,
                    laser_y=det.laser_y,
                    radius=det.radius,
                    age_s=dt,
                    matched=True,
                )
            )
            self._next_id += 1

        self._tracks = [
            t for t in self._tracks if t.missed_s <= self.track_keep_time_s
        ]

    @staticmethod
    def _is_dynamic(track: Track, speed_threshold: float, min_age_s: float) -> bool:
        return track.age_s >= min_age_s and track.speed >= speed_threshold

    def _publish_empty(self) -> None:
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
        marker_array = MarkerArray()
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
