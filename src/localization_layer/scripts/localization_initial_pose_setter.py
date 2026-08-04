#!/usr/bin/env python3
"""Cartographer localization pose: RViz /initialpose (optional scan refine)."""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_TRAJECTORY_ACTIVE = 0
_LIDAR_X_IN_BASE = 0.31
_LIDAR_Y_IN_BASE = 0.0


def _optional_float(value) -> float:
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ('', 'nan', 'none', 'null'):
            return float('nan')
    try:
        return float(value)
    except (TypeError, ValueError):
        return float('nan')


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _load_origin_pose(pbstream: str, use_saved: bool) -> tuple[float, float, float]:
    if not use_saved or not pbstream:
        return 0.0, 0.0, 0.0

    origin_yaml = f'{os.path.splitext(pbstream)[0]}_origin.yaml'
    if not os.path.isfile(origin_yaml):
        return 0.0, 0.0, 0.0

    with open(origin_yaml, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    pose = data.get('initial_pose', {})
    return (
        float(pose.get('x', 0.0)),
        float(pose.get('y', 0.0)),
        float(pose.get('yaw', 0.0)),
    )


def _grid_lookup(grid: np.ndarray, msg: OccupancyGrid, wx: float, wy: float) -> int:
    gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
    gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
    if gx < 0 or gy < 0 or gx >= msg.info.width or gy >= msg.info.height:
        return -1
    return int(grid[gy, gx])


def _score_pose(
    scan: LaserScan,
    grid: np.ndarray,
    map_msg: OccupancyGrid,
    x: float,
    y: float,
    yaw: float,
) -> float:
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    score = 0.0
    used = 0
    step = max(1, int(round(math.radians(2.0) / max(scan.angle_increment, 1e-6))))

    for i in range(0, len(scan.ranges), step):
        rng = scan.ranges[i]
        if not math.isfinite(rng) or rng < scan.range_min or rng > min(scan.range_max, 12.0):
            continue
        angle = scan.angle_min + i * scan.angle_increment
        lx = _LIDAR_X_IN_BASE + rng * math.cos(angle)
        ly = _LIDAR_Y_IN_BASE + rng * math.sin(angle)
        wx = x + cos_y * lx - sin_y * ly
        wy = y + sin_y * lx + cos_y * ly
        occ = _grid_lookup(grid, map_msg, wx, wy)
        if occ < 0:
            continue
        used += 1
        if occ >= 50:
            score += 2.0
        elif occ == 0:
            score -= 0.5

    return float('-inf') if used < 8 else score / used


def _refine_pose(
    scan: LaserScan,
    map_msg: OccupancyGrid,
    x: float,
    y: float,
    yaw: float,
    *,
    xy_window_m: float,
    xy_step_m: float,
    yaw_window_deg: float,
    yaw_step_deg: float,
) -> tuple[float, float, float, float]:
    grid = np.array(map_msg.data, dtype=np.int16).reshape(
        (map_msg.info.height, map_msg.info.width)
    )
    # Reject jumping into unknown / outside free corridor.
    if _grid_lookup(grid, map_msg, x, y) != 0:
        return x, y, yaw, float('-inf')

    best = (float('-inf'), x, y, yaw)
    yaw_step = math.radians(yaw_step_deg)
    yaw_window = math.radians(yaw_window_deg)

    dyaw = -yaw_window
    while dyaw <= yaw_window + 1e-9:
        cyaw = yaw + dyaw
        dy = -xy_window_m
        while dy <= xy_window_m + 1e-9:
            dx = -xy_window_m
            while dx <= xy_window_m + 1e-9:
                cx, cy = x + dx, y + dy
                if _grid_lookup(grid, map_msg, cx, cy) != 0:
                    dx += xy_step_m
                    continue
                s = _score_pose(scan, grid, map_msg, cx, cy, cyaw)
                if s > best[0]:
                    best = (s, cx, cy, cyaw)
                dx += xy_step_m
            dy += xy_step_m
        dyaw += yaw_step

    return best[1], best[2], best[3], best[0]


class LocalizationInitialPoseSetter(Node):
    def __init__(self):
        super().__init__('localization_initial_pose_setter')

        self.declare_parameter('pbstream_filename', '')
        self.declare_parameter('configuration_directory', '')
        self.declare_parameter('configuration_basename', 'cartographer_2d_localization.lua')
        self.declare_parameter('use_saved_mapping_origin', False)
        self.declare_parameter('wait_for_rviz_initial_pose', True)
        # Default OFF: rosmap is sparse; auto refine/relock caused outside-map snaps.
        self.declare_parameter('refine_with_scan_matching', False)
        self.declare_parameter('refine_xy_window_m', 0.5)
        self.declare_parameter('refine_xy_step_m', 0.08)
        self.declare_parameter('refine_yaw_window_deg', 20.0)
        self.declare_parameter('refine_yaw_step_deg', 4.0)
        self.declare_parameter('initial_pose_x', float('nan'))
        self.declare_parameter('initial_pose_y', float('nan'))
        self.declare_parameter('initial_pose_yaw', float('nan'))
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('relative_to_trajectory_id', 0)
        self.declare_parameter('service_retry_sec', 0.15)
        self.declare_parameter('finish_settle_sec', 1.0)
        self.declare_parameter('start_trajectory_max_attempts', 5)
        self.declare_parameter('refine_min_score', 0.4)
        # 2D Pose Estimate 연속 클릭 시 finish/start가 TF를 폭주시키므로 쿨다운.
        self.declare_parameter('initialpose_cooldown_sec', 3.0)

        self._config_dir = self.get_parameter('configuration_directory').get_parameter_value().string_value
        self._config_base = self.get_parameter('configuration_basename').get_parameter_value().string_value
        self._use_saved = self.get_parameter('use_saved_mapping_origin').get_parameter_value().bool_value
        self._wait_for_rviz = self.get_parameter('wait_for_rviz_initial_pose').get_parameter_value().bool_value
        self._refine = self.get_parameter('refine_with_scan_matching').get_parameter_value().bool_value
        self._refine_xy_window = float(self.get_parameter('refine_xy_window_m').value)
        self._refine_xy_step = float(self.get_parameter('refine_xy_step_m').value)
        self._refine_yaw_window = float(self.get_parameter('refine_yaw_window_deg').value)
        self._refine_yaw_step = float(self.get_parameter('refine_yaw_step_deg').value)
        self._rel_traj = int(self.get_parameter('relative_to_trajectory_id').value)
        self._service_retry = float(self.get_parameter('service_retry_sec').value)
        self._refine_min_score = float(self.get_parameter('refine_min_score').value)
        self._finish_settle = float(self.get_parameter('finish_settle_sec').value)
        self._start_max_attempts = int(self.get_parameter('start_trajectory_max_attempts').value)
        self._initialpose_cooldown = float(self.get_parameter('initialpose_cooldown_sec').value)
        self._last_restart_mono = 0.0

        pbstream = self.get_parameter('pbstream_filename').get_parameter_value().string_value
        if pbstream and not os.path.isfile(pbstream):
            raise RuntimeError(f'pbstream not found: {pbstream}')
        if not os.path.isdir(self._config_dir):
            raise RuntimeError(f'configuration_directory not found: {self._config_dir}')

        x = _optional_float(self.get_parameter('initial_pose_x').value)
        y = _optional_float(self.get_parameter('initial_pose_y').value)
        yaw = _optional_float(self.get_parameter('initial_pose_yaw').value)
        self._manual_launch_pose = all(not math.isnan(v) for v in (x, y, yaw))
        self._fallback_pose = (x, y, yaw) if self._manual_launch_pose else _load_origin_pose(
            pbstream, self._use_saved
        )

        self._service_cb = MutuallyExclusiveCallbackGroup()
        self._finish_client = self.create_client(
            FinishTrajectory, '/finish_trajectory', callback_group=self._service_cb
        )
        self._start_client = self.create_client(
            StartTrajectory, '/start_trajectory', callback_group=self._service_cb
        )
        self._states_client = self.create_client(
            GetTrajectoryStates, '/get_trajectory_states', callback_group=self._service_cb
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_msg: OccupancyGrid | None = None
        self._latest_scan: LaserScan | None = None
        self._apply_pose: tuple[float, float, float] | None = None
        self._busy = False
        self._startup_done = False
        self._logged_wait = False
        self._warned_no_initialpose_pub = False
        self._lock = threading.Lock()

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            self._on_initialpose,
            10,
        )
        self.create_timer(0.1, self._on_tick)
        self.create_timer(3.0, self._check_initialpose_publishers)

        self.get_logger().info(
            f'Pose manager ready (refine={self._refine}, topic={initial_pose_topic})'
        )

    def _check_initialpose_publishers(self):
        topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value
        try:
            pub_count = self.count_publishers(topic)
        except Exception:
            return
        if pub_count > 0:
            self._warned_no_initialpose_pub = False
            return
        if self._warned_no_initialpose_pub or not self._wait_for_rviz:
            return
        self._warned_no_initialpose_pub = True
        self.get_logger().error(
            f'RViz가 {topic}에 연결되지 않음 (publisher=0). '
            'Jetson 데스크톱에서 launch와 같은 ROS로 RViz 실행:\n'
            '  ros2 run localization_layer run_localization_rviz.sh'
        )

    def _on_map(self, msg: OccupancyGrid):
        if msg.info.width > 0 and msg.info.height > 0:
            self._map_msg = msg

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg

    def _queue_pose(self, x: float, y: float, yaw: float, source: str):
        with self._lock:
            self._apply_pose = (x, y, yaw)
        self.get_logger().info(
            f'Queued pose from {source}: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°'
        )

    def _on_initialpose(self, msg: PoseWithCovarianceStamped):
        frame = msg.header.frame_id
        if frame and frame not in ('map', ''):
            self.get_logger().warn(f'Ignoring /initialpose frame_id={frame!r}')
            return
        now = time.monotonic()
        if (now - self._last_restart_mono) < self._initialpose_cooldown:
            self.get_logger().warn(
                f'Ignoring /initialpose (쿨다운 {self._initialpose_cooldown:.1f}s — '
                '연속 클릭하면 TF가 튀고 Cartographer가 꼬임)'
            )
            return
        pose = msg.pose.pose
        self._queue_pose(
            pose.position.x,
            pose.position.y,
            _quat_to_yaw(pose.orientation),
            'RViz 2D Pose Estimate',
        )

    def _services_ready(self) -> bool:
        return (
            self._finish_client.service_is_ready()
            and self._start_client.service_is_ready()
            and self._states_client.service_is_ready()
        )

    def _call_service(self, client, request, timeout_sec: float = 8.0):
        future = client.call_async(request)
        done = threading.Event()
        result: dict = {'resp': None}

        def _done_cb(fut):
            try:
                result['resp'] = fut.result()
            except Exception as exc:
                self.get_logger().error(f'service call exception: {exc}')
            finally:
                done.set()

        future.add_done_callback(_done_cb)
        if not done.wait(timeout_sec):
            self.get_logger().error('service call timed out')
            return None
        return result['resp']

    def _active_trajectory_ids(self) -> list[int]:
        resp = self._call_service(self._states_client, GetTrajectoryStates.Request(), 3.0)
        if resp is None or resp.status.code != 0:
            return []
        active = []
        for traj_id, state in zip(resp.trajectory_states.trajectory_id,
                                  resp.trajectory_states.trajectory_state):
            if state == _TRAJECTORY_ACTIVE:
                active.append(int(traj_id))
        return active

    def _wait_until_no_active(self, timeout_sec: float = 4.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self._active_trajectory_ids():
                return True
            time.sleep(0.05)
        return not self._active_trajectory_ids()

    def _finish_all_active(self) -> bool:
        active = sorted(self._active_trajectory_ids(), reverse=True)
        if not active:
            return True
        for traj_id in active:
            req = FinishTrajectory.Request()
            req.trajectory_id = traj_id
            resp = self._call_service(self._finish_client, req)
            if resp is None or resp.status.code != 0:
                self.get_logger().error(f'finish_trajectory({traj_id}) failed')
                return False
            self.get_logger().info(f'Finished trajectory {traj_id}')
        time.sleep(self._finish_settle)
        if not self._wait_until_no_active():
            self.get_logger().error('Active trajectory still running after finish')
            return False
        return True

    def _start_trajectory(self, x: float, y: float, yaw: float) -> bool:
        req = StartTrajectory.Request()
        req.configuration_directory = self._config_dir
        req.configuration_basename = self._config_base
        req.use_initial_pose = True
        req.relative_to_trajectory_id = self._rel_traj
        req.initial_pose = Pose()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.orientation = _yaw_to_quat(yaw)

        resp = self._call_service(self._start_client, req)
        if resp is None or resp.status.code != 0:
            msg = resp.status.message if resp is not None else 'timeout'
            self.get_logger().error(f'start_trajectory failed: {msg}')
            return False

        self.get_logger().info(
            f'Localization OK (traj={resp.trajectory_id}, '
            f'x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.0f}°)'
        )
        return True

    def _restart_localization(self, x: float, y: float, yaw: float) -> bool:
        for attempt in range(1, self._start_max_attempts + 1):
            if not self._finish_all_active():
                time.sleep(0.3)
                continue
            if self._start_trajectory(x, y, yaw):
                return True
            self.get_logger().warn(
                f'start_trajectory retry {attempt}/{self._start_max_attempts} '
                '(이전 trajectory가 scan 토픽 점유 중)'
            )
            time.sleep(0.5)
        return False

    def _process_apply(self) -> bool:
        with self._lock:
            if self._apply_pose is None:
                return False
            x, y, yaw = self._apply_pose
            self._apply_pose = None

        scan = self._latest_scan
        map_msg = self._map_msg
        if self._refine:
            if scan is None or map_msg is None or map_msg.info.width == 0:
                self.get_logger().warn('/map 또는 /scan 대기 중 — pose 재시도 예정')
                with self._lock:
                    self._apply_pose = (x, y, yaw)
                return True

            grid = np.array(map_msg.data, dtype=np.int16).reshape(
                (map_msg.info.height, map_msg.info.width)
            )
            clicked_score = _score_pose(scan, grid, map_msg, x, y, yaw)
            rx, ry, ryaw, best_score = _refine_pose(
                scan,
                map_msg,
                x,
                y,
                yaw,
                xy_window_m=self._refine_xy_window,
                xy_step_m=self._refine_xy_step,
                yaw_window_deg=self._refine_yaw_window,
                yaw_step_deg=self._refine_yaw_step,
            )
            if (
                math.isfinite(best_score)
                and best_score >= self._refine_min_score
                and best_score > clicked_score + 0.15
            ):
                self.get_logger().info(
                    f'Scan refine: ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}°) -> '
                    f'({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°), '
                    f'score {clicked_score:.2f}->{best_score:.2f}'
                )
                x, y, yaw = rx, ry, ryaw
            else:
                self.get_logger().info(
                    f'Scan refine skipped (clicked={clicked_score:.2f}, best={best_score:.2f}) '
                    '— RViz/seed pose 그대로 사용'
                )

        ok = self._restart_localization(x, y, yaw)
        if ok:
            self._last_restart_mono = time.monotonic()
        if not ok:
            with self._lock:
                self._apply_pose = (x, y, yaw)
            self.get_logger().error(
                'Localization 실패 — launch 터미널 확인 후 2D Pose Estimate 다시 클릭'
            )
        return ok

    def _on_tick(self):
        if not self._startup_done:
            if not self._services_ready():
                return
            self._startup_done = True
            if self._manual_launch_pose:
                x, y, yaw = self._fallback_pose
                self._queue_pose(x, y, yaw, 'launch args')
            elif self._wait_for_rviz and not self._logged_wait:
                self._logged_wait = True
                self.get_logger().warn(
                    '=== RViz: Fixed Frame=map ===\n'
                    '  1) 차가 있는 구간에 "2D Pose Estimate" 클릭 (방향까지)\n'
                    '  2) "Localization OK" 확인 → 스캔이 벽에 맞는지 확인\n'
                    '  (안 맞으면 다시 클릭. 자동 점프 없음)'
                )
            elif not self._wait_for_rviz:
                x, y, yaw = self._fallback_pose
                self._queue_pose(x, y, yaw, 'fallback pose')

        if self._busy:
            return
        with self._lock:
            has_work = self._apply_pose is not None
        if not has_work:
            return

        if not self._services_ready():
            return

        self._busy = True
        try:
            ok = self._process_apply()
            if not ok:
                with self._lock:
                    if self._apply_pose is None:
                        self.get_logger().error(
                            'Pose apply failed — click 2D Pose Estimate again.'
                        )
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationInitialPoseSetter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
