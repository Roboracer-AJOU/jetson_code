#!/usr/bin/env python3

import math
import os
import subprocess
import threading
from datetime import datetime

import rclpy
import yaml
from cartographer_ros_msgs.srv import TrajectoryQuery, WriteState
from PIL import Image
from rclpy.node import Node

# ROS 맵 규약에서 unknown 픽셀은 205 다. 그 점유도는
#   1 - 205/255 = 0.196078
# 이라서 free_thresh 는 이 값보다 **작아야** unknown 이 free 로 안 넘어온다.
# ROS 기본값 0.196 은 우연이 아니라 딱 이 목적으로 정해진 값이다.
#
# 그런데 map_saver_cli 와 cartographer_pbstream_to_ros_map 은 0.25 를 써 놓는다.
# 그러면 미탐색 영역(맵 대부분)이 전부 주행 가능으로 분류된다. 실제로
# 332x494 맵에서 free 가 98.1% 로 잡혀 중앙선 추출이 폐루프를 못 찾았다 —
# 트랙 밖으로 도로가 새어 나가 인필드 섬이 섬이 아니게 되기 때문이다.
#
# 두 툴 다 이 값을 인자로 받지 않으므로 내보낸 뒤 YAML 을 고친다.
MAP_FREE_THRESH = 0.196


class MapAutoSaver(Node):
    def __init__(self):
        super().__init__('map_auto_saver')

        self.declare_parameter('map_save_dir', '/home/nvidia/f1tenth_ajou/maps')
        self.declare_parameter('map_file_prefix', 'cartographer_map')
        self.declare_parameter('include_unfinished_submaps', True)
        self.declare_parameter('save_on_shutdown', True)
        self.declare_parameter('save_interval_sec', 60.0)
        self.declare_parameter('export_ros_map', True)
        self.declare_parameter('ros_map_topic', '/map')
        self.declare_parameter('ros_map_format', 'png')
        self.declare_parameter('ros_map_mode', 'trinary')
        self.declare_parameter('ros_map_timeout_sec', 45.0)
        self.declare_parameter('write_state_timeout_sec', 120.0)
        self.declare_parameter('service_wait_timeout_sec', 5.0)
        self.declare_parameter('export_ros_map_on_shutdown', True)
        self.declare_parameter('shutdown_write_state_timeout_sec', 120.0)
        self.declare_parameter('shutdown_ros_map_timeout_sec', 60.0)
        self.declare_parameter('pbstream_to_ros_map_resolution', 0.05)
        self.declare_parameter('min_pbstream_bytes', 4096)

        self.map_save_dir = self.get_parameter('map_save_dir').get_parameter_value().string_value
        if not os.path.isabs(self.map_save_dir):
            self.map_save_dir = os.path.abspath(self.map_save_dir)
        self.map_file_prefix = self.get_parameter('map_file_prefix').get_parameter_value().string_value
        self.include_unfinished_submaps = self.get_parameter(
            'include_unfinished_submaps'
        ).get_parameter_value().bool_value
        self.save_on_shutdown = self.get_parameter('save_on_shutdown').get_parameter_value().bool_value
        self.save_interval_sec = self.get_parameter('save_interval_sec').get_parameter_value().double_value
        self.export_ros_map = self.get_parameter('export_ros_map').get_parameter_value().bool_value
        self.ros_map_topic = self.get_parameter('ros_map_topic').get_parameter_value().string_value
        self.ros_map_format = self.get_parameter('ros_map_format').get_parameter_value().string_value
        self.ros_map_mode = self.get_parameter('ros_map_mode').get_parameter_value().string_value
        self.ros_map_timeout_sec = self.get_parameter('ros_map_timeout_sec').get_parameter_value().double_value
        self.write_state_timeout_sec = self.get_parameter(
            'write_state_timeout_sec'
        ).get_parameter_value().double_value
        self.service_wait_timeout_sec = self.get_parameter(
            'service_wait_timeout_sec'
        ).get_parameter_value().double_value
        self.export_ros_map_on_shutdown = self.get_parameter(
            'export_ros_map_on_shutdown'
        ).get_parameter_value().bool_value
        self.shutdown_write_state_timeout_sec = self.get_parameter(
            'shutdown_write_state_timeout_sec'
        ).get_parameter_value().double_value
        self.shutdown_ros_map_timeout_sec = self.get_parameter(
            'shutdown_ros_map_timeout_sec'
        ).get_parameter_value().double_value
        self.pbstream_to_ros_map_resolution = self.get_parameter(
            'pbstream_to_ros_map_resolution'
        ).get_parameter_value().double_value
        self.min_pbstream_bytes = self.get_parameter(
            'min_pbstream_bytes'
        ).get_parameter_value().integer_value

        os.makedirs(self.map_save_dir, exist_ok=True)
        self.ros_log_dir = os.path.join(self.map_save_dir, '.roslog')
        os.makedirs(self.ros_log_dir, exist_ok=True)

        self.write_state_client = self.create_client(WriteState, '/write_state')
        self.trajectory_query_client = self.create_client(TrajectoryQuery, '/trajectory_query')
        self._save_lock = threading.Lock()
        self._shutdown_save_done = False

        if self.save_interval_sec > 0.0:
            self.create_timer(self.save_interval_sec, self._periodic_save_callback)
            self.get_logger().info(
                f'Periodic auto-save: every {self.save_interval_sec:.0f}s -> {self.map_save_dir}'
            )
        else:
            self.get_logger().warn(
                'Periodic auto-save OFF (save_interval_sec=0). '
                'pbstream/png는 Ctrl+C 종료 시에만 저장됩니다.'
            )

    def _context_ok(self) -> bool:
        try:
            return rclpy.ok() and self.context.ok()
        except Exception:
            return False

    def _timestamped_stem(self) -> str:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.join(self.map_save_dir, f'{self.map_file_prefix}_{ts}')

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _write_mapping_origin_yaml(
        self,
        map_filestem: str,
        pbstream_path: str,
        ros_map_yaml: str | None = None,
    ) -> None:
        pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

        if self.trajectory_query_client.wait_for_service(timeout_sec=1.0):
            req = TrajectoryQuery.Request()
            req.trajectory_id = 0
            future = self.trajectory_query_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result() is not None and future.result().status.code == 0:
                trajectory = future.result().trajectory
                if trajectory:
                    first = trajectory[0].pose
                    pose = {
                        'x': float(first.position.x),
                        'y': float(first.position.y),
                        'z': float(first.position.z),
                        'yaw': float(self._quat_to_yaw(first.orientation)),
                    }

        origin_path = f'{map_filestem}_origin.yaml'
        data = {
            'map_frame': 'map',
            'relative_to_trajectory_id': 0,
            'initial_pose': pose,
            'pbstream': os.path.abspath(pbstream_path),
            'ros_map_yaml': ros_map_yaml or '',
            'note': (
                'Localization should start from this map-frame pose so waypoint CSV '
                'coordinates match the saved Cartographer map.'
            ),
        }
        with open(origin_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(f'Saved mapping origin -> {origin_path}')

    def _fix_map_yaml_free_thresh(self, yaml_path: str) -> None:
        """내보낸 맵 YAML 의 free_thresh 를 ROS 규약값으로 되돌린다."""
        name = os.path.basename(yaml_path)
        if not os.path.exists(yaml_path):
            # 내보내기 자체가 실패한 경우다. 그건 호출부가 이미 보고한다.
            return

        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                meta = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f'맵 YAML 을 읽지 못해 free_thresh 보정 생략: {exc}')
            return

        old = meta.get('free_thresh')
        if old is not None and abs(float(old) - MAP_FREE_THRESH) < 1e-9:
            self.get_logger().info(f'맵 YAML free_thresh 이미 {MAP_FREE_THRESH} {name}')
            return

        meta['free_thresh'] = MAP_FREE_THRESH
        try:
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None)
        except Exception as exc:
            self.get_logger().warn(f'맵 YAML free_thresh 보정 실패: {exc}')
            return

        # 되읽어 확인한다. 여기서 어긋나면 다른 프로세스가 덮어쓴 것이므로
        # 조용히 넘기면 안 된다. 이걸 놓쳐서 0.25 맵이 계속 저장됐다.
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                got = (yaml.safe_load(f) or {}).get('free_thresh')
        except Exception as exc:
            self.get_logger().warn(f'맵 YAML free_thresh 재확인 실패: {exc}')
            return

        if got is None or abs(float(got) - MAP_FREE_THRESH) >= 1e-9:
            self.get_logger().error(
                f'맵 YAML free_thresh 보정이 유지되지 않았다 (={got}). '
                f'다른 프로세스가 덮어쓰는지 확인 필요: {name}'
            )
            return

        self.get_logger().info(
            f'맵 YAML free_thresh {old} -> {MAP_FREE_THRESH} '
            f'(unknown 을 free 로 세지 않게) {name}'
        )

    def _save_ros_map(self, map_filestem: str, reason: str, timeout_sec: float) -> bool:
        if timeout_sec <= 0.0:
            return True

        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-t', self.ros_map_topic,
            '-f', map_filestem,
            '--fmt', self.ros_map_format,
            '--mode', self.ros_map_mode,
        ]
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = self.ros_log_dir

        self.get_logger().info(
            f'Exporting ROS map ({reason}) -> {map_filestem}.{self.ros_map_format} + .yaml'
        )
        try:
            completed = subprocess.run(
                cmd,
                check=False,
                timeout=timeout_sec,
                env=env,
                capture_output=True,
                text=True,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli timed out.')
            return False
        except Exception as exc:
            self.get_logger().error(f'Failed to run map_saver_cli: {exc}')
            return False

        # 보정은 종료코드와 무관하게 시도한다. map_saver_cli 는 파일을 다 써
        # 놓고도 1 을 반환하는 경우가 있어, 성공 코드에만 걸어 두면 0.25 인
        # YAML 이 그대로 남는다.
        self._fix_map_yaml_free_thresh(f'{map_filestem}.yaml')

        if completed.returncode == 0:
            self.get_logger().info('ROS map export succeeded.')
            return True

        self.get_logger().error(
            f'ROS map export failed (code={completed.returncode}). '
            f'stdout="{completed.stdout.strip()}" stderr="{completed.stderr.strip()}"'
        )
        return False

    def _save_ros_map_from_pbstream(
        self,
        map_filestem: str,
        pbstream_path: str,
        reason: str,
        timeout_sec: float,
    ) -> bool:
        if timeout_sec <= 0.0:
            return False

        out_filestem = f'{map_filestem}_rosmap'
        cmd = [
            'ros2',
            'run',
            'cartographer_ros',
            'cartographer_pbstream_to_ros_map',
            '-pbstream_filename',
            pbstream_path,
            '-map_filestem',
            out_filestem,
            '-resolution',
            f'{self.pbstream_to_ros_map_resolution:.6f}',
        ]
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = self.ros_log_dir

        self.get_logger().warn(
            f'Falling back to pbstream->ros_map export ({reason}) -> {out_filestem}.yaml'
        )
        try:
            completed = subprocess.run(
                cmd,
                check=False,
                timeout=timeout_sec,
                env=env,
                capture_output=True,
                text=True,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('pbstream_to_ros_map timed out.')
            return False
        except Exception as exc:
            self.get_logger().error(f'Failed to run pbstream_to_ros_map: {exc}')
            return False

        if completed.returncode != 0:
            self.get_logger().error(
                f'pbstream_to_ros_map failed (code={completed.returncode}). '
                f'stdout="{completed.stdout.strip()}" stderr="{completed.stderr.strip()}"'
            )
            return False

        # cartographer tool writes .pgm + .yaml. Convert to .png when requested.
        if self.ros_map_format.strip().lower() == 'png':
            pgm_path = f'{out_filestem}.pgm'
            png_path = f'{out_filestem}.png'
            yaml_path = f'{out_filestem}.yaml'
            try:
                Image.open(pgm_path).save(png_path)
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    yaml_text = f.read()
                yaml_text = yaml_text.replace(os.path.basename(pgm_path), os.path.basename(png_path))
                with open(yaml_path, 'w', encoding='utf-8') as f:
                    f.write(yaml_text)
            except Exception as exc:
                self.get_logger().error(f'Failed to convert fallback map to png: {exc}')
                return False

        self._fix_map_yaml_free_thresh(f'{out_filestem}.yaml')
        self.get_logger().info('Fallback ROS map export from pbstream succeeded.')
        return True

    def _should_export_ros_map(self, on_shutdown: bool) -> bool:
        if on_shutdown:
            return self.export_ros_map or self.export_ros_map_on_shutdown
        return self.export_ros_map

    def save_map(
        self,
        reason: str,
        *,
        service_wait_timeout_sec: float | None = None,
        write_state_timeout_sec: float | None = None,
        export_ros_map: bool | None = None,
        ros_map_timeout_sec: float | None = None,
        allow_when_context_invalid: bool = False,
        on_shutdown: bool = False,
    ) -> bool:
        with self._save_lock:
            if not allow_when_context_invalid and not self._context_ok():
                return False

            service_wait_timeout_sec = (
                self.service_wait_timeout_sec
                if service_wait_timeout_sec is None
                else service_wait_timeout_sec
            )
            write_state_timeout_sec = (
                self.write_state_timeout_sec
                if write_state_timeout_sec is None
                else write_state_timeout_sec
            )
            export_ros_map = (
                self._should_export_ros_map(on_shutdown)
                if export_ros_map is None
                else export_ros_map
            )
            ros_map_timeout_sec = (
                self.ros_map_timeout_sec
                if ros_map_timeout_sec is None
                else ros_map_timeout_sec
            )

            try:
                service_ready = self.write_state_client.wait_for_service(
                    timeout_sec=service_wait_timeout_sec
                )
            except Exception:
                return False

            if not service_ready:
                self.get_logger().error('/write_state service is not available.')
                return False

            map_filestem = self._timestamped_stem()
            output_path = f'{map_filestem}.pbstream'
            req = WriteState.Request()
            req.filename = output_path
            req.include_unfinished_submaps = self.include_unfinished_submaps

            self.get_logger().info(f'Saving map ({reason}) -> {output_path}')
            future = self.write_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=write_state_timeout_sec)

            if future.done() and future.result() is not None:
                if os.path.getsize(output_path) < self.min_pbstream_bytes:
                    self.get_logger().warn(
                        f'Map save skipped export: pbstream too small ({os.path.getsize(output_path)} bytes). '
                        'Drive the car slowly so Cartographer can build a map first.'
                    )
                    return False

                self.get_logger().info('Map saved successfully.')
                ros_map_yaml = None
                if export_ros_map:
                    if self._save_ros_map(map_filestem, reason, ros_map_timeout_sec):
                        ros_map_yaml = f'{map_filestem}.yaml'
                    elif self._save_ros_map_from_pbstream(
                        map_filestem,
                        output_path,
                        reason,
                        ros_map_timeout_sec,
                    ):
                        ros_map_yaml = f'{map_filestem}_rosmap.yaml'
                self._write_mapping_origin_yaml(map_filestem, output_path, ros_map_yaml)
                return True

            self.get_logger().error('Map save failed or timed out.')
            return False

    def _periodic_save_callback(self):
        # Run periodic save outside timer callback to avoid executor deadlock
        # while waiting for /write_state service completion.
        threading.Thread(
            target=self.save_map,
            args=('periodic',),
            daemon=True,
        ).start()

    def save_once_on_shutdown(self):
        if not self.save_on_shutdown or self._shutdown_save_done:
            return
        self._shutdown_save_done = True
        if not self._context_ok():
            return
        self.save_map(
            'shutdown',
            service_wait_timeout_sec=min(3.0, self.service_wait_timeout_sec),
            write_state_timeout_sec=self.shutdown_write_state_timeout_sec,
            ros_map_timeout_sec=self.shutdown_ros_map_timeout_sec,
            on_shutdown=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapAutoSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Try final save immediately on Ctrl+C before/while context teardown.
        try:
            node.get_logger().info('KeyboardInterrupt received. Attempting shutdown save...')
        except Exception:
            pass
        node._shutdown_save_done = True
        node.save_map(
            'shutdown',
            service_wait_timeout_sec=min(3.0, node.service_wait_timeout_sec),
            write_state_timeout_sec=node.shutdown_write_state_timeout_sec,
            ros_map_timeout_sec=node.shutdown_ros_map_timeout_sec,
            allow_when_context_invalid=True,
            on_shutdown=True,
        )
    finally:
        node.save_once_on_shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
