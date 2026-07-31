#!/usr/bin/env python3
"""런치 터미널용 한 줄 상태 — 정상 시 '정상작동중'만 갱신."""
from __future__ import annotations

import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


def _age(last: float | None) -> float | None:
    if last is None:
        return None
    return time.monotonic() - last


class StackStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("stack_status_node")
        self.declare_parameter("period_s", 1.0)
        self.declare_parameter("scan_stale_s", 1.0)
        self.declare_parameter("path_stale_s", 1.5)
        self.declare_parameter("drive_stale_s", 1.5)

        self._period = max(0.2, float(self.get_parameter("period_s").value))
        self._scan_stale = float(self.get_parameter("scan_stale_s").value)
        self._path_stale = float(self.get_parameter("path_stale_s").value)
        self._drive_stale = float(self.get_parameter("drive_stale_s").value)

        self._t_scan: float | None = None
        self._t_path: float | None = None
        self._t_drive: float | None = None
        self._t_static: float | None = None
        self._t_dyn: float | None = None
        self._n_static = 0
        self._n_dyn = 0

        self.create_subscription(LaserScan, "/scan", self._cb_scan, 10)
        self.create_subscription(Path, "/local_path", self._cb_path, 10)
        self.create_subscription(
            AckermannDriveStamped, "/drive", self._cb_drive, 10
        )
        self.create_subscription(
            Float32MultiArray, "/static_obstacles", self._cb_static, 10
        )
        self.create_subscription(
            Float32MultiArray, "/dynamic_obstacles", self._cb_dyn, 10
        )
        self.create_timer(self._period, self._tick)

    def _cb_scan(self, _msg: LaserScan) -> None:
        self._t_scan = time.monotonic()

    def _cb_path(self, _msg: Path) -> None:
        self._t_path = time.monotonic()

    def _cb_drive(self, _msg: AckermannDriveStamped) -> None:
        self._t_drive = time.monotonic()

    def _cb_static(self, msg: Float32MultiArray) -> None:
        self._t_static = time.monotonic()
        self._n_static = len(msg.data) // 4

    def _cb_dyn(self, msg: Float32MultiArray) -> None:
        self._t_dyn = time.monotonic()
        self._n_dyn = len(msg.data) // 6

    def _tick(self) -> None:
        missing: list[str] = []
        a_scan = _age(self._t_scan)
        a_path = _age(self._t_path)
        a_drive = _age(self._t_drive)

        if a_scan is None or a_scan > self._scan_stale:
            missing.append("scan")

        path_ok = a_path is not None and a_path <= self._path_stale
        drive_ok = a_drive is not None and a_drive <= self._drive_stale
        if not (path_ok or drive_ok):
            missing.append("local_path|drive")

        if not missing:
            src = "local_path" if path_ok else "drive"
            if path_ok and drive_ok:
                src = "local_path+drive"
            line = (
                f"[stack] 정상작동중  ({src})  "
                f"static={self._n_static}  dynamic={self._n_dyn}"
            )
        else:
            line = f"[stack] 대기/이상  — 미수신: {', '.join(missing)}"

        if sys.stdout.isatty():
            sys.stdout.write("\r\033[K" + line)
        else:
            sys.stdout.write(line + "\n")
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = StackStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if sys.stdout.isatty():
            sys.stdout.write("\n")
            sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
