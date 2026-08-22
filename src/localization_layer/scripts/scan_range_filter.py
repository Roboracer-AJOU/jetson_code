#!/usr/bin/env python3
"""Drop LiDAR returns beyond max_range_m before AMCL.

On a loop track AMCL with 25m range sees the opposite straight / parallel wall and
locks to the wrong pose (drifts in every direction in corners). Cartographer uses
submaps; AMCL needs a local window of walls only.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRangeFilter(Node):
    def __init__(self) -> None:
        super().__init__('scan_range_filter')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_amcl')
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('min_range_m', 0.12)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._max_range = float(self.get_parameter('max_range_m').value)
        self._min_range = float(self.get_parameter('min_range_m').value)

        self._pub = self.create_publisher(LaserScan, output_topic, 10)
        self.create_subscription(LaserScan, input_topic, self._on_scan, 10)
        self.get_logger().info(
            f'scan_range_filter: {input_topic} -> {output_topic}, '
            f'keep [{self._min_range:.2f}, {self._max_range:.2f}] m'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = min(msg.range_max, self._max_range)
        out.ranges = []
        for r in msg.ranges:
            if not math.isfinite(r) or r < self._min_range or r > self._max_range:
                out.ranges.append(float('inf'))
            else:
                out.ranges.append(r)
        out.intensities = list(msg.intensities) if msg.intensities else []
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRangeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
