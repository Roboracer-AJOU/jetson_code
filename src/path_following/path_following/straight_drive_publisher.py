#!/usr/bin/env python3
"""
[임시] MANUAL 직진 속도 실험용 /drive 발행기.

  - CH5 MANUAL 일 때만 /drive 발행 (AUTO 전환 시 즉시 발행 중지)
  - control_node 와 동일 PI 목표속도

사용:
  ros2 run path_following control_node
  ros2 run path_following straight_drive_publisher --ros-args -p target_speed_mps:=1.0
"""
from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


CFG = {
    "drive_topic": "/drive",
    "telemetry_topic": "/vehicle/telemetry",
    "measured_speed_topic": "/vehicle/speed_mps",
    "publish_hz": 40.0,
    "status_log_hz": 2.0,
    "enabled": True,
    "target_speed_mps": 4.0,
    "speed_sign": -1.0,           # 전진 duty 방향 (후진이면 1.0 으로 토글)
    "steering_angle_rad": 0.0,
    "frame_id": "base_link",
}


def _param_bool(val) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).lower() in ("1", "true", "yes")


class StraightDrivePublisher(Node):
    def __init__(self) -> None:
        super().__init__("straight_drive_publisher")

        for key, value in CFG.items():
            self.declare_parameter(key, value)

        self._drive_topic = str(self.get_parameter("drive_topic").value)
        self._telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self._speed_topic = str(self.get_parameter("measured_speed_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self._status_log_hz = max(0.0, float(self.get_parameter("status_log_hz").value))
        self._status_log_period = 1.0 / self._status_log_hz if self._status_log_hz > 0 else 0.0
        self._status_log_accum = 0.0

        self._v_act = 0.0
        self._v_act_fresh = False
        self._telemetry_fresh = False
        self._autonomous = False
        self._was_autonomous = False

        self._pub = self.create_publisher(AckermannDriveStamped, self._drive_topic, 10)
        self.create_subscription(Float64, self._speed_topic, self._cb_speed, 10)
        self.create_subscription(
            Float64MultiArray, self._telemetry_topic, self._cb_telemetry, 10
        )
        self.create_timer(1.0 / hz, self._tick)

        sign = float(self.get_parameter("speed_sign").value)
        if not math.isfinite(sign) or sign == 0.0:
            sign = 1.0
        tgt = float(self.get_parameter("target_speed_mps").value)
        self.get_logger().info(
            f"[TEMP] straight_drive_publisher @ {hz:.0f}Hz | "
            f"target={tgt:.2f} m/s speed_sign={sign:+.0f} | CH5 MANUAL 일 때만 발행"
        )

    def _cb_telemetry(self, msg: Float64MultiArray) -> None:
        if len(msg.data) <= 6:
            return
        self._telemetry_fresh = True
        self._autonomous = float(msg.data[6]) > 0.5

    def _cb_speed(self, msg: Float64) -> None:
        v = float(msg.data)
        if math.isfinite(v):
            self._v_act = abs(v)
            self._v_act_fresh = True

    def _tick(self) -> None:
        if not _param_bool(self.get_parameter("enabled").value):
            return

        if not self._telemetry_fresh:
            return

        if self._autonomous:
            if not self._was_autonomous:
                self.get_logger().info("AUTO 감지 — /drive 발행 중지")
                self._was_autonomous = True
            return

        if self._was_autonomous:
            self.get_logger().info("MANUAL 복귀 — /drive 발행 재개")
            self._was_autonomous = False

        target = float(self.get_parameter("target_speed_mps").value)
        if not math.isfinite(target) or target < 0.0:
            target = 0.0
        sign = float(self.get_parameter("speed_sign").value)
        if not math.isfinite(sign) or sign == 0.0:
            sign = 1.0

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.drive.speed = target * sign
        msg.drive.steering_angle = float(self.get_parameter("steering_angle_rad").value)
        self._pub.publish(msg)
        self._maybe_log(target)

    def _maybe_log(self, target: float) -> None:
        if self._status_log_period <= 0.0:
            return
        self._status_log_accum += 1.0 / max(1.0, float(self.get_parameter("publish_hz").value))
        if self._status_log_accum < self._status_log_period:
            return
        self._status_log_accum = 0.0
        if self._v_act_fresh:
            self.get_logger().info(
                f"SPEED_TEST | target={target:.2f} v_act={self._v_act:.2f} "
                f"err={target - self._v_act:+.2f} m/s"
            )
        else:
            self.get_logger().info(f"SPEED_TEST | target={target:.2f} v_act=—")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightDrivePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
