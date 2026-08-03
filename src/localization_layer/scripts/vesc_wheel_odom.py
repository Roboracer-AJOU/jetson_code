#!/usr/bin/env python3
"""Wheel odometry from VESC measured speed + ESP servo feedback angle.

Speed feedback: /vehicle/speed_mps (Float64, m/s from VESC ERPM via control_node)
Steer feedback: /esp32/target_angle_deg (abs deg, 90=center, left+, right-)
       fallback: /esp32/servo_command_deg, then /drive.steering_angle (rad)

Publishes /odom only (no TF) so Cartographer can provide_odom_frame.
"""

from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float64


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class VescWheelOdom(Node):
    def __init__(self) -> None:
        super().__init__('vesc_wheel_odom')

        self.declare_parameter('wheelbase_m', 0.33)
        self.declare_parameter('speed_topic', '/vehicle/speed_mps')
        # Primary: servo feedback (user: /esp32/target_angle_deg)
        self.declare_parameter('servo_feedback_deg_topic', '/esp32/target_angle_deg')
        self.declare_parameter('servo_command_deg_topic', '/esp32/servo_command_deg')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('center_servo_deg', 90.0)
        # ESP abs deg span used as ±1.0 (50°/130° → ±40°). Road-wheel angle is smaller.
        self.declare_parameter('servo_span_deg', 40.0)
        self.declare_parameter('max_steer_rad', 0.349)  # ~20° road wheel (was 40° → yaw 과대)
        self.declare_parameter('steer_scale', 1.0)
        self.declare_parameter('invert_steer', False)
        # False: 속도만 적분 (코너에서 틀린 조향 yaw prior가 헤딩을 망가뜨리지 않음)
        self.declare_parameter('use_steer_for_yaw', False)
        self.declare_parameter('min_speed_for_yaw_mps', 0.05)
        # Cartographer는 odom을 켜면 스캔 간 예측의 각속도를 odom 것으로 갈아탄다.
        # steer_scale은 정상상태 이득만 깎을 뿐 급조향 순간의 스파이크는 그대로 전달됨.
        # 여기서 발행되는 yaw(및 그걸로 적분하는 x,y)를 낮은 통과 필터로 한 번 더 눌러서
        # 코너 진입/탈출 같은 순간 오차가 prior를 확 흔들지 못하게 한다 (지속 회전은 계속 따라감).
        self.declare_parameter('yaw_filter_tau_sec', 0.3)
        # 종방향 스케일. 한 바퀴 돌수록 점점 밀리면 0.95~1.05로 조절.
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('publish_hz', 50.0)

        self._wheelbase = max(1e-3, float(self.get_parameter('wheelbase_m').value))
        self._center_deg = float(self.get_parameter('center_servo_deg').value)
        self._servo_span = max(1e-3, float(self.get_parameter('servo_span_deg').value))
        self._max_steer = max(1e-3, float(self.get_parameter('max_steer_rad').value))
        self._steer_scale = max(0.0, float(self.get_parameter('steer_scale').value))
        self._invert_steer = bool(self.get_parameter('invert_steer').value)
        self._use_steer_for_yaw = bool(self.get_parameter('use_steer_for_yaw').value)
        self._min_speed_yaw = max(0.0, float(self.get_parameter('min_speed_for_yaw_mps').value))
        self._speed_scale = float(self.get_parameter('speed_scale').value)
        self._yaw_tau = max(0.0, float(self.get_parameter('yaw_filter_tau_sec').value))
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        hz = max(1.0, float(self.get_parameter('publish_hz').value))

        self._v = 0.0
        self._steer_rad = 0.0
        self._have_speed = False
        self._steer_source = 'none'
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._yaw_raw = 0.0  # full-trust integrated yaw, chased by the filtered self._yaw
        self._last_time = self.get_clock().now()

        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value
        feedback_topic = self.get_parameter(
            'servo_feedback_deg_topic'
        ).get_parameter_value().string_value
        command_topic = self.get_parameter(
            'servo_command_deg_topic'
        ).get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.create_subscription(Float64, speed_topic, self._on_speed, 10)
        self.create_subscription(Float32, feedback_topic, self._on_feedback_deg, 10)
        self.create_subscription(Float32, command_topic, self._on_command_deg, 10)
        self.create_subscription(AckermannDriveStamped, drive_topic, self._on_drive, 10)
        self.create_timer(1.0 / hz, self._on_timer)

        self.get_logger().info(
            f'VESC wheel odom: speed_fb={speed_topic}, '
            f'steer_fb={feedback_topic} (center={self._center_deg}°, '
            f'span=±{self._servo_span:.0f}°→±{math.degrees(self._max_steer):.0f}° wheel, '
            f'scale={self._steer_scale:.2f}, invert_steer={self._invert_steer}, '
            f'use_steer_for_yaw={self._use_steer_for_yaw}, speed_scale={self._speed_scale:.3f}, '
            f'yaw_filter_tau={self._yaw_tau:.2f}s), '
            f'L={self._wheelbase:.3f}m, out={odom_topic} @ {hz:.0f}Hz (no TF)'
        )

    def _on_speed(self, msg: Float64) -> None:
        if math.isfinite(msg.data):
            self._v = float(msg.data) * self._speed_scale
            self._have_speed = True

    def _set_steer_from_deg(self, deg: float, source: str) -> None:
        if not math.isfinite(deg):
            return
        # Absolute servo 90=center. Map servo span → road-wheel angle.
        norm = (float(deg) - self._center_deg) / self._servo_span  # ≈ [-1, 1]
        norm = max(-1.0, min(1.0, norm))
        if self._invert_steer:
            norm = -norm
        steer = norm * self._max_steer * self._steer_scale
        self._steer_rad = max(-self._max_steer, min(self._max_steer, steer))
        self._steer_source = source

    def _on_feedback_deg(self, msg: Float32) -> None:
        self._set_steer_from_deg(msg.data, 'target_angle_deg')

    def _on_command_deg(self, msg: Float32) -> None:
        # Only if feedback has never arrived.
        if self._steer_source == 'target_angle_deg':
            return
        self._set_steer_from_deg(msg.data, 'servo_command_deg')

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        if self._steer_source in ('target_angle_deg', 'servo_command_deg'):
            return
        steer = float(msg.drive.steering_angle)
        if not math.isfinite(steer):
            return
        self._steer_rad = max(-self._max_steer, min(self._max_steer, steer))
        self._steer_source = 'drive'

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        v = self._v
        steer = max(-self._max_steer, min(self._max_steer, self._steer_rad))
        # 조향 yaw는 코너에서 스케일/부호 오차가 커서 prior를 망가뜨림 → 기본 off.
        # 병진만 적분하고 헤딩은 LiDAR scan match가 담당.
        if (not self._use_steer_for_yaw) or abs(v) < self._min_speed_yaw:
            omega = 0.0
        else:
            omega = (v / self._wheelbase) * math.tan(steer)

        self._yaw_raw += omega * dt
        # Low-pass the yaw actually published: sustained turns still get tracked,
        # but a sudden steering-yaw error (corner entry/exit) only leaks in gradually.
        if self._yaw_tau > 1e-6:
            alpha = dt / (self._yaw_tau + dt)
        else:
            alpha = 1.0
        dyaw = math.atan2(
            math.sin(self._yaw_raw - self._yaw), math.cos(self._yaw_raw - self._yaw)
        )
        filtered_omega = (alpha * dyaw) / dt if dt > 1e-6 else 0.0
        self._yaw += alpha * dyaw

        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation = _yaw_to_quat(self._yaw)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = filtered_omega

        # Modest covariance: trust enough for Cartographer prior, not more than LiDAR.
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.1
        msg.twist.covariance[0] = 0.1
        msg.twist.covariance[35] = 0.2

        self._odom_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VescWheelOdom()
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
