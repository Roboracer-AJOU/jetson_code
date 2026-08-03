#!/usr/bin/env python3
"""Wheel odometry from VESC measured speed + IMU gyro yaw rate.

Speed feedback: /vehicle/speed_mps (Float64, m/s from VESC ERPM via control_node)
Yaw rate feedback: /imu/data (sensor_msgs/Imu, angular_velocity.<imu_yaw_axis>)
       조향각 기반 yaw 추정(서보각→bicycle model)은 오차가 커서 제거함 — IMU 실측으로 대체.

Publishes /odom only (no TF) so Cartographer can provide_odom_frame.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _yaw_from_quat(q: Quaternion) -> float:
    """EBIMU(9-DOF)가 지자기까지 융합해서 낸 orientation에서 yaw만 추출."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class VescWheelOdom(Node):
    def __init__(self) -> None:
        super().__init__('vesc_wheel_odom')

        self.declare_parameter('speed_topic', '/vehicle/speed_mps')
        self.declare_parameter('imu_topic', '/imu/data')
        # 아직 gz/gy 중 어느 축이 실제 yaw(좌우 회전)인지 실측 확인 전 — 확인되면 여기만 바꾸면 됨.
        # az≈1g로 측정돼서 z축이 위(수직)로 추정 → 기본은 'z'.
        self.declare_parameter('imu_yaw_axis', 'z')
        # 'gyro': 각속도만 직접 적분 (지자기 간섭 영향 없지만 장기 드리프트).
        # 'orientation': EBIMU 자체 지자기 융합 yaw(msg.orientation)를 그대로 사용
        #   (장기 드리프트는 덜하지만, 모터 근처 지자기 간섭에 취약할 수 있음).
        # 'fused': 평소엔 자이로 적분, 아주 느리게(yaw_fusion_tau_sec)만 orientation
        #   쪽으로 당겨서 장기 드리프트만 서서히 보정 (순간 지자기 튐은 무시됨) — 기본값.
        self.declare_parameter('yaw_source', 'fused')
        self.declare_parameter('yaw_fusion_tau_sec', 5.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_speed_for_yaw_mps', 0.05)
        # IMU 자이로는 실측이라 조향각 추정보다 신뢰도가 높음 — 그래도 정지 중 노이즈로
        # 헤딩이 흔들리는 걸 막기 위해 약하게만 눌러줌.
        self.declare_parameter('yaw_filter_tau_sec', 0.1)
        # 종방향 스케일. 한 바퀴 돌수록 점점 밀리면 0.95~1.05로 조절.
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('publish_hz', 50.0)

        self._imu_yaw_axis = self.get_parameter(
            'imu_yaw_axis'
        ).get_parameter_value().string_value
        self._yaw_source = self.get_parameter('yaw_source').get_parameter_value().string_value
        self._yaw_fusion_tau = max(0.0, float(self.get_parameter('yaw_fusion_tau_sec').value))
        self._min_speed_yaw = max(0.0, float(self.get_parameter('min_speed_for_yaw_mps').value))
        self._speed_scale = float(self.get_parameter('speed_scale').value)
        self._yaw_tau = max(0.0, float(self.get_parameter('yaw_filter_tau_sec').value))
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        hz = max(1.0, float(self.get_parameter('publish_hz').value))

        self._v = 0.0
        self._omega_imu = 0.0
        self._have_speed = False
        self._have_imu = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._yaw_raw = 0.0  # full-trust integrated yaw, chased by the filtered self._yaw
        self._last_time = self.get_clock().now()
        self._last_imu_stamp: Time | None = None

        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.create_subscription(Float64, speed_topic, self._on_speed, 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_timer(1.0 / hz, self._on_timer)

        self.get_logger().info(
            f'VESC wheel odom: speed_fb={speed_topic}, imu_fb={imu_topic} '
            f'(yaw_axis={self._imu_yaw_axis}, yaw_source={self._yaw_source}), '
            f'speed_scale={self._speed_scale:.3f}, '
            f'yaw_filter_tau={self._yaw_tau:.2f}s, out={odom_topic} @ {hz:.0f}Hz (no TF)'
        )

    def _on_speed(self, msg: Float64) -> None:
        if math.isfinite(msg.data):
            self._v = float(msg.data) * self._speed_scale
            self._have_speed = True

    def _on_imu(self, msg: Imu) -> None:
        w = msg.angular_velocity
        omega = w.z if self._imu_yaw_axis == 'z' else w.y
        if not math.isfinite(omega):
            return
        self._omega_imu = omega
        self._have_imu = True

        if self._yaw_source == 'orientation':
            # EBIMU 자체 지자기 융합 yaw를 그대로 사용. 순간 튐(자기 간섭)이 있어도
            # _on_timer의 저역필터(yaw_filter_tau_sec)가 완충해줌.
            self._yaw_raw = _yaw_from_quat(msg.orientation)
            return

        # 'gyro'/'fused' 공통: yaw는 50Hz publish 타이머가 아니라 IMU 메시지
        # 도착마다(실측 ~100Hz) 바로 적분. (ebimu_driver의 burst 보정 timestamp 사용)
        stamp = Time.from_msg(msg.header.stamp)
        if self._last_imu_stamp is not None:
            dt = (stamp - self._last_imu_stamp).nanoseconds * 1e-9
            if 0.0 < dt <= 0.5:
                omega_used = omega if abs(self._v) >= self._min_speed_yaw else 0.0
                self._yaw_raw += omega_used * dt

                if self._yaw_source == 'fused' and self._yaw_fusion_tau > 1e-6:
                    # 장기 드리프트만 아주 느리게 지자기 융합 yaw 쪽으로 당김.
                    # 순간 자기 간섭은 짧게 끝나서 이 느린 보정엔 거의 안 묻어남.
                    target = _yaw_from_quat(msg.orientation)
                    err = math.atan2(
                        math.sin(target - self._yaw_raw), math.cos(target - self._yaw_raw)
                    )
                    self._yaw_raw += (dt / self._yaw_fusion_tau) * err
        self._last_imu_stamp = stamp

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        v = self._v
        # self._yaw_raw는 _on_imu()에서 IMU 메시지 도착마다 이미 적분됨 (100Hz).
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
