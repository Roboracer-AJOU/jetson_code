#!/usr/bin/env python3
"""Wheel odometry from VESC measured speed + IMU gyro yaw rate.

Speed feedback: /vehicle/speed_mps (Float64, m/s from VESC ERPM via control_node)
Yaw rate feedback: /imu/data (sensor_msgs/Imu, angular_velocity.<imu_yaw_axis>)
       조향각 기반 yaw 추정(서보각→bicycle model)은 오차가 커서 제거함 — IMU 실측으로 대체.

Publishes /odom; optional odom->base_link TF when publish_tf=true (AMCL stack).
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
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


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
        # gz가 yaw. /imu/data 는 ROS 축이라 기본 'z'.
        self.declare_parameter('imu_yaw_axis', 'z')
        # /imu/data 는 ROS 축(+Z 위). 왼쪽 회전이 +yaw → 기본 +1.
        self.declare_parameter('imu_yaw_sign', 1.0)
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
        # 1차 지연이라 정상선회 중 헤딩 지연 = tau * omega. 0.1s면 맵핑 속도
        # (omega ~2.2rad/s)에서도 12도, 3m/s 코너면 20도 넘게 밀려서 그 밀린 헤딩으로
        # 병진이 적분된다. yaw_source='gyro'의 _yaw_raw는 이미 깨끗한 적분값이라
        # 누를 이유가 없음 -> 0(=필터 끔)이 기본. 서보각 기반 yaw를 쓸 때만 올릴 것.
        self.declare_parameter('yaw_filter_tau_sec', 0.0)
        # 종방향 스케일. 한 바퀴 돌수록 점점 밀리면 0.95~1.05로 조절.
        self.declare_parameter('speed_scale', 1.0)
        # 발행 주기일 뿐 적분 주기가 아니다. 적분은 _on_imu에서 IMU rate(~100Hz)로
        # 돌고, VESC 속도 피드백 상한도 50Hz라 발행은 50Hz면 충분하다.
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('publish_tf', False)
        # AMCL: scan stamp 에 맞는 odom->base_link TF 를 IMU rate(~100Hz)로 내보냄.
        # 50Hz 타이머만 쓰면 회전 중 scan 투영이 틀어져 사방으로 밀린다.
        self.declare_parameter('publish_on_imu', False)

        self._imu_yaw_axis = self.get_parameter(
            'imu_yaw_axis'
        ).get_parameter_value().string_value
        sign = float(self.get_parameter('imu_yaw_sign').value)
        self._imu_yaw_sign = -1.0 if sign < 0.0 else 1.0
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
        self._omega_filtered = 0.0
        # 시간 기준은 wall clock이 아니라 IMU stamp 하나뿐이다.
        self._last_imu_stamp: Time | None = None
        self._last_pub_stamp_ns: int | None = None

        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._publish_on_imu = bool(self.get_parameter('publish_on_imu').value)

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(Float64, speed_topic, self._on_speed, 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_timer(1.0 / hz, self._on_timer)

        tf_note = ' + odom->base_link TF' if self._publish_tf else ' (no TF)'
        imu_pub_note = ' imu-rate publish' if self._publish_on_imu else ''
        self.get_logger().info(
            f'VESC wheel odom: speed_fb={speed_topic}, imu_fb={imu_topic} '
            f'(yaw_axis={self._imu_yaw_axis}, yaw_sign={self._imu_yaw_sign:.0f}, '
            f'yaw_source={self._yaw_source}), '
            f'speed_scale={self._speed_scale:.3f}, '
            f'yaw_filter_tau={self._yaw_tau:.2f}s, out={odom_topic} @ {hz:.0f}Hz{tf_note}{imu_pub_note}'
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
        omega *= self._imu_yaw_sign
        self._omega_imu = omega
        self._have_imu = True

        # IMU stamp가 이 노드의 유일한 시간축이다. yaw뿐 아니라 x/y까지 전부 여기서
        # IMU dt로 적분하고, publish 타이머는 그 상태를 그대로 내보내기만 한다.
        # (예전엔 yaw는 IMU stamp 기준, x/y는 타이머의 wall now 기준이라 stamp가
        #  가리키는 시각과 내용물의 시각이 서로 달랐다.)
        # ebimu_driver가 burst를 역산해 매긴 단조증가 stamp를 그대로 신뢰한다.
        stamp = Time.from_msg(msg.header.stamp)
        prev = self._last_imu_stamp
        self._last_imu_stamp = stamp

        if prev is None:
            # 첫 샘플: dt를 모르니 적분 없이 기준만 잡는다.
            if self._yaw_source == 'orientation':
                self._yaw_raw = _yaw_from_quat(msg.orientation)
                self._yaw = self._yaw_raw
            return

        dt = (stamp - prev).nanoseconds * 1e-9
        if not (0.0 < dt <= 0.5):
            return

        if self._yaw_source == 'orientation':
            # EBIMU 자체 지자기 융합 yaw를 그대로 사용. 순간 튐(자기 간섭)이 있어도
            # 아래 저역필터(yaw_filter_tau_sec)가 완충해줌.
            self._yaw_raw = _yaw_from_quat(msg.orientation)
        else:
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

        self._integrate(dt)
        if self._publish_on_imu:
            self._publish_state(allow_same_stamp=False)

    def _integrate(self, dt: float) -> None:
        """IMU 한 스텝만큼 헤딩 필터와 위치를 전진시킨다. dt는 IMU stamp 차이."""
        # Low-pass the yaw actually used: sustained turns still get tracked, but a
        # sudden steering-yaw error only leaks in gradually. tau=0이면 그대로 통과.
        if self._yaw_tau > 1e-6:
            alpha = dt / (self._yaw_tau + dt)
        else:
            alpha = 1.0
        dyaw = math.atan2(
            math.sin(self._yaw_raw - self._yaw), math.cos(self._yaw_raw - self._yaw)
        )
        step = alpha * dyaw
        self._omega_filtered = step / dt

        # 스텝 끝 yaw로 적분하면 코너에서 스텝당 dyaw/2 만큼 헤딩이 앞선 상태로
        # 병진이 쌓인다. 스텝 중앙 yaw를 쓰면 그 1차항이 사라진다.
        yaw_mid = self._yaw + 0.5 * step
        self._yaw += step

        v = self._v
        self._x += v * math.cos(yaw_mid) * dt
        self._y += v * math.sin(yaw_mid) * dt

    def _publish_state(self, *, allow_same_stamp: bool) -> None:
        if self._last_imu_stamp is None:
            return
        stamp_ns = self._last_imu_stamp.nanoseconds
        if not allow_same_stamp:
            if self._last_pub_stamp_ns is not None and stamp_ns <= self._last_pub_stamp_ns:
                return
        self._last_pub_stamp_ns = stamp_ns

        msg = Odometry()
        msg.header.stamp = self._last_imu_stamp.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation = _yaw_to_quat(self._yaw)
        msg.twist.twist.linear.x = self._v
        msg.twist.twist.angular.z = self._omega_filtered

        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.1
        msg.twist.covariance[0] = 0.1
        msg.twist.covariance[35] = 0.2

        self._odom_pub.publish(msg)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    def _on_timer(self) -> None:
        if self._publish_on_imu:
            return
        self._publish_state(allow_same_stamp=False)


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
