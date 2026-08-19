#!/usr/bin/env python3
"""base_link->laser TF의 yaw가 실제 라이다 장착 방향과 맞는지 10초 안에 판정.

원리: 차체가 라이다 후방 약 95도를 가린다(sensor_static_tf.cpp 실측 기록).
그 '무효 구간'이 스캔의 어디에 있는지 보면 스캔 0도가 어느 쪽인지 알 수 있다.
TF가 맞다면 무효 구간의 중심이 base_link 기준 180도(=차 뒤)여야 한다.

사용: 차를 사방이 트인 곳에 세워두고 실행. 이동 불필요.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros


class Check(Node):
    def __init__(self):
        super().__init__('laser_dir_check')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.done = False
        self.get_logger().info('/scan 대기 중...')

    def _on_scan(self, m):
        if self.done:
            return
        try:
            tr = self.buf.lookup_transform('base_link', 'laser', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF base_link->laser 아직 없음: {e}')
            return
        self.done = True
        q = tr.transform.rotation
        laser_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        n = len(m.ranges)
        valid = [math.isfinite(r) and m.range_min <= r <= m.range_max
                 for r in m.ranges]
        # 원형 배열에서 가장 긴 연속 무효 구간 = 차체 가림 구간
        best_len, best_start = 0, 0
        cur_len, cur_start = 0, 0
        for i in range(2 * n):
            if not valid[i % n]:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len, best_start = cur_len, cur_start
            else:
                cur_len = 0
        best_len = min(best_len, n)

        def ang(i):
            return m.angle_min + (i % n) * m.angle_increment

        width_deg = best_len * math.degrees(m.angle_increment)
        center_scan = ang(best_start + best_len // 2)
        center_base = math.atan2(math.sin(center_scan + laser_yaw),
                                 math.cos(center_scan + laser_yaw))

        print('\n================ 라이다 방향 점검 ================')
        print(f'스캔 점 개수        : {n},  유효 {sum(valid)},  무효 {n - sum(valid)}')
        print(f'TF base_link->laser : yaw = {math.degrees(laser_yaw):+.1f} deg')
        print(f'가장 긴 무효 구간    : 폭 {width_deg:.0f} deg, '
              f'중심 = 스캔 {math.degrees(center_scan):+.1f} deg')
        print(f'  -> base_link 기준  : {math.degrees(center_base):+.1f} deg '
              f'(차 뒤 = ±180, 차 앞 = 0)')
        err = abs(abs(math.degrees(center_base)) - 180.0)
        print()
        if width_deg < 40:
            print('[판정 불가] 무효 구간이 너무 좁습니다. 차체 가림이 아닐 수 있어요.')
            print('            사방이 트인 곳에서 다시 돌려보세요.')
        elif err < 35:
            print(f'[정상] 무효 구간이 차 뒤({math.degrees(center_base):+.0f} deg)에 있습니다.')
            print('       lidar_yaw = 0.0 이 맞습니다. 스캔 0deg = 차 정면.')
        else:
            print(f'[!! 틀림 !!] 차체 가림 구간이 차 뒤가 아니라 '
                  f'{math.degrees(center_base):+.0f} deg 에 있습니다.')
            need = math.atan2(math.sin(laser_yaw + math.pi - center_base - laser_yaw),
                              math.cos(laser_yaw + math.pi - center_base - laser_yaw))
            fix = math.atan2(math.sin(laser_yaw + (math.pi - center_base)),
                             math.cos(laser_yaw + (math.pi - center_base)))
            print(f'       sensor_static_tf.cpp 의 lidar_yaw 를 '
                  f'{math.degrees(fix):+.1f} deg ({fix:+.3f} rad) 로 고쳐야 합니다.')
            print('       이게 틀리면 Cartographer도 스캔을 돌아간 채로 받습니다.')
        print('=================================================\n')
        rclpy.shutdown()


def main():
    rclpy.init()
    n = Check()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
