#!/usr/bin/env python3
"""/scan 의 시간 필드가 실제와 맞는지 검증.

핵심: scan_time(=드라이버의 scan_duration)이 실제 회전 주기와 같아야 한다.
훨씬 짧으면 grabScanDataHq가 버퍼에서 즉시 리턴한다는 뜻이고,
그러면 Cartographer의 회전 중 de-skew가 덜 먹는다.
"""
import statistics as st
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class T(Node):
    def __init__(self):
        super().__init__('scan_timing')
        self.rows = []
        self.prev_stamp = None
        self.prev_rx = None
        self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.get_logger().info('/scan 수집 중... 10초 뒤 자동 종료')

    def cb(self, m):
        rx = self.get_clock().now().nanoseconds * 1e-9
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        n = len(m.ranges)
        span = m.time_increment * (n - 1)
        d_stamp = (stamp - self.prev_stamp) if self.prev_stamp else 0.0
        d_rx = (rx - self.prev_rx) if self.prev_rx else 0.0
        self.prev_stamp, self.prev_rx = stamp, rx
        self.rows.append(dict(n=n, scan_time=m.scan_time, span=span,
                              lat=rx - stamp, d_stamp=d_stamp, d_rx=d_rx))


def main():
    rclpy.init()
    node = T()
    t0 = node.get_clock().now().nanoseconds * 1e-9
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    r = node.rows[2:]
    if len(r) < 5:
        print('샘플 부족 — /scan 나오는지 확인')
        rclpy.shutdown(); return

    def s(k):
        v = [x[k] for x in r]
        return f'{st.mean(v):8.4f} ± {st.pstdev(v):.4f}   (min {min(v):.4f} / max {max(v):.4f})'

    period = st.mean([x['d_rx'] for x in r])
    print(f'\n샘플 {len(r)}개,  점 개수 {r[0]["n"]}')
    print(f'실제 수신 주기      : {s("d_rx")}   <- 이게 진짜 회전 주기')
    print(f'stamp 간격          : {s("d_stamp")}')
    print(f'scan_time 필드      : {s("scan_time")}   <- 위 주기와 같아야 정상')
    print(f'time_increment×(n-1): {s("span")}')
    print(f'수신지연(rx - stamp): {s("lat")}')

    m_scan = st.mean([x['scan_time'] for x in r])
    print('\n--- 판정 ---')
    if period > 0 and m_scan < period * 0.6:
        print(f'!! scan_time({m_scan:.4f}s)이 실제 주기({period:.4f}s)보다 크게 짧음.')
        print('   grabScanDataHq가 버퍼에서 즉시 리턴 -> 회전 중 de-skew 부족.')
        print('   => scan_duration을 실측 주기로 대체해야 함.')
    elif period > 0 and m_scan > period * 1.4:
        print(f'!! scan_time({m_scan:.4f}s)이 실제 주기({period:.4f}s)보다 김 -> de-skew 과보정.')
    else:
        print(f'scan_time({m_scan:.4f}s) ~ 실제 주기({period:.4f}s). 시간 필드는 정상.')
        print('   회전 문제라면 de-skew가 아니라 IMU/TF 쪽을 봐야 함.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
