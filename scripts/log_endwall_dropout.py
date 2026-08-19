#!/usr/bin/env python3
"""급가속 중 끝벽 리턴이 사라지는지 기록한다.

가설: 가속하면 차체가 뒤로 눌리며 라이다가 위를 향하고, 20m 앞 끝벽 빔이
벽 위로 지나가 리턴이 통째로 없어진다. 20m 에서 1deg = 35cm 이다.

주행 중 실행하고 Ctrl-C 로 끝내면 상관관계 요약이 나온다.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
FAR_M = 8.0


def pitch_deg(q):
    """쿼터니언에서 pitch(deg). 라이다가 위를 향하면 부호가 일정하게 나온다."""
    s = 2.0 * (q.w * q.y - q.z * q.x)
    s = max(-1.0, min(1.0, s))
    return math.degrees(math.asin(s))


class Logger(Node):
    def __init__(self):
        super().__init__('endwall_logger')
        self.rows = []
        self.speed = 0.0
        self.prev_speed = 0.0
        self.prev_t = None
        self.accel = 0.0
        self.pitch = 0.0
        self.pitch0 = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, BEST_EFFORT)
        self.create_subscription(Imu, '/imu/data', self.on_imu, BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self.on_odom, BEST_EFFORT)
        self.t0 = time.time()

    def on_imu(self, m):
        p = pitch_deg(m.orientation)
        if self.pitch0 is None:
            self.pitch0 = p
        self.pitch = p - self.pitch0

    def on_odom(self, m):
        v = m.twist.twist.linear.x
        now = time.time()
        if self.prev_t is not None and now > self.prev_t:
            a = (v - self.prev_speed) / (now - self.prev_t)
            self.accel = 0.7 * self.accel + 0.3 * a
        self.prev_speed, self.prev_t = v, now
        self.speed = v

    def on_scan(self, m):
        r = [v for v in m.ranges if math.isfinite(v) and m.range_min < v < m.range_max]
        if not r:
            return
        self.rows.append((time.time() - self.t0, self.speed, self.accel,
                          self.pitch, sum(1 for v in r if v > FAR_M), max(r), len(r)))


def report(rows):
    if len(rows) < 20:
        print('샘플이 부족하다. 주행하면서 다시 실행해라.')
        return
    print(f'\n총 {len(rows)}스캔, {rows[-1][0]:.0f}초\n')
    print(f'{"구간":22s} {"스캔":>6s} {"평균속도":>9s} {"평균pitch":>10s} '
          f'{">8m 점":>8s} {"최대거리":>9s}')
    print('-' * 72)

    def line(name, sel):
        if not sel:
            print(f'{name:22s} {"(없음)":>6s}')
            return
        n = len(sel)
        print(f'{name:22s} {n:>6d} {sum(s[1] for s in sel)/n:>8.2f}m/s '
              f'{sum(s[3] for s in sel)/n:>9.2f}° {sum(s[4] for s in sel)/n:>8.1f} '
              f'{sum(s[5] for s in sel)/n:>8.1f}m')

    line('정지/저속 <1m/s', [s for s in rows if s[1] < 1.0])
    line('정속 (|a|<1.5)', [s for s in rows if s[1] >= 1.0 and abs(s[2]) < 1.5])
    line('급가속 (a>2.5)', [s for s in rows if s[2] > 2.5])
    line('급감속 (a<-2.5)', [s for s in rows if s[2] < -2.5])

    cruise = [s for s in rows if s[1] >= 1.0 and abs(s[2]) < 1.5]
    hard = [s for s in rows if s[2] > 2.5]
    print()
    if cruise and hard:
        fc = sum(s[4] for s in cruise) / len(cruise)
        fh = sum(s[4] for s in hard) / len(hard)
        dp = sum(s[3] for s in hard)/len(hard) - sum(s[3] for s in cruise)/len(cruise)
        print(f'급가속 시 >8m 점 변화 : {fc:.1f} -> {fh:.1f}개 ({100*(fh-fc)/max(fc,1e-9):+.0f}%)')
        print(f'급가속 시 pitch 변화  : {dp:+.2f}°  (20m 에서 {abs(dp)*349:.0f}cm 빔 이동)')
        print()
        if fc > 3 and fh < fc * 0.6:
            print('=> 끝벽 리턴이 급가속 중 실제로 사라진다. 가설이 맞다.')
            print('   해결은 파라미터가 아니라 라이다 마운트 강성/피치 보정이다.')
        elif fc <= 3:
            print('=> 정속에서도 8m 초과 점이 거의 없다. 끝벽이 아예 라이다 사거리 밖이거나')
            print('   반사가 안 돌아온다. 이 코스에선 종방향을 스캔매칭으로 잡을 수 없다.')
        else:
            print('=> 끝벽 리턴은 급가속 중에도 유지된다. 원인은 다른 데 있다.')
            print('   (지연 스파이크 또는 오돔 스케일 쪽을 봐야 한다)')
    else:
        print('급가속 구간이 안 잡혔다. 더 세게 가속하면서 다시 재라.')


def main():
    rclpy.init()
    node = Logger()
    print('기록 중... 급가속 직진을 몇 번 해라. 끝내려면 Ctrl-C')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        report(node.rows)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
