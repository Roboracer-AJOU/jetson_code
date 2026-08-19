#!/usr/bin/env python3
"""위치추정이 '튀는' 순간을 잡아낸다.

map->odom 은 카토그래퍼가 내는 '보정량'이다. 로컬 매칭이 잘 돌면 이 값은
천천히 흐르고, 포즈그래프가 최적화하거나 서브맵이 트리밍될 때만 계단처럼
점프한다. 그래서 여기서 계단이 언제/얼마나 생기는지만 보면
튐의 원인이 로컬 매칭인지 포즈그래프인지 갈린다.

주행 중 실행하고 Ctrl-C 로 끝내면 요약이 나온다.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
# 이 값을 넘는 한 샘플 변화를 '점프'로 본다. 100Hz 로 보므로
# 정상 흐름이면 샘플당 수 mm 를 넘지 않는다.
JUMP_M = 0.02
JUMP_DEG = 0.5
# 주행으로 인정할 최소 속도. 정지/수동배치 구간을 통계에서 뺀다.
DRIVING_MPS = 0.5
# 이보다 큰 점프는 rviz 2D pose estimate 나 차를 들어 옮긴 것으로 본다.
TELEPORT_M = 0.5


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class JumpLogger(Node):
    def __init__(self):
        super().__init__('pose_jump_logger')
        # tf2 Buffer + 100Hz 타이머 조회는 젯슨에서 CPU 를 눈에 띄게 먹는다.
        # /tf 를 직접 받아 map->odom 만 걸러 쓰면 이벤트 구동이라 거의 공짜다.
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(Odometry, '/odom', self.on_odom, BEST_EFFORT)
        self.speed = 0.0
        self.prev = None
        self.jumps = []
        self.samples = 0
        self.driving_samples = 0
        self.t0 = time.time()

    def on_odom(self, m):
        self.speed = m.twist.twist.linear.x

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                self._on_map_odom(tf)

    def _on_map_odom(self, tf):
        t = tf.transform.translation
        cur = (t.x, t.y, yaw_of(tf.transform.rotation))
        self.samples += 1
        if abs(self.speed) >= DRIVING_MPS:
            self.driving_samples += 1
        if self.prev is not None:
            dx, dy = cur[0] - self.prev[0], cur[1] - self.prev[1]
            d = math.hypot(dx, dy)
            dth = abs(math.degrees(math.atan2(math.sin(cur[2] - self.prev[2]),
                                              math.cos(cur[2] - self.prev[2]))))
            if d > JUMP_M or dth > JUMP_DEG:
                self.jumps.append((time.time() - self.t0, d, dth, self.speed))
        self.prev = cur


def report(node):
    dur = time.time() - node.t0
    if node.samples < 100:
        print('\nTF 를 거의 못 받았다. 로컬라이제이션이 떠 있는지 확인해라.')
        return

    path = f'/tmp/pose_jump_{int(node.t0)}.csv'
    with open(path, 'w') as f:
        f.write('t_sec,jump_m,jump_deg,speed_mps\n')
        for t, d, dth, v in node.jumps:
            f.write(f'{t:.3f},{d:.4f},{dth:.3f},{v:.3f}\n')

    tele = [x for x in node.jumps if x[1] > TELEPORT_M]
    drive = [x for x in node.jumps if x[1] <= TELEPORT_M and x[3] >= DRIVING_MPS]
    # /tf 발행 주기는 pose_publish_period_sec 에 따라 달라지므로 실측으로 환산한다.
    rate = node.samples / dur if dur > 0 else 0
    drive_t = node.driving_samples / rate if rate > 0 else 0

    print(f'\n{dur:.0f}초 기록, 그중 주행({DRIVING_MPS}m/s 이상) {drive_t:.0f}초')
    print(f'전체 로그: {path}')
    if tele:
        print(f'제외됨: {TELEPORT_M*100:.0f}cm 초과 점프 {len(tele)}회 '
              f'(2D pose estimate / 수동배치로 간주)')
    print()

    if drive_t < 5:
        print('주행 구간이 5초도 안 된다. 실제로 달리면서 다시 재라.')
        return
    if not drive:
        print('주행 중 점프가 없다. map->odom 은 연속적이다.')
        print('=> 라인 이탈 원인은 위치추정이 아니라 제어(경로추종) 쪽이다.')
        return

    print('--- 주행 구간만 ---')
    print(f'점프 {len(drive)}회, {len(drive)/drive_t:.2f}회/초')
    print()
    print(f'{"시각":>8s} {"이동":>9s} {"회전":>8s} {"속도":>9s}')
    print('-' * 40)
    for t, d, dth, v in sorted(drive, key=lambda x: -x[1])[:10]:
        print(f'{t:>7.1f}s {d*100:>8.1f}cm {dth:>7.2f}° {v:>7.2f}m/s')

    ds = sorted(x[1] for x in drive)
    p90 = ds[min(int(len(ds) * 0.9), len(ds) - 1)]
    print(f'\n점프 크기: 중앙 {ds[len(ds)//2]*100:.1f}cm  '
          f'p90 {p90*100:.1f}cm  최대 {ds[-1]*100:.1f}cm')

    big = [x for x in drive if x[1] > 0.05]
    print(f'5cm 초과 보정: {len(big)}회 ({len(big)/drive_t:.2f}회/초)')
    print()
    if p90 < 0.03:
        print('=> 보정이 3cm 미만으로 잘게 들어온다. 위치추정은 충분히 안정적이다.')
    elif p90 < 0.07:
        print('=> 보정이 3~7cm 다. 라인 이탈이 남으면 제어 쪽을 봐야 한다.')
    else:
        print(f'=> 보정이 한 번에 {p90*100:.0f}cm 씩 들어온다. 그 사이 쌓인 드리프트를')
        print('   뒤늦게 갚는 것이므로, 제약이 더 자주 들어오게 해야 한다')
        print('   (constraint_builder.sampling_ratio / min_score).')


def main():
    rclpy.init()
    node = JumpLogger()
    print(f'기록 중... (점프 기준 {JUMP_M*100:.0f}cm 또는 {JUMP_DEG}°) 끝내려면 Ctrl-C')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        report(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
