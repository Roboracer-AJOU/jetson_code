#!/usr/bin/env python3
"""자이로 바이어스와 스케일 팩터를 잰다. 주행 불필요 — 손으로 돌려도 된다.

바이어스: 정지 중 드리프트 (deg/s). 오래 서 있을수록 헤딩이 샌다.
스케일  : 회전량에 비례하는 오차. 2%면 90도 코너마다 1.8도씩 잃는다.
          맵이 회전 구간에서 틀어지는 전형적 원인.

1단계: 차를 가만히 둔 채 Enter -> 10초 측정 -> 바이어스
2단계: 차를 제자리에서 정확히 N바퀴 돌린 뒤 Enter -> 스케일
       (바닥에 앞바퀴 방향 표시를 해두고 정확히 원래 자리로 돌아올 것)
"""
import argparse
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class Gyro(Node):
    def __init__(self, axis):
        super().__init__('gyro_scale_check')
        self.axis = axis
        self.lock = threading.Lock()
        self.samples = []          # (t_sec, wz)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 50)

    def _on_imu(self, m):
        w = m.angular_velocity
        wz = w.z if self.axis == 'z' else (w.y if self.axis == 'y' else w.x)
        if not math.isfinite(wz):
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.samples.append((t, wz))

    def mark(self):
        with self.lock:
            return len(self.samples)

    def slice(self, i0, i1):
        with self.lock:
            return list(self.samples[i0:i1])


def integrate(rows, bias=0.0):
    """사다리꼴 적분. (총 회전 rad, 지속시간 s)"""
    if len(rows) < 2:
        return 0.0, 0.0
    total = 0.0
    for (t0, w0), (t1, w1) in zip(rows, rows[1:]):
        dt = t1 - t0
        if not (0.0 < dt <= 0.5):
            continue
        total += ((w0 - bias) + (w1 - bias)) * 0.5 * dt
    return total, rows[-1][0] - rows[0][0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turns', type=float, default=1.0,
                    help='2단계에서 돌릴 바퀴 수 (기본 1.0 = 360도)')
    ap.add_argument('--axis', default='z', choices=['x', 'y', 'z'])
    ap.add_argument('--bias-sec', type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Gyro(args.axis)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    # 데몬 스레드가 spin 중인 채로 인터프리터가 죽으면 rclpy가 core dump를 낸다.
    # 무슨 경로로 빠져나가든 shutdown -> join 순서를 보장한다.
    import atexit
    def _cleanup():
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=2.0)
    atexit.register(_cleanup)

    print(f'\n/imu/data 의 angular_velocity.{args.axis} 를 씁니다.')
    input(f'\n[1단계] 차를 완전히 가만히 두고 Enter (약 {args.bias_sec:.0f}초 측정) > ')
    i0 = node.mark()
    import time as _t
    _t.sleep(args.bias_sec)
    i1 = node.mark()
    rows = node.slice(i0, i1)
    if len(rows) < 20:
        print(f'IMU 샘플이 너무 적습니다 ({len(rows)}개). /imu/data 가 나오는지 확인하세요.')
        return
    drift, dur = integrate(rows)
    bias = drift / dur if dur > 0 else 0.0
    hz = len(rows) / dur if dur > 0 else 0.0
    print(f'  샘플 {len(rows)}개 / {dur:.1f}s  (~{hz:.0f} Hz)')
    print(f'  바이어스 = {math.degrees(bias):+.4f} deg/s'
          f'   -> 방치 1분당 {math.degrees(bias) * 60:+.1f} deg 샘')

    truth = args.turns * 2.0 * math.pi
    print(f'\n[2단계] 차를 제자리에서 정확히 {args.turns:g}바퀴'
          f' ({math.degrees(truth):.0f} deg) 돌린 뒤 Enter')
    input('        (바닥에 방향 표시를 해두고 정확히 원위치로) > ')
    i2 = node.mark()
    input('        다 돌렸으면 Enter > ')
    i3 = node.mark()
    rows = node.slice(i2, i3)
    if len(rows) < 20:
        print('회전 구간 샘플이 너무 적습니다.')
        return
    raw, dur = integrate(rows)
    corr, _ = integrate(rows, bias)

    print(f'\n  회전 시간        : {dur:.1f}s')
    print(f'  실제 회전        : {math.degrees(truth):8.1f} deg')
    print(f'  자이로 적분(raw) : {math.degrees(raw):8.1f} deg')
    print(f'  바이어스 보정 후 : {math.degrees(corr):8.1f} deg')
    if abs(truth) < 1e-6:
        return
    scale = corr / truth
    print(f'\n  스케일 팩터 = 적분/실제 = {scale:.4f}')
    err90 = (scale - 1.0) * 90.0
    print(f'  -> 90deg 코너마다 {err90:+.2f} deg 오차, 4코너 랩당 {err90 * 4:+.1f} deg')
    print()
    if abs(scale - 1.0) < 0.005:
        print('[정상] 자이로 스케일 문제 아님. 회전 틀어짐은 다른 원인.')
    else:
        print(f'[스케일 오차 {abs(scale - 1.0) * 100:.1f}%] 회전량에 비례해 헤딩이 틀어집니다.')
        print(f'       ebimu_driver 에서 gz 에 {1.0 / scale:.4f} 를 곱하거나,')
        print('       라이다가 고칠 수 있게 ceres_scan_matcher.rotation_weight 를 낮추세요.')
    print()


if __name__ == '__main__':
    main()
