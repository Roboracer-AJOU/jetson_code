#!/usr/bin/env python3
"""Cartographer 헤딩이 자이로 적분 대비 어디서 새는지 잰다.

gyro_scale_check 로 자이로가 정확하다는 게 확인된 뒤에 쓴다(바이어스~0, 스케일~1).
그러면 자이로 적분을 헤딩 기준자로 쓸 수 있다.

핵심 가설: 코너에서 걸리는 횡가속도가 ImuTracker 의 '중력' 방향을 기울여서
(imu_gravity_time_constant 로 방어) 그 평면에서 뽑은 yaw 가 새는 것.
그래서 발산량을 '횡가속도 구간별'로 쪼개서 보여준다.

맵핑(또는 localization) 돌려놓고 같이 실행 -> 몇 바퀴 주행 -> Ctrl+C.
CSV: /tmp/yaw_drift.csv
"""
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
import tf2_ros


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class YawDrift(Node):
    def __init__(self):
        super().__init__('yaw_drift_check')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.gyro_yaw = 0.0          # 자이로 적분 (기준자)
        self.last_stamp = None
        self.lat_acc = 0.0           # 횡가속도 |ay|
        self.wz = 0.0
        self.carto0 = None           # 시작 시점 carto yaw (오프셋 제거용)
        self.gyro0 = None
        self.t0 = None
        self.rows = []
        self.miss = 0
        self.create_subscription(Imu, '/imu/data', self._on_imu, 50)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('주행 시작 -> 몇 바퀴 돈 뒤 Ctrl+C')

    def _on_imu(self, m):
        wz = m.angular_velocity.z
        if not math.isfinite(wz):
            return
        self.wz = wz
        ay = m.linear_acceleration.y
        self.lat_acc = abs(ay) if math.isfinite(ay) else 0.0
        stamp = Time.from_msg(m.header.stamp)
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds * 1e-9
            if 0.0 < dt <= 0.5:
                self.gyro_yaw += wz * dt
        self.last_stamp = stamp

    def _tick(self):
        if self.last_stamp is None:
            return
        # 자이로 적분은 last_stamp 시각까지의 값이다. TF도 "최신"이 아니라
        # '같은 시각'으로 조회해야 한다. Time()으로 최신을 받아 비교하면
        # 둘의 시각차가 그대로 각도차로 둔갑한다(70dps에서 50ms = 3.5deg).
        try:
            tr = self.buf.lookup_transform('map', 'base_link', self.last_stamp)
        except Exception:
            self.miss += 1
            return
        # 참고용: cartographer TF가 IMU 대비 얼마나 뒤처져 있는지
        try:
            latest = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            lag_ms = (self.last_stamp.nanoseconds
                      - Time.from_msg(latest.header.stamp).nanoseconds) * 1e-6
        except Exception:
            lag_ms = float('nan')

        q = tr.transform.rotation
        carto = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = self.last_stamp.nanoseconds * 1e-9
        if self.carto0 is None:
            self.carto0, self.gyro0, self.t0 = carto, self.gyro_yaw, now
            return
        d_carto = _wrap(carto - self.carto0)
        d_gyro = self.gyro_yaw - self.gyro0
        diff = _wrap(d_carto - _wrap(d_gyro))
        self.rows.append([round(now - self.t0, 3),
                          round(math.degrees(diff), 3),
                          round(math.degrees(self.wz), 2),
                          round(self.lat_acc, 3),
                          round(lag_ms, 1) if math.isfinite(lag_ms) else float('nan')])

    def report(self):
        if len(self.rows) < 20:
            print('데이터 부족 (map->base_link TF가 안 잡혔을 수 있음)')
            return
        with open('/tmp/yaw_drift.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'carto_minus_gyro_deg', 'yaw_rate_dps', 'lat_acc_mps2',
                        'tf_lag_ms'])
            w.writerows(self.rows)

        lags = [r[4] for r in self.rows if isinstance(r[4], float) and math.isfinite(r[4])]
        if lags:
            lags_sorted = sorted(lags)
            print(f'\ncartographer TF 지연 (IMU stamp 대비): 중앙값 '
                  f'{lags_sorted[len(lags) // 2]:.0f} ms, 최대 {max(lags):.0f} ms')
            print(f'  (TF 시각조회 실패 {self.miss}회 — 버퍼에 없던 시각)')
        dev = [abs(r[1]) for r in self.rows]
        print(f'\n총 {self.rows[-1][0]:.1f}s, 샘플 {len(self.rows)}개')
        print(f'carto - 자이로 헤딩 차이 : 평균 {sum(dev) / len(dev):.2f} deg, '
              f'최대 {max(dev):.2f} deg, 마지막 {self.rows[-1][1]:+.2f} deg')

        print('\n횡가속도 구간별 (여기서 커지면 = 중력 기울기 문제)')
        print('  |ay| m/s2     샘플   평균차이   최대차이')
        for lo, hi in [(0.0, 0.5), (0.5, 1.5), (1.5, 3.0), (3.0, 99.0)]:
            sel = [r for r in self.rows if lo <= r[3] < hi]
            if not sel:
                continue
            d = [abs(r[1]) for r in sel]
            print(f'  {lo:4.1f}~{hi:<5.1f} {len(sel):7d} {sum(d) / len(d):9.2f} {max(d):10.2f}')

        print('\n회전속도 구간별 (여기서만 커지면 = 스캔 왜곡/매칭 문제)')
        print('  |yaw rate| dps 샘플   평균차이   최대차이')
        for lo, hi in [(0.0, 20.0), (20.0, 60.0), (60.0, 150.0), (150.0, 999.0)]:
            sel = [r for r in self.rows if lo <= abs(r[2]) < hi]
            if not sel:
                continue
            d = [abs(r[1]) for r in sel]
            print(f'  {lo:5.0f}~{hi:<6.0f}{len(sel):7d} {sum(d) / len(d):9.2f} {max(d):10.2f}')
        print('\nCSV: /tmp/yaw_drift.csv')


def main():
    rclpy.init()
    n = YawDrift()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.report()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
