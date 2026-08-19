#!/usr/bin/env python3
"""직선 구간에서 odom과 Cartographer pose가 얼마나 어긋나는지 실측.

맵핑 돌리는 중에 같이 띄워두고 직선을 한 번 지나간 뒤 Ctrl+C.
- odom_dist   : /odom 적분 이동거리 (휠+IMU)
- carto_dist  : map->base_link TF 이동거리 (Cartographer 최종 추정)
- 둘의 비(ratio)가 직선 구간에서 1.0에서 얼마나 벗어나는지가 핵심.
  carto < odom  -> Cartographer가 덜 감 (스캔이 앞으로 땡겨지는 증상)
  carto > odom  -> odom이 덜 감 (휠 오돔 지연/미달)
CSV: /tmp/straight_diag.csv
"""
import math, csv, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import tf2_ros


class Diag(Node):
    def __init__(self):
        super().__init__('straight_diag')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.odom = None
        self.speed = 0.0
        self.prev_o = None
        self.prev_c = None
        self.odom_d = 0.0
        self.carto_d = 0.0
        self.t0 = None
        self.rows = []
        # 정면 벽까지의 라이다 거리 = 절대 기준자. 벽은 안 움직이므로
        # (시작거리 - 끝거리)가 실제 이동거리다. odom도 carto도 안 낀 값.
        self.front = float('nan')
        self.laser_yaw = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Float64, '/vehicle/speed_mps', self._on_spd, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            '정면에 끝벽이 보이는 상태로 직선 진입 전 시작 -> 직선 통과 후 Ctrl+C')

    def _on_spd(self, m): self.speed = m.data

    def _get_laser_yaw(self):
        """base_link->laser의 yaw. 스캔 어느 각도가 차 정면인지 하드코딩하지 않는다."""
        if self.laser_yaw is not None:
            return self.laser_yaw
        try:
            tr = self.buf.lookup_transform('base_link', 'laser', rclpy.time.Time())
        except Exception:
            return None
        q = tr.transform.rotation
        self.laser_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(
            f'base_link->laser yaw = {math.degrees(self.laser_yaw):.1f}deg '
            f'-> 정면은 스캔 {math.degrees(-self.laser_yaw):.1f}deg')
        return self.laser_yaw

    def _on_scan(self, m):
        lyaw = self._get_laser_yaw()
        if lyaw is None:
            return
        # 차 정면(base_link +x)에 해당하는 스캔 각도
        target = -lyaw
        half = math.radians(2.5)
        vals = []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < m.range_min or r > m.range_max:
                continue
            a = m.angle_min + i * m.angle_increment
            d = math.atan2(math.sin(a - target), math.cos(a - target))
            if abs(d) <= half:
                vals.append(r)
        if len(vals) >= 3:
            vals.sort()
            self.front = vals[len(vals) // 2]   # 중앙값 (노이즈 제거)
        else:
            self.front = float('nan')

    def _on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _tick(self):
        if self.odom is None:
            return
        try:
            tr = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        c = (tr.transform.translation.x, tr.transform.translation.y)
        q = tr.transform.rotation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        if self.prev_o is not None:
            do = math.dist(self.odom, self.prev_o)
            dc = math.dist(c, self.prev_c)
            self.odom_d += do
            self.carto_d += dc
            self.rows.append([round(now - self.t0, 3), round(self.speed, 3),
                              round(self.odom_d, 4), round(self.carto_d, 4),
                              round(do, 5), round(dc, 5),
                              round(self.front, 4) if math.isfinite(self.front)
                              else float('nan'),
                              round(yaw, 2)])
        self.prev_o, self.prev_c = self.odom, c

    def report(self):
        if not self.rows:
            print('데이터 없음 (map->base_link TF가 안 잡혔을 수 있음)')
            return
        with open('/tmp/straight_diag.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'speed_mps', 'odom_dist', 'carto_dist', 'd_odom',
                        'd_carto', 'front_range', 'yaw_deg'])
            w.writerows(self.rows)
        print(f'\n총 odom  이동거리 : {self.odom_d:.2f} m')
        print(f'총 carto 이동거리 : {self.carto_d:.2f} m')
        if self.odom_d > 0.1:
            print(f'carto/odom 비     : {self.carto_d/self.odom_d:.3f}  (1.0이면 일치)')
        # 속도 구간별로 쪼개서 가속 구간이 특히 나쁜지 확인
        print('\n속도구간   샘플   odom(m)  carto(m)  비율')
        bins = [(0.0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 99.0)]
        for lo, hi in bins:
            sel = [r for r in self.rows if lo <= r[1] < hi]
            if not sel:
                continue
            o = sum(r[4] for r in sel); c = sum(r[5] for r in sel)
            ratio = f'{c/o:.3f}' if o > 1e-3 else '-'
            print(f'{lo:4.1f}~{hi:<4.1f} {len(sel):6d} {o:8.2f} {c:9.2f}  {ratio}')
        self._report_lidar_truth()
        print('\nCSV: /tmp/straight_diag.csv')

    def _report_lidar_truth(self):
        """정면 벽이 단조롭게 가까워지는 '직선 구간'만 골라 실제 이동거리를 구한다.

        한 번 실행에 직선/코너/다음 랩이 섞여 있으므로 전체 첫~끝 비교는 무의미하다.
        (코너를 돌면 정면에 잡히는 물체 자체가 바뀐다.)
        """
        print('\n--- 직선 구간 자동 검출 (라이다 정면벽 = 절대 기준자) ---')
        seg = self._find_straight_segment()
        if seg is None:
            print('쓸 만한 직선 구간을 못 찾음.')
            print('  조건: 정면 벽이 3m 이상 계속 가까워지고, 그 동안 헤딩 변화 8deg 이내.')
            print('  끝벽이 보이는 직선을 한 번 쭉 통과하는 구간이 포함돼야 합니다.')
            return
        a, b = seg
        lidar_d = a[6] - b[6]
        odom_d = b[2] - a[2]
        carto_d = b[3] - a[3]
        dyaw = abs(math.atan2(math.sin(math.radians(b[7] - a[7])),
                              math.cos(math.radians(b[7] - a[7]))))
        dur = b[0] - a[0]
        print(f'구간            : t={a[0]:.1f}s ~ {b[0]:.1f}s  ({dur:.1f}s, '
              f'평균 {odom_d / dur if dur > 0 else 0:.2f} m/s)')
        print(f'정면벽 거리     : {a[6]:.2f} m -> {b[6]:.2f} m')
        print(f'헤딩 변화       : {math.degrees(dyaw):.1f} deg')
        print(f'실제 이동(라이다): {lidar_d:.2f} m')
        print(f'odom  이동      : {odom_d:6.2f} m   odom/실제  = {odom_d / lidar_d:.3f}')
        print(f'carto 이동      : {carto_d:6.2f} m   carto/실제 = {carto_d / lidar_d:.3f}')
        print()
        print('  odom/실제 > 1  -> 휠 오돔이 과다 (wheel_diameter를 그 비율로 나눌 것)')
        print('  carto/실제 < 1 -> Cartographer가 직선에서 덜 감 (스캔이 당겨져 보이는 원인)')
        self._report_stop_recovery(seg)

    def _report_stop_recovery(self, seg):
        """직선 뒤 정지 구간에서 carto가 '따라잡는지' 본다.

        차가 멈추면 벽까지 거리는 고정된다. 그런데도 carto_dist가 계속 늘면
        Cartographer pose가 뒤처져 있었다는 직접 증거다(= 지연, 속도 비례 오차).
        스케일 오차라면 멈추는 순간 같이 멈추고 부족분이 그대로 남는다.
        """
        b = seg[1]
        after = [r for r in self.rows if r[0] > b[0]]
        stop = []
        for r in after:
            if r[1] < 0.15:
                stop.append(r)
            elif stop:
                break
        print('\n--- 정지 후 따라잡기 ---')
        if len(stop) < 2 or (stop[-1][0] - stop[0][0]) < 1.0:
            print('직선 뒤 정지 구간이 1초 미만이라 판정 불가.')
            print('  직선을 지난 뒤 차를 세우고 2~3초 기다렸다가 Ctrl+C 하세요.')
            return
        s0, s1 = stop[0], stop[-1]
        dur = s1[0] - s0[0]
        d_front = s0[6] - s1[6] if (math.isfinite(s0[6]) and math.isfinite(s1[6])) else float('nan')
        d_odom = s1[2] - s0[2]
        d_carto = s1[3] - s0[3]
        print(f'정지 구간       : t={s0[0]:.1f}s ~ {s1[0]:.1f}s ({dur:.1f}s)')
        print(f'정면벽 거리 변화 : {d_front:+.3f} m   (멈췄으니 0이어야 정상)')
        print(f'odom  추가 이동  : {d_odom:+.3f} m')
        print(f'carto 추가 이동  : {d_carto:+.3f} m')
        print()
        if d_carto > 0.15 and d_carto > d_odom + 0.1:
            print(f'[지연] 멈춘 뒤에도 carto가 {d_carto:.2f} m 더 갔습니다.')
            print('       주행 중 pose가 그만큼 뒤처져 있었다는 뜻 = 스캔이 당겨져 보이는 원인.')
            print('       스케일이 아니라 타이밍/모션보정 문제입니다.')
        elif abs(d_carto) < 0.1:
            print('[스케일] 멈추자 carto도 같이 멈췄습니다. 따라잡기가 없습니다.')
            print('         부족분이 지연이 아니라 실제 스케일 차이입니다.')
        else:
            print('[애매] 따라잡기가 뚜렷하지 않습니다. 정지를 더 길게 잡고 다시 재보세요.')

    def _find_straight_segment(self):
        """정면거리가 계속 줄고 헤딩이 거의 안 변하는 가장 긴 구간 [start, end]."""
        best = None
        best_drop = 0.0
        start = None
        run_min = None
        NOISE = 0.25      # 정면거리 노이즈 허용 (m)
        MAX_DYAW = 8.0    # 직선 판정 헤딩 허용 (deg)
        MIN_DROP = 3.0    # 이보다 적게 접근하면 판정 불가 (m)
        for r in self.rows:
            f = r[6]
            if not math.isfinite(f):
                start = None
                continue
            if start is None:
                start, run_min = r, f
                continue
            dy = abs(math.degrees(math.atan2(
                math.sin(math.radians(r[7] - start[7])),
                math.cos(math.radians(r[7] - start[7])))))
            # 벽이 다시 멀어지거나(코너로 정면 물체가 바뀜) 헤딩이 꺾이면 구간을 끊는다
            if f > run_min + NOISE or dy > MAX_DYAW:
                start, run_min = r, f
                continue
            run_min = min(run_min, f)
            drop = start[6] - f
            if drop > best_drop:
                best_drop, best = drop, (start, r)
        return best if best_drop >= MIN_DROP else None


def main():
    rclpy.init()
    n = Diag()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.report()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
