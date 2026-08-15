#!/usr/bin/env python3
"""물체를 차 주위로 한 바퀴 돌려서, 스캔 각도가 어떻게 움직이는지 기록한다.

단계 전환을 스크립트가 지시하지 않는다. "가까운 물체가 처음 보인 순간"이
기준점이고, 사용자는 그 순간을 자기가 정한다. 그래서 타이밍이 밀릴 수 없다.

사용법
  1. 박스나 손을 차 '정면' 30cm 에 댄다  <- 이때부터 기록이 시작된다
  2. 차에 가깝게 붙인 채로 천천히 반시계방향(= 차 왼쪽 -> 뒤 -> 오른쪽)으로
     차 주위를 한 바퀴 돈다
  3. 정면으로 돌아오면 끝
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


NEAR_M = 0.60        # 이보다 가까우면 "손에 든 물체"로 본다 (주변 벽은 0.96m+)
SAMPLE_DT = 0.4
TIMEOUT_S = 90.0
LOST_S = 6.0         # 물체가 이만큼 안 보이면 끝난 것으로 본다


class Bearing(Node):
    def __init__(self) -> None:
        super().__init__("tmp_scan_bearing")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.scan: LaserScan | None = None
        self.create_subscription(LaserScan, "/scan", self._cb, qos)

    def _cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def nearest(self) -> tuple[float, float] | None:
        """NEAR_M 안에서 가장 가까운 점의 (각도deg, 거리m)."""
        m = self.scan
        if m is None:
            return None
        r = np.asarray(m.ranges, dtype=float)
        ok = np.isfinite(r) & (r > m.range_min) & (r < NEAR_M)
        if not np.any(ok):
            return None
        idx = np.flatnonzero(ok)
        j = idx[int(np.argmin(r[idx]))]
        return math.degrees(m.angle_min + j * m.angle_increment), float(r[j])


def unwrap(prev: float, cur: float) -> float:
    """각도 점프를 이어붙여 누적 회전을 볼 수 있게 한다."""
    d = cur - prev
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return prev + d


def main() -> int:
    rclpy.init()
    n = Bearing()
    for _ in range(100):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.scan is not None:
            break
    if n.scan is None:
        print("/scan 안 들어옴", flush=True)
        return 1

    print("준비됐습니다.", flush=True)
    print("박스나 손을 차 '정면' 30cm 에 대세요 — 그 순간부터 기록합니다.", flush=True)
    print("그 다음 차에 붙인 채로 천천히 왼쪽 -> 뒤 -> 오른쪽 으로 한 바퀴.", flush=True)

    t0 = time.monotonic()
    samples: list[tuple[float, float, float]] = []
    acc = None
    last_seen = None

    while time.monotonic() - t0 < TIMEOUT_S:
        rclpy.spin_once(n, timeout_sec=0.05)
        hit = n.nearest()
        now = time.monotonic()
        if hit is None:
            if samples and last_seen is not None and now - last_seen > LOST_S:
                break
            continue
        if samples and now - samples[-1][0] < SAMPLE_DT:
            continue
        ang, dist = hit
        last_seen = now
        acc = ang if acc is None else unwrap(acc, ang)
        samples.append((now, ang, dist))
        el = now - t0
        print(f"  t={el:5.1f}s  스캔 {ang:+7.1f}deg  {dist:.2f}m  (누적 {acc:+7.1f})",
              flush=True)

    print("", flush=True)
    if len(samples) < 5:
        print("샘플 부족 — 물체를 30cm 안으로 더 가깝게 대주세요.", flush=True)
        n.destroy_node()
        rclpy.shutdown()
        return 1

    start = samples[0][1]
    accs = []
    a = samples[0][1]
    for _, ang, _ in samples[1:]:
        a = unwrap(a, ang)
        accs.append(a)
    total = accs[-1] - samples[0][1] if accs else 0.0

    print(f"샘플 {len(samples)}개", flush=True)
    print(f"시작 각도 (= 차 정면) : {start:+.1f} deg", flush=True)
    print(f"누적 회전량           : {total:+.1f} deg", flush=True)
    print("", flush=True)
    print("해석:", flush=True)
    print("  시작 각도 ~0deg   -> 스캔 0deg = 차 정면. lidar_yaw 는 0 이어야 한다.", flush=True)
    print("  시작 각도 ~180deg -> 스캔 0deg = 차 후방. lidar_yaw = pi 가 맞다.", flush=True)
    print("  시작 각도 ~+90 / -90deg -> 라이다가 90도 돌아 장착돼 있다.", flush=True)
    print("  누적 회전 +360deg -> 반시계 이동이 스캔에서도 각도 증가 (부호 정상)", flush=True)
    print("  누적 회전 -360deg -> 부호가 뒤집혀 있다 (미러)", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
