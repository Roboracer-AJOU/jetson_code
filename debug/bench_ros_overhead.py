#!/usr/bin/env python3
"""ROS 자체 오버헤드 측정 — TF 조회, 발행, LaserScan 역직렬화.

차가 멈춰 있어도 노드가 코어의 40 % 를 쓴다면 계산이 아니라 여기다.
주기당 몇 번씩 도는 것들이라 한 번 값이 작아 보여도 합치면 크다.

    python3 debug/bench_ros_overhead.py
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

PLANNER_HZ = 40.0
STANLEY_HZ = 33.3
AEB_HZ = 50.0
SCAN_HZ = 40.0
N_BEAMS = 1080


def us(fn, k=300):
    fn()
    t0 = time.perf_counter()
    for _ in range(k):
        fn()
    return (time.perf_counter() - t0) / k * 1e6


def main():
    rclpy.init()
    n = Node("overhead_probe")

    # --- TF: 자체 브로드캐스트 후 조회 (실제 스택과 같은 경로) ---
    buf = Buffer()
    TransformListener(buf, n)
    br = TransformBroadcaster(n)

    def send_tf():
        t = TransformStamped()
        t.header.stamp = n.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.rotation.w = 1.0
        br.sendTransform(t)

    for _ in range(20):
        send_tf()
        rclpy.spin_once(n, timeout_sec=0.02)

    from rclpy.duration import Duration
    from rclpy.time import Time

    def lookup_latest():
        try:
            return buf.lookup_transform("map", "base_link", Time())
        except Exception:
            return None

    def lookup_timeout():
        try:
            return buf.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.05)
            )
        except Exception:
            return None

    ok = lookup_latest() is not None
    rows = []
    if ok:
        rows.append(("TF lookup (latest, 성공)", us(lookup_latest), PLANNER_HZ * 2))
        rows.append(("TF lookup (timeout 지정)", us(lookup_timeout, 100), PLANNER_HZ))
    else:
        print("TF 조회가 안 된다 — 이 항목은 건너뛴다\n")

    # --- 발행 오버헤드 ---
    pb = n.create_publisher(Bool, "~/b", 1)
    pf = n.create_publisher(Float64, "~/f", 1)
    ps = n.create_publisher(String, "~/s", 1)
    pa = n.create_publisher(Float64MultiArray, "~/a", 1)
    pp = n.create_publisher(Path, "~/p", 1)

    bmsg, fmsg, smsg = Bool(), Float64(), String()
    smsg.data = "GLOBAL"
    amsg = Float64MultiArray()
    amsg.data = [0.0] * 12
    empty_path = Path()
    empty_path.header.frame_id = "map"

    rows += [
        ("publish Bool", us(lambda: pb.publish(bmsg)), PLANNER_HZ * 2),
        ("publish Float64", us(lambda: pf.publish(fmsg)), PLANNER_HZ * 3),
        ("publish String", us(lambda: ps.publish(smsg)), PLANNER_HZ * 2),
        ("publish Float64MultiArray(12)", us(lambda: pa.publish(amsg)), STANLEY_HZ),
        ("publish 빈 Path", us(lambda: pp.publish(empty_path)), PLANNER_HZ),
    ]

    # --- 시계 / 스탬프 ---
    rows += [
        ("get_clock().now()", us(lambda: n.get_clock().now()), PLANNER_HZ * 6),
        ("now().to_msg()", us(lambda: n.get_clock().now().to_msg()), PLANNER_HZ * 4),
        (
            "now().nanoseconds",
            us(lambda: n.get_clock().now().nanoseconds),
            PLANNER_HZ * 8,
        ),
    ]

    # --- LaserScan 역직렬화 흉내 ---
    scan = LaserScan()
    scan.angle_min = -2.356
    scan.angle_increment = 4.712 / N_BEAMS
    scan.ranges = [3.0] * N_BEAMS
    rows += [
        (
            "np.array(scan.ranges, float64)",
            us(lambda: np.array(scan.ranges, dtype=np.float64)),
            SCAN_HZ * 3,
        ),
        ("list(scan.ranges)", us(lambda: list(scan.ranges)), SCAN_HZ),
    ]

    print(f"{'항목':34s} {'1회 µs':>9} {'호출/s':>8} {'코어%':>7}")
    print("-" * 64)
    tot = 0.0
    for name, t, rate in sorted(rows, key=lambda r: -r[1] * r[2]):
        pct = 100.0 * t * 1e-6 * rate
        tot += pct
        print(f"{name:34s} {t:9.1f} {rate:8.0f} {pct:7.2f}")
    print("-" * 64)
    print(f"{'합계 (가정한 호출율 기준)':34s} {'':9} {'':8} {tot:7.2f}")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
