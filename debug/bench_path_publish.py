#!/usr/bin/env python3
"""큰 Path 를 발행하는 비용. 조립이 아니라 **발행** 이 얼마인지 본다.

앞선 최적화에서 조립(PoseStamped 750개 만들기)은 없앴지만, 같은 메시지를
계속 발행하면 rclpy 가 매번 파이썬 객체 750개를 C 구조체로 변환한다. 그
값이 크면 "정적인 건 한 번만 래치해서 보내라" 가 답이 된다.

    python3 debug/bench_path_publish.py
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def build(n):
    p = Path()
    p.header.frame_id = "map"
    for i in range(n):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(i) * 0.1
        ps.pose.position.y = 0.0
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def us(fn, k):
    fn()
    t0 = time.perf_counter()
    for _ in range(k):
        fn()
    return (time.perf_counter() - t0) / k * 1e6


def main():
    rclpy.init()
    n = Node("path_pub_probe")
    pub = n.create_publisher(Path, "~/big", 1)

    latched = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub_l = n.create_publisher(Path, "~/big_latched", latched)

    print(f"{'포즈 수':>8}  {'조립 µs':>10}  {'발행 µs':>10}  {'2Hz 코어%':>10}  {'40Hz 코어%':>11}")
    print("-" * 60)
    for size in (140, 300, 750):
        msg = build(size)
        t_build = us(lambda: build(size), 20)
        t_pub = us(lambda: pub.publish(msg), 30)
        print(
            f"{size:8d}  {t_build:10.0f}  {t_pub:10.0f}  "
            f"{100*t_pub*1e-6*2:10.2f}  {100*t_pub*1e-6*40:11.2f}"
        )

    msg = build(750)
    t_l = us(lambda: pub_l.publish(msg), 30)
    print(f"\n래치 퍼블리셔로 750개 1회 발행: {t_l:.0f} µs (한 번만 내면 이후 0)")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
