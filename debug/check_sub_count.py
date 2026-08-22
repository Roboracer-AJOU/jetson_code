#!/usr/bin/env python3
"""`publisher.get_subscription_count()` 를 믿어도 되는지 확인한다.

시각화 게이트가 이 값 하나로 발행 여부를 정하므로, 구독자가 실제로 붙었는데
0 으로 보이는 구간이 길면 화면이 오래 비어 보인다. (제어 토픽에는 게이트를
걸지 않았으므로 주행 위험은 아니다.)

외부 탐침 노드로 `get_subscriptions_info_by_topic` 을 부르는 것과 달리,
이 값은 **퍼블리셔 자신이 매칭한 상대** 를 세는 것이라 원격 디스커버리에
의존하지 않는다. 그 차이를 실제로 보여 준다.

    python3 debug/check_sub_count.py
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

TOPIC = "/subcount_probe"


def pump(nodes, seconds):
    end = time.time() + seconds
    while time.time() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.005)


def main():
    rclpy.init()
    pub_node = Node("count_pub")
    pub = pub_node.create_publisher(Float64, TOPIC, 10)

    print("1) 구독자 없음")
    pump([pub_node], 1.0)
    print(f"   count = {pub.get_subscription_count()}  (0 이어야 한다)")

    print("\n2) 구독자 하나 붙임 — 몇 ms 만에 보이나")
    sub_node = Node("count_sub")
    sub_node.create_subscription(Float64, TOPIC, lambda _m: None, 10)
    t0 = time.time()
    seen = None
    while time.time() - t0 < 5.0:
        pump([pub_node, sub_node], 0.02)
        if pub.get_subscription_count() > 0:
            seen = time.time() - t0
            break
    print(
        f"   count = {pub.get_subscription_count()}  "
        + (f"({seen*1e3:.0f} ms 만에 반영)" if seen is not None else "(5초 안에 못 봄!)")
    )

    print("\n3) 구독자 둘")
    sub2 = Node("count_sub2")
    sub2.create_subscription(Float64, TOPIC, lambda _m: None, 10)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        pump([pub_node, sub_node, sub2], 0.02)
        if pub.get_subscription_count() >= 2:
            break
    print(f"   count = {pub.get_subscription_count()}  (2 여야 한다)")

    print("\n4) 구독자 떼어냄")
    sub2.destroy_node()
    sub_node.destroy_node()
    t0 = time.time()
    while time.time() - t0 < 5.0:
        pump([pub_node], 0.02)
        if pub.get_subscription_count() == 0:
            break
    gone = time.time() - t0
    print(f"   count = {pub.get_subscription_count()}  ({gone*1e3:.0f} ms 만에 0)")

    print("\n5) 이미 돌고 있는 실제 스택에서 — 퍼블리셔 자신이 세는 값")
    print("   (외부 탐침의 get_subscriptions_info_by_topic 과 달리")
    print("    로컬 매칭 상태라 디스커버리 지연을 안 탄다)")

    pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
