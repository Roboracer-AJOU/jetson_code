#!/usr/bin/env python3
"""path_following 이 내는 토픽을 **누가 듣는지** 와 발행 빈도를 모은다.

foxglove 만 듣는 토픽은 시각화 전용이다. 그런 건 빈도를 낮추거나 래치로
바꿔도 주행에 닿지 않는다. 반대로 다른 노드가 듣는 건 건드리면 안 된다.

    python3 debug/topic_consumers.py
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

OURS = {
    "local_planner_node",
    "stanley_waypoint_follow_node",
    "emergency_brake_node",
    "integrated_obstacle_node",
    "fgm_node",
    "control_node",
}
SKIP = {"/parameter_events", "/rosout", "/tf", "/tf_static", "/clock"}


def main():
    rclpy.init()
    n = Node("topic_consumer_probe")
    import time

    time.sleep(2.0)

    rows = []
    for topic, types in n.get_topic_names_and_types():
        if topic in SKIP:
            continue
        pubs = n.get_publishers_info_by_topic(topic)
        if not any(p.node_name in OURS for p in pubs):
            continue
        subs = n.get_subscriptions_info_by_topic(topic)
        names = sorted({s.node_name for s in subs})
        producer = sorted({p.node_name for p in pubs if p.node_name in OURS})
        real = [x for x in names if x != "foxglove_bridge" and x not in ("_ros2cli",)]
        real = [x for x in real if not x.startswith("_")]
        rows.append((topic, types[0].split("/")[-1], producer, names, real))

    rows.sort(key=lambda r: (len(r[4]) > 0, r[0]))

    print("=== 시각화 전용 (foxglove 만 듣거나 아무도 안 들음) ===")
    for topic, typ, prod, names, real in rows:
        if real:
            continue
        who = ", ".join(names) if names else "(구독자 없음)"
        print(f"  {topic:38s} {typ:22s} <- {','.join(prod):26s}  듣는이: {who}")

    print("\n=== 제어 경로 (다른 노드가 듣는다 — 건들지 말 것) ===")
    for topic, typ, prod, names, real in rows:
        if not real:
            continue
        print(
            f"  {topic:38s} {typ:22s} <- {','.join(prod):26s}  듣는이: {', '.join(real)}"
        )

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
