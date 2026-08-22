#!/usr/bin/env python3
"""rclpy 실행기 자체가 얼마를 먹는지, 엔티티 수에 얼마나 비례하는지 잰다.

콜백을 전부 비워 둔 노드를 실제 스택 옆에 띄우고 같은 토픽을 구독한다.
그러면 남는 CPU 는 100 % rclpy 오버헤드다. 구성 요소(TF 리스너, 퍼블리셔
개수)를 바꿔 가며 재면 "무엇을 줄이면 얼마가 빠지는지" 가 나온다.

    python3 debug/bench_executor.py full     # local_planner 와 같은 구성
    python3 debug/bench_executor.py notf     # TF 리스너만 뺀 것
    python3 debug/bench_executor.py nopubs   # 퍼블리셔만 뺀 것
    python3 debug/bench_executor.py bare     # 타이머 하나만
"""
from __future__ import annotations

import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Float64, Float32, String, UInt8
from tf2_ros import Buffer, TransformListener

CLK = os.sysconf("SC_CLK_TCK")
SUBS = [
    ("/static_obstacles", Float32MultiArray),
    ("/dynamic_obstacles", Float32MultiArray),
    ("/vehicle/speed_mps", Float64),
    ("/emergency_brake", Bool),
    ("/fgm_target", PointStamped),
]


def main_thread_jiffies():
    pid = os.getpid()
    with open(f"/proc/{pid}/task/{pid}/stat") as f:
        body = f.read()
    parts = body[body.rindex(")") + 2 :].split()
    return int(parts[11]) + int(parts[12])


def main():
    variant = sys.argv[1] if len(sys.argv) > 1 else "full"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

    rclpy.init()
    n = Node(f"exec_probe_{variant}")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    kept = []
    if variant != "bare":
        for topic, typ in SUBS:
            kept.append(n.create_subscription(typ, topic, lambda _m: None, qos))
        kept.append(
            n.create_subscription(OccupancyGrid, "/map", lambda _m: None, 1)
        )

    if variant in ("full", "nopubs"):
        buf = Buffer()
        kept.append(TransformListener(buf, n))

    if variant in ("full", "notf"):
        # local_planner 가 내는 것과 같은 개수/종류
        kept.append(n.create_publisher(Path, "~/local_path", 1))
        kept.append(n.create_publisher(Path, "~/csv", 1))
        for t, typ in [
            ("fgm_enable", Bool),
            ("fgm_angle", Float32),
            ("planned", Bool),
            ("mode", String),
            ("cond", UInt8),
            ("reason", String),
            ("scale", Float64),
            ("override", Bool),
        ]:
            kept.append(n.create_publisher(typ, f"~/{t}", 1))

    n.create_timer(1.0 / 40.0, lambda: None)
    n.create_timer(0.5, lambda: None)

    def stop_later():
        time.sleep(dur + 1.0)
        try:
            rclpy.shutdown()
        except Exception:
            pass

    threading.Thread(target=stop_later, daemon=True).start()

    # 워밍업 1 s 후 측정 시작
    t_end = time.time() + 1.0
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)

    j0, t0 = main_thread_jiffies(), time.time()
    t_end = t0 + dur
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    j1, t1 = main_thread_jiffies(), time.time()

    pct = 100.0 * (j1 - j0) / CLK / (t1 - t0)
    print(f"{variant:8s}  메인스레드 CPU {pct:5.1f}%   (콜백은 전부 빈 함수)")


if __name__ == "__main__":
    main()
