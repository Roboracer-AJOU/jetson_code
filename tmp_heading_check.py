#!/usr/bin/env python3
"""차를 손으로 앞으로 밀 때, map 상 이동 방향이 base_link 헤딩과 같은지 본다.

laser TF 의 yaw 가 180도 틀리면 Cartographer 가 추정하는 차량 헤딩도 180도
돌아간다. 그러면 "물리적으로 앞으로 밀었는데 map 에서는 뒤로 간다" 가 된다.
이걸 직접 재서 lidar_yaw = pi 와 0 중 어느 쪽이 실제 장착인지 판정한다.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


DURATION_S = 60.0
MIN_STEP_M = 0.02


def yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class HeadingCheck(Node):
    def __init__(self) -> None:
        super().__init__("tmp_heading_check")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.odom_v = 0.0
        self.create_subscription(Odometry, "/odom", self._cb_odom, 10)

    def _cb_odom(self, msg: Odometry) -> None:
        self.odom_v = float(msg.twist.twist.linear.x)

    def pose(self):
        try:
            t = self.buf.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_of(t.transform.rotation))


def main() -> int:
    rclpy.init()
    n = HeadingCheck()
    for _ in range(60):
        rclpy.spin_once(n, timeout_sec=0.05)

    print("차를 손으로 천천히 앞으로 1 m 정도 밀어주세요. 60초 동안 봅니다.", flush=True)

    prev = None
    fwd_sum = 0.0
    path_len = 0.0
    odom_fwd_sum = 0.0
    t_end = time.monotonic() + DURATION_S
    last_report = 0.0

    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        odom_fwd_sum += n.odom_v * 0.05
        p = n.pose()
        if p is None:
            continue
        if prev is None:
            prev = p
            continue
        dx, dy = p[0] - prev[0], p[1] - prev[1]
        step = math.hypot(dx, dy)
        if step < MIN_STEP_M:
            continue
        # 이동 벡터를 헤딩 방향으로 투영
        hx, hy = math.cos(prev[2]), math.sin(prev[2])
        fwd_sum += dx * hx + dy * hy
        path_len += step
        prev = p
        if time.monotonic() - last_report > 1.0:
            last_report = time.monotonic()
            print(
                f"  이동 {path_len:5.2f}m  헤딩방향투영 {fwd_sum:+6.2f}m  "
                f"휠오도 적분 {odom_fwd_sum:+6.2f}m",
                flush=True,
            )

    print("", flush=True)
    print(f"총 이동거리      = {path_len:.3f} m", flush=True)
    print(f"헤딩방향 투영합  = {fwd_sum:+.3f} m", flush=True)
    print(f"휠오도 전진 적분 = {odom_fwd_sum:+.3f} m", flush=True)
    if path_len < 0.15:
        print("판정: 이동량 부족 — 차를 더 밀어야 합니다.", flush=True)
    elif fwd_sum > 0.5 * path_len:
        print("판정: map 이동방향 == base_link 헤딩 → lidar_yaw=pi 가 실제 장착과 맞다.", flush=True)
        print("      => 라이다는 뒤를 보고 있고, TF 는 옳다. 알고리즘(FGM/AEB)이 뒤를 본다.", flush=True)
    elif fwd_sum < -0.5 * path_len:
        print("판정: map 이동방향 == base_link 헤딩의 반대 → lidar_yaw=pi 가 틀렸다.", flush=True)
        print("      => 라이다는 앞을 보고 있다. lidar_yaw=0 으로 재실행해야 한다.", flush=True)
    else:
        print("판정: 애매함(옆으로 밀었거나 회전 위주) — 직선으로 다시 밀어주세요.", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
