#!/usr/bin/env python3
"""레이스라인이 벽 팽창대 안에 들어가 있는지 잰다.

    python3 debug/raceline_clearance.py

실차 로그에서 재합류 경로가 **차 바로 앞**(5~55 cm)에서 계속 막혔다. 이탈은
0.22~0.35 m 로 작고 헤딩도 21° 이내였다. 그 조건에서 막힌다는 건 경로가
나쁜 게 아니라 **그 자리 자체가 이미 막힌 것으로 판정되고 있다** 는 뜻이다.

여기서는 라이브 `/map` 을 받아 `InflatedMap` 과 똑같은 distance transform 을
돌리고, 레이스라인 각 점에서 벽까지의 실제 여유를 잰다. 여유가 팽창반경보다
작은 구간에서는 그 위를 지나는 어떤 경로도 통과할 수 없다 — 복귀는 물론
회피도 마찬가지다.
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "path_following"))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.avoidance_safety import InflatedMap  # noqa: E402

CSV = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "path_following"
    / "config"
    / "raceline.csv"
)


def load_raceline():
    pts = []
    with open(CSV) as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"])))
    return pts


class Probe(Node):
    def __init__(self):
        super().__init__("raceline_clearance")
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.grid = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)

    def _on_map(self, msg):
        self.grid = msg


def main():
    rclpy.init()
    n = Probe()
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.grid is not None:
            break
    if n.grid is None:
        print("맵을 못 받았다 — 런치가 떠 있는지 확인할 것")
        return

    infl = vg.PATH_CHECK_HALF_WIDTH_M
    im = InflatedMap(n.grid, infl)
    pts = load_raceline()
    print(f"팽창반경 {infl:.3f} m, 레이스라인 {len(pts)} 점, 해상도 {im.res:.3f} m/px")
    print()

    clear = []
    for x, y in pts:
        ix = int((x - im.ox) / im.res)
        iy = int((y - im.oy) / im.res)
        if 0 <= ix < im.w and 0 <= iy < im.h:
            clear.append(float(im.clearance[iy, ix]))
        else:
            clear.append(0.0)
    clear = np.asarray(clear)

    print("=== 레이스라인에서 벽까지 여유 ===")
    print(f"  최소 {clear.min():.3f} m   중앙 {np.median(clear):.3f} m   최대 {clear.max():.3f} m")
    print()
    print("=== 팽창대 안에 들어간 구간 ===")
    for thr, label in ((infl, "라인 자체가 막힘"), (infl + 0.2, "이탈 0.2 m 면 막힘"),
                       (infl + 0.35, "이탈 0.35 m 면 막힘")):
        bad = clear < thr
        print(f"  여유 < {thr:.3f} m : {bad.sum():4d} 점 ({100.0*bad.mean():5.1f} %)  — {label}")

    # 연속 구간으로 묶어서 어디인지 보여 준다
    print()
    print("=== 라인 자체가 막힌 연속 구간 (상위 6개) ===")
    bad = clear < infl
    runs, start = [], None
    for i, b in enumerate(bad):
        if b and start is None:
            start = i
        elif not b and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(bad) - 1))
    runs.sort(key=lambda r: r[0] - r[1])
    if not runs:
        print("  없음 — 라인 자체는 전 구간 통과한다")
    for a, b in runs[:6]:
        x0, y0 = pts[a]
        print(
            f"  idx {a:3d}~{b:3d} ({b - a + 1:3d}점)  최소여유 {clear[a:b+1].min():.3f} m"
            f"  x={x0:+.2f} y={y0:+.2f}"
        )

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
