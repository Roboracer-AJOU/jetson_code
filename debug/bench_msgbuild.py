#!/usr/bin/env python3
"""ROS 메시지 조립 비용. Path/Marker 를 매 주기 만드는 게 얼마인지 본다."""
from __future__ import annotations

import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

STANLEY_HZ = 33.3
PLANNER_HZ = 40.0
SCAN_HZ = 40.0


def us(fn, k=400):
    fn()
    t0 = time.perf_counter()
    for _ in range(k):
        fn()
    return (time.perf_counter() - t0) / k * 1e6


def build_path(n):
    p = Path()
    p.header.frame_id = "map"
    for i in range(n):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(i) * 0.05
        ps.pose.position.y = 0.1
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def build_path_batch(n):
    """append 대신 리스트를 한 번에 넣는다 — 검증이 한 번만 돈다."""
    p = Path()
    p.header.frame_id = "map"
    poses = []
    for i in range(n):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(i) * 0.05
        ps.pose.position.y = 0.1
        ps.pose.orientation.w = 1.0
        poses.append(ps)
    p.poses = poses
    return p


def build_markers(n):
    ma = MarkerArray()
    for i in range(n):
        m = Marker()
        m.header.frame_id = "laser"
        m.ns = "obs"
        m.id = i
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = 1.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.a = 0.8
        ma.markers.append(m)
    return ma


_POOL: list = []


def build_path_pooled(n):
    """PoseStamped 를 미리 만들어 두고 좌표만 갈아 끼운다."""
    while len(_POOL) < n:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.orientation.w = 1.0
        _POOL.append(ps)
    p = Path()
    p.header.frame_id = "map"
    for i in range(n):
        ps = _POOL[i]
        ps.pose.position.x = float(i) * 0.05
        ps.pose.position.y = 0.1
    p.poses = _POOL[:n]
    return p


def build_path_pooled_direct(n):
    """속성 접근을 지역변수로 줄인 판. position 객체를 직접 잡는다."""
    while len(_POOL) < n:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.orientation.w = 1.0
        _POOL.append(ps)
    p = Path()
    p.header.frame_id = "map"
    pool = _POOL
    for i in range(n):
        pos = pool[i].pose.position
        pos.x = float(i) * 0.05
        pos.y = 0.1
    p.poses = pool[:n]
    return p


_PREBUILT: list = []


def build_path_prebuilt_slice(n, total=750):
    """좌표가 안 변하는 경로 — 통째로 미리 만들어 두고 잘라 쓴다."""
    if not _PREBUILT:
        for i in range(total):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(i) * 0.05
            ps.pose.position.y = 0.1
            ps.pose.orientation.w = 1.0
            _PREBUILT.append(ps)
    p = Path()
    p.header.frame_id = "map"
    p.poses = _PREBUILT[:n]
    return p


def main():
    rows = [
        ("Path 140점 (Stanley tracked_path)", lambda: build_path(140), STANLEY_HZ),
        ("Path 140점, 일괄 대입", lambda: build_path_batch(140), STANLEY_HZ),
        ("Path 180점 (local_path)", lambda: build_path(180), PLANNER_HZ),
        ("Path 750점 (raceline viz)", lambda: build_path(750), 2.0),
        ("MarkerArray 8개 (장애물)", lambda: build_markers(8), SCAN_HZ),
        ("Float64MultiArray 12개", lambda: _f64(12), STANLEY_HZ),
        ("[대안] Path 140, 포즈 풀 재사용", lambda: build_path_pooled(140), STANLEY_HZ),
        ("[대안] Path 140, 풀+지역변수", lambda: build_path_pooled_direct(140), STANLEY_HZ),
        ("[대안] Path 140, 통째 사전조립", lambda: build_path_prebuilt_slice(140), STANLEY_HZ),
        ("[대안] Path 180, 포즈 풀 재사용", lambda: build_path_pooled_direct(180), PLANNER_HZ),
    ]
    print(f"{'메시지':42s} {'1회 µs':>9} {'호출/s':>8} {'코어%':>7}")
    print("-" * 72)
    tot = 0.0
    for name, fn, rate in rows:
        t = us(fn)
        pct = 100.0 * t * 1e-6 * rate
        tot += pct
        print(f"{name:42s} {t:9.1f} {rate:8.1f} {pct:7.2f}")
    print("-" * 72)
    print(f"{'합계 (일괄대입 행 제외하고 대략)':42s} {'':9} {'':8} {tot:7.2f}")


def _f64(n):
    m = Float64MultiArray()
    m.data = [float(i) for i in range(n)]
    return m


if __name__ == "__main__":
    main()
