#!/usr/bin/env python3
"""핫패스 함수 단위 벤치마크. 최적화 전후를 같은 자로 잰다.

    python3 debug/bench_hotpaths.py

노드를 띄우지 않고 실제 데이터(레이스라인 750점, 스캔 1080빔)로 각 함수를
반복 호출해 1회당 마이크로초를 낸다. 주기율을 곱하면 그 함수가 차지하는
코어 비율이 나온다 — 그게 최적화 대상을 고르는 기준이다.

`--json out.json` 으로 저장해 두고 고친 뒤 다시 돌리면 차이를 찍어 준다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "path_following"))

from path_following import track_sliding as ts  # noqa: E402
from path_following.avoidance_safety import first_blocked_index  # noqa: E402
from path_following.scan_cluster import ClusterParams, cluster_scan_xy  # noqa: E402

N_BEAMS = 1080
SCAN_HZ = 40.0
PLANNER_HZ = 40.0
STANLEY_HZ = 33.3


def bench(fn, n=None, budget_s=0.35):
    """fn 을 budget_s 동안 돌려 1회당 초를 낸다."""
    fn()  # 워밍업 (첫 호출의 캐시·임포트 비용 제외)
    if n is None:
        t0 = time.perf_counter()
        fn()
        one = max(time.perf_counter() - t0, 1e-9)
        n = max(3, min(20000, int(budget_s / one)))
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n


def load_track():
    csv = ROOT / "src" / "path_following" / "config" / "raceline.csv"
    pts = ts.load_csv_xy(str(csv))
    return pts


def make_planner_stub(pts):
    """`_closest_on_loop` 만 돌릴 수 있는 최소 스텁. 노드는 안 띄운다."""
    from path_following.local_planner_node import LocalPlannerNode

    class _P:
        _closest_on_loop = LocalPlannerNode._closest_on_loop

    p = _P()
    p._xs_np = np.asarray([q[0] for q in pts], dtype=np.float64)
    p._ys_np = np.asarray([q[1] for q in pts], dtype=np.float64)
    bx = np.roll(p._xs_np, -1)
    by = np.roll(p._ys_np, -1)
    p._abx_np = bx - p._xs_np
    p._aby_np = by - p._ys_np
    p._ab2_np = p._abx_np**2 + p._aby_np**2
    p._ab_ok_np = p._ab2_np >= 1e-14
    p._ab_bad_np = ~p._ab_ok_np
    p._ab_zeros_np = np.zeros_like(p._ab2_np)
    return p


def make_fgm_stub():
    from path_following.fgm_node import FGMNode

    class _F:
        _corridor_clear_distance = FGMNode._corridor_clear_distance
        _corridor_clear_reference = FGMNode._corridor_clear_reference
        _scan_xy = FGMNode._scan_xy

    f = _F()
    f.corridor_half_width = 0.254
    f.corridor_stop_margin = 0.15
    f.preprocess_dist = 10.0
    f._scan_positive = None
    f._scan_cx = None
    f._scan_cy = None
    f._xy_src_ranges = None
    f._xy_src_wrapped = None
    return f


def fake_scan(n=N_BEAMS):
    """실제와 비슷한 스캔: 복도 벽 + 중간에 박스 하나."""
    ang = np.linspace(-2.356, 2.356, n)
    rng = np.full(n, 6.0)
    rng += 0.6 * np.cos(3.0 * ang)  # 벽이 굽은 흉내
    box = np.abs(ang - 0.25) < 0.09
    rng[box] = 2.4
    rng += np.random.default_rng(0).normal(0.0, 0.01, n)
    return ang, rng


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=str, default="")
    ap.add_argument("--base", type=str, default="")
    args = ap.parse_args()

    pts = load_track()
    n_pts = len(pts)
    ang, rng = fake_scan()
    xs = rng * np.cos(ang)
    ys = rng * np.sin(ang)

    res = {}

    # --- 1. 코리도 필터: 장애물 하나당 한 번, 매 플래너 주기 ---
    res["lateral_distance_to_closed_polyline"] = (
        bench(lambda: ts.lateral_distance_to_closed_polyline(1.3, -0.4, pts)),
        PLANNER_HZ * 4,  # 장애물 4개 가정
        "코리도 필터 (플래너 40 Hz × 장애물 4)",
    )

    # --- 2. Stanley 앵커 투영 ---
    loop = ts.LoopTrackSliding(pts, 140, 120)
    loop.closest_projection_on_loop(pts[10][0], pts[10][1])  # 앵커 초기화
    res["closest_projection_on_loop"] = (
        bench(lambda: loop.closest_projection_on_loop(pts[12][0] + 0.2, pts[12][1])),
        STANLEY_HZ,
        "Stanley CSV 투영 (33 Hz)",
    )
    res["sliding_xy"] = (
        bench(lambda: loop.sliding_xy(pts[12][0] + 0.2, pts[12][1])),
        STANLEY_HZ,
        "Stanley 윈도우 조립 (33 Hz)",
    )

    # --- 3. 플래너 Frenet 투영 — 실제 노드 메서드를 그대로 잰다 ---
    planner = make_planner_stub(pts)
    res["_closest_on_loop"] = (
        bench(lambda: planner._closest_on_loop(1.3, -0.4)),
        PLANNER_HZ * 6,  # ego + 장애물 + rejoin/maneuver
        "플래너 Frenet 투영 (40 Hz × 6)",
    )

    # --- 4. FGM 코리도 여유 (스캔당 수십 회) ---
    fgm = make_fgm_stub()
    wrapped = ang.copy()
    # 매번 새 스캔인 척하지 않는다 — 실제로도 한 스캔에 수십 번 불린다.
    res["_corridor_clear_distance"] = (
        bench(lambda: fgm._corridor_clear_distance(rng, wrapped, 0.2)),
        SCAN_HZ * 30,  # gap fit(갭수×5) + pick(11)
        "FGM 코리도 여유 (40 Hz × ~30)",
    )

    # --- 5. 클러스터링 ---
    cp = ClusterParams()
    ainc = float(ang[1] - ang[0])
    res["cluster_scan_xy"] = (
        bench(lambda: cluster_scan_xy(xs, ys, angle_increment=ainc, params=cp)),
        SCAN_HZ * 2,  # integrated + (있으면) static
        "스캔 클러스터링 (40 Hz × 2)",
    )

    # --- 5b. Stanley 최근접 세그먼트 루프 (경로 140점, 매 틱) ---
    from path_following.stanley_waypoint_follow_node import closest_point_on_segment

    win = loop.sliding_xy(pts[12][0], pts[12][1])

    def stanley_nearest():
        bd2, bi, bqx, bqy = float("inf"), 0, 0.0, 0.0
        qx0, qy0 = win[0]
        for i in range(len(win) - 1):
            ax, ay = win[i]
            bx_, by_ = win[i + 1]
            qx, qy, _ = closest_point_on_segment(qx0 + 0.1, qy0 + 0.1, ax, ay, bx_, by_)
            d2 = (qx0 + 0.1 - qx) ** 2 + (qy0 + 0.1 - qy) ** 2
            if d2 < bd2:
                bd2, bi, bqx, bqy = d2, i, qx, qy
        return bi, bqx, bqy

    res["stanley 최근접 세그먼트 루프"] = (
        bench(stanley_nearest),
        STANLEY_HZ,
        "Stanley 경로 최근접 (33 Hz, 140점)",
    )

    # --- 5c. 파이썬 튜플 리스트 → ndarray (경로를 벡터화하려면 내야 할 값) ---
    res["np.asarray(경로 140점)"] = (
        bench(lambda: np.asarray(win, dtype=np.float64)),
        STANLEY_HZ,
        "리스트→배열 변환 비용 (참고)",
    )

    # --- 6. 경로 충돌 검사 ---
    path_pts = [(1.0 + i * 0.055, 0.2 + i * 0.01) for i in range(180)]
    disks = [(3.0, 0.5, 0.4), (5.0, 0.2, 0.4)]
    res["first_blocked_index"] = (
        bench(lambda: first_blocked_index(path_pts, None, disks, start_index=1)),
        PLANNER_HZ,
        "경로 충돌 검사, 장애물만 (40 Hz)",
    )

    # --- 출력 ---
    print(f"레이스라인 {n_pts} 점, 스캔 {N_BEAMS} 빔\n")
    print(f"{'함수':38s} {'1회 µs':>9} {'호출/s':>8} {'코어%':>7}  설명")
    print("-" * 100)
    total = 0.0
    out = {}
    for name, (sec, rate, desc) in sorted(
        res.items(), key=lambda kv: -kv[1][0] * kv[1][1]
    ):
        pct = 100.0 * sec * rate
        total += pct
        out[name] = {"us": sec * 1e6, "rate": rate, "pct": pct}
        print(f"{name:38s} {sec*1e6:9.1f} {rate:8.0f} {pct:7.2f}  {desc}")
    print("-" * 100)
    print(f"{'합계':38s} {'':9} {'':8} {total:7.2f}")

    if args.base:
        try:
            base = json.load(open(args.base))
        except OSError:
            base = None
        if base:
            print(f"\n{'함수':38s} {'전 µs':>9} {'후 µs':>9} {'배속':>7}")
            print("-" * 70)
            for k, v in out.items():
                if k in base:
                    b = base[k]["us"]
                    print(f"{k:38s} {b:9.1f} {v['us']:9.1f} {b/max(v['us'],1e-9):6.2f}x")
            bt = sum(x["pct"] for x in base.values())
            print(f"\n코어 합계 {bt:.2f}% → {total:.2f}%  ({bt-total:+.2f}%p)")

    if args.json:
        json.dump(out, open(args.json, "w"), indent=1)
        print(f"\n저장: {args.json}")


if __name__ == "__main__":
    main()
