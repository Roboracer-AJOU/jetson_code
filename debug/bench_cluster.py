#!/usr/bin/env python3
"""cluster_scan_xy 내부 분해. 어디가 300 µs 를 쓰는지 본다."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "path_following"))
from path_following.scan_cluster import ClusterParams, cluster_scan_xy  # noqa: E402

N = 1080
ang = np.linspace(-2.356, 2.356, N)
rng = np.full(N, 6.0) + 0.6 * np.cos(3.0 * ang)
rng[np.abs(ang - 0.25) < 0.09] = 2.4
px = rng * np.cos(ang)
py = rng * np.sin(ang)
P = ClusterParams()


def us(fn, k=1500):
    fn()
    t0 = time.perf_counter()
    for _ in range(k):
        fn()
    return (time.perf_counter() - t0) / k * 1e6


def main():
    theta = np.arctan2(py, px)
    order = np.argsort(theta)
    sx, sy = px[order], py[order]
    rr = np.hypot(sx, sy)
    step = np.hypot(np.diff(sx), np.diff(sy))
    breaks = np.nonzero(step > P.gap_threshold_m)[0] + 1
    n_seg = len(breaks) + 1

    print(f"점 {N}, 세그먼트 {n_seg}, 클러스터 {len(cluster_scan_xy(px, py, angle_increment=float(ang[1]-ang[0]), params=P))}\n")
    rows = [
        ("arctan2(py, px)", lambda: np.arctan2(py, px)),
        ("argsort", lambda: np.argsort(theta)),
        ("px[order], py[order]", lambda: (px[order], py[order])),
        ("hypot(sx, sy)", lambda: np.hypot(sx, sy)),
        ("hypot(diff, diff)", lambda: np.hypot(np.diff(sx), np.diff(sy))),
        ("nonzero(step > thr)", lambda: np.nonzero(step > P.gap_threshold_m)[0] + 1),
    ]
    for name, fn in rows:
        print(f"{name:34s} {us(fn):7.2f} us")

    # 세그먼트 루프만
    starts = np.concatenate(([0], breaks))
    ends = np.concatenate((breaks, [N]))

    def seg_loop():
        out = []
        for s, e in zip(starts, ends):
            if e <= s:
                continue
            cx = float(np.mean(sx[s:e]))
            cy = float(np.mean(sy[s:e]))
            r_rep = float(np.hypot(cx, cy))
            if (e - s) < P.min_points:
                continue
            seg_x, seg_y = sx[s:e], sy[s:e]
            span = float(max(np.max(seg_x) - np.min(seg_x), np.max(seg_y) - np.min(seg_y)))
            kmin = int(np.argmin(seg_x * seg_x + seg_y * seg_y))
            out.append((cx, cy, r_rep, span, kmin))
        return out

    print(f"{'세그먼트 루프 전체':34s} {us(seg_loop):7.2f} us")
    print("-" * 44)
    print(
        f"{'cluster_scan_xy 전체':34s} "
        f"{us(lambda: cluster_scan_xy(px, py, angle_increment=float(ang[1]-ang[0]), params=P)):7.2f} us"
    )


if __name__ == "__main__":
    main()
