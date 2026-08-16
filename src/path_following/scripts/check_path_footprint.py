#!/usr/bin/env python3
"""경로 위에 실제 사각형 차체를 얹어 벽을 긁는지 검사.

점-벽 거리(clearance)만 보는 검증은 차를 점으로 취급한다. 앞끝이 뒷축에서
0.50 m 나 되는 차는 코너에서 앞 코너가 경로 바깥으로 쓸리므로, 점 거리가
충분해도 차체는 벽에 닿을 수 있다. 여기서는 base_link 를 경로에 놓고
경로 접선을 yaw 로 삼아 차체 사각형 둘레를 샘플링해 전부 확인한다.

사용:
  python3 check_path_footprint.py <map.yaml> <path.csv> [path2.csv ...]

예:
  python3 check_path_footprint.py ../../../maps/cartographer_map_XXXX.yaml \
      ../config/raceline.csv ../config/centerline.csv
"""
from __future__ import annotations

import csv
import os
import sys

import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
_scripts = os.path.dirname(os.path.abspath(__file__))
_pkg = os.path.join(os.path.dirname(_scripts), "path_following")
for p in (_scripts, _pkg):
    if p not in sys.path:
        sys.path.insert(0, p)

from extract_centerline_from_map import load_map, world_to_pixel  # noqa: E402
import vehicle_geometry as vg  # noqa: E402


def load_xy(path):
    pts = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            try:
                pts.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return np.asarray(pts, dtype=float)


def footprint_offsets(step=0.02):
    """차체 사각형 둘레 + 내부 격자를 body frame 오프셋으로."""
    xs = np.arange(-vg.REAR_M, vg.FRONT_M + 1e-9, step)
    ys = np.arange(-vg.HALF_WIDTH_M, vg.HALF_WIDTH_M + 1e-9, step)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    return gx.ravel(), gy.ravel()


def check(map_yaml, csv_path):
    free, res, ox, oy, (h, w) = load_map(map_yaml)
    pts = load_xy(csv_path)
    n = len(pts)
    if n < 8:
        print(f"  {csv_path}: 점이 너무 적다 ({n})")
        return False

    # 경로 접선 = yaw. 폐루프라 wrap.
    nxt = np.roll(pts, -1, axis=0)
    prv = np.roll(pts, 1, axis=0)
    yaw = np.arctan2(nxt[:, 1] - prv[:, 1], nxt[:, 0] - prv[:, 0])

    bx, by = footprint_offsets()
    cos, sin = np.cos(yaw), np.sin(yaw)

    worst_i = -1
    bad_idx = []
    # 각 웨이포인트에서 차체 전체를 맵에 찍어 본다.
    for i in range(n):
        wx = pts[i, 0] + cos[i] * bx - sin[i] * by
        wy = pts[i, 1] + sin[i] * bx + cos[i] * by
        rr = np.rint((h - 1) - (wy - oy) / res).astype(int)
        cc = np.rint((wx - ox) / res).astype(int)
        inside = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        hit = (~inside).any()
        if not hit:
            hit = (free[rr[inside], cc[inside]] == 0).any()
        if hit:
            bad_idx.append(i)

    # 차체 어느 점이든 벽까지 최소 여유 [m] (distance transform 사용)
    from scipy.ndimage import distance_transform_edt

    dist = distance_transform_edt(free > 0) * res
    min_clear = np.full(n, np.inf)
    for i in range(n):
        wx = pts[i, 0] + cos[i] * bx - sin[i] * by
        wy = pts[i, 1] + sin[i] * bx + cos[i] * by
        rr = np.clip(np.rint((h - 1) - (wy - oy) / res).astype(int), 0, h - 1)
        cc = np.clip(np.rint((wx - ox) / res).astype(int), 0, w - 1)
        d = dist[rr, cc].min()
        min_clear[i] = d
    worst_i = int(np.argmin(min_clear))

    ok = not bad_idx
    tag = "OK  " if ok else "FAIL"
    print(f"  [{tag}] {os.path.basename(csv_path)}  ({n} pts)")
    print(
        f"         차체~벽 최소 여유 = {min_clear[worst_i]:.3f} m "
        f"(idx {worst_i}), 중앙값 {np.median(min_clear):.3f} m"
    )
    if bad_idx:
        print(
            f"         벽 침범 {len(bad_idx)} 점: "
            f"{bad_idx[:12]}{' ...' if len(bad_idx) > 12 else ''}"
        )
    return ok


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    map_yaml = sys.argv[1]
    print(f"map: {os.path.basename(map_yaml)}")
    print(f"vehicle: {vg.describe()}")
    print(
        f"footprint: x∈[{-vg.REAR_M:.2f}, {vg.FRONT_M:.2f}] "
        f"y∈[{-vg.HALF_WIDTH_M:.2f}, {vg.HALF_WIDTH_M:.2f}] (base_link 기준)"
    )
    all_ok = True
    for csv_path in sys.argv[2:]:
        all_ok &= check(map_yaml, csv_path)
    print()
    print("결과:", "전부 통과 — 차체가 어디서도 벽에 닿지 않는다" if all_ok else "침범 있음")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
