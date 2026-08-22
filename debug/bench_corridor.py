#!/usr/bin/env python3
"""FGM `_corridor_clear_distance` 내부 분해 측정.

스캔당 수십 번 도는 자리라 어디가 무거운지 알아야 손댈 곳이 정해진다.
"""
from __future__ import annotations

import math
import time

import numpy as np

N = 1080
HALF_W = 0.254
MARGIN = 0.15
PRE = 10.0

ang = np.linspace(-2.356, 2.356, N)
rng = np.full(N, 6.0) + 0.6 * np.cos(3.0 * ang)
A = 0.2


def us(fn, k=3000):
    fn()
    t0 = time.perf_counter()
    for _ in range(k):
        fn()
    return (time.perf_counter() - t0) / k * 1e6


def wrap(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def current(angle=A):
    d = wrap(ang - angle)
    valid = (rng > 0.0) & (np.abs(d) < math.pi * 0.5)
    if not np.any(valid):
        return PRE
    r = rng[valid]
    da = d[valid]
    along = r * np.cos(da)
    perp = np.abs(r * np.sin(da))
    blocking = (perp < HALF_W) & (along > 0.0)
    if not np.any(blocking):
        return PRE
    return max(0.0, float(along[blocking].min()) - MARGIN)


def main():
    d = wrap(ang - A)
    valid = (rng > 0.0) & (np.abs(d) < math.pi * 0.5)
    r = rng[valid]
    da = d[valid]
    along = r * np.cos(da)
    perp = np.abs(r * np.sin(da))

    rows = [
        ("wrap_pi_np(wrapped - angle)", lambda: wrap(ang - A)),
        ("(r>0) & (|d| < pi/2)", lambda: (rng > 0.0) & (np.abs(d) < math.pi * 0.5)),
        ("|d| < pi/2 만 (r>0 은 사전계산)", lambda: np.abs(d) < math.pi * 0.5),
        ("r[valid], d[valid] 팬시인덱싱", lambda: (rng[valid], d[valid])),
        ("cos/sin/abs (부분배열)", lambda: (r * np.cos(da), np.abs(r * np.sin(da)))),
        ("blocking 마스크 + min", lambda: along[(perp < HALF_W) & (along > 0.0)].min()),
        ("np.any(valid)", lambda: np.any(valid)),
    ]
    print(f"빔 {N}, valid {int(valid.sum())}\n")
    for name, fn in rows:
        print(f"{name:36s} {us(fn):7.2f} us")
    print("-" * 46)
    print(f"{'현재 전체':36s} {us(current):7.2f} us")


if __name__ == "__main__":
    main()
