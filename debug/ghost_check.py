#!/usr/bin/env python3
"""검출이 **한 곳** 에서만 나오는지 본다 — 유령이면 트랙 전체에 흩어진다.

팽창을 0.25 → 0.10 으로 내리면 벽 잔차가 장애물로 샐 수 있다. 정지 측정
에서는 0 이었지만 주행 중에는 스캔왜곡이 더해지므로 확인해야 한다.

박스 위치를 따로 안 알려줘도 된다. 트랙에 박스 하나만 뒀다면 검출은 맵
좌표 한 점 주위에 모여야 하고, 흩어진 나머지가 곧 유령이다. 그래서
검출된 맵 좌표를 1 m 격자로 묶어서, 가장 많이 나온 자리를 박스로 보고
나머지를 센다.

두 기록을 같이 주면 수정 전후를 나란히 비교한다.

    python3 debug/ghost_check.py debug/lap2.csv debug/lap3.csv
"""

from __future__ import annotations

import csv
import math
import sys
from collections import Counter
from pathlib import Path


def load(path: Path):
    rows = []
    with path.open() as f:
        for r in csv.DictReader(f):
            try:
                rows.append(
                    {
                        "t": float(r["t"]),
                        "v": float(r["v"] or 0.0),
                        "n": int(float(r["obs_n"] or 0)),
                        "x": float(r["obs_x"]) if r["obs_x"] else None,
                        "y": float(r["obs_y"]) if r["obs_y"] else None,
                        "d": float(r["obs_d"]) if r["obs_d"] else None,
                        "mode": (r["mode"] or "").strip(),
                        "aeb": (r["aeb"] or "").strip() in ("1", "True", "true"),
                    }
                )
            except (ValueError, KeyError):
                continue
    return rows


def report(path: Path):
    rows = load(path)
    if not rows:
        print(f"{path.name}: 데이터 없음")
        return
    moving = [r for r in rows if abs(r["v"]) > 0.5]
    hits = [r for r in rows if r["n"] > 0 and r["x"] is not None]
    fast = [r for r in hits if abs(r["v"]) > 3.0]

    cells = Counter((round(r["x"]), round(r["y"])) for r in hits)
    if not cells:
        print(f"\n=== {path.name} ===\n  검출 없음")
        return
    (bx, by), top = cells.most_common(1)[0]
    near = [r for r in hits if math.hypot(r["x"] - bx, r["y"] - by) <= 1.5]
    ghost = [r for r in hits if math.hypot(r["x"] - bx, r["y"] - by) > 1.5]
    ghost_fast = [r for r in ghost if abs(r["v"]) > 3.0]
    gcells = Counter((round(r["x"]), round(r["y"])) for r in ghost)
    aeb = sum(1 for r in rows if r["aeb"])

    print(f"\n=== {path.name} ===")
    print(f"  주행 {len(moving)*0.02:.0f}s 중 검출 프레임 {len(hits)}"
          f" (3 m/s 이상에서 {len(fast)})")
    print(f"  가장 많이 나온 자리 ({bx}, {by}) — 이걸 박스로 본다")
    print(f"    그 주변 1.5 m 안   {len(near)} 프레임")
    print(f"    흩어진 나머지      {len(ghost)} 프레임"
          f" ({100.0*len(ghost)/len(hits):.1f}%),"
          f" 그중 3 m/s 이상 {len(ghost_fast)}")
    if gcells:
        spots = ", ".join(f"({x},{y})×{c}" for (x, y), c in gcells.most_common(5))
        print(f"    흩어진 자리 상위   {spots}")
    print(f"  AEB 프레임 {aeb}")


def main():
    args = [Path(a) for a in sys.argv[1:]]
    if not args:
        print(__doc__)
        return
    for p in args:
        if p.is_file():
            report(p)
        else:
            print(f"{p} 없음")
    print("\n  유령이면 여러 자리에 흩어진다. 한 자리에 몰려 있으면 그건 박스다.")


if __name__ == "__main__":
    main()
