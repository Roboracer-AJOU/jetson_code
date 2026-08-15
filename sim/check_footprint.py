"""레이스라인을 '완벽하게' 따라갈 때 차체가 벽에 닿는지 검사.

check_map.py 는 라인의 *중심점* 이 벽에서 얼마나 떨어졌는지만 본다. 하지만
차는 점이 아니다. 뒷축 기준 앞으로 0.33m, 옆으로 0.15m 인 사각형이라
코너에서 앞바깥 모서리가 중심보다 훨씬 바깥을 지난다. 중심 여유가 0.36m 인데
모서리까지 거리가 hypot(0.33,0.15)=0.36m 면 여유가 0 이다.

여기서 걸리면 그건 추종기 버그가 아니라 **라인 자체가 이 차로는 못 도는 라인**
이라는 뜻이다. 컨트롤러를 아무리 튜닝해도 안 된다.

  python3 sim/check_footprint.py [raceline|centerline]
"""
from __future__ import annotations

import math
import sys

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/sim")
import numpy as np
from race_sim import CAR_FRONT_M, CAR_HALF_WIDTH, CAR_REAR_M, GridMap
from race_sim import MAP_YAML as MAP

NAME = sys.argv[1] if len(sys.argv) > 1 else "raceline"
CSV = f"/home/nvidia/f1tenth_ajou/src/path_following/config/{NAME}.csv"

m = GridMap(MAP)
pts = []
with open(CSV) as f:
    for ln in f:
        ln = ln.strip()
        if not ln or ln[0] in "#xX":
            continue
        c = ln.split(",")
        pts.append((float(c[0]), float(c[1])))
n = len(pts)

# 차체 외곽 (뒷축 원점). 모서리만 보면 변 중간이 벽을 파고드는 걸 놓쳐서
# 변도 촘촘히 샘플한다.
body = []
for fx in np.linspace(-CAR_REAR_M, CAR_FRONT_M, 9):
    for fy in np.linspace(-CAR_HALF_WIDTH, CAR_HALF_WIDTH, 5):
        body.append((float(fx), float(fy)))

print(f"맵 {MAP.split('/')[-1]}")
print(f"라인 {NAME}: {n}점")
print(
    f"차체 뒷축 -{CAR_REAR_M} ~ +{CAR_FRONT_M} m, 반폭 {CAR_HALF_WIDTH} m "
    f"(앞모서리 반경 {math.hypot(CAR_FRONT_M, CAR_HALF_WIDTH):.3f} m)"
)

hits = []
margins = []
for i in range(n):
    x, y = pts[i]
    x2, y2 = pts[(i + 3) % n]
    yaw = math.atan2(y2 - y, x2 - x)
    c, s = math.cos(yaw), math.sin(yaw)
    worst = 9e9
    hit = False
    for fx, fy in body:
        px = x + fx * c - fy * s
        py = y + fx * s + fy * c
        if m.is_wall(px, py):
            hit = True
        worst = min(worst, m.wall_clearance(px, py))
    margins.append(worst)
    if hit:
        hits.append((i, x, y, math.degrees(yaw)))

margins = np.array(margins)
print(f"\n차체가 벽에 닿는 웨이포인트 = {len(hits)} / {n}")
if hits:
    print("  (인덱스, x, y, yaw°)")
    for h in hits[:15]:
        print(f"    {h[0]:4d} ({h[1]:7.2f},{h[2]:6.2f}) {h[3]:7.1f}")
    if len(hits) > 15:
        print(f"    ... 외 {len(hits)-15}개")

print(f"\n차체 최소 벽여유: {margins.min():.3f} m @ wp{int(margins.argmin())}")
print(f"  5% 분위 {np.percentile(margins,5):.3f} m, 중앙 {np.median(margins):.3f} m")
for th in (0.05, 0.10, 0.15):
    k = int((margins < th).sum())
    print(f"  여유 {th:.2f}m 미만: {k}점 ({100*k/n:.1f}%)")
