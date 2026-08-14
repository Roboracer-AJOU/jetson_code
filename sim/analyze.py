"""궤적 vs 레이스라인 추종오차 분석."""
import json
import math
import sys

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/sim")
sys.path.insert(0, "/home/nvidia/f1tenth_ajou/src/path_following")
import numpy as np
from run_scenario import LINE

name = sys.argv[1] if len(sys.argv) > 1 else "clean"
d = json.load(open(f"/tmp/trace_{name}.json"))
t = d["trace"]
P = np.array([[p[0], p[1]] for p in LINE])


def cte(x, y):
    dd = np.hypot(P[:, 0] - x, P[:, 1] - y)
    i = int(dd.argmin())
    return dd[i], i


print(" t     x      y     v    steer  CTE   wp   요구a_lat")
prev = None
for k, s in enumerate(t):
    if k % 2 and k < len(t) - 8:
        continue
    e, i = cte(s[0], s[1])
    # 실제 선회 반경에서 역산한 횡가속도
    alat = 0.0
    if prev is not None:
        dyaw = math.radians(s[2] - prev[2])
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        if abs(dyaw) > 1e-6:
            alat = s[3] * dyaw / 0.05
    prev = s
    print(
        f"{k*0.05:5.2f} {s[0]:7.2f} {s[1]:6.2f} {s[3]:5.2f} {s[4]:5.0f} "
        f"{e:5.2f} {i:4d} {abs(alat):7.2f}"
    )

# 레이스라인 자체의 요구 횡가속도 (CSV v 로 돌 때)
print("\n레이스라인 구간별 요구 a_lat (CSV v 기준):")
n = len(LINE)
worst = []
for i in range(n):
    a, b, c = LINE[(i - 4) % n], LINE[i], LINE[(i + 4) % n]
    v1 = (b[0] - a[0], b[1] - a[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    cr = v1[0] * v2[1] - v1[1] * v2[0]
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    l3 = math.hypot(c[0] - a[0], c[1] - a[1])
    if l1 * l2 * l3 < 1e-9:
        continue
    kappa = 2.0 * cr / (l1 * l2 * l3)
    worst.append((abs(kappa) * LINE[i][2] ** 2, i, abs(kappa), LINE[i][2]))
worst.sort(reverse=True)
for a, i, kp, v in worst[:6]:
    print(
        f"  wp{i:4d} ({LINE[i][0]:6.2f},{LINE[i][1]:5.2f}) kappa={kp:5.2f} "
        f"R={1/max(kp,1e-6):5.2f}m v={v:4.2f} -> a_lat={a:5.2f}"
    )
print(f"  최대 요구 a_lat = {worst[0][0]:.2f} m/s^2")
