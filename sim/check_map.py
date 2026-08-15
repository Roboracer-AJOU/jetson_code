"""raceline/centerline 이 로컬라이제이션 맵의 free space 안에 있는지 확인."""
import sys

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/sim")
import numpy as np
from race_sim import MAP_YAML as MAP
from race_sim import GridMap

m = GridMap(MAP)
print(f"map {m.w}x{m.h} res={m.res} origin=({m.ox:.2f},{m.oy:.2f})")
print(f"  free={m.free.sum()}px wall={m.wall.sum()}px unknown={(~m.free & ~m.wall).sum()}px")

for name in ("raceline", "centerline"):
    p = f"/home/nvidia/f1tenth_ajou/src/path_following/config/{name}.csv"
    pts = []
    with open(p) as f:
        for line in f:
            line = line.strip()
            if not line or line[0] in "#xX":
                continue
            c = line.split(",")
            pts.append((float(c[0]), float(c[1]), float(c[2]) if len(c) > 2 else 0.0))
    xs = np.array([q[0] for q in pts])
    ys = np.array([q[1] for q in pts])
    vs = np.array([q[2] for q in pts])
    clear = np.array([m.wall_clearance(x, y) for x, y in zip(xs, ys)])
    nwall = sum(1 for x, y in zip(xs, ys) if m.is_wall(x, y))
    print(
        f"\n{name}: n={len(pts)} x[{xs.min():.1f},{xs.max():.1f}] "
        f"y[{ys.min():.1f},{ys.max():.1f}]"
    )
    print(f"  v[{vs.min():.2f},{vs.max():.2f}] 평균 {vs.mean():.2f} m/s")
    print(f"  벽 위 점 개수 = {nwall}")
    print(
        f"  벽 여유: 최소 {clear.min():.3f} m, 5% {np.percentile(clear,5):.3f} m, "
        f"중앙 {np.median(clear):.3f} m"
    )
    tight = (clear < 0.30).sum()
    print(f"  여유 0.30m 미만 점 = {tight} ({100*tight/len(pts):.1f}%)")
