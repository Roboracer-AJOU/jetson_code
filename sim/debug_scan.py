"""시작 포즈에서의 스캔을 뜯어본다. AEB 오검 원인 추적용."""
import math
import sys

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/sim")
import numpy as np
from race_sim import LASER_OFFSET_X, GridMap, RayCaster
from run_scenario import LINE, pose_at

MAP = "/home/nvidia/f1tenth_ajou/maps/cartographer_map_20260814_232850_rosmap.yaml"
m = GridMap(MAP)
rc = RayCaster(m)

for idx in (0, 100, 300, 500):
    x, y, yaw = pose_at(idx)
    lx = x + LASER_OFFSET_X * math.cos(yaw)
    ly = y + LASER_OFFSET_X * math.sin(yaw)
    r = rc.scan(lx, ly, yaw, [])
    ang = rc.angles
    # AEB 코리도: |lateral| <= 0.22, 전방 50deg 이내
    lat = r * np.sin(ang)
    fwd = r * np.cos(ang)
    inb = (np.abs(ang) <= math.radians(50)) & (np.abs(lat) <= 0.22) & (fwd > 0)
    n_in = int(inb.sum())
    closest = float(r[inb].min()) if n_in else float("nan")
    print(
        f"wp{idx:4d} pose=({x:6.2f},{y:5.2f}) yaw={math.degrees(yaw):6.1f}deg "
        f"clear={m.wall_clearance(x,y):.2f}m | 정면 r={r[len(r)//2]:5.2f} "
        f"min={r.min():.2f} | 코리도 beams={n_in:3d} closest={closest:.2f}"
    )

# 정면 30도 범위 프로파일
x, y, yaw = pose_at(0)
lx = x + LASER_OFFSET_X * math.cos(yaw)
ly = y + LASER_OFFSET_X * math.sin(yaw)
r = rc.scan(lx, ly, yaw, [])
print("\nwp0 정면 ±30° 스캔:")
c = len(r) // 2
for k in range(-60, 61, 10):
    i = c + k
    print(f"  {math.degrees(rc.angles[i]):6.1f}deg -> {r[i]:5.2f} m")
