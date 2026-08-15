#!/usr/bin/env python3
"""실행 중인 스택에서 /scan, /fgm_target, 장애물, 모드를 한 번에 떠본다 (읽기만)."""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

got = {}


def main():
    rclpy.init()
    n = rclpy.create_node("scan_diag")
    n.create_subscription(LaserScan, "/scan", lambda m: got.__setitem__("scan", m), 1)
    n.create_subscription(
        PointStamped, "/fgm_target", lambda m: got.__setitem__("fgm", m), 1
    )
    n.create_subscription(
        Float32MultiArray, "/static_obstacles", lambda m: got.__setitem__("obs", m), 1
    )
    n.create_subscription(
        Float32MultiArray, "/dynamic_obstacles", lambda m: got.__setitem__("dyn", m), 1
    )
    n.create_subscription(
        String, "/planner/mode", lambda m: got.__setitem__("mode", m.data), 1
    )
    n.create_subscription(
        Bool, "/planner/fgm_enable", lambda m: got.__setitem__("en", m.data), 1
    )

    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.1)

    print(f"planner mode = {got.get('mode')}   fgm_enable = {got.get('en')}")
    for key, stride, label in (("obs", 4, "static"), ("dyn", 6, "dynamic")):
        m = got.get(key)
        cnt = len(m.data) // stride if m else None
        print(f"  {label}_obstacles = {cnt}")

    f = got.get("fgm")
    if f is None:
        print("  /fgm_target 미수신")
    else:
        a = math.degrees(math.atan2(f.point.y, f.point.x))
        d = math.hypot(f.point.x, f.point.y)
        print(
            f"  fgm_target frame={f.header.frame_id} "
            f"x={f.point.x:+.3f} y={f.point.y:+.3f} → {a:+.1f}deg, {d:.2f}m"
        )

    s = got.get("scan")
    if s is None:
        print("\n/scan 미수신!")
        return
    r = np.array(s.ranges, dtype=float)
    ang = s.angle_min + np.arange(r.size) * s.angle_increment
    print(f"\n/scan frame={s.header.frame_id} n={r.size}")
    print(
        f"  angle: min={math.degrees(s.angle_min):+.1f} "
        f"max={math.degrees(s.angle_max):+.1f} "
        f"inc={math.degrees(s.angle_increment):.3f} deg"
    )
    print(f"  range_min={s.range_min:.3f} range_max={s.range_max:.1f}")
    fin = np.isfinite(r) & (r > 0)
    print(
        f"  유효 {int(fin.sum())}/{r.size}  inf={int(np.isinf(r).sum())} "
        f"nan={int(np.isnan(r).sum())} zero={int(np.sum(r == 0))}"
    )
    if fin.sum():
        v = r[fin]
        print(
            f"  유효 range: min={v.min():.3f} p10={np.percentile(v,10):.3f} "
            f"median={np.median(v):.3f} max={v.max():.3f}"
        )
        print(f"  0.3m 미만 빔 = {int(np.sum(v < 0.3))}개")

    print("\n  30도 섹터 최소거리 (0=정면, +=왼쪽):")
    deg = np.degrees(ang)
    for lo in range(-180, 180, 30):
        m = fin & (deg >= lo) & (deg < lo + 30)
        txt = f"{r[m].min():6.2f}m  n={int(m.sum()):4d}" if m.sum() else "    --   n=0"
        print(f"    [{lo:+4d},{lo+30:+4d}) : {txt}")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
