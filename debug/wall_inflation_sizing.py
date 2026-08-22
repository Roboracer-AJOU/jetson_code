#!/usr/bin/env python3
"""벽 팽창이 장애물을 얼마나 먹는지 **거리로** 잰다.

`is_wall` 은 맵 점유셀을 `wall_match_radius_m` 만큼 부풀린 밴드에 들어간
스캔 점을 지운다. 벽에 붙은 박스는 이 밴드에 걸려 사라지는데, 얼마를
줄이면 살아나는지는 "박스 점이 원래 벽에서 몇 m 떨어져 있나" 를 봐야 안다.

그래서 **팽창 전** 점유셀까지의 거리를 재고, 0.5 m 밖 스캔 점들이 그 거리
어디에 몰려 있는지 히스토그램으로 본다. 박스가 만드는 봉우리가 현재 팽창
반경보다 안쪽이면 그게 지워지는 이유다.

    python3 debug/wall_inflation_sizing.py --sec 25
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "path_following"))
from path_following.integrated_obstacle_node import CFG as OBS_CFG  # noqa: E402
from path_following.static_obstacle_node import resolve_map_yaml  # noqa: E402

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
BINS = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.60, 1e9]


class Sizing(Node):
    def __init__(self, dur: float):
        super().__init__("wall_inflation_sizing")
        yp = Path(resolve_map_yaml(OBS_CFG["map_name"], OBS_CFG["map_dir"]))
        meta = yaml.safe_load(yp.open())
        img = Path(meta["image"])
        if not img.is_absolute():
            img = yp.parent / img
        gray = np.asarray(Image.open(img).convert("L"), dtype=np.float64)
        occ = (gray / 255.0 if int(meta.get("negate", 0)) else (255.0 - gray) / 255.0)
        occupied = occ >= float(meta.get("occupied_thresh", 0.65))

        from scipy.ndimage import distance_transform_edt

        self.res = float(meta["resolution"])
        # 팽창 **전** 점유셀까지의 거리 [m]
        self.dist = distance_transform_edt(~occupied).astype(np.float32) * self.res
        self.h, self.w = occupied.shape
        self.ox = float(meta["origin"][0])
        self.oy = float(meta["origin"][1])
        self.infl = float(OBS_CFG["wall_match_radius_m"])
        self.get_logger().info(
            f"맵 {yp.name}  해상도 {self.res} m  현재 팽창 {self.infl} m"
        )

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.t0 = time.monotonic()
        self.dur = dur
        self.hist = np.zeros(len(BINS) - 1, dtype=np.int64)
        self.frames = 0
        self.box = []
        self.create_subscription(LaserScan, "/scan", self._scan, SENSOR_QOS)

    def _wall_dist(self, x, y):
        col = np.floor((x - self.ox) / self.res).astype(np.int64)
        row = np.floor((self.oy + self.h * self.res - y) / self.res).astype(np.int64)
        ok = (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)
        out = np.zeros(x.shape, dtype=np.float32)
        out[ok] = self.dist[row[ok], col[ok]]
        return out, ok

    def _scan(self, msg):
        if time.monotonic() - self.t0 > self.dur:
            self._report()
            raise SystemExit(0)
        try:
            tf = self.buf.lookup_transform("map", msg.header.frame_id, rclpy.time.Time())
        except TransformException:
            return
        r = np.asarray(msg.ranges, dtype=np.float64)
        i = np.arange(r.size, dtype=np.float64)
        ok = np.isfinite(r) & (r > 0.5) & (r <= float(OBS_CFG["max_obstacle_range_m"]))
        if not np.any(ok):
            return
        rr = r[ok]
        th = float(msg.angle_min) + i[ok] * float(msg.angle_increment)
        lx, ly = rr * np.cos(th), rr * np.sin(th)
        t, q = tf.transform.translation, tf.transform.rotation
        s = 2.0 * (q.w * q.z + q.x * q.y)
        c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        mx = c * lx - s * ly + t.x
        my = s * lx + c * ly + t.y
        d, inside = self._wall_dist(mx, my)
        self.hist += np.histogram(d[inside], bins=BINS)[0]
        self.frames += 1
        self._find_box(lx, ly, rr, d)
        if self.frames % 40 == 0:
            keep = int((d[inside] >= self.infl).sum())
            band = int(((d[inside] > 0.02) & (d[inside] < self.infl)).sum())
            msg_box = ""
            if self.box:
                b = self.box[-1]
                msg_box = (
                    f"  |  박스후보 {b[0]}점 폭 {b[1]:.2f}m "
                    f"거리 {b[2]:.2f}m 벽까지 {b[3]:.2f}~{b[4]:.2f}m"
                )
            self.get_logger().info(
                f"[{time.monotonic()-self.t0:5.1f}s] 0.5m밖 {int(inside.sum())}점 — "
                f"밴드에서 지워짐 {band}, 살아남음 {keep}{msg_box}"
            )

    def _find_box(self, lx, ly, rr, d):
        """벽에 딱 붙지 않은 점들 중 **덩어리** 를 찾는다.

        히스토그램만으로는 밴드 안의 점이 박스인지 벽 잔차인지 못 가른다.
        노드와 같은 기준(연속점 간격 0.28 m)으로 묶어서, 장애물 크기 창에
        들어오는 덩어리만 박스 후보로 본다.
        """
        sel = d > 0.05  # 벽면 자체는 뺀다
        if int(sel.sum()) < 3:
            return
        px, py, pr, pd = lx[sel], ly[sel], rr[sel], d[sel]
        gap = np.hypot(np.diff(px), np.diff(py))
        cut = np.where(gap > float(OBS_CFG["cluster_gap_threshold_m"]))[0] + 1
        best = None
        for a, b in zip(np.r_[0, cut], np.r_[cut, len(px)]):
            n = b - a
            if n < 3:
                continue
            span = float(math.hypot(px[b - 1] - px[a], py[b - 1] - py[a]))
            if not (float(OBS_CFG["min_obstacle_size_m"]) <= span
                    <= float(OBS_CFG["max_obstacle_size_m"])):
                continue
            cand = (n, span, float(pr[a:b].mean()),
                    float(pd[a:b].min()), float(pd[a:b].max()))
            if best is None or cand[0] > best[0]:
                best = cand
        if best:
            self.box.append(best)

    def _report(self):
        if not self.frames:
            print("샘플 없음")
            return
        tot = self.hist.sum()
        print(f"\n스캔 {self.frames} 장, 0.5 m 밖 점 {tot}개")
        print(f"현재 팽창 {self.infl} m — 이 안쪽은 전부 지워진다\n")
        print("  맵 벽까지 거리      점 수    비율")
        acc = 0
        for k in range(len(BINS) - 1):
            lo, hi = BINS[k], BINS[k + 1]
            n = int(self.hist[k])
            acc += n
            label = f"{lo:.2f}~{hi:.2f} m" if hi < 1e8 else f"{lo:.2f} m 이상"
            mark = "  ← 지워짐" if hi <= self.infl + 1e-9 else ""
            print(f"  {label:>16}  {n:7d}  {100*n/max(tot,1):5.1f}%{mark}")
        # 팽창을 줄이면 몇 점이 살아나나
        print("\n  팽창을 줄였을 때 추가로 살아나는 점 (스캔당)")
        for cand in (0.20, 0.15, 0.12, 0.10):
            if cand >= self.infl:
                continue
            gain = int(
                self.hist[
                    [k for k in range(len(BINS) - 1)
                     if BINS[k] >= cand and BINS[k + 1] <= self.infl + 1e-9]
                ].sum()
            )
            print(f"    {cand:.2f} m → 스캔당 +{gain/self.frames:.1f}점")

        if not self.box:
            print("\n  박스 후보 덩어리를 못 찾았다 — 앞에 물체가 없거나 완전히 벽에 묻혔다.")
            return
        b = np.array(self.box, dtype=float)
        print(f"\n  박스 후보 (스캔 {len(b)}/{self.frames} 장에서 발견)")
        print(f"    점 수      중앙 {np.median(b[:,0]):.0f}")
        print(f"    폭         중앙 {np.median(b[:,1]):.2f} m")
        print(f"    거리       중앙 {np.median(b[:,2]):.2f} m")
        print(f"    맵 벽까지   가장 가까운 점 중앙 {np.median(b[:,3]):.2f} m, "
              f"가장 먼 점 중앙 {np.median(b[:,4]):.2f} m")
        near = np.median(b[:, 3])
        far_p = np.median(b[:, 4])
        if far_p < self.infl:
            print(f"\n    → 박스가 통째로 팽창 밴드({self.infl} m) 안이다. "
                  f"팽창을 {far_p:.2f} m 밑으로 줘야 살아난다.")
        elif near < self.infl:
            print(f"\n    → 박스의 벽쪽 절반이 밴드에 먹힌다 "
                  f"({near:.2f} m 부터 {self.infl} m 까지). "
                  "남는 점으로는 클러스터가 안 선다.")
        else:
            print("\n    → 이 박스는 밴드 밖이다 — 팽창은 원인이 아니다.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=25.0)
    a = ap.parse_args()
    rclpy.init()
    n = Sizing(a.sec)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n._report()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
