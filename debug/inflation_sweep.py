#!/usr/bin/env python3
"""팽창 반경을 바꿔 가며 **실제 검출 파이프라인** 을 그대로 돌려 본다.

벽에 붙은 박스가 안 잡히는 건 두 관문에 걸리기 때문이다.

  1. 맵 팽창(`wall_match_radius_m` 0.25 m) 이 박스의 벽쪽 절반을 지운다
  2. 남은 조각이 `min_obstacle_size_m`(0.12) 과 근접벽 가드
     (`near_wall_min_points` 14, `near_wall_min_span_m` 0.20) 에 또 걸린다

팽창만 낮추면 벽 잔차가 장애물로 샌다. 그 대가가 얼마인지 **세어 봐야**
값을 고를 수 있다. 그래서 후보 팽창값마다 노드와 같은 클러스터링·가드를
돌려서, 박스를 잡는지와 그 대신 유령이 몇 개 생기는지를 같이 센다.

박스는 "앞쪽(|각| < 60°), 사거리 안, 크기 창 안" 인 클러스터로 본다. 그
밖에 잡히는 건 전부 유령으로 센다 — 트랙에 박스 하나만 두고 재는 전제다.

    python3 debug/inflation_sweep.py --sec 25
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
from path_following.integrated_obstacle_node import CFG as C  # noqa: E402
from path_following.scan_cluster import ClusterParams, cluster_scan_xy  # noqa: E402
from path_following.static_obstacle_node import (  # noqa: E402
    near_wall_point_gate,
    resolve_map_yaml,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
CANDIDATES = [0.25, 0.20, 0.15, 0.12, 0.10]
BOX_CONE_DEG = 60.0


def dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask.astype(bool, copy=True)
    ys, xs = np.where(mask)
    out = np.zeros_like(mask, dtype=bool)
    h, w = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            yy, xx = ys + dy, xs + dx
            ok = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
            out[yy[ok], xx[ok]] = True
    return out


class Sweep(Node):
    def __init__(self, dur: float):
        super().__init__("inflation_sweep")
        yp = Path(resolve_map_yaml(C["map_name"], C["map_dir"]))
        meta = yaml.safe_load(yp.open())
        img = Path(meta["image"])
        if not img.is_absolute():
            img = yp.parent / img
        gray = np.asarray(Image.open(img).convert("L"), dtype=np.float64)
        occ = gray / 255.0 if int(meta.get("negate", 0)) else (255.0 - gray) / 255.0
        occupied = occ >= float(meta.get("occupied_thresh", 0.65))

        from scipy.ndimage import distance_transform_edt

        self.res = float(meta["resolution"])
        self.h, self.w = occupied.shape
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])

        self.walls, self.wdist = {}, {}
        for r in CANDIDATES:
            g = dilate(occupied, int(math.ceil(r / self.res)))
            self.walls[r] = g
            self.wdist[r] = distance_transform_edt(~g).astype(np.float32) * self.res
        self.get_logger().info(
            f"맵 {yp.name} 해상도 {self.res} m — 팽창 {CANDIDATES} 준비"
        )

        self.params = ClusterParams(
            mode=str(C["cluster_mode"]).strip().lower(),
            gap_threshold_m=float(C["cluster_gap_threshold_m"]),
            lambda_deg=float(C["abd_lambda_deg"]),
            sigma_r_m=float(C["abd_sigma_r_m"]),
            min_gap_m=float(C["abd_min_gap_m"]),
            max_gap_m=float(C["abd_max_gap_m"]),
            min_points=int(C["min_cluster_points"]),
            adaptive_min_points=bool(C["adaptive_min_points"]),
            min_points_floor=int(C["min_cluster_points_floor"]),
            min_arc_m=float(C["min_arc_m"]),
        )
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.t0, self.dur = time.monotonic(), dur
        self.frames = 0
        self.box_hit = {r: 0 for r in CANDIDATES}
        self.ghosts = {r: 0 for r in CANDIDATES}
        self.box_span = {r: [] for r in CANDIDATES}
        self.create_subscription(LaserScan, "/scan", self._scan, SENSOR_QOS)

    def _cells(self, x, y):
        col = np.floor((x - self.ox) / self.res).astype(np.int64)
        row = np.floor((self.oy + self.h * self.res - y) / self.res).astype(np.int64)
        ok = (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)
        return row, col, ok

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
        inc = float(msg.angle_increment)
        ok = np.isfinite(r) & (r > 0.05) & (r <= float(C["max_obstacle_range_m"]))
        if not np.any(ok):
            return
        rr = r[ok]
        th = float(msg.angle_min) + i[ok] * inc
        lx, ly = rr * np.cos(th), rr * np.sin(th)
        t, q = tf.transform.translation, tf.transform.rotation
        s = 2.0 * (q.w * q.z + q.x * q.y)
        c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        mx, my = c * lx - s * ly + t.x, s * lx + c * ly + t.y
        row, col, inside = self._cells(mx, my)
        self.frames += 1

        for cand in CANDIDATES:
            wall = np.ones(rr.shape, dtype=bool)
            wall[inside] = self.walls[cand][row[inside], col[inside]]
            keep = ~wall
            if not np.any(keep):
                continue
            cls = cluster_scan_xy(
                lx[keep], ly[keep],
                angle_increment=inc,
                params=self.params,
                radius_percentile=float(C["radius_percentile"]),
                radius_min_m=float(C["radius_min_m"]),
                radius_max_m=float(C["max_obstacle_size_m"]) / 2.0,
                consistent_centroid=bool(C["consistent_centroid"]),
            )
            if not cls:
                continue
            cx = np.array([o.center_x for o in cls])
            cy = np.array([o.center_y for o in cls])
            gmx, gmy = c * cx - s * cy + t.x, s * cx + c * cy + t.y
            grow, gcol, gin = self._cells(gmx, gmy)
            wd = np.zeros(cx.shape, dtype=np.float32)
            wd[gin] = self.wdist[cand][grow[gin], gcol[gin]]

            box, ghost = False, 0
            for k, o in enumerate(cls):
                if not (float(C["min_obstacle_size_m"]) <= o.span_m
                        <= float(C["max_obstacle_size_m"])):
                    continue
                if abs(o.near_y) > float(C["max_obstacle_lateral_m"]):
                    continue
                gate = near_wall_point_gate(
                    int(C["near_wall_min_points"]),
                    float(C["near_wall_min_span_m"]),
                    math.hypot(o.center_x, o.center_y),
                    inc,
                )
                if wd[k] < float(C["wall_clearance_m"]) and (
                    o.n_points < gate or o.span_m < float(C["near_wall_min_span_m"])
                ):
                    continue
                ang = abs(math.degrees(math.atan2(o.center_y, o.center_x)))
                if ang <= BOX_CONE_DEG and o.center_x > 0.5:
                    box = True
                    self.box_span[cand].append(o.span_m)
                else:
                    ghost += 1
            self.box_hit[cand] += int(box)
            self.ghosts[cand] += ghost

    def _report(self):
        if not self.frames:
            print("샘플 없음")
            return
        print(f"\n스캔 {self.frames} 장  (현재 설정 = 팽창 {C['wall_match_radius_m']} m)\n")
        print("  팽창    앞쪽 물체 검출률   유령/스캔   검출된 폭(중앙)")
        for cand in CANDIDATES:
            hit = 100.0 * self.box_hit[cand] / self.frames
            gh = self.ghosts[cand] / self.frames
            sp = self.box_span[cand]
            spm = f"{np.median(sp):.2f} m" if sp else "—"
            cur = "  ← 지금" if abs(cand - float(C["wall_match_radius_m"])) < 1e-9 else ""
            print(f"  {cand:.2f} m   {hit:6.1f}%          {gh:5.2f}      {spm:>8}{cur}")
        print("\n  '유령' = 앞쪽 콘 밖에서 잡힌 클러스터. 트랙에 박스 하나만"
              " 뒀다는 전제라, 이게 늘면 벽 잔차가 새는 것이다.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=25.0)
    a = ap.parse_args()
    rclpy.init()
    n = Sweep(a.sec)
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
