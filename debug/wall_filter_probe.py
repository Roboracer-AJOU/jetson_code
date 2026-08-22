#!/usr/bin/env python3
"""맵 벽 필터가 스캔을 얼마나 먹는지 잰다.

integrated_obstacle_node 는 클러스터링 **전에** 맵에서 벽으로 판정된 점을
통째로 지운다 (`obs_mask = ~wall_hit`). 맵이 어긋나 있거나 측위가 밀리면
여기서 진짜 장애물까지 지워지고, 그러면 회피는 시작조차 못 한다 — AEB 만
남는다.

노드와 **같은 맵·같은 TF** 로 같은 계산을 돌려서, 살아남는 점이 몇 %인지와
그 점들이 어디 있는지 본다.

    python3 debug/wall_filter_probe.py --sec 20
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "path_following"))
from path_following.integrated_obstacle_node import CFG as OBS_CFG  # noqa: E402
from path_following.static_obstacle_node import StaticMap, resolve_map_yaml  # noqa: E402

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class WallProbe(Node):
    def __init__(self, dur: float):
        super().__init__("wall_filter_probe")
        yaml_path = resolve_map_yaml(OBS_CFG["map_name"], OBS_CFG["map_dir"])
        self.map = StaticMap(yaml_path, float(OBS_CFG["wall_match_radius_m"]))
        self.get_logger().info(
            f"맵 {Path(yaml_path).name}  팽창 {OBS_CFG['wall_match_radius_m']} m"
        )
        self.max_r = float(OBS_CFG["max_obstacle_range_m"])
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.t0 = time.monotonic()
        self.dur = dur
        self.n = 0
        self.stats = []
        self.pub_n = 0          # /static_obstacles 가 낸 클러스터 수 (최대)
        self.pub_now = 0        # 같은 것 (직전 프레임)
        self.pub_frames = 0
        self.create_subscription(LaserScan, "/scan", self._scan, SENSOR_QOS)
        from std_msgs.msg import Float32MultiArray

        self.create_subscription(
            Float32MultiArray, "/static_obstacles", self._obs, 10
        )

    def _obs(self, m):
        """노드가 최종적으로 낸 결과. 생존점은 있는데 여기가 0 이면 클러스터
        문턱에서 떨어진 것이고, 생존점부터 0 이면 맵 필터가 지운 것이다."""
        self.pub_frames += 1
        self.pub_now = len(m.data) // 4
        self.pub_n = max(self.pub_n, self.pub_now)

    def _scan(self, msg):
        if time.monotonic() - self.t0 > self.dur:
            self._report()
            raise SystemExit(0)
        try:
            tf = self.buf.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time()
            )
        except TransformException as e:
            if self.n == 0:
                self.get_logger().warn(f"TF 없음: {e}")
            return

        r = np.asarray(msg.ranges, dtype=np.float64)
        idx = np.arange(r.size, dtype=np.float64)
        ok = np.isfinite(r) & (r > 0.05) & (r < float(msg.range_max))
        r2 = r[ok]
        th = float(msg.angle_min) + idx[ok] * float(msg.angle_increment)
        near = r2 <= self.max_r
        r2, th = r2[near], th[near]
        if r2.size == 0:
            return
        lx, ly = r2 * np.cos(th), r2 * np.sin(th)

        t = tf.transform.translation
        q = tf.transform.rotation
        s = 2.0 * (q.w * q.z + q.x * q.y)
        c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        mx = c * lx - s * ly + t.x
        my = s * lx + c * ly + t.y

        wall = self.map.is_wall(mx, my)
        keep = ~wall
        nk = int(keep.sum())
        self.stats.append((r2.size, nk, float(r2[keep].max()) if nk else 0.0))
        self.n += 1
        if self.n % 40 != 0:
            return
        tot, k, far = self.stats[-1]
        # 0.5 m 밖에서 살아남은 점들 — 진짜 장애물 후보다. 이게 몇 개인지가
        # 곧 클러스터 최소점수를 넘느냐를 가른다.
        sel = keep & (r2 > 0.5)
        t = time.monotonic() - self.t0
        if not int(sel.sum()):
            self.get_logger().info(
                f"[{t:5.1f}s] 지워짐 {100*(tot-k)/tot:5.1f}%  "
                f"0.5m밖 생존점 0개  →  /static_obstacles {self.pub_now}개"
            )
            return
        rr, tt = r2[sel], np.degrees(th[sel])
        need = max(
            int(OBS_CFG["min_cluster_points_floor"]),
            int(np.ceil(float(OBS_CFG["min_arc_m"])
                        / max(float(np.median(rr))
                              * float(msg.angle_increment), 1e-9))),
        )
        span = float(rr.max() * np.radians(tt.max() - tt.min()))
        self.get_logger().info(
            f"[{t:5.1f}s] 지워짐 {100*(tot-k)/tot:5.1f}%  "
            f"0.5m밖 생존점 {int(sel.sum()):3d}개 "
            f"({rr.min():.2f}~{rr.max():.2f}m, 폭 {span:.2f}m, 최소 {need}개 필요)"
            f"  →  /static_obstacles {self.pub_now}개"
        )

    def _report(self):
        if not self.stats:
            print("샘플 없음")
            return
        a = np.array(self.stats, dtype=float)
        tot, keep, far = a[:, 0], a[:, 1], a[:, 2]
        erased = 100.0 * (tot - keep) / np.maximum(tot, 1)
        print(f"\n스캔 {len(a)} 장")
        print(f"  스캔당 유효점      중앙 {np.median(tot):.0f}")
        print(f"  벽으로 지워진 비율  중앙 {np.median(erased):.1f}%  "
              f"최소 {erased.min():.1f}%  최대 {erased.max():.1f}%")
        print(f"  살아남은 점 수      중앙 {np.median(keep):.0f}  최대 {keep.max():.0f}")
        print(f"  살아남은 최원거리    중앙 {np.median(far):.2f} m  "
              f"95% {np.percentile(far, 95):.2f} m  최대 {far.max():.2f} m")
        print(f"  /static_obstacles 최대 클러스터 수  {self.pub_n} "
              f"({self.pub_frames} 프레임 수신)")
        far_keep = np.median(far)
        if far_keep < 1.0:
            print("\n  판정: 0.5 m 밖에 살아남는 점이 거의 없다 — 맵 필터(팽창 "
                  f"{OBS_CFG['wall_match_radius_m']} m)가 장애물까지 지우고 있다.")
        elif self.pub_n == 0:
            print("\n  판정: 점은 살아남는데 /static_obstacles 가 비었다 — "
                  "클러스터 최소점수/최소폭 문턱에서 떨어진다.")
        else:
            print(f"\n  판정: 검출까지 정상 ({self.pub_n}개). "
                  "늦음의 원인은 검출부가 아니다.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    n = WallProbe(a.sec)
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
