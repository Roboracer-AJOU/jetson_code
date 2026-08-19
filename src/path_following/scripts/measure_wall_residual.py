#!/usr/bin/env python3
"""wall_match_radius_m 의 마지노선을 실측한다.

빈 트랙을 레이스 페이스로 한 바퀴 돌면, 모든 스캔점은 정의상 '벽'이다.
그 점들이 맵의 벽 셀에서 얼마나 떨어져 나타나는지가 곧 정합 잔차이고,
그 분포의 꼬리가 팽창 반경이 덮어야 하는 값이다. 예산 계산 대신 이걸 쓴다.

    # 트랙을 비우고, 로컬라이제이션 스택을 띄운 상태에서
    python3 src/path_following/scripts/measure_wall_residual.py

    Ctrl-C 로 종료하면 백분위와 권장값을 출력한다.

주의: 트랙에 실제 장애물이 있으면 그 점들이 큰 잔차로 섞여 꼬리를 부풀린다.
반드시 빈 트랙에서 측정할 것. 히스토그램이 두 덩어리로 갈라져 보이면
장애물이 섞인 것이다.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.static_obstacle_node import StaticMap, resolve_map_yaml  # noqa: E402

# 이 백분위까지 덮으면 벽 잔차가 장애물로 새지 않는다고 본다.
REPORT_PCTS = (50.0, 90.0, 99.0, 99.5, 99.9)
RECOMMEND_PCT = 99.5
MAP_RESOLUTION_ROUND = 0.05  # 팽창은 ceil(r/res) 셀이라 격자 배수로만 의미가 있다


class ResidualMeter(Node):
    def __init__(self, map_name: str, map_dir: str, scan_topic: str, laser_frame: str):
        super().__init__("measure_wall_residual")

        # 팽창 0 → wall_distance() 가 '원본 벽 셀까지의 거리' 를 준다.
        self.map = StaticMap(resolve_map_yaml(map_name, map_dir), 0.0)
        self.laser_frame = laser_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            QoSProfile(
                depth=5,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
        )

        self._res: list[np.ndarray] = []
        self._scans = 0
        self._tf_miss = 0
        self._last_report_ns = 0

        self.get_logger().info(
            f"맵={Path(self.map.yaml_path).name} res={self.map.resolution:.3f}m "
            f"| {scan_topic} 구독 — 빈 트랙을 레이스 페이스로 주행할 것"
        )

    def _pose_at(self, stamp):
        for t in (stamp, rclpy.time.Time()):
            try:
                return self.tf_buffer.lookup_transform(
                    "map",
                    self.laser_frame,
                    t,
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
            except TransformException:
                continue
        return None

    def _on_scan(self, msg: LaserScan) -> None:
        tf = self._pose_at(msg.header.stamp)
        if tf is None:
            self._tf_miss += 1
            return

        rng = np.asarray(msg.ranges, dtype=np.float64)
        idx = np.arange(rng.size, dtype=np.float64)
        ok = np.isfinite(rng) & (rng > 0.05) & (rng < float(msg.range_max))
        if not np.any(ok):
            return
        r = rng[ok]
        th = msg.angle_min + idx[ok] * msg.angle_increment

        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = r * np.cos(th), r * np.sin(th)
        mx = c * lx - s * ly + tf.transform.translation.x
        my = s * lx + c * ly + tf.transform.translation.y

        self._res.append(self.map.wall_distance(mx, my).astype(np.float32))
        self._scans += 1

        now = self.get_clock().now().nanoseconds
        if now - self._last_report_ns > 5_000_000_000:
            self._last_report_ns = now
            d = np.concatenate(self._res)
            self.get_logger().info(
                f"{self._scans} 스캔 / {d.size} 점 | "
                f"p99={np.percentile(d, 99):.3f}m p99.9={np.percentile(d, 99.9):.3f}m"
            )

    def report(self) -> None:
        if not self._res:
            print("\n측정된 점이 없다. TF 또는 /scan 을 확인할 것.")
            if self._tf_miss:
                print(f"  TF 조회 실패 {self._tf_miss}회")
            return

        d = np.concatenate(self._res)
        print(f"\n{'='*58}")
        print(f"벽 정합 잔차 — {self._scans} 스캔, {d.size} 점")
        if self._tf_miss:
            print(f"TF 조회 실패 {self._tf_miss} 스캔 (제외됨)")
        print("=" * 58)
        for p in REPORT_PCTS:
            print(f"  p{p:<5} {np.percentile(d, p):.3f} m")
        print(f"  최대   {d.max():.3f} m")

        print("\n분포 (장애물이 섞이면 두 덩어리로 갈라진다)")
        edges = np.arange(0.0, 0.65, 0.05)
        hist, _ = np.histogram(d, bins=np.append(edges, 1e9))
        for i, lo in enumerate(edges):
            share = 100.0 * hist[i] / d.size
            label = f"{lo:.2f}~{lo+0.05:.2f}" if i < len(edges) - 1 else f"{lo:.2f}+"
            print(f"  {label:>10}m {'#' * int(share / 2):<50}{share:5.1f}%")

        need = float(np.percentile(d, RECOMMEND_PCT))
        cells = max(1, math.ceil(need / MAP_RESOLUTION_ROUND))
        rec = cells * MAP_RESOLUTION_ROUND
        print(f"\np{RECOMMEND_PCT} = {need:.3f}m → 격자 올림 {cells}셀")
        print(f"권장 wall_match_radius_m = {rec:.2f}")
        print(
            "이보다 낮추면 벽 잔차가 클러스터로 살아남아 FGM 오작동으로 이어진다.\n"
            "near_wall 가드(점수·span)가 2차 방어선이지만 그걸 믿고 낮추진 말 것."
        )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--map-name", default="cartographer_map_20260818_204522_rosmap.yaml"
    )
    ap.add_argument("--map-dir", default="")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--laser-frame", default="laser")
    args = ap.parse_args()

    rclpy.init()
    node = ResidualMeter(
        args.map_name, args.map_dir, args.scan_topic, args.laser_frame
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
