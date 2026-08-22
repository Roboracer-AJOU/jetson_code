#!/usr/bin/env python3
"""회피가 한 박자 늦는 지점을 찾는 계측기.

체인의 각 단계가 **장애물까지 몇 m 남았을 때** 반응했는지 기록한다.

    scan → integrated_obstacle_node → /static_obstacles
         → local_planner (코리도 필터 · 게이트 · 모드)
         → /planner/fgm_enable → fgm_node → /fgm_target
         → /local_path → stanley → /drive

늦는 원인은 크게 셋인데 셋이 서로 다른 흔적을 남긴다.

  1. **안 보인다** — /static_obstacles 에 늦게 뜬다 (검출·M-of-N 확인 지연)
  2. **보이는데 무시한다** — 떠 있는데 mode 가 GLOBAL 이다 (코리도 필터가
     떨궜거나 게이트가 안 열렸다). 이러면 AEB 가 먼저 터진다.
  3. **정했는데 늦게 나간다** — mode 는 AVOID 인데 /local_path 나 조향이
     늦다 (FGM 목표 대기, 경로 막힘, 발행 주기)

그래서 세 지점의 **거리** 를 각각 남긴다. 시간만 재면 어느 쪽인지 못 가른다.

    python3 debug/avoid_latency_probe.py            # 기본 60분
    python3 debug/avoid_latency_probe.py --sec 300
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as PathMsg
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

# local_planner 의 검출 게이트와 **같은 값이어야 한다.** 여기가 좁으면
# planner 는 이미 반응했는데 계측기에는 장애물이 안 보여서, 없는 지연을
# 있다고 읽는다 (처음에 |y|≤0.42·콘 35° 로 재다가 그렇게 속았다. planner 가
# AVOID 로 바뀌고 0.15 s 뒤에야 계측기에 떴다).
#
#   obstacle_forward_min_m / _max_m        0.30 / 12.0
#   obstacle_lateral_abs_max_corridor_m    1.50  (코리도 켜졌을 때 쓰는 값)
#   forward_cone_deg                       75
FORWARD_MIN_M = 0.30
FORWARD_MAX_M = 12.0
LATERAL_ABS_MAX_M = 1.50
FORWARD_CONE_RAD = math.radians(75.0)

# 코리도(레이스라인 기준 0.18 m)는 TF·트랙점이 있어야 해서 못 흉내낸다.
# 대신 게이트를 아예 안 건 "원시 검출" 을 따로 남긴다. 원시에는 떠 있는데
# 게이트 통과분이 없으면 그 사이에서 걸러진 것이고, 게이트는 통과했는데
# planner 가 안 움직이면 코리도가 떨군 것이다.
RAW_MIN_X_M = 0.20

BEST_EFFORT = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
ROSOUT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,
)

COLS = [
    "t",             # 시작 이후 [s]
    "v",             # 차속 [m/s]
    "obs_n",         # /static_obstacles 클러스터 수 (전방 게이트 통과분)
    "obs_d",         # 가장 가까운 전방 장애물 표면거리 [m], 없으면 빈칸
    "obs_x",
    "obs_y",
    "obs_r",
    "raw_n",         # 게이트 없이 전방(x>0.2) 검출 수
    "raw_d",         # 그중 최근접 [m] — 검출이 정말 늦는지 보는 값
    "raw_far",       # 그중 최원거리 [m] — 실효 검출 범위
    "dyn_d",
    "mode",
    "scale",
    "reason",
    "fgm_en",
    "fgm_ang",       # FGM 조준각 [deg]
    "path_n",        # /local_path 점 수 (0 = 미발행)
    "path_age",      # 마지막 /local_path 이후 경과 [s]
    "override",
    "aeb",
    "ttc",
    "steer",         # /drive 조향 [deg]
    "cmd_v",         # /drive 속도 [m/s]
    "scan_age",      # scan 헤더시각 → 지금 [s] (파이프라인 신선도)
    "obs_age",       # 마지막 /static_obstacles 이후 경과 [s]
]


class Probe(Node):
    def __init__(self, out: Path, dur: float):
        super().__init__("avoid_latency_probe")
        self._t0 = time.monotonic()
        self._dur = dur
        self._fh = out.open("w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(COLS)
        self._log_fh = out.with_suffix(".log").open("w")

        self.v = 0.0
        self.obs = []           # [(d, x, y, r)] 전방 게이트 통과분, 가까운 순
        self.raw = []           # 게이트 없는 표면거리, 오름차순
        self.dyn_d = None
        self.mode = ""
        self.scale = 1.0
        self.reason = ""
        self.fgm_en = 0
        self.fgm_ang = None
        self.path_n = 0
        self.path_ns = 0.0
        self.override = 0
        self.aeb = 0
        self.ttc = None
        self.steer = 0.0
        self.cmd_v = 0.0
        self.scan_stamp = None
        self.obs_ns = 0.0
        self.rows = 0

        self.create_subscription(LaserScan, "/scan", self._scan, BEST_EFFORT)
        self.create_subscription(
            Float32MultiArray, "/static_obstacles", self._static, 10
        )
        self.create_subscription(
            Float32MultiArray, "/dynamic_obstacles", self._dynamic, 10
        )
        self.create_subscription(String, "/planner/mode", self._mode, 10)
        self.create_subscription(Float64, "/planner/speed_scale", self._scale, 10)
        self.create_subscription(String, "/planner/speed_reason", self._reason, 10)
        self.create_subscription(Bool, "/planner/fgm_enable", self._fgm, 10)
        self.create_subscription(PointStamped, "/fgm_target", self._target, 10)
        self.create_subscription(PathMsg, "/local_path", self._path, 10)
        self.create_subscription(
            Bool, "/planner_path_override_active", self._override, 10
        )
        self.create_subscription(Bool, "/emergency_brake", self._aeb, 10)
        self.create_subscription(Float64, "/emergency_brake/ttc", self._ttc, 10)
        self.create_subscription(Float64, "/vehicle/speed_mps", self._speed, 10)
        self.create_subscription(AckermannDriveStamped, "/drive", self._drive, 10)
        self.create_subscription(Log, "/rosout", self._rosout, ROSOUT_QOS)

        self.create_timer(0.02, self._tick)  # 50 Hz — 40 Hz 파이프라인보다 촘촘히
        self.get_logger().info(f"기록 시작 → {out}  ({dur:.0f}s)")

    # ---------------------------------------------------------------- 수신
    def _scan(self, m):
        self.scan_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _static(self, m):
        self.obs_ns = time.monotonic()
        d = list(m.data)
        found, raw = [], []
        for i in range(0, len(d) - 3, 4):
            x, y, r = float(d[i + 1]), float(d[i + 2]), float(d[i + 3])
            s = max(0.0, math.hypot(x, y) - abs(r))
            if x >= RAW_MIN_X_M:
                raw.append(s)
            if not (FORWARD_MIN_M <= x <= FORWARD_MAX_M):
                continue
            if abs(y) > LATERAL_ABS_MAX_M:
                continue
            if abs(math.atan2(y, max(x, 1e-6))) > FORWARD_CONE_RAD:
                continue
            found.append((s, x, y, r))
        found.sort()
        self.obs = found
        self.raw = sorted(raw)

    def _dynamic(self, m):
        d = list(m.data)
        best = None
        for i in range(0, len(d) - 5, 6):
            x, y, r = float(d[i + 1]), float(d[i + 2]), float(d[i + 5])
            if x < FORWARD_MIN_M or abs(y) > LATERAL_ABS_MAX_M:
                continue
            s = max(0.0, math.hypot(x, y) - abs(r))
            best = s if best is None else min(best, s)
        self.dyn_d = best

    def _mode(self, m):
        self.mode = m.data

    def _scale(self, m):
        self.scale = float(m.data)

    def _reason(self, m):
        self.reason = m.data

    def _fgm(self, m):
        self.fgm_en = 1 if m.data else 0

    def _target(self, m):
        self.fgm_ang = math.degrees(math.atan2(m.point.y, m.point.x))

    def _path(self, m):
        self.path_n = len(m.poses)
        self.path_ns = time.monotonic()

    def _override(self, m):
        self.override = 1 if m.data else 0

    def _aeb(self, m):
        self.aeb = 1 if m.data else 0

    def _ttc(self, m):
        self.ttc = float(m.data)

    def _speed(self, m):
        self.v = float(m.data)

    def _drive(self, m):
        self.steer = math.degrees(m.drive.steering_angle)
        self.cmd_v = float(m.drive.speed)

    def _rosout(self, m):
        if m.level < 30:  # WARN 이상만
            return
        if m.name not in (
            "local_planner_node",
            "fgm_node",
            "emergency_brake_node",
            "integrated_obstacle_node",
            "stanley_waypoint_follow_node",
        ):
            return
        t = time.monotonic() - self._t0
        near = f"{self.obs[0][0]:.2f}" if self.obs else "-"
        self._log_fh.write(
            f"{t:8.3f}  v={self.v:4.2f}  d={near:>5}  mode={self.mode:<8} "
            f"[{m.name}] {m.msg}\n"
        )
        self._log_fh.flush()

    # ---------------------------------------------------------------- 기록
    def _tick(self):
        now = time.monotonic()
        t = now - self._t0
        if t > self._dur:
            self.get_logger().info(f"기록 종료 — {self.rows} 행")
            raise SystemExit(0)

        o = self.obs[0] if self.obs else None
        scan_age = ""
        if self.scan_stamp is not None:
            wall = self.get_clock().now().nanoseconds * 1e-9
            scan_age = f"{max(0.0, wall - self.scan_stamp):.4f}"

        self._csv.writerow([
            f"{t:.3f}",
            f"{self.v:.3f}",
            len(self.obs),
            f"{o[0]:.3f}" if o else "",
            f"{o[1]:.3f}" if o else "",
            f"{o[2]:.3f}" if o else "",
            f"{o[3]:.3f}" if o else "",
            len(self.raw),
            f"{self.raw[0]:.3f}" if self.raw else "",
            f"{self.raw[-1]:.3f}" if self.raw else "",
            f"{self.dyn_d:.3f}" if self.dyn_d is not None else "",
            self.mode,
            f"{self.scale:.3f}",
            self.reason,
            self.fgm_en,
            f"{self.fgm_ang:.1f}" if self.fgm_ang is not None else "",
            self.path_n,
            f"{now - self.path_ns:.3f}" if self.path_ns else "",
            self.override,
            self.aeb,
            f"{self.ttc:.3f}" if self.ttc is not None else "",
            f"{self.steer:.2f}",
            f"{self.cmd_v:.2f}",
            scan_age,
            f"{now - self.obs_ns:.4f}" if self.obs_ns else "",
        ])
        self.rows += 1
        if self.rows % 250 == 0:  # 5초마다 flush — 중간에 죽어도 남는다
            self._fh.flush()

    def close(self):
        self._fh.flush()
        self._fh.close()
        self._log_fh.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=3600.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    out = Path(a.out) if a.out else (
        Path(__file__).resolve().parent
        / f"avoid_probe_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    )
    rclpy.init()
    n = Probe(out, a.sec)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        n.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\n저장됨: {out}\n로그:   {out.with_suffix('.log')}")


if __name__ == "__main__":
    main()
