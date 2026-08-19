#!/usr/bin/env python3
"""FGM 회피 직후 AEB 가 걸리는 원인을 잡는 타임라인 로거.

AEB 상승엣지마다 직전 2초의 맥락(플래너 모드 전이, 속도, CTE, 전방 최근접
거리, 회피 상태)을 통째로 남긴다. AEB 는 /scan 을 직접 보고 판단하는데
플래너 모드에 따라 임계가 완화되므로, "모드가 언제 GLOBAL 로 돌아갔는지" 와
"그 순간 전방이 얼마나 가까웠는지" 의 관계가 핵심이다.

    python3 scripts/log_aeb_after_avoid.py            # Ctrl-C 로 요약
    python3 scripts/log_aeb_after_avoid.py --csv /tmp/aeb.csv

한 바퀴 돌고 Ctrl-C 하면 이벤트별 리포트와 가설 판정을 출력한다.
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import deque

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

HISTORY_SEC = 2.5      # AEB 직전 몇 초를 되짚을지
POST_SEC = 1.0         # AEB 이후 몇 초를 더 볼지
TICK_HZ = 50.0
FRONT_CONE_DEG = 20.0  # 전방 최근접 거리를 잴 콘 (AEB 가 보는 방향)

BEST_EFFORT = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class AebContextLogger(Node):
    def __init__(self, csv_path: str | None):
        super().__init__("log_aeb_after_avoid")

        self.mode = "?"
        self.aeb = False
        self.ttc = float("inf")
        self.speed = 0.0
        self.scale = 1.0
        self.cte = 0.0
        self.override = False
        self.fgm_en = False
        self.n_obs = 0
        self.front_m = float("inf")

        self._hist: deque = deque(maxlen=int(HISTORY_SEC * TICK_HZ))
        self._events: list[dict] = []
        self._pending: dict | None = None
        self._mode_changes: list[tuple[float, str, str]] = []
        self._t0 = self._now()
        self._last_beat = 0.0

        self._csv = None
        self._csv_w = None
        if csv_path:
            self._csv = open(csv_path, "w", newline="")
            self._csv_w = csv.writer(self._csv)
            self._csv_w.writerow(
                ["t", "mode", "aeb", "ttc", "speed", "scale", "cte",
                 "override", "fgm_enable", "n_obs", "front_m"]
            )

        sub = self.create_subscription
        sub(String, "/planner/mode", self._on_mode, 10)
        sub(Bool, "/emergency_brake", self._on_aeb, 10)
        sub(Float64, "/emergency_brake/ttc", lambda m: setattr(self, "ttc", m.data), 10)
        sub(Float64, "/vehicle/speed_mps", lambda m: setattr(self, "speed", m.data), 10)
        sub(Float64, "/planner/speed_scale", lambda m: setattr(self, "scale", m.data), 10)
        sub(Float64, "/control/cross_track_error", lambda m: setattr(self, "cte", m.data), 10)
        sub(Bool, "/planner_path_override_active",
            lambda m: setattr(self, "override", m.data), 10)
        sub(Bool, "/planner/fgm_enable", lambda m: setattr(self, "fgm_en", m.data), 10)
        sub(Float32MultiArray, "/static_obstacles", self._on_obs, 10)
        sub(LaserScan, "/scan", self._on_scan, BEST_EFFORT)

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            "AEB 맥락 로거 시작 — 한 바퀴 돌고 Ctrl-C. "
            f"AEB 상승엣지마다 직전 {HISTORY_SEC:.1f}s 를 남긴다"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_mode(self, msg: String) -> None:
        if msg.data != self.mode:
            self._mode_changes.append((self._now() - self._t0, self.mode, msg.data))
            self.mode = msg.data

    def _on_obs(self, msg: Float32MultiArray) -> None:
        self.n_obs = len(msg.data) // 4

    def _on_scan(self, msg: LaserScan) -> None:
        # 40Hz × 1500빔이라 파이썬 루프로 돌면 안 된다 (주행 스택과 코어를
        # 나눠 쓰는 중이다).
        r = np.asarray(msg.ranges, dtype=np.float32)
        if r.size == 0:
            return
        a = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
        half = math.radians(FRONT_CONE_DEG)
        ok = (np.abs(a) <= half) & (r > 0.05) & (r < msg.range_max) & np.isfinite(r)
        self.front_m = float(r[ok].min()) if np.any(ok) else float("inf")

    def _on_aeb(self, msg: Bool) -> None:
        rising = msg.data and not self.aeb
        self.aeb = msg.data
        if rising:
            self._pending = {
                "t": self._now() - self._t0,
                "before": list(self._hist),
                "after": [],
            }

    def _tick(self) -> None:
        row = {
            "t": round(self._now() - self._t0, 3),
            "mode": self.mode,
            "aeb": int(self.aeb),
            "ttc": round(self.ttc, 3) if math.isfinite(self.ttc) else 99.0,
            "speed": round(self.speed, 2),
            "scale": round(self.scale, 2),
            "cte": round(self.cte, 3),
            "override": int(self.override),
            "fgm_enable": int(self.fgm_en),
            "n_obs": self.n_obs,
            "front_m": round(self.front_m, 2) if math.isfinite(self.front_m) else 99.0,
        }
        self._hist.append(row)

        now = row["t"]
        if now - self._last_beat >= 5.0:
            self._last_beat = now
            self.get_logger().info(
                f"[{now:6.0f}s] mode={row['mode']:>6} v={row['speed']:4.1f} "
                f"front={row['front_m']:5.2f}m obs={row['n_obs']} "
                f"ovr={row['override']} | AEB {len(self._events)}건"
            )

        if self._csv_w:
            self._csv_w.writerow([row[k] for k in
                                  ("t", "mode", "aeb", "ttc", "speed", "scale",
                                   "cte", "override", "fgm_enable", "n_obs", "front_m")])

        if self._pending is not None:
            self._pending["after"].append(row)
            if len(self._pending["after"]) >= POST_SEC * TICK_HZ:
                self._events.append(self._pending)
                self._pending = None
                self.get_logger().warn(
                    f"AEB #{len(self._events)} 기록됨 (t={self._events[-1]['t']:.1f}s)"
                )

    # ------------------------------------------------------------- 리포트

    def report(self) -> None:
        if self._pending is not None:
            self._events.append(self._pending)
        if self._csv:
            self._csv.close()

        print(f"\n{'='*66}")
        print(f"AEB 이벤트 {len(self._events)}건 / 주행 {self._now()-self._t0:.0f}s")
        print("=" * 66)
        if not self._events:
            print("AEB 가 한 번도 안 걸렸다.")
            self._print_modes()
            return

        after_avoid = 0
        for i, ev in enumerate(self._events, 1):
            before = ev["before"]
            trig = before[-1] if before else {}
            modes = [r["mode"] for r in before]
            # 직전 이력에 회피가 있었나
            avoided = any(m in ("AVOID", "REJOIN") for m in modes)
            if avoided:
                after_avoid += 1
            # 회피 종료 후 AEB 까지 걸린 시간
            gap = None
            for r in reversed(before):
                if r["mode"] in ("AVOID", "REJOIN"):
                    gap = ev["t"] - r["t"]
                    break

            print(f"\n--- AEB #{i}  t={ev['t']:.1f}s "
                  f"{'[회피 직후]' if avoided else '[회피와 무관]'}")
            print(f"    발동 시점: mode={trig.get('mode')} "
                  f"v={trig.get('speed')}m/s ttc={trig.get('ttc')}s "
                  f"front={trig.get('front_m')}m cte={trig.get('cte')}m")
            if gap is not None:
                print(f"    회피 종료 → AEB 까지 {gap:.2f}s")
            print(f"    직전 모드 전이: {self._mode_seq(before)}")
            print("      t      mode  v     front  ttc   cte    ovr obs")
            for r in before[::5] + ev["after"][::10]:
                print(f"    {r['t']:7.2f} {r['mode']:>6} {r['speed']:5.2f} "
                      f"{r['front_m']:6.2f} {r['ttc']:5.2f} {r['cte']:+6.3f} "
                      f"{r['override']:3d} {r['n_obs']:3d}")

        print(f"\n{'='*66}")
        print(f"회피 직후 AEB: {after_avoid}/{len(self._events)}건")
        self._verdict(after_avoid)
        self._print_modes()

    @staticmethod
    def _mode_seq(rows: list[dict]) -> str:
        seq = []
        for r in rows:
            if not seq or seq[-1] != r["mode"]:
                seq.append(r["mode"])
        return " → ".join(seq) if seq else "?"

    def _verdict(self, after_avoid: int) -> None:
        if after_avoid == 0:
            print("→ 회피와 무관한 AEB 다. 장애물·벽 거리 자체를 봐야 한다.")
            return
        # 회피 직후 이벤트만 모아 공통 패턴을 본다
        gaps, fronts, modes_at_trig = [], [], []
        for ev in self._events:
            before = ev["before"]
            if not any(r["mode"] in ("AVOID", "REJOIN") for r in before):
                continue
            for r in reversed(before):
                if r["mode"] in ("AVOID", "REJOIN"):
                    gaps.append(ev["t"] - r["t"])
                    break
            if before:
                fronts.append(before[-1]["front_m"])
                modes_at_trig.append(before[-1]["mode"])

        if gaps:
            print(f"→ 회피 종료 후 AEB 까지: 중앙값 {sorted(gaps)[len(gaps)//2]:.2f}s "
                  f"(최소 {min(gaps):.2f}s)")
        if fronts:
            print(f"→ 발동 시 전방 최근접: 중앙값 {sorted(fronts)[len(fronts)//2]:.2f}m")
        glob = sum(1 for m in modes_at_trig if m == "GLOBAL")
        if glob and gaps and min(gaps) < 0.5:
            print("→ 유력: 회피가 끝나 GLOBAL 로 돌아간 직후 AEB 임계가 다시")
            print("  빡빡해지면서 걸린다. emergency_brake_node 의 avoid_modes")
            print("  완화가 REJOIN 종료와 함께 사라지는 게 원인일 수 있다.")
            print("  → REJOIN 을 더 오래 유지하거나 완화를 짧게 연장할 것.")

    def _print_modes(self) -> None:
        if not self._mode_changes:
            return
        print(f"\n모드 전이 전체 ({len(self._mode_changes)}회)")
        for t, a, b in self._mode_changes:
            print(f"  {t:7.2f}s  {a:>7} → {b}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default="/tmp/aeb_after_avoid.csv")
    args = ap.parse_args()

    rclpy.init()
    node = AebContextLogger(args.csv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
