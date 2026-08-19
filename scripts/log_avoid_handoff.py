#!/usr/bin/env python3
"""회피 → 재합류 인수인계가 매끄러운지 보는 로거.

증상은 "FGM 회피 중 차가 한 번 멈추고, 갑자기 라인으로 확 되돌아간다" 였다.
원인은 두 개가 겹친 것이었다.

1. 회피 선감속이 거의 정지까지 깎는다 (배율 0.17, 0.46 m/s)
2. 회피가 풀리는 순간 override 가 내려가 Stanley 기준경로가 로컬경로 → CSV 로
   순간이동한다. 라인에서 벗어나 있으면 CTE 가 계단으로 뛰며 급조향이 나가고,
   동시에 속도가 무제한으로 회복된다 (6 m/s^2)

그래서 이 셋을 같이 본다: **override 연속성**, **속도 프로파일**,
**어느 정책이 속도를 깎았는지**(`/planner/speed_reason`).

    python3 scripts/log_avoid_handoff.py
    python3 scripts/log_avoid_handoff.py --csv /tmp/handoff.csv

한 바퀴 돌고 Ctrl-C 하면 회피 구간별로 판정을 출력한다. 합격 기준은
리포트 맨 아래에 같이 찍는다.
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import deque

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

TICK_HZ = 50.0
PRE_SEC = 1.5    # 회피 진입 전 맥락
POST_SEC = 4.0   # 회피 해제 후 — 인수인계는 여기서 일어난다

# 합격 기준
MAX_ACCEL = 5.0        # m/s^2, 회복 가속 상한 (여유 포함)
MAX_OVR_GAP_SEC = 0.15  # override 가 끊겨도 되는 최대 시간
MIN_SPEED_FRAC = 0.25   # 회피 중 최저속도 / 진입속도


class HandoffLogger(Node):
    def __init__(self, csv_path: str | None):
        super().__init__("log_avoid_handoff")

        self.mode = "?"
        self.speed = 0.0
        self.scale = 1.0
        self.reason = "none"
        self.cte = 0.0
        self.override = False
        self.aeb = False
        self.n_obs = 0

        self._hist: deque = deque(maxlen=int(PRE_SEC * TICK_HZ))
        self._events: list[dict] = []
        self._active: dict | None = None
        self._post_left = 0
        self._t0 = self._now()
        self._last_beat = 0.0

        self._csv = None
        self._csv_w = None
        if csv_path:
            self._csv = open(csv_path, "w", newline="")
            self._csv_w = csv.writer(self._csv)
            self._csv_w.writerow(self._cols())

        sub = self.create_subscription
        sub(String, "/planner/mode", lambda m: setattr(self, "mode", m.data), 10)
        sub(String, "/planner/speed_reason",
            lambda m: setattr(self, "reason", m.data), 10)
        sub(Float64, "/vehicle/speed_mps", lambda m: setattr(self, "speed", m.data), 10)
        sub(Float64, "/planner/speed_scale", lambda m: setattr(self, "scale", m.data), 10)
        sub(Float64, "/control/cross_track_error",
            lambda m: setattr(self, "cte", m.data), 10)
        sub(Bool, "/planner_path_override_active",
            lambda m: setattr(self, "override", m.data), 10)
        sub(Bool, "/emergency_brake", lambda m: setattr(self, "aeb", m.data), 10)
        sub(Float32MultiArray, "/static_obstacles",
            lambda m: setattr(self, "n_obs", len(m.data) // 4), 10)

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            "회피 인수인계 로거 시작 — 한 바퀴 돌고 Ctrl-C. "
            "AVOID 구간마다 해제 후 4초까지 추적한다"
        )

    @staticmethod
    def _cols() -> list[str]:
        return ["t", "mode", "speed", "scale", "reason", "cte",
                "override", "aeb", "n_obs"]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        row = {
            "t": round(self._now() - self._t0, 3),
            "mode": self.mode,
            "speed": round(self.speed, 3),
            "scale": round(self.scale, 3),
            "reason": self.reason,
            "cte": round(self.cte, 3),
            "override": int(self.override),
            "aeb": int(self.aeb),
            "n_obs": self.n_obs,
        }
        if self._csv_w:
            self._csv_w.writerow([row[c] for c in self._cols()])

        avoiding = self.mode in ("AVOID", "REJOIN")
        if avoiding and self._active is None:
            self._active = {"rows": list(self._hist), "t": row["t"]}
        if self._active is not None:
            self._active["rows"].append(row)
            if avoiding:
                self._post_left = int(POST_SEC * TICK_HZ)
            else:
                self._post_left -= 1
                if self._post_left <= 0:
                    self._events.append(self._active)
                    self._active = None
                    self.get_logger().warn(f"회피 #{len(self._events)} 구간 기록됨")
        self._hist.append(row)

        now = row["t"]
        if now - self._last_beat >= 5.0:
            self._last_beat = now
            self.get_logger().info(
                f"[{now:6.0f}s] {row['mode']:>6} v={row['speed']:4.1f} "
                f"scale={row['scale']:4.2f}({row['reason']}) ovr={row['override']} "
                f"| 회피 {len(self._events)}건"
            )

    # ------------------------------------------------------------- 리포트

    def report(self) -> None:
        if self._active is not None:
            self._events.append(self._active)
        if self._csv:
            self._csv.close()

        print(f"\n{'=' * 70}")
        print(f"회피 구간 {len(self._events)}건 / 주행 {self._now() - self._t0:.0f}s")
        print("=" * 70)
        if not self._events:
            print("회피가 한 번도 안 걸렸다.")
            return

        verdicts = []
        for i, ev in enumerate(self._events, 1):
            verdicts.append(self._one(i, ev))

        print(f"\n{'=' * 70}")
        ok = sum(1 for v in verdicts if not v)
        print(f"합격 {ok}/{len(verdicts)}건")
        for i, v in enumerate(verdicts, 1):
            if v:
                print(f"  #{i}: " + "; ".join(v))
        print("\n합격 기준")
        print(f"  - 회복 가속 ≤ {MAX_ACCEL} m/s^2   (avoid_a_accel_mps2=4.0)")
        print(f"  - override 끊김 ≤ {MAX_OVR_GAP_SEC}s (재합류 경로로 이어 붙임)")
        print(f"  - 회피 중 최저속도 ≥ 진입속도의 {MIN_SPEED_FRAC:.0%}")

    def _one(self, i: int, ev: dict) -> list[str]:
        rows = ev["rows"]
        bad: list[str] = []

        v_in = max((r["speed"] for r in rows if r["mode"] == "GLOBAL"), default=0.0)
        av = [r for r in rows if r["mode"] in ("AVOID", "REJOIN")]
        if not av:
            return bad
        v_min = min(r["speed"] for r in av)

        # 최대 가속
        a_max, a_at = 0.0, None
        for a, b in zip(rows, rows[1:]):
            dt = b["t"] - a["t"]
            if dt <= 0:
                continue
            acc = (b["speed"] - a["speed"]) / dt
            if acc > a_max:
                a_max, a_at = acc, b["t"]

        # override 끊김: 회피 시작 후 라인에서 벗어난 채 override 가 0 인 구간
        gap, gap_at, run = 0.0, None, 0.0
        for a, b in zip(rows, rows[1:]):
            off_line = abs(b["cte"]) > 0.20
            if b["mode"] in ("AVOID", "REJOIN") and not b["override"] and off_line:
                run += b["t"] - a["t"]
                if run > gap:
                    gap, gap_at = run, b["t"]
            else:
                run = 0.0

        # CTE 계단 (기준경로 전환 순간)
        step, step_at = 0.0, None
        for a, b in zip(rows, rows[1:]):
            if a["override"] != b["override"]:
                d = abs(b["cte"] - a["cte"])
                if d > step:
                    step, step_at = d, b["t"]

        # 왜 느려졌나
        reasons: dict[str, int] = {}
        for r in av:
            if r["scale"] < 0.95:
                reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        top = sorted(reasons.items(), key=lambda kv: -kv[1])[:3]

        print(f"\n--- 회피 #{i}  t={ev['t']:.1f}s  ({len(rows) / TICK_HZ:.1f}s)")
        print(f"    진입속도 {v_in:.2f} → 최저 {v_min:.2f} m/s "
              f"({v_min / v_in:.0%})" if v_in > 0.1 else
              f"    최저 {v_min:.2f} m/s")
        print(f"    최대 회복가속 {a_max:.1f} m/s^2"
              + (f" @ t={a_at:.2f}" if a_at else ""))
        print(f"    override 끊김 {gap:.2f}s"
              + (f" @ t={gap_at:.2f}" if gap_at else "")
              + f"   기준경로 전환 시 CTE 계단 {step:.2f}m"
              + (f" @ t={step_at:.2f}" if step_at else ""))
        if top:
            print("    감속 사유: "
                  + ", ".join(f"{k}×{v}" for k, v in top))
        print("       t     mode   v    scale  reason      cte    ovr")
        for r in rows[::5]:
            print(f"    {r['t']:7.2f} {r['mode']:>6} {r['speed']:5.2f} "
                  f"{r['scale']:5.2f} {r['reason']:>10} {r['cte']:+6.2f} "
                  f"{r['override']:3d}")

        if a_max > MAX_ACCEL:
            bad.append(f"회복가속 {a_max:.1f} m/s^2 초과")
        if gap > MAX_OVR_GAP_SEC:
            bad.append(f"override 끊김 {gap:.2f}s (라인 밖에서 CSV 생짜 추종)")
        if v_in > 0.1 and v_min < MIN_SPEED_FRAC * v_in:
            bad.append(f"과도한 감속 {v_in:.1f}→{v_min:.1f} m/s "
                       f"(사유 {top[0][0] if top else '?'})")
        return bad


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default="/tmp/avoid_handoff.csv")
    args = ap.parse_args()

    rclpy.init()
    node = HandoffLogger(args.csv)
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
