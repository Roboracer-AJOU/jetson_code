#!/usr/bin/env python3
"""회피가 장애물 크기에 비해 과한지, 과하다면 **어디서** 과해지는지 가른다.

두 가지가 섞여 보인다.

1. **경로가 크게 그려진다** — FGM 버블(r+0.20+0.15)이 섹터를 넓게 지워
   갭 중심이 멀어진다. 이러면 차는 경로를 잘 따라가는데 경로 자체가 크다.
2. **차가 경로를 넘어간다** — LOCAL_PATH 에선 FF 가 꺼지고
   `local_path_cte_speed_cap_mps`(1.2) 때문에 3 m/s 로 달려도 1.2 m/s 인 양
   계산해 게인이 2.5 배로 뻥튀기된다. 이러면 경로는 얌전한데 차가 넘친다.

둘은 고치는 데가 완전히 다르다. 그래서 한 회피 구간마다 이렇게 본다.

    필요량  need   = r + 차폭절반 + 여유      (이만큼만 비키면 된다)
    실제이탈 swerve = max|csv_lat|            (실제로 이만큼 비켰다)
    추종오차 track  = max|path_lat|           (경로에서 이만큼 벗어났다)

    swerve ≈ need              → 과회피 아님
    swerve ≫ need, track 작음  → (1) 경로가 크다   → 버블/갭 선정
    track 큼                   → (2) 차가 넘친다   → 게인/속도캡

    python3 scripts/log_overavoid.py
    python3 scripts/log_overavoid.py --csv /tmp/overavoid.csv

장애물 하나 놓고 몇 번 지나간 뒤 Ctrl-C 하면 구간별 판정을 출력한다.
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
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32MultiArray, Float64, Float64MultiArray, String

TICK_HZ = 50.0
PRE_SEC = 1.0
POST_SEC = 2.5

EGO_HALF_WIDTH_M = 0.15
CLEAR_MARGIN_M = 0.10

# 판정 문턱
OVER_RATIO = 1.6      # swerve / need 가 이보다 크면 과회피
TRACK_ERR_M = 0.40    # AVOID 추종오차 상한 (실측 정상치가 0.33 이었다)
REJOIN_MIN_V = 0.8    # 복귀 중 이 아래로 떨어지면 "멈췄다"
REJOIN_TRACK_M = 0.60  # 복귀 경로 추종오차 상한

# /stanley/debug 배열 인덱스
I_PATH_LAT = 12       # 지금 따라가는 경로까지의 횡거리
I_CSV_LAT = 15        # CSV 레이스라인까지의 횡거리
I_HDG_TERM = 2
I_CTE_TERM = 3


class OverAvoidLogger(Node):
    def __init__(self, csv_path: str | None):
        super().__init__("log_overavoid")

        self.mode = "?"
        self.speed = 0.0
        self.override = False
        self.fgm_on = False
        self.reason = "none"
        self.scale = 1.0
        self.path_lat = 0.0
        self.csv_lat = 0.0
        self.hdg_term = 0.0
        self.cte_term = 0.0
        self.aim_deg = 0.0
        self.obs_r = 0.0
        self.obs_x = float("nan")
        self.obs_y = float("nan")

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
        sub(Float64, "/vehicle/speed_mps", lambda m: setattr(self, "speed", m.data), 10)
        sub(Bool, "/planner_path_override_active",
            lambda m: setattr(self, "override", m.data), 10)
        sub(Bool, "/planner/fgm_enable", lambda m: setattr(self, "fgm_on", m.data), 10)
        sub(String, "/planner/speed_reason",
            lambda m: setattr(self, "reason", m.data), 10)
        sub(Float64, "/planner/speed_scale", lambda m: setattr(self, "scale", m.data), 10)
        sub(Float64MultiArray, "/stanley/debug", self._on_debug, 10)
        sub(PointStamped, "/fgm_target", self._on_target, 10)
        sub(Float32MultiArray, "/static_obstacles", self._on_obs, 10)

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            "과회피 로거 시작 — 장애물 앞으로 몇 번 지나간 뒤 Ctrl-C"
        )

    # ------------------------------------------------------------ 콜백

    def _on_debug(self, m: Float64MultiArray) -> None:
        d = m.data
        if len(d) > I_CSV_LAT:
            self.path_lat = float(d[I_PATH_LAT])
            self.csv_lat = float(d[I_CSV_LAT])
            self.hdg_term = float(d[I_HDG_TERM])
            self.cte_term = float(d[I_CTE_TERM])

    def _on_target(self, m: PointStamped) -> None:
        self.aim_deg = math.degrees(math.atan2(m.point.y, max(m.point.x, 1e-3)))

    def _on_obs(self, m: Float32MultiArray) -> None:
        """전방에서 제일 가까운 장애물 하나만 본다 ([id, x, y, r] 반복)."""
        best = None
        for i in range(len(m.data) // 4):
            x, y, r = float(m.data[4 * i + 1]), float(m.data[4 * i + 2]), \
                float(m.data[4 * i + 3])
            if x <= 0.0:
                continue
            if best is None or x < best[0]:
                best = (x, y, r)
        if best is None:
            self.obs_x = self.obs_y = float("nan")
            self.obs_r = 0.0
        else:
            self.obs_x, self.obs_y, self.obs_r = best

    # ------------------------------------------------------------ 수집

    @staticmethod
    def _cols() -> list[str]:
        return ["t", "mode", "speed", "scale", "reason", "override", "fgm",
                "csv_lat", "path_lat", "hdg_deg", "cte_deg", "aim_deg",
                "obs_x", "obs_y", "obs_r"]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        row = {
            "t": round(self._now() - self._t0, 3),
            "mode": self.mode,
            "speed": round(self.speed, 3),
            "scale": round(self.scale, 3),
            "reason": self.reason,
            "override": int(self.override),
            "fgm": int(self.fgm_on),
            "csv_lat": round(self.csv_lat, 3),
            "path_lat": round(self.path_lat, 3),
            "hdg_deg": round(math.degrees(self.hdg_term), 2),
            "cte_deg": round(math.degrees(self.cte_term), 2),
            "aim_deg": round(self.aim_deg, 1),
            "obs_x": round(self.obs_x, 2),
            "obs_y": round(self.obs_y, 2),
            "obs_r": round(self.obs_r, 3),
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
                    self.get_logger().warn(f"회피 #{len(self._events)} 기록됨")
        self._hist.append(row)

        now = row["t"]
        if now - self._last_beat >= 5.0:
            self._last_beat = now
            self.get_logger().info(
                f"[{now:6.0f}s] {row['mode']:>6} v={row['speed']:4.1f} "
                f"csv_lat={row['csv_lat']:+5.2f} r={row['obs_r']:4.2f} "
                f"| 회피 {len(self._events)}건"
            )

    # ---------------------------------------------------------- 리포트

    def report(self) -> None:
        if self._active is not None:
            self._events.append(self._active)
        if self._csv:
            self._csv.close()

        print(f"\n{'=' * 72}")
        print(f"회피 {len(self._events)}건 / 주행 {self._now() - self._t0:.0f}s")
        print("=" * 72)
        if not self._events:
            print("회피가 한 번도 안 걸렸다.")
            return

        bad: list[str] = []
        for i, ev in enumerate(self._events, 1):
            bad += [f"#{i} {m}" for m in self._one(i, ev)]

        print(f"\n{'=' * 72}")
        if not bad:
            print(f"합격 — {len(self._events)}건 모두 정상")
        else:
            print(f"불합격 {len(bad)}건")
            for b in bad:
                print(f"  {b}")
        print("\n합격 기준")
        print(f"  AVOID  : 실제 이탈 ≤ 필요량의 {OVER_RATIO}배  (과회피 아님)")
        print(f"           추종오차 ≤ {TRACK_ERR_M}m           (차가 안 넘침)")
        print(f"  REJOIN : 최저속도 ≥ {REJOIN_MIN_V} m/s        (안 멈춤)")
        print(f"           추종오차 ≤ {REJOIN_TRACK_M}m          (경로를 따라감)")

    def _one(self, i: int, ev: dict) -> list[str]:
        """구간 하나를 AVOID / REJOIN 으로 **나눠서** 본다.

        예전엔 둘을 한 덩어리로 묶어 최대값을 냈다. 그러면 회피가 끝난 뒤의
        난장판이 회피 구간 점수로 들어가서, 추종이 멀쩡했는데도 "게인 문제"
        라고 잘못 판정했다.
        """
        rows = ev["rows"]
        bad: list[str] = []
        print(f"\n--- 회피 #{i}  t={ev['t']:.1f}s  ({len(rows) / TICK_HZ:.1f}s)")

        for phase in ("AVOID", "REJOIN"):
            ph = [r for r in rows if r["mode"] == phase]
            if len(ph) < 5:
                continue
            radii = sorted(r["obs_r"] for r in ph if r["obs_r"] > 0.0)
            r_obs = radii[len(radii) // 2] if radii else 0.0
            need = r_obs + EGO_HALF_WIDTH_M + CLEAR_MARGIN_M
            swerve = max(abs(r["csv_lat"]) for r in ph)
            on_path = [abs(r["path_lat"]) for r in ph if r["override"]]
            track = max(on_path) if on_path else float("nan")
            aim = max(abs(r["aim_deg"]) for r in ph)
            v_lo, v_hi = min(r["speed"] for r in ph), max(r["speed"] for r in ph)

            why: dict[str, int] = {}
            for r in ph:
                if r["scale"] < 0.95:
                    why[r["reason"]] = why.get(r["reason"], 0) + 1
            top = ", ".join(f"{k}x{v}" for k, v in
                            sorted(why.items(), key=lambda kv: -kv[1])[:3])

            print(f"  [{phase}] {len(ph) / TICK_HZ:.1f}s  "
                  f"v {v_lo:.1f}~{v_hi:.1f}  조준각 최대 {aim:.0f}°")
            print(f"      이탈 {swerve:.2f}m / 필요 {need:.2f}m "
                  f"({swerve / need:.1f}배)   추종오차 {track:.2f}m")
            if top:
                print(f"      감속 사유: {top}")

            if phase == "AVOID":
                if need > 1e-3 and swerve / need > OVER_RATIO:
                    bad.append(f"AVOID 과회피 {swerve / need:.1f}배")
                if not math.isnan(track) and track > TRACK_ERR_M:
                    bad.append(f"AVOID 추종오차 {track:.2f}m")
            else:
                if v_lo < REJOIN_MIN_V:
                    bad.append(f"REJOIN 정지 {v_lo:.2f} m/s ({top or '사유없음'})")
                if not math.isnan(track) and track > REJOIN_TRACK_M:
                    bad.append(f"REJOIN 추종오차 {track:.2f}m")

        print("       t     mode  ovr  v   scale reason     csv_lat path_lat  aim")
        for r in rows[::5]:
            print(f"    {r['t']:7.2f} {r['mode']:>6} {r['override']:3d} "
                  f"{r['speed']:4.1f} {r['scale']:5.2f} {r['reason']:>10} "
                  f"{r['csv_lat']:+7.2f} {r['path_lat']:+7.2f} {r['aim_deg']:+6.1f}")
        return bad


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default="/tmp/overavoid.csv")
    args = ap.parse_args()

    rclpy.init()
    node = OverAvoidLogger(args.csv)
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
