#!/usr/bin/env python3
"""라인 추종 실패를 조향 체인 단계별로 분해한다 (AUTO 구간만).

    python3 scripts/log_line_tracking.py --csv /tmp/track

"라인을 못 따라간다" 는 증상 하나에 원인이 여러 개 겹칠 수 있어서, 사슬을
끊어서 각 마디를 따로 본다. 어느 마디가 끊겼는지 알면 고칠 데가 하나로 좁혀진다.

    (1) 경로 오차      CTE / 헤딩오차 — 얼마나 못 따라가는가
    (2) Stanley 판단   그 오차에 대해 얼마를 요구했는가 (FF / CTE / 헤딩 분해)
    (3) 명령 전달      요구한 값이 ESP 까지 그대로 갔는가
    (4) 서보 추종      ESP 가 그걸 실제로 냈는가 (스무딩 지연)
    (5) 포화           애초에 낼 수 있는 각을 넘어선 건 아닌가

MANUAL 은 통째로 버린다. ESP 가 MANUAL 에서는 젯슨 조향을 무시하고 RC 스틱을
쓰기 때문에 (3)(4) 가 무의미해진다.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/path_following"))

from path_following.control_node import CFG as CTL_CFG  # noqa: E402
from path_following.stanley_waypoint_follow_node import CFG as STA_CFG  # noqa: E402
from path_following.track_sliding import load_csv_xyv, resolve_csv_path  # noqa: E402

SERVO_CENTER_DEG = 90.0
SERVO_HALF_DEG = 50.0

TEL_CURRENT_STEER = 4
TEL_AUTONOMOUS = 6
TEL_ESTOP = 7

MODE_STALE_S = 0.5


def _yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Tracker(Node):
    def __init__(self, csv_prefix: str | None) -> None:
        super().__init__("log_line_tracking")
        self.csv_prefix = csv_prefix
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        pts, _ = load_csv_xyv(resolve_csv_path("", "raceline"))
        self.line = np.asarray(pts, dtype=np.float64)
        d = np.roll(self.line, -1, axis=0) - self.line
        self.line_yaw = np.arctan2(d[:, 1], d[:, 0])

        self.max_steer_rad = float(CTL_CFG["max_steering_angle_rad"])
        self.servo_full = float(STA_CFG["max_steering_angle"])
        self.real_full = float(STA_CFG["max_steering_angle_real_rad"])
        self.rebase = (
            self.real_full / self.servo_full
            if STA_CFG["steer_scale_calibrated"]
            else 1.0
        )
        self.delivery = (self.rebase / self.max_steer_rad) * self.servo_full

        self.auto = False
        self.estop = False
        self.mode_t = 0.0
        self.auto_secs = 0.0
        self.manual_secs = 0.0
        self._last_tel = None

        self.rows: list[dict] = []
        self.cmd: list[tuple[float, float]] = []
        self.esp_tgt: list[tuple[float, float]] = []
        self.esp_srv: list[tuple[float, float]] = []
        self.sat_count = 0
        self.drive_count = 0
        self.planner_mode = "?"
        self.mode_hist: dict[str, int] = {}

        self.create_subscription(
            Float64MultiArray, "/vehicle/telemetry", self._on_tel, 10
        )
        self.create_subscription(AckermannDriveStamped, "/drive", self._on_drive, 20)
        self.create_subscription(
            Float32, "/esp32/target_angle_deg", self._on_tgt, 20
        )
        self.create_subscription(
            Float32, "/esp32/servo_command_deg", self._on_srv, 20
        )
        self.create_subscription(String, "/planner/mode", self._on_pmode, 10)

        self.create_timer(5.0, self._beat)
        self.get_logger().info(
            f"라인추종 로깅 시작 | 조향 실효배율 {self.delivery:.3f} "
            f"(분모 {self.max_steer_rad:.4f}) | AUTO 로 주행하세요"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _active(self) -> bool:
        if self._now() - self.mode_t > MODE_STALE_S:
            return False
        return self.auto and not self.estop

    def _beat(self) -> None:
        s = "AUTO" if self._active() else ("E-STOP" if self.estop else "MANUAL")
        cte = (
            f"{np.mean([abs(r['cte']) for r in self.rows[-100:]]):.2f}m"
            if self.rows
            else "-"
        )
        self.get_logger().info(
            f"[{s}] AUTO {self.auto_secs:.0f}s | 표본 {len(self.rows)} "
            f"| 최근 |CTE| {cte} | 포화 {self.sat_count}"
        )

    def _on_pmode(self, msg: String) -> None:
        self.planner_mode = msg.data

    def _on_tel(self, msg: Float64MultiArray) -> None:
        d = msg.data
        if len(d) <= TEL_ESTOP:
            return
        now = self._now()
        was = self.auto and not self.estop
        self.auto = d[TEL_AUTONOMOUS] > 0.5
        self.estop = d[TEL_ESTOP] > 0.5
        self.mode_t = now
        if self._last_tel is not None:
            dt = min(now - self._last_tel, 0.5)
            if was:
                self.auto_secs += dt
            else:
                self.manual_secs += dt
        self._last_tel = now
        if was:
            self.cmd.append(
                (now, SERVO_CENTER_DEG + float(d[TEL_CURRENT_STEER]) * SERVO_HALF_DEG)
            )

    def _on_tgt(self, msg: Float32) -> None:
        if self._active():
            self.esp_tgt.append((self._now(), float(msg.data)))

    def _on_srv(self, msg: Float32) -> None:
        if self._active():
            self.esp_srv.append((self._now(), float(msg.data)))

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        if not self._active():
            return
        self.drive_count += 1
        steer = float(msg.drive.steering_angle)
        speed = float(msg.drive.speed)
        if abs(steer) >= self.real_full * 0.98:
            self.sat_count += 1

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except TransformException:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = _yaw_of(tf.transform.rotation)

        i = int(np.argmin(np.hypot(self.line[:, 0] - x, self.line[:, 1] - y)))
        pyaw = float(self.line_yaw[i])
        # 경로 오른쪽 법선에 투영 → cte>0 이면 차가 경로 오른쪽
        cte = (x - self.line[i, 0]) * math.sin(pyaw) - (
            y - self.line[i, 1]
        ) * math.cos(pyaw)

        self.mode_hist[self.planner_mode] = self.mode_hist.get(self.planner_mode, 0) + 1
        self.rows.append(
            {
                "t": self._now(),
                "x": x,
                "y": y,
                "idx": i,
                "cte": cte,
                "hdg_err_deg": math.degrees(_wrap(pyaw - yaw)),
                "steer_rad": steer,
                "steer_servo_deg": SERVO_CENTER_DEG
                + max(-1.0, min(1.0, steer / self.max_steer_rad)) * SERVO_HALF_DEG,
                "speed_cmd": speed,
                "pmode": self.planner_mode,
            }
        )

    @staticmethod
    def _pair(a, b):
        if len(a) < 20 or len(b) < 20:
            return None
        at = np.array([p[0] for p in a])
        av = np.array([p[1] for p in a])
        bt = np.array([p[0] for p in b])
        bv = np.array([p[1] for p in b])
        idx = np.clip(np.searchsorted(bt, at) - 1, 0, len(bt) - 1)
        ok = np.abs(bt[idx] - at) < 0.10
        if ok.sum() < 20:
            return None
        dt = float(np.median(np.diff(at))) if at.size > 2 else 0.02
        return av[ok], bv[idx][ok], dt

    @staticmethod
    def _lag_gain(c, a, dt):
        best_lag, best_err = 0, float("inf")
        for lag in range(0, 26):
            if lag >= len(c) - 10:
                break
            e = float(np.mean(np.abs(c[: len(c) - lag] - a[lag:])))
            if e < best_err:
                best_err, best_lag = e, lag
        mv = np.abs(c - SERVO_CENTER_DEG) > 2.0
        g = float("nan")
        if mv.sum() > 10:
            n = c[mv] - SERVO_CENTER_DEG
            d = a[mv] - SERVO_CENTER_DEG
            g = float(np.sum(n * d) / max(np.sum(n * n), 1e-9))
        return best_lag * dt, best_err, g

    def report(self) -> None:
        print("\n" + "=" * 70)
        print(f"AUTO {self.auto_secs:.0f}s / MANUAL {self.manual_secs:.0f}s")
        print(f"조향 실효배율 {self.delivery:.3f} "
              f"(게인재환산 {self.rebase:.3f} / 분모 {self.max_steer_rad:.4f})")
        if self.auto_secs < 3.0 or not self.rows:
            print("\nAUTO 구간이 없다 — CH5 를 AUTO 로 올리고 주행해야 한다.")
            print("=" * 70)
            return

        cte = np.array([r["cte"] for r in self.rows])
        hdg = np.array([r["hdg_err_deg"] for r in self.rows])
        srad = np.array([r["steer_rad"] for r in self.rows])
        spd = np.array([r["speed_cmd"] for r in self.rows])

        print(f"\n[1] 경로 오차  표본 {len(cte)}")
        print(f"  |CTE| 중앙 {np.median(np.abs(cte)):.3f} m  "
              f"90% {np.percentile(np.abs(cte),90):.3f}  최대 {np.abs(cte).max():.3f}")
        print(f"  CTE 부호 평균 {cte.mean():+.3f} m", end="  ")
        print("← 한쪽으로 치우침(정상상태 오차)" if abs(cte.mean()) > 0.15
              else "← 편향 없음")
        print(f"  헤딩오차 중앙 {np.median(np.abs(hdg)):.1f}°  최대 {np.abs(hdg).max():.1f}°")

        # 진동인지 편향인지: CTE 부호가 얼마나 자주 바뀌나
        sign_flips = int(np.sum(np.diff(np.sign(cte)) != 0))
        rate = sign_flips / max(len(cte), 1)
        print(f"  CTE 부호 전환 {sign_flips}회 ({rate*100:.1f}%/표본)", end="  ")
        print("← 진동 (게인 과다)" if rate > 0.08 else "← 진동 아님")

        print(f"\n[2] Stanley 요구  속도 {np.median(spd):.1f} m/s")
        print(f"  조향 요구 중앙 {math.degrees(np.median(np.abs(srad))):.1f}°  "
              f"최대 {math.degrees(np.abs(srad).max()):.1f}°")
        print(f"  포화(풀락) {self.sat_count}/{self.drive_count} "
              f"({self.sat_count/max(self.drive_count,1)*100:.1f}%)", end="  ")
        print("← 조향이 모자란다" if self.sat_count > self.drive_count * 0.05
              else "← 여유 있음")

        if self.mode_hist:
            tot = sum(self.mode_hist.values())
            modes = ", ".join(
                f"{k} {v/tot*100:.0f}%"
                for k, v in sorted(self.mode_hist.items(), key=lambda kv: -kv[1])
            )
            print(f"  플래너 모드: {modes}")

        for title, src, dst in (
            ("[3] 명령 전달  젯슨 → ESP 목표각", self.cmd, self.esp_tgt),
            ("[4] 서보 추종  ESP 목표각 → 실제각", self.esp_tgt, self.esp_srv),
        ):
            got = self._pair(src, dst)
            print(f"\n{title}")
            if got is None:
                print(f"  표본 부족 ({len(src)}/{len(dst)})")
                continue
            c, a, dt = got
            lag, err, g = self._lag_gain(c, a, dt)
            print(f"  표본 {len(c)}쌍  오차 {err:.2f}°  지연 {lag*1000:.0f} ms  "
                  f"기울기 {g:.3f}")
            if g == g and abs(g - 1.0) > 0.10:
                print(f"    → 기울기가 1 에서 벗어났다. 이 마디에서 값이 변형된다")
            if lag > 0.06:
                print(f"    → 지연이 크다")

        # 구간별로 어디서 벌어지는지
        print("\n[5] 트랙 구간별 |CTE| (라인 인덱스 10등분)")
        idx = np.array([r["idx"] for r in self.rows])
        n = len(self.line)
        for k in range(10):
            m = (idx >= k * n // 10) & (idx < (k + 1) * n // 10)
            if m.sum() < 5:
                continue
            bar = "#" * int(np.median(np.abs(cte[m])) / 0.05)
            print(f"  {k*n//10:4d}~{(k+1)*n//10:4d}  "
                  f"{np.median(np.abs(cte[m])):.3f} m {bar}")
        print("=" * 70)

        if self.csv_prefix:
            p = Path(f"{self.csv_prefix}_track.csv")
            cols = list(self.rows[0].keys())
            with p.open("w", encoding="utf-8") as f:
                f.write(",".join(cols) + "\n")
                for r in self.rows:
                    f.write(",".join(f"{r[c]}" for c in cols) + "\n")
            print(f"원시 {len(self.rows)}행 → {p}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()
    rclpy.init()
    node = Tracker(args.csv)
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
