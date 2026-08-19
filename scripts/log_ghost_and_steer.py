#!/usr/bin/env python3
"""AUTO 구간만 골라서 오검 + 조향 추종을 진단한다.

    python3 scripts/log_ghost_and_steer.py            # Ctrl-C 로 종료하며 요약
    python3 scripts/log_ghost_and_steer.py --csv out  # 원시 샘플까지 CSV 로

**MANUAL 구간은 통째로 버린다.** ESP 는 MANUAL 에서 젯슨 조향(`S:`)을 무시하고
RC 스틱(CH1)을 그대로 서보에 넣기 때문에, 섞어서 보면 명령과 실제가 무상관으로
나와 아무것도 못 읽는다. `/vehicle/telemetry[6]` 의 autonomous 플래그로 자른다.

보는 것 두 가지.

1) 유령 장애물
   검출을 map 프레임으로 옮겨 실제 맵 벽까지 거리를 잰다.

     - 벽 위(≤0.20m)        → 벽 정합 실패
     - 벽 근처(0.20~0.45m)  → 팽창 반경 경계
     - 트인 공간(>0.45m)    → 진짜 물체이거나 라이다 노이즈

   여기에 **스캔-맵 정합도**를 같이 낸다. 이게 원인을 가른다: 측위가 틀어졌으면
   점구름 전체가 벽에서 떠서 정합도가 무너지고, 진짜 물체면 대부분은 벽에 붙어
   있고 그 물체 부분만 뜬다.

2) 조향 명령 대 실제 — 두 단계로 쪼갠다
     젯슨 `current_steer` → ESP `target_angle_deg`   : 통신·환산이 맞는가
     ESP `target_angle_deg` → `servo_command_deg`    : 스무딩 지연이 얼마인가
   한 덩어리로 보면 환산 오류와 지연이 섞여서 구분이 안 된다.
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
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/path_following"))

from path_following.integrated_obstacle_node import (  # noqa: E402
    CFG as OBS_CFG,
    resolve_map_yaml,
)
from path_following.track_sliding import load_csv_xyv, resolve_csv_path  # noqa: E402

# ESP normToAngle: S=±1 → 서보 90±50°.
SERVO_CENTER_DEG = 90.0
SERVO_HALF_DEG = 50.0

# /vehicle/telemetry 인덱스 (control_node._publish_telemetry)
TEL_CURRENT_STEER = 4
TEL_CH5 = 5
TEL_AUTONOMOUS = 6
TEL_ESTOP = 7

ON_WALL_M = 0.20
NEAR_WALL_M = 0.45
# AUTO 판정이 이보다 오래되면 모드를 모르는 것으로 본다.
MODE_STALE_S = 0.5


def _yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class WallGrid:
    def __init__(self, yaml_path: str) -> None:
        import yaml
        from PIL import Image
        from scipy import ndimage

        meta = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8"))
        img = Path(meta["image"])
        if not img.is_absolute():
            img = Path(yaml_path).parent / img
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        gray = np.asarray(Image.open(img).convert("L"), dtype=np.float64)
        occ = gray / 255.0 if int(meta.get("negate", 0)) else (255.0 - gray) / 255.0
        wall = occ >= float(meta.get("occupied_thresh", 0.65))
        self.h, self.w = wall.shape
        self.dist = ndimage.distance_transform_edt(~wall) * self.res
        self.name = Path(yaml_path).name

    def wall_distance(self, x, y):
        col = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        row = np.floor(
            (self.oy + self.h * self.res - np.asarray(y)) / self.res
        ).astype(np.int64)
        ok = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        out = np.full(np.shape(col), np.nan, dtype=np.float64)
        if np.any(ok):
            out[ok] = self.dist[row[ok], col[ok]]
        return out


class Logger(Node):
    def __init__(self, csv_prefix: str | None) -> None:
        super().__init__("log_ghost_and_steer")
        self.csv_prefix = csv_prefix
        self.grid = WallGrid(resolve_map_yaml(OBS_CFG["map_name"]))
        pts, _ = load_csv_xyv(resolve_csv_path("", "raceline"))
        self.line = np.asarray(pts, dtype=np.float64)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 모드 상태
        self.auto = False
        self.estop = False
        self.mode_t = 0.0
        self.auto_secs = 0.0
        self.manual_secs = 0.0
        self._last_tel_t = None

        self.ghosts: list[dict] = []
        self.cmd: list[tuple[float, float]] = []     # (t, 젯슨이 보낸 목표 서보각)
        self.esp_tgt: list[tuple[float, float]] = []  # (t, ESP 가 받은 목표각)
        self.esp_srv: list[tuple[float, float]] = []  # (t, 실제 write 각)
        self.fit: list[tuple[float, float]] = []      # (중앙 잔차, 벽 위 비율)
        self.aeb_events = 0
        self.scan_min = float("nan")
        self.scan_frames = 0
        self.frames_total = 0
        self.frames_with_obs = 0

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            Float64MultiArray, "/vehicle/telemetry", self._on_tel, 10
        )
        self.create_subscription(
            Float32MultiArray, "/static_obstacles", self._on_static, 10
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)
        self.create_subscription(
            Float32, "/esp32/target_angle_deg", self._on_esp_target, 10
        )
        self.create_subscription(
            Float32, "/esp32/servo_command_deg", self._on_esp_servo, 10
        )
        self.create_subscription(Bool, "/emergency_brake", self._on_aeb, 10)

        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info(
            f"로깅 시작 | map={self.grid.name} | AUTO 구간만 집계한다. "
            "CH5 를 AUTO 로 올리고 주행하세요."
        )

    # ------------------------------------------------------------ 모드

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _active(self) -> bool:
        """AUTO 이고 E-STOP 이 안 걸린 상태인가."""
        if self._now() - self.mode_t > MODE_STALE_S:
            return False
        return self.auto and not self.estop

    def _on_tel(self, msg: Float64MultiArray) -> None:
        d = msg.data
        if len(d) <= TEL_ESTOP:
            return
        now = self._now()
        was_auto = self.auto and not self.estop
        self.auto = d[TEL_AUTONOMOUS] > 0.5
        self.estop = d[TEL_ESTOP] > 0.5
        self.mode_t = now

        if self._last_tel_t is not None:
            dt = min(now - self._last_tel_t, 0.5)
            if was_auto:
                self.auto_secs += dt
            else:
                self.manual_secs += dt
        self._last_tel_t = now

        if self.auto and not self.estop:
            steer = float(d[TEL_CURRENT_STEER])
            self.cmd.append((now, SERVO_CENTER_DEG + steer * SERVO_HALF_DEG))

    def _heartbeat(self) -> None:
        state = "AUTO" if self._active() else ("E-STOP" if self.estop else "MANUAL")
        self.get_logger().info(
            f"[{state}] AUTO {self.auto_secs:.0f}s / MANUAL {self.manual_secs:.0f}s "
            f"| 검출 {len(self.ghosts)}건 AEB {self.aeb_events}회"
        )

    # ------------------------------------------------------------ 콜백

    def _on_esp_target(self, msg: Float32) -> None:
        if self._active():
            self.esp_tgt.append((self._now(), float(msg.data)))

    def _on_esp_servo(self, msg: Float32) -> None:
        if self._active():
            self.esp_srv.append((self._now(), float(msg.data)))

    def _on_aeb(self, msg: Bool) -> None:
        if msg.data and self._active():
            self.aeb_events += 1

    def _on_scan(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=np.float64)
        good = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        self.scan_min = float(r[good].min()) if np.any(good) else float("nan")
        if not self._active():
            return

        # 스캔 전체가 맵 벽에 얹히는지. 측위 틀어짐과 진짜 물체를 가르는 지표다.
        self.scan_frames += 1
        if self.scan_frames % 10:
            return
        try:
            tf = self.tf_buffer.lookup_transform("map", "laser", rclpy.time.Time())
        except TransformException:
            return
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        r_ok, a_ok = r[good], ang[good]
        if r_ok.size < 50:
            return
        yaw = _yaw_of(tf.transform.rotation)
        d = self.grid.wall_distance(
            tf.transform.translation.x + r_ok * np.cos(yaw + a_ok),
            tf.transform.translation.y + r_ok * np.sin(yaw + a_ok),
        )
        d = d[np.isfinite(d)]
        if d.size >= 50:
            self.fit.append((float(np.median(d)), float(np.mean(d <= 0.15))))

    def _on_static(self, msg: Float32MultiArray) -> None:
        if not self._active():
            return
        self.frames_total += 1
        data = list(msg.data)
        if len(data) < 4:
            return
        self.frames_with_obs += 1
        try:
            tf = self.tf_buffer.lookup_transform("map", "laser", rclpy.time.Time())
        except TransformException:
            return
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        yaw = _yaw_of(tf.transform.rotation)
        cs, sn = math.cos(yaw), math.sin(yaw)

        now = self._now()
        for i in range(0, len(data) - 3, 4):
            lx, ly, radius = data[i + 1], data[i + 2], data[i + 3]
            mx = tx + cs * lx - sn * ly
            my = ty + sn * lx + cs * ly
            self.ghosts.append(
                {
                    "t": now,
                    "id": int(data[i]),
                    "obs_x": mx,
                    "obs_y": my,
                    "range": math.hypot(lx, ly),
                    "bearing": math.degrees(math.atan2(ly, lx)),
                    "radius": radius,
                    "d_wall": float(self.grid.wall_distance(mx, my)),
                    "d_line": float(
                        np.min(np.hypot(self.line[:, 0] - mx, self.line[:, 1] - my))
                    ),
                    "car_x": tx,
                    "car_y": ty,
                    "car_yaw": math.degrees(yaw),
                    "scan_min": self.scan_min,
                }
            )

    # ------------------------------------------------------------ 요약

    @staticmethod
    def _pair(a: list, b: list):
        """a 의 시각에 맞춰 b 를 붙인다. (a값, b값, dt중앙)"""
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
    def _lag_and_gain(c: np.ndarray, a: np.ndarray, dt: float):
        best_lag, best_err = 0, float("inf")
        for lag in range(0, 26):
            if lag >= len(c) - 10:
                break
            err = float(np.mean(np.abs(c[: len(c) - lag] - a[lag:])))
            if err < best_err:
                best_err, best_lag = err, lag
        moving = np.abs(c - SERVO_CENTER_DEG) > 2.0
        gain = float("nan")
        if moving.sum() > 10:
            num = c[moving] - SERVO_CENTER_DEG
            den = a[moving] - SERVO_CENTER_DEG
            gain = float(np.sum(num * den) / max(np.sum(num * num), 1e-9))
        return best_lag, best_err, gain, best_lag * dt

    def _steer_report(self) -> None:
        print("\n[조향] AUTO 구간만")
        stages = (
            ("젯슨 명령 → ESP 수신 목표각", self.cmd, self.esp_tgt,
             "통신·환산. 어긋나면 시리얼 유실이나 상수 불일치다"),
            ("ESP 목표각 → 실제 서보각", self.esp_tgt, self.esp_srv,
             "스무딩 지연. 크면 리드 보상을 봐야 한다"),
        )
        for title, src, dst, hint in stages:
            got = self._pair(src, dst)
            if got is None:
                print(f"  {title}: 표본 부족 ({len(src)}/{len(dst)})")
                continue
            c, a, dt = got
            lag, err, gain, lag_s = self._lag_and_gain(c, a, dt)
            print(f"  {title}")
            print(f"    표본 {len(c)}쌍  명령 {c.min():.0f}~{c.max():.0f}° "
                  f"실제 {a.min():.0f}~{a.max():.0f}°")
            print(f"    지연 보정 후 오차 {err:.2f}°  지연 {lag_s*1000:.0f} ms  "
                  f"기울기 {gain:.3f}")
            if gain == gain and abs(gain - 1.0) > 0.10:
                print(f"      → {hint}")
            elif lag_s > 0.06:
                print(f"      → {hint}")

    def report(self) -> None:
        print("\n" + "=" * 70)
        print(f"AUTO {self.auto_secs:.0f}s / MANUAL {self.manual_secs:.0f}s")
        if self.auto_secs < 3.0:
            print("\nAUTO 구간이 거의 없다 — CH5 를 AUTO 로 올리고 다시 주행해야 한다.")
            print("=" * 70)
            return

        print(f"AUTO 중 장애물 프레임 {self.frames_with_obs}/{self.frames_total}, "
              f"AEB {self.aeb_events}회")

        if self.fit:
            med = np.array([f[0] for f in self.fit])
            onw = np.array([f[1] for f in self.fit])
            print(f"\n[스캔-맵 정합] {len(self.fit)}프레임")
            print(f"  점→벽 거리 중앙 {np.median(med):.3f} m "
                  f"(최악 프레임 {med.max():.3f} m)")
            print(f"  벽 위(≤0.15m) 비율 평균 {onw.mean()*100:.0f}% "
                  f"(최악 {onw.min()*100:.0f}%)")
            if np.median(med) > 0.20 or onw.mean() < 0.60:
                print("  → 측위가 틀어졌다. 점구름 전체가 벽에서 떠 있다.")
                print("     이러면 벽이 통째로 장애물이 된다 — 오검의 근본 원인.")
            else:
                print("  → 측위는 맞다. 점구름이 벽에 얹혀 있다.")
                print("     그럼 트인 곳 검출은 진짜 물체이거나 라이다 노이즈다.")

        if not self.ghosts:
            print("\nAUTO 중 검출된 정적 장애물 없음")
        else:
            g = self.ghosts
            dw = np.array([x["d_wall"] for x in g])
            ok = np.isfinite(dw)
            n = max(int(ok.sum()), 1)
            on_wall = int((dw[ok] <= ON_WALL_M).sum())
            near = int(((dw[ok] > ON_WALL_M) & (dw[ok] <= NEAR_WALL_M)).sum())
            far = int((dw[ok] > NEAR_WALL_M).sum())
            print(f"\n[검출 분류] {len(g)}건")
            print(f"  벽 위   (≤{ON_WALL_M:.2f}m)     {on_wall:5d}  {on_wall/n*100:5.1f}%")
            print(f"  벽 근처 ({ON_WALL_M:.2f}~{NEAR_WALL_M:.2f}m) {near:5d}  {near/n*100:5.1f}%")
            print(f"  트인 곳 (>{NEAR_WALL_M:.2f}m)     {far:5d}  {far/n*100:5.1f}%")

            ox = np.array([x["obs_x"] for x in g])
            oy = np.array([x["obs_y"] for x in g])
            dl = np.array([x["d_line"] for x in g])
            rng = np.array([x["range"] for x in g])
            print(f"\n  검출 map 위치 산포 {np.hypot(ox.std(), oy.std()):.2f} m", end="  ")
            print("← 한 곳 고정 (실물체 가능성)" if np.hypot(ox.std(), oy.std()) < 0.5
                  else "← 여기저기 (노이즈 또는 측위 문제)")
            print(f"  주행라인까지 중앙 {np.median(dl):.2f} m, "
                  f"0.5m 이내 {(dl < 0.5).sum()}건")
            print(f"  검출 거리 중앙 {np.median(rng):.2f} m")

            # 같은 자리에 반복해서 뜨는 것은 실물체다.
            cell = {}
            for x, y in zip(ox, oy):
                cell[(round(x * 4) / 4, round(y * 4) / 4)] = (
                    cell.get((round(x * 4) / 4, round(y * 4) / 4), 0) + 1
                )
            top = sorted(cell.items(), key=lambda kv: -kv[1])[:5]
            print("\n  가장 자주 뜬 위치 (0.25m 격자):")
            for (cx, cy), cnt in top:
                d = float(self.grid.wall_distance(cx, cy))
                print(f"    ({cx:+.2f}, {cy:+.2f})  {cnt:3d}회  벽까지 {d:.2f} m")

        self._steer_report()
        print("=" * 70)

        if self.csv_prefix and self.ghosts:
            p = Path(f"{self.csv_prefix}_ghosts.csv")
            cols = list(self.ghosts[0].keys())
            with p.open("w", encoding="utf-8") as f:
                f.write(",".join(cols) + "\n")
                for row in self.ghosts:
                    f.write(",".join(f"{row[c]}" for c in cols) + "\n")
            print(f"원시 검출 {len(self.ghosts)}건 → {p}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    rclpy.init()
    node = Logger(args.csv)
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
