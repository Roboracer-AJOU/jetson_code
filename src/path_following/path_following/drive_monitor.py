#!/usr/bin/env python3
"""
실차 회피/속도 디버그 모니터 — 별도 터미널에서 고정 레이아웃 + 숫자만 갱신.

실행 (런치와 다른 터미널):
  source install/setup.bash && ros2 run path_following drive_monitor
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64, Float64MultiArray, String, UInt8
from tf2_ros import Buffer, TransformListener

from path_following import vehicle_geometry as vg


_WHEELBASE_M = vg.WHEELBASE_M


def _rad2deg(r: float) -> float:
    return math.degrees(r)


def _esp_steer_to_servo_deg(norm: float) -> float:
    """ESP: S:+1→140°(실차 좌), S:-1→40°(실차 우)."""
    return 90.0 + float(norm) * 50.0


def _age_str(last_mono: float | None, *, stale: float = 0.5) -> str:
    if last_mono is None:
        return "없음"
    age = time.monotonic() - last_mono
    flag = " STALE" if age > stale else ""
    return f"{age:.2f}s{flag}"


@dataclass
class TopicStamp:
    last_mono: float | None = None
    hz: float = 0.0
    _times: list[float] = field(default_factory=list)

    def mark(self) -> None:
        now = time.monotonic()
        self.last_mono = now
        self._times.append(now)
        cutoff = now - 2.0
        self._times = [t for t in self._times if t >= cutoff]
        if len(self._times) >= 2:
            self.hz = (len(self._times) - 1) / (self._times[-1] - self._times[0])
        else:
            self.hz = 0.0


class DriveMonitor(Node):
    def __init__(self) -> None:
        super().__init__("drive_monitor")
        self.declare_parameter("refresh_hz", 2.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._st_drive = TopicStamp()
        self._drive_speed = 0.0
        self._drive_steer = 0.0
        self.create_subscription(
            AckermannDriveStamped, "/drive", self._cb_drive, 10
        )

        self._st_odom = TopicStamp()
        self._odom_v = 0.0
        self.create_subscription(Odometry, "/odom", self._cb_odom, 10)

        self._st_tel = TopicStamp()
        self._tel: list[float] = []
        self.create_subscription(
            Float64MultiArray, "/vehicle/telemetry", self._cb_telemetry, 10
        )

        self._st_scan = TopicStamp()
        self._scan_min_m = float("inf")
        self._scan_min_deg = 0.0
        self._scan_n = 0
        self.create_subscription(LaserScan, "/scan", self._cb_scan, 10)

        self._st_obs = TopicStamp()
        self._obs_count = 0
        self._obs_nearest_m = float("inf")
        self._obs_nearest_xy = (0.0, 0.0)
        self.create_subscription(
            Float32MultiArray, "/static_obstacles", self._cb_obs, 10
        )

        self._st_dyn = TopicStamp()
        self._dyn_count = 0
        self._dyn_nearest_m = float("inf")
        self._dyn_nearest_xy = (0.0, 0.0)
        self._dyn_nearest_v = 0.0
        self._dyn_nearest_vx = 0.0
        self._dyn_nearest_vy = 0.0
        self._dyn_nearest_closing = 0.0
        self._dyn_tracks: list[tuple[int, float, float, float, float, float, float]] = []
        self.create_subscription(
            Float32MultiArray, "/dynamic_obstacles", self._cb_dyn, 10
        )

        self._st_v_act = TopicStamp()
        self._v_act = 0.0
        self.create_subscription(
            Float64, "/vehicle/speed_mps", self._cb_v_act, 10
        )

        self._st_stanley = TopicStamp()
        self._stanley: list[float] = []
        self.create_subscription(
            Float64MultiArray, "/stanley/debug", self._cb_stanley, 10
        )

        self._st_fgm = TopicStamp()
        self._fgm_x = 0.0
        self._fgm_y = 0.0
        self._fgm_dist = float("inf")
        self._fgm_heading_deg = 0.0
        self.create_subscription(
            PointStamped, "/fgm_target", self._cb_fgm, 10
        )

        self._st_override = TopicStamp()
        self._override = False
        self.create_subscription(
            Bool, "/planner_path_override_active", self._cb_override, 10
        )

        self._st_planner_mode = TopicStamp()
        self._planner_mode = "?"
        self.create_subscription(String, "/planner/mode", self._cb_planner_mode, 10)

        self._st_local_path = TopicStamp()
        self._local_path_n = 0
        self.create_subscription(Path, "/local_path", self._cb_local_path, 10)

        self._st_speed_scale = TopicStamp()
        self._planner_speed_scale = 1.0
        self.create_subscription(
            Float64, "/planner/speed_scale", self._cb_speed_scale, 10
        )

        self._st_strategy = TopicStamp()
        self._strategy_mul = 1.0
        self.create_subscription(
            Float64, "/strategy/speed_multiplier", self._cb_strategy, 10
        )

        self._st_speed_cond = TopicStamp()
        self._speed_cond = 0
        self.create_subscription(
            UInt8, "/planner/speed_condition", self._cb_speed_cond, 10
        )

        self._st_imu = TopicStamp()
        self._imu_yaw_rate = 0.0
        self._imu_a_lat = 0.0
        self._peak_a_lat_imu = 0.0
        self._peak_a_lat_kin = 0.0
        self._peak_yaw_rate = 0.0
        self.create_subscription(Imu, "/imu/data", self._cb_imu, 20)

        self._last_tf_xy: tuple[float, float] | None = None
        self._last_tf_mono: float | None = None
        self._tf_speed = 0.0
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw_deg = 0.0
        self._tf_ok = False

        hz = float(self.get_parameter("refresh_hz").value)
        self.create_timer(1.0 / max(0.5, hz), self._refresh)

    def _cb_drive(self, msg: AckermannDriveStamped) -> None:
        self._st_drive.mark()
        self._drive_speed = float(msg.drive.speed)
        self._drive_steer = float(msg.drive.steering_angle)

    def _cb_odom(self, msg: Odometry) -> None:
        self._st_odom.mark()
        self._odom_v = float(msg.twist.twist.linear.x)

    def _cb_imu(self, msg: Imu) -> None:
        self._st_imu.mark()
        self._imu_yaw_rate = float(msg.angular_velocity.z)
        self._imu_a_lat = float(msg.linear_acceleration.y)
        # 최대치 유지 — 그립 한계(max_lateral_accel_mps2) 튜닝 근거
        v = abs(self._v_act)
        if v >= 0.5:
            self._peak_a_lat_imu = max(self._peak_a_lat_imu, abs(self._imu_a_lat))
            self._peak_a_lat_kin = max(
                self._peak_a_lat_kin, abs(v * self._imu_yaw_rate)
            )
            self._peak_yaw_rate = max(self._peak_yaw_rate, abs(self._imu_yaw_rate))

    def _cb_telemetry(self, msg: Float64MultiArray) -> None:
        self._st_tel.mark()
        self._tel = list(msg.data)

    def _cb_scan(self, msg: LaserScan) -> None:
        self._st_scan.mark()
        self._scan_n = len(msg.ranges)
        min_r = float("inf")
        min_deg = 0.0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            ang = _rad2deg(msg.angle_min + i * msg.angle_increment)
            if abs(ang) > 60.0:
                continue
            if r < min_r:
                min_r = r
                min_deg = ang
        self._scan_min_m = min_r
        self._scan_min_deg = min_deg

    def _cb_obs(self, msg: Float32MultiArray) -> None:
        self._st_obs.mark()
        data = msg.data
        n = len(data) // 4
        self._obs_count = n
        nearest = float("inf")
        nxy = (0.0, 0.0)
        for i in range(n):
            base = i * 4
            x = float(data[base + 1])
            y = float(data[base + 2])
            d = math.hypot(x, y)
            if d < nearest:
                nearest = d
                nxy = (x, y)
        self._obs_nearest_m = nearest
        self._obs_nearest_xy = nxy

    def _cb_dyn(self, msg: Float32MultiArray) -> None:
        self._st_dyn.mark()
        data = msg.data
        n = len(data) // 6
        self._dyn_count = n
        nearest = float("inf")
        nxy = (0.0, 0.0)
        nv = nvx = nvy = nclose = 0.0
        tracks: list[tuple[int, float, float, float, float, float, float]] = []
        for i in range(n):
            base = i * 6
            oid = int(data[base])
            x = float(data[base + 1])
            y = float(data[base + 2])
            vx = float(data[base + 3])
            vy = float(data[base + 4])
            r = float(data[base + 5])
            speed = math.hypot(vx, vy)
            rng = math.hypot(x, y)
            closing = -(x * vx + y * vy) / rng if rng > 1e-3 else 0.0
            tracks.append((oid, x, y, speed, vx, vy, closing))
            d = math.hypot(x, y) - r
            if d < nearest:
                nearest = max(0.0, d)
                nxy = (x, y)
                nv, nvx, nvy, nclose = speed, vx, vy, closing
        tracks.sort(key=lambda t: math.hypot(t[1], t[2]))
        self._dyn_tracks = tracks[:4]
        self._dyn_nearest_m = nearest
        self._dyn_nearest_xy = nxy
        self._dyn_nearest_v = nv
        self._dyn_nearest_vx = nvx
        self._dyn_nearest_vy = nvy
        self._dyn_nearest_closing = nclose

    def _cb_v_act(self, msg: Float64) -> None:
        self._st_v_act.mark()
        v = float(msg.data)
        if math.isfinite(v):
            self._v_act = abs(v)

    def _cb_stanley(self, msg: Float64MultiArray) -> None:
        self._st_stanley.mark()
        self._stanley = list(msg.data)

    def _cb_fgm(self, msg: PointStamped) -> None:
        self._st_fgm.mark()
        self._fgm_x = float(msg.point.x)
        self._fgm_y = float(msg.point.y)
        self._fgm_dist = math.hypot(self._fgm_x, self._fgm_y)
        self._fgm_heading_deg = _rad2deg(math.atan2(self._fgm_y, self._fgm_x))

    def _cb_override(self, msg: Bool) -> None:
        self._st_override.mark()
        self._override = bool(msg.data)

    def _cb_planner_mode(self, msg: String) -> None:
        self._st_planner_mode.mark()
        self._planner_mode = msg.data.strip() or "?"

    def _cb_local_path(self, msg: Path) -> None:
        self._st_local_path.mark()
        self._local_path_n = len(msg.poses)

    def _cb_speed_scale(self, msg: Float64) -> None:
        self._st_speed_scale.mark()
        self._planner_speed_scale = float(msg.data)

    def _cb_strategy(self, msg: Float64) -> None:
        self._st_strategy.mark()
        self._strategy_mul = float(msg.data)

    def _cb_speed_cond(self, msg: UInt8) -> None:
        self._st_speed_cond.mark()
        self._speed_cond = int(msg.data)

    def _update_tf(self) -> None:
        try:
            tf = self._tf_buf.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            self._tf_ok = False
            return

        self._tf_ok = True
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._pose_x = x
        self._pose_y = y
        self._pose_yaw_deg = _rad2deg(yaw)

        now = time.monotonic()
        if self._last_tf_xy is not None and self._last_tf_mono is not None:
            dt = now - self._last_tf_mono
            if dt > 1e-3:
                dx = x - self._last_tf_xy[0]
                dy = y - self._last_tf_xy[1]
                self._tf_speed = math.hypot(dx, dy) / dt
        self._last_tf_xy = (x, y)
        self._last_tf_mono = now

    def _stanley_follow_label(self) -> str:
        if not self._tf_ok:
            return "NO_TF"
        if self._override and self._local_path_n >= 2:
            return "LOCAL_PATH"
        if self._override:
            return "STOP(override, no path)"
        return "CSV_TRACKING"

    def _planner_mode_ko(self) -> str:
        m = self._planner_mode.upper()
        return {
            "GLOBAL": "CSV 직진 (GLOBAL)",
            "AVOID": "회피 (AVOID)",
            "REJOIN": "CSV 복귀 (REJOIN)",
            "TRAILING": "갭 유지 추종 (TRAILING)",
        }.get(m, m)

    def _control_mode_ko(self) -> str:
        if len(self._tel) >= 7:
            if self._tel[7] >= 0.5:
                return "ESTOP"
            if self._tel[6] >= 0.5:
                return "AUTO (CH5 자율)"
            return "MANUAL (CH5 수동)"
        return "control_node 없음"

    def _refresh(self) -> None:
        self._update_tf()
        lines: list[str] = []
        w = 78
        lines.append("=" * w)
        lines.append(" F1TENTH 회피/속도 모니터  (Ctrl+C 종료)  — 숫자만 실시간 갱신")
        lines.append("=" * w)

        lines.append("[ 모드 ]")
        lines.append(f"  차량 제어(AUTO/MANUAL) : {self._control_mode_ko()}")
        if len(self._tel) >= 10:
            lines.append(
                f"  RC CH5(모드 스위치)   : {self._tel[5]:.0f} us  "
                f"(CH1조향={self._tel[8]:.0f} CH2쓰로틀={self._tel[9]:.0f})"
            )
        elif len(self._tel) >= 6:
            lines.append(f"  RC CH5(모드 스위치)   : {self._tel[5]:.0f} us")
        lines.append(
            f"  Planner(회피상태)     : {self._planner_mode_ko()}  "
            f"(override={self._override})"
        )
        lines.append(
            f"  Stanley 추종(경로선택): {self._stanley_follow_label()}"
        )
        lines.append(
            f"  local_path(회피경로)  : {self._local_path_n} pts  "
            f"(age {_age_str(self._st_local_path.last_mono)})"
        )

        lines.append("")
        lines.append("[ 속도 · 상대속도 ]")
        v_ego = self._v_act
        if self._st_v_act.last_mono is None and len(self._tel) >= 11:
            v_ego = abs(float(self._tel[10]))
        v_obs = self._dyn_nearest_v if self._dyn_count > 0 else 0.0
        v_rel = self._dyn_nearest_closing if self._dyn_count > 0 else 0.0
        threat = (
            "APPROACH(가까워짐)"
            if self._dyn_count > 0 and v_rel > 0.0
            else ("RECEDE(멀어짐)" if self._dyn_count > 0 and v_rel < 0.0 else "—")
        )
        lines.append(
            f"  v_ego(내차속도)       : {v_ego:6.2f} m/s  "
            f"(age {_age_str(self._st_v_act.last_mono, stale=0.5)})"
        )
        if self._dyn_count > 0:
            lines.append(
                f"  v_obs(장애물속력)     : {v_obs:6.2f} m/s  "
                f"vx={self._dyn_nearest_vx:+.2f} vy={self._dyn_nearest_vy:+.2f}"
            )
            lines.append(
                f"  v_rel(상대·가까워짐+) : {v_rel:+6.2f} m/s  "
                f"(+가까움/-멀어짐)  threat={threat}"
            )
        else:
            lines.append("  v_obs(장애물속력)     : — (동적 장애 없음)")
            lines.append("  v_rel(접근속도)       : —")
        if len(self._tel) >= 12:
            lines.append(
                f"  v_tgt(목표속도)/duty  : {self._tel[11]:.2f} m/s  "
                f"duty={self._tel[2]:+.3f}"
            )
        lines.append(
            f"  속도배율(감속배수)    : strategy×{self._strategy_mul:.2f}  "
            f"planner×{self._planner_speed_scale:.2f}  "
            f"cond(조건코드)={self._speed_cond}"
        )

        lines.append("")
        lines.append("[ Stanley ]")
        s = self._stanley
        if len(s) >= 11:
            cte = s[0]
            hdg_err = s[1]
            hdg_term = s[2]
            cte_term = s[3]
            fb_sum = s[4]
            steer_raw = s[5]
            steer_filt = s[6]
            v_ctrl = s[7]
            closest_idx = int(s[8])
            kappa = s[9]
            ff = s[10]
            total_pre = s[11] if len(s) >= 12 else (ff + fb_sum)
            lat = s[12] if len(s) >= 13 else float("nan")
            path_x = s[13] if len(s) >= 14 else float("nan")
            path_y = s[14] if len(s) >= 15 else float("nan")
            csv_lat = s[15] if len(s) >= 16 else float("nan")
            lines.append(
                f"  cte(횡방향오차·부호)  : {cte:+.3f} m  "
                f"(age {_age_str(self._st_stanley.last_mono, stale=0.5)})"
            )
            if math.isfinite(lat):
                lines.append(
                    f"  lat(경로점까지거리)   : {lat:.3f} m  "
                    f"← 현재 추종 경로점과의 거리"
                )
            if math.isfinite(csv_lat):
                lines.append(
                    f"  csv_lat(레이스라인거리): {csv_lat:.3f} m  "
                    f"← CSV 레이스라인 최근접점"
                )
            if math.isfinite(path_x) and math.isfinite(path_y):
                lines.append(
                    f"  path(추종목표점)      : ({path_x:.2f}, {path_y:.2f}) m"
                )
            lines.append(
                f"  hdg_err(헤딩오차)     : {_rad2deg(hdg_err):+.1f}°  "
                f"({hdg_err:+.3f} rad)"
            )
            lines.append(
                f"  cte_term(횡오차조향)  : {_rad2deg(cte_term):+.1f}°   "
                f"hdg_term(헤딩조향) {_rad2deg(hdg_term):+.1f}°   "
                f"ff(곡률피드포워드) {_rad2deg(ff):+.1f}°"
            )
            lines.append(
                f"  fb_sum(피드백합)      : {_rad2deg(fb_sum):+.1f}°   "
                f"total_pre_sat(포화전합) {_rad2deg(total_pre):+.1f}°"
            )
            lines.append(
                f"  kappa(경로곡률)       : {kappa:+.3f} 1/m   "
                f"closest_idx(최근접인덱스)={closest_idx}   "
                f"v_ctrl(스탠리속도)={v_ctrl:.2f} m/s"
            )
            lines.append(
                f"  steer_raw(조향원시)   : {_rad2deg(steer_raw):+.1f}°  "
                f"→ filtered(필터후) {_rad2deg(steer_filt):+.1f}°  "
                f"→ /drive {_rad2deg(self._drive_steer):+.1f}°"
            )
            lines.append(
                f"  추종모드(경로소스)    : {self._stanley_follow_label()}"
            )
        else:
            lines.append(
                f"  /stanley/debug(없음)  : —  "
                f"(age {_age_str(self._st_stanley.last_mono)})"
            )

        lines.append("")
        lines.append("[ 조향 · VESC ]")
        lines.append(
            f"  /drive(조향명령)      : {_rad2deg(self._drive_steer):+.1f}°  "
            f"({self._drive_steer:+.3f} rad)"
        )
        if len(self._tel) >= 5:
            esp = self._tel[4]
            lines.append(
                f"  ESP S(서보정규화)     : {esp:+.3f}  "
                f"(서보 약 {_esp_steer_to_servo_deg(esp):.0f}°)"
            )
        else:
            lines.append("  ESP S(서보정규화)     : — (control_node 필요)")
        if len(self._tel) >= 19:
            lines.append(
                f"  VESC PI(속도제어)     : "
                f"v_act(실측)={self._tel[10]:+.2f}  "
                f"v_tgt(목표)={self._tel[11]:.2f}  "
                f"err(오차)={self._tel[12]:+.2f}  "
                f"duty={self._tel[2]:+.3f}"
            )
            lines.append(
                f"  VESC 전원             : "
                f"motor {self._tel[17]:+.2f} A, "
                f"in {self._tel[16]:+.2f} A, {self._tel[18]:.1f} V"
            )

        lines.append("")
        lines.append("[ 그립 · 요레이트 (IMU) ]")
        if self._st_imu.last_mono is None:
            lines.append("  /imu/data             : 미수신 (localization 런치 확인)")
        else:
            v = abs(self._v_act)
            # 자전거 모델 기대 요레이트: ω = v·tanδ/L
            w_exp = v * math.tan(self._drive_steer) / _WHEELBASE_M
            w_meas = self._imu_yaw_rate
            if abs(w_exp) > 0.15:
                ratio = w_meas / w_exp
                if ratio < 0.6:
                    verdict = "언더스티어(안돌아감·감속필요)"
                elif ratio > 1.4:
                    verdict = "오버스티어(뒤가흐름)"
                else:
                    verdict = "정상추종"
                ratio_s = f"{ratio:.2f}  {verdict}"
            else:
                ratio_s = "— (조향/속도 작음)"
            lines.append(
                f"  요레이트 실측/기대     : {w_meas:+.2f} / {w_exp:+.2f} rad/s"
                f"   age {_age_str(self._st_imu.last_mono, stale=0.3)}"
            )
            lines.append(f"  추종비(실측÷기대)     : {ratio_s}")
            lines.append(
                f"  횡가속 IMU/운동학      : {self._imu_a_lat:+.2f} / "
                f"{v * w_meas:+.2f} m/s²   ← 운동학 = v×요레이트"
            )
            lines.append(
                f"  횡가속 최대(누적)      : IMU {self._peak_a_lat_imu:.2f}  "
                f"운동학 {self._peak_a_lat_kin:.2f} m/s²  "
                f"(요레이트 최대 {self._peak_yaw_rate:.2f} rad/s)"
            )
            lines.append(
                f"  → max_lateral_accel_mps2 는 운동학 최대의 0.8배 권장 "
                f"({self._peak_a_lat_kin * 0.8:.1f})"
            )

        lines.append("")
        lines.append("[ 위치 (map) ]")
        if self._tf_ok:
            lines.append(
                f"  pose(차량위치·자세)   : "
                f"x={self._pose_x:.2f} y={self._pose_y:.2f}  "
                f"yaw={self._pose_yaw_deg:+.1f}°"
            )
        else:
            lines.append("  pose(차량위치·자세)   : TF map→base_link 없음")

        lines.append("")
        lines.append("[ LiDAR / 장애물 ]")
        scan_hz = f"{self._st_scan.hz:.1f} Hz" if self._st_scan.hz > 0 else "—"
        lines.append(
            f"  /scan(라이다)         : {scan_hz}  n={self._scan_n}  "
            f"age {_age_str(self._st_scan.last_mono, stale=0.3)}"
        )
        if math.isfinite(self._scan_min_m):
            lines.append(
                f"  전방최소거리(±60°)    : "
                f"{self._scan_min_m:.2f} m @ {self._scan_min_deg:+.0f}°"
            )
        else:
            lines.append("  전방최소거리(±60°)    : —")
        if self._obs_count > 0:
            ox, oy = self._obs_nearest_xy
            lines.append(
                f"  static(정적장애)      : {self._obs_count}개  "
                f"최근접 {self._obs_nearest_m:.2f} m  "
                f"laser({ox:+.2f},{oy:+.2f})  "
                f"age {_age_str(self._st_obs.last_mono)}"
            )
        else:
            lines.append(
                f"  static(정적장애)      : 0  "
                f"age {_age_str(self._st_obs.last_mono)}"
            )
        if self._dyn_count > 0:
            dx, dy = self._dyn_nearest_xy
            lines.append(
                f"  dynamic(동적장애)     : {self._dyn_count}개  "
                f"최근접 {self._dyn_nearest_m:.2f} m  "
                f"v={self._dyn_nearest_v:.2f}m/s  "
                f"laser({dx:+.2f},{dy:+.2f})  "
                f"age {_age_str(self._st_dyn.last_mono)}"
            )
            for oid, x, y, sp, vx, vy, closing in self._dyn_tracks:
                lines.append(
                    f"    · id={oid:<3d}  laser({x:+.2f},{y:+.2f})  "
                    f"v={sp:.2f}  close={closing:+.2f}  "
                    f"vx={vx:+.2f} vy={vy:+.2f}"
                )
        else:
            lines.append(
                f"  dynamic(동적장애)     : 0  "
                f"age {_age_str(self._st_dyn.last_mono)}"
            )
        if math.isfinite(self._fgm_dist):
            lines.append(
                f"  /fgm_target(회피목표) : {self._fgm_dist:.2f} m  "
                f"heading {self._fgm_heading_deg:+.0f}°  "
                f"laser({self._fgm_x:.2f},{self._fgm_y:.2f})  "
                f"age {_age_str(self._st_fgm.last_mono)}"
            )
        else:
            lines.append(
                f"  /fgm_target(회피목표) : —  "
                f"age {_age_str(self._st_fgm.last_mono)}"
            )

        lines.append("")
        lines.append("=" * w)

        if sys.stdout.isatty():
            sys.stdout.write("\033[2J\033[H")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = DriveMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
