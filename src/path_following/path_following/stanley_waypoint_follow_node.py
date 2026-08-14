#!/usr/bin/env python3
"""
Stanley controller waypoint follower — CSV 슬라이딩 + /local_path override.

Pure Pursuit 버전(waypoint_follow_node)과 별도 executable.
기본 CSV·TF·속도 스케일·회피 게이트 구조는 동일, 조향은 Stanley + 곡률 FF.
"""
from __future__ import annotations

import math
import os
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from path_following.track_sliding import (
    DEFAULT_REVERSE_TRACK,
    LoopTrackSliding,
    apply_track_direction,
    apply_track_direction_scalars,
    load_csv_xyv,
    param_bool,
    resolve_csv_path,
)


# ============================================================
# USER TUNING — Stanley 경로 추종 (여기만 수정)
# ============================================================
CFG = {
    # 주행 라인 선택: "raceline" | "centerline" | "auto" | "" (=track_sliding.DEFAULT_TRACK)
    # 런치로 한 번에 바꾸려면: ros2 launch ... track:=centerline
    # local_planner 와 반드시 같은 값이어야 한다 (런치 인자가 둘 다 세팅).
    "track": "",
    # track 을 무시하고 특정 CSV 를 쓰고 싶을 때만 절대경로 지정
    "csv_path": "",
    # 주행 방향. local_planner 와 반드시 같아야 해서 track_sliding 한 곳에서 온다.
    "reverse_track_direction": DEFAULT_REVERSE_TRACK,
    # ---- 목표 속도 (CSV 3번째 열) ----
    # CSV 에 v 열이 있으면 그 값을 /drive.speed 로 내보낸다. control_node 가
    # use_drive_speed_command=True 면 이 값을 그대로 추종한다.
    # v 열이 없는 구형 CSV 면 speed_fallback_mps 로 전 구간 정속 주행.
    "speed_from_csv": True,
    "speed_fallback_mps": 0.0,   # 0 = 정지 (안전측). 정속 실험은 여기에 값
    "speed_scale": 1.0,          # 실차에서 급하게 줄일 때. CSV 재생성 없이 배율
    "speed_max_mps": 0.0,        # 하드 상한. 0 = 제한 없음
    # 앞으로 이 거리만큼의 최소 속도를 취해 제어·측위 지연을 흡수. 0 = 끔
    "speed_lookahead_m": 0.3,
    # 전략(회피·추월) 배율. local_planner 가 발행. 없으면 1.0
    "speed_scale_topic": "/planner/speed_scale",
    "path_window_size": 140,
    "path_anchor_half_width": 120,
    "map_frame": "map",
    "base_frame": "base_link",
    "tf_lookup_timeout_sec": 0.2,
    "local_path_topic": "/local_path",
    "planner_path_override_topic": "/planner_path_override_active",
    "measured_speed_topic": "/vehicle/speed_mps",
    "measured_speed_stale_sec": 0.3,
    "measured_speed_filter_alpha": 0.25,
    # control_node 텔레메트리: 목표속도/실측/duty 표시용 (속도명령은 control_node 전담)
    "telemetry_topic": "/vehicle/telemetry",
    "telemetry_stale_sec": 0.5,
    "drive_topic": "/drive",
    "tracked_path_topic": "/waypoint_tracked_path",
    "timer_period_ms": 30,
    "max_steering_angle": 0.6981,  # ±40° — control_node / ESP S±1.0 과 동일
    "steering_smooth_alpha": 0.45,
    "wheelbase": 0.33,
    "stanley_k": 0.5,
    "stanley_softening": 0.12,
    # |cte|가 클수록 heading_error 가중치↓ (직선 평행주행 시 상쇄 방지)
    # 0.08 은 실주행 오차보다 훨씬 작아서(실측 |cte| p50=0.13, p95=0.40)
    # 대부분의 시간을 하한 0.25 에 붙어 달렸다 — 곡선에서 밀릴수록 복구력이
    # 약해지는 역효과. 하한 도달점을 p95 부근(0.375 m)으로 옮긴다.
    "stanley_heading_cte_blend_m": 0.5,
    "stanley_heading_min_weight": 0.20,
    # LOCAL_PATH(회피) 전용.
    # heading_gain 은 "경로 헤딩 오차 → 조향" 배율. FGM 각도가 이미 필요한
    # 회피량을 담고 있어서 1.0 을 넘기면 목표점을 지나쳐 과회피가 된다.
    # 목표점 추종(pure pursuit) 등가 게인은 2L/Ld ≈ 0.7 수준.
    "local_path_stanley_k": 1.0,
    "local_path_heading_gain": 0.8,
    "local_path_cte_speed_cap_mps": 1.2,  # 고속에서 cte_term 약화 방지
    "local_path_lookahead_m": 0.70,      # heading 기준을 앞쪽 경로로
    "local_path_steering_smooth_alpha": 0.55,
    "local_path_steering_rate_limit_radps": 10.0,
    # 횡가속 한계 조향 상한 — LOCAL_PATH(회피)에만 적용 (≤0 이면 끔).
    # CSV 추종은 낮은 stanley_k + 곡률 FF 조합으로 이미 튜닝돼 있어서
    # 여기에 상한을 걸면 곡선에서 FF가 잘려 언더스티어가 난다.
    # 5.0 은 IMU 실측 기준: 2.5~2.8 m/s 코너링에서 v·ω 피크가 4.84~5.59 였고
    # 그 지점에서 이미 밀리고 있었다. 그 위를 명령해도 조향이 곡률로 바뀌지 않는다.
    "max_lateral_accel_mps2": 5.0,
    # 곡률 피드포워드: δ = δ_ff(κ) + Stanley. 직선용 stanley_k 는 유지.
    "enable_steer_ff": True,
    "ff_gain": 2.3,              # δ_ff = ff_gain * ff_sign * atan(L·κ)
    "ff_sign": 1.0,              # 좌우 반대면 -1.0
    "ff_lookahead_m": 0.8,       # best_i 기준 앞쪽 평균 곡률 구간 [m]
    "ff_kappa_clip": 2.5,        # |κ| 상한 [1/m] (스파이크 방지)
    "stanley_debug_log_hz": 2.0,
    "status_log_hz": 2.0,
    "planner_gate_stale_sec": 0.15,  # override False 미수신 시 빠르게 CSV 복귀
    "steering_rate_limit_radps": 6.5,
    # Foxglove/모니터용 시각화·디버그 토픽
    "publish_tracked_path": True,
    "publish_stanley_debug_topic": True,
    "publish_control_diagnostics": True,
}


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def closest_point_on_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> Tuple[float, float, float]:
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return ax, ay, 0.0

    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))

    qx = ax + t * abx
    qy = ay + t * aby
    return qx, qy, t


class StanleyWaypointFollowNode(Node):
    def __init__(self):
        super().__init__("stanley_waypoint_follow_node")

        for key, value in CFG.items():
            self.declare_parameter(key, value)

        self.track = self.get_parameter("track").get_parameter_value().string_value
        self.csv_path = resolve_csv_path(
            self.get_parameter("csv_path").get_parameter_value().string_value,
            self.track,
        )
        if not self.csv_path:
            raise RuntimeError("stanley_waypoint_follow_node: csv_path is required.")
        # 해석 결과를 파라미터에 되써서 `ros2 param get ... csv_path` 로 확인 가능하게
        self.set_parameters(
            [Parameter("csv_path", Parameter.Type.STRING, self.csv_path)]
        )

        self.path_window_size = int(self.get_parameter("path_window_size").value)
        self.path_anchor_half_width = int(
            self.get_parameter("path_anchor_half_width").value
        )

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_timeout = float(self.get_parameter("tf_lookup_timeout_sec").value)

        self.local_path_topic = self.get_parameter("local_path_topic").value
        self.gate_topic = self.get_parameter("planner_path_override_topic").value
        self.measured_speed_topic = self.get_parameter("measured_speed_topic").value
        self.telemetry_topic = self.get_parameter("telemetry_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.tracked_path_topic = self.get_parameter("tracked_path_topic").value

        self.timer_period = float(self.get_parameter("timer_period_ms").value) / 1000.0

        self.measured_speed_stale_ns = int(
            float(self.get_parameter("measured_speed_stale_sec").value) * 1e9
        )
        self.measured_speed_filter_alpha = float(
            self.get_parameter("measured_speed_filter_alpha").value
        )
        self.telemetry_stale_ns = int(
            float(self.get_parameter("telemetry_stale_sec").value) * 1e9
        )

        self.max_steering = float(self.get_parameter("max_steering_angle").value)
        self.steering_smooth_alpha = float(
            self.get_parameter("steering_smooth_alpha").value
        )
        self.wheelbase = max(1e-3, float(self.get_parameter("wheelbase").value))

        self.stanley_k = float(self.get_parameter("stanley_k").value)
        self.stanley_softening = float(self.get_parameter("stanley_softening").value)
        self.stanley_heading_cte_blend_m = max(
            1e-3, float(self.get_parameter("stanley_heading_cte_blend_m").value)
        )
        self.stanley_heading_min_weight = max(
            0.0,
            min(1.0, float(self.get_parameter("stanley_heading_min_weight").value)),
        )
        self.local_path_stanley_k = max(
            0.05, float(self.get_parameter("local_path_stanley_k").value)
        )
        self.local_path_heading_gain = max(
            0.1, float(self.get_parameter("local_path_heading_gain").value)
        )
        self.local_path_cte_speed_cap_mps = max(
            0.2, float(self.get_parameter("local_path_cte_speed_cap_mps").value)
        )
        self.local_path_lookahead_m = max(
            0.0, float(self.get_parameter("local_path_lookahead_m").value)
        )
        self.local_path_steering_smooth_alpha = float(
            self.get_parameter("local_path_steering_smooth_alpha").value
        )
        self.local_path_steering_rate_limit_radps = max(
            0.1, float(self.get_parameter("local_path_steering_rate_limit_radps").value)
        )
        self.max_lateral_accel_mps2 = float(
            self.get_parameter("max_lateral_accel_mps2").value
        )
        self.enable_steer_ff = param_bool(self.get_parameter("enable_steer_ff").value)
        self.ff_gain = float(self.get_parameter("ff_gain").value)
        self.ff_sign = float(self.get_parameter("ff_sign").value)
        self.ff_lookahead_m = max(0.0, float(self.get_parameter("ff_lookahead_m").value))
        self.ff_kappa_clip = max(0.0, float(self.get_parameter("ff_kappa_clip").value))

        self.gate_stale_ns = int(
            float(self.get_parameter("planner_gate_stale_sec").value) * 1e9
        )
        self.steering_rate_limit_radps = float(
            self.get_parameter("steering_rate_limit_radps").value
        )

        _ptp = self.get_parameter("publish_tracked_path").value
        self.publish_tracked_path = param_bool(_ptp)
        self.publish_stanley_debug_topic = param_bool(
            self.get_parameter("publish_stanley_debug_topic").value
        )
        self.publish_control_diagnostics = param_bool(
            self.get_parameter("publish_control_diagnostics").value
        )

        csv_points, csv_speeds = load_csv_xyv(self.csv_path)
        reverse_track = param_bool(
            self.get_parameter("reverse_track_direction").value
        )
        csv_points = apply_track_direction(csv_points, reverse_track)
        csv_speeds = apply_track_direction_scalars(csv_speeds, reverse_track)
        if len(csv_points) < 2:
            raise RuntimeError(f"CSV needs at least 2 points: {self.csv_path}")

        self.track = LoopTrackSliding(
            csv_points,
            self.path_window_size,
            self.path_anchor_half_width,
        )
        self._setup_speed_profile(csv_points, csv_speeds)

        self._local_path: List[Tuple[float, float]] = []
        self._path_poses: List[Tuple[float, float]] = []

        self._planner_override_active = False
        self._planner_gate_recv_ns = 0

        self._measured_speed_mps = 0.0
        self._filtered_speed_mps = 0.0
        self._measured_speed_recv_ns = 0
        self._measured_speed_initialized = False

        # control_node /vehicle/telemetry snapshot
        self._ctrl_target_speed_mps = 0.0
        self._ctrl_measured_speed_mps = 0.0
        self._ctrl_vesc_duty = 0.0
        self._ctrl_auto = False
        self._telemetry_recv_ns = 0

        self._last_steering_cmd = 0.0
        self._last_heading_err = 0.0
        self._last_cte_term = 0.0
        self._last_ff_term = 0.0
        self._last_kappa_used = 0.0
        dbg_hz = max(0.0, float(self.get_parameter("stanley_debug_log_hz").value))
        self._stanley_debug_period = 1.0 / dbg_hz if dbg_hz > 0.0 else 0.0
        self._stanley_debug_accum = 0.0
        status_hz = max(0.0, float(self.get_parameter("status_log_hz").value))
        self._status_log_period = 1.0 / status_hz if status_hz > 0.0 else 0.0
        self._status_log_accum = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, self.local_path_topic, self._cb_local_path, 10)
        self.create_subscription(Bool, self.gate_topic, self._cb_planner_gate, 10)
        self.create_subscription(
            Float64,
            self.measured_speed_topic,
            self._cb_measured_speed,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.telemetry_topic,
            self._cb_vehicle_telemetry,
            10,
        )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        self.stanley_debug_pub = (
            self.create_publisher(Float64MultiArray, "/stanley/debug", 10)
            if self.publish_stanley_debug_topic
            else None
        )
        if self.publish_control_diagnostics:
            self.raw_steer_cmd_pub = self.create_publisher(
                Float64, "/control/raw_steer_cmd", 10
            )
            self.filtered_steer_cmd_pub = self.create_publisher(
                Float64, "/control/filtered_steer_cmd", 10
            )
            self.cte_pub = self.create_publisher(
                Float64, "/control/cross_track_error", 10
            )
            self.heading_error_pub = self.create_publisher(
                Float64, "/control/heading_error", 10
            )
            self.path_curvature_pub = self.create_publisher(
                Float64, "/control/path_curvature", 10
            )
        else:
            self.raw_steer_cmd_pub = None
            self.filtered_steer_cmd_pub = None
            self.cte_pub = None
            self.heading_error_pub = None
            self.path_curvature_pub = None

        self.tracked_path_pub = None
        if self.publish_tracked_path:
            self.tracked_path_pub = self.create_publisher(
                Path, self.tracked_path_topic, 10
            )

        self.timer = self.create_timer(self.timer_period, self._timer_cb)

        self.get_logger().info(
            f"Stanley waypoint follower | track=[{os.path.basename(self.csv_path)}] "
            f"CSV={self.csv_path}, "
            f"points={len(csv_points)}, reverse_track={reverse_track}, "
            f"drive={self.drive_topic} (steer + speed), "
            f"speed_src={self._speed_source} "
            f"v[{min(self._speed_profile):.2f}~{max(self._speed_profile):.2f}]m/s "
            f"scale={self._speed_scale_cfg:.2f} "
            f"cap={self._speed_max_mps if self._speed_max_mps > 0 else 'none'}, "
            f"measured_speed={self.measured_speed_topic}, "
            f"telemetry={self.telemetry_topic}, "
            f"stanley_k={self.stanley_k}, soft={self.stanley_softening}, "
            f"hdg_blend={self.stanley_heading_cte_blend_m:.2f}m, "
            f"steer_ff={self.enable_steer_ff} gain={self.ff_gain:.2f} "
            f"sign={self.ff_sign:.1f} lookahead={self.ff_lookahead_m:.2f}m "
            f"kappa_clip={self.ff_kappa_clip:.2f}, "
            f"steering_rate_limit={self.steering_rate_limit_radps:.2f} rad/s"
        )

    def _cb_local_path(self, msg: Path) -> None:
        self._local_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _cb_planner_gate(self, msg: Bool) -> None:
        self._planner_override_active = bool(msg.data)
        self._planner_gate_recv_ns = self.get_clock().now().nanoseconds

    def _cb_vehicle_telemetry(self, msg: Float64MultiArray) -> None:
        """control_node /vehicle/telemetry 스냅샷.

        layout (control_node._publish_telemetry):
          2 current_duty, 6 autonomous, 10 measured_speed, 11 target_speed
        """
        data = msg.data
        if len(data) < 12:
            return
        self._ctrl_vesc_duty = float(data[2])
        self._ctrl_auto = bool(float(data[6]) >= 0.5)
        self._ctrl_measured_speed_mps = abs(float(data[10]))
        self._ctrl_target_speed_mps = abs(float(data[11]))
        self._telemetry_recv_ns = self.get_clock().now().nanoseconds

    def _cb_measured_speed(self, msg: Float64) -> None:
        speed = float(msg.data)
        if not math.isfinite(speed):
            return

        # Stanley 분모에는 진행 방향과 무관한 속력 크기만 사용한다.
        speed = abs(speed)
        if not self._measured_speed_initialized:
            self._filtered_speed_mps = speed
            self._measured_speed_initialized = True
        else:
            alpha = max(0.0, min(1.0, self.measured_speed_filter_alpha))
            self._filtered_speed_mps += alpha * (
                speed - self._filtered_speed_mps
            )

        self._measured_speed_mps = speed
        self._measured_speed_recv_ns = self.get_clock().now().nanoseconds

    def _telemetry_alive(self) -> bool:
        if self._telemetry_recv_ns <= 0:
            return False
        now_ns = self.get_clock().now().nanoseconds
        return now_ns - self._telemetry_recv_ns < self.telemetry_stale_ns

    def _get_control_speed(self) -> Tuple[float, bool]:
        now_ns = self.get_clock().now().nanoseconds
        speed_alive = (
            self._measured_speed_recv_ns > 0
            and now_ns - self._measured_speed_recv_ns
            < self.measured_speed_stale_ns
        )
        if speed_alive and self._measured_speed_initialized:
            return abs(self._filtered_speed_mps), True
        if self._telemetry_alive():
            return abs(self._ctrl_measured_speed_mps), True
        return 0.0, False

    def _get_pose_map(self) -> Tuple[float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)
        return x, y, yaw

    def _select_mode_and_path(self, x: float, y: float) -> str:
        now_ns = self.get_clock().now().nanoseconds

        gate_alive = (
            self._planner_gate_recv_ns > 0
            and now_ns - self._planner_gate_recv_ns < self.gate_stale_ns
        )

        if gate_alive and self._planner_override_active:
            if len(self._local_path) >= 2:
                self._path_poses = list(self._local_path)
                return "LOCAL_PATH"

            self._path_poses = []
            return "STOP"

        self._path_poses = self.track.sliding_xy(x, y)
        return "CSV_TRACKING"

    def _stanley_control(
        self,
        path: List[Tuple[float, float]],
        x: float,
        y: float,
        yaw: float,
        speed: float,
        mode: str,
    ) -> Tuple[float, float, float, float, float, float, float, int]:
        if len(path) < 2:
            return 0.0, 0.0, x, y, 0.0, 0.0, 0.0, 0, 0.0, 0.0

        # Ackermann 조향은 전륜 기준 — PP와 동일 wheelbase
        px = x + self.wheelbase * math.cos(yaw)
        py = y + self.wheelbase * math.sin(yaw)

        best_d2 = float("inf")
        best_i = 0
        best_qx = 0.0
        best_qy = 0.0

        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]

            qx, qy, _ = closest_point_on_segment(px, py, ax, ay, bx, by)
            d2 = (px - qx) ** 2 + (py - qy) ** 2

            if d2 < best_d2:
                best_d2 = d2
                best_i = i
                best_qx = qx
                best_qy = qy

        ax, ay = path[best_i]
        bx, by = path[min(best_i + 1, len(path) - 1)]

        closest_yaw = math.atan2(by - ay, bx - ax)
        path_yaw = closest_yaw

        # LOCAL_PATH: 최근접이 아니라 전방 lookahead 구간의 헤딩을 따라 강하게 추종
        if mode == "LOCAL_PATH" and self.local_path_lookahead_m > 1e-3:
            path_yaw = self._path_yaw_at_lookahead(
                path, best_i, best_qx, best_qy, self.local_path_lookahead_m
            )

        heading_error = wrap_pi(path_yaw - yaw)

        dx = px - best_qx
        dy = py - best_qy

        # CTE는 최근접 세그먼트 기준으로 유지
        right_x = math.sin(closest_yaw)
        right_y = -math.cos(closest_yaw)
        cte = dx * right_x + dy * right_y

        if mode == "LOCAL_PATH":
            k = self.local_path_stanley_k
            eff_speed = min(abs(speed), self.local_path_cte_speed_cap_mps)
            cte_term = math.atan2(
                k * cte,
                eff_speed + self.stanley_softening,
            )
            heading_term = self.local_path_heading_gain * heading_error
        else:
            cte_term = math.atan2(
                self.stanley_k * cte,
                abs(speed) + self.stanley_softening,
            )
            cte_abs = abs(cte)
            hdg_w = max(
                self.stanley_heading_min_weight,
                1.0 - cte_abs / self.stanley_heading_cte_blend_m,
            )
            # 조향 부호 (실차 ESP 서보 실측과 동일, 추가 반전 없음):
            #   +steering = 좌, -steering = 우
            # cte>0(경로 오른쪽) → 좌회전(+)으로 복귀
            heading_term = hdg_w * heading_error

        kappa_used = 0.0
        ff_term = 0.0
        if self.enable_steer_ff and mode != "LOCAL_PATH":
            kappa_used = self._lookahead_curvature(path, best_i)
            # 자전거 모델: δ_ff = atan(L·κ). +κ(좌로 휨) → +조향(좌).
            ff_term = (
                self.ff_gain
                * self.ff_sign
                * math.atan(self.wheelbase * kappa_used)
            )

        steering = ff_term + heading_term + cte_term

        # Stanley 조향값은 원형 각도가 아니라 bounded control input 이므로
        # wrap_pi()로 다시 감싸면 반응이 과하게 휘어질 수 있다.
        steering = max(-self.max_steering, min(self.max_steering, steering))

        return (
            steering,
            cte,
            best_qx,
            best_qy,
            heading_error,
            heading_term,
            cte_term,
            best_i,
            ff_term,
            kappa_used,
        )

    def _maybe_log_stanley_debug(
        self,
        mode: str,
        cte: float,
        heading_error: float,
        cte_term: float,
        ff_term: float,
        kappa_used: float,
        steering: float,
        control_speed: float,
        measured_speed_alive: bool,
    ) -> None:
        if self._stanley_debug_period <= 0.0:
            return
        self._stanley_debug_accum += self.timer_period
        if self._stanley_debug_accum < self._stanley_debug_period:
            return
        self._stanley_debug_accum = 0.0
        speed_source = "MEASURED" if measured_speed_alive else "ZERO_FALLBACK"
        tel = "OK" if self._telemetry_alive() else "STALE"
        self.get_logger().info(
            f"stanley dbg [{mode}]: cte={cte:+.3f}m "
            f"hdg_err={math.degrees(heading_error):+.1f}deg "
            f"cte_term={math.degrees(cte_term):+.1f}deg "
            f"ff={math.degrees(ff_term):+.1f}deg kappa={kappa_used:+.3f} "
            f"steer={math.degrees(steering):+.1f}deg "
            f"v_tgt={self._ctrl_target_speed_mps:.2f} "
            f"v_act={self._ctrl_measured_speed_mps:.2f} "
            f"duty={self._ctrl_vesc_duty:+.3f} tel={tel} "
            f"v_ctrl={control_speed:.2f} speed_source={speed_source}"
        )

    def _maybe_log_status(
        self,
        *,
        pose_ok: bool,
        x: float,
        y: float,
        yaw: float,
        csv_x: float,
        csv_y: float,
        cte: float,
        measured_speed: float,
        control_speed: float,
        steering: float,
        mode: str,
        path_x: float | None = None,
        path_y: float | None = None,
        steering_raw: float | None = None,
    ) -> None:
        if self._status_log_period <= 0.0:
            return

        self._status_log_accum += self.timer_period
        if self._status_log_accum < self._status_log_period:
            return
        self._status_log_accum = 0.0

        if not pose_ok:
            self.get_logger().info("STATUS | TF 없음 (map -> base_link)")
            return

        lat = math.hypot(x - csv_x, y - csv_y)
        steer_deg = math.degrees(steering)
        raw_part = ""
        if steering_raw is not None:
            raw_part = f" steer_raw={math.degrees(steering_raw):+.1f}°"

        if self._telemetry_alive():
            mode_tag = "AUTO" if self._ctrl_auto else "MANUAL"
            speed_part = (
                f"v_tgt={self._ctrl_target_speed_mps:.2f}m/s "
                f"v_act={self._ctrl_measured_speed_mps:.2f}m/s "
                f"duty={self._ctrl_vesc_duty:+.3f} ({mode_tag})"
            )
        else:
            speed_part = (
                f"v_tgt=? v_act={measured_speed:.2f}m/s duty=? (NO_CTRL_TELEM)"
            )

        if mode == "LOCAL_PATH":
            px = path_x if path_x is not None else csv_x
            py = path_y if path_y is not None else csv_y
            self.get_logger().info(
                f"STATUS | LOCAL_PATH | "
                f"veh=({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):+.1f}°) "
                f"path=({px:.2f}, {py:.2f}) "
                f"cte={cte:+.2f}m "
                f"hdg_err={math.degrees(self._last_heading_err):+.1f}° "
                f"cte_term={math.degrees(self._last_cte_term):+.1f}° "
                f"ff={math.degrees(self._last_ff_term):+.1f}° "
                f"kappa={self._last_kappa_used:+.3f} "
                f"{speed_part} v_ctrl={control_speed:.2f}m/s "
                f"steer={steer_deg:+.1f}°{raw_part}"
            )
            return

        self.get_logger().info(
            f"STATUS | veh=({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):+.1f}°) "
            f"csv=({csv_x:.2f}, {csv_y:.2f}) lat={lat:.2f}m cte={cte:+.2f}m "
            f"hdg_err={math.degrees(self._last_heading_err):+.1f}° "
            f"cte_term={math.degrees(self._last_cte_term):+.1f}° "
            f"ff={math.degrees(self._last_ff_term):+.1f}° "
            f"kappa={self._last_kappa_used:+.3f} "
            f"{speed_part} v_ctrl={control_speed:.2f}m/s "
            f"steer={steer_deg:+.1f}° mode={mode}"
        )

    def _path_yaw_at_lookahead(
        self,
        path: List[Tuple[float, float]],
        best_i: int,
        best_qx: float,
        best_qy: float,
        lookahead_m: float,
    ) -> float:
        """Walk forward from closest projection by lookahead_m; return that segment yaw."""
        if len(path) < 2:
            return 0.0
        i = max(0, min(best_i, len(path) - 2))
        cx, cy = best_qx, best_qy
        remain = lookahead_m
        while i < len(path) - 1 and remain > 1e-6:
            nx, ny = path[i + 1]
            seg = math.hypot(nx - cx, ny - cy)
            if seg < 1e-6:
                i += 1
                cx, cy = nx, ny
                continue
            if remain <= seg:
                return math.atan2(ny - cy, nx - cx)
            remain -= seg
            cx, cy = nx, ny
            i += 1
        ax, ay = path[-2]
        bx, by = path[-1]
        return math.atan2(by - ay, bx - ax)

    def _smooth_steering(self, target: float, *, alpha: float | None = None) -> float:
        a = self.steering_smooth_alpha if alpha is None else alpha
        a = max(0.0, min(1.0, a))
        if a <= 0.0:
            return target
        return self._last_steering_cmd + a * (target - self._last_steering_cmd)

    def _rate_limit_steering(
        self, target: float, *, rate_limit_radps: float | None = None
    ) -> float:
        rate = (
            self.steering_rate_limit_radps
            if rate_limit_radps is None
            else rate_limit_radps
        )
        max_step = max(0.0, rate) * self.timer_period
        diff = target - self._last_steering_cmd
        if max_step <= 0.0:
            out = target
        else:
            diff = max(-max_step, min(max_step, diff))
            out = self._last_steering_cmd + diff
        out = max(-self.max_steering, min(self.max_steering, out))
        self._last_steering_cmd = out
        return out

    def _limit_lateral_accel(self, steering: float, speed: float) -> float:
        """a_lat = v²·tan(δ)/L 이 한계를 넘지 않도록 조향 상한 (LOCAL_PATH 전용).

        저속에서는 상한이 max_steering 보다 커서 걸리지 않고, 고속에서만 조인다.
        """
        a_max = self.max_lateral_accel_mps2
        v = abs(speed)
        if a_max <= 0.0 or v < 0.5:
            return steering
        kappa_max = a_max / (v * v)
        delta_max = math.atan(self.wheelbase * kappa_max)
        return max(-delta_max, min(delta_max, steering))

    def _compute_path_curvature(
        self, path: List[Tuple[float, float]], nearest_idx: int
    ) -> float:
        """Calculate signed path curvature near nearest_idx (3-point)."""
        if len(path) < 3 or nearest_idx < 0 or nearest_idx >= len(path) - 2:
            return 0.0
        x0, y0 = path[nearest_idx]
        x1, y1 = path[nearest_idx + 1]
        x2, y2 = path[nearest_idx + 2]
        dx1, dy1 = x1 - x0, y1 - y0
        dx2, dy2 = x2 - x1, y2 - y1
        d1 = math.sqrt(dx1 * dx1 + dy1 * dy1)
        d2 = math.sqrt(dx2 * dx2 + dy2 * dy2)
        if d1 < 1e-6 or d2 < 1e-6:
            return 0.0
        yaw1 = math.atan2(dy1, dx1)
        yaw2 = math.atan2(dy2, dx2)
        dyaw = wrap_pi(yaw2 - yaw1)
        avg_dist = (d1 + d2) / 2.0
        if avg_dist < 1e-6:
            return 0.0
        return dyaw / avg_dist

    def _lookahead_curvature(
        self, path: List[Tuple[float, float]], nearest_idx: int
    ) -> float:
        """best_i 부터 앞쪽 ff_lookahead_m 구간의 평균 signed curvature."""
        if len(path) < 3 or nearest_idx < 0:
            return 0.0

        samples: List[float] = []
        traveled = 0.0
        i = nearest_idx
        max_i = len(path) - 3

        while i <= max_i:
            kappa = self._compute_path_curvature(path, i)
            samples.append(kappa)

            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            traveled += math.hypot(x1 - x0, y1 - y0)
            if traveled >= self.ff_lookahead_m:
                break
            i += 1

        if not samples:
            return 0.0

        kappa_used = sum(samples) / float(len(samples))
        if self.ff_kappa_clip > 0.0:
            kappa_used = max(
                -self.ff_kappa_clip, min(self.ff_kappa_clip, kappa_used)
            )
        return kappa_used

    def _publish_control_diagnostics(
        self,
        raw_steer: float,
        filtered_steer: float,
        cte: float,
        heading_err: float,
        kappa_used: float,
    ) -> None:
        """Publish the existing normalized/scalar control diagnostics."""
        if not self.publish_control_diagnostics:
            return
        if abs(self.max_steering) > 1e-6:
            raw_norm = raw_steer / self.max_steering
            filtered_norm = filtered_steer / self.max_steering
        else:
            raw_norm = 0.0
            filtered_norm = 0.0

        msg = Float64()
        msg.data = float(raw_norm)
        self.raw_steer_cmd_pub.publish(msg)
        msg.data = float(filtered_norm)
        self.filtered_steer_cmd_pub.publish(msg)
        msg.data = float(cte)
        self.cte_pub.publish(msg)
        msg.data = float(heading_err)
        self.heading_error_pub.publish(msg)
        msg.data = float(kappa_used)
        self.path_curvature_pub.publish(msg)

    def _publish_stanley_debug(
        self,
        cte: float,
        heading_error: float,
        heading_term: float,
        cross_track_term: float,
        stanley_fb_sum: float,
        raw_steering: float,
        filtered_or_limited_steering: float,
        speed: float,
        closest_path_index: int,
        kappa_used: float,
        ff_term: float,
        lat_m: float,
        path_x: float,
        path_y: float,
        csv_lat_m: float,
        veh_x: float,
        veh_y: float,
        veh_yaw: float,
    ) -> None:
        """Publish one coherent control-cycle snapshot.

        Float64MultiArray layout and units:
          0 cte [m], 1 heading error [rad], 2 heading term [rad],
          3 cross-track term [rad], 4 Stanley FB sum (hdg+cte) [rad],
          5 raw command after saturation (FF+FB) [rad],
          6 command after smoothing/rate limiting [rad], 7 speed [m/s],
          8 closest path segment index [-],
          9 kappa_used [1/m], 10 delta_ff [rad], 11 total_before_sat [rad],
          12 lat to active path point [m], 13 path_x [m], 14 path_y [m],
          15 lat to CSV raceline point [m],
          16 veh_x [m], 17 veh_y [m], 18 veh_yaw [rad].
        """
        if self.stanley_debug_pub is None:
            return
        total_before_sat = ff_term + stanley_fb_sum
        msg = Float64MultiArray()
        msg.data = [
            float(cte),
            float(heading_error),
            float(heading_term),
            float(cross_track_term),
            float(stanley_fb_sum),
            float(raw_steering),
            float(filtered_or_limited_steering),
            float(speed),
            float(closest_path_index),
            float(kappa_used),
            float(ff_term),
            float(total_before_sat),
            float(lat_m),
            float(path_x),
            float(path_y),
            float(csv_lat_m),
            float(veh_x),
            float(veh_y),
            float(veh_yaw),
        ]
        self.stanley_debug_pub.publish(msg)

    # ------------------------------------------------------------
    # 목표 속도 (CSV 3번째 열)
    # ------------------------------------------------------------
    def _setup_speed_profile(self, csv_points, csv_speeds) -> None:
        """웨이포인트별 목표 속도 배열을 만든다.

        CSV v 열(오프라인에서 곡률·가감속 한계로 계산됨)을 그대로 쓰되,
        speed_lookahead_m 구간의 최솟값을 취해 제어·측위 지연을 흡수한다.
        (감속은 조금 일찍, 가속은 늦게 → 안전측)
        """
        self._speed_from_csv = param_bool(
            self.get_parameter("speed_from_csv").value
        )
        self._speed_fallback_mps = max(
            0.0, float(self.get_parameter("speed_fallback_mps").value)
        )
        self._speed_scale_cfg = max(
            0.0, float(self.get_parameter("speed_scale").value)
        )
        self._speed_max_mps = max(
            0.0, float(self.get_parameter("speed_max_mps").value)
        )
        lookahead_m = max(0.0, float(self.get_parameter("speed_lookahead_m").value))

        n = len(csv_points)
        if self._speed_from_csv and csv_speeds and len(csv_speeds) == n:
            profile = [max(0.0, float(v)) for v in csv_speeds]
            self._speed_source = "csv"
        else:
            profile = [self._speed_fallback_mps] * n
            self._speed_source = (
                "fallback(csv has no v column)"
                if self._speed_from_csv
                else "fallback(speed_from_csv=False)"
            )

        if lookahead_m > 0.0 and n >= 2 and self._speed_source == "csv":
            profile = self._min_over_lookahead(csv_points, profile, lookahead_m)

        self._speed_profile = profile
        # 전략(회피·추월) 배율. local_planner 가 /planner/speed_scale 로 발행.
        self._strategy_speed_scale = 1.0
        scale_topic = str(self.get_parameter("speed_scale_topic").value).strip()
        if scale_topic:
            self.create_subscription(
                Float64, scale_topic, self._speed_scale_cb, 10
            )

    @staticmethod
    def _min_over_lookahead(points, profile, lookahead_m: float):
        """각 점에서 앞으로 lookahead_m 안에 있는 목표 속도의 최솟값."""
        n = len(points)
        seg = [
            math.hypot(
                points[(i + 1) % n][0] - points[i][0],
                points[(i + 1) % n][1] - points[i][1],
            )
            for i in range(n)
        ]
        out = [0.0] * n
        for i in range(n):
            v_min = profile[i]
            travelled = 0.0
            k = i
            while travelled < lookahead_m:
                travelled += seg[k]
                k = (k + 1) % n
                if k == i:
                    break
                if profile[k] < v_min:
                    v_min = profile[k]
            out[i] = v_min
        return out

    def _speed_scale_cb(self, msg: Float64) -> None:
        # 감속만 허용한다. CSV v 는 이미 접지력 한계라 1.0 을 넘겨 올리면
        # 코너에서 그립을 잃는다 (전략 노드는 직선용으로 2.0 을 보내기도 한다).
        value = float(msg.data)
        if math.isfinite(value) and value >= 0.0:
            self._strategy_speed_scale = min(1.0, value)

    def _target_speed_at(self, index: int) -> float:
        """웨이포인트 index 의 목표 속도 [m/s]. 배율·상한 적용 후."""
        if not self._speed_profile:
            return 0.0
        v = self._speed_profile[int(index) % len(self._speed_profile)]
        v *= self._speed_scale_cfg * self._strategy_speed_scale
        if self._speed_max_mps > 0.0:
            v = min(v, self._speed_max_mps)
        return max(0.0, v)

    def _publish_drive(self, steering: float, speed: float = 0.0) -> None:
        """조향 + 목표 속도. control_node 가 use_drive_speed_command 면 speed 를 추종."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(max(0.0, speed))
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def _publish_tracked_path(self) -> None:
        if self.tracked_path_pub is None or len(self._path_poses) < 2:
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        for px, py in self._path_poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.tracked_path_pub.publish(msg)

    def _timer_cb(self) -> None:
        pose = self._get_pose_map()

        if pose is None:
            self._publish_drive(0.0, 0.0)
            self._maybe_log_status(
                pose_ok=False,
                x=0.0,
                y=0.0,
                yaw=0.0,
                csv_x=0.0,
                csv_y=0.0,
                cte=0.0,
                measured_speed=self._filtered_speed_mps,
                control_speed=0.0,
                steering=0.0,
                mode="NO_TF",
            )
            return

        x, y, yaw = pose
        csv_x, csv_y, csv_seg = self.track.closest_projection_on_loop(x, y)
        target_speed = self._target_speed_at(csv_seg)

        mode = self._select_mode_and_path(x, y)

        if mode == "STOP" or len(self._path_poses) < 2:
            self._publish_drive(0.0, 0.0)
            self._maybe_log_status(
                pose_ok=True,
                x=x,
                y=y,
                yaw=yaw,
                csv_x=csv_x,
                csv_y=csv_y,
                cte=0.0,
                measured_speed=self._filtered_speed_mps,
                control_speed=0.0,
                steering=0.0,
                mode=mode,
            )
            return

        self._publish_tracked_path()

        control_speed, measured_speed_alive = self._get_control_speed()

        (
            steering_raw,
            cte,
            path_x,
            path_y,
            heading_err,
            heading_term,
            cte_term,
            closest_path_index,
            ff_term,
            kappa_used,
        ) = self._stanley_control(
            self._path_poses,
            x,
            y,
            yaw,
            control_speed,
            mode,
        )
        self._maybe_log_stanley_debug(
            mode,
            cte,
            heading_err,
            cte_term,
            ff_term,
            kappa_used,
            steering_raw,
            control_speed,
            measured_speed_alive,
        )
        self._last_heading_err = heading_err
        self._last_cte_term = cte_term
        self._last_ff_term = ff_term
        self._last_kappa_used = kappa_used

        if mode == "LOCAL_PATH":
            steering_smoothed = self._smooth_steering(
                steering_raw, alpha=self.local_path_steering_smooth_alpha
            )
            steering_cmd = self._rate_limit_steering(
                self._limit_lateral_accel(steering_smoothed, control_speed),
                rate_limit_radps=self.local_path_steering_rate_limit_radps,
            )
        else:
            # CSV 추종은 횡가속 상한을 걸지 않는다 — 곡선에서 곡률 FF 가 잘려
            # 오히려 언더스티어가 난다 (FF 는 언더스티어 방지용으로 넣은 항).
            steering_smoothed = self._smooth_steering(steering_raw)
            steering_cmd = self._rate_limit_steering(steering_smoothed)

        stanley_fb_sum = heading_term + cte_term
        lat_m = math.hypot(x - path_x, y - path_y)
        csv_lat_m = math.hypot(x - csv_x, y - csv_y)
        try:
            self._publish_stanley_debug(
                cte,
                heading_err,
                heading_term,
                cte_term,
                stanley_fb_sum,
                steering_raw,
                steering_cmd,
                control_speed,
                closest_path_index,
                kappa_used,
                ff_term,
                lat_m,
                path_x,
                path_y,
                csv_lat_m,
                x,
                y,
                yaw,
            )
        except Exception as exc:
            # Telemetry is observational and must never interrupt /drive.
            self.get_logger().warning(f"Failed to publish /stanley/debug: {exc}")

        self._publish_control_diagnostics(
            steering_raw,
            steering_cmd,
            cte,
            heading_err,
            kappa_used,
        )

        self._publish_drive(steering_cmd, target_speed)

        self._maybe_log_status(
            pose_ok=True,
            x=x,
            y=y,
            yaw=yaw,
            csv_x=csv_x,
            csv_y=csv_y,
            cte=cte,
            measured_speed=self._filtered_speed_mps,
            control_speed=control_speed,
            steering=steering_cmd,
            mode=mode,
            path_x=path_x,
            path_y=path_y,
            steering_raw=steering_raw,
        )


def main(args=None):
    rclpy.init(args=args)
    node = StanleyWaypointFollowNode()

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
