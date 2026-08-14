#!/usr/bin/env python3
"""
로컬 플래너: **제어(웨이포인트)의 기본 주행 경로는 건드리지 않음** — 디스크 CSV는 회피 꼬리 합치기·디버깅용만.

- `drive_strategy` 가 내는 `/strategy/speed_multiplier`·`/strategy/speed_condition` 을 받아
  `{0.5,1,2}` 로 스냅한 **`/planner/speed_scale`** 과 조건 코드를 웨이포인트에 전달(전략 브리지).

- 매 주기 `planner_path_override_topic`(기본 **`/planner_path_override_active`**, std_msgs/Bool)
  로 알림: **False** = 장애/전략 개입 없음 → 웨이포인트가 **자기 CSV**만 따라가면 됨.
  **True** = 지금 회피·재합류 궤적을 `/local_path` 로 내고 있으니 웨이포인트가 그거 사용.
- GLOBAL/AVOID/REJOIN 상태 머신으로 회피·Frenet Quintic 재합류를 관리.

`static_obstacle_node` 는 맵잔차 장애 검출, `fgm_node` 는 **회피 주 경로(갭)**.
REJOIN 은 CSV 복귀 보조. **게이트·AVOID 타이밍은 이 파일 CFG.**

CSV 전 코스 시각화(선택): `csv_track_viz_topic`(기본 `/raceline_csv_path`).

디버그(회피 시): 슬라이딩/송신 Path, 앵커 점 등.
"""
from __future__ import annotations

import bisect
import math
import os
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import UInt8
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException

from path_following.obstacle_filter import (
    closest_dynamic_obstacle_speed_mps,
    closest_dynamic_obstacle_surface_m,
    closest_obstacle_surface_m,
    csv_path_blocked_by_obstacles,
    filter_dynamic_obstacles_for_exit,
    filter_dynamic_obstacles_laser_frame,
    filter_obstacles_for_exit,
    filter_obstacles_laser_frame,
    obstacles_remain_for_avoid,
    _pack_dynamic_as_static_gate,
)
from path_following.track_sliding import (
    DEFAULT_REVERSE_TRACK,
    LoopTrackSliding,
    apply_track_direction,
    apply_track_direction_scalars,
    load_csv_xyv,
    param_bool,
    resolve_csv_path,
)
from .avoidance_safety import (
    AvoidSpeedParams,
    InflatedMap,
    avoid_speed_limit,
    first_blocked_index,
    trim_back,
)

# ============================================================
# USER TUNING — local_planner (실차: 장애·LOCAL_PATH·FGM 타이밍은 여기만)
# ============================================================
CFG = {
    # 주행 라인 선택: "raceline" | "centerline" | "auto" | "" (=track_sliding.DEFAULT_TRACK)
    # 런치로 한 번에 바꾸려면: ros2 launch ... track:=centerline
    # stanley_waypoint_follow_node 와 반드시 같은 값이어야 한다.
    "track": "",
    # track 을 무시하고 특정 CSV 를 쓰고 싶을 때만 절대경로 지정
    "csv_path": "",
    # 주행 방향. stanley 와 반드시 같아야 해서 track_sliding 한 곳에서 온다.
    # 어긋나면 Frenet s 가 역방향이 되어 선감속·rejoin 이 차 뒤를 본다.
    "reverse_track_direction": DEFAULT_REVERSE_TRACK,
    "static_obstacles_topic": "/static_obstacles",
    "dynamic_obstacles_topic": "/dynamic_obstacles",
    "ego_speed_topic": "/vehicle/speed_mps",  # control_node 실측 속도
    "fgm_target_topic": "/fgm_target",
    "local_path_topic": "/local_path",
    "planner_path_override_topic": "/planner_path_override_active",
    "raceline_corridor_enable": True,
    "corridor_max_lateral_from_raceline_m": 0.40,
    "obstacle_forward_min_m": 0.30,
    "obstacle_forward_max_m": 12.0,
    "obstacle_lateral_abs_max_m": 0.42,
    "obstacle_tf_timeout_sec": 0.15,
    "laser_to_base_x_m": 0.275,
    # [차량 버블] 뒷축→전방 길이. 게이트 거리 d에서 빼서 앞범퍼 기준으로 회피.
    # 폭(ego_safety_width)은 fgm_node 에서 섹터 반폭으로 사용.
    "ego_front_safety_m": 0.30,
    "use_fgm": True,
    # 회피 거리 베이스 (avoid_timing_ref_mps 기준). 실제 게이트는 속도×마진으로 스케일.
    # 예: v=2m/s, margin=1.3 → avoid_on ≈ 3.5×1.3 = 4.55m (기존보다 ~30% 일찍)
    "avoid_on_m": 3.5,
    "avoid_off_m": 5.0,
    "fgm_enable_m": 8.0,
    "avoid_timing_ref_mps": 2.0,
    "avoid_timing_margin": 1.30,
    "avoid_on_min_m": 2.8,
    # 고속에서 회피를 얼마나 일찍 켤지의 상한. 장애물 검출 상한
    # (obstacle_forward_max_m) 과 맞춰 둔다 — 더 크게 잡아도 안 보인다.
    "avoid_on_max_m": 12.0,
    "avoid_off_min_m": 3.8,
    "avoid_off_max_m": 9.0,
    "fgm_enable_min_m": 5.0,
    "fgm_enable_max_m": 12.0,
    "fgm_enable_topic": "/planner/fgm_enable",
    "avoid_on_count_th": 1,
    "avoid_off_count_th": 3,
    "forward_cone_deg": 75.0,
    "avoid_min_forward_x_m": 0.2,
    "avoid_trigger_lateral_abs_max_m": 0.55,
    "fgm_target_stale_sec": 0.18,
    "avoid_exit_use_passed": True,
    "avoid_pass_rear_x_m": -1.20,
    "avoid_exit_lateral_abs_max_m": 2.80,
    "avoid_exit_use_trigger_cone": False,
    "exit_require_csv_clear": True,
    "exit_csv_clear_lookahead_m": 2.5,
    "exit_csv_clear_radius_m": 0.45,
    "avoid_forward_step_m": 0.15,
    "avoid_forward_num_points": 30,
    "rejoin_enable": False,
    "rejoin_min_length_m": 0.50,
    "rejoin_time_sec": 0.8,
    "rejoin_max_length_m": 0.70,
    "rejoin_sample_count": 30,
    "rejoin_tail_count": 40,
    "rejoin_finish_lateral_m": 0.20,
    "rejoin_finish_require_heading": False,
    "rejoin_finish_heading_deg": 15.0,
    "avoid_skip_rejoin_if_cte_ok": False,
    "rejoin_speed_scale": 0.5,
    # ---- 회피 경로 충돌검사 ----
    # 회피 경로는 FGM 목표점 너머로 avoid_forward_num_points 만큼 직선 연장된다.
    # 그 구간은 아무도 검사한 적이 없어서 코너에서는 그대로 벽을 향한다.
    # 맵과 장애물로 잘라낸다. 맵이 없으면 장애물 검사만 동작한다.
    "path_check_enable": True,
    "map_topic": "/map",
    "path_check_inflation_m": 0.25,   # 차량 반폭 + 여유. 이만큼 벽에서 떨어져야 통과
    "path_check_obstacle_margin_m": 0.10,
    "path_check_backoff_m": 0.20,     # 충돌 지점에서 이만큼 더 물러나 끝냄
    "path_check_min_length_m": 0.6,   # 잘라낸 경로가 이보다 짧으면 회피를 포기
    # ---- 회피 구간 속도 ----
    # 회피 중엔 CSV 속도가 의미 없다 (레이싱라인 곡률 기준으로 뽑은 값이라).
    # 아래 물리값으로 매 주기 목표속도를 구해 CSV 대비 배율로 내보낸다.
    # avoid_speed_enable=False 면 기존처럼 rejoin_speed_scale 일괄 적용.
    "avoid_speed_enable": True,
    "avoid_a_lat_mps2": 4.0,      # 회피 조향에서 허용할 횡가속도
    "avoid_a_brake_mps2": 3.0,    # 회피 실패 시 정지에 쓸 감속도 (AEB 보다 보수적)
    "avoid_safety_factor": 0.7,   # 센서 지연·추종 오차 몫. 낮출수록 느리고 안전
    "avoid_standoff_m": 0.35,     # 장애물 앞 최소 이격
    "avoid_lateral_margin_m": 0.10,
    "avoid_speed_min_mps": 0.6,   # 이 아래로는 안 줄인다 (기어가지 않게)
    "avoid_speed_ref_mps": 2.0,   # CSV 에 속도 열이 없을 때 쓸 기준속도
    "avoid_merge_tail_max": 180,
    "publish_hz": 40.0,
    "path_window_size": 140,
    "path_anchor_half_width": 120,
    "map_frame": "map",
    "laser_frame": "laser",
    "base_frame": "base_link",
    "publish_planner_debug": False,
    "publish_planner_anchor": False,
    "planner_sliding_path_topic": "/local_planner_sliding_path",
    "planner_output_path_topic": "/local_planner_sent_path",
    "planner_anchor_topic": "/local_planner_track_anchor",
    "publish_csv_track_viz": True,
    "csv_track_viz_topic": "/raceline_csv_path",
    "csv_track_viz_hz": 2.0,
    "csv_track_viz_stride": 1,
    # drive_strategy 미사용 시 브리지/20Hz 재발행 끔 (코드는 유지)
    "strategy_bridge_enable": False,
    "strategy_speed_multiplier_topic": "/strategy/speed_multiplier",
    "strategy_speed_condition_topic": "/strategy/speed_condition",
    "planner_speed_scale_out_topic": "/planner/speed_scale",
    "planner_speed_condition_out_topic": "/planner/speed_condition",
    "planner_mode_topic": "/planner/mode",
    "status_log_hz": 0.0,  # ego/obs/rel 속도 STATUS (0=끔)
    "verbose_logs": False,
}

def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def _quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def _point_laser_to_map(
    px: float,
    py: float,
    tx: float,
    ty: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
) -> Tuple[float, float]:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    cx = math.cos(yaw)
    sx = math.sin(yaw)
    mx = cx * px - sx * py + tx
    my = sx * px + cx * py + ty
    return (mx, my)

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner_node")

        for key, value in CFG.items():
            self.declare_parameter(key, value)

        csv_path = resolve_csv_path(
            self.get_parameter("csv_path").get_parameter_value().string_value,
            self.get_parameter("track").get_parameter_value().string_value,
        )
        # 해석 결과를 파라미터에 되써서 `ros2 param get ... csv_path` 로 확인 가능하게
        self.set_parameters([Parameter("csv_path", Parameter.Type.STRING, csv_path)])
        obs_topic = self.get_parameter("static_obstacles_topic").value
        dyn_obs_topic = str(self.get_parameter("dynamic_obstacles_topic").value)
        odom_topic = str(self.get_parameter("ego_speed_topic").value)
        fgm_topic = self.get_parameter("fgm_target_topic").value
        out_topic = self.get_parameter("local_path_topic").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self._raceline_corridor_enable = param_bool(
            self.get_parameter("raceline_corridor_enable").value
        )
        self._corridor_max_lat = max(
            0.05,
            float(self.get_parameter("corridor_max_lateral_from_raceline_m").value),
        )
        self._obstacle_forward_min_m = float(
            self.get_parameter("obstacle_forward_min_m").value
        )
        self._obstacle_forward_max_m = float(
            self.get_parameter("obstacle_forward_max_m").value
        )
        self._obstacle_lateral_abs_max_m = max(
            0.05, float(self.get_parameter("obstacle_lateral_abs_max_m").value)
        )
        self._obstacle_tf_timeout = float(
            self.get_parameter("obstacle_tf_timeout_sec").value
        )

        self.avoid_on_m = float(self.get_parameter("avoid_on_m").value)
        self.avoid_off_m = float(self.get_parameter("avoid_off_m").value)
        if self.avoid_off_m <= self.avoid_on_m:
            self.avoid_off_m = self.avoid_on_m + 0.3
        self.fgm_enable_m = max(
            self.avoid_on_m,
            float(self.get_parameter("fgm_enable_m").value),
        )
        self.avoid_timing_ref_mps = max(
            0.3, float(self.get_parameter("avoid_timing_ref_mps").value)
        )
        self.avoid_timing_margin = max(
            0.5, float(self.get_parameter("avoid_timing_margin").value)
        )
        self.avoid_on_min_m = max(0.5, float(self.get_parameter("avoid_on_min_m").value))
        self.avoid_on_max_m = max(
            self.avoid_on_min_m, float(self.get_parameter("avoid_on_max_m").value)
        )
        self.avoid_off_min_m = max(0.5, float(self.get_parameter("avoid_off_min_m").value))
        self.avoid_off_max_m = max(
            self.avoid_off_min_m, float(self.get_parameter("avoid_off_max_m").value)
        )
        self.fgm_enable_min_m = max(
            0.5, float(self.get_parameter("fgm_enable_min_m").value)
        )
        self.fgm_enable_max_m = max(
            self.fgm_enable_min_m, float(self.get_parameter("fgm_enable_max_m").value)
        )
        self.avoid_on_count_th = max(
            1, int(self.get_parameter("avoid_on_count_th").value)
        )
        self.avoid_off_count_th = max(
            1, int(self.get_parameter("avoid_off_count_th").value)
        )
        self.rejoin_enable = param_bool(self.get_parameter("rejoin_enable").value)
        self.rejoin_min_length_m = max(
            0.15, float(self.get_parameter("rejoin_min_length_m").value)
        )
        self.rejoin_time_sec = max(
            0.1, float(self.get_parameter("rejoin_time_sec").value)
        )
        self.rejoin_max_length_m = max(
            self.rejoin_min_length_m,
            float(self.get_parameter("rejoin_max_length_m").value),
        )
        self.rejoin_sample_count = max(
            2, int(self.get_parameter("rejoin_sample_count").value)
        )
        self.rejoin_tail_count = max(
            0, int(self.get_parameter("rejoin_tail_count").value)
        )
        self.rejoin_finish_lateral_m = max(
            0.02, float(self.get_parameter("rejoin_finish_lateral_m").value)
        )
        self.rejoin_finish_require_heading = param_bool(
            self.get_parameter("rejoin_finish_require_heading").value
        )
        self.rejoin_finish_heading_rad = math.radians(
            max(1.0, float(self.get_parameter("rejoin_finish_heading_deg").value))
        )
        self.avoid_skip_rejoin_if_cte_ok = param_bool(
            self.get_parameter("avoid_skip_rejoin_if_cte_ok").value
        )
        self.rejoin_speed_scale = max(
            0.05, float(self.get_parameter("rejoin_speed_scale").value)
        )

        g = self.get_parameter
        self.path_check_enable = param_bool(g("path_check_enable").value)
        self.path_check_inflation_m = max(
            0.0, float(g("path_check_inflation_m").value)
        )
        self.path_check_obstacle_margin_m = max(
            0.0, float(g("path_check_obstacle_margin_m").value)
        )
        self.path_check_min_length_m = max(
            0.0, float(g("path_check_min_length_m").value)
        )
        self.path_check_backoff_m = max(0.0, float(g("path_check_backoff_m").value))
        self.avoid_speed_enable = param_bool(g("avoid_speed_enable").value)
        self.avoid_speed_ref_mps = max(0.1, float(g("avoid_speed_ref_mps").value))
        self.avoid_speed_params = AvoidSpeedParams(
            a_lat=float(g("avoid_a_lat_mps2").value),
            a_brake=float(g("avoid_a_brake_mps2").value),
            safety_factor=float(g("avoid_safety_factor").value),
            standoff_m=float(g("avoid_standoff_m").value),
            ego_half_width_m=0.5 * float(g("obstacle_lateral_abs_max_m").value),
            ego_front_m=float(g("ego_front_safety_m").value),
            lateral_margin_m=float(g("avoid_lateral_margin_m").value),
            v_min=float(g("avoid_speed_min_mps").value),
        )
        self._inflated_map: InflatedMap | None = None
        self._map_warned = False
        self._last_avoid_speed = float("nan")
        self._last_avoid_reason = ""
        self._last_path_cut = 0
        self._last_block_warn_ns = 0
        self._last_pose_for_speed: PoseStamped | None = None
        self._speed_static_obs: list = []
        self._speed_dynamic_obs: list = []
        self._slew_prev_v: float | None = None
        self._slew_prev_ns = 0
        self._override_active = False

        self.use_fgm = param_bool(self.get_parameter("use_fgm").value)
        cone_deg = float(self.get_parameter("forward_cone_deg").value)
        self.forward_cone_rad = math.radians(cone_deg)
        self.avoid_min_forward_x_m = max(
            0.0, float(self.get_parameter("avoid_min_forward_x_m").value)
        )
        _alat = self.get_parameter("avoid_trigger_lateral_abs_max_m").value
        self.avoid_trigger_lateral_abs_max_m = max(0.1, float(_alat))
        self.fgm_target_stale_ns = int(
            max(0.05, float(self.get_parameter("fgm_target_stale_sec").value)) * 1e9
        )
        self._avoid_exit_use_trigger_cone = param_bool(
            self.get_parameter("avoid_exit_use_trigger_cone").value
        )
        self._avoid_exit_use_passed = param_bool(
            self.get_parameter("avoid_exit_use_passed").value
        )
        self.avoid_pass_rear_x_m = float(
            self.get_parameter("avoid_pass_rear_x_m").value
        )
        self.avoid_exit_lateral_abs_max_m = max(
            self._obstacle_lateral_abs_max_m,
            float(self.get_parameter("avoid_exit_lateral_abs_max_m").value),
        )
        self.laser_to_base_x_m = max(
            0.0, float(self.get_parameter("laser_to_base_x_m").value)
        )
        self.ego_front_safety_m = max(
            0.0, float(self.get_parameter("ego_front_safety_m").value)
        )
        self.exit_require_csv_clear = param_bool(
            self.get_parameter("exit_require_csv_clear").value
        )
        self.exit_csv_clear_lookahead_m = max(
            0.0, float(self.get_parameter("exit_csv_clear_lookahead_m").value)
        )
        self.exit_csv_clear_radius_m = max(
            0.05, float(self.get_parameter("exit_csv_clear_radius_m").value)
        )
        self.avoid_forward_step_m = max(
            0.05, float(self.get_parameter("avoid_forward_step_m").value)
        )
        self.avoid_forward_num_points = max(
            2, int(self.get_parameter("avoid_forward_num_points").value)
        )
        self._last_tf_warn_ns = 0
        self.map_frame = self.get_parameter("map_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.avoid_merge_tail_max = max(
            50, int(self.get_parameter("avoid_merge_tail_max").value)
        )
        self.path_window_size = max(10, int(self.get_parameter("path_window_size").value))
        self.path_anchor_half_width = max(
            30, int(self.get_parameter("path_anchor_half_width").value)
        )
        self.verbose_logs = param_bool(self.get_parameter("verbose_logs").value)
        self._status_log_hz = max(0.0, float(self.get_parameter("status_log_hz").value))
        self._status_log_period = (
            1.0 / self._status_log_hz if self._status_log_hz > 0.0 else 0.0
        )
        self._status_log_accum = 0.0
        self._last_obs_speed_mps = 0.0
        self._last_rel_speed_mps = 0.0
        self._last_d_dyn_closest = float("inf")

        self.points: List[Tuple[float, float]] = []

        if not csv_path:
            raise RuntimeError("local_planner: csv_path is required.")
        reverse_track = param_bool(
            self.get_parameter("reverse_track_direction").value
        )
        csv_points, csv_speeds = load_csv_xyv(csv_path)
        self.points = apply_track_direction(csv_points, reverse_track)
        # 회피 감속을 "CSV 속도 대비 배율" 로 내보내려면 기준 속도를 알아야 한다.
        self.csv_speeds = apply_track_direction_scalars(csv_speeds, reverse_track)
        if len(self.points) < 2:
            raise RuntimeError(
                f"local_planner: csv_path needs ≥2 points: {csv_path} ({len(self.points)})"
            )
        self.track = LoopTrackSliding(
            self.points, self.path_window_size, self.path_anchor_half_width
        )
        self._build_loop_geometry()
        self.get_logger().info(
            f"CSV track loaded: [{os.path.basename(csv_path)}] {csv_path} "
            f"({len(self.points)} pts), "
            f"window={self.path_window_size}, anchor_half_width={self.path_anchor_half_width}"
        )
        self._obstacle_data: list = []
        self._dynamic_obstacle_data: list = []
        self._ego_speed_mps: float = 0.0
        self._fgm_target: PointStamped | None = None
        self._last_obs_recv_ns: int = 0
        self._last_fgm_recv_ns: int = 0
        self._last_latency_log_ns: int = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_obs = self.create_subscription(
            Float32MultiArray, obs_topic, self.cb_static_obstacles, 10
        )
        if dyn_obs_topic:
            self.sub_dyn_obs = self.create_subscription(
                Float32MultiArray,
                dyn_obs_topic,
                self.cb_dynamic_obstacles,
                10,
            )
        self.sub_ego_speed = self.create_subscription(
            Float64, odom_topic, self.cb_ego_speed, 10
        )
        self.sub_fgm = self.create_subscription(
            PointStamped, fgm_topic, self.cb_fgm_target, 10
        )
        if self.path_check_enable:
            # 맵은 latch 로 한 번만 오므로 transient_local 이어야 늦게 떠도 받는다
            self.sub_map = self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("map_topic").value),
                self.cb_map,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
        gate_topic = self.get_parameter("planner_path_override_topic").value
        self.pub_override_gate = self.create_publisher(Bool, gate_topic, 10)
        self.pub_path = self.create_publisher(Path, out_topic, 10)

        _sbe = self.get_parameter("strategy_bridge_enable").value
        self._strategy_bridge_enable = param_bool(_sbe)
        st_mul = self.get_parameter("strategy_speed_multiplier_topic").value
        st_cond = self.get_parameter("strategy_speed_condition_topic").value
        out_sc = self.get_parameter("planner_speed_scale_out_topic").value
        out_co = self.get_parameter("planner_speed_condition_out_topic").value
        self.pub_planner_speed_scale = self.create_publisher(Float64, out_sc, 10)
        self.pub_planner_speed_condition = self.create_publisher(UInt8, out_co, 10)
        mode_topic = self.get_parameter("planner_mode_topic").value
        self.pub_planner_mode = self.create_publisher(String, mode_topic, 10)
        fgm_en_topic = self.get_parameter("fgm_enable_topic").value
        self.pub_fgm_enable = self.create_publisher(Bool, fgm_en_topic, 10)
        self._strategy_mul_recv = 1.0
        self._strategy_cond_recv = 0
        self.mode = "GLOBAL"
        self._avoid_on_count = 0
        self._avoid_off_count = 0
        self._rejoin_path_msg: Path | None = None
        self._rejoin_target_s: float | None = None
        self._last_mode_log_ns = 0
        self._last_avoid_warn_ns = 0
        if self._strategy_bridge_enable:
            self.create_subscription(Float64, st_mul, self._cb_strategy_multiplier, 10)
            self.create_subscription(UInt8, st_cond, self._cb_strategy_condition, 10)
            self.create_timer(0.05, self._republish_planner_speed)

        _dbg = self.get_parameter("publish_planner_debug").value
        self.publish_planner_debug = (
            _dbg if isinstance(_dbg, bool) else str(_dbg).lower() in ("1", "true", "yes")
        )
        sliding_t = self.get_parameter("planner_sliding_path_topic").value
        output_t = self.get_parameter("planner_output_path_topic").value
        self.pub_sliding_dbg = (
            self.create_publisher(Path, sliding_t, 10)
            if self.publish_planner_debug
            else None
        )
        self.pub_sent_dbg = (
            self.create_publisher(Path, output_t, 10)
            if self.publish_planner_debug
            else None
        )
        _anca = self.get_parameter("publish_planner_anchor").value
        self.publish_planner_anchor = (
            _anca
            if isinstance(_anca, bool)
            else str(_anca).lower() in ("1", "true", "yes")
        )
        anch_t = self.get_parameter("planner_anchor_topic").value
        self.pub_anchor = (
            self.create_publisher(PointStamped, anch_t, 10)
            if self.publish_planner_anchor
            else None
        )

        _pcv = self.get_parameter("publish_csv_track_viz").value
        self.publish_csv_track_viz = (
            _pcv if isinstance(_pcv, bool) else str(_pcv).lower() in ("1", "true", "yes")
        )
        self.pub_csv_track = None  # rclpy Publisher for full CSV Path viz
        self._csv_viz_stride = max(1, int(self.get_parameter("csv_track_viz_stride").value))
        csv_viz_hz = float(self.get_parameter("csv_track_viz_hz").value)
        csv_viz_topic = self.get_parameter("csv_track_viz_topic").value
        if self.publish_csv_track_viz:
            self.pub_csv_track = self.create_publisher(Path, csv_viz_topic, 10)
            self.create_timer(
                1.0 / max(csv_viz_hz, 0.1), self._publish_csv_track_viz
            )
        self._need_sliding_for_debug = (
            self.publish_planner_debug or self.publish_planner_anchor
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_hz, 1.0), self.timer_publish
        )
        dbg_bits = ""
        if self.publish_planner_debug:
            dbg_bits += f", dbg_sliding->{sliding_t}, dbg_sent->{output_t}"
        if self.publish_planner_anchor:
            dbg_bits += f", anchor->{anch_t}"
        if self.publish_csv_track_viz:
            dbg_bits += f", csv_track_viz->{csv_viz_topic}@{csv_viz_hz}Hz stride={self._csv_viz_stride}"
        self.get_logger().info(
            f"Local planner: gate `{gate_topic}`, out={out_topic}, "
            f"corridor≤{self._corridor_max_lat}m fwd=[{self._obstacle_forward_min_m},"
            f"{self._obstacle_forward_max_m}]m, "
            f"avoid_on_base={self.avoid_on_m}m@{self.avoid_timing_ref_mps:.1f}m/s "
            f"×{self.avoid_timing_margin:.2f} "
            f"fgm_base={self.fgm_enable_m}m->{fgm_en_topic}, "
            f"dynamic={dyn_obs_topic or 'OFF'}, ego_speed={odom_topic}, "
            f"cone={cone_deg}deg, rejoin={self.rejoin_enable}, use_fgm={self.use_fgm}"
            + dbg_bits
            + (
                f", strategy_bridge->{out_sc},{out_co}"
                if self._strategy_bridge_enable
                else ""
            )
        )

    def _lookup_laser_to_map_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                self.laser_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._obstacle_tf_timeout),
            )
        except TransformException:
            return None

    def _filter_obstacles_for_planner(self, raw: list) -> list:
        corridor_on, laser_to_map = self._corridor_lookup(warn=True)
        if corridor_on and laser_to_map is None:
            return []
        return filter_obstacles_laser_frame(
            raw,
            forward_min_m=self._obstacle_forward_min_m,
            forward_max_m=self._obstacle_forward_max_m,
            lateral_abs_max_m=self._obstacle_lateral_abs_max_m,
            corridor_enable=corridor_on,
            corridor_max_lat_m=self._corridor_max_lat,
            track_pts=self.points,
            laser_to_map=laser_to_map,
            require_corridor_tf=True,
        )

    def _filter_obstacles_for_exit(self, raw: list) -> list:
        """회피 해제용: 코리도 안 장애만 (벽 raw 제외)."""
        corridor_on, laser_to_map = self._corridor_lookup()
        if corridor_on and laser_to_map is None:
            return []
        return filter_obstacles_for_exit(
            raw,
            pass_rear_x_m=self.avoid_pass_rear_x_m,
            lateral_abs_max_m=self.avoid_exit_lateral_abs_max_m,
            corridor_enable=corridor_on,
            corridor_max_lat_m=self._corridor_max_lat,
            track_pts=self.points,
            laser_to_map=laser_to_map,
        )

    def _make_laser_to_map_fn(self, tf_lm):
        tr = tf_lm.transform if tf_lm is not None else None

        def laser_to_map(lx: float, ly: float):
            if tr is None:
                return None
            return _point_laser_to_map(
                lx,
                ly,
                tr.translation.x,
                tr.translation.y,
                tr.rotation.w,
                tr.rotation.x,
                tr.rotation.y,
                tr.rotation.z,
            )

        return laser_to_map

    def _corridor_lookup(self, *, warn: bool = False):
        """(코리도 ON 여부, laser→map 함수). TF 실패면 (True, None)."""
        corridor_on = self._raceline_corridor_enable and len(self.points) >= 2
        if not corridor_on:
            return False, None
        tf_lm = self._lookup_laser_to_map_transform()
        if tf_lm is None:
            if warn:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self._last_tf_warn_ns > 2_000_000_000:
                    self.get_logger().warn(
                        f"TF {self.map_frame}<-{self.laser_frame} 실패 — "
                        "코리도 필수: 회피 게이트 장애 없음으로 처리(벽 오검 방지)."
                    )
                    self._last_tf_warn_ns = now_ns
            return True, None
        return True, self._make_laser_to_map_fn(tf_lm)

    def _filter_dynamic_for_planner(self, raw: list) -> list:
        corridor_on, laser_to_map = self._corridor_lookup()
        if corridor_on and laser_to_map is None:
            return []
        return filter_dynamic_obstacles_laser_frame(
            raw,
            forward_min_m=self._obstacle_forward_min_m,
            forward_max_m=self._obstacle_forward_max_m,
            lateral_abs_max_m=self._obstacle_lateral_abs_max_m,
            corridor_enable=corridor_on,
            corridor_max_lat_m=self._corridor_max_lat,
            track_pts=self.points,
            laser_to_map=laser_to_map,
            require_corridor_tf=True,
        )

    def _filter_dynamic_for_exit(self, raw: list) -> list:
        corridor_on, laser_to_map = self._corridor_lookup()
        if corridor_on and laser_to_map is None:
            return []
        return filter_dynamic_obstacles_for_exit(
            raw,
            pass_rear_x_m=self.avoid_pass_rear_x_m,
            lateral_abs_max_m=self.avoid_exit_lateral_abs_max_m,
            corridor_enable=corridor_on,
            corridor_max_lat_m=self._corridor_max_lat,
            track_pts=self.points,
            laser_to_map=laser_to_map,
        )

    def _dynamic_threat_metrics(
        self, filtered_dynamic: list
    ) -> tuple[float, float, float, float]:
        """
        Returns (d_closest_cone, d_gate, rel_speed_mps, obs_speed_mps)
        for closest dynamic obstacle.

        rel_speed = closing rate (+가까워짐 / -멀어짐).
        |v_ego|-|v_obs| 가 아니라 laser-frame 거리 변화 기반.
        """
        if len(filtered_dynamic) < 6:
            return float("inf"), float("inf"), 0.0, 0.0

        d_closest, obs_speed, closing = closest_dynamic_obstacle_speed_mps(
            filtered_dynamic,
            forward_cone_rad=self.forward_cone_rad,
            min_forward_x_m=self.avoid_min_forward_x_m,
            lateral_abs_max_m=self.avoid_trigger_lateral_abs_max_m,
            laser_to_base_x_m=self.laser_to_base_x_m,
        )
        d_gate = closest_dynamic_obstacle_surface_m(
            filtered_dynamic,
            forward_cone_rad=None,
            min_forward_x_m=self.avoid_min_forward_x_m,
            lateral_abs_max_m=self._obstacle_lateral_abs_max_m,
            laser_to_base_x_m=self.laser_to_base_x_m,
        )
        return d_closest, d_gate, closing, obs_speed

    def _dynamic_obstacles_remain(self, filtered_dynamic: list, rel_speed: float) -> bool:
        if rel_speed <= 0.0:
            return False
        exit_dyn = self._filter_dynamic_for_exit(self._dynamic_obstacle_data)
        gate = _pack_dynamic_as_static_gate(exit_dyn)
        if len(gate) < 4:
            return False
        return obstacles_remain_for_avoid(
            gate,
            pass_rear_x_m=self.avoid_pass_rear_x_m,
            lateral_abs_max_m=self.avoid_exit_lateral_abs_max_m,
        )

    def _static_obstacles_remain(self) -> bool:
        exit_obs = self._filter_obstacles_for_exit(self._obstacle_data)
        if len(exit_obs) < 4:
            return False
        return obstacles_remain_for_avoid(
            exit_obs,
            pass_rear_x_m=self.avoid_pass_rear_x_m,
            lateral_abs_max_m=self.avoid_exit_lateral_abs_max_m,
        )

    def _obstacles_remain(self, filtered: list) -> bool:
        """
        AVOID 유지용. 정적 + 동적(상대속도>0) 장애 후방 통과 전까지 True.
        """
        if self._static_obstacles_remain():
            return True
        filtered_dynamic = self._filter_dynamic_for_planner(self._dynamic_obstacle_data)
        _, _, rel_speed, _ = self._dynamic_threat_metrics(filtered_dynamic)
        return self._dynamic_obstacles_remain(filtered_dynamic, rel_speed)

    def _speed_scaled_dist(self, base_m: float, min_m: float, max_m: float) -> float:
        """ref 속도에서의 base 거리를 (v/ref)*margin 으로 스케일 후 clamp."""
        v = max(0.0, float(self._ego_speed_mps))
        # 정지/극저속에서도 최소 게이트는 유지 (너무 늦게 켜지지 않게)
        scale = self.avoid_timing_margin * (max(v, 0.5) / self.avoid_timing_ref_mps)
        return max(min_m, min(max_m, base_m * scale))

    def _effective_avoid_gates(self) -> tuple[float, float, float]:
        """속도 기반 (avoid_on, avoid_off, fgm_enable) [m]."""
        on_m = self._speed_scaled_dist(
            self.avoid_on_m, self.avoid_on_min_m, self.avoid_on_max_m
        )
        off_m = self._speed_scaled_dist(
            self.avoid_off_m, self.avoid_off_min_m, self.avoid_off_max_m
        )
        if off_m <= on_m:
            off_m = on_m + 0.3
        fgm_m = self._speed_scaled_dist(
            self.fgm_enable_m, self.fgm_enable_min_m, self.fgm_enable_max_m
        )
        fgm_m = max(fgm_m, on_m)
        return on_m, off_m, fgm_m

    def _nose_adjusted_dist(self, d: float) -> float:
        """뒷축 기준 표면거리 → 앞범퍼 여유(ego_front_safety)만큼 더 가까운 것으로 취급."""
        if not math.isfinite(d):
            return d
        return max(0.0, float(d) - self.ego_front_safety_m)

    def _static_wants_fgm_local_path(
        self, filtered: list, d_closest: float, d_gate: float
    ) -> bool:
        if len(filtered) < 4:
            return False
        if self._static_obstacles_remain():
            return True
        _, _, fgm_m = self._effective_avoid_gates()
        if math.isfinite(d_gate) and d_gate <= fgm_m:
            return True
        if math.isfinite(d_closest) and d_closest <= fgm_m:
            return True
        return False

    def _dynamic_wants_fgm_local_path(
        self, filtered_dynamic: list, d_dyn_closest: float, rel_speed: float
    ) -> bool:
        if len(filtered_dynamic) < 6:
            return False
        if rel_speed <= 0.0:
            return False
        if not math.isfinite(d_dyn_closest):
            return False
        on_m, _, _ = self._effective_avoid_gates()
        return d_dyn_closest <= on_m

    def _avoidance_fully_cleared(
        self, filtered: list, current_pose: PoseStamped | None
    ) -> bool:
        """장애 후방 통과 + (옵션) 전방 CSV 클리어 — 둘 다 만족해야 REJOIN."""
        if self._obstacles_remain(filtered):
            return False
        if self.exit_require_csv_clear and self._csv_ahead_blocked(current_pose):
            return False
        return True

    def _csv_ahead_blocked(self, current_pose: PoseStamped | None) -> bool:
        if not self.exit_require_csv_clear or current_pose is None:
            return False
        # 코리도 안 장애만 — 벽이 CSV 근처라고 계속 blocked 되면 안 됨
        corridor_obs = self._filter_obstacles_for_planner(self._obstacle_data)
        exit_obs = self._filter_obstacles_for_exit(self._obstacle_data)
        obs = exit_obs if len(exit_obs) >= 4 else corridor_obs
        if len(obs) < 4 or len(self.points) < 2:
            return False

        tf_lm = self._lookup_laser_to_map_transform()
        if tf_lm is None:
            return False
        tr = tf_lm.transform

        def laser_to_map(lx: float, ly: float):
            return _point_laser_to_map(
                lx,
                ly,
                tr.translation.x,
                tr.translation.y,
                tr.rotation.w,
                tr.rotation.x,
                tr.rotation.y,
                tr.rotation.z,
            )

        return csv_path_blocked_by_obstacles(
            obs,
            track_pts=self.points,
            vehicle_xy=(
                float(current_pose.pose.position.x),
                float(current_pose.pose.position.y),
            ),
            laser_to_map=laser_to_map,
            lookahead_m=self.exit_csv_clear_lookahead_m,
            clear_radius_m=self.exit_csv_clear_radius_m,
        )

    def cb_map(self, msg: OccupancyGrid) -> None:
        """맵 수신 → 차폭만큼 부풀린 클리어런스 격자 생성 (수신 시 1회)."""
        try:
            self._inflated_map = InflatedMap(msg, self.path_check_inflation_m)
        except Exception as exc:  # 맵이 깨져도 플래너는 살아 있어야 한다
            self.get_logger().error(f"map inflation failed: {exc}")
            self._inflated_map = None
            return
        self.get_logger().info(
            f"path check map ready — {msg.info.width}x{msg.info.height} "
            f"@{msg.info.resolution:.3f}m/px, inflation={self.path_check_inflation_m:.2f}m"
        )

    def _obstacle_disks_map(self, tf_lm) -> list:
        """장애물을 맵 좌표 원판 [(x, y, r), ...] 으로. 반경엔 차폭이 포함된다."""
        if tf_lm is None:
            return []
        to_map = self._make_laser_to_map_fn(tf_lm)
        grow = (
            self.avoid_speed_params.ego_half_width_m
            + self.path_check_obstacle_margin_m
        )
        disks = []
        obs = self._obstacle_data
        for k in range(0, max(0, len(obs) - 3), 4):
            mx, my = to_map(float(obs[k + 1]), float(obs[k + 2]))
            disks.append((mx, my, float(obs[k + 3]) + grow))
        dyn = self._dynamic_obstacle_data
        for k in range(0, max(0, len(dyn) - 5), 6):
            mx, my = to_map(float(dyn[k + 1]), float(dyn[k + 2]))
            disks.append((mx, my, float(dyn[k + 5]) + grow))
        return disks

    def _truncate_path_at_collision(self, path: Path, tf_lm) -> tuple[Path, bool]:
        """회피 경로를 첫 충돌 지점 앞에서 자른다. (경로, 쓸만한가).

        FGM 목표점 너머 직선 연장이 벽을 향하는 경우가 이걸로 걸린다.
        남은 길이가 너무 짧으면 회피 자체를 포기한다 — 그 짧은 경로를 주면
        Stanley 가 끝점에서 이상하게 돌고, 차라리 CSV 로 두고 AEB 에 맡기는
        편이 안전하다.
        """
        if not self.path_check_enable or len(path.poses) < 2:
            return path, True

        if self._inflated_map is None and not self._map_warned:
            # 조용히 벽 검사만 빠지면 "검사하고 있다" 고 착각하게 된다
            self._map_warned = True
            self.get_logger().warn(
                "path_check 켜져 있는데 /map 이 아직 없음 — 장애물 검사만 동작하고 "
                "벽 검사는 빠진다. map_server 를 띄우거나 path_check_enable=false."
            )

        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        cut = first_blocked_index(
            pts,
            self._inflated_map,
            self._obstacle_disks_map(tf_lm),
            start_index=1,
        )
        self._last_path_cut = cut
        if cut >= len(pts):
            return path, True

        kept = trim_back(pts, cut, self.path_check_backoff_m)
        length = 0.0
        for i in range(1, kept):
            length += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])

        if length < self.path_check_min_length_m:
            return path, False

        path.poses = path.poses[:kept]
        return path, True

    def _warn_avoid_path_blocked(self) -> None:
        """회피 경로가 통째로 막힘 (1초에 한 번). 감속·정지는 속도정책과 AEB 몫."""
        now = self.get_clock().now().nanoseconds
        if now - getattr(self, "_last_block_warn_ns", 0) < 1_000_000_000:
            return
        self._last_block_warn_ns = now
        self.get_logger().warn(
            f"회피 경로가 {self._last_path_cut}번째 점에서 막힘 — 쓸 만한 길이가 "
            "안 나와 회피 포기, CSV 유지. 감속 후 AEB 가 받는다."
        )

    def _csv_speed_near(self, x: float, y: float) -> float:
        """현재 위치에서 가장 가까운 CSV 웨이포인트의 목표속도 [m/s].

        회피 감속을 배율로 내보내야 해서 기준값이 필요하다. 속도 열이 없는
        구형 CSV 면 avoid_speed_ref_mps 로 대신한다.
        """
        if not self.csv_speeds:
            return self.avoid_speed_ref_mps
        d2 = (self._xs_np - x) ** 2 + (self._ys_np - y) ** 2
        v = float(self.csv_speeds[int(np.argmin(d2))])
        return v if v > 0.05 else self.avoid_speed_ref_mps

    def _avoid_speed_scale(
        self, current: PoseStamped | None, *, avoiding: bool
    ) -> float:
        """속도 배율. 물리로 목표속도를 구해 CSV 속도로 나눈다.

        avoiding=False 는 접근 구간(GLOBAL) 선감속. 조향 한계는 빼고 거리
        기반만 건다.
        """
        if not self.avoid_speed_enable:
            return self.rejoin_speed_scale if avoiding else 1.0

        # FGM 목표점을 차량 기준 (전방, 횡) 으로 — 조향이 얼마나 급한지가 여기서 나온다
        fwd, lat = 2.0, 0.0
        tgt = self._fgm_target_fresh()
        if tgt is not None:
            # /fgm_target 은 laser frame 이라 그대로 전방/횡으로 쓸 수 있다
            fwd = max(0.1, float(tgt.point.x))
            lat = float(tgt.point.y)
        else:
            # 목표가 없거나 오래됐으면 조향 한계를 걸 근거가 없다
            avoiding = False

        v_target, reason = avoid_speed_limit(
            self._speed_static_obs,
            self._speed_dynamic_obs,
            self._ego_speed_mps,
            fwd,
            lat,
            self.avoid_speed_params,
            laser_to_base_x_m=self.laser_to_base_x_m,
            include_maneuver=avoiding,
        )
        v_target = self._slew_limit_speed(v_target)
        self._last_avoid_speed = v_target
        self._last_avoid_reason = reason

        v_csv = self.avoid_speed_ref_mps
        if current is not None:
            v_csv = self._csv_speed_near(
                float(current.pose.position.x), float(current.pose.position.y)
            )
        return min(1.0, v_target / max(0.05, v_csv))

    def _slew_limit_speed(self, v_target: float) -> float:
        """감속 명령이 차가 낼 수 있는 감속도를 넘지 않게 완만화.

        장애물이 검출 범위에 처음 들어오는 순간 목표속도가 뚝 떨어지는데,
        그대로 내보내면 못 따라가는 명령이라 속도 PI 가 포화되고 적분이
        쌓인다. a_brake 로 기울기를 제한하면 명령 자체가 추종 가능해진다.
        가속 방향은 제한하지 않는다 (위험이 사라지면 바로 회복).
        """
        now = self.get_clock().now().nanoseconds
        prev = self._slew_prev_v
        prev_ns = self._slew_prev_ns
        self._slew_prev_ns = now

        if prev is None or prev_ns == 0:
            self._slew_prev_v = v_target
            return v_target
        dt = (now - prev_ns) * 1e-9
        if dt <= 0.0 or dt > 0.5:  # 오래 끊겼으면 이력 버림
            self._slew_prev_v = v_target
            return v_target

        floor = prev - self.avoid_speed_params.a_brake * dt
        v = max(v_target, floor)
        self._slew_prev_v = v
        return v

    def _planner_gate_closest_m(self, filtered: list) -> float:
        """게이트 통과 장애 — 전방 콘 없이(조향 후에도 '아직 있음' 판정용)."""
        return closest_obstacle_surface_m(
            filtered,
            forward_cone_rad=None,
            min_forward_x_m=self.avoid_min_forward_x_m,
            lateral_abs_max_m=self._obstacle_lateral_abs_max_m,
            laser_to_base_x_m=self.laser_to_base_x_m,
        )

    def _planner_closest_obstacle_m(self, filtered: list) -> float:
        return closest_obstacle_surface_m(
            filtered,
            forward_cone_rad=self.forward_cone_rad,
            min_forward_x_m=self.avoid_min_forward_x_m,
            lateral_abs_max_m=self.avoid_trigger_lateral_abs_max_m,
            laser_to_base_x_m=self.laser_to_base_x_m,
        )

    @staticmethod
    def _snap_speed_scale(x: float) -> float:
        # 전략 배율(곡선·회피 0.5, 중간 1, 직선 2)에 맞춤
        return min((0.5, 1.0, 2.0), key=lambda c: abs(c - float(x)))

    def _cb_strategy_multiplier(self, msg: Float64) -> None:
        self._strategy_mul_recv = float(msg.data)
        self._publish_planner_speed_out()

    def _cb_strategy_condition(self, msg: UInt8) -> None:
        self._strategy_cond_recv = int(msg.data)
        self._publish_planner_speed_out()

    def _publish_planner_speed_out(self) -> None:
        if not self._strategy_bridge_enable:
            sc = 1.0
            cd = 0
        else:
            sc = self._snap_speed_scale(self._strategy_mul_recv)
            cd = int(self._strategy_cond_recv) & 0xFF

        # GLOBAL 에서도 접근 선감속을 건다. 모드가 바뀌는 순간이 아니라
        # 장애물이 가까워지는 정도에 따라 연속적으로 줄어들어야 한다.
        # mode 가 아니라 실제로 회피 경로를 내보내는 중인지로 판단한다.
        # 장애물이 사라진 뒤에도 mode 는 잠시 AVOID 로 남는데, 그동안 조향
        # 한계까지 걸면 아무것도 없는 구간에서 속도가 묶인다.
        sc = min(
            sc,
            self._avoid_speed_scale(
                self._last_pose_for_speed, avoiding=self._override_active
            ),
        )

        self.pub_planner_speed_scale.publish(Float64(data=sc))
        self.pub_planner_speed_condition.publish(UInt8(data=cd))

    def _republish_planner_speed(self) -> None:
        self._publish_planner_speed_out()

    def cb_static_obstacles(self, msg: Float32MultiArray):
        self._obstacle_data = list(msg.data)
        self._last_obs_recv_ns = self.get_clock().now().nanoseconds

    def cb_dynamic_obstacles(self, msg: Float32MultiArray):
        self._dynamic_obstacle_data = list(msg.data)

    def cb_ego_speed(self, msg: Float64) -> None:
        speed = float(msg.data)
        if math.isfinite(speed):
            self._ego_speed_mps = abs(speed)

    def cb_fgm_target(self, msg: PointStamped):
        self._fgm_target = msg
        self._last_fgm_recv_ns = self.get_clock().now().nanoseconds

    def _publish_csv_track_viz(self) -> None:
        if self.pub_csv_track is None or len(self.points) < 2:
            return
        now = self.get_clock().now().to_msg()
        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = now
        s = self._csv_viz_stride
        for i in range(0, len(self.points), s):
            x, y = self.points[i]
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
        self.pub_csv_track.publish(out)

    def _build_sliding_path(
        self, mx: float | None = None, my: float | None = None
    ) -> Path | None:
        """슬라이딩 경로. mx,my 가 있으면 TF 조회 생략(회피 타이머에서 중복 lookup 방지)."""
        now = self.get_clock().now().to_msg()
        if mx is None or my is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.15),
                )
                mx = t.transform.translation.x
                my = t.transform.translation.y
            except TransformException:
                return None
        pts_xy = self.track.sliding_xy(float(mx), float(my))

        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = now
        for x, y in pts_xy:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
        return out

    def _stamp_copy_of_path(self, src: Path) -> Path:
        out = Path()
        out.header.frame_id = src.header.frame_id or self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()
        for p in src.poses:
            np = PoseStamped()
            np.header = out.header
            np.pose = p.pose
            out.poses.append(np)
        return out

    def _get_current_pose_map(self) -> PoseStamped | None:
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException:
            return None
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.position.z = t.transform.translation.z
        p.pose.orientation = t.transform.rotation
        return p

    def _get_fgm_target_in_map(self) -> Tuple[float, float] | None:
        if self._fgm_target is None:
            return None
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = (
            self._fgm_target.header.stamp.sec * 1_000_000_000
            + self._fgm_target.header.stamp.nanosec
        )
        if now_ns - stamp_ns > self.fgm_target_stale_ns:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self._fgm_target.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException:
            return None
        px = self._fgm_target.point.x
        py = self._fgm_target.point.y
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q = t.transform.rotation
        return _point_laser_to_map(px, py, tx, ty, q.w, q.x, q.y, q.z)

    def _publish_sliding_dbg(self, base_path: Path) -> None:
        if self.pub_sliding_dbg is None:
            return
        self.pub_sliding_dbg.publish(self._stamp_copy_of_path(base_path))

    def _publish_track_anchor(self, base_path: Path) -> None:
        if self.pub_anchor is None or not base_path.poses:
            return
        px = base_path.poses[0].pose.position.x
        py = base_path.poses[0].pose.position.y
        m = PointStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.point.x = float(px)
        m.point.y = float(py)
        m.point.z = 0.0
        self.pub_anchor.publish(m)

    def _publish_local_path_bundle(self, out: Path, sliding_src: Path) -> None:
        """waypoint 로 가는 내용과 동일 디버그 토픽."""
        self.pub_path.publish(out)
        self._publish_sliding_dbg(sliding_src)
        self._publish_track_anchor(sliding_src)
        if self.pub_sent_dbg is not None:
            self.pub_sent_dbg.publish(self._stamp_copy_of_path(out))

    def _publish_override_gate(self, active: bool) -> None:
        # 속도 정책이 "실제로 회피 경로를 주고 있는지" 를 봐야 해서 기억해 둔다.
        # rejoin 이 꺼져 있으면 장애물이 사라진 뒤 mode 는 바로 GLOBAL 이 된다.
        self._override_active = bool(active)
        g = Bool()
        g.data = bool(active)
        self.pub_override_gate.publish(g)

    def _fgm_target_fresh(self) -> PointStamped | None:
        """신선한 /fgm_target 만. 오래된 목표로 속도를 묶으면 안 된다."""
        tgt = self._fgm_target
        if tgt is None:
            return None
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = (
            tgt.header.stamp.sec * 1_000_000_000 + tgt.header.stamp.nanosec
        )
        if now_ns - stamp_ns > self.fgm_target_stale_ns:
            return None
        return tgt

    def _build_loop_geometry(self) -> None:
        n = len(self.points)
        self._xs = [p[0] for p in self.points]
        self._ys = [p[1] for p in self.points]
        self._seg_len: List[float] = []
        for i in range(n):
            ax, ay = self._xs[i], self._ys[i]
            bx, by = self._xs[(i + 1) % n], self._ys[(i + 1) % n]
            self._seg_len.append(math.hypot(bx - ax, by - ay))
        cum0 = [0.0]
        for i in range(n):
            cum0.append(cum0[-1] + self._seg_len[i])
        self._total_l = cum0[-1]
        self._seg_start = cum0[:-1]
        self._seg_end = cum0[1:]
        self._n = n
        self._xs_np = np.asarray(self._xs, dtype=np.float64)
        self._ys_np = np.asarray(self._ys, dtype=np.float64)
        self._bx_np = np.roll(self._xs_np, -1)
        self._by_np = np.roll(self._ys_np, -1)

    def _closest_on_loop(
        self, xp: float, yp: float
    ) -> Tuple[float, float, int, float]:
        ax, ay = self._xs_np, self._ys_np
        bx, by = self._bx_np, self._by_np
        abx, aby = bx - ax, by - ay
        ab2 = abx * abx + aby * aby
        t = np.divide(
            (xp - ax) * abx + (yp - ay) * aby,
            ab2,
            out=np.zeros_like(ab2),
            where=ab2 >= 1e-14,
        )
        t = np.clip(t, 0.0, 1.0)
        qx = ax + t * abx
        qy = ay + t * aby
        d2 = (xp - qx) ** 2 + (yp - qy) ** 2
        d2 = np.where(ab2 < 1e-14, np.inf, d2)
        i = int(np.argmin(d2))
        return float(qx[i]), float(qy[i]), i, float(t[i])

    def _xy_yaw_at_s(self, s: float) -> Tuple[float, float, float]:
        n = self._n
        if self._total_l < 1e-6:
            return self._xs[0], self._ys[0], 0.0
        s = s % self._total_l
        i = bisect.bisect_left(self._seg_end, s - 1e-9)
        if i >= n:
            i = n - 1
        tloc = (s - self._seg_start[i]) / max(self._seg_len[i], 1e-9)
        tloc = max(0.0, min(1.0, tloc))
        j = (i + 1) % n
        x = self._xs[i] + tloc * (self._xs[j] - self._xs[i])
        y = self._ys[i] + tloc * (self._ys[j] - self._ys[i])
        yaw = math.atan2(self._ys[j] - self._ys[i], self._xs[j] - self._xs[i])
        return x, y, yaw

    def _project_to_frenet(
        self, x: float, y: float, yaw: float
    ) -> Tuple[float, float, float, float, float, float]:
        qx, qy, seg_i, t = self._closest_on_loop(x, y)
        s0 = self._seg_start[seg_i] + t * self._seg_len[seg_i]
        i = seg_i
        yaw_ref = math.atan2(
            self._ys[(i + 1) % self._n] - self._ys[i],
            self._xs[(i + 1) % self._n] - self._xs[i],
        )
        nx = -math.sin(yaw_ref)
        ny = math.cos(yaw_ref)
        d0 = (x - qx) * nx + (y - qy) * ny
        yaw_err = _wrap_pi(yaw - yaw_ref)
        d0p = math.tan(yaw_err)
        d0p = max(-1.0, min(1.0, d0p))
        d0pp = 0.0
        return s0, d0, d0p, d0pp, yaw_ref, yaw_err

    @staticmethod
    def _solve_quintic(
        d0: float,
        d0p: float,
        d0pp: float,
        df: float,
        dfp: float,
        dfpp: float,
        L: float,
    ) -> Tuple[float, float, float, float, float, float]:
        a0 = d0
        a1 = d0p
        a2 = 0.5 * d0pp
        if L < 1e-6:
            return a0, a1, a2, 0.0, 0.0, 0.0
        A = np.array(
            [
                [L**3, L**4, L**5],
                [3 * L**2, 4 * L**3, 5 * L**4],
                [6 * L, 12 * L**2, 20 * L**3],
            ],
            dtype=float,
        )
        b = np.array(
            [
                df - (a0 + a1 * L + a2 * L**2),
                dfp - (a1 + 2 * a2 * L),
                dfpp - (2 * a2),
            ],
            dtype=float,
        )
        a3, a4, a5 = np.linalg.solve(A, b)
        return a0, a1, a2, float(a3), float(a4), float(a5)

    @staticmethod
    def _eval_quintic(coeff: Tuple[float, ...], ds: float) -> float:
        a0, a1, a2, a3, a4, a5 = coeff
        return (
            a0
            + a1 * ds
            + a2 * ds**2
            + a3 * ds**3
            + a4 * ds**4
            + a5 * ds**5
        )

    def _append_pose(self, path: Path, x: float, y: float) -> None:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)

    def _build_frenet_quintic_rejoin_path(
        self, current_pose: PoseStamped
    ) -> Path | None:
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        yaw = _quat_to_yaw(current_pose.pose.orientation)

        s0, d0, d0p, d0pp, _, _ = self._project_to_frenet(x, y, yaw)

        L = self.rejoin_min_length_m
        L = min(L, self.rejoin_max_length_m)

        coeff = self._solve_quintic(d0, d0p, d0pp, 0.0, 0.0, 0.0, L)
        self._rejoin_target_s = (s0 + L) % self._total_l

        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()

        n_samples = self.rejoin_sample_count
        for k in range(n_samples):
            ds = L * k / max(n_samples - 1, 1)
            d = self._eval_quintic(coeff, ds)
            s = s0 + ds
            x_ref, y_ref, yaw_ref = self._xy_yaw_at_s(s)
            px = x_ref - d * math.sin(yaw_ref)
            py = y_ref + d * math.cos(yaw_ref)
            self._append_pose(out, px, py)

        tail_step = self._total_l / max(self._n, 1)
        tail_step = max(0.05, min(0.1, tail_step))
        for k in range(self.rejoin_tail_count):
            s_tail = s0 + L + k * tail_step
            x_ref, y_ref, _ = self._xy_yaw_at_s(s_tail)
            self._append_pose(out, x_ref, y_ref)

        if len(out.poses) < 2:
            return None

        if self.verbose_logs:
            self.get_logger().info(
                f"REJOIN path generated: d0={d0:.2f}m, L={L:.2f}m, samples={len(out.poses)}"
            )
        return out

    def _csv_cte_abs_m(self, current_pose: PoseStamped) -> float:
        """CSV(raceline) 기준 |CTE| = Frenet lateral |d|."""
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        yaw = _quat_to_yaw(current_pose.pose.orientation)
        _, d_now, _, _, _, _ = self._project_to_frenet(x, y, yaw)
        return abs(float(d_now))

    def _is_rejoin_finished(self, current_pose: PoseStamped) -> bool:
        """CTE(|d|) ≤ rejoin_finish_lateral_m 이면 CSV 복귀 완료."""
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        yaw = _quat_to_yaw(current_pose.pose.orientation)
        _, d_now, _, _, _, yaw_err = self._project_to_frenet(x, y, yaw)
        if abs(d_now) >= self.rejoin_finish_lateral_m:
            return False
        if self.rejoin_finish_require_heading:
            return abs(yaw_err) < self.rejoin_finish_heading_rad
        return True

    def _go_global(self) -> None:
        self.mode = "GLOBAL"
        self._rejoin_path_msg = None
        self._avoid_on_count = 0
        self._avoid_off_count = 0

    def _log_mode_transition(self, old_mode: str, d_closest: float) -> None:
        if not self.verbose_logs:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_mode_log_ns < 100_000_000:
            return
        self._last_mode_log_ns = now_ns
        d_str = "inf" if d_closest == float("inf") else f"{d_closest:.2f}"
        self.get_logger().info(
            f"mode transition: {old_mode} -> {self.mode}, d_closest={d_str}"
        )

    def _update_mode(
        self,
        d_closest: float,
        d_gate: float,
        filtered: list,
        current_pose: PoseStamped | None,
        filtered_dynamic: list,
        d_dyn_closest: float,
        d_dyn_gate: float,
        rel_speed: float,
    ) -> None:
        if not self.use_fgm and self.mode == "AVOID":
            old_mode = self.mode
            self._go_global()
            if old_mode != self.mode:
                self._log_mode_transition(old_mode, d_closest)
            return

        on_m, _off_m, fgm_m = self._effective_avoid_gates()
        static_obstacle_on = d_closest <= on_m
        dynamic_obstacle_on = (
            len(filtered_dynamic) >= 6
            and math.isfinite(d_dyn_closest)
            and d_dyn_closest <= on_m
            and rel_speed > 0.0
        )
        obstacle_on = static_obstacle_on or dynamic_obstacle_on
        still_blocking = self._obstacles_remain(filtered)
        fully_cleared = self._avoidance_fully_cleared(filtered, current_pose)

        old_mode = self.mode

        if self.mode == "GLOBAL":
            if obstacle_on and self.use_fgm:
                self._avoid_on_count += 1
            else:
                self._avoid_on_count = 0

            if self._avoid_on_count >= self.avoid_on_count_th:
                self.mode = "AVOID"
                self._avoid_off_count = 0
                self._rejoin_path_msg = None

        elif self.mode == "AVOID":
            static_still_ahead = (
                still_blocking
                or (math.isfinite(d_gate) and d_gate <= fgm_m)
                or (math.isfinite(d_closest) and d_closest <= fgm_m)
            )
            dynamic_still_ahead = (
                len(filtered_dynamic) >= 6
                and rel_speed > 0.0
                and (
                    (math.isfinite(d_dyn_gate) and d_dyn_gate <= fgm_m)
                    or (
                        math.isfinite(d_dyn_closest)
                        and d_dyn_closest <= fgm_m
                    )
                )
            )
            obstacle_still_ahead = static_still_ahead or dynamic_still_ahead
            if obstacle_still_ahead:
                self._avoid_off_count = 0
            elif fully_cleared:
                self._avoid_off_count += 1
            else:
                self._avoid_off_count = 0

            if (
                not obstacle_still_ahead
                and self._avoid_off_count >= self.avoid_off_count_th
            ):
                cte_ok = (
                    current_pose is not None
                    and self._csv_cte_abs_m(current_pose)
                    <= self.rejoin_finish_lateral_m
                )
                # rejoin 을 쓸 때만 CTE 가 줄 때까지 AVOID 를 붙든다. rejoin 이
                # 꺼져 있으면 이미 override 를 내려 Stanley 가 CSV 로 복귀하는
                # 중이라, mode 만 AVOID 로 남겨두면 AEB 완화와 속도 캡이
                # 이유 없이 길어진다.
                if not cte_ok and self.rejoin_enable:
                    pass
                elif current_pose is not None and self.rejoin_enable:
                    self._rejoin_path_msg = self._build_frenet_quintic_rejoin_path(
                        current_pose
                    )
                    if (
                        self._rejoin_path_msg is not None
                        and len(self._rejoin_path_msg.poses) >= 2
                    ):
                        self.mode = "REJOIN"
                    else:
                        self._go_global()
                else:
                    self._go_global()

        elif self.mode == "REJOIN":
            if obstacle_on and self.use_fgm:
                self.mode = "AVOID"
                self._rejoin_path_msg = None
                self._avoid_off_count = 0
            elif current_pose is not None and self._is_rejoin_finished(current_pose):
                self._go_global()

        if old_mode != self.mode:
            self._log_mode_transition(old_mode, d_closest)

    def _build_avoid_path(
        self,
        current: PoseStamped,
        fgm_x: float,
        fgm_y: float,
        *,
        merge_csv_tail: bool,
    ) -> Path:
        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()

        p0 = PoseStamped()
        p0.header = out.header
        p0.pose = current.pose
        out.poses.append(p0)

        # FGM 방향으로 연장 (차량 heading 직진이면 Stanley hdg_err≈0 되어 조향이 약해짐)
        cx = float(current.pose.position.x)
        cy = float(current.pose.position.y)
        dx = float(fgm_x) - cx
        dy = float(fgm_y) - cy
        span = math.hypot(dx, dy)
        if span > 1e-3:
            fx = dx / span
            fy = dy / span
        else:
            yaw = _quat_to_yaw(current.pose.orientation)
            fx = math.cos(yaw)
            fy = math.sin(yaw)

        # 차량 → FGM 목표 구간 촘촘히 (목표점이 멀어져도 Stanley 최근접/전방주시가 정확)
        n_lead = int(span / self.avoid_forward_step_m)
        for k in range(1, n_lead + 1):
            s = k * self.avoid_forward_step_m
            q = PoseStamped()
            q.header = out.header
            q.pose.position.x = float(cx + fx * s)
            q.pose.position.y = float(cy + fy * s)
            q.pose.position.z = 0.0
            q.pose.orientation.w = 1.0
            out.poses.append(q)

        p1 = PoseStamped()
        p1.header = out.header
        p1.pose.position.x = fgm_x
        p1.pose.position.y = fgm_y
        p1.pose.position.z = 0.0
        p1.pose.orientation.w = 1.0
        out.poses.append(p1)
        for k in range(1, self.avoid_forward_num_points + 1):
            s = k * self.avoid_forward_step_m
            q = PoseStamped()
            q.header = out.header
            q.pose.position.x = float(fgm_x + fx * s)
            q.pose.position.y = float(fgm_y + fy * s)
            q.pose.position.z = 0.0
            q.pose.orientation.w = 1.0
            out.poses.append(q)

        if not merge_csv_tail:
            return out

        n = len(self.points)
        d2 = (self._xs_np - fgm_x) ** 2 + (self._ys_np - fgm_y) ** 2
        best_i = int(np.argmin(d2))

        for t_idx in range(self.avoid_merge_tail_max):
            i = (best_i + t_idx) % n
            q = PoseStamped()
            q.header = out.header
            q.pose.position.x = float(self.points[i][0])
            q.pose.position.y = float(self.points[i][1])
            q.pose.position.z = 0.0
            q.pose.orientation.w = 1.0
            out.poses.append(q)

        return out

    def _maybe_log_speed_status(
        self,
        d_static: float,
        d_dyn: float,
        obs_speed: float,
        rel_speed: float,
    ) -> None:
        if self._status_log_period <= 0.0:
            return
        self._status_log_accum += 1.0 / max(self.publish_hz, 1.0)
        if self._status_log_accum < self._status_log_period:
            return
        self._status_log_accum = 0.0

        d_s = "inf" if not math.isfinite(d_static) else f"{d_static:.2f}"
        d_d = "inf" if not math.isfinite(d_dyn) else f"{d_dyn:.2f}"
        threat = "APPROACH" if rel_speed > 0.0 and math.isfinite(d_dyn) else "—"
        self.get_logger().info(
            f"SPEED_STATUS | mode={self.mode} | "
            f"v_ego={self._ego_speed_mps:.2f} "
            f"v_obs={obs_speed:.2f} "
            f"v_rel(close)={rel_speed:+.2f} m/s | "
            f"d_static={d_s} d_dyn={d_d} m | threat={threat}"
        )

    def _publish_fgm_enable(
        self,
        filtered: list,
        d_gate: float,
        filtered_dynamic: list,
        d_dyn_gate: float,
    ) -> None:
        """
        FGM = 회피 주체. AVOID 전 구간 켜 두고, 접근 중에도 미리 켠다.
        정적/동적 모두 속도 스케일된 fgm_enable 이내면 enable.
        """
        _, _, fgm_m = self._effective_avoid_gates()
        static_approaching = (
            len(filtered) >= 4
            and math.isfinite(d_gate)
            and d_gate <= fgm_m
        )
        dynamic_approaching = (
            len(filtered_dynamic) >= 6
            and math.isfinite(d_dyn_gate)
            and d_dyn_gate <= fgm_m
        )
        approaching = static_approaching or dynamic_approaching
        enable = self.use_fgm and (self.mode == "AVOID" or approaching)
        msg = Bool()
        msg.data = bool(enable)
        self.pub_fgm_enable.publish(msg)

    def timer_publish(self):
        filtered = self._filter_obstacles_for_planner(self._obstacle_data)
        filtered_dynamic = self._filter_dynamic_for_planner(self._dynamic_obstacle_data)
        # 뒷축 거리 − 전방 오버행 → 앞범퍼 기준으로 게이트 판단
        d_closest = self._nose_adjusted_dist(self._planner_closest_obstacle_m(filtered))
        d_gate = self._nose_adjusted_dist(self._planner_gate_closest_m(filtered))
        d_dyn_closest, d_dyn_gate, rel_speed, obs_speed = self._dynamic_threat_metrics(
            filtered_dynamic
        )
        d_dyn_closest = self._nose_adjusted_dist(d_dyn_closest)
        d_dyn_gate = self._nose_adjusted_dist(d_dyn_gate)
        self._last_obs_speed_mps = obs_speed
        self._last_rel_speed_mps = rel_speed
        self._last_d_dyn_closest = d_dyn_closest
        current = self._get_current_pose_map()
        # strategy 콜백에서도 속도 배율을 다시 내므로 캐시해 둔다.
        # 속도 판단엔 코리도 통과 장애만 쓴다 — 트랙 밖 물체로 감속하면 안 된다.
        self._last_pose_for_speed = current
        self._speed_static_obs = filtered
        self._speed_dynamic_obs = filtered_dynamic

        self._update_mode(
            d_closest,
            d_gate,
            filtered,
            current,
            filtered_dynamic,
            d_dyn_closest,
            d_dyn_gate,
            rel_speed,
        )
        self._publish_planner_speed_out()
        self.pub_planner_mode.publish(String(data=self.mode))
        self._publish_fgm_enable(filtered, d_gate, filtered_dynamic, d_dyn_gate)
        self._maybe_log_speed_status(
            d_closest, d_dyn_closest, obs_speed, rel_speed
        )

        if self.mode == "GLOBAL":
            self._publish_override_gate(False)
            return

        if self.mode == "AVOID":
            static_path = self._static_wants_fgm_local_path(
                filtered, d_closest, d_gate
            )
            dynamic_path = self._dynamic_wants_fgm_local_path(
                filtered_dynamic, d_dyn_closest, rel_speed
            )
            if not static_path and not dynamic_path:
                self._publish_override_gate(False)
                return

            fgm_xy = self._get_fgm_target_in_map()
            if current is None or fgm_xy is None:
                if not hasattr(self, "_last_avoid_warn_ns"):
                    self._last_avoid_warn_ns = 0
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self._last_avoid_warn_ns > 2_000_000_000:
                    self.get_logger().warn(
                        "FGM 회피 분기인데 pose 또는 /fgm_target 없음 — /local_path 미발행."
                    )
                    self._last_avoid_warn_ns = now_ns
                self._publish_override_gate(False)
                return

            fgm_x, fgm_y = fgm_xy[0], fgm_xy[1]
            out = self._build_avoid_path(
                current, fgm_x, fgm_y, merge_csv_tail=False
            )
            out, usable = self._truncate_path_at_collision(
                out, self._lookup_laser_to_map_transform()
            )
            if not usable:
                self._warn_avoid_path_blocked()
                self._publish_override_gate(False)
                return

            if len(out.poses) >= 2:
                base_path = (
                    self._build_sliding_path(
                        current.pose.position.x, current.pose.position.y
                    )
                    if self._need_sliding_for_debug
                    else None
                )
                if base_path is not None and len(base_path.poses) >= 2:
                    self._publish_local_path_bundle(out, base_path)
                else:
                    self.pub_path.publish(out)
                    if self.pub_sent_dbg is not None:
                        self.pub_sent_dbg.publish(self._stamp_copy_of_path(out))
                self._publish_override_gate(True)
                now_ns = self.get_clock().now().nanoseconds
                if (
                    self.verbose_logs
                    and now_ns - self._last_latency_log_ns > 500_000_000
                ):
                    obs_ms = (
                        (now_ns - self._last_obs_recv_ns) / 1e6
                        if self._last_obs_recv_ns > 0
                        else float("nan")
                    )
                    fgm_ms = (
                        (now_ns - self._last_fgm_recv_ns) / 1e6
                        if self._last_fgm_recv_ns > 0
                        else float("nan")
                    )
                    self.get_logger().info(
                        f"[latency] obs->planner_out={obs_ms:.1f}ms, "
                        f"fgm->planner_out={fgm_ms:.1f}ms"
                    )
                    self._last_latency_log_ns = now_ns
            else:
                self._publish_override_gate(False)
            return

        if self.mode == "REJOIN":
            if (
                self._rejoin_path_msg is not None
                and len(self._rejoin_path_msg.poses) >= 2
            ):
                self._rejoin_path_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_path.publish(self._rejoin_path_msg)
                if self.pub_sent_dbg is not None:
                    self.pub_sent_dbg.publish(
                        self._stamp_copy_of_path(self._rejoin_path_msg)
                    )
                self._publish_override_gate(True)
            else:
                self._publish_override_gate(False)
                self.mode = "GLOBAL"
                self._rejoin_path_msg = None

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
