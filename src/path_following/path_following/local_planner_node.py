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

from path_following import vehicle_geometry as vg
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
from .offset_maneuver import (
    ManeuverConfig,
    ObstacleSD,
    OffsetManeuver,
    plan_maneuver,
)

# 실측 전륜 조향 한계 [rad]. Stanley 의 max_steering_angle_real_rad 와 같은 값이며,
# 계획 단계에서 "탈 수 있는 경로인가" 를 판정하는 데만 쓴다.
_MAX_STEER_RAD = 0.3735

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
    # "이 장애물이 레이스라인 위의 우리 차를 막는가" 의 기준 [m].
    #
    # 판정은 `_outside_corridor` 에서 `(레이스라인까지 거리 - 장애물 반경)`
    # 이 이 값을 넘으면 무시하는 식이다. 즉 **장애물의 가까운 쪽 끝**이 라인
    # 에서 이만큼 떨어져 있으면 없는 것으로 본다.
    #
    # 그래서 기준은 차 반폭이어야 한다. 0.40 은 반폭(0.15)보다 25 cm 넉넉해서,
    # 라인을 그대로 타고 가면 25 cm 여유로 지나갈 물체까지 회피 대상으로
    # 잡았다 — 피할 이유가 없는데 라인을 벗어나니 그 자체가 위험했다.
    #
    # 반폭(0.15) + 3 cm. 여유는 슬립·추종 오차로 라인에서 조금 밀린 채 지나갈
    # 경우 몫이다. 이 밖의 물체는 **없는 것으로 본다** — 회피도 안 하고 감속도
    # 안 한다 (`_speed_static_obs` 가 이 필터를 거친 목록이다). 그냥 CSV 속도로
    # 글로벌 패스를 탄다.
    "corridor_max_lateral_from_raceline_m": round(vg.HALF_WIDTH_M + 0.03, 3),
    "obstacle_forward_min_m": 0.30,
    "obstacle_forward_max_m": 12.0,
    # laser frame |y| 검출 게이트. 차폭보다 넉넉해야 한다 — 여기서 버리면
    # 뒤에서 되살릴 방법이 없다. 실측 반폭 0.15 에 0.27 여유.
    "obstacle_lateral_abs_max_m": 0.42,
    # 위 값은 차 진행축 기준 직선 튜브라 곡선·헤딩오차에 약하다. 반경 10m
    # 코너면 레이스라인 정중앙 장애물도 3m 앞에서 |y|=0.45 라 잘려 나가고,
    # 직선에서도 헤딩 5° 면 5m 앞에서 0.44 다. 회피를 시작해야 할 거리에서
    # 장애물이 사라지는 게 이것 때문이다.
    # 레이스라인 코리도는 맵 좌표로 재므로 곡선에서도 정확하다. 그게 도는
    # 동안에는 여기를 넓게 열고 판단을 코리도에 맡긴다. 위 값은 코리도를
    # 못 쓸 때(TF 실패/비활성)의 보수적 폴백으로만 남는다.
    "obstacle_lateral_abs_max_corridor_m": 1.50,
    "obstacle_tf_timeout_sec": 0.15,
    # sensor_static_tf.cpp 의 base_link->laser translation.x 와 같아야 한다.
    # 이전 0.275 는 TF(0.31)와 어긋나 있어서, 라이다 거리를 base_link 로 옮길 때
    # 3.5 cm 씩 가깝게 봤다.
    "laser_to_base_x_m": vg.LASER_X_M,      # 이전 0.275 (TF 는 0.31)
    # [차량 버블] 뒷축→전방 길이. 게이트 거리 d에서 빼서 앞범퍼 기준으로 회피.
    # 폭(ego_safety_width)은 fgm_node 에서 섹터 반폭으로 사용.
    # 실측 앞끝은 0.50 인데 0.30 이라, 앞범퍼 기준 여유를 20 cm 더 있는 것으로
    # 착각하고 있었다.
    "ego_front_safety_m": vg.FRONT_M,       # 이전 0.30
    "use_fgm": True,
    # FGM 조준각 때문에 차가 서지는 않게 하는 하한.
    #
    # `_avoid_target_speed` 의 maneuver 항은 "FGM 조준각을 낼 수 있는 속도" 다.
    # 조준각이 클수록 이 값이 작아져서, 45° 가 나오면 0.1 m/s 까지 떨어진다 —
    # 회피하려다 장애물 앞에서 선다. 고속 쪽은 `fgm_node` 가 FOV 를 좁혀
    # 애초에 큰 각이 안 나오게 막고, 여기서는 남은 경우의 바닥을 잡는다.
    # 장애물 거리 기반 감속(static/dynamic)에는 안 걸린다 — 실제로 못 지나가는
    # 상황은 그쪽과 AEB 몫이다.
    "avoid_fgm_min_speed": 3.0,
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
    # 저속에서 CSV → local_path 로 갈아타는 시점을 늦춘다.
    #
    # FGM 은 그대로 켜 둔다 (fgm_enable_m 은 안 건드린다). 저속에서 일찍
    # 갈아타 봐야 장애물이 아직 멀어 조준이 흔들리고, 그동안 레이스라인만
    # 놓친다. 고속은 손대지 않는다 — 거기서 늦추면 피할 거리가 안 나온다.
    "avoid_on_late_scale": 0.7,        # 저속 avoid_on 배율 (30% 늦춤)
    "avoid_on_late_max_speed": 4.0,    # 이 아래가 저속
    "avoid_on_late_blend_mps": 1.0,    # 문턱 위 이 폭에 걸쳐 원복
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
    # 회피 후 레이스라인 복귀를 Frenet quintic 으로.
    #
    # 레이스라인이 벽에 붙어 있는 구간에서는 이게 없으면 안 된다. 회피로
    # 벗어난 뒤 Stanley 피드백만으로 붙으면 접근각이 서서 벽에 꽂힌다.
    # quintic 은 끝점에서 d'=d''=0 이라 라인에 접선으로 눕혀 붙는다.
    "rejoin_enable": True,
    "rejoin_min_length_m": 0.50,
    # 재합류 길이는 속도 연동: clip(rejoin_time_sec * v_ego, min, max).
    # 예전엔 항상 min(0.50m) 이라 3m/s 에서 0.17초 만에 붙으라는 소리였다.
    "rejoin_time_sec": 0.8,
    # 시간 연동만으로는 부족하다. quintic d(s) 의 최대 곡률은 5.77*|d0|/L^2 라
    # 요구 횡가속도가 v^2*5.77*|d0|/L^2 로 튄다. 7m/s 에서 1.5m 벗어난 채
    # L=2.5m 로 붙으라면 67.9m/s^2 (타이어 한계의 7배) 를 요구하는 셈이고,
    # 따라갈 수 없는 경로가 나와 조향만 포화된 채 벽으로 밀린다.
    # 그래서 이탈량과 속도로부터 L 을 역산한다: L = sqrt(5.77*|d0|*v^2/a_lat).
    # max 는 그 역산값을 자를 만큼 작으면 안 된다 (7m/s, 1.5m 이면 9.2m 필요).
    "rejoin_a_lat_mps2": 4.0,
    # 회피 해제 후 속도 회복 기울기 상한 [m/s^2]. 0 이면 무제한(예전 동작).
    "avoid_a_accel_mps2": 4.0,
    # 재합류 경로가 레이스라인과 이룰 수 있는 최대 각 [deg]. 속도에 따라
    # 아래(min)~위(max) 사이에서 움직인다 — `_rejoin_heading_limit_rad` 참고.
    #
    # 이 각이 곧 "라인에 얼마나 비스듬히 꽂히는가" 다. 각이 서면 벽에 붙은
    # 라인 구간에서 경로 자체가 벽을 향하고, 추종이 조금만 늦어도 넘어간다.
    # 각을 눕히는 방법은 길이뿐인데(L >= 1.875*|d0|/tan) 트랙 둘레가 41 m 라
    # 무한정 늘릴 수 없어서, 고속쪽 하한을 10° 로 막아 둔다.
    "rejoin_max_heading_deg": 18.0,
    "rejoin_min_heading_deg": 7.0,
    # 합류 시 라인을 넘어가도 좋은 양 [m] 과 추종 지연 [s].
    # 오버슈트 ≈ v·sin(ψ)·τ 를 이 값 이하로 누르는 각을 쓴다.
    # 6 m/s 에서 10°, 4 m/s 에서 14.5°, 2.5 m/s 에서 18° 가 나온다.
    "rejoin_merge_overshoot_m": 0.20,
    "rejoin_track_lag_s": 0.30,
    # 차가 실제로 낼 수 있는 최대 경로곡률 [1/m] = tan(전륜각)/축거.
    # 실측 전륜각 21.4°, 축거 0.33 m → 1.19. 이걸 넘는 복귀 경로는 아무리
    # 천천히 가도 못 따라간다 — 감속이 아니라 포기해야 하는 경우다.
    "rejoin_max_path_curvature": 1.19,
    # Frenet quintic 이 성립하는 헤딩오차 한계 [deg].
    #
    # d(s) 는 s 의 함수라 차가 라인과 수직에 가까워지면 정의 자체가 무너진다
    # (ds/dt → 0, d0p = tan(ψ) → ∞). 예전엔 d0p 를 ±1 로 잘라 뒀는데, 그건
    # 한계를 다루는 게 아니라 **거짓말을 하는** 것이었다: 실제 75° 로 벽을
    # 향하고 있어도 플래너는 45° 로 알고 얌전한 경로를 냈다. 경로는 충돌검사를
    # 통과하고, 차는 그 경로를 따라갈 수 없어 그대로 벽으로 갔다.
    #
    # 이제 이 각까지만 quintic 을 믿는다. 넘으면 복귀가 아니라 **정렬** 이
    # 먼저다 — `_build_alignment_path` 참고.
    "rejoin_yaw_err_limit_deg": 55.0,
    # 위 헤딩 제약이 요구하는 길이 (이탈 1.2 m + 15° → 8.4 m). 여기서 잘리면
    # 그만큼 각이 서므로 여유를 두되, 랩의 1/4 을 넘기지 않는 선.
    "rejoin_max_length_m": 10.0,  # 이전 2.50 → 12.0 → 헤딩 제약에 맞춰 재산정
    # 하한이다. 실제 개수는 길이/간격으로 정해진다 (충돌 검사 해상도 유지).
    "rejoin_sample_count": 30,
    "rejoin_tail_count": 40,
    "rejoin_finish_lateral_m": 0.20,
    "rejoin_finish_require_heading": False,
    "rejoin_finish_heading_deg": 15.0,
    # REJOIN 탈출 안전장치. 완료 판정은 |CTE| 가 줄어야 성립하는데, 차가
    # 서 있으면 CTE 는 절대 줄지 않아 영원히 못 나온다 (실측: 정지 후
    # 수 분간 REJOIN + override=true 유지, Stanley 는 묵은 캐시 경로 추종).
    "rejoin_stall_speed_mps": 0.25,
    "rejoin_stall_sec": 1.0,
    "rejoin_max_active_sec": 5.0,
    "rejoin_speed_scale": 0.7,  # 이전 기본값 0.5 (avoid_speed_enable=False 일 때만 쓰임)
    # 이탈량 연동 감속. rejoin_max_length_m 안에 붙을 수 있는 속도로 상한을
    # 건다: v = L_max * sqrt(a_lat / (5.77*|d|)). 크게 벗어난 채 고속을 유지하면
    # 복귀 시점에 조향이 포화되므로, 붙기 전에 미리 깎아 둔다.
    # free_m 이하에서는 상한이 무한대 — 정상 주행의 작은 CTE 로는 안 걸린다.
    "deviation_speed_enable": True,
    "deviation_speed_free_m": 0.35,
    # ---- 회피 경로 충돌검사 ----
    # 회피 경로는 FGM 목표점 너머로 avoid_forward_num_points 만큼 직선 연장된다.
    # 그 구간은 아무도 검사한 적이 없어서 코너에서는 그대로 벽을 향한다.
    # 맵과 장애물로 잘라낸다. 맵이 없으면 장애물 검사만 동작한다.
    "path_check_enable": True,
    "map_topic": "/map",
    # 이만큼 벽에서 떨어져야 통과. 직선 반폭(0.15)이 아니라 코너 스윕폭을 쓴다 —
    # 회피 경로는 급하게 휘고, 앞끝이 0.50 이라 반경 1 m 코너에서 앞 외측 코너가
    # 경로보다 0.254 m 바깥을 지난다. 이전 0.25 는 우연히 거의 같은 값이었다.
    "path_check_inflation_m": vg.PATH_CHECK_HALF_WIDTH_M,  # 이전 0.25
    "path_check_obstacle_margin_m": 0.10,
    "path_check_backoff_m": 0.20,     # 충돌 지점에서 이만큼 더 물러나 끝냄
    "path_check_min_length_m": 0.6,   # 잘라낸 경로가 이보다 짧으면 회피를 포기
    # 회피 경로가 막혔을 때 AVOID 를 재시도할 주기 [s].
    # 예전엔 "막혔음" 이 한 프레임짜리 bool 이라, AVOID→TRAILING 전이에서
    # 바로 리셋돼 다음 프레임에 TRAILING→AVOID 로 튕겨 나갔다. 2 프레임 주기로
    # 왕복(≈20 Hz)하면서 AEB 완화 기준까지 같이 깜빡였다 — AVOID 프레임은
    # 완화, TRAILING 프레임은 엄격. 시간 래치로 바꿔 이 주기만큼 붙들어 둔다.
    # 대가는 "틈이 생겼는데 최대 이 시간만큼 늦게 AVOID 복귀" 뿐이다.
    "avoid_retry_sec": 0.5,
    # 회피를 포기하기까지 연속으로 몇 프레임 막혀야 하는가.
    #
    # 예전엔 **한 프레임**만 실패해도 0.5 초 래치가 걸리고 그동안 GLOBAL 로
    # 내려갔다. 회피 경로 생성은 FGM 조준·TF·잘림 판정이 겹쳐 있어서 한
    # 프레임쯤은 쉽게 실패하는데, 그 대가가 0.5 초였다 — 5 m/s 면 장애물을
    # 향해 2.5 m 직진이다. 실제로 "로컬패스 갔다가 갑자기 글로벌패스" 로
    # 보이던 게 이것이고, 회피를 시작해 놓고 라인으로 돌아가니 오히려 박는다.
    #
    # 회피하기로 정했으면 붙들고 있어야 한다. 진짜로 못 지나가는 상황은
    # 몇 프레임이면 확실히 드러나고, 그때까지 속도 정책과 AEB 가 받는다.
    # 40 Hz 에서 5 프레임 = 125 ms.
    "avoid_blocked_frames_th": 5,
    # 직전 회피 경로를 붙들 수 있는 최대 나이 [s].
    #
    # 위 프레임 수와 같은 시간대여야 한다. 더 오래 붙들면 차가 이미 지나친
    # 경로를 쫓는다. 40 Hz × 5 프레임 = 125 ms 라 여유를 조금 얹는다.
    "avoid_hold_max_sec": 0.2,
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
    # 회피 조향 중, "이대로 가면 확실히 옆으로 비켜 지나간다" 로 판정해
    # 정지거리 한계를 면제할 때 추가로 요구하는 횡여유. 면제 판단이라
    # 최소 여유보다 조금 더 받는다. 0 이면 최소 여유만으로 면제.
    "avoid_pass_clear_extra_m": 0.15,
    # 회피 구간 순항속도 [m/s]. 둘 다 0 이면 안 건다.
    #
    # 진입 속도로 고속/저속을 가른다. 저속에서 만나는 장애물은 대개 코너나
    # 좁은 구간이라 같은 회피라도 여유가 적다 — 거기서만 한 단 더 깎는다.
    #
    # **상한이자 하한이다.** 위의 물리 한계들(maneuver/static/dynamic/
    # deviation)은 각자 다른 걸 보는데, 구간마다 다른 놈이 이기면 접근에서
    # 한 번, 회피 중에 또 한 번, 복귀에서 다시 속도가 바뀐다. 특히 거리 기반
    # 항은 장애물이 가까워질수록 계속 낮아져서 정작 피하는 순간에 제일 느리다.
    # 그래서 더 낮게 불러도 여기서 되돌린다 — 급감속은 AEB 뿐이다.
    #
    # 적용 구간은 `_avoid_speed_capped` 다. 로컬패스로 갈아타는 시점부터
    # 걸려서 복귀가 끝날 때까지 유지되고, 글로벌패스로 돌아가면 풀린다.
    #
    # 고른 값은 회피가 끝날 때까지 **붙든다** (`_avoid_cruise_target`).
    # 목표(3.0)가 문턱(4.0)보다 낮아서, 안 붙들면 5 m/s 로 진입해 3.0 을
    # 고른 차가 감속 도중 문턱을 지나며 2.0 으로 또 내려간다. 회피 한복판에
    # 목표가 바뀌는 게 제일 위험하다.
    "avoid_cruise_speed_high_mps": 3.0,
    "avoid_cruise_speed_low_mps": 2.0,
    "avoid_cruise_high_speed_th": 4.0,
    # 회피 구간이 잠깐 끊겼다 다시 켜졌을 때, 이 시간 안이면 **직전에 고른
    # 값을 그대로 되쓴다** [s].
    #
    # 래치만으로는 부족하다. 래치는 구간을 벗어나는 순간 풀리는데, 접근
    # 중에 검출이 한 프레임 깜빡이거나 모드가 잠시 GLOBAL 로 튕기면 그때
    # 풀린다. 그러면 다음 프레임에 **이미 줄어든 속도로** 다시 고르게 되어
    # 6 m/s 로 진입해 3.0 을 골랐던 차가 3.5 m/s 지점에서 2.0 으로 떨어진다.
    # 같은 장애물 하나를 피하는 중에 목표가 바뀌는 것이라, 래치를 둔 이유가
    # 그대로 무너진다.
    #
    # 풀기를 늦추면 안 된다 — 그건 글로벌 복귀 후 CSV 속도 회복을 늦춘다.
    # 그래서 푸는 건 즉시 하되, 이 시간 안에 다시 켜지면 되쓴다.
    "avoid_cruise_regrab_sec": 0.5,
    # 회피 경로가 벽에 걸려 잘렸을 때, **그 앞에서 설 수 있어야** 받는다.
    #
    # 예전에는 고정 길이(path_check_min_length_m, 0.6m)만 봤다. 6 m/s 에서
    # 0.6 m 는 0.1 초라, 벽을 향한 경로를 "쓸 만하다" 며 내보냈다. 레이스라인이
    # 벽에 붙어 있는 구간에서 FGM 이 바깥쪽 갭을 고르면 그대로 사고다.
    # 자세한 근거는 `_wall_stop_distance_m` 참고.
    "wall_stop_check_enable": True,
    "wall_stop_reaction_sec": 0.15,  # 검출~조향 반영 지연 몫
    "avoid_speed_min_mps": 0.6,   # 이 아래로는 안 줄인다 (기어가지 않게)
    "avoid_speed_ref_mps": 2.0,   # CSV 에 속도 열이 없을 때 쓸 기준속도
    # ---- AVOID 경로 생성 방식 ----
    # "offset"   = 장애물 기하에서 필요 횡오프셋을 뽑아 **미리** 계획한 기동.
    #              진입 길이가 속도에 비례해 늘어나서 고속일수록 멀리서부터
    #              완만하게 시작한다 (offset_maneuver.py).
    # "straight" = FGM 목표점까지 직선 + 전방 직선 연장 (반응형, 폴백)
    # "frenet"   = FGM 목표점의 d 로 고정 길이 quintic (구버전)
    #
    # straight/frenet 은 둘 다 FGM 조준각을 그대로 경로로 삼는다. 조준각은
    # 지금 보이는 갭에서 나오는 값이라 속도와 무관하고, 그래서 6 m/s 에서
    # 90 m/s² 짜리 경로가 태연히 나왔다. offset 은 반대로 예산을 먼저 정하고
    # 길이를 역산한다. FGM 은 계획이 안 서는 상황(트랙 밖으로 나가야만
    # 지나갈 수 있는 등)의 폴백으로만 남는다.
    # 회피는 전부 FGM 이 한다. "offset"(횡오프셋 기동)은 고속 전용으로
    # 만들었지만 실주행에서 계획이 자주 실패했고, 실패하면 어차피 감속 후
    # FGM 이 받았다 — 두 단계를 거치느라 반응만 늦었다. 아래 offset_* 파라미터
    # 들은 avoid_path_mode="offset" 로 되돌릴 때를 위해 남겨 둔다.
    "avoid_path_mode": "straight",
    # ---- 횡오프셋 기동 (avoid_path_mode="offset") ----
    # 장애물 표면 ↔ 차체 옆면 여유. 속도제한이 "이 정도 벌어지면 정지거리
    # 한계를 면제한다" 고 보는 값(lateral_margin + pass_clear_extra)이 하한이라,
    # 그보다 작게 넣으면 자동으로 올라간다. 둘이 어긋나면 계획대로 비켜
    # 가는데도 제동이 안 풀려 회피 내내 기어간다.
    "avoid_offset_margin_m": 0.25,
    # |d| 의 **상한**. 실제로 얼마나 나갈지는 점유맵에서 잰 좌/우 예산이 정한다
    # (`_build_wall_budget`). 여기 값만 믿으면 안 된다 — 실측 레이스라인→벽
    # 여유는 중앙값 0.70 m / 최소 0.30 m 라 0.70 은 구간의 76% 에서 못 낸다.
    # (예전 주석이 근거로 든 "최소 0.70 / 중앙값 1.50" 은 센터라인의 전체
    # 폭이었다. 레이스라인 한쪽 여유가 아니다. 그래서 벽에 박았다.)
    "avoid_offset_max_m": 0.70,
    "avoid_offset_a_lat_enter": 3.0,    # 진입 횡가속 예산. 낮출수록 멀리서 시작
    "avoid_offset_a_lat_exit": 1.8,     # 복귀 예산. 진입보다 낮게 = 천천히 붙음
    "avoid_offset_a_lat_hard": 4.5,     # 넘으면 조향 대신 감속으로 답한다
    # 기동 예산을 **기준선 곡률까지 포함**해서 잡는다.
    #
    # 끄면 예전처럼 기동 자신의 d'' 만 본다. 그건 기준선이 직선일 때만 맞는
    # 가정이다. 코너에서는 v²κ 를 코너가 이미 쓰고 있어서, 복귀 곡선의
    # v²·d'' 가 그 위에 얹힌다. R=6 m 를 6 m/s 로 돌면 코너만 6.0 m/s² 라
    # 접지력(5~6)에 남는 게 없는데, d'' 만 보는 검사는 그냥 통과했다.
    # 감속도 계획 실패도 안 나서 차가 라인을 가로질러 바깥 벽으로 갔다.
    #
    # 켜면 구간별로 |κ| 를 예산에서 먼저 빼고(→ 복귀가 길어짐), 그래도 합이
    # a_lat_hard 를 넘으면 speed_cap 으로 답한다. 회피 자체가 줄지는 않는다 —
    # 코너와 겹칠 때만 완만해지거나 느려진다.
    "avoid_offset_corner_aware": True,
    "avoid_offset_enter_min_m": 1.0,
    "avoid_offset_enter_max_m": 9.0,
    "avoid_offset_exit_min_m": 1.5,
    "avoid_offset_exit_max_m": 12.0,
    "avoid_offset_hold_rear_extra_m": 0.30,  # 장애물 뒤로 더 유지할 거리
    "avoid_offset_merge_gap_m": 3.0,    # 이 안의 연속 장애물은 한 기동으로
    # 계획에 허용할 조향 비율. 1.0 을 쓰면 기동만으로 풀락이라 Stanley 가
    # 오차를 지울 여지가 없다. 남는 만큼이 피드백 몫이다.
    "avoid_offset_steer_frac": 0.60,
    # 계획을 다시 그리는 조건. 매 주기 다시 그리면 진입 곡선의 제일 급한
    # 앞부분만 반복해서 타게 되어(경로가 계속 자차로 리앵커됨) 조향이 안
    # 풀린다. 실제로 이전 구현이 그랬다.
    "avoid_offset_replan_lateral_m": 0.20,  # 계획 대비 이만큼 벗어나면 다시
    "avoid_offset_replan_obstacle_m": 0.35,  # 필요 오프셋이 이만큼 변하면 다시
    # 기동이 트랙에 안 들어가면 속도를 낮춰 다시 뽑는다. 길이가 v 에 비례해서
    # 절반 속도면 절반 길이다. 이 아래로는 안 내려가고 FGM 폴백으로 넘긴다 —
    # 그보다 느리면 기동으로 풀 상황이 아니라 제동으로 풀 상황이다.
    "avoid_offset_plan_v_floor_mps": 2.0,
    "avoid_offset_plan_v_step_mps": 0.5,
    "avoid_offset_step_m": 0.10,
    "avoid_frenet_step_m": 0.10,
    "avoid_frenet_hold_m": 0.60,       # apex 통과용 오프셋 유지 구간
    "avoid_frenet_enter_len_m": 1.2,   # 목표 오프셋까지 붙는 데 쓸 s 길이
    "avoid_frenet_exit_len_m": 1.5,    # d → 0 복귀에 쓸 s 길이
    "avoid_frenet_max_offset_m": 0.65,  # |d| 상한. 넘으면 클램프 → 막히면 TRAILING
    # ---- TRAILING (못 지나갈 때 갭 두고 따라가기) ----
    # 앞차를 옆으로 못 지나가는 상황에서 예전에는 AVOID 를 계속 시도하다
    # AEB 로 급정거했다. TRAILING 은 CSV 를 그대로 타면서 속도만 줄여
    # 일정 갭을 유지한다. 경로는 발행하지 않는다(override=False).
    "trailing_enable": True,
    "trailing_enter_m": 3.0,       # 전방 s 갭이 이 안이면 진입 (회피 불가일 때)
    "trailing_exit_m": 4.5,        # 이 밖으로 벗어나야 해제 (히스테리시스)
    "trailing_exit_count_th": 5,
    "trailing_target_gap_m": 1.5,  # 유지할 갭
    # 트랙 진행방향 절대속도가 이 값 이상이어야 "따라갈 앞차" 로 본다.
    # 미만이면(정지·역주행) 정적 장애물처럼 AVOID 로 넘긴다 — 서 있는 차
    # 뒤에 붙으면 갭만 지키며 영원히 안 움직인다.
    "trailing_min_leader_speed_mps": 0.5,
    # 앞차가 우리보다 이만큼 느리면 따라가지 않고 AVOID 로 비켜 간다.
    # 절대속도만 보면 1 m/s 로 기어가는 차 뒤에 붙어 같이 1 m/s 로 간다.
    # 레이싱이므로 기본 ON. 대신 임계를 넉넉히 잡는다 — 움직이는 차 옆을
    # 지나는 건 콘을 지나는 것과 달라서, 반응형 회피는 상대의 라인 변경을
    # 예측하지 못한다. 비슷한 속도면 붙어 가고, 확실히 느릴 때만 비켜 간다.
    "trailing_speed_deficit_enable": True,
    "trailing_max_speed_deficit_mps": 0.5,
    # 앞차 판정 히스테리시스 (40Hz 기준 프레임 수). 추적기의 static/dynamic
    # 플리커가 모드 떨림으로 새어 나오는 걸 막는다. → _update_leader_latch
    "leader_enter_count_th": 3,
    "leader_lost_count_th": 8,
    "trailing_kp": 0.45,
    "trailing_ki": 0.0,            # windup 위험 — 기본 0
    "trailing_kd": 0.25,
    # trailing_min_speed_scale 은 제거했다. 갭 제어가 "CSV 속도 × 배율" 에서
    # "앞차 속도 기준 절대속도" 로 바뀌면서 쓰이지 않게 됐고, 하한을 두면
    # 갭이 무너졌을 때 필요한 만큼 못 늦춰서 오히려 AEB 를 부른다.
    # TRAILING 이 제 몫을 하면 AEB 발동이 줄어야 한다. 그걸 눈으로 보려고
    # AEB 상승엣지를 세서 같이 찍는다 (AEB 노드는 건드리지 않는다).
    "trailing_log_hz": 1.0,
    "emergency_brake_topic": "/emergency_brake",
    # ---- AEB 탈출 (정지 후 회피경로 찾아 빠져나가기) ----
    # AEB 는 최종 안전망이라 "멈추는" 것까지만 한다. 그 뒤 빠져나가는 건
    # 플래너 몫인데, TRAILING 은 /local_path 를 안 내고 CSV 를 그대로 타므로
    # 장애물 정면에 멈춘 경우 조향할 경로가 없다. 결과는 0.6 m/s 로 기어가
    # 다시 AEB → 해제 → 또 기어감의 반복이고, 조금씩 장애물에 닿는다.
    #
    # 그래서 AEB 로 멈춘 뒤에는 TRAILING 대신 FGM 회피 경로를 강제로 발행한다.
    # 정지 중에 조향을 미리 돌려 두고, AEB 노드의 탈출 창이 열리면 그 방향으로
    # 빠져나간다.
    "aeb_escape_enable": True,
    # 이 속도 이하로 실제로 멈춘 뒤에만 경로를 바꾼다. 제동 중 고속에서
    # 조향을 새 경로로 틀면 거동이 예측 밖으로 간다.
    "aeb_escape_arm_speed_mps": 0.20,
    "aeb_escape_hold_sec": 2.0,      # AEB 해제 후 이만큼 더 탈출 모드 유지
    "aeb_escape_speed_mps": 0.8,     # 탈출 중 속도 상한 (기어 나가는 수준)
    # 탈출 중에는 짧은 경로도 받는다. 여기서 path_check_min_length_m(0.6) 를
    # 그대로 요구하면 정면이 막힌 상황에서 경로가 늘 기각돼 탈출이 안 된다.
    "aeb_escape_min_path_m": 0.25,
    # ---- 탈출 방향 고정 ----
    # FGM 은 "지금 제일 열린 각도" 만 본다. 장애물 정면에 멈추면 정면 섹터가
    # 통째로 막혀 있으므로 옆이 이기고, 조준각이 FOV 끝(±80°)까지 간다.
    # 탈출 속도 0.8 m/s 로 2 초면 1.6 m 인데, 최대 조향(0.3735 rad, 축거
    # 0.33) 의 회전반경이 0.85 m 라 그 사이 헤딩이 100° 넘게 돈다. 옆으로
    # 돌다가 역주행 방향이 되거나 벽에 붙는 게 이래서 생긴다.
    #
    # 그래서 멈춘 순간의 헤딩을 기억해 두고, 그 방향을 기준으로 조준각을
    # 제한한다. 차가 돌아간 만큼 선호각이 반대로 움직이므로 스스로 되돌아온다.
    "aeb_escape_heading_lock_enable": True,
    # 기억한 헤딩에서 이 각도 밖으로는 조준하지 않는다. 넓히면 좁은 틈도
    # 쓰지만 그만큼 많이 돌고, 좁히면 안 돌지만 빠져나갈 길이 줄어든다.
    "aeb_escape_heading_cone_deg": 55.0,
    "fgm_prefer_angle_topic": "/planner/fgm_prefer_angle",
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
    # ---- Frenet 스냅샷 (판정 로직 교체 아님, 정보 추가) ----
    # 매 주기 자차와 장애물을 CSV 폐곡선에 투영해 (s, d) 로 갖고 있는다.
    # 진행방향 거리를 유클리드가 아니라 s 차이로 재야 코너에서 "옆에 있는데
    # 앞이라고" 보는 오차가 없어진다.
    "publish_frenet_debug": False,
    "frenet_debug_topic": "/planner/frenet_debug",
    # 상대속도 등속 예측. True 면 회피 진입·갭 계산에 "현재 s" 대신
    # "pred_horizon_sec 뒤 s" 를 쓴다. 앞차가 빠르게 멀어지는 중이면
    # 불필요한 회피/감속을 줄여 준다. 궤적 학습 같은 건 하지 않는다.
    "use_predicted_s": False,
    "pred_horizon_sec": 1.0,
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

def _forbid_order(forbid_side: int) -> Tuple[int, ...]:
    """한 계획 속도에서 시도할 방향 순서.

    호출부가 이미 한쪽을 막았으면 그것만 존중한다. 안 막았으면 계획기가 고른
    쪽을 먼저 보고, 그게 벽이면 반대쪽도 **같은 속도에서** 시도한다 — 방향만
    바꾸면 될 일에 속도까지 깎을 이유가 없다.
    """
    if forbid_side != 0:
        return (forbid_side,)
    return (0, +1, -1)

def _min_opt(a: float | None, b: float | None) -> float | None:
    """둘 다 상한일 수 있고 둘 다 없을 수도 있는 값의 최솟값."""
    if a is None:
        return b
    if b is None:
        return a
    return min(a, b)

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
        self._obstacle_lateral_abs_max_corridor_m = max(
            self._obstacle_lateral_abs_max_m,
            float(self.get_parameter("obstacle_lateral_abs_max_corridor_m").value),
        )
        self._obstacle_tf_timeout = float(
            self.get_parameter("obstacle_tf_timeout_sec").value
        )
        # laser→map TF 주기 캐시 → _lookup_laser_to_map_transform
        self._tf_cycle_id = 0
        self._tf_cache_cycle = -1
        self._tf_cache = None

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
        self.avoid_on_late_scale = min(
            1.0, max(0.05, float(self.get_parameter("avoid_on_late_scale").value))
        )
        self.avoid_on_late_max_speed = max(
            0.0, float(self.get_parameter("avoid_on_late_max_speed").value)
        )
        self.avoid_on_late_blend_mps = max(
            0.05, float(self.get_parameter("avoid_on_late_blend_mps").value)
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
        self.rejoin_a_lat_mps2 = max(
            0.1, float(self.get_parameter("rejoin_a_lat_mps2").value)
        )
        self.avoid_a_accel_mps2 = max(
            0.0, float(self.get_parameter("avoid_a_accel_mps2").value)
        )
        self.deviation_speed_enable = param_bool(
            self.get_parameter("deviation_speed_enable").value
        )
        self.deviation_speed_free_m = max(
            0.05, float(self.get_parameter("deviation_speed_free_m").value)
        )
        self.rejoin_max_length_m = max(
            self.rejoin_min_length_m,
            float(self.get_parameter("rejoin_max_length_m").value),
        )
        # 속도 연동 합류각의 위/아래 한계. sin 으로 접어 둔다 (0° 는 무한 길이).
        hi_deg = min(60.0, max(3.0, float(self.get_parameter("rejoin_max_heading_deg").value)))
        lo_deg = min(hi_deg, max(3.0, float(self.get_parameter("rejoin_min_heading_deg").value)))
        self._rejoin_heading_sin_hi = math.sin(math.radians(hi_deg))
        self._rejoin_heading_sin_lo = math.sin(math.radians(lo_deg))
        self.rejoin_merge_overshoot_m = max(
            0.02, float(self.get_parameter("rejoin_merge_overshoot_m").value)
        )
        self.rejoin_track_lag_s = max(
            0.05, float(self.get_parameter("rejoin_track_lag_s").value)
        )
        self._rejoin_yaw_err_limit = math.radians(
            min(80.0, max(20.0, float(self.get_parameter("rejoin_yaw_err_limit_deg").value)))
        )
        # 정렬을 놓는 각은 진입각보다 낮다. 같은 문턱을 쓰면 두 가지가 겹쳐
        # 터진다 — 경계에서 정렬↔복귀가 떨리고, 더 나쁘게는 딱 한계각에서
        # 넘겨받은 quintic 이 tan(55°)=1.43 이라 대개 곡률 예산에 걸려 포기로
        # 간다. 문턱을 옮기기만 한 꼴이다. 여유를 두고 넘겨야 받는 쪽이
        # 실제로 풀 수 있다.
        self._alignment_release_rad = 0.6 * self._rejoin_yaw_err_limit
        self.rejoin_max_path_curvature = max(
            0.1, float(self.get_parameter("rejoin_max_path_curvature").value)
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
        self.rejoin_stall_speed_mps = max(
            0.0, float(self.get_parameter("rejoin_stall_speed_mps").value)
        )
        self.rejoin_stall_ns = int(
            max(0.1, float(self.get_parameter("rejoin_stall_sec").value)) * 1e9
        )
        self.rejoin_max_active_ns = int(
            max(0.5, float(self.get_parameter("rejoin_max_active_sec").value)) * 1e9
        )
        self._rejoin_start_ns = 0
        self._rejoin_moving_ns = 0
        self._rejoin_travel_m = 0.0
        self._rejoin_last_xy: tuple[float, float] | None = None
        self._rejoin_progress_cycle = -1
        # 지금 따라가는 재합류 경로의 실제 길이. 속도 상한을 이 길이로
        # 역산해야 "이 경로를 접지력 안에서 추종 가능한 속도" 가 나온다.
        self._rejoin_length_m = 0.0
        # 지금 경로의 실제 최대 곡률 (기준선 곡률 포함). 속도 상한을 여기서
        # 역산한다. 0 이면 아직 안 잰 것.
        self._rejoin_kappa_max = 0.0
        # 경로 길이에 맞춘 포기 시한. 경로를 그릴 때 다시 계산한다.
        self._rejoin_budget_ns = self.rejoin_max_active_ns
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
        self.avoid_cruise_speed_high_mps = max(
            0.0, float(g("avoid_cruise_speed_high_mps").value)
        )
        self.avoid_cruise_speed_low_mps = max(
            0.0, float(g("avoid_cruise_speed_low_mps").value)
        )
        self.avoid_cruise_high_speed_th = max(
            0.0, float(g("avoid_cruise_high_speed_th").value)
        )
        # 이번 회피에서 붙들고 있는 순항속도. None = 지금 회피 구간이 아님.
        self._avoid_cruise_latched: float | None = None
        # 방금 푼 값과 푼 시각. 짧게 끊겼다 다시 켜지면 이걸 되쓴다.
        self._avoid_cruise_prev: float | None = None
        self._avoid_cruise_release_ns = 0
        self.avoid_cruise_regrab_ns = int(
            max(0.0, float(g("avoid_cruise_regrab_sec").value)) * 1e9
        )
        self.wall_stop_check_enable = param_bool(g("wall_stop_check_enable").value)
        self.wall_stop_reaction_sec = max(
            0.0, float(g("wall_stop_reaction_sec").value)
        )
        self.avoid_speed_params = AvoidSpeedParams(
            a_lat=float(g("avoid_a_lat_mps2").value),
            a_brake=float(g("avoid_a_brake_mps2").value),
            safety_factor=float(g("avoid_safety_factor").value),
            standoff_m=float(g("avoid_standoff_m").value),
            # 검출 게이트(obstacle_lateral_abs_max_m)의 절반을 차 반폭으로 쓰던
            # 것을 실측 치수로 바꿨다. 그 게이트는 "무엇을 볼지" 를 정하는 값이라
            # 넉넉하게 잡혀 있어서, 반으로 나눠도 차폭이 되지 않았다 (0.21 vs 0.15).
            ego_half_width_m=vg.HALF_WIDTH_M,  # 이전 0.5*0.42 = 0.21
            ego_front_m=float(g("ego_front_safety_m").value),
            lateral_margin_m=float(g("avoid_lateral_margin_m").value),
            pass_clear_extra_m=float(g("avoid_pass_clear_extra_m").value),
            v_min=float(g("avoid_speed_min_mps").value),
        )
        self._inflated_map: InflatedMap | None = None
        # 레이스라인 점별 좌/우 오프셋 예산. 맵이 와야 깔린다 (`_build_wall_budget`).
        self._budget_left: np.ndarray | None = None
        self._budget_right: np.ndarray | None = None
        self._map_warned = False
        self._last_avoid_speed = float("nan")
        self._last_avoid_reason = ""
        self._last_path_cut = 0
        self._last_block_warn_ns = 0
        self._last_rejoin_warn_ns = 0
        self._last_align_warn_ns = 0
        self._rejoin_is_alignment = False
        self._cleared_len_m = float("inf")
        self._blocked_diag = None
        self._last_pose_for_speed: PoseStamped | None = None
        self._speed_static_obs: list = []
        self._speed_dynamic_obs: list = []
        self._slew_prev_v: float | None = None
        self._slew_prev_ns = 0
        self._override_active = False

        # Frenet 스냅샷 (매 주기 갱신). None = 이번 주기에 pose/TF 가 없었음.
        self._s_ego: float | None = None
        self._d_ego: float | None = None
        self._static_sd: list = []   # [(s, d, r), ...]
        self._dynamic_sd: list = []  # [(s, d, r, vs, closing), ...]
        self._publish_frenet_debug_enable = param_bool(g("publish_frenet_debug").value)
        self._use_predicted_s = param_bool(g("use_predicted_s").value)
        self._pred_horizon_sec = max(0.0, float(g("pred_horizon_sec").value))

        # AVOID 경로 모드
        self.avoid_path_mode = str(g("avoid_path_mode").value).strip().lower()
        if self.avoid_path_mode not in ("offset", "straight", "frenet"):
            self.get_logger().warn(
                f"avoid_path_mode='{self.avoid_path_mode}' 는 모르는 값 — offset 로 둔다"
            )
            self.avoid_path_mode = "offset"
        self.avoid_frenet_step_m = max(0.02, float(g("avoid_frenet_step_m").value))
        self.avoid_frenet_hold_m = max(0.0, float(g("avoid_frenet_hold_m").value))
        self.avoid_frenet_enter_len_m = max(
            0.3, float(g("avoid_frenet_enter_len_m").value)
        )
        self.avoid_frenet_exit_len_m = max(
            0.3, float(g("avoid_frenet_exit_len_m").value)
        )
        self.avoid_frenet_max_offset_m = max(
            0.05, float(g("avoid_frenet_max_offset_m").value)
        )
        self._last_frenet_avoid_warn_ns = 0

        # ---- 횡오프셋 기동 ----
        self.avoid_offset_step_m = max(0.02, float(g("avoid_offset_step_m").value))
        self.avoid_offset_plan_v_floor_mps = max(
            0.5, float(g("avoid_offset_plan_v_floor_mps").value)
        )
        self.avoid_offset_plan_v_step_mps = max(
            0.1, float(g("avoid_offset_plan_v_step_mps").value)
        )
        self.avoid_offset_replan_lateral_m = max(
            0.02, float(g("avoid_offset_replan_lateral_m").value)
        )
        self.avoid_offset_replan_obstacle_m = max(
            0.02, float(g("avoid_offset_replan_obstacle_m").value)
        )
        # ManeuverConfig 자체는 트랙 길이를 알아야 해서 CSV 를 읽은 뒤
        # `_build_maneuver_config()` 에서 만든다.
        # 계획은 한 번 세우면 붙들고 간다. 매 주기 다시 그리면 진입 곡선의
        # 제일 급한 앞부분만 반복해서 타게 되어 조향이 영영 안 풀린다.
        self._maneuver: OffsetManeuver | None = None
        self._maneuver_s0: float | None = None
        self._maneuver_last_s: float | None = None
        self._maneuver_ds_cache: float | None = None
        self._maneuver_speed_cap: float | None = None
        self._last_maneuver_log_ns = 0

        # TRAILING
        self.trailing_enable = param_bool(g("trailing_enable").value)
        self.trailing_enter_m = max(0.1, float(g("trailing_enter_m").value))
        self.trailing_exit_m = max(
            self.trailing_enter_m + 0.1, float(g("trailing_exit_m").value)
        )
        self.trailing_exit_count_th = max(1, int(g("trailing_exit_count_th").value))
        self.trailing_target_gap_m = max(0.1, float(g("trailing_target_gap_m").value))
        self.trailing_min_leader_speed_mps = max(
            0.0, float(g("trailing_min_leader_speed_mps").value)
        )
        self.trailing_speed_deficit_enable = param_bool(
            g("trailing_speed_deficit_enable").value
        )
        self.trailing_max_speed_deficit_mps = max(
            0.1, float(g("trailing_max_speed_deficit_mps").value)
        )
        self.leader_enter_count_th = max(1, int(g("leader_enter_count_th").value))
        self.leader_lost_count_th = max(1, int(g("leader_lost_count_th").value))
        self._leader_latched = False
        self._leader_seen_count = 0
        self._leader_lost_count = 0
        self.trailing_kp = float(g("trailing_kp").value)
        self.trailing_ki = float(g("trailing_ki").value)
        self.trailing_kd = float(g("trailing_kd").value)
        self._trailing_exit_count = 0
        self._trail_prev_err: float | None = None
        self._trail_prev_ns = 0
        self._trail_integral = 0.0
        # 회피 경로가 막혀 있는 동안의 시간 래치 (AVOID → TRAILING 근거).
        # bool 이면 전이 한 번에 리셋돼 모드가 프레임 단위로 떨린다.
        self.avoid_retry_ns = int(max(0.0, float(g("avoid_retry_sec").value)) * 1e9)
        self._avoid_blocked_until_ns = 0
        self.avoid_blocked_frames_th = max(
            1, int(g("avoid_blocked_frames_th").value)
        )
        self._avoid_blocked_frames = 0
        # 포기 판정이 서기 전까지 붙들 마지막 정상 회피 경로.
        self._last_good_avoid_path: Path | None = None
        self._last_good_avoid_ns = 0
        self.avoid_hold_max_ns = int(
            max(0.0, float(g("avoid_hold_max_sec").value)) * 1e9
        )
        self._aeb_count = 0
        self._aeb_active = False
        self.aeb_escape_enable = param_bool(g("aeb_escape_enable").value)
        self.aeb_escape_arm_speed = max(
            0.0, float(g("aeb_escape_arm_speed_mps").value)
        )
        self.aeb_escape_hold_ns = int(
            max(0.0, float(g("aeb_escape_hold_sec").value)) * 1e9
        )
        self.aeb_escape_speed_mps = max(0.1, float(g("aeb_escape_speed_mps").value))
        self.aeb_escape_min_path_m = max(
            0.0, float(g("aeb_escape_min_path_m").value)
        )
        self._aeb_escape_until_ns = 0
        self._aeb_escape_logged = False
        self.aeb_escape_heading_lock = param_bool(
            g("aeb_escape_heading_lock_enable").value
        )
        self.aeb_escape_heading_cone_rad = math.radians(
            max(0.0, float(g("aeb_escape_heading_cone_deg").value))
        )
        # 멈춘 순간의 맵 프레임 헤딩. 탈출이 끝나면 지운다.
        self._aeb_escape_yaw: float | None = None
        _tl_hz = max(0.0, float(g("trailing_log_hz").value))
        self._trailing_log_period_ns = int(1e9 / _tl_hz) if _tl_hz > 0.0 else 0
        self._last_trailing_log_ns = 0

        self.use_fgm = param_bool(self.get_parameter("use_fgm").value)
        self.avoid_fgm_min_speed_mps = max(
            0.0, float(self.get_parameter("avoid_fgm_min_speed").value)
        )
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
        self._obstacle_on = False
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
        self._build_maneuver_config()
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
        # 탈출 동작이 이 토픽에 걸려 있으므로 로그가 꺼져 있어도 구독한다.
        if self.aeb_escape_enable or (
            self.trailing_enable and self._trailing_log_period_ns > 0
        ):
            self.create_subscription(
                Bool, str(g("emergency_brake_topic").value), self._cb_aeb, 10
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
        # 어느 정책이 속도를 깎았는지 (stop/maneuver/trailing/deviation/...).
        # 배율만 봐서는 "왜 기어갔는지" 를 사후에 알 수 없다.
        self.pub_speed_reason = self.create_publisher(
            String, "/planner/speed_reason", 10
        )
        # 지금 내보내는 /local_path 가 **계획된 기하 경로**인가.
        #
        # Stanley 는 LOCAL_PATH 모드에서 FF 를 끈다. FGM 폴백 경로는 조준점까지
        # 그은 직선이라 그 곡률이 기하학적 의미가 없어서 맞는 처리다. 하지만
        # 횡오프셋 기동은 곡률이 정확히 우리가 타려는 값이라 FF 가 처리해야
        # 한다. 안 그러면 그 곡률을 오차가 쌓인 뒤 피드백으로 뒤늦게 만들어야
        # 하고, 마침 그 피드백에는 횡가속 상한(6 m/s 에서 2.1°)이 걸려 있어서
        # 계획을 따라가지 못한다.
        self.pub_path_planned = self.create_publisher(
            Bool, "/planner/local_path_planned", 10
        )
        self._path_planned = False
        self.pub_frenet_debug = (
            self.create_publisher(
                Float32MultiArray, str(g("frenet_debug_topic").value), 10
            )
            if self._publish_frenet_debug_enable
            else None
        )
        fgm_en_topic = self.get_parameter("fgm_enable_topic").value
        self.pub_fgm_enable = self.create_publisher(Bool, fgm_en_topic, 10)
        self.pub_fgm_prefer = self.create_publisher(
            Float32MultiArray, str(g("fgm_prefer_angle_topic").value), 10
        )
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
        """laser→map TF. 한 주기 안에서는 한 번만 조회한다.

        한 주기에 이 함수가 4~6번 불린다 (코리도 필터 ×3, 이탈 판정, 경로
        충돌검사…). TF 가 정상일 때는 버퍼 조회라 싸지만, 끊기면 호출마다
        timeout(기본 0.15초) 만큼 블로킹돼 40 Hz 주기가 1~2 Hz 로 주저앉는다.
        측정값: TF 없는 상태에서 _update_mode 한 번에 650 ms.

        같은 주기 안에서 TF 가 변할 이유는 없으니 첫 결과를 재사용한다.
        실패도 캐시한다 — 비싼 쪽이 실패다.
        """
        if self._tf_cache_cycle == self._tf_cycle_id:
            return self._tf_cache
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.laser_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._obstacle_tf_timeout),
            )
        except TransformException:
            tf = None
        self._tf_cache_cycle = self._tf_cycle_id
        self._tf_cache = tf
        return tf

    @property
    def _gated_lateral_abs_max_m(self) -> float:
        """코리도를 이미 통과한 목록에 다시 걸 |y| 상한.

        코리도가 도는 동안에는 "레이스라인 위에 있는가" 를 이미 맵 좌표로
        판정했으므로, 여기서 좁은 직선 튜브를 또 걸면 곡선 구간 장애물만
        골라서 버리게 된다.
        """
        return (
            self._obstacle_lateral_abs_max_corridor_m
            if self._raceline_corridor_enable
            else self._obstacle_lateral_abs_max_m
        )

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
            lateral_abs_max_corridor_m=self._obstacle_lateral_abs_max_corridor_m,
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
            lateral_abs_max_corridor_m=self._obstacle_lateral_abs_max_corridor_m,
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
            lateral_abs_max_corridor_m=self._obstacle_lateral_abs_max_corridor_m,
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
            lateral_abs_max_corridor_m=self._obstacle_lateral_abs_max_corridor_m,
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
            lateral_abs_max_m=max(
                self.avoid_trigger_lateral_abs_max_m,
                self._gated_lateral_abs_max_m,
            ),
            laser_to_base_x_m=self.laser_to_base_x_m,
        )
        d_gate = closest_dynamic_obstacle_surface_m(
            filtered_dynamic,
            forward_cone_rad=None,
            min_forward_x_m=self.avoid_min_forward_x_m,
            lateral_abs_max_m=self._gated_lateral_abs_max_m,
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

    def _avoid_on_late_factor(self) -> float:
        """저속에서 로컬패스 전환을 늦추는 배율.

        FGM 은 그대로 켜 둔다 (`fgm_enable_m` 은 안 건드린다). 늦추는 건
        **CSV → local_path 로 갈아타는 시점**뿐이다. 저속에서는 일찍 갈아타
        봐야 장애물이 아직 멀어서 조준이 흔들리고, 그동안 레이스라인을 놓친다.

        고속은 손대지 않는다 — 거기서 늦추면 피할 거리가 안 나온다.
        문턱에서 게이트가 튀면 AVOID↔GLOBAL 이 떨릴 수 있어 blend 를 둔다.
        """
        v = abs(self._ego_speed_mps)
        w = (v - self.avoid_on_late_max_speed) / self.avoid_on_late_blend_mps
        w = min(1.0, max(0.0, w))
        return self.avoid_on_late_scale + w * (1.0 - self.avoid_on_late_scale)

    def _effective_avoid_gates(self) -> tuple[float, float, float]:
        """속도 기반 (avoid_on, avoid_off, fgm_enable) [m]."""
        on_m = self._speed_scaled_dist(
            self.avoid_on_m, self.avoid_on_min_m, self.avoid_on_max_m
        )
        on_m *= self._avoid_on_late_factor()
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
        self._build_wall_budget()

    # 오프셋 예산을 훑을 때의 전진 간격 [m]. 맵 해상도(0.05)의 절반이면
    # 격자를 건너뛰지 않는다.
    _WALL_BUDGET_STEP_M = 0.025

    def _build_wall_budget(self) -> None:
        """레이스라인 각 점에서 **좌/우로 얼마나 비킬 수 있는지** 를 미리 잰다.

        기동 계획기는 장애물만 보고 트랙 경계를 모른다. 원래는 경로 충돌검사가
        그걸 받아 주기로 했는데, 실측해 보니 그 가정이 틀렸다. 이 트랙에서
        레이스라인→벽 여유는 중앙값 0.70 m / 최소 0.30 m 라, 상한으로 박아 둔
        0.70 m 는 **구간의 76% 에서 낼 수 없는 값**이다. 벽 쪽으로 비키는 계획이
        일상적으로 나왔다는 뜻이다.

        중요한 건 좌우가 다르다는 점이다. 레이스라인은 코너 안쪽에 붙으므로
        한쪽은 벽이고 반대쪽은 트랙이 통째로 남는다. 실측 좌 중앙값 0.88 m /
        우 0.78 m 인데, **둘 중 좋은 쪽**을 고르면 최소가 0.58 m 로 올라간다 —
        즉 방향만 제대로 고르면 이 트랙 어디서든 비킬 수 있다. 그래서 예산을
        한 개가 아니라 좌/우 두 개로 만든다.

        판정 기준은 경로 충돌검사와 **같은 팽창맵**이다. 다른 기준을 쓰면
        계획기는 된다고 하고 검사기는 아니라고 하는 상태가 생긴다.
        """
        im = self._inflated_map
        if im is None or self._n < 3 or self._total_l < 1e-6:
            self._budget_left = None
            self._budget_right = None
            return

        tx = np.roll(self._xs_np, -1) - np.roll(self._xs_np, 1)
        ty = np.roll(self._ys_np, -1) - np.roll(self._ys_np, 1)
        norm = np.hypot(tx, ty)
        norm[norm < 1e-9] = 1.0
        # 좌 법선 (+d 방향)
        nx, ny = -ty / norm, tx / norm

        step = self._WALL_BUDGET_STEP_M
        cap = self.maneuver_cfg.max_offset_m

        def side_budget(sign: float) -> np.ndarray:
            budget = np.zeros(self._n, dtype=np.float64)
            alive = np.ones(self._n, dtype=bool)
            t = step
            while t <= cap + 1e-9:
                free = ~im.blocked_many(
                    self._xs_np + sign * nx * t, self._ys_np + sign * ny * t
                )
                # 한 번 막히면 그 뒤는 안 본다. 벽 너머의 빈 공간이 예산으로
                # 잡히면 안 된다.
                alive &= free
                if not alive.any():
                    break
                budget[alive] = t
                t += step
            return budget

        self._budget_left = side_budget(+1.0)
        self._budget_right = side_budget(-1.0)
        best = np.maximum(self._budget_left, self._budget_right)
        self.get_logger().info(
            f"오프셋 예산 (상한 {cap:.2f}m): "
            f"좌 최소 {self._budget_left.min():.2f} 중앙 "
            f"{float(np.median(self._budget_left)):.2f}m, "
            f"우 최소 {self._budget_right.min():.2f} 중앙 "
            f"{float(np.median(self._budget_right)):.2f}m, "
            f"좋은쪽 최소 {best.min():.2f}m"
        )

    def _index_at_s(self, s: float) -> int:
        return int((s % self._total_l) / self._total_l * self._n) % self._n

    def _wall_budget_over(self, s_from: float, s_to: float) -> Tuple[float, float]:
        """[s_from, s_to] 구간에서 **끝까지 유지할 수 있는** 좌/우 오프셋.

        구간 최솟값을 쓴다. 기동은 그 구간 내내 오프셋을 물고 있으므로,
        한 점이라도 못 내면 그 계획은 못 쓴다.
        """
        if self._budget_left is None or self._total_l < 1e-6:
            cap = self.maneuver_cfg.max_offset_m
            return cap, cap
        i0 = self._index_at_s(s_from)
        i1 = self._index_at_s(s_to)
        if self._delta_s(s_from, s_to) >= self._total_l - 1e-6:
            sl = slice(None)
            bl, br = self._budget_left[sl], self._budget_right[sl]
        elif i0 <= i1:
            bl = self._budget_left[i0 : i1 + 1]
            br = self._budget_right[i0 : i1 + 1]
        else:  # 랩어라운드
            bl = np.concatenate((self._budget_left[i0:], self._budget_left[: i1 + 1]))
            br = np.concatenate((self._budget_right[i0:], self._budget_right[: i1 + 1]))
        if bl.size == 0:
            cap = self.maneuver_cfg.max_offset_m
            return cap, cap
        return float(bl.min()), float(br.min())

    def _obstacle_disks_map(self, tf_lm) -> list:
        """장애물을 맵 좌표 원판 [(x, y, r), ...] 으로. 반경엔 차폭이 포함된다."""
        if tf_lm is None:
            return []
        to_map = self._make_laser_to_map_fn(tf_lm)
        # 맵 팽창과 같은 기준(코너 스윕폭)을 써야 한다. 벽은 0.254 로 재고
        # 장애물은 회피속도용 반폭 0.15 로 재면 같은 경로가 두 검사에서 다르게
        # 판정된다.
        grow = vg.PATH_CHECK_HALF_WIDTH_M + self.path_check_obstacle_margin_m
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

    def _truncate_path_at_collision(
        self, path: Path, tf_lm, min_length_m: float | None = None
    ) -> tuple[Path, bool]:
        """회피 경로를 첫 충돌 지점 앞에서 자른다. (경로, 쓸만한가).

        FGM 목표점 너머 직선 연장이 벽을 향하는 경우가 이걸로 걸린다.
        남은 길이가 너무 짧으면 회피 자체를 포기한다 — 그 짧은 경로를 주면
        Stanley 가 끝점에서 이상하게 돌고, 차라리 CSV 로 두고 AEB 에 맡기는
        편이 안전하다.

        min_length_m 은 그 최소 길이를 이번 호출에만 바꾼다. AEB 탈출 중에는
        기본값(0.6 m)을 요구하면 경로가 늘 기각돼 빠져나갈 방법이 없어서,
        저속인 걸 전제로 더 짧은 경로를 받는다.
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
            self._cleared_len_m = float("inf")
            return path, True

        kept = trim_back(pts, cut, self.path_check_backoff_m)
        length = 0.0
        for i in range(1, kept):
            length += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])

        min_len = (
            self.path_check_min_length_m if min_length_m is None else min_length_m
        )
        if length < min_len:
            self._blocked_diag = (pts, cut, length, min_len, tf_lm)
            return path, False

        # 확보 길이는 버리는 근거가 아니라 **속도 상한** 이다.
        #
        # 예전에는 여기서 정지거리(`_wall_stop_distance_m`)까지 요구해서, 그
        # 안에 못 서면 경로를 통째로 버렸다. 그 기준은 `v²` 로 자라기 때문에
        # 빠를수록 더 긴 확보를 요구한다 — 정작 급할 때 제일 엄격해진다.
        # 실측(20260822) 세 건 전부 벽에서 잘렸고, 장애물은 훨씬 뒤(42~55번째
        # 점)에서야 걸렸다. 즉 **장애물은 잘 피하는 경로** 였는데, 그 중 둘은
        # 요구치에 4 cm 모자라서 버려졌다 (2.10 < 2.14, 1.05 < 1.09).
        #
        # 버린 뒤의 대안이 문제다. 회피에 들어간 이유가 "라인 위의 장애물"
        # 이므로 CSV 는 정의상 그 장애물을 향한다. 짧아도 비켜 가는 경로를
        # 버리고 정면으로 가는 경로를 택하는 셈이라, 기각이 곧 충돌 코스다.
        # 실제로 4.6 m/s 에서 0.46 초 동안 CSV 로 직진하다 AEB 가 받았다.
        #
        # 지켜야 할 명제는 "확보한 길이 안에서 설 수 있어야 한다" 이지
        # "설 수 없으면 그 경로를 쓰지 말라" 가 아니다. 전자는 속도로 지킬 수
        # 있고, 50 Hz 로 다시 계획하므로 다음 주기에 더 긴 경로가 나온다.
        self._cleared_len_m = length
        path.poses = path.poses[:kept]
        return path, True

    def _wall_stop_distance_m(self) -> float:
        """지금 속도로 벽 앞에 서려면 최소 몇 m 가 열려 있어야 하는가.

        회피 경로는 벽에 걸리면 잘려서 나간다. 그런데 받아들이는 기준이 고정
        길이(`path_check_min_length_m`, 0.6 m)뿐이라, 6 m/s 에서는 0.1 초짜리
        경로를 "쓸 만하다" 며 내보냈다. 차는 그 짧은 경로가 가리키는 대로
        벽을 향해 돌고, 다음 주기에 다시 잘린 경로를 받는다 — 레이스라인이
        벽에 붙어 있는 구간에서 FGM 이 바깥쪽 갭을 고르면 이게 그대로 사고다.

        그래서 길이 기준을 속도로 만든다. 잘린 끝이 벽이므로, 그 앞에서 설 수
        있어야 그 경로를 받을 자격이 있다.

            제동거리 = v·t_react + v²/(2a)

        모자라면 회피를 포기하고 CSV 를 유지한다. 그러면 속도 정책이 회피
        순항속도(`_avoid_cruise_target`)를 걸고, 느려진 다음
        주기에는 같은 경로도 통과한다 — "박을 것 같으면 속도를 줄인다" 가
        이렇게 나온다. 끝까지 안 되면 AEB 가 받는다.
        """
        if not self.wall_stop_check_enable:
            return 0.0
        v = abs(self._ego_speed_mps)
        if v < 1e-3:
            return 0.0
        a = max(0.1, self.avoid_speed_params.a_brake)
        return v * self.wall_stop_reaction_sec + v * v / (2.0 * a)

    def _cleared_path_speed_limit(self) -> float:
        """확보된 경로 안에서 설 수 있는 최대 속도 [m/s].

        `_wall_stop_distance_m` 의 역함수다. 그쪽은 "이 속도면 몇 m 필요한가",
        여기는 "이만큼 확보됐으면 몇 m/s 까지 되는가" 를 묻는다.

            L = v·t + v²/(2a)   →   v = -a·t + sqrt((a·t)² + 2aL)

        경로가 잘리지 않았으면 상한이 없다. 잘렸을 때만 그 끝(=벽)을 존중해서
        속도를 깎는다. 다음 주기에 더 긴 경로가 나오면 상한도 같이 풀린다.
        """
        if not self.wall_stop_check_enable:
            return float("inf")
        length = getattr(self, "_cleared_len_m", float("inf"))
        if not math.isfinite(length):
            return float("inf")
        a = max(0.1, self.avoid_speed_params.a_brake)
        at = a * self.wall_stop_reaction_sec
        return max(0.0, -at + math.sqrt(at * at + 2.0 * a * max(0.0, length)))

    def _path_fully_clear(self, path: Path, tf_lm) -> bool:
        """경로 전체가 벽·장애물에 안 걸리는가. 한 점이라도 걸리면 False."""
        if not self.path_check_enable or len(path.poses) < 2:
            return True
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        cut = first_blocked_index(
            pts, self._inflated_map, self._obstacle_disks_map(tf_lm), start_index=1
        )
        self._last_path_cut = cut
        return cut >= len(pts)

    def _warn_avoid_path_blocked(self) -> None:
        """회피 경로가 통째로 막힘 (1초에 한 번). 감속·정지는 속도정책과 AEB 몫.

        무엇에 걸렸는지까지 찍는다. "막혔다" 만 알면 손댈 곳이 안 나온다 —
        벽이면 FGM 이 못 지나갈 갭을 고른 것이고, 장애물이면 오프셋이 모자란
        것이라 고칠 데가 정반대다. 길이도 같이 찍는다. 요구 길이는 정지거리라
        `v²` 로 늘어나서, 빠를수록 더 긴 확보를 요구한다 — 정작 급할 때 제일
        엄격해지는 구조라 그 격차가 보여야 판단이 된다.
        """
        now = self.get_clock().now().nanoseconds
        if now - getattr(self, "_last_block_warn_ns", 0) < 1_000_000_000:
            return
        self._last_block_warn_ns = now

        diag = getattr(self, "_blocked_diag", None)
        detail = ""
        if diag is not None:
            pts, cut, length, min_len, tf_lm = diag
            wall_cut = first_blocked_index(pts, self._inflated_map, None, start_index=1)
            disk_cut = first_blocked_index(
                pts, None, self._obstacle_disks_map(tf_lm), start_index=1
            )
            if wall_cut <= disk_cut:
                cause = f"벽 (벽 {wall_cut}, 장애물 {disk_cut})"
            else:
                cause = f"장애물 (장애물 {disk_cut}, 벽 {wall_cut})"
            detail = (
                f" 원인={cause}"
                f" 확보 {length:.2f} m < 요구 {min_len:.2f} m"
                f" (v={abs(self._ego_speed_mps):.1f})"
            )
            tgt = self._fgm_target_fresh()
            if tgt is not None:
                aim = math.degrees(math.atan2(tgt.point.y, tgt.point.x))
                detail += f" FGM={aim:+.0f}°"

        self.get_logger().warn(
            f"회피 경로가 {self._last_path_cut}번째 점에서 막힘 — 쓸 만한 길이가 "
            f"안 나와 회피 포기, CSV 유지. 감속 후 AEB 가 받는다.{detail}"
        )

    def _warn_frenet_avoid_fallback(self) -> None:
        now = self.get_clock().now().nanoseconds
        if now - self._last_frenet_avoid_warn_ns < 2_000_000_000:
            return
        self._last_frenet_avoid_warn_ns = now
        self.get_logger().warn(
            "frenet 회피 경로 생성 실패 — straight 방식으로 대체한다."
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

    def _avoid_target_speed(self, *, avoiding: bool) -> tuple[float, str]:
        """회피 물리 목표속도 [m/s] — slew 전 원본.

        avoiding=False 는 접근 구간(GLOBAL) 선감속. 조향 한계는 빼고 거리
        기반만 건다.
        """
        # FGM 목표점을 차량 기준 (전방, 횡) 으로 — 조향이 얼마나 급한지가 여기서 나온다
        fwd, lat = 2.0, 0.0
        passing = avoiding
        path_lat_at = None

        if self._maneuver is not None and self._d_ego is not None:
            # 계획 기동 중이면 "앞으로 낼 횡이동" 을 정확히 알고 있다. FGM
            # 조준점보다 이쪽이 맞다 — 실제로 그 경로를 탈 것이기 때문이다.
            # 조향 한계(maneuver)는 계획이 이미 예산 안으로 맞춰 놨으므로
            # 끄고, 정지거리 면제(passing)만 받는다.
            lat = self._maneuver.d_pass - self._d_ego
            fwd = max(0.1, self._maneuver.obstacle_s_first)
            avoiding = False
            passing = True
            path_lat_at = self._maneuver_lat_at
        else:
            tgt = self._fgm_target_fresh()
            if tgt is not None:
                # /fgm_target 은 laser frame 이라 그대로 전방/횡으로 쓸 수 있다
                fwd = max(0.1, float(tgt.point.x))
                lat = float(tgt.point.y)
            else:
                # 목표가 없거나 오래됐으면 조향 한계를 걸 근거가 없다
                avoiding = False
                passing = False

        v, reason = avoid_speed_limit(
            self._speed_static_obs,
            self._speed_dynamic_obs,
            self._ego_speed_mps,
            fwd,
            lat,
            self.avoid_speed_params,
            laser_to_base_x_m=self.laser_to_base_x_m,
            include_maneuver=avoiding,
            passing=passing,
            path_lat_at=path_lat_at,
        )
        # "maneuver" 는 FGM 조준각을 낼 수 있는 속도라는 뜻이다. FGM 은 이미
        # 저속에서만 큰 각이 나오는데, 거기서 조준각을 이유로 또 세우면
        # 장애물 앞에서 기어가다 멈춘다. 하한을 둔다 — 실제로 못 지나가는
        # 상황이면 static/dynamic 항이나 AEB 가 따로 받는다.
        if reason == "maneuver" and v < self.avoid_fgm_min_speed_mps:
            v = self.avoid_fgm_min_speed_mps
        return v, reason

    def _trailing_target_speed(self, v_csv: float) -> float:
        """갭 유지 목표속도 [m/s] (slew 전).

        기준은 **앞차 속도** 다. 예전에는 CSV 속도에 배율을 곱했다:

            raw = 1.0 + kp*err + kd*derr      # err = gap - target_gap
            v   = raw * v_csv

        갭이 목표에 맞으면 err=0 → raw=1.0 → **CSV 전속** 이 나온다. 앞차가
        1.2 m/s 로 가는데 자차는 5 m/s 를 명령하니 갭이 순식간에 무너지고,
        그때서야 err 가 음수로 커져 급제동한다. 서면 갭이 벌어져 다시 전속.
        이 왕복이 "갔다 멈췄다" 하는 버벅임의 정체다. 정상상태가 없는 제어다.

        앞차 속도를 기준으로 두면 err=0 에서 v = v_lead 라 정상상태가 생긴다.
        갭 오차는 그 위에 얹는 보정이다 (adaptive cruise 의 기본형).
        """
        gap, v_lead = self._forward_leader()
        if not math.isfinite(gap):
            self._trail_prev_err = None
            self._trail_integral = 0.0
            return v_csv

        now_ns = self.get_clock().now().nanoseconds
        err = gap - self.trailing_target_gap_m
        derr = 0.0
        if self._trail_prev_err is not None and self._trail_prev_ns > 0:
            dt = (now_ns - self._trail_prev_ns) * 1e-9
            if 0.0 < dt < 0.5:
                derr = (err - self._trail_prev_err) / dt
                if self.trailing_ki != 0.0:
                    self._trail_integral = max(
                        -2.0, min(2.0, self._trail_integral + err * dt)
                    )
        self._trail_prev_err = err
        self._trail_prev_ns = now_ns

        v = (
            max(0.0, v_lead)
            + self.trailing_kp * err
            + self.trailing_ki * self._trail_integral
            + self.trailing_kd * derr
        )

        # 제동거리 상한. P 항만 두면 갭이 넓을 때(err 가 클 때) CSV 전속을
        # 명령하고, 목표갭에 닿았을 땐 이미 그 거리 안에서 못 서는 속도가 돼
        # 있다. 그러면 AEB 가 대신 잡는데, AEB 는 역토크라 급정거 → 갭이
        # 벌어짐 → 다시 전속 … 으로 버벅인다. 접근 자체를 "목표갭에 맞춰
        # 설 수 있는 속도" 로 제한해야 AEB 를 안 부른다.
        #   v ≤ v_lead + √(2·a·여유)      (여유 = gap - target_gap)
        slack = max(0.0, err)
        v_cap = max(0.0, v_lead) + math.sqrt(
            2.0 * self.avoid_speed_params.a_brake * slack
        )
        v = min(v, v_cap)

        # 앞차보다 빨리 갈 이유는 없고(추월 로직 없음), CSV 속도도 못 넘는다.
        return min(v_csv, max(0.0, v))

    def _rejoin_speed_length_m(self) -> float:
        """속도 상한을 역산할 때 쓸 재합류 길이 [m].

        `REJOIN` 중에는 **지금 실제로 따라가는 경로의 길이** 를 쓴다. 그래야
        상한이 "이 경로를 접지력 안에서 추종 가능한 속도" 라는 뜻이 된다.
        `rejoin_max_length_m` 을 쓰면 상한이 이탈 1.6 m 에서도 전속 위라
        사실상 안 걸리고, 복귀 내내 CSV 전속이 나가 라인에 세게 꽂힌다.

        그 밖(GLOBAL/AVOID 에서 크게 벗어난 상태)에서는 아직 경로가 없으니
        "최대한 길게 잡아도 가능한가" 를 묻는 최대 길이를 쓴다.
        """
        if self.mode == "REJOIN" and self._rejoin_length_m > 1e-3:
            return self._rejoin_length_m
        return self.rejoin_max_length_m

    def _avoid_speed_capped(self) -> bool:
        """지금이 "라인을 벗어나 피해야 하는" 구간인가.

        AVOID/REJOIN 은 이미 벗어나 있거나 돌아오는 중이다. 접근 구간
        (`_obstacle_on`)까지 포함해야 장애물 앞에 **도착했을 때 이미** 그
        속도다 — AVOID 로 바뀐 뒤에 줄이기 시작하면 늦는다.
        """
        return self.mode in ("AVOID", "REJOIN") or self._obstacle_on

    def _avoid_cruise_target(self) -> float:
        """이번 회피에서 유지할 속도 [m/s]. 0 이면 안 건다.

        회피 구간에 처음 들어설 때의 속도로 고속/저속을 가르고, 구간이 끝날
        때까지 그 값을 붙든다. 접근에서 복귀까지 한 속도여야 회피 도중에
        목표가 바뀌지 않는다.

        붙드는 게 필수다. 목표(3.0)가 문턱(4.0)보다 낮아서, 안 붙들면
        5 m/s 로 진입해 고속(3.0)을 고른 차가 감속 도중 4.0 을 지나며
        저속(2.0)으로 또 내려간다.

        구간을 벗어나면(=글로벌패스 복귀) 즉시 풀어서 CSV 속도로 돌아간다.
        다만 `avoid_cruise_regrab_sec` 안에 다시 켜지면 **직전 값을 되쓴다** —
        접근 중 검출이 한 프레임 깜빡이거나 모드가 잠시 GLOBAL 로 튕기는
        것만으로 목표가 재결정되면 안 되기 때문이다.
        """
        now = self.get_clock().now().nanoseconds
        if not self._avoid_speed_capped():
            if self._avoid_cruise_latched is not None:
                self._avoid_cruise_prev = self._avoid_cruise_latched
                self._avoid_cruise_release_ns = now
                self._avoid_cruise_latched = None
            return 0.0
        if self._avoid_cruise_latched is None:
            recent = (
                self._avoid_cruise_prev is not None
                and now - self._avoid_cruise_release_ns <= self.avoid_cruise_regrab_ns
            )
            if recent:
                self._avoid_cruise_latched = self._avoid_cruise_prev
            else:
                fast = abs(self._ego_speed_mps) > self.avoid_cruise_high_speed_th
                self._avoid_cruise_latched = (
                    self.avoid_cruise_speed_high_mps
                    if fast
                    else self.avoid_cruise_speed_low_mps
                )
        return self._avoid_cruise_latched

    def _deviation_speed_limit(self) -> float:
        """레이스라인에서 멀어진 만큼 속도를 낮춘다.

        quintic `d(s)` 의 최대 |d″| 는 `5.77·|d|/L²` 이고 요구 횡가속도는
        여기에 `v²` 가 곱해진다. 그 값을 예산 이하로 두는 속도가
        `v = L·sqrt(a_lat / (5.77·|d|))` 다.

        `REJOIN` 중에는 `L` 이 실제 경로 길이라, |d| 가 줄수록 상한이 저절로
        커진다 — 라인에 붙어 갈수록 속도가 자연스럽게 원래대로 회복된다.
        회복 기울기는 `avoid_a_accel_mps2` 가 따로 묶는다.

        AEB 나 큰 회피로 경로에서 크게 벗어난 채 고속을 유지하면 복귀 시점에
        조향이 포화되어 벽으로 밀린다. 그 전에 속도를 미리 깎는 게 목적이다.
        이탈이 작을 때는 상한이 매우 커서 걸리지 않는다.
        """
        if not self.deviation_speed_enable:
            return float("inf")
        pose = self._last_pose_for_speed
        if pose is None:
            return float("inf")

        # 계획 기동 중의 이탈은 **의도된** 것이고, 그 경로의 곡률은 계획할 때
        # 이미 예산 안으로 맞춰 뒀다 (`_maneuver_speed_cap`). 여기서 또 깎으면
        # 오프셋에 올라선 것만으로 감속이 걸려, 조향을 아끼려고 옆으로 비킨
        # 대가를 속도로 치르게 된다.
        if self._maneuver is not None:
            return float("inf")

        # REJOIN 중이면 경로가 기준선보다 **얼마나 더** 휘는지 이미 재 뒀다.
        # 코너 안쪽 복귀는 기준선 곡률이 1/σ 로 증폭돼 들어가서, 직선 가정은
        # 이 초과분을 통째로 놓친다. 잰 값이 있으면 그걸 쓴다.
        if self.mode == "REJOIN" and self._rejoin_kappa_max > 1e-6:
            return max(
                self.avoid_speed_params.v_min,
                math.sqrt(self.rejoin_a_lat_mps2 / self._rejoin_kappa_max),
            )

        d = self._csv_cte_abs_m(pose)
        if not math.isfinite(d) or d <= self.deviation_speed_free_m:
            return float("inf")
        v = self._rejoin_speed_length_m() * math.sqrt(
            self.rejoin_a_lat_mps2 / (self._QUINTIC_D2_PEAK * d)
        )
        return max(self.avoid_speed_params.v_min, v)

    def _planner_speed_scale(self) -> float:
        """회피 선감속과 TRAILING 갭 유지 중 더 느린 쪽. slew 는 마지막에 한 번만.

        두 정책이 각자 slew 를 돌리면 두 번째 호출이 dt≈0 이라 제한이 풀린다.
        """
        v_csv = self._csv_speed_now()
        trailing = self.trailing_enable and self.mode == "TRAILING"

        if not self.avoid_speed_enable:
            base = self.rejoin_speed_scale if self._override_active else 1.0
            if not trailing:
                return base
            scale = self._trailing_target_speed(v_csv) / max(0.05, v_csv)
            return min(base, scale)

        # 기동 속도 한계는 **지금 따라가는 경로** 의 곡률에서 나와야 한다.
        # AVOID 는 FGM 목표를 쫓으니 그 조준각이 맞다. REJOIN 은 재합류
        # 경로를 쫓으므로 FGM 조준각과 무관하고, 그쪽 곡률은 아래
        # `_deviation_speed_limit` 이 이미 건다 (v = L·√(a_lat/(5.77|d|))).
        #
        # 여기서 mode 를 안 보면 "하지도 않는 조향" 을 이유로 제동한다.
        # 예전엔 REJOIN 중 FGM 이 꺼져 목표가 상하면서 한계가 저절로
        # 빠졌는데, 연속 장애물 대응으로 FGM 을 켜 두자 되살아났다.
        # 실측: REJOIN 중 조준각이 40° 를 넘고 속도가 0.1 m/s 까지 떨어졌다.
        #
        # 계획 기동(offset) 중에도 마찬가지다. 그때 따라가는 건 계획된 d(s)
        # 이지 FGM 조준점이 아니고, 그 경로의 곡률 한계는 계획 단계에서
        # `_maneuver_speed_cap` 으로 이미 나와 있다. 여기서 FGM 조준각을
        # 근거로 또 깎으면 "옆으로 살짝 비켜 지나가려는" 기동이 매번 감속을
        # 부르게 되어, 감속을 줄이려고 만든 기동이 감속을 만든다.
        maneuvering = (
            self._override_active
            and self.mode == "AVOID"
            and self._maneuver is None
        )
        v_target, reason = self._avoid_target_speed(avoiding=maneuvering)
        if trailing:
            v_trail = self._trailing_target_speed(v_csv)
            if v_trail < v_target:
                v_target, reason = v_trail, "trailing"

        if self._aeb_escape_active():
            # 탈출은 "기어 나가는" 동작이다. 여기서 상한을 안 걸면 장애물이
            # 시야에서 빠지는 순간 CSV 전속으로 튀어 나간다.
            if self.aeb_escape_speed_mps < v_target:
                v_target, reason = self.aeb_escape_speed_mps, "aeb_escape"

        v_rejoin = self._deviation_speed_limit()
        if v_rejoin < v_target:
            v_target, reason = v_rejoin, "deviation"

        # 계획된 기동이 예산을 넘는다 = 늦게 봤다. 조향을 더 넣는 대신
        # 여기서 속도로 답한다. 여유 있게 만난 경우엔 None 이라 감속이 없다.
        if self._maneuver_speed_cap is not None and self.mode == "AVOID":
            if self._maneuver_speed_cap < v_target:
                v_target, reason = self._maneuver_speed_cap, "maneuver"

        # 회피 구간은 순항속도 하나로 간다 — 상한이자 하한이다.
        #
        # 위 한계들은 각자 다른 걸 본다 (조향 가능성, 정지거리, 이탈량).
        # 구간마다 다른 놈이 이기면 접근에서 한 번, 회피 중에 또 한 번,
        # 복귀에서 다시 속도가 바뀐다. 그 오르내림 자체가 라인 이탈을 키우고,
        # 특히 거리 기반(static/dynamic) 항은 장애물이 가까워질수록 계속
        # 낮아져서 정작 피하는 순간에 제일 느리다.
        #
        # 그래서 마지막에 덮어쓴다. 접근·회피·복귀가 같은 속도가 되고,
        # 감속은 `_slew_limit_speed` 가 a_brake 로 완만하게 만든다.
        # 진짜로 못 서는 상황은 AEB 몫이다 — 급감속은 거기서만 난다.
        #
        # TRAILING 은 앞차 속도를 따라야 하고, AEB 탈출은 기어 나가는
        # 동작이라 둘 다 예외다.
        cruise = self._avoid_cruise_target()
        if cruise > 0.0 and not trailing and not self._aeb_escape_active():
            v_target, reason = cruise, "avoid_cruise"

        # 이건 순항속도보다 뒤에 온다 — 덮어쓰기가 아니라 안전 상한이라서다.
        # 경로가 벽에서 잘렸으면 그 끝 앞에서 설 수 있는 속도가 진짜 상한이고,
        # 회피 순항속도라도 그걸 넘을 수는 없다.
        v_clear = self._cleared_path_speed_limit()
        if v_clear < v_target:
            v_target, reason = v_clear, "path_clear"

        v_target = self._slew_limit_speed(v_target, ceiling=v_csv)
        self._last_avoid_speed = v_target
        self._last_avoid_reason = reason
        return min(1.0, v_target / max(0.05, v_csv))

    def _slew_limit_speed(self, v_target: float, ceiling: float | None = None) -> float:
        """감속 명령이 차가 낼 수 있는 감속도를 넘지 않게 완만화.

        장애물이 검출 범위에 처음 들어오는 순간 목표속도가 뚝 떨어지는데,
        그대로 내보내면 못 따라가는 명령이라 속도 PI 가 포화되고 적분이
        쌓인다. a_brake 로 기울기를 제한하면 명령 자체가 추종 가능해진다.
        가속 방향은 제한하지 않는다 (위험이 사라지면 바로 회복).

        ceiling 은 이력의 상한이다. 장애물이 없을 때 목표속도는 정책 상한
        (8 m/s) 에 머무는데, 실제로 나가는 명령은 CSV 속도(~3 m/s) 로
        잘린다. 이력을 8 로 들고 있으면 위협이 나타났을 때 3 까지 내려오는
        1.6초 동안 배율이 1.0 에 붙어 있어 감속이 그만큼 늦는다. 의미 없는
        여유분을 잘라 두면 첫 프레임부터 제대로 된 기울기로 내려간다.
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
        if ceiling is not None and math.isfinite(ceiling):
            prev = min(prev, float(ceiling))

        floor = prev - self.avoid_speed_params.a_brake * dt
        v = max(v_target, floor)
        if self.avoid_a_accel_mps2 > 0.0:
            # 회복 방향도 묶는다. 안 묶으면 장애물이 시야에서 빠지는 순간
            # 목표속도가 한 프레임에 정책 상한까지 뛴다 (실측: 배율 0.17 →
            # 1.00, 0.46 → 3.39 m/s, 6 m/s^2). 그 급가속이 재합류 조향과
            # 겹치면 타이어 예산을 앞뒤로 다 써 버려 라인 밖으로 밀린다.
            v = min(v, prev + self.avoid_a_accel_mps2 * dt)
        self._slew_prev_v = v
        return v

    def _planner_gate_closest_m(self, filtered: list) -> float:
        """게이트 통과 장애 — 전방 콘 없이(조향 후에도 '아직 있음' 판정용)."""
        return closest_obstacle_surface_m(
            filtered,
            forward_cone_rad=None,
            min_forward_x_m=self.avoid_min_forward_x_m,
            lateral_abs_max_m=self._gated_lateral_abs_max_m,
            laser_to_base_x_m=self.laser_to_base_x_m,
        )

    def _planner_closest_obstacle_m(self, filtered: list) -> float:
        # 방향 판정은 전방 콘이 한다. 여기에 좁은 |y| 튜브를 겹쳐 걸면
        # 코리도를 통과한(=레이스라인 위) 곡선 구간 장애물이 다시 잘린다.
        return closest_obstacle_surface_m(
            filtered,
            forward_cone_rad=self.forward_cone_rad,
            min_forward_x_m=self.avoid_min_forward_x_m,
            lateral_abs_max_m=max(
                self.avoid_trigger_lateral_abs_max_m,
                self._gated_lateral_abs_max_m,
            ),
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
        sc = min(sc, self._planner_speed_scale())

        self.pub_planner_speed_scale.publish(Float64(data=sc))
        self.pub_planner_speed_condition.publish(UInt8(data=cd))
        self.pub_speed_reason.publish(String(data=self._last_avoid_reason or "none"))

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

    def _set_path_planned(self, planned: bool) -> None:
        """/local_path 가 계획 기하 경로인지 알린다 (Stanley 의 FF 스위치)."""
        self._path_planned = bool(planned)
        self.pub_path_planned.publish(Bool(data=self._path_planned))

    def _publish_override_gate(self, active: bool) -> None:
        # 속도 정책이 "실제로 회피 경로를 주고 있는지" 를 봐야 해서 기억해 둔다.
        # rejoin 이 꺼져 있으면 장애물이 사라진 뒤 mode 는 바로 GLOBAL 이 된다.
        self._override_active = bool(active)
        if not active:
            # 경로를 안 주면 Stanley 는 CSV 를 탄다 — 거기선 FF 가 원래 켜진다.
            self._set_path_planned(False)
            # 잘린 경로를 더 안 쓰므로 그 길이로 건 속도 상한도 같이 푼다.
            # 안 풀면 회피가 끝난 뒤에도 옛 상한이 남아 CSV 속도를 막는다.
            self._cleared_len_m = float("inf")
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
        self._build_curvature()

    # 곡률 베이스라인 [m]. 폴리라인 간격이 0.05 m 라 인접 3점으로 재면
    # 노이즈만 잡힌다 — 실측에서 ±0.2 m 는 R=1.26 m, ±2 m 는 R=2.42 m 로
    # 수렴하지 않는다. 차가 실제로 겪는 규모(차체 길이)로 고정한다.
    _CURV_BASELINE_M = 1.0

    def _build_curvature(self) -> None:
        """기준선 곡률 κ(s) 와 그 변화율 κ′(s) 를 미리 깔아 둔다 (좌회전 +).

        재합류 경로는 기준선에서 `d` 만큼 떨어진 오프셋 곡선이고, 그 곡률에는
        기준선 곡률이 **`1/(1-d·κ)` 로 증폭돼** 들어간다. 코너 안쪽으로 벗어나
        있으면(`d·κ > 0`) 분모가 작아져 같은 복귀 경로라도 실제 곡률이 몇 배가
        된다. `d·κ → 1` 이면 오프셋 좌표계 자체가 접힌다 — 서로 다른 s 의 법선이
        곡률 중심에서 만나 한 점으로 뭉개진다.

        직선 가정(`5.77·|d0|/L²`)만으로 길이·속도를 정하면 이걸 통째로 놓친다.
        실측: 이탈 1.5 m 로 R≈2 m 코너 안쪽에 있으면 σ=0.38 이라 6 m/s 요구
        횡가속도가 **예산의 17 배** 인데, 직선 가정은 예산 안이라고 답한다.
        """
        n = self._n
        if n < 5 or self._total_l < 1e-6:
            self._kappa_np = np.zeros(max(n, 1), dtype=np.float64)
            self._kappa_d_np = np.zeros(max(n, 1), dtype=np.float64)
            return
        step = self._total_l / n
        half = max(1, int(round(0.5 * self._CURV_BASELINE_M / max(step, 1e-6))))
        # 접선각을 베이스라인의 절반으로 잡고, 그 각을 다시 절반만큼 떨어뜨려
        # 차분한다 — 합쳐서 _CURV_BASELINE_M 규모가 된다.
        th = np.arctan2(
            np.roll(self._ys_np, -half) - np.roll(self._ys_np, half),
            np.roll(self._xs_np, -half) - np.roll(self._xs_np, half),
        )
        dth = np.roll(th, -half) - np.roll(th, half)
        dth = (dth + np.pi) % (2.0 * np.pi) - np.pi
        self._kappa_np = dth / (2.0 * half * step)
        dk = np.roll(self._kappa_np, -half) - np.roll(self._kappa_np, half)
        self._kappa_d_np = dk / (2.0 * half * step)

    def _kappa_at_s(self, s: float) -> Tuple[float, float]:
        """기준선의 (κ, κ′) at s."""
        if self._total_l < 1e-6:
            return 0.0, 0.0
        i = int((s % self._total_l) / self._total_l * self._n) % self._n
        return float(self._kappa_np[i]), float(self._kappa_d_np[i])

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
        # d0p 는 유효 한계까지만 자른다. 예전엔 ±1 (45°) 로 잘랐는데 그 값에
        # 근거가 없었고, 무엇보다 **잘렸다는 사실이 호출부에 안 보였다**.
        # 75° 로 벽을 향한 차를 45° 로 알려주면 플래너는 따라갈 수 없는 경로를
        # 자신 있게 낸다. 자르는 건 tan 의 발산을 막기 위해 여전히 필요하지만,
        # 한계를 넘었는지는 `yaw_err` 로 직접 보고 판단해야 한다 — 그래서
        # 원본 각도 그대로 같이 돌려준다.
        lim = math.tan(getattr(self, "_rejoin_yaw_err_limit", math.radians(55.0)))
        d0p = max(-lim, min(lim, math.tan(yaw_err)))
        d0pp = 0.0
        return s0, d0, d0p, d0pp, yaw_ref, yaw_err

    # ------------------------------------------------------------------
    # Frenet 공용 유틸 — REJOIN 전용이던 투영을 플래너 전역에서 쓴다.
    # ------------------------------------------------------------------
    def _delta_s(self, s_a: float, s_b: float) -> float:
        """랩을 감안한 s_a − s_b. [-L/2, +L/2) 로 정규화.

        +면 a 가 b 보다 진행방향 앞이다. 폐곡선이라 그냥 빼면 결승선 부근에서
        한 바퀴만큼 튄다.
        """
        total = self._total_l
        if total <= 1e-6:
            return 0.0
        d = (float(s_a) - float(s_b)) % total
        if d >= 0.5 * total:
            d -= total
        return d

    def _frenet_xy(self, mx: float, my: float) -> Tuple[float, float]:
        """맵 점 → (s, d). d 는 진행방향 왼쪽이 +."""
        qx, qy, seg_i, t = self._closest_on_loop(mx, my)
        s = self._seg_start[seg_i] + t * self._seg_len[seg_i]
        j = (seg_i + 1) % self._n
        yaw_ref = math.atan2(self._ys[j] - self._ys[seg_i], self._xs[j] - self._xs[seg_i])
        d = (mx - qx) * (-math.sin(yaw_ref)) + (my - qy) * math.cos(yaw_ref)
        return s, d

    def _track_yaw_at_s(self, s: float) -> float:
        return self._xy_yaw_at_s(s)[2]

    def _update_frenet_snapshot(
        self, current: PoseStamped | None, filtered: list, filtered_dynamic: list, tf_lm
    ) -> None:
        """자차/장애물의 (s, d) 를 매 주기 갱신. 판정은 바꾸지 않고 정보만 만든다."""
        self._s_ego = None
        self._d_ego = None
        self._static_sd = []
        self._dynamic_sd = []

        if current is not None:
            self._s_ego, self._d_ego = self._frenet_xy(
                float(current.pose.position.x), float(current.pose.position.y)
            )

        if tf_lm is None:
            return
        to_map = self._make_laser_to_map_fn(tf_lm)
        q = tf_lm.transform.rotation
        yaw_lm = _quat_to_yaw(q)
        cos_l, sin_l = math.cos(yaw_lm), math.sin(yaw_lm)

        for k in range(0, max(0, len(filtered) - 3), 4):
            mx, my = to_map(float(filtered[k + 1]), float(filtered[k + 2]))
            s, d = self._frenet_xy(mx, my)
            self._static_sd.append((s, d, float(filtered[k + 3])))

        # 동적: vx,vy 는 laser frame 상대속도다. 절대속도 ≈ 상대 + 자차속도.
        # 자차 요레이트로 생기는 항은 무시한다 (1초 예측이라 영향이 작다).
        ego_yaw = (
            _quat_to_yaw(current.pose.orientation) if current is not None else yaw_lm
        )
        ego_vx = self._ego_speed_mps * math.cos(ego_yaw)
        ego_vy = self._ego_speed_mps * math.sin(ego_yaw)
        for k in range(0, max(0, len(filtered_dynamic) - 5), 6):
            lx = float(filtered_dynamic[k + 1])
            ly = float(filtered_dynamic[k + 2])
            vlx = float(filtered_dynamic[k + 3])
            vly = float(filtered_dynamic[k + 4])
            r = float(filtered_dynamic[k + 5])
            mx, my = to_map(lx, ly)
            s, d = self._frenet_xy(mx, my)
            vmx = cos_l * vlx - sin_l * vly + ego_vx
            vmy = sin_l * vlx + cos_l * vly + ego_vy
            tyaw = self._track_yaw_at_s(s)
            vs = vmx * math.cos(tyaw) + vmy * math.sin(tyaw)
            rng = math.hypot(lx, ly)
            closing = -(lx * vlx + ly * vly) / rng if rng > 1e-3 else 0.0
            self._dynamic_sd.append((s, d, r, vs, closing))

    def _obstacle_s_for_gap(self, entry) -> float:
        """장애물의 s. use_predicted_s 면 등속 예측을 적용한 s."""
        s = float(entry[0])
        if not self._use_predicted_s or len(entry) < 4:
            return s
        return s + float(entry[3]) * self._pred_horizon_sec

    def _forward_leader(self) -> tuple[float, float]:
        """전방 최근접 동적 장애물의 (s 갭 [m], 트랙방향 절대속도 [m/s]).

        없으면 (inf, 0.0). 표면 기준으로 반경과 앞범퍼 여유를 뺀다
        (XY 게이트와 같은 규약).

        속도를 같이 돌려주는 이유: 추종 속도는 CSV 속도가 아니라 **앞차
        속도** 를 기준으로 잡아야 하기 때문이다. 자세한 건
        `_trailing_target_speed` 주석 참고.
        """
        if self._s_ego is None or not self._dynamic_sd:
            return float("inf"), 0.0
        best = float("inf")
        best_vs = 0.0
        for entry in self._dynamic_sd:
            # 전방 여부는 "지금" 기준으로 판정한다. 예측 s 로 걸러 버리면
            # 마주 오는 물체가 자차를 지나친 것으로 계산되어 갭 계산에서
            # 통째로 사라진다 — 가장 위험한 대상이 없어지는 셈이다.
            if self._delta_s(float(entry[0]), self._s_ego) <= 0.0:
                continue
            ds = self._delta_s(self._obstacle_s_for_gap(entry), self._s_ego)
            gap = ds - float(entry[2]) - self.ego_front_safety_m
            gap = max(0.0, gap)
            if gap < best:
                best = gap
                best_vs = float(entry[3])
        return best, best_vs

    def _forward_gap_s_m(self) -> float:
        return self._forward_leader()[0]

    def _has_followable_leader(self) -> bool:
        """전방 동적 장애물이 '따라갈 수 있는 앞차' 인가.

        추월 로직이 아직 없으므로, 같은 방향으로 달리는 차는 비켜 가려
        하지 말고 뒤에 붙어야 한다. 반대로 아래 둘은 따라갈 대상이 아니다.

          - 역주행(vs < 0): 마주 오는 차 뒤에 붙는다는 말은 성립하지 않는다.
          - 사실상 정지(|vs| 가 임계 미만): 서 있는 차를 따라가면 영원히
            그 뒤에 서 있게 된다. 정지한 순간 '정적 장애물' 로 넘겨서
            회피가 돌아가게 해야 한다.

        둘 다 AVOID 로 보낸다.

        trailing_speed_deficit_enable 을 켜면 여기에 상대속도 조건이 하나
        더 붙는다 — 아래 _leader_too_slow 참고.
        """
        gap, vs = self._forward_leader()
        if not math.isfinite(gap):
            return False
        if vs < self.trailing_min_leader_speed_mps:
            return False
        return not self._leader_too_slow(vs)

    def _csv_speed_now(self) -> float:
        """현재 위치의 CSV 목표속도 [m/s]. 포즈가 없으면 기준속도."""
        current = self._last_pose_for_speed
        if current is None:
            return self.avoid_speed_ref_mps
        return self._csv_speed_near(
            float(current.pose.position.x), float(current.pose.position.y)
        )

    def _leader_too_slow(self, vs: float) -> bool:
        """우리보다 한참 느린 앞차인가 (→ 따라가지 말고 비켜 간다).

        절대속도만 보면 1 m/s 로 기어가는 차 뒤에 5 m/s 를 낼 수 있는 우리가
        붙어서 1 m/s 로 간다. 레이싱에서 그건 지는 것이다. CSV 목표속도와의
        차이가 이 임계를 넘으면 '따라갈 만한 앞차' 로 보지 않고 AVOID 로
        넘겨서, 정적 장애물과 똑같이 FGM 반응형 회피로 지나가게 한다.

        새 상태나 추월 판단 로직은 넣지 않는다 — 분류 기준만 하나 늘린 것이다.

        기본 OFF 인 이유: 움직이는 차를 옆으로 지나는 건 콘을 지나는 것과
        다르다. 상대는 우리가 옆에 붙은 순간 라인을 바꿀 수 있고 반응형
        회피는 그걸 예측하지 못한다. 임계를 낮게 잡으면 접촉 위험이 실제로
        올라간다.
        """
        if not self.trailing_speed_deficit_enable:
            return False
        deficit = self._csv_speed_now() - float(vs)
        return deficit > self.trailing_max_speed_deficit_mps

    def _update_leader_latch(self) -> bool:
        """따라갈 앞차가 있는가 — 프레임 단위 흔들림을 걸러낸 값.

        클러스터 추적(integrated_obstacle_node)은 같은 물체를 한두 프레임씩
        static/dynamic 으로 오간다. speed_threshold_mps=0.45 경계에서 속도
        추정 노이즈가 그대로 분류를 뒤집기 때문이다. 그 값을 매 프레임
        그대로 쓰면 avoid_on 이 같이 뒤집혀 TRAILING↔AVOID↔GLOBAL 을 오가고,
        모드가 바뀔 때마다 속도 정책이 통째로 바뀌어 명령속도가 1.2 →
        3.9 m/s 로 튄다. 실제 파형이 그랬다 — 갭 제어는 멀쩡한데 그 위에서
        모드가 떨려서 버벅였다.

        그래서 판정을 양방향 히스테리시스로 굳힌다.
          진입: enter_th 프레임 연속 — 정적 콘이 노이즈로 한 프레임 dynamic
                으로 튀었다고 "앞차" 로 붙잡지 않게.
          해제: lost_th 프레임 연속 — 실제 앞차를 한 프레임 놓쳤다고
                회피로 튕겨 나가지 않게.
        """
        if self._has_followable_leader():
            self._leader_lost_count = 0
            if not self._leader_latched:
                self._leader_seen_count += 1
                if self._leader_seen_count >= self.leader_enter_count_th:
                    self._leader_latched = True
        else:
            self._leader_seen_count = 0
            if self._leader_latched:
                self._leader_lost_count += 1
                if self._leader_lost_count >= self.leader_lost_count_th:
                    self._leader_latched = False
        return self._leader_latched

    def _publish_frenet_debug(self) -> None:
        if self.pub_frenet_debug is None:
            return
        data = [
            float(self._s_ego if self._s_ego is not None else float("nan")),
            float(self._d_ego if self._d_ego is not None else float("nan")),
        ]
        for s, d, _r in self._static_sd:
            data.extend([float(s), float(d)])
        for s, d, _r, _vs, _c in self._dynamic_sd:
            data.extend([float(s), float(d)])
        self.pub_frenet_debug.publish(Float32MultiArray(data=data))

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

    @staticmethod
    def _eval_quintic_d1(coeff: Tuple[float, ...], ds: float) -> float:
        _, a1, a2, a3, a4, a5 = coeff
        return a1 + 2 * a2 * ds + 3 * a3 * ds**2 + 4 * a4 * ds**3 + 5 * a5 * ds**4

    @staticmethod
    def _eval_quintic_d2(coeff: Tuple[float, ...], ds: float) -> float:
        _, _, a2, a3, a4, a5 = coeff
        return 2 * a2 + 6 * a3 * ds + 12 * a4 * ds**2 + 20 * a5 * ds**3

    def _append_pose(self, path: Path, x: float, y: float) -> None:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)

    # quintic d(s)=d0*(1-10u^3+15u^4-6u^5) 의 미분 최대값 계수.
    #   |d'|  최대: u=0.5 에서 1.875*|d0|/L        (경로가 라인과 이루는 각)
    #   |d''| 최대: u=(1±1/sqrt(3))/2 에서 5.7735*|d0|/L^2
    _QUINTIC_D1_PEAK = 1.875
    _QUINTIC_D2_PEAK = 5.7735

    # σ = 1-d·κ 가 이 아래면 Frenet 오프셋 좌표계가 접히기 직전이라 곡률이
    # 발산한다. 그런 경로는 "급한 경로" 가 아니라 **좌표계가 깨진 경로** 라
    # 속도를 낮춰서 해결되지 않는다. 실측 최소반경 1.85 m 기준 σ=0.25 는
    # 안쪽 이탈 1.4 m 근처다.
    _REJOIN_SIGMA_FLOOR = 0.25

    def _rejoin_heading_limit_rad(self, v: float) -> float:
        """이 속도에서 허용할 합류각 [rad]. 빠를수록 좁힌다.

        합류각 ψ 로 라인에 붙으면 라인을 가로지르는 속도성분이 `v·sin(ψ)` 다.
        추종에 지연 τ 가 있으면 그만큼 **라인을 넘어간다**: 오버슈트 ≈
        `v·sin(ψ)·τ`. 즉 같은 각이라도 빠를수록 더 많이 넘어간다. 각이 아니라
        **넘어가는 양** 을 예산으로 잡아야 속도에 대해 일관된다.

            sin(ψ) ≤ rejoin_merge_overshoot_m / (v · rejoin_track_lag_s)

        레이스라인이 벽에 붙어 있어서 넘어간 만큼이 곧 벽까지의 여유를 먹는다.
        위아래는 묶어 둔다 — 저속에서 무한정 벌리면 예전 33° 로 돌아가고,
        고속에서 무한정 좁히면 트랙 둘레(41 m)로는 못 만드는 길이를 요구한다.
        """
        v = max(0.1, abs(v))
        sin_max = self.rejoin_merge_overshoot_m / (v * self.rejoin_track_lag_s)
        sin_max = min(self._rejoin_heading_sin_hi, max(self._rejoin_heading_sin_lo, sin_max))
        return math.asin(min(1.0, sin_max))

    def _rejoin_line_crossing_m(
        self, coeff: Tuple[float, ...], length: float, d0: float, n: int = 40
    ) -> float:
        """이 경로가 레이스라인을 **가로질러 반대편으로** 나가는 최대 거리 [m].

        복귀는 라인에 붙는 기동이지 라인을 넘는 기동이 아니다. 그런데 차가
        이미 라인을 향해 비스듬히 달리는 중이면(회피 직후가 늘 그렇다)
        quintic 은 그 헤딩을 그대로 물려받아 출발하므로, 길이가 길면 초기
        기울기가 라인을 지나쳐 반대편까지 밀고 간 뒤 되돌아온다.

        **레이스라인은 벽에 붙어 있다.** 넘어간 만큼이 그대로 반대쪽 벽
        여유를 먹으므로, 이건 "조금 지나침" 이 아니라 충돌 경로다.

        정규화하면 넘어가는 양은 `|d0|·F(p)`, `p = d0p·L/d0` 로 **p 만의**
        함수다. 즉 같은 이탈량이라도 헤딩이 서 있을수록, 그리고 길이가
        길수록 더 넘어간다 — 길게 잡으면 완만해진다는 직관이 여기서는
        거꾸로다. 그래서 재는 수밖에 없다.
        """
        if abs(d0) < 1e-6:
            return 0.0
        sgn = 1.0 if d0 > 0.0 else -1.0
        worst = 0.0
        for k in range(n):
            ds = length * k / max(n - 1, 1)
            worst = min(worst, sgn * self._eval_quintic(coeff, ds))
        return -worst

    def _rejoin_path_curvature(
        self, s0: float, coeff: Tuple[float, ...], length: float, n: int = 40
    ) -> Tuple[float, float]:
        """이 재합류 경로의 (최대 |초과곡률|, 최소 σ).

        Frenet→직교 변환의 곡률식 (σ=1-d·κ, Δθ=atan2(d′, σ)):

            κ_path = ((d″ + (κ′d + κd′)·tanΔθ)·cos²Δθ/σ + κ)·cosΔθ/σ

        돌려주는 건 `κ_path` 가 아니라 **`κ_path − κ`** 다. 기준선 곡률만큼은
        레이스라인을 그냥 달려도 어차피 감당해야 하고 CSV 속도 프로파일이 이미
        그걸로 짜여 있다. `rejoin_a_lat_mps2`(4.0) 는 복귀 기동이 **추가로**
        만드는 몫의 예산이므로 총량과 비교하면 안 된다 — 총량으로 재면 중간
        속도 코너(κ≈0.16)만 지나도 v²κ=5.8 이라 직선 복귀조차 3.8 m/s 로
        묶인다. 직선(κ=0)에서는 이 값이 예전 `5.77·|d0|/L²` 로 그대로 환원된다.

        σ 가 바닥을 치면 경로 자체가 무효라 무한대를 돌려준다 — 호출부가
        감속이 아니라 **포기** 로 처리해야 하는 경우다.
        """
        excess_max = 0.0
        sigma_min = 1.0
        for k in range(n):
            ds = length * k / max(n - 1, 1)
            d = self._eval_quintic(coeff, ds)
            dp = self._eval_quintic_d1(coeff, ds)
            dpp = self._eval_quintic_d2(coeff, ds)
            kap, kapd = self._kappa_at_s(s0 + ds)
            sigma = 1.0 - d * kap
            sigma_min = min(sigma_min, sigma)
            if sigma <= self._REJOIN_SIGMA_FLOOR:
                return float("inf"), sigma_min
            dth = math.atan2(dp, sigma)
            c = math.cos(dth)
            kappa_path = (
                (dpp + (kapd * d + kap * dp) * math.tan(dth)) * c * c / sigma + kap
            ) * (c / sigma)
            # 총곡률이 조향 한계를 넘으면 속도와 무관하게 못 따라간다.
            # 초과분만 보면 이게 안 걸린다 — 기준선이 이미 급한 코너에서
            # 조금만 더 휘어도 핸들이 끝까지 돌아간 상태가 되기 때문이다.
            if abs(kappa_path) > self.rejoin_max_path_curvature:
                return float("inf"), sigma_min
            excess_max = max(excess_max, abs(kappa_path - kap))
        return excess_max, sigma_min

    def _plan_rejoin(
        self, s0: float, d0: float, d0p: float, d0pp: float, v: float
    ) -> Tuple[float, Tuple[float, ...], float, float]:
        """(길이, quintic 계수, 최대곡률, 최소σ). 실제 코너 형상을 보고 고른다.

        직선 가정으로 뽑은 길이를 출발점 삼되, 그게 코너에서 맞는다는 보장이
        없으므로 후보를 훑는다. 코너 **안쪽** 복귀에서는 길이를 늘려도 σ 가
        나아지지 않는다(σ 는 시작점 이탈이 지배한다) — 그래서 "무조건 길게" 가
        답이 아니고, 요구 곡률이 가장 낮은 길이를 실제로 찾아야 한다.

        기동당 한 번만 도는 계산이라 후보 탐색 비용은 문제되지 않는다.
        """
        lo = self.rejoin_min_length_m
        hi = self.rejoin_max_length_m
        base = self._rejoin_length_for(d0, v, d0p)
        budget = self.rejoin_a_lat_mps2 / max(v * v, 1e-3)  # 예산을 만족하는 κ

        best = None  # (kappa_max, length, coeff, sigma_min)
        for k in range(self._REJOIN_LENGTH_CANDIDATES):
            frac = k / (self._REJOIN_LENGTH_CANDIDATES - 1)
            length = lo + (hi - lo) * frac
            coeff = self._solve_quintic(d0, d0p, d0pp, 0.0, 0.0, 0.0, length)
            kappa, sigma = self._rejoin_path_curvature(s0, coeff, length)
            if not math.isfinite(kappa):
                continue
            cross = self._rejoin_line_crossing_m(coeff, length, d0)
            # 순위는 위험한 순서다. 라인을 넘는 경로는 벽으로 가는 경로라
            # 곡률이 아무리 얌전해도 뒤로 보낸다. 그 다음이 못 따라가는
            # 경로, 마지막이 "둘 다 통과" — 거기서는 직선 가정값에 가장
            # 가까운 걸 쓴다. 굳이 더 길게 갈 이유가 없다.
            if cross > self.rejoin_merge_overshoot_m:
                rank = (2, cross)
            elif kappa > budget:
                rank = (1, kappa)
            else:
                rank = (0, abs(length - base))
            if best is None or rank < best[0]:
                best = (rank, length, coeff, kappa, sigma)
        if best is None:
            coeff = self._solve_quintic(d0, d0p, d0pp, 0.0, 0.0, 0.0, base)
            _, sigma = self._rejoin_path_curvature(s0, coeff, base)
            return base, coeff, float("inf"), sigma
        if best[0][0] >= 2:
            # 남은 게 '라인을 넘는 경로' 뿐이다. 급한 코너에 비스듬히 선
            # 채로 들어오면 이렇게 된다 — 짧게 잡으면 조향 한계를 넘고,
            # 길게 잡으면 라인 너머로 밀린다. 사이에 답이 없다.
            #
            # 이럴 땐 만들지 않는 게 맞다. CSV 를 유지하면 Stanley 가
            # 접지력 예산 안에서 알아서 붙는다. 벽을 향하는 경로를 쥐여
            # 주는 것보다 낫다 — 호출부가 무한대를 보고 포기한다.
            return best[1], best[2], float("inf"), best[4]
        return best[1], best[2], best[3], best[4]

    _REJOIN_LENGTH_CANDIDATES = 16

    def _rejoin_length_for(self, d0: float, v: float, d0p: float = 0.0) -> float:
        """재합류 길이. 세 제약 중 '가장 긴 쪽'을 쓴다.

        1. 시간 연동 `rejoin_time_sec * v` — 최소한 이만큼은 걸려야 한다
        2. 횡가속 예산 — 요구 횡가속도가 `v^2*5.77*|d0|/L^2` 이므로
           `L >= sqrt(5.77*|d0|*v^2 / a_lat)`
        3. **헤딩 편차** — 경로가 라인과 이루는 최대 각은 `atan(1.875*|d0|/L)`
           이므로 `L >= 1.875*|d0| / tan(ψ)`

        여기까지는 **기준선이 직선이라고 가정** 한 값이다. 코너에서는 실제
        곡률이 다르므로 `_plan_rejoin` 이 이 값을 출발점으로 다시 고른다.

        3번이 없으면 짧고 급한 경로가 나온다. 실측 조건(이탈 1.2 m, 2.6 m/s)
        에서 1·2번만으로는 L=3.4 m 가 나오는데, 그 경로는 중간에서 라인과
        **33도** 를 이룬다. 횡가속도로는 통과지만 차는 라인을 향해 비스듬히
        꽂히듯 들어가고, 레이스라인이 벽에 붙어 있으면 그대로 벽을 향한다.

        ### ψ 는 허용각이 아니라 "이미 서 있는 각" 과의 큰 쪽이다

        3번의 `atan(1.875*|d0|/L)` 은 **차가 라인과 나란할 때**(`d0p=0`) 의
        식이다. 회피 직후는 그 반대다 — 이미 라인을 향해 비스듬히 달리고
        있고, 경로는 그 헤딩에서 출발할 수밖에 없다(C1 연속). 그 상태에서
        허용각만 보고 L 을 늘리면 초기 기울기가 라인을 지나쳐 **반대편으로
        넘어갔다가** 되돌아오는 경로가 된다. 넘어가는 양은 `p = d0p·L/|d0|`
        가 지배해서 L 에 **비례해 커진다**. 실측 조건(이탈 1.0 m, 헤딩 30°,
        3 m/s)에서 허용각 12.8° 로 잡은 L=8.2 m 는 라인을 0.25 m 넘어간다 —
        벽에 붙은 라인에서는 그게 곧 접촉이다.

        그래서 이미 선 각이 허용각보다 크면 **그 각을 목표로 삼는다.**
        `L = 1.875*|d0|/tan(ψ_now)` 는 `p ≈ −1.875` 라 넘어가지 않으면서
        헤딩이 자연스럽게 눕는 길이다. 같은 조건이 L=3.3 m / 넘어감 0 이
        된다. 라인에서 멀어지는 중이면(`d0·d0p ≥ 0`) 경로가 각을 먼저
        되돌려야 하므로 예전대로 허용각을 쓴다.

        헤딩 제약이 L 을 늘리는 쪽으로 작동할 때, 그건 부작용이 아니라
        이득이다. L 이 커지면 요구 횡가속도(1/L^2)가 급감하고
        `_deviation_speed_limit`(L 에 비례)이 느슨해져서 **더 빠르게**
        복귀한다. 느리고 길게 도는 게 아니라 완만하고 빠르게 붙는다.
        """
        v = abs(v)
        d = abs(d0)
        l_time = self.rejoin_time_sec * v
        l_accel = math.sqrt(self._QUINTIC_D2_PEAK * d * v * v / self.rejoin_a_lat_mps2)
        psi = self._rejoin_heading_limit_rad(v)
        toward = d0 * d0p < 0.0  # 라인을 향해 달리는 중 = 회피 직후
        if toward:
            psi = max(psi, abs(math.atan(d0p)))
        l_heading = self._QUINTIC_D1_PEAK * d / math.tan(psi)
        length = max(self.rejoin_min_length_m, l_time, l_accel, l_heading)
        if toward:
            # 위 셋은 전부 **하한** 이라 제일 긴 놈이 이긴다. 그런데 라인을
            # 향해 달리는 중에는 길이가 곧 넘어가는 양이라, 하한만 쌓으면
            # 벽으로 간다 (0.6 m / 30° / 6 m/s 에서 l_accel 이 5.6 m 를
            # 요구하고 그 경로는 0.21 m 넘어간다).
            #
            # 넘어감이 이긴다. 횡가속 예산을 못 지키는 건 속도로 갚을 수
            # 있지만(`_deviation_speed_limit`), 벽은 못 갚는다.
            length = min(length, self._rejoin_crossing_cap_m(d0, d0p))
        return min(self.rejoin_max_length_m, max(self.rejoin_min_length_m, length))

    def _rejoin_crossing_cap_m(self, d0: float, d0p: float) -> float:
        """라인을 예산 이상 넘지 않는 **최대** 길이 [m].

        넘어가는 양은 길이에 대해 단조증가라(0 에서 시작해 계속 커진다)
        이분법이 성립한다. 닫힌 형태로 풀 수도 있지만 역함수가 지저분해서,
        기동당 한 번 도는 계산에 12 번 이분하는 편이 읽기 쉽다.

        예산의 95 % 를 목표로 잡는다. 이분법은 한계선에 정확히 올라앉는데,
        넘어감 측정 자체가 40 점 표본이라 실제값이 0.4 mm 쯤 더 클 수 있다.
        여유를 두면 여기서 나온 길이는 `_plan_rejoin` 의 게이트도 항상
        통과한다 — 계획이 자기 검사에 걸리는 일이 없다.
        """
        budget = 0.95 * self.rejoin_merge_overshoot_m
        lo = self.rejoin_min_length_m
        hi = self.rejoin_max_length_m

        def crosses(length: float) -> bool:
            coeff = self._solve_quintic(d0, d0p, 0.0, 0.0, 0.0, 0.0, length)
            return self._rejoin_line_crossing_m(coeff, length, d0) > budget

        if not crosses(hi):
            return hi
        if crosses(lo):
            return lo  # 최소 길이로도 넘는다 — 더 짧게 갈 수는 없다
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            if crosses(mid):
                hi = mid
            else:
                lo = mid
        return lo

    def _build_alignment_path(self, s0: float, d0: float, yaw_err: float) -> Path | None:
        """지금 이탈량을 **유지한 채** 트랙 방향으로 나란히 가는 경로.

        헤딩이 라인과 수직에 가까울 때 쓴다. 그 상태에서 "라인으로 돌아와라"
        는 경로는 두 가지를 동시에 시키는 셈인데 — 방향을 돌리는 것과 옆으로
        붙는 것 — 차는 앞의 것부터 할 수밖에 없다. 그런데 경로는 뒤의 것을
        기준으로 그려져 있으니 추종 오차가 벌어지고, 벌어진 방향이 하필 차가
        향하던 벽 쪽이다. 실차가 여기서 박았다.

        그래서 옆으로 붙는 요구를 뺀다. `d = d0` 고정이면 Stanley 의 CTE 항이
        0 근처라 헤딩 항만 남고, 그게 곧 정렬이다. 정렬이 되면(`yaw_err` 가
        한계 밑으로) `_refresh_rejoin_path` 가 이 경로를 버리고 정식 복귀를
        다시 그린다. 이탈은 그때 줄인다.

        CSV 로 넘기지 않는 이유는 `_publish_rejoin_bridge` 주석 그대로다 —
        override 가 내려가는 순간 기준경로가 튀면서 CTE 가 계단으로 뛴다.
        여기서는 override 를 쥔 채로 방향만 맞춘다.
        """
        v = abs(self._ego_speed_mps)
        length = min(
            self.rejoin_max_length_m,
            max(self.rejoin_min_length_m, 2.0, v * self.rejoin_time_sec * 2.0),
        )
        step = max(0.05, min(0.1, self._total_l / max(self._n, 1)))

        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()
        for k in range(int(length / step) + 1):
            s = s0 + k * step
            x_ref, y_ref, yaw_ref = self._xy_yaw_at_s(s)
            self._append_pose(
                out, x_ref - d0 * math.sin(yaw_ref), y_ref + d0 * math.cos(yaw_ref)
            )
        if len(out.poses) < 2:
            return None

        out, usable = self._truncate_path_at_collision(
            out, self._lookup_laser_to_map_transform()
        )
        if not usable or len(out.poses) < 2:
            self._warn_rejoin_given_up(
                d0,
                math.tan(yaw_err),
                f"정렬 경로도 {self._last_path_cut}번째 점에서 막힘",
            )
            return None

        self._rejoin_is_alignment = True
        self._rejoin_length_m = length
        self._rejoin_kappa_max = 0.0
        self._rejoin_target_s = (s0 + length) % self._total_l
        self._rejoin_budget_ns = self.rejoin_max_active_ns
        self._warn_rejoin_aligning(d0, yaw_err)
        return out

    def _warn_rejoin_aligning(self, d0: float, yaw_err: float) -> None:
        """정렬 경로로 우회했다 (1초에 한 번)."""
        now = self.get_clock().now().nanoseconds
        if now - getattr(self, "_last_align_warn_ns", 0) < 1_000_000_000:
            return
        self._last_align_warn_ns = now
        self.get_logger().warn(
            f"REJOIN 대신 정렬 — 헤딩 {math.degrees(yaw_err):+.0f}° 가 한계 "
            f"{math.degrees(self._rejoin_yaw_err_limit):.0f}° 초과. 이탈 {d0:+.2f} m "
            f"유지한 채 방향부터 맞춘다 (v={abs(self._ego_speed_mps):.1f})"
        )

    def _build_frenet_quintic_rejoin_path(
        self, current_pose: PoseStamped
    ) -> Path | None:
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        yaw = _quat_to_yaw(current_pose.pose.orientation)

        s0, d0, d0p, d0pp, _, yaw_err = self._project_to_frenet(x, y, yaw)
        self._rejoin_kappa_max = 0.0

        # 라인과 너무 비스듬하면 복귀 이전에 정렬이다. quintic 은 이 상태를
        # 표현하지 못하고, 억지로 뽑으면 못 따라갈 경로가 나온다.
        if abs(yaw_err) > self._rejoin_yaw_err_limit:
            return self._build_alignment_path(s0, d0, yaw_err)
        self._rejoin_is_alignment = False

        L, coeff, kappa_max, sigma_min = self._plan_rejoin(
            s0, d0, d0p, d0pp, self._ego_speed_mps
        )
        if not math.isfinite(kappa_max):
            # 쓸 만한 복귀 경로가 없다. 둘 중 하나다.
            #
            # - Frenet 접힘: 코너 안쪽으로 곡률 중심만큼 들어와 있어서 이
            #   좌표계로는 경로를 정의할 수 없다 (σ_min 이 바닥).
            # - 넘김: 급한 코너에 비스듬히 서 있어 짧으면 조향 한계, 길면
            #   라인 너머로 밀린다.
            #
            # 어느 쪽이든 CSV 로 넘긴다. Stanley 의 피드백은 접지력 예산에
            # 묶여 있어서 깨진 경로나 벽으로 가는 경로를 주는 것보다 안전하다.
            why = (
                f"Frenet 접힘 (σ_min={sigma_min:.2f})"
                if sigma_min <= self._REJOIN_SIGMA_FLOOR
                else f"헤딩 {math.degrees(math.atan(d0p)):+.0f}° 로는 "
                f"라인을 넘지 않고 붙을 길이가 없음"
            )
            self._warn_rejoin_given_up(d0, d0p, why)
            return None
        self._rejoin_length_m = L
        self._rejoin_kappa_max = kappa_max

        # 시간 상한을 계획 길이에 맞춘다. 고정 5 초는 헤딩 제약으로 경로가
        # 길어지기 전 기준이라, 저속·큰이탈(1.5 m / 2.5 m/s → 13.2 m)에서는
        # 정상 복귀가 완료 직전에 끊긴다. 끊기면 override 가 내려가면서
        # 기준경로가 CSV 로 튀어 — 막으려던 급조향이 바로 그때 나온다.
        # 계획 통과시간의 2.5 배를 주되 설정값 밑으로는 안 내려간다.
        # '멈춤' 은 이 상한이 아니라 정지 감지가 따로 잡는다.
        # (저속·큰이탈 예: 1.5 m / 2.5 m/s → 10 m, 4.0 초)
        expect_s = L / max(abs(self._ego_speed_mps), 1.0)
        self._rejoin_budget_ns = max(self.rejoin_max_active_ns, int(2.5 * expect_s * 1e9))

        self._rejoin_target_s = (s0 + L) % self._total_l

        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()

        tail_step = self._total_l / max(self._n, 1)
        tail_step = max(0.05, min(0.1, tail_step))

        # 점 개수가 아니라 '간격' 을 고정한다. first_blocked_index 는 경로 점만
        # 검사하고 사이를 보간하지 않아서, 점 간격이 곧 충돌 검사 해상도다.
        # 개수를 고정하면 L 이 길어질수록 간격이 벌어져(16 m/30점 = 0.55 m)
        # 얇아진 벽 팽창대를 그대로 건너뛴다. 꼬리와 같은 간격으로 깐다.
        n_samples = max(self.rejoin_sample_count, int(L / tail_step) + 1)
        for k in range(n_samples):
            ds = L * k / max(n_samples - 1, 1)
            d = self._eval_quintic(coeff, ds)
            s = s0 + ds
            x_ref, y_ref, yaw_ref = self._xy_yaw_at_s(s)
            px = x_ref - d * math.sin(yaw_ref)
            py = y_ref + d * math.cos(yaw_ref)
            self._append_pose(out, px, py)

        for k in range(self.rejoin_tail_count):
            s_tail = s0 + L + k * tail_step
            x_ref, y_ref, _ = self._xy_yaw_at_s(s_tail)
            self._append_pose(out, x_ref, y_ref)

        if len(out.poses) < 2:
            return None

        # 재합류 경로도 벽/장애물 검사를 받아야 한다. 회피 직후라 옆으로
        # 나가 있는 상태이고, 그 상태에서 레이스라인으로 비스듬히 붙는
        # 경로는 안쪽 벽이나 아직 남은 장애물을 스칠 수 있다.
        out, usable = self._truncate_path_at_collision(
            out, self._lookup_laser_to_map_transform()
        )
        if not usable or len(out.poses) < 2:
            self._warn_rejoin_given_up(
                d0, d0p, f"경로가 {self._last_path_cut}번째 점에서 막힘"
            )
            return None

        if self.verbose_logs:
            self.get_logger().info(
                f"REJOIN path generated: d0={d0:.2f}m, L={L:.2f}m, "
                f"v_ego={self._ego_speed_mps:.2f}m/s, samples={len(out.poses)}"
            )
        return out

    def _warn_rejoin_given_up(self, d0: float, d0p: float, why: str) -> None:
        """재합류를 못 만들어 CSV 로 넘긴다 (1초에 한 번).

        `verbose_logs` 뒤에 두면 안 되는 사건이다. 이건 "로그가 조금 아쉽다"
        가 아니라 **차가 라인에서 벗어나 있는 채로 override 가 내려가는**
        순간이다. 그 뒤로는 Stanley 가 CSV 를 직접 겨누므로, 이탈이 클수록
        복귀가 급해진다 — 부드럽게 붙이려고 만든 게 재합류인데 그게 실패한
        자리에서 가장 거친 복귀가 나온다. 조용히 넘어가면 원인이 안 보인다.

        이탈량과 속도를 같이 찍는다. 둘이 있어야 "얼마나 급한 복귀를 CSV 에
        떠넘겼는지" 가 나온다.
        """
        now = self.get_clock().now().nanoseconds
        if now - getattr(self, "_last_rejoin_warn_ns", 0) < 1_000_000_000:
            return
        self._last_rejoin_warn_ns = now
        self.get_logger().warn(
            f"REJOIN 포기 — {why}. 이탈 {d0:+.2f} m, 헤딩 "
            f"{math.degrees(math.atan(d0p)):+.0f}°, v={abs(self._ego_speed_mps):.1f}. "
            f"CSV 유지 — 이 이탈을 Stanley 가 직접 받는다"
        )

    def _csv_cte_abs_m(self, current_pose: PoseStamped) -> float:
        """CSV(raceline) 기준 |CTE| = Frenet lateral |d|."""
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        yaw = _quat_to_yaw(current_pose.pose.orientation)
        _, d_now, _, _, _, _ = self._project_to_frenet(x, y, yaw)
        return abs(float(d_now))

    def _rejoin_reset_progress(self, current_pose: PoseStamped | None) -> None:
        now = self.get_clock().now().nanoseconds
        self._rejoin_start_ns = now
        self._rejoin_moving_ns = now
        self._rejoin_travel_m = 0.0
        self._rejoin_last_xy = (
            None
            if current_pose is None
            else (
                float(current_pose.pose.position.x),
                float(current_pose.pose.position.y),
            )
        )

    def _rejoin_track_progress(self, current_pose: PoseStamped | None) -> None:
        """REJOIN 유지 중 이동거리와 '마지막으로 움직인 시각' 을 갱신.

        모드 갱신과 발행 양쪽에서 불릴 수 있어 주기당 1회로 묶는다. 두 번
        세면 이동거리가 두 배로 잡혀 경로를 필요보다 자주 다시 그린다.
        """
        if self._rejoin_progress_cycle == self._tf_cycle_id:
            return
        self._rejoin_progress_cycle = self._tf_cycle_id
        if abs(self._ego_speed_mps) >= self.rejoin_stall_speed_mps:
            self._rejoin_moving_ns = self.get_clock().now().nanoseconds
        if current_pose is None:
            return
        xy = (
            float(current_pose.pose.position.x),
            float(current_pose.pose.position.y),
        )
        if self._rejoin_last_xy is not None:
            self._rejoin_travel_m += math.hypot(
                xy[0] - self._rejoin_last_xy[0], xy[1] - self._rejoin_last_xy[1]
            )
        self._rejoin_last_xy = xy

    def _rejoin_abandon_reason(self) -> str:
        """REJOIN 을 포기하고 CSV 로 돌아가야 하면 사유, 아니면 빈 문자열.

        완료 판정(`_is_rejoin_finished`)은 |CTE| 가 줄어야 성립한다. 차가 서
        있으면 CTE 는 절대 줄지 않으므로 그것만으로는 영원히 못 빠져나온다 —
        실제로 정지 상태에서 REJOIN 에 갇힌 채 override 를 계속 내보내
        Stanley 가 수십 초 묵은 캐시 경로를 붙들고 있었다. AVOID→REJOIN
        대기에 시간 상한을 뒀던 것과 같은 이유다.

        CSV 로 돌아가는 건 안전한 실패다 — override 가 내려가고 Stanley 가
        평소대로 레이스라인을 따라가며, 피드백은 접지력 예산으로 묶여 있다.
        """
        now = self.get_clock().now().nanoseconds
        if now - self._rejoin_moving_ns > self.rejoin_stall_ns:
            return "정지"
        if now - self._rejoin_start_ns > self._rejoin_budget_ns:
            return f"{self._rejoin_budget_ns / 1e9:.1f}s 초과"
        return ""

    def _refresh_rejoin_path(self, current_pose: PoseStamped) -> bool:
        """재합류 경로를 확보한다. 쓸 수 있으면 True.

        **한 번 그리면 끝까지 그대로 쓴다.** 주기적으로 다시 그려 봤더니
        오히려 복귀가 거칠어졌다: 차가 경로를 못 따라가 벌어지는 중에
        재생성이 겹치면 Stanley 기준경로가 통째로 갈아치워진다 (실측 CTE
        −0.11 → −0.43 → −0.64 로 벌어지다 재생성 순간 +0.01 로 리셋,
        0.3 초 주기 반복). 추종 오차를 경로를 바꿔서 없애면 안 된다 —
        그건 오차를 지우는 게 아니라 기준을 지우는 것이다.

        기동은 1~2 초면 끝나므로 경로가 묵을 일도 없다. 차가 서서 오래
        붙들리는 경우는 `_rejoin_abandon_reason` 이 따로 끊는다.
        """
        if self._rejoin_path_msg is not None and not self._alignment_done(current_pose):
            self._rejoin_track_progress(current_pose)
            return len(self._rejoin_path_msg.poses) >= 2

        path = self._build_frenet_quintic_rejoin_path(current_pose)
        if path is None or len(path.poses) < 2:
            return False
        self._rejoin_path_msg = path
        self._rejoin_reset_progress(current_pose)
        return True

    def _alignment_done(self, current_pose: PoseStamped) -> bool:
        """정렬 경로를 쥐고 있는데 방향이 맞았으면 True — 다시 그릴 때다.

        "한 번 그리면 끝까지" 규칙의 유일한 예외다. 정렬 경로는 애초에 이탈을
        줄이지 않으므로, 방향이 맞은 뒤에도 붙들고 있으면 차가 라인 옆을
        나란히 달리기만 한다. 그 규칙이 막으려던 건 *추종 오차 때문에* 경로를
        갈아치우는 것이지, 계획의 전제가 바뀐 경우가 아니다.
        """
        if not getattr(self, "_rejoin_is_alignment", False):
            return False
        _, _, _, _, _, yaw_err = self._project_to_frenet(
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            _quat_to_yaw(current_pose.pose.orientation),
        )
        if abs(yaw_err) > self._alignment_release_rad:
            return False
        self._rejoin_is_alignment = False
        self._rejoin_path_msg = None
        return True

    def _publish_rejoin_bridge(self, current_pose: PoseStamped | None) -> bool:
        """회피 경로가 더는 필요 없을 때 CSV 로 '생짜 전환' 하는 걸 막는다.

        override 를 그냥 내리면 Stanley 의 기준경로가 로컬경로 → CSV 로
        순간이동한다. 라인에서 벗어나 있으면 그 순간 CTE 가 계단으로 뛰고
        (실측 0.00 → −1.21 m) 급조향이 나간다. 실측에서는 이 상태로 2.4 초를
        달렸고, 정작 REJOIN 은 난폭한 복귀가 다 끝난 뒤에야 만들어졌다.

        라인에서 멀면 여기서 바로 재합류 경로를 깔아 override 를 유지한다.
        다음 주기에 `_update_mode` 가 정식으로 REJOIN 으로 넘긴다.
        """
        if not self.rejoin_enable or current_pose is None:
            return False
        if self._csv_cte_abs_m(current_pose) <= self.rejoin_finish_lateral_m:
            return False  # 이미 라인 위다 — 붙일 게 없으니 CSV 로 넘겨도 된다
        if not self._refresh_rejoin_path(current_pose):
            return False
        self._rejoin_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self._rejoin_path_msg)
        if self.pub_sent_dbg is not None:
            self.pub_sent_dbg.publish(self._stamp_copy_of_path(self._rejoin_path_msg))
        self._set_path_planned(False)  # REJOIN 게인은 FF 가 꺼진 채로 맞춰 뒀다
        self._publish_override_gate(True)
        return True

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
        self._clear_maneuver()
        self._reset_trailing_state()

    def _avoid_blocked(self) -> bool:
        """회피 경로가 막힌 상태인가 (시간 래치)."""
        if self._avoid_blocked_until_ns <= 0:
            return False
        if self.get_clock().now().nanoseconds >= self._avoid_blocked_until_ns:
            self._avoid_blocked_until_ns = 0
            return False
        return True

    def _mark_avoid_blocked(self) -> None:
        self._avoid_blocked_until_ns = (
            self.get_clock().now().nanoseconds + self.avoid_retry_ns
        )
        self._avoid_blocked_frames += 1

    def _clear_avoid_blocked(self) -> None:
        self._avoid_blocked_until_ns = 0
        self._avoid_blocked_frames = 0

    def _hold_last_avoid_path(self) -> bool:
        """직전 회피 경로를 다시 내보냈으면 True.

        포기 판정(`_avoid_give_up`)이 서기 전, 그리고 그 경로가 아직 신선할
        때만이다. 오래된 경로는 차가 이미 지나쳐서 뒤를 가리킨다.
        """
        if self._avoid_give_up():
            return False
        if self._last_good_avoid_path is None:
            return False
        age_ns = self.get_clock().now().nanoseconds - self._last_good_avoid_ns
        if age_ns > self.avoid_hold_max_ns:
            return False
        self.pub_path.publish(self._last_good_avoid_path)
        self._publish_override_gate(True)
        return True

    def _avoid_give_up(self) -> bool:
        """회피를 접고 GLOBAL/TRAILING 으로 내려갈 때인가.

        래치(`_avoid_blocked`)만으로는 부족하다. 그건 한 프레임 실패에도
        걸리는데, 걸리는 순간 0.5 초를 통째로 CSV 로 보내 버린다. 회피 중에
        라인으로 돌아가는 건 장애물 쪽으로 되돌아가는 것과 같다.

        연속 실패 프레임까지 봐야 "진짜 못 지나간다" 다.
        """
        return (
            self._avoid_blocked()
            and self._avoid_blocked_frames >= self.avoid_blocked_frames_th
        )

    def _cb_aeb(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active and not self._aeb_active:
            self._aeb_count += 1
        if self._aeb_active and not active and self.aeb_escape_enable:
            # 하강엣지 — AEB 가 풀렸다. AEB 노드의 탈출 창이 열려 있는 동안
            # 실제로 빠져나가야 하므로 그 시간만큼 탈출 모드를 유지한다.
            self._aeb_escape_until_ns = (
                self.get_clock().now().nanoseconds + self.aeb_escape_hold_ns
            )
        self._aeb_active = active

    def _aeb_escape_active(self) -> bool:
        """AEB 탈출 모드인가.

        두 구간을 합친다.
          1. AEB 가 걸린 채 **실제로 멈춘** 동안 — 정지 상태에서 조향을 미리
             돌려 둔다. control_node 는 AEB 중에도 조향은 /drive 를 따르므로
             바퀴가 탈출 방향으로 꺾인 채 대기하게 된다.
          2. AEB 해제 후 aeb_escape_hold_sec — 그 방향으로 빠져나가는 구간.

        1 에 속도 조건을 거는 이유: 아직 고속으로 제동 중일 때 조향을 새
        경로로 틀면 거동이 예측 밖으로 간다. 멈춘 뒤에만 바꾼다.
        """
        if not self.aeb_escape_enable:
            return False
        if self._aeb_active:
            return self._ego_speed_mps <= self.aeb_escape_arm_speed
        return (
            self._aeb_escape_until_ns > 0
            and self.get_clock().now().nanoseconds < self._aeb_escape_until_ns
        )

    def _log_aeb_escape(self, active: bool) -> None:
        if active == self._aeb_escape_logged:
            return
        self._aeb_escape_logged = active
        if active:
            self.get_logger().warn(
                f"AEB 탈출 모드 진입 — FGM 회피경로 강제 발행, "
                f"속도 ≤{self.aeb_escape_speed_mps:.2f}m/s "
                f"(aeb_total={self._aeb_count})"
            )
        else:
            self.get_logger().info("AEB 탈출 모드 종료 — 정상 판정 복귀")

    def _update_escape_heading(self, current: PoseStamped | None) -> None:
        """탈출 조준의 기준 방향을 FGM 에 알린다.

        멈춘 순간의 맵 헤딩을 한 번 잡아 두고, 매 주기 **지금 헤딩과의 차이**
        를 낸다. 그 차이가 곧 "기억한 방향" 을 차체 기준으로 본 각도다. 탈출
        중에 차가 왼쪽으로 돌아가면 이 값이 오른쪽으로 커지므로, FGM 이 이걸
        선호하는 것만으로 원래 방향으로 되돌아온다.

        기준을 트랙 접선이 아니라 **멈춘 순간의 헤딩** 으로 잡는 이유: AEB 는
        직진 제동이라 멈춘 헤딩이 진행 방향과 거의 같고, 로컬라이제이션이
        틀어져 있어도 이 값은 영향을 안 받는다.

        보내는 것은 [기준각, 허용 콘] 두 값이다. 콘까지 같이 보내는 이유는
        탈출 정책이 여기 있기 때문이다 — FGM 쪽에 따로 두면 두 값이 어긋난다.
        탈출이 아니거나 자세를 모르면 빈 배열을 보내고, FGM 은 그때 원래대로
        정면을 선호한다.
        """
        data: list[float] = []
        if self._aeb_escape_active() and self.aeb_escape_heading_lock:
            yaw_now = (
                _quat_to_yaw(current.pose.orientation) if current is not None else None
            )
            if yaw_now is not None:
                if self._aeb_escape_yaw is None:
                    self._aeb_escape_yaw = yaw_now
                    self.get_logger().warn(
                        f"탈출 헤딩 고정 — 멈춘 방향 ±"
                        f"{math.degrees(self.aeb_escape_heading_cone_rad):.0f}° "
                        f"안에서만 빠져나간다"
                    )
                data = [
                    _wrap_pi(self._aeb_escape_yaw - yaw_now),
                    self.aeb_escape_heading_cone_rad,
                ]
        else:
            self._aeb_escape_yaw = None
        self.pub_fgm_prefer.publish(Float32MultiArray(data=data))

    def _maybe_log_trailing(self) -> None:
        if self._trailing_log_period_ns <= 0:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_trailing_log_ns < self._trailing_log_period_ns:
            return
        self._last_trailing_log_ns = now_ns
        gap, v_lead = self._forward_leader()
        gap_s = "inf" if not math.isfinite(gap) else f"{gap:.2f}"
        self.get_logger().info(
            f"[TRAILING] gap={gap_s}m target={self.trailing_target_gap_m:.2f}m "
            f"lead={v_lead:.2f}m/s cmd={self._last_avoid_speed:.2f}m/s "
            f"ego={self._ego_speed_mps:.2f}m/s aeb_total={self._aeb_count}"
        )

    def _trailing_should_enter(self) -> bool:
        """GLOBAL → TRAILING 조건. 호출부에서 AVOID 진입이 안 선 것이 이미 확인됐다."""
        if not self.trailing_enable:
            return False
        # 따라갈 수 있는 앞차일 때만. 서 있는 차 뒤에 TRAILING 으로 붙으면
        # 갭만 지키며 영원히 그 자리에 선다 — 그건 AVOID 가 할 일이다.
        # _update_mode 가 이번 주기에 갱신해 둔 래치를 그대로 쓴다.
        if not self._leader_latched:
            return False
        gap = self._forward_gap_s_m()
        return math.isfinite(gap) and gap <= self.trailing_enter_m

    def _reset_trailing_state(self) -> None:
        self._trailing_exit_count = 0
        self._trail_prev_err = None
        self._trail_prev_ns = 0
        self._trail_integral = 0.0

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
        # use_predicted_s 면 XY 최근접 대신 "예측 s 갭" 으로 진입을 본다.
        # 앞차가 빠르게 멀어지는 중이면 지금 가까워도 회피할 이유가 없다.
        d_dyn_for_gate = d_dyn_closest
        if self._use_predicted_s:
            gap_s = self._forward_gap_s_m()
            if math.isfinite(gap_s):
                d_dyn_for_gate = gap_s
        dynamic_obstacle_on = (
            len(filtered_dynamic) >= 6
            and math.isfinite(d_dyn_for_gate)
            and d_dyn_for_gate <= on_m
            and rel_speed > 0.0
        )
        obstacle_on = static_obstacle_on or dynamic_obstacle_on
        self._obstacle_on = obstacle_on
        # ---- 분류: 이 장애물을 무엇으로 볼 것인가 ----
        # 추월 로직이 없으므로 "같은 방향으로 달리는 앞차" 는 회피 대상이
        # 아니다. 뒤에 붙는다. 예전에는 dynamic 도 AVOID 를 트리거해서 앞차를
        # 만날 때마다 AVOID↔TRAILING↔GLOBAL 을 오갔고, 그때마다 속도 정책이
        # 통째로 바뀌어 버벅였다.
        #
        #   정적 / 사실상 정지 / 역주행  → AVOID    (비켜 간다)
        #   같은 방향 주행 앞차          → TRAILING (뒤에 붙는다)
        #   여유 안쪽으로 들어온 것      → AEB (emergency_brake_node)
        #
        # 모든 분기가 이 하나를 봐야 한다. GLOBAL 만 고치고 TRAILING 분기가
        # obstacle_on 을 보면, 붙자마자 다시 AVOID 로 튕겨 나간다.
        followable = self.trailing_enable and self._update_leader_latch()
        avoid_on = static_obstacle_on or (dynamic_obstacle_on and not followable)
        still_blocking = self._obstacles_remain(filtered)
        fully_cleared = self._avoidance_fully_cleared(filtered, current_pose)

        old_mode = self.mode

        # AEB 로 멈췄으면 다른 전이보다 탈출이 먼저다. TRAILING 은 CSV 를
        # 그대로 타므로 정면이 막힌 상황에서는 조향할 경로가 없다. AVOID 로
        # 밀어 넣어 FGM 경로가 나오게 한다.
        escaping = self._aeb_escape_active()
        self._log_aeb_escape(escaping)
        if escaping:
            self._clear_avoid_blocked()
            if self.mode != "AVOID":
                self.mode = "AVOID"
                self._avoid_off_count = 0
                self._rejoin_path_msg = None
                self._reset_trailing_state()
                self._log_mode_transition(old_mode, d_closest)
            return

        if self.mode == "GLOBAL":
            if avoid_on and self.use_fgm:
                self._avoid_on_count += 1
            else:
                self._avoid_on_count = 0

            # 래치가 살아 있으면 방금 회피 경로가 막힌 것이다. 바로 다시
            # AVOID 로 올라가면 같은 실패를 반복하며 모드만 떤다. 카운트는
            # 계속 세서 래치가 풀리는 즉시 재시도하게 둔다.
            if (
                self._avoid_on_count >= self.avoid_on_count_th
                and not self._avoid_blocked()
            ):
                self.mode = "AVOID"
                self._avoid_off_count = 0
                self._rejoin_path_msg = None
                self._clear_avoid_blocked()
            elif self._trailing_should_enter():
                # 앞차가 s 방향으로 가까운데 회피 진입 조건은 안 선다
                # = 옆으로 못 간다. 붙어서 따라가되 갭만 지킨다.
                self.mode = "TRAILING"
                self._reset_trailing_state()

        elif self.mode == "AVOID" and self._avoid_give_up():
            # 회피 경로가 쓸 만한 길이로 안 나온다 = 지나갈 틈이 없다.
            # 래치는 지우지 않는다 — 지우면 다음 프레임에 바로 AVOID 로 튕긴다.
            #
            # 어디로 보낼지는 "따라갈 앞차가 있나" 로 갈린다.
            #   있다  → TRAILING. 갭을 지키며 뒤에 붙는다.
            #   없다  → GLOBAL.  정적 장애물을 TRAILING 으로 보내면 전방 갭이
            #           inf 라 5 프레임 뒤 곧바로 GLOBAL 로 빠져나가고,
            #           AVOID→TRAILING→GLOBAL→AVOID 가 200 ms 주기로 돈다.
            #           그 사이 AEB 완화 기준이 같이 깜빡인다. 처음부터
            #           GLOBAL 로 보내면 완화 없이 엄격한 기준을 유지한다.
            #
            # 어느 쪽이든 경로는 발행되지 않으므로 Stanley 는 CSV 를 탄다.
            # 감속은 모드와 무관한 속도 정책이 하고, 그래도 못 서면 AEB 가
            # 잡은 뒤 탈출 로직(_aeb_escape_active)이 빠져나간다.
            if self.trailing_enable and followable:
                self.mode = "TRAILING"
                self._reset_trailing_state()
            else:
                self._go_global()

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

            # 계획 기동은 복귀 곡선까지 자기 안에 갖고 있다. 장애물이 시야에서
            # 빠졌다고 여기서 모드를 내리면, 아직 오프셋에 나가 있는 상태에서
            # override 가 풀려 Stanley 가 CSV 로 직접 되꺾는다 — 예전에 벽으로
            # 가던 경로가 정확히 이거였다. 기동이 끝날 때까지 붙들고, 끝나면
            # 이미 라인 위이므로 REJOIN 없이 바로 GLOBAL 로 간다.
            if self._maneuver is not None and current_pose is not None:
                ds = self._maneuver_ds_cache
                if ds is not None and ds < self._maneuver.total_length_m:
                    self._avoid_off_count = 0
                    return
                self._clear_maneuver()
                if not obstacle_still_ahead:
                    self._go_global()
                    if old_mode != self.mode:
                        self._log_mode_transition(old_mode, d_closest)
                    return

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
                # pose 가 없으면 rejoin 경로를 만들 수도, CTE 를 잴 수도 없다.
                # 그 상태로 기다리면 영구히 못 나온다.
                can_rejoin = self.rejoin_enable and current_pose is not None
                cte_ok = (
                    current_pose is not None
                    and self._csv_cte_abs_m(current_pose)
                    <= self.rejoin_finish_lateral_m
                )
                # 라인에서 벗어나 있으면 **지금 바로** 재합류로 넘긴다.
                #
                # 예전에는 여기서 "CTE 가 줄 때까지" 최대 1.5 초를 AVOID 로
                # 붙들고 기다렸다. 그런데 그 시점엔 이미 발행
                # 쪽에서 override 를 내려 Stanley 가 CSV 를 생짜로 쫓는
                # 중이었다 — 즉 기다리는 동안 보호받는 게 아니라 정확히
                # 그 반대였다. 실측: 회피 해제 68.6 s → REJOIN 71.0 s, 그
                # 2.4 초 동안 1.7 m 벗어난 채 6 m/s^2 로 가속하며 라인으로
                # 되꺾였고, 부드럽게 붙여야 할 REJOIN 은 다 끝난 뒤 v=0
                # 에서 시작했다. CTE 를 줄이는 건 재합류 경로가 할 일이지
                # 기다린다고 줄어드는 게 아니다.
                if can_rejoin and not cte_ok:
                    if self._refresh_rejoin_path(current_pose):
                        self.mode = "REJOIN"
                    else:
                        self._go_global()
                else:
                    self._go_global()

        elif self.mode == "REJOIN":
            if avoid_on and self.use_fgm:
                self.mode = "AVOID"
                self._rejoin_path_msg = None
                self._avoid_off_count = 0
                self._clear_avoid_blocked()
            elif current_pose is not None and self._is_rejoin_finished(current_pose):
                self._go_global()
            else:
                self._rejoin_track_progress(current_pose)
                reason = self._rejoin_abandon_reason()
                if reason:
                    self.get_logger().warn(f"REJOIN 포기({reason}) — CSV 로 복귀")
                    self._go_global()

        elif self.mode == "TRAILING":
            gap = self._forward_gap_s_m()
            if avoid_on and self.use_fgm and not self._avoid_blocked():
                # 앞차가 서거나 돌아섰다(=따라갈 대상이 아니다) 또는 정적
                # 장애물이 새로 들어왔다 — 회피로 넘긴다. 래치가 아직 살아
                # 있으면 방금 막혔던 것이므로 재시도 주기까지 기다린다.
                self.mode = "AVOID"
                self._avoid_off_count = 0
                self._rejoin_path_msg = None
                self._reset_trailing_state()
            elif self._s_ego is None:
                # pose 가 없어서 갭을 못 잰 것뿐이다. 이걸 "앞차가 사라졌다"
                # 로 세면 TF 가 한 번 끊길 때마다 TRAILING 이 풀려서 앞차
                # 쪽으로 다시 가속한다. 판단 못 할 때는 상태를 유지한다.
                pass
            else:
                if (not math.isfinite(gap)) or gap > self.trailing_exit_m:
                    self._trailing_exit_count += 1
                else:
                    self._trailing_exit_count = 0
                if self._trailing_exit_count >= self.trailing_exit_count_th:
                    self._go_global()

        if old_mode != self.mode:
            self._log_mode_transition(old_mode, d_closest)

    # ------------------------------------------------------------------
    # 횡오프셋 기동 — 계획하고 붙들고 간다
    # ------------------------------------------------------------------
    def _maneuver_obstacles_sd(self) -> list:
        """기준선 좌표계 장애물. s 는 자차 앞쪽으로의 거리.

        동적 장애물도 사실상 정지해 있으면(추종 대상이 아니면) 같이 넘긴다.
        움직이는 앞차는 TRAILING 이 따로 본다.
        """
        if self._s_ego is None:
            return []
        out = []
        for s, d, r in self._static_sd:
            out.append(ObstacleSD(self._delta_s(s, self._s_ego), float(d), float(r)))
        for entry in self._dynamic_sd:
            s, d, r, vs = entry[0], entry[1], entry[2], entry[3]
            if abs(float(vs)) > self.trailing_min_leader_speed_mps:
                continue  # 같이 달리는 앞차 — 비켜갈 대상이 아니다
            out.append(ObstacleSD(self._delta_s(s, self._s_ego), float(d), float(r)))
        return out

    #: 한 주기에 s 가 이만큼 넘게 변했다면 위치추정이 튄 것이다. 그대로
    #: 누적하면 기동 진행도가 통째로 어긋나므로 버린다.
    _MANEUVER_MAX_STEP_M = 2.0

    #: 진입/복귀 한 구간이 차지해도 되는 트랙 길이 비율. 이 트랙은 37 m 라
    #: 예산대로 뽑으면 한 기동이 반바퀴를 먹고, 그러면 랩 대부분을 라인 밖에서
    #: 보내게 된다. 0.18(=6.7 m)은 7 m/s 까지 감속 없이 지나갈 수 있는
    #: 최소값이다 — 더 줄이면 곡률이 올라가 속도 상한이 걸린다.
    _MANEUVER_LEN_TRACK_FRAC = 0.18

    def _maneuver_len_cap(self, requested_m: float) -> float:
        cap = self._MANEUVER_LEN_TRACK_FRAC * self._total_l
        return max(0.5, min(float(requested_m), cap))

    def _build_maneuver_config(self) -> None:
        """트랙 길이를 알아야 길이 상한이 정해져서 CSV 로드 뒤에 만든다."""
        g = self.get_parameter
        steer_frac = min(1.0, max(0.1, float(g("avoid_offset_steer_frac").value)))
        # 기동 여유는 **속도제한이 면제를 내주는 기준과 같아야** 한다. 이보다
        # 좁게 계획하면, 계획대로 비켜 가는데도 정지거리 한계가 안 풀려서
        # 회피 내내 기어간다 (감속을 줄이려고 만든 기동이 감속을 만든다).
        # 이보다 넓게 잡을 이유도 없다 — 트랙 폭만 낭비한다.
        margin_floor = (
            self.avoid_speed_params.lateral_margin_m
            + self.avoid_speed_params.pass_clear_extra_m
        )
        self.maneuver_cfg = ManeuverConfig(
            half_width_m=vg.HALF_WIDTH_M,
            lateral_margin_m=max(
                margin_floor, float(g("avoid_offset_margin_m").value)
            ),
            max_offset_m=max(0.05, float(g("avoid_offset_max_m").value)),
            a_lat_enter_mps2=max(0.2, float(g("avoid_offset_a_lat_enter").value)),
            a_lat_exit_mps2=max(0.2, float(g("avoid_offset_a_lat_exit").value)),
            a_lat_hard_mps2=max(0.3, float(g("avoid_offset_a_lat_hard").value)),
            enter_min_m=max(0.2, float(g("avoid_offset_enter_min_m").value)),
            enter_max_m=self._maneuver_len_cap(
                float(g("avoid_offset_enter_max_m").value)
            ),
            exit_min_m=max(0.2, float(g("avoid_offset_exit_min_m").value)),
            exit_max_m=self._maneuver_len_cap(
                float(g("avoid_offset_exit_max_m").value)
            ),
            # 앞범퍼가 장애물 표면에 닿기 전에 오프셋에 올라와 있어야 한다.
            hold_front_m=vg.FRONT_M + 0.20,
            hold_rear_m=vg.LENGTH_M
            + max(0.0, float(g("avoid_offset_hold_rear_extra_m").value)),
            merge_gap_m=max(0.0, float(g("avoid_offset_merge_gap_m").value)),
            v_plan_min_mps=1.5,
            max_steer_rad=steer_frac * _MAX_STEER_RAD,
            wheelbase_m=vg.WHEELBASE_M,
        )
        self.avoid_offset_corner_aware = param_bool(
            g("avoid_offset_corner_aware").value
        )
        self.get_logger().info(
            f"회피 기동: 여유 {self.maneuver_cfg.lateral_margin_m:.2f}m, "
            f"오프셋≤{self.maneuver_cfg.max_offset_m:.2f}m, "
            f"진입≤{self.maneuver_cfg.enter_max_m:.1f}m 복귀≤"
            f"{self.maneuver_cfg.exit_max_m:.1f}m (트랙 {self._total_l:.0f}m), "
            f"a_lat {self.maneuver_cfg.a_lat_enter_mps2:.1f}/"
            f"{self.maneuver_cfg.a_lat_exit_mps2:.1f}/"
            f"{self.maneuver_cfg.a_lat_hard_mps2:.1f}, "
            f"계획조향≤{math.degrees(self.maneuver_cfg.max_steer_rad):.0f}°"
            f"{', 코너 κ 반영' if self.avoid_offset_corner_aware else ''}"
        )

    def _advance_maneuver_ds(self, current: PoseStamped | None) -> float | None:
        """계획 앵커로부터 진행한 s [m]. **주기당 한 번만** 부르고 나머지는 캐시를 본다.

        `_delta_s(s_now, s0)` 를 직접 쓰면 안 된다. 그건 [-L/2, +L/2) 로
        정규화하므로, 기동이 트랙 반바퀴에 가까워지는 순간(37 m 트랙에서는
        18.7 m) 진행도가 음수로 접힌다. 실제로 19.4 m 짜리 기동에서 ds 가
        +19.8 대신 -17.7 로 나와 완료 판정이 영영 서지 않았다.

        그래서 매 주기의 **증분**만 _delta_s 로 재고(그 정도 거리는 모호하지
        않다) 누적한다. 이러면 한 바퀴를 넘겨도 진행도가 단조증가한다.
        """
        if self._maneuver is None or self._maneuver_s0 is None or current is None:
            self._maneuver_ds_cache = None
            return None
        s_now, _ = self._frenet_xy(
            float(current.pose.position.x), float(current.pose.position.y)
        )
        if self._maneuver_last_s is None:
            self._maneuver_ds_cache = self._delta_s(s_now, self._maneuver_s0)
        else:
            step = self._delta_s(s_now, self._maneuver_last_s)
            if abs(step) <= self._MANEUVER_MAX_STEP_M:
                self._maneuver_ds_cache = (self._maneuver_ds_cache or 0.0) + step
        self._maneuver_last_s = s_now
        return self._maneuver_ds_cache

    def _maneuver_still_clears(self, obstacles: list, ds_now: float) -> bool:
        """지금 계획이 현재 보이는 장애물을 여전히 비켜 가는가."""
        m = self._maneuver
        if m is None:
            return False
        need = vg.HALF_WIDTH_M + self.maneuver_cfg.lateral_margin_m
        for o in obstacles:
            ds_obs = ds_now + o.s
            if ds_obs < 0.0 or ds_obs > m.total_length_m + 1.0:
                continue
            if abs(m.d_at(ds_obs) - o.d) < o.r + need - 1e-6:
                return False
        return True

    def _maneuver_lat_at(self, x_base: float) -> float:
        """앞으로 x_base 만큼 갔을 때 계획 경로의 **차량 기준** 횡위치 [m].

        속도제한이 "이 장애물 옆을 지날 때 우리가 어디 있나" 를 물을 때 쓴다.
        장애물 y 는 차량 기준이고 계획 d 는 트랙 기준이라, 자차의 d 를 빼서
        기준을 맞춘다. 회피 중에는 차가 트랙 방향과 거의 나란하므로 이
        평행이동으로 충분하다.
        """
        m = self._maneuver
        if m is None or self._maneuver_ds_cache is None or self._d_ego is None:
            return 0.0
        return m.d_at(self._maneuver_ds_cache + max(0.0, float(x_base))) - self._d_ego

    def _plan_or_keep_maneuver(
        self, current: PoseStamped, *, forbid_side: int = 0
    ) -> bool:
        """기동을 새로 세우거나 기존 것을 유지한다. 쓸 게 있으면 True.

        재계획 조건을 좁게 잡는 게 핵심이다. 매 주기 자차 위치로 다시 그리면
        차는 진입 곡선의 제일 급한 앞부분을 계속 새로 타고, 오프셋에는 영영
        도달하지 못한 채 조향만 물고 있게 된다.
        """
        if self._s_ego is None:
            return False
        obstacles = self._maneuver_obstacles_sd()
        ds_now = self._maneuver_ds_cache

        keep = False
        if self._maneuver is not None and ds_now is not None and forbid_side == 0:
            _, d_now = self._frenet_xy(
                float(current.pose.position.x), float(current.pose.position.y)
            )
            off_plan = abs(d_now - self._maneuver.d_at(ds_now))
            keep = (
                ds_now >= -0.5
                and ds_now < self._maneuver.total_length_m
                and off_plan <= self.avoid_offset_replan_lateral_m
                and self._maneuver_still_clears(obstacles, ds_now)
            )
        if keep:
            self._maneuver_speed_cap = self._maneuver.speed_cap_mps
            return True

        if not obstacles:
            self._clear_maneuver()
            return False

        cx = float(current.pose.position.x)
        cy = float(current.pose.position.y)
        yaw = _quat_to_yaw(current.pose.orientation)
        _, d0, d0p, _, _, _ = self._project_to_frenet(cx, cy, yaw)
        s_now, _ = self._frenet_xy(cx, cy)
        # 오프셋을 물고 지나갈 구간의 좌/우 여유. 이걸 안 주면 계획기가 벽
        # 쪽으로 비키는 계획을 낸다 — 실차에서 그렇게 박았다.
        max_left, max_right = self._wall_budget_over(*self._hold_window(obstacles))
        fresh, plan_v = self._plan_fitting_the_track(
            obstacles,
            d0,
            d0p,
            s_now,
            forbid_side=forbid_side,
            max_left=max_left,
            max_right=max_right,
        )
        if fresh is None:
            self._clear_maneuver()
            return False

        self._maneuver = fresh
        self._maneuver_s0 = s_now
        self._maneuver_last_s = s_now
        self._maneuver_ds_cache = 0.0
        self._maneuver_speed_cap = _min_opt(fresh.speed_cap_mps, plan_v)
        self._log_maneuver(fresh)
        return True

    def _plan_fitting_the_track(
        self,
        obstacles,
        d0: float,
        d0p: float,
        s_now: float,
        *,
        forbid_side: int,
        max_left: float,
        max_right: float,
    ) -> Tuple[OffsetManeuver | None, float | None]:
        """트랙 안에 실제로 들어가는 기동을 찾는다. (기동, 계획속도).

        진입·복귀 길이는 `sqrt(D2·|Δd|·v²/a)` 라 **v 에 선형**이다. 6 m/s 에서
        0.58 m 를 비키려면 진입만 6.3 m 고 기동 전체가 16 m 인데, 37 m 짜리
        트랙에서 그건 반 바퀴다. 코너를 몇 개씩 지나가므로 벽에 걸릴 수밖에 없다.

        그래서 **속도를 낮춰 가며** 다시 뽑는다. 절반 속도면 절반 길이라
        조금만 낮춰도 기동이 트랙 안에 들어온다. 실측(37 m 트랙, 전 지점에
        정면 장애물): 6 m/s 고정이면 FGM 폴백이 51% 인데, 이 탐색을 붙이면
        14% 로 떨어지고 76% 를 기동으로 처리한다. 감속한 경우의 목표속도는
        중앙값 3.5 m/s 다.

        이건 "고속에서 감속을 최대한 덜 한다" 와 어긋나지 않는다. 풀속도로
        되는 자리에서는 첫 시도가 바로 통과하고, 안 되는 자리는 **어차피
        기동이 트랙에 안 들어가는** 자리다. 거기서 대안은 FGM 으로 벽에
        박거나 감속하거나 둘 중 하나다.
        """
        v_now = max(abs(self._ego_speed_mps), self.avoid_offset_plan_v_floor_mps)
        v_try = v_now
        while True:
            for forbid in _forbid_order(forbid_side):
                m = plan_maneuver(
                    obstacles,
                    self.maneuver_cfg,
                    d_ego=d0,
                    d_ego_prime=d0p,
                    v=v_try,
                    forbid_side=forbid,
                    max_left=max_left,
                    max_right=max_right,
                    kappa_ref=(
                        (lambda ds: self._kappa_at_s(s_now + ds)[0])
                        if self.avoid_offset_corner_aware
                        else None
                    ),
                )
                if m is None:
                    continue
                if self._maneuver_fits_walls(m, s_now):
                    return m, (None if v_try >= v_now - 1e-9 else v_try)
            if v_try <= self.avoid_offset_plan_v_floor_mps + 1e-9:
                return None, None
            v_try = max(
                self.avoid_offset_plan_v_floor_mps,
                v_try - self.avoid_offset_plan_v_step_mps,
            )

    # 벽 적합성 검사 간격 [m]. 예산 격자가 0.025 라 이보다 촘촘히 볼 필요 없다.
    _WALL_FIT_STEP_M = 0.20

    def _maneuver_fits_walls(self, m: OffsetManeuver, s0: float) -> bool:
        """계획된 d(s) 가 s 마다의 좌/우 예산 안에 들어오는가.

        `_wall_budget_over` 는 유지 구간만 봐서 진입·복귀 램프를 놓친다.
        그쪽은 |d| 가 오르내리는 중이라 스칼라 하나로 묶으면 과하게
        보수적이거나(전 구간 최솟값) 과하게 낙관적이다(유지 구간만). 계획이
        나온 뒤에는 d(s) 를 아니까 점마다 정확히 보면 된다.
        """
        if self._budget_left is None or self._total_l < 1e-6:
            return True
        ds = np.arange(0.0, m.total_length_m + 1e-9, self._WALL_FIT_STEP_M)
        if ds.size == 0:
            return True
        d = np.array([m.d_at(float(t)) for t in ds])
        idx = (
            ((s0 + ds) % self._total_l) / self._total_l * self._n
        ).astype(np.int64) % self._n
        cap = np.where(d >= 0.0, self._budget_left[idx], self._budget_right[idx])
        return bool(np.all(np.abs(d) <= cap + 1e-9))

    def _hold_window(self, obstacles) -> Tuple[float, float]:
        """차가 **최대 오프셋을 물고 있을** 절대 s 구간 — 벽 예산을 물을 곳.

        기동 전체(진입~복귀)로 물으면 안 된다. 그 구간은 37 m 트랙에서 16 m 나
        되고, 최솟값을 취하니 트랙에서 제일 좁은 한 곳이 언제나 걸린다. 실측에서
        그렇게 했더니 **전 지점에서 계획이 거부**됐다.

        진입·복귀 구간의 |d| 는 0 에서 목표까지 오르내리는 중이라 예산을 덜
        쓴다. 거기까지 한 값으로 묶으면 과하게 보수적이다. 그쪽은 경로 점
        단위 충돌검사(`_path_fully_clear`)가 정확히 잡아 주므로, 여기서는
        **오프셋을 끝까지 물고 있는 구간**만 본다.
        """
        s0 = self._s_ego or 0.0
        s_first = min((o.s for o in obstacles), default=0.0)
        s_last = max((o.s for o in obstacles), default=0.0)
        return (
            s0 + s_first - self.maneuver_cfg.hold_front_m,
            s0 + s_last + self.maneuver_cfg.hold_rear_m,
        )

    def _clear_maneuver(self) -> None:
        self._maneuver = None
        self._maneuver_s0 = None
        self._maneuver_last_s = None
        self._maneuver_ds_cache = None
        self._maneuver_speed_cap = None

    def _log_maneuver(self, m: OffsetManeuver) -> None:
        now = self.get_clock().now().nanoseconds
        if now - self._last_maneuver_log_ns < 500_000_000:
            return
        self._last_maneuver_log_ns = now
        cap = (
            f", 속도상한 {m.speed_cap_mps:.1f}m/s"
            if m.speed_cap_mps is not None
            else ""
        )
        self.get_logger().info(
            f"회피 기동: {'왼' if m.side > 0 else '오른'}쪽 {abs(m.d_pass):.2f}m, "
            f"진입 {m.enter_len_m:.1f}m, 유지끝 {m.hold_end_ds:.1f}m, "
            f"복귀 {m.exit_len_m:.1f}m, 최대횡가속 "
            f"{m.peak_lateral_accel_mps2:.1f}m/s²{cap}"
        )

    def _build_offset_path(self, current: PoseStamped) -> Path | None:
        """계획된 d(s) 를 맵 좌표 경로로 편다.

        계획은 앵커 s0 기준이지만 경로는 **자차 앞쪽**만 낸다. 지나온 부분을
        같이 발행하면 Stanley 최근접점이 뒤로 잡혀 조향이 뒤집힌다.
        """
        m = self._maneuver
        ds_now = self._maneuver_ds_cache
        if m is None or ds_now is None:
            return None

        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()

        # 복귀가 끝난 뒤에도 기준선을 따라 조금 더 이어 붙인다. 경로 끝에서
        # Stanley 전방주시가 끊기면 그 지점에서 조향이 튄다.
        tail = self.avoid_frenet_exit_len_m
        step = self.avoid_offset_step_m
        span = max(step, m.total_length_m - max(0.0, ds_now) + tail)
        n = int(span / step)
        lim = self.maneuver_cfg.max_offset_m
        for k in range(n + 1):
            ds = max(0.0, ds_now) + k * step
            d = max(-lim, min(lim, m.d_at(ds)))
            x_ref, y_ref, yaw_ref = self._xy_yaw_at_s(self._maneuver_s0 + ds)
            self._append_pose(
                out, x_ref - d * math.sin(yaw_ref), y_ref + d * math.cos(yaw_ref)
            )
        if len(out.poses) < 2:
            return None
        return out

    def _build_avoid_path_frenet(
        self, current: PoseStamped, fgm_x: float, fgm_y: float
    ) -> Path | None:
        """레이스라인을 기준선으로 d(s) quintic 회피 경로.

        진입(자차 d → 목표 d) → 유지(apex) → 복귀(d → 0) 3단이다. 직선
        방식과 달리 기준선 곡률을 따라가므로 코너에서 조향이 튀지 않는다.
        기하만 만들고 안전성은 호출부의 _truncate_path_at_collision 이 본다.
        """
        cx = float(current.pose.position.x)
        cy = float(current.pose.position.y)
        yaw = _quat_to_yaw(current.pose.orientation)
        s0, d0, d0p, d0pp, _, _ = self._project_to_frenet(cx, cy, yaw)
        s_target, d_target = self._frenet_xy(float(fgm_x), float(fgm_y))

        lim = self.avoid_frenet_max_offset_m
        d_goal = max(-lim, min(lim, d_target))

        # 목표까지 남은 s 가 설정값보다 짧으면 그만큼만 쓴다 — 장애물 옆을
        # 지날 때는 이미 오프셋에 올라와 있어야 한다.
        ds_to_target = self._delta_s(s_target, s0)
        l_enter = self.avoid_frenet_enter_len_m
        if ds_to_target > 0.3:
            l_enter = min(l_enter, ds_to_target)
        l_enter = max(0.3, l_enter)
        l_exit = self.avoid_frenet_exit_len_m

        enter = self._solve_quintic(d0, d0p, d0pp, d_goal, 0.0, 0.0, l_enter)
        exit_ = self._solve_quintic(d_goal, 0.0, 0.0, 0.0, 0.0, 0.0, l_exit)

        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()

        step = self.avoid_frenet_step_m
        total = l_enter + self.avoid_frenet_hold_m + l_exit
        n = int(total / step)
        for k in range(n + 1):
            ds = min(total, k * step)
            if ds <= l_enter:
                d = self._eval_quintic(enter, ds)
            elif ds <= l_enter + self.avoid_frenet_hold_m:
                d = d_goal
            else:
                d = self._eval_quintic(exit_, ds - l_enter - self.avoid_frenet_hold_m)
            d = max(-lim, min(lim, d))
            x_ref, y_ref, yaw_ref = self._xy_yaw_at_s(s0 + ds)
            self._append_pose(
                out, x_ref - d * math.sin(yaw_ref), y_ref + d * math.cos(yaw_ref)
            )

        if len(out.poses) < 2:
            return None
        return out

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

    def _emit_avoid_path(
        self, out: Path, current: PoseStamped, *, planned: bool = False
    ) -> None:
        """회피 경로 발행 + override 게이트. 계획 기동과 FGM 폴백이 공유한다."""
        if len(out.poses) < 2:
            self._publish_override_gate(False)
            return
        self._set_path_planned(planned)
        self._last_good_avoid_path = out
        self._last_good_avoid_ns = self.get_clock().now().nanoseconds

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
        if self.verbose_logs and now_ns - self._last_latency_log_ns > 500_000_000:
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
        # REJOIN 도 켜 둔다. 연속 장애물 구간에서 하나를 피하고 복귀하는 도중
        # 다음 게 나타나면 즉시 AVOID 로 돌아가는데(디바운스 없음), 그때
        # FGM 이 꺼져 있었으면 목표점이 오래된 값이라 한두 프레임을 헛돈다.
        # 켜 두면 복귀 중에도 갭이 계속 갱신돼 곧바로 이어서 회피한다.
        enable = self.use_fgm and (
            self.mode in ("AVOID", "REJOIN") or approaching
        )
        msg = Bool()
        msg.data = bool(enable)
        self.pub_fgm_enable.publish(msg)

    def timer_publish(self):
        self._tf_cycle_id += 1  # laser→map TF 캐시 무효화 (주기당 1회 조회)
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

        # 투영할 장애물이 없으면 TF 를 굳이 찾지 않는다 — TF 가 끊긴 동안
        # lookup 이 timeout 만큼 블로킹돼 타이머 주기가 흔들린다.
        tf_lm = (
            self._lookup_laser_to_map_transform()
            if (filtered or filtered_dynamic)
            else None
        )
        self._update_frenet_snapshot(current, filtered, filtered_dynamic, tf_lm)
        self._publish_frenet_debug()
        # 기동 진행도는 주기당 한 번만 전진시킨다. 아래 모드판정·속도·경로가
        # 모두 이 캐시를 읽는다 (여러 번 부르면 증분이 중복 누적된다).
        self._advance_maneuver_ds(current)

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
        # 속도 정책보다 **먼저** 기동을 세운다. 속도는 계획을 보고 "비켜
        # 지나갈 거니까 정지거리 제동은 필요 없다" 를 판단하는데, 계획이 한
        # 주기 늦게 서면 AVOID 로 넘어가는 순간마다 속도가 한 번 꺼졌다 켜진다.
        # 아래 AVOID 분기는 이걸 그대로 재사용한다(유지 조건에 걸려 재계산 없음).
        if (
            self.mode == "AVOID"
            and self.avoid_path_mode == "offset"
            and current is not None
        ):
            self._plan_or_keep_maneuver(current)

        self._publish_planner_speed_out()
        self.pub_planner_mode.publish(String(data=self.mode))
        self._publish_fgm_enable(filtered, d_gate, filtered_dynamic, d_dyn_gate)
        self._update_escape_heading(current)
        self._maybe_log_speed_status(
            d_closest, d_dyn_closest, obs_speed, rel_speed
        )

        if self.mode == "GLOBAL":
            self._publish_override_gate(False)
            return

        if self.mode == "TRAILING":
            # 경로는 CSV 그대로 — 속도만 줄여서 갭을 지킨다.
            self._publish_override_gate(False)
            self._maybe_log_trailing()
            return

        if self.mode == "AVOID":
            escaping = self._aeb_escape_active()
            static_path = self._static_wants_fgm_local_path(
                filtered, d_closest, d_gate
            )
            dynamic_path = self._dynamic_wants_fgm_local_path(
                filtered_dynamic, d_dyn_closest, rel_speed
            )
            # 탈출 중에는 이 게이트를 건너뛴다. 여기서 걸러 버리면 정면에 멈춰
            # 선 상황에서 경로가 안 나와 빠져나갈 방법이 없다.
            if not escaping and not static_path and not dynamic_path:
                # 회피 경로는 더 필요 없지만, 아직 AVOID 해제 디바운스
                # (avoid_off_count_th) 가 안 끝났을 수 있다. 그 몇 주기 동안
                # override 를 내려 버리면 라인에서 벗어난 만큼 조향이 튄다.
                # 재합류 경로로 이어 붙여 기준경로가 연속이도록 한다.
                if self._publish_rejoin_bridge(current):
                    return
                self._publish_override_gate(False)
                return

            # 1순위: 미리 계획한 횡오프셋 기동. FGM 조준각과 무관하게
            # 장애물 기하와 속도만으로 나오므로 고속에서도 조향이 완만하다.
            if self.avoid_path_mode == "offset" and current is not None:
                # 플래너는 장애물만 보고 트랙 경계는 모른다. 벽 쪽으로 비키는
                # 계획이 나오면 충돌검사에 걸리므로, 그 방향을 막고 반대쪽으로
                # 한 번 더 뽑는다.
                blocked_side = 0
                for _attempt in (1, 2):
                    if not self._plan_or_keep_maneuver(
                        current, forbid_side=blocked_side
                    ):
                        break
                    out = self._build_offset_path(current)
                    if out is None:
                        break
                    # 계획 기동은 **통째로** 통과해야 쓴다. 잘라서 쓰면 안 된다.
                    #
                    # FGM 경로는 조준점까지의 직선이라 앞부분만 살려도 의미가
                    # 있지만, 기동은 진입-유지-복귀가 한 덩어리다. 벽에 걸려
                    # 복귀가 잘려 나간 걸 "쓸 만한 길이가 남았다" 며 받으면,
                    # 차는 진입만 타고 최대 오프셋에 도달한다 — 그 지점이 바로
                    # 잘려 나간 벽이다. 반대편으로 다시 뽑는 게 맞다.
                    if self._path_fully_clear(out, tf_lm):
                        self._clear_avoid_blocked()
                        self._emit_avoid_path(out, current, planned=True)
                        return
                    blocked_side = self._maneuver.side if self._maneuver else 0
                    self._clear_maneuver()
                    if blocked_side == 0:
                        break
                # 양쪽 다 막혔다 — 계획을 버린다.
                self._clear_maneuver()

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
            out = None
            if self.avoid_path_mode == "frenet":
                out = self._build_avoid_path_frenet(current, fgm_x, fgm_y)
                if out is None:
                    self._warn_frenet_avoid_fallback()
            if out is None:
                out = self._build_avoid_path(
                    current, fgm_x, fgm_y, merge_csv_tail=False
                )
            out, usable = self._truncate_path_at_collision(
                out,
                tf_lm,
                min_length_m=self.aeb_escape_min_path_m if escaping else None,
            )
            if not usable:
                # 연속으로 계속 막히면 다음 주기 _update_mode 가 접는다.
                self._mark_avoid_blocked()
                self._warn_avoid_path_blocked()
                # 포기 판정 전까지는 직전 경로를 유지한다.
                #
                # 여기서 게이트를 내리면 Stanley 가 그 즉시 CSV 로 돌아간다.
                # 회피 중에 라인으로 돌아가는 건 장애물 쪽으로 되돌아가는
                # 것과 같아서, 한 프레임 실패가 그대로 충돌이 된다. 직전
                # 프레임에 검증을 통과한 경로이므로 몇 프레임은 붙들 수 있다.
                # 정말 못 지나가면 카운트가 차서 접고, 그 뒤는 AEB 몫이다.
                if self._hold_last_avoid_path():
                    return
                self._publish_override_gate(False)
                return
            self._clear_avoid_blocked()
            self._emit_avoid_path(out, current)
            return

        if self.mode == "REJOIN":
            # 경로를 진입 시 한 번만 만들고 계속 재발행하면, 차가 나아간
            # 만큼 과거 위치에 앵커된 경로를 쫓게 된다. 일정 거리마다
            # 현재 위치에서 다시 그린다.
            if current is not None:
                self._refresh_rejoin_path(current)

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
                # 재합류 경로도 quintic 이라 곡률은 실제 값이지만, 여기 게인은
                # FF 가 꺼진 상태에서 맞춰 둔 것이다. 켜려면 따로 재튜닝해야
                # 하고 REJOIN 은 이제 AEB 탈출 뒤처리 정도로만 남는다.
                self._set_path_planned(False)
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
