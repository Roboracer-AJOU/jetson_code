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

from path_following import vehicle_geometry as vg
from path_following.viz_gate import has_listener
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
    "speed_fallback_mps": 3.0,   # 0 = 정지 (안전측). 정속 실험은 여기에 값
    "speed_scale": 1.0,          # 실차에서 급하게 줄일 때. CSV 재생성 없이 배율
    "speed_max_mps": 0.0,        # 하드 상한. 0 = 제한 없음
    # 앞으로 이 거리만큼의 최소 속도를 취해 제어·측위 지연을 흡수. 0 = 끔.
    # 아래 speed_lookahead_time_sec 를 쓰면 이 값은 "하한" 으로만 쓰인다.
    "speed_lookahead_m": 0.3,
    # 속도 연동 선견거리 [s]. 고정 거리로 두면 안 된다 — 0.3 m 는 3 m/s 에서
    # 100 ms 앞을 보지만 8 m/s 에서는 37 ms 뿐이라 감속 시작이 그만큼 늦는다.
    # 흡수하려는 것은 "거리" 가 아니라 제어·구동·측위의 시간 지연이므로
    # 시간으로 잡고 거리로 환산한다: lookahead = clamp(v·t, 하한, 상한).
    #   3m/s 0.75m, 5m/s 1.25m, 8m/s 2.00m (상한)
    # 웨이포인트별로 CSV 목표속도를 써서 기동 시 한 번만 계산한다 (런타임 0).
    # 0 으로 두면 speed_lookahead_m 고정으로 되돌아간다.
    "speed_lookahead_time_sec": 0.25,
    # 선견거리 상한 [m]. 너무 길면 코너 한참 전부터 감속해 랩타임을 버린다.
    "speed_lookahead_max_m": 2.0,
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
    # ---- 조향 스케일 보정 ----
    # ESP 펌웨어 normToAngle 은 S=±1 을 서보 혼 40°/140°, 즉 중립 90° 기준
    # ±50° 로 보낸다. 이 50° 는 "서보 혼 각도" 지 "전륜 조향각" 이 아니다.
    # 그런데 Stanley 는 atan(L·κ), a_lat=v²tan(δ)/L 같은 자전거 모델 식을 쓴다.
    # 그 식들의 δ 는 전륜 조향각이라, 서보 각도를 그대로 넣으면 링키지 비만큼
    # 어긋난다.
    #
    # 20260816 실측 (요레이트 역산, 조향·속도가 0.4s 간 일정한 준정상 구간
    # 242개 회귀): 실효/명령 = 0.429. 명령 21.8° 일 때 실제 전륜각 8.3°,
    # 랩 전체 최대 전륜각 20.4°. 즉 S=1.0 의 실제 전륜각은 약 21.4° 다.
    # ESP 가 목표 0.5° 이내에서 스냅하므로 정상상태 서보는 명령각에 정확히
    # 가 있다 — 이 비는 스무딩이 아니라 순수 기구비다.
    "max_steering_angle": 0.8726646,  # ±50° (서보 혼). 보정 끄면 이 값을 쓴다
    "max_steering_angle_real_rad": 0.3735,  # ±21.4° 실측 전륜각
    # True 면 max_steering 을 실측 전륜각으로 바꾸고, 튜닝 게인들을 같은 비율
    # (0.3735/0.8727 = 0.428) 로 나눠 **거동을 그대로 재현**한다. 즉 이 플래그
    # 하나만으로는 주행이 달라지지 않는다. 달라지는 것은 단위의 정직함이다:
    #   - atan(L·κ) 가 만드는 FF 가 비로소 "필요량의 몇 배" 로 읽힌다
    #   - _limit_lateral_accel / _accel_hold_cap 의 횡가속 상한이 실제로 걸린다
    #     (지금은 실각도 상한을 2.33배 부풀린 명령과 비교해서 사실상 무효다)
    # True 면 실측 전륜각(±21.4°) 기준으로 Stanley 출력을 보정한다.
    "steer_scale_calibrated": True,
    "steering_smooth_alpha": 0.45,
    "wheelbase": vg.WHEELBASE_M,
    "stanley_k": 1.8,
    "stanley_softening": 0.12,
    # Stanley heading 항 배율. 원래 정의는 1.0 (heading_error 를 그대로 조향에
    # 더한다). 스케일 보정이 켜지면 여기에도 같은 비율이 곱해진다.
    "stanley_heading_gain": 1.0,
    # |cte|가 클수록 heading_error 가중치↓ (직선 평행주행 시 상쇄 방지)
    # 0.08 은 실주행 오차보다 훨씬 작아서(실측 |cte| p50=0.13, p95=0.40)
    # 대부분의 시간을 하한 0.25 에 붙어 달렸다 — 곡선에서 밀릴수록 복구력이
    # 약해지는 역효과. 하한 도달점을 p95 부근(0.375 m)으로 옮긴다.
    "stanley_heading_cte_blend_m": 0.45,
    "stanley_heading_min_weight": 0.15,
    # 위 억제를 **고속에서는 풀어 준다.**
    #
    # Stanley 에서 헤딩항은 오버슈트를 만드는 항이 아니라 막는 항이다.
    # δ = θ_e + atan(k·e/v) 에서 θ_e 가 있어야 오차 동역학이 1차(ė = −k·e)가
    # 되어 라인을 안 넘는다. 그걸 깎으면 2차 무감쇠계가 되어 지나친다.
    # oppose_only_blend 가 켜진 지금, 억제가 걸리는 경우는 정확히 "헤딩항이
    # 복귀를 되받는 중" 뿐이다 — 즉 남은 억제는 감쇠만 골라서 깎고 있다.
    #
    # 폐루프 시뮬(실제 _stanley_control + 접지력 6 m/s², d0=0.5 m 에서 라인
    # 쪽으로 각을 물고 도착):
    #
    #   도착각   v=5      v=6      v=7      | min_w=1.0 일 때 v=7
    #    15°    0.000    0.000    0.000    |  0.000
    #    20°    0.000    0.006    0.102    |  0.000
    #    25°    0.033    0.155    0.336    |  0.263
    #
    # 저속에서는 넘지도 않고, 20260816 실측 튜닝이 2.5~3 m/s 구간에서 나온
    # 값이라 그대로 둔다. 오버슈트가 실제로 생기는 6~7 m/s 에서만 1.0 으로
    # 올린다. 두 속도 사이는 선형보간. lo ≥ hi 로 두면 스케줄이 꺼진다.
    "stanley_heading_weight_speed_lo": 4.0,   # 이 아래는 min_weight 그대로
    "stanley_heading_weight_speed_hi": 6.0,   # 이 위는 억제 없음(가중치 1.0)
    # 위 억제를 "헤딩항이 CTE 복귀와 반대로 밀 때" 로만 한정한다.
    #
    # 억제의 원래 목적은 경로로 되돌아오는 중에 헤딩항이 복귀를 되받아쳐
    # 오버슈트하는 것을 막는 것이다. 그런데 조건이 |cte| 하나뿐이라, 코너에서
    # 밖으로 밀리는 상황 — 헤딩 오차와 횡오차가 같은 방향이라 헤딩항이 복귀를
    # 도와주는 상황 — 에서도 똑같이 깎였다.
    #
    # 20260816 실측: |cte|>0.25 이고 v>1.0 인 175 샘플 중 49% 가 "도와주는
    # 방향" 이었고, 거기서 평균 14.6° (실제 전륜각 6.3°) 를 버리고 있었다.
    # 부호가 같으면 full weight, 반대일 때만 기존대로 억제한다.
    # False 로 두면 이전처럼 |cte| 만 보고 무조건 억제한다.
    "stanley_heading_oppose_only_blend": True,
    # LOCAL_PATH(회피) 전용.
    # heading_gain 은 "경로 헤딩 오차 → 조향" 배율. FGM 각도가 이미 필요한
    # 회피량을 담고 있어서 1.0 을 넘기면 목표점을 지나쳐 과회피가 된다.
    # 목표점 추종(pure pursuit) 등가 게인은 2L/Ld ≈ 0.7 수준.
    "local_path_stanley_k": 0.8,
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
    # 피드백 보정(heading+CTE+가속보정)만 따로 묶는 접지력 예산 (≤0 이면 끔).
    # 위 상한과 마찬가지로 **LOCAL_PATH(회피/복귀) 전용**이다.
    # 이게 없으면 장애물을 지나 경로에서 벌어진 상태로 복귀할 때 헤딩을
    # 되돌리는 조향이 접지력을 넘어 그대로 벽으로 밀린다
    # (7m/s 에서 10° 만 꺾어도 26 m/s² 요구).
    # 4.0 은 max_lateral_accel 보다 낮게 둬서, 복귀 기동이 코너링보다
    # 언제나 완만하도록 한 값이다.
    # CSV 추종에는 걸지 않는다 — 곡선은 FF 가 처리하고, 피드백은 남은 오차를
    # 정확히 지워야 한다. 2° 상한을 씌우면 라인을 못 따라간다.
    "feedback_lateral_accel_mps2": 4.0,
    # 곡률 피드포워드: δ = δ_ff(κ) + Stanley. 직선용 stanley_k 는 유지.
    "enable_steer_ff": True,
    "ff_gain": 2.3,              # δ_ff = ff_gain * ff_sign * atan(L·κ)
    # 속도별 FF 게인 스케줄. ff_gain 하나로 고정하면 저속 코너에서 과조향이
    # 난다 — atan(L·κ) 는 속도와 무관한데 저속에서는 그만큼 꺾을 필요가 없다.
    # 두 배열은 같은 길이여야 하고 속도는 오름차순이어야 한다. 구간 사이는
    # 선형보간, 양 끝 밖은 끝값으로 고정(외삽 안 함 — 저속에서 게인이 음수로
    # 뒤집혀 코너에서 반대로 꺾는 것을 막는다).
    # 끄면(False) 기존처럼 ff_gain 고정값을 쓴다.
    #
    # 여기 숫자는 서보각 단위다. 스케일 보정(steer_scale_calibrated)이 켜지면
    # 0.428 이 곱해져 "실제 전륜각 기준 운동학 배율" 이 된다. 그 눈으로 보면:
    #   서보 2.33 = 실각 1.00 = atan(L·κ) 를 정확히 그만큼 꺾는다(운동학적 정확)
    #   아래 표는 1m/s 0.98 → 3 1.07 → 5 1.20 → 7 1.41 → 10 1.50 배가 된다.
    # 한때 5·7·10 을 3.3/3.5/3.8 로 올려둔 적이 있는데, 그건 control_node 분모가
    # 0.8726 이라 조향이 43% 로 깎이던 걸 FF 로 메우려던 것이었다. 분모를
    # 0.3735 로 되돌린 지금 그대로 두면 코너에서 18% 과조향이 된다.
    # 20260816 랩에서 t=186~188s, κ=-0.13(반경 7.5m), 2.7→3.0 m/s 구간의
    # cte 가 0.11 → 0.59 m 로 벌어졌다. 그때 FF 는 실각 1.3° 였고 운동학적
    # 필요량은 2.46° 였다. 모자란 1.16° 는 횡가속 0.55 m/s² 에 해당해서,
    # 1.7 초 동안 0.8 m 를 밀어낸다 — 관측된 0.48 m 이탈과 자릿수가 맞는다.
    # 그래서 스케줄을 켠다. 저속은 오히려 지금보다 약하게(1m/s: 실각 0.43),
    # 문제 구간인 3~5 m/s 에서 실각 0.98 로 운동학적 정확값에 맞춘다.
    "ff_gain_schedule_enable": True,  # 이전 기본값 False (고정 ff_gain 1.3)
    "ff_gain_speed_bp": [1.0, 3.0, 5.0, 7.0, 10.0],
    "ff_gain_bp": [2.3, 2.5, 3.0, 3.5, 3.8],
    "ff_sign": 1.0,              # 좌우 반대면 -1.0
    "ff_lookahead_m": 0.8,       # best_i 기준 앞쪽 평균 곡률 구간 [m]
    "ff_kappa_clip": 2.5,        # |κ| 상한 [1/m] (스파이크 방지)
    # ---- 가속 구간 라인 홀드 (CSV 추종 전용) ----
    # "감속했다가 갑자기 가속" 하는 구간에서만 라인을 놓치는 문제를 잡는다.
    # 그 구간에서 두 가지가 동시에 일어난다:
    #   1) cte 항 atan(k·e/v) 는 v 에 반비례한다. 속도가 2배가 되면 같은 횡오차를
    #      되돌리는 힘이 절반이 된다. 하필 밖으로 밀리기 시작하는 그 순간이다.
    #   2) 종가속이 접지원(friction circle)의 횡방향 몫을 먹어서, 같은 조향각이
    #      만드는 실제 곡률이 줄어든다 (가속 언더스티어).
    # stanley_k / ff_gain / heading 가중치를 올려서 잡으면 랩 전체 거동이 바뀐다.
    # 그래서 "가속 중" 이라는 조건에서만 켜지는 별도 항으로 분리했다.
    # 정속·감속 구간에서는 u=0 이고 아래 배율이 전부 1.0 이라 기존 거동과 같다.
    # 되돌리려면 accel_hold_enable=False 하나만 끄면 된다.
    "accel_hold_enable": False,
    # 문턱을 절대값으로 박으면 안 된다. speed_scale·v_ref 를 낮춰 프로파일이
    # 통째로 느려지면 종가속도 같이 줄어서 문턱을 영영 못 넘는다. 실제로
    # v[1.06~3.00] 프로파일의 최대 종가속은 0.99 m/s² 라, 아래 절대 문턱
    # 1.0 에서는 랩 전체에서 단 한 번도 켜지지 않았다.
    # 그래서 CSV 프로파일이 실제로 내는 가속(a = v·dv/ds 의 피크)을 기동 시
    # 한 번 계산해서, 그 비율로 문턱을 잡는다. v_ref 를 올리면 문턱도 같이
    # 올라가므로 속도를 바꿔도 재튜닝이 필요 없다.
    "accel_hold_auto_threshold": True,
    "accel_hold_on_frac": 0.30,    # 프로파일 피크 가속의 30% 부터 u>0
    "accel_hold_full_frac": 0.90,  # 90% 에서 u=1
    # auto_threshold=False 일 때만 쓰이는 절대 문턱.
    "accel_hold_on_mps2": 1.0,    # 이 종가속도부터 보정 시작 (u=0)
    "accel_hold_full_mps2": 4.0,  # 여기서 보정 최대 (u=1). on 보다 커야 한다
    # u=1 에서 stanley_k 에 더해지는 배율. 1.0 = 최대 2배 (0.50 → 1.00)
    "accel_hold_cte_boost": 1.0,
    # u=1 에서 곡률 FF 에 더해지는 배율. 0.4 = 최대 1.4배 (1.30 → 1.82).
    # FF 는 κ 에 비례하므로 직선(κ≈0)에서는 이 값이 아무 일도 하지 않는다.
    # 즉 코너 탈출 가속에서만 실제로 먹는다.
    "accel_hold_ff_boost": 0.4,
    # 추가 조향의 절대 상한 [deg]. 큰 횡오차·큰 곡률·낮은 속도가 겹치면 두 항이
    # 같이 커져서, 무제한이면 20° 넘게 들어간다. 추정기가 한 번 튀었을 때
    # 조향이 홱 꺾이는 것도 이 값이 막는다. 주로 저속에서 먹는 상한이다.
    "accel_hold_max_extra_deg": 8.0,  # 이전 6.0 (속도 연동 상한 추가 전)
    # 속도 연동 상한 — 이 보정이 추가로 쓸 수 있는 횡가속 예산 [m/s²].
    # 각도 상한 하나로 고정하면 안 된다. 같은 6° 라도 만들어내는 횡가속은
    # v² 로 커져서, 2 m/s 에서 1.3 m/s² 인 것이 8 m/s 에서는 20.4 m/s² 가 된다.
    # 접지 한계(레이스라인 생성 기준 a_lat=6.0)를 3배 넘겨 명령하는 셈이라,
    # 조향이 곡률로 바뀌지 않고 타이어만 긁는다.
    #   δ_cap = atan(L · 예산 / v²)  →  3m/s 7.3°, 5m/s 2.6°, 8m/s 1.0°
    # 저속에서는 이 값이 커져서 위 절대 상한이 먹고, 고속에서는 v² 때문에
    # 알아서 조여진다. 0 으로 두면 속도 연동을 끄고 절대 상한만 쓴다.
    "accel_hold_max_extra_alat_mps2": 3.5,
    # 실측 속도 미분에 거는 저역통과 계수. 크면 빠르게 반응하지만 ERPM
    # 양자화 노이즈가 그대로 실린다.
    "accel_est_filter_alpha": 0.30,
    # ---- ESP 조향 지연 보상 ----
    # ESP 펌웨어 loop 은 20ms 마다 currentAngle += (target-current)*SMOOTH_FACTOR
    # 를 돌린다. SMOOTH_FACTOR=0.20 은 펌웨어 주석에 "진단 중에는 0.2 정도로
    # 제한" 이라고 적혀 있는 값이고, 시정수 90ms (95% 도달 269ms) 짜리 1차
    # 지연이다. 여기에 젯슨의 steering_smooth_alpha=0.45 (30ms 주기, 시정수
    # 50ms) 가 직렬로 겹쳐 합계 약 140ms 다. 3 m/s 면 0.42 m 를 지나가는 동안
    # 조향이 아직 다 안 들어간다 — 코너 진입과 가속 탈출에서 밖으로 밀리는
    # 직접 원인이다.
    #
    # 펌웨어를 못 건드리므로 젯슨에서 맞춘다. ESP 내부 상태를 같은 식으로
    # 모사해 두고, 그 모델 출력이 원하는 각도가 되도록 명령을 앞질러 보낸다.
    #   c ← c + a_esp·(보낸값 - c)                (모델 갱신)
    #   보낼값 = c + G·(원하는각 - c),  G = a_target / a_esp
    # G>1 이라 명령이 원하는 각보다 크게 나가지만, ESP 가 그만큼 뒤처져 받으므로
    # 서보 실제 각도는 원하는 각을 따라간다. 모델이 틀어져도 극점이 0.8 이라
    # 오차가 스스로 감쇠한다.
    #
    # 보상을 켜면 젯슨 자체 1차 필터(steering_smooth_alpha)는 쓰지 않는다.
    # 지연을 지우려는 마당에 앞단에서 다시 지연을 넣을 이유가 없다.
    # steering_rate_limit_radps 는 그대로 살아 있어서 급변은 여전히 막힌다.
    # False 로 두면 기존 경로(_smooth_steering)로 완전히 되돌아간다.
    "esp_lag_compensation_enable": True,
    "esp_smooth_factor": 0.20,      # 펌웨어 SMOOTH_FACTOR 와 같아야 한다
    "esp_loop_period_sec": 0.020,   # 펌웨어 loop 의 delay(20)
    # 보상 후 목표 시정수 [s]. ESP 실제 시정수(90ms)보다 작게 잡는다.
    # 작을수록 선행량 G 가 커지고 노이즈도 같이 증폭되므로 아래 상한으로 막는다.
    "esp_lag_target_tau_sec": 0.035,
    # 선행 배율 G 의 상한. 완전 역보상(G≈3.5)은 명령을 3배 넘게 부풀려
    # 서보가 떨린다. 2.5 면 시정수 90ms → 약 33ms 로 줄면서도 과도하지 않다.
    "esp_lag_max_lead_gain": 2.5,
    # ESP 는 |target-current| ≤ 0.5° (서보각) 면 current 를 target 으로 스냅한다.
    # 모델에도 같은 스냅을 넣어야 정상상태에서 모델이 실제와 어긋나지 않는다.
    # 서보 풀스케일 50° 기준의 비율로 둬서 스케일 보정과 무관하게 맞는다.
    "esp_snap_frac_of_full": 0.01,  # 0.5° / 50°
    "stanley_debug_log_hz": 2.0,
    "status_log_hz": 2.0,
    "planner_gate_stale_sec": 0.15,  # override False 미수신 시 빠르게 CSV 복귀
    "steering_rate_limit_radps": 6.5,
    # Foxglove/모니터용 시각화·디버그 토픽
    "publish_tracked_path": True,
    "publish_stanley_debug_topic": True,
    "publish_control_diagnostics": True,
}


# 종가속 추정 가드. VESC ERPM 기반 속도라 샘플 간격이 너무 짧으면 미분에
# 양자화 노이즈가 그대로 실리고, 너무 길면(토픽이 끊겼다 돌아온 경우) 그
# 구간의 평균 미분은 지금 상태와 무관하다.
_ACCEL_EST_MIN_DT_SEC = 0.004
_ACCEL_EST_MAX_DT_SEC = 0.20
_ACCEL_EST_CLIP_MPS2 = 20.0

# 프로파일 피크 가속이 이보다 작으면 사실상 정속 주행이라 가속 보정할 게 없다.
_ACCEL_HOLD_MIN_PROFILE_ACCEL = 0.15
# 프로파일 종가속을 다듬는 이동평균 창 [m]. 점 하나의 튐으로 문턱이
# 좌우되지 않게 한다.
_PROFILE_ACCEL_SMOOTH_M = 0.30


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

        servo_full = float(self.get_parameter("max_steering_angle").value)
        self.steer_scale_calibrated = param_bool(
            self.get_parameter("steer_scale_calibrated").value
        )
        if self.steer_scale_calibrated:
            self.max_steering = max(
                1e-3, float(self.get_parameter("max_steering_angle_real_rad").value)
            )
        else:
            self.max_steering = servo_full
        # 서보각 단위로 튜닝된 게인들을 실전륜각 단위로 옮기는 배율.
        # 보정을 끄면 1.0 이라 아래 곱셈들이 전부 무해해진다.
        self._steer_gain_rebase = (
            self.max_steering / servo_full if servo_full > 1e-9 else 1.0
        )
        self.steering_smooth_alpha = float(
            self.get_parameter("steering_smooth_alpha").value
        )
        self.wheelbase = max(1e-3, float(self.get_parameter("wheelbase").value))

        g = self._steer_gain_rebase
        # atan 안에 들어가는 k 는 엄밀히는 선형이 아니다. 다만 실주행 cte_term
        # 은 대부분 10° 이내(atan 인자 0.2 이하)라 그 구간에서는 선형과 5%
        # 이내로 일치한다.
        self.stanley_k = float(self.get_parameter("stanley_k").value) * g
        self.stanley_heading_gain = (
            float(self.get_parameter("stanley_heading_gain").value) * g
        )
        self.stanley_softening = float(self.get_parameter("stanley_softening").value)
        self.stanley_heading_cte_blend_m = max(
            1e-3, float(self.get_parameter("stanley_heading_cte_blend_m").value)
        )
        self.stanley_heading_min_weight = max(
            0.0,
            min(1.0, float(self.get_parameter("stanley_heading_min_weight").value)),
        )
        self.stanley_heading_oppose_only_blend = param_bool(
            self.get_parameter("stanley_heading_oppose_only_blend").value
        )
        self.stanley_heading_weight_speed_lo = max(
            0.0, float(self.get_parameter("stanley_heading_weight_speed_lo").value)
        )
        self.stanley_heading_weight_speed_hi = max(
            0.0, float(self.get_parameter("stanley_heading_weight_speed_hi").value)
        )
        self.local_path_stanley_k = (
            max(0.05, float(self.get_parameter("local_path_stanley_k").value)) * g
        )
        self.local_path_heading_gain = (
            max(0.1, float(self.get_parameter("local_path_heading_gain").value)) * g
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
        self.local_path_steering_rate_limit_radps = (
            max(
                0.1,
                float(self.get_parameter("local_path_steering_rate_limit_radps").value),
            )
            * g
        )
        # 횡가속 한계는 물리량이라 재환산하지 않는다. 스케일 보정이 켜지면
        # 이 한계에서 나오는 δ 상한이 비로소 같은 단위의 명령과 비교된다.
        self.max_lateral_accel_mps2 = float(
            self.get_parameter("max_lateral_accel_mps2").value
        )
        self.feedback_lateral_accel_mps2 = float(
            self.get_parameter("feedback_lateral_accel_mps2").value
        )
        self.enable_steer_ff = param_bool(self.get_parameter("enable_steer_ff").value)
        self.ff_gain = float(self.get_parameter("ff_gain").value) * g
        self._init_ff_gain_schedule()
        self.ff_sign = float(self.get_parameter("ff_sign").value)
        self.ff_lookahead_m = max(0.0, float(self.get_parameter("ff_lookahead_m").value))
        self.ff_kappa_clip = max(0.0, float(self.get_parameter("ff_kappa_clip").value))

        self.gate_stale_ns = int(
            float(self.get_parameter("planner_gate_stale_sec").value) * 1e9
        )
        self.steering_rate_limit_radps = (
            float(self.get_parameter("steering_rate_limit_radps").value) * g
        )
        self._init_esp_lag_comp()

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
        # 프로파일이 확정된 뒤에 불러야 한다 — 가속 문턱을 그 프로파일이 실제로
        # 내는 종가속에서 뽑기 때문이다.
        self._init_accel_hold()

        self._local_path: List[Tuple[float, float]] = []
        self._path_poses: List[Tuple[float, float]] = []
        # tracked_path 재사용 버퍼 — `_publish_tracked_path` 참고
        self._tracked_path_msg = Path()
        self._tracked_pose_pool: List[PoseStamped] = []

        self._planner_override_active = False
        # 기본값은 False = FGM 폴백처럼 곡률이 의미 없는 경로로 가정. 계획
        # 기동이 실제로 들어올 때만 켜진다.
        self._local_path_planned = False
        self._planner_gate_recv_ns = 0

        self._measured_speed_mps = 0.0
        self._filtered_speed_mps = 0.0
        self._measured_speed_recv_ns = 0
        self._measured_speed_initialized = False

        # 종가속 추정 상태. 미분은 필터 전 원신호로 잡고 결과에만 저역통과를
        # 건다 — 이미 필터된 속도를 또 미분하면 지연이 두 번 쌓인다.
        self._accel_mps2 = 0.0
        self._accel_prev_speed = 0.0
        self._accel_prev_ns = 0
        self._accel_hold_u = 0.0
        self._last_accel_extra = 0.0

        # control_node /vehicle/telemetry snapshot
        self._ctrl_target_speed_mps = 0.0
        self._ctrl_measured_speed_mps = 0.0
        self._ctrl_vesc_duty = 0.0
        self._ctrl_auto = False
        self._telemetry_recv_ns = 0

        self._last_steering_cmd = 0.0
        self._wrong_way_frames = 0
        self._wrong_way_warned = False
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
            Bool, "/planner/local_path_planned", self._cb_local_path_planned, 10
        )
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

        # 발행 순서 = _publish_control_diagnostics 가 넘기는 값 순서
        self._diag_pubs = (
            self.raw_steer_cmd_pub,
            self.filtered_steer_cmd_pub,
            self.cte_pub,
            self.heading_error_pub,
            self.path_curvature_pub,
        )

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
            f"cap={self._speed_max_mps if self._speed_max_mps > 0 else 'none'} "
            f"lookahead={self._speed_lookahead_desc}, "
            f"measured_speed={self.measured_speed_topic}, "
            f"telemetry={self.telemetry_topic}, "
            f"steer_scale={'실전륜각' if self.steer_scale_calibrated else '서보각'} "
            f"max={math.degrees(self.max_steering):.1f}deg "
            f"(게인 재환산 ×{self._steer_gain_rebase:.3f}), "
            f"esp_lag={self._esp_lag_desc}, "
            f"stanley_k={self.stanley_k:.3f}, soft={self.stanley_softening}, "
            f"hdg_gain={self.stanley_heading_gain:.3f} "
            f"blend={self.stanley_heading_cte_blend_m:.2f}m"
            f"{'(반대방향만)' if self.stanley_heading_oppose_only_blend else ''}, "
            f"steer_ff={self.enable_steer_ff} gain={self._ff_gain_desc()} "
            f"sign={self.ff_sign:.1f} lookahead={self.ff_lookahead_m:.2f}m "
            f"kappa_clip={self.ff_kappa_clip:.2f}, "
            f"accel_hold={self._accel_hold_desc()}, "
            f"steering_rate_limit={self.steering_rate_limit_radps:.2f} rad/s"
        )

    def _cb_local_path(self, msg: Path) -> None:
        self._local_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _cb_planner_gate(self, msg: Bool) -> None:
        self._planner_override_active = bool(msg.data)
        self._planner_gate_recv_ns = self.get_clock().now().nanoseconds

    def _cb_local_path_planned(self, msg: Bool) -> None:
        """/local_path 가 계획된 기하 경로인가 (FF 를 켤지 여부)."""
        self._local_path_planned = bool(msg.data)

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

        now_ns = self.get_clock().now().nanoseconds
        self._update_accel_estimate(speed, now_ns)

        self._measured_speed_mps = speed
        self._measured_speed_recv_ns = now_ns

    def _init_accel_hold(self) -> None:
        """가속 구간 라인 홀드 파라미터 검증. 이상하면 꺼서 기존 거동을 남긴다."""
        self.accel_est_alpha = max(
            0.0, min(1.0, float(self.get_parameter("accel_est_filter_alpha").value))
        )
        self.accel_hold_on_mps2 = float(self.get_parameter("accel_hold_on_mps2").value)
        self.accel_hold_full_mps2 = float(
            self.get_parameter("accel_hold_full_mps2").value
        )
        self._accel_hold_threshold_src = "고정"
        if param_bool(self.get_parameter("accel_hold_auto_threshold").value):
            ref = self._profile_accel_ref
            if ref >= _ACCEL_HOLD_MIN_PROFILE_ACCEL:
                on_f = float(self.get_parameter("accel_hold_on_frac").value)
                full_f = float(self.get_parameter("accel_hold_full_frac").value)
                self.accel_hold_on_mps2 = ref * on_f
                self.accel_hold_full_mps2 = ref * full_f
                self._accel_hold_threshold_src = f"auto(피크 {ref:.2f})"
            else:
                # 정속에 가까운 프로파일이면 보정할 가속 자체가 없다. 문턱을
                # 0 근처로 내리면 노이즈에 상시 켜지므로 그냥 끈다.
                self._accel_hold_threshold_src = f"auto→off(피크 {ref:.2f} too small)"
        self.accel_hold_cte_boost = max(
            0.0, float(self.get_parameter("accel_hold_cte_boost").value)
        )
        self.accel_hold_ff_boost = max(
            0.0, float(self.get_parameter("accel_hold_ff_boost").value)
        )
        # 이 상한도 조향각 단위라 재환산 대상이다. 아래 횡가속 예산 상한은
        # 물리량이라 건드리지 않는다 — 스케일 보정이 켜져야 비로소 둘이 같은
        # 단위로 비교된다.
        self.accel_hold_max_extra_rad = (
            math.radians(
                max(0.0, float(self.get_parameter("accel_hold_max_extra_deg").value))
            )
            * self._steer_gain_rebase
        )
        self.accel_hold_max_extra_alat = max(
            0.0,
            float(self.get_parameter("accel_hold_max_extra_alat_mps2").value),
        )
        self.accel_hold_enable = param_bool(
            self.get_parameter("accel_hold_enable").value
        )
        if not self.accel_hold_enable:
            return
        if self.accel_hold_full_mps2 <= self.accel_hold_on_mps2:
            self.accel_hold_enable = False
            self.get_logger().warn(
                f"accel_hold 무시 ({self._accel_hold_threshold_src}: "
                f"full={self.accel_hold_full_mps2:.2f} <= on="
                f"{self.accel_hold_on_mps2:.2f}) — 기존 거동 사용"
            )

    def _accel_hold_desc(self) -> str:
        if not self.accel_hold_enable:
            return "off"
        return (
            f"{self.accel_hold_on_mps2:.2f}~{self.accel_hold_full_mps2:.2f}m/s2"
            f"[{self._accel_hold_threshold_src}] "
            f"k×≤{1.0 + self.accel_hold_cte_boost:.2f} "
            f"ff×≤{1.0 + self.accel_hold_ff_boost:.2f} "
            f"cap≤{math.degrees(self.accel_hold_max_extra_rad):.1f}deg"
            + (
                f"&{self.accel_hold_max_extra_alat:.1f}m/s2"
                f"(3m/s {math.degrees(self._accel_hold_cap(3.0)):.1f}deg, "
                f"8m/s {math.degrees(self._accel_hold_cap(8.0)):.1f}deg)"
                if self.accel_hold_max_extra_alat > 0.0
                else "(속도연동 off)"
            )
        )

    def _update_accel_estimate(self, speed: float, now_ns: int) -> None:
        """실측 속력의 종가속 [m/s²]. 저역통과 한 단만 거친다."""
        prev_ns = self._accel_prev_ns
        prev_speed = self._accel_prev_speed
        if prev_ns <= 0:
            self._accel_prev_ns = now_ns
            self._accel_prev_speed = speed
            return

        dt = (now_ns - prev_ns) * 1e-9
        if dt < _ACCEL_EST_MIN_DT_SEC:
            # 간격이 너무 짧다. prev 를 그대로 두고 다음 샘플에서 더 긴
            # 구간으로 본다 (여기서 갱신하면 영원히 짧은 dt 만 보게 된다).
            return

        self._accel_prev_ns = now_ns
        self._accel_prev_speed = speed
        if dt > _ACCEL_EST_MAX_DT_SEC:
            self._accel_mps2 = 0.0
            return

        a_raw = (speed - prev_speed) / dt
        a_raw = max(-_ACCEL_EST_CLIP_MPS2, min(_ACCEL_EST_CLIP_MPS2, a_raw))
        self._accel_mps2 += self.accel_est_alpha * (a_raw - self._accel_mps2)

    def _accel_hold_term(
        self, u: float, cte: float, speed: float, ff_term: float
    ) -> float:
        """가속 중에만 붙는 추가 조향 [rad]. u=0 이면 정확히 0.

        두 가지를 되돌려 놓는다:
          - cte 항 atan(k·e/v) 가 속도에 반비례해 잃어버린 복귀력
          - 종가속이 접지원의 횡방향 몫을 먹어 생기는 가속 언더스티어(FF 증분)
        마지막에 상한을 걸어서, 추정기가 튀거나 큰 오차·큰 곡률이 겹쳐도
        조향이 한 번에 꺾이지 않게 한다.
        """
        if u <= 0.0:
            return 0.0

        denom = abs(speed) + self.stanley_softening
        cte_base = math.atan2(self.stanley_k * cte, denom)
        cte_boosted = math.atan2(
            self.stanley_k * (1.0 + u * self.accel_hold_cte_boost) * cte, denom
        )
        # ff_term 은 게인에 선형이라 배율만 곱하면 증분이 나온다.
        extra = (cte_boosted - cte_base) + ff_term * (
            u * self.accel_hold_ff_boost
        )
        cap = self._accel_hold_cap(speed)
        return max(-cap, min(cap, extra))

    def _accel_hold_cap(self, speed: float) -> float:
        """추가 조향 상한 [rad]. 속도가 오를수록 자동으로 조인다.

        각도 상한만 두면 안 된다 — 같은 조향각이 요구하는 횡가속은 v² 로
        커지기 때문이다. 그래서 "이 보정이 추가로 쓸 수 있는 횡가속" 예산을
        각도로 되돌려 상한으로 쓴다: a_lat = v²·tan(δ)/L 을 δ 로 푼 것.
        저속에서는 이 값이 절대 상한보다 커서 절대 상한이 먹는다.
        """
        hard = self.accel_hold_max_extra_rad
        budget = self.accel_hold_max_extra_alat
        v = abs(speed)
        # v 가 0 에 가까우면 예산 상한이 발산한다. 그 구간은 어차피 어떤
        # 조향각도 횡가속을 못 만들어서 절대 상한만으로 충분하다.
        if budget <= 0.0 or v < 0.5:
            return hard
        return min(hard, math.atan(self.wheelbase * budget / (v * v)))

    def _accel_hold_intensity(self, measured_speed_alive: bool) -> float:
        """가속 보정 강도 u ∈ [0,1]. on 에서 0, full 에서 1, 그 사이 선형."""
        if not self.accel_hold_enable or not measured_speed_alive:
            return 0.0
        excess = self._accel_mps2 - self.accel_hold_on_mps2
        if excess <= 0.0:
            return 0.0
        span = self.accel_hold_full_mps2 - self.accel_hold_on_mps2
        return min(1.0, excess / span)

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

        # LOCAL_PATH(회피)는 이미 cte 분모 속도를 local_path_cte_speed_cap_mps
        # 로 붙들어 복귀력을 확보해 둔 상태라 가속 보정을 겹쳐 걸지 않는다.
        accel_u = 0.0 if mode == "LOCAL_PATH" else self._accel_hold_u

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
        self._warn_if_wrong_way(heading_error, speed)

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
            # 조향 부호 (실차 ESP 서보 실측과 동일, 추가 반전 없음):
            #   +steering = 좌, -steering = 우
            # cte>0(경로 오른쪽) → 좌회전(+)으로 복귀
            heading_raw = self.stanley_heading_gain * heading_error
            if (
                self.stanley_heading_oppose_only_blend
                and heading_raw * cte_term > 0.0
            ):
                # 헤딩항이 CTE 복귀와 같은 방향이다. 코너에서 밖으로 밀릴 때가
                # 여기다 — 둘 다 안쪽으로 밀고 있으니 억제할 이유가 없다.
                hdg_w = 1.0
            else:
                hdg_w = max(
                    self._heading_min_weight_at(speed),
                    1.0 - abs(cte) / self.stanley_heading_cte_blend_m,
                )
            heading_term = hdg_w * heading_raw

        kappa_used = 0.0
        ff_term = 0.0
        # LOCAL_PATH 에서도 경로가 **계획된 기하**면 FF 를 쓴다.
        #
        # FF 를 껐던 이유는 FGM 폴백 경로 때문이다. 그건 조준점까지 그은
        # 직선이라 곡률이 목표점 흔들림에서 나오는 잡음이고, 그걸 FF 로
        # 증폭하면 조향이 떤다.
        #
        # 횡오프셋 기동은 반대다. 그 곡률(6 m/s 에서 0.08 1/m ≈ 1.6°)은 우리가
        # 타기로 **계획한** 값이고, FF 로 안 내보내면 오차가 쌓인 뒤 피드백이
        # 뒤늦게 만들어야 한다. 그런데 그 피드백에는 횡가속 상한이 걸려 있어
        # (6 m/s 에서 2.1°, 7 m/s 에서 1.5°) 기하를 내는 데만 예산을 다 쓰고
        # 정작 오차를 지울 여지가 안 남는다. 계획대로 못 따라가는 것이다.
        #
        # 이렇게 나누면 역할이 제자리를 찾는다: FF 가 계획된 곡률을 내고,
        # 피드백 상한은 순수하게 **오차 보정** 크기를 묶는다.
        ff_ok = mode != "LOCAL_PATH" or self._local_path_planned
        if self.enable_steer_ff and ff_ok:
            kappa_used = self._lookahead_curvature(path, best_i)
            # 자전거 모델: δ_ff = atan(L·κ). +κ(좌로 휨) → +조향(좌).
            ff_term = (
                self._ff_gain_at(speed)
                * self.ff_sign
                * math.atan(self.wheelbase * kappa_used)
            )

        # 가속 보정은 위 세 항을 건드리지 않고 뒤에 더하는 가산항으로 둔다.
        # u=0 이면 정확히 0 이라 기존 거동이 그대로 남고, 상한이 하나 걸려
        # 있어서 추정기가 튀어도 조향이 갑자기 꺾이지 않는다.
        accel_extra = self._accel_hold_term(
            accel_u, cte, speed, ff_term
        )

        # 피드백 보정을 접지력 예산으로 묶는다 — 단 LOCAL_PATH(회피/복귀)에서만.
        #
        # 이 상한은 v² 에 반비례해서 6 m/s 면 2.1°, 7 m/s 면 1.5° 까지 조인다.
        # 회피/복귀에서는 그게 목적이다: 장애물 옆을 지나느라 경로에서 벌어진
        # 상태로 헤딩을 되돌릴 때 조향을 크게 넣으면 벽으로 간다. 조향 대신
        # 미리 피하거나 속도를 줄이는 쪽이 맞다.
        #
        # 반대로 깨끗한 트랙(CSV_TRACKING)에 걸면 안 된다. 곡선은 FF 가 이미
        # 처리하고 있고, 피드백은 남은 CTE/헤딩 오차를 정확히 지우는 역할이다.
        # 거기에 2° 상한을 씌우면 경로를 못 따라간다. 한때 CSV 에도 걸려
        # 있었는데, 요청받은 건 회피/복귀 한정이었다.
        #
        # 세 항을 함께 비례 축소해야 진단 토픽의 합이 실제 명령과 맞는다.
        fb_term = heading_term + cte_term + accel_extra
        fb_limit = (
            self._steering_for_lateral_accel(self.feedback_lateral_accel_mps2, speed)
            if mode == "LOCAL_PATH" and self.feedback_lateral_accel_mps2 > 0.0
            else None
        )
        if fb_limit is not None and abs(fb_term) > fb_limit:
            shrink = fb_limit / abs(fb_term)
            heading_term *= shrink
            cte_term *= shrink
            accel_extra *= shrink
            fb_term = math.copysign(fb_limit, fb_term)

        self._last_accel_extra = accel_extra
        steering = ff_term + fb_term

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
            f"v_ctrl={control_speed:.2f} speed_source={speed_source} "
            f"a_x={self._accel_mps2:+.2f}m/s2 accel_u={self._accel_hold_u:.2f} "
            f"accel_add={math.degrees(self._last_accel_extra):+.1f}deg"
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

    # 경로를 역방향으로 타면 헤딩오차가 180° 근처에 눌러앉고 Stanley 는 즉시
    # 풀락을 때린다. 원인은 대개 CSV 재생성 시 폐루프 감김방향이 뒤집힌 것이라
    # 게인을 아무리 만져도 낫지 않는다. 증상 대신 원인을 바로 지목한다.
    # 차를 들어 옮기거나 돌려놓는 동안에도 오차는 180° 가 되므로 주행 중일
    # 때만 센다. 안 그러면 셋업할 때마다 에러가 떠서 무시하게 된다.
    _WRONG_WAY_RAD = math.radians(120.0)
    _WRONG_WAY_FRAMES = 25
    _WRONG_WAY_MIN_MPS = 1.0

    def _warn_if_wrong_way(self, heading_error: float, speed: float) -> None:
        if abs(speed) < self._WRONG_WAY_MIN_MPS:
            return
        if abs(heading_error) < self._WRONG_WAY_RAD:
            self._wrong_way_frames = 0
            self._wrong_way_warned = False
            return
        self._wrong_way_frames += 1
        if self._wrong_way_frames < self._WRONG_WAY_FRAMES or self._wrong_way_warned:
            return
        self._wrong_way_warned = True
        self.get_logger().error(
            f"경로를 역방향으로 타고 있다: 헤딩오차 "
            f"{math.degrees(heading_error):+.0f}° 가 "
            f"{self._WRONG_WAY_FRAMES} 프레임 지속. "
            f"raceline/centerline CSV 의 감김방향이 뒤집혔거나 차가 반대로 "
            f"출발했다. 게인 문제가 아니다."
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

    def _init_esp_lag_comp(self) -> None:
        """ESP 1차지연 보상기 설정. 이상하면 꺼서 기존 경로를 남긴다."""
        self._esp_lag_state = 0.0
        self._esp_lag_enable = param_bool(
            self.get_parameter("esp_lag_compensation_enable").value
        )
        self._esp_lag_desc = "off"
        if not self._esp_lag_enable:
            return

        a_esp = float(self.get_parameter("esp_smooth_factor").value)
        dt_esp = float(self.get_parameter("esp_loop_period_sec").value)
        tau_target = float(self.get_parameter("esp_lag_target_tau_sec").value)
        max_lead = float(self.get_parameter("esp_lag_max_lead_gain").value)
        if not (0.0 < a_esp < 1.0) or dt_esp <= 0.0 or tau_target <= 0.0:
            self._esp_lag_enable = False
            self.get_logger().warn(
                "esp_lag 보상 무시 (smooth_factor/loop_period/target_tau 값 이상) "
                "— 기존 스무딩 사용"
            )
            return

        # 펌웨어의 이산 필터를 연속 시정수로 환산한 뒤, 우리 주기로 다시 이산화.
        tau_esp = -dt_esp / math.log(1.0 - a_esp)
        self._esp_a_here = 1.0 - math.exp(-self.timer_period / tau_esp)
        a_target = 1.0 - math.exp(-self.timer_period / tau_target)
        self._esp_lead_gain = min(max_lead, a_target / self._esp_a_here)
        # 실제로 달성되는 시정수. 상한에 걸리면 목표보다 느려진다.
        a_real = min(1.0, self._esp_a_here * self._esp_lead_gain)
        tau_real = (
            -self.timer_period / math.log(1.0 - a_real) if a_real < 1.0 else 0.0
        )
        self._esp_snap = (
            max(0.0, float(self.get_parameter("esp_snap_frac_of_full").value))
            * self.max_steering
        )
        self._esp_lag_desc = (
            f"lead×{self._esp_lead_gain:.2f} "
            f"tau {tau_esp * 1e3:.0f}→{tau_real * 1e3:.0f}ms"
        )

    def _esp_lag_compensate(self, desired: float) -> float:
        """ESP 가 desired 에 빨리 도달하도록 앞질러 보낼 명령 [rad]."""
        c = self._esp_lag_state
        err = desired - c
        # 스냅 임계 안에서는 선행을 걸면 안 된다. ESP 가 이 오차를 한 스텝에
        # 없애버리므로, 목표를 넘겨 보낸 명령이 그대로 서보에 실리고 다음 주기에
        # 반대로 넘긴다. 정상상태에서 ±0.15° 짜리 리밋사이클이 남아 서보가 떤다.
        # 이 폭은 어차피 서보 명령 분해능(1° 서보 = 0.43° 실각)보다 작다.
        if abs(err) <= self._esp_snap:
            return desired
        return c + self._esp_lead_gain * err

    def _esp_lag_update(self, sent: float) -> None:
        """실제로 내보낸 명령으로 ESP 내부 상태 모델을 굴린다.

        레이트 제한과 포화를 거친 뒤의 값을 넣어야 모델이 실제와 어긋나지
        않는다. 모델 극점이 1보다 작아서 초기 오차나 외부 개입(AEB 등)으로
        어긋나도 스스로 감쇠한다.
        """
        c = self._esp_lag_state
        c += self._esp_a_here * (sent - c)
        # 펌웨어의 0.5° 스냅. 이게 없으면 모델만 영원히 목표에 못 닿아
        # 정상상태에서도 선행량이 남는다.
        if abs(sent - c) <= self._esp_snap:
            c = sent
        self._esp_lag_state = c

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

    def _steering_for_lateral_accel(self, a_max: float, speed: float) -> float | None:
        """a_lat = v²·tan(δ)/L = a_max 가 되는 조향각. 제한이 무의미하면 None.

        저속에서는 상한이 max_steering 보다 커서 어차피 걸리지 않는다.
        """
        v = abs(speed)
        if a_max <= 0.0 or v < 0.5:
            return None
        return math.atan(self.wheelbase * a_max / (v * v))

    def _limit_lateral_accel(self, steering: float, speed: float) -> float:
        """LOCAL_PATH 전용 총조향 상한 (이 모드에선 FF 가 꺼져 있어 전부 피드백)."""
        delta_max = self._steering_for_lateral_accel(
            self.max_lateral_accel_mps2, speed
        )
        if delta_max is None:
            return steering
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

    def _init_ff_gain_schedule(self) -> None:
        """속도별 FF 게인 표를 검증해 구간 기울기까지 미리 계산해 둔다.

        매 주기 도는 경로라 런타임에서는 비교 몇 번과 곱셈 하나만 하도록,
        기울기를 여기서 한 번만 구해 놓는다.
        """
        self._ff_gain_sched: Tuple[List[float], List[float], List[float]] | None = None
        if not param_bool(self.get_parameter("ff_gain_schedule_enable").value):
            return

        def _fail(why: str) -> None:
            self.get_logger().warn(
                f"ff_gain 스케줄 무시 ({why}) — ff_gain={self.ff_gain:.2f} 고정 사용"
            )

        vs = [float(v) for v in self.get_parameter("ff_gain_speed_bp").value]
        gs = [float(g) for g in self.get_parameter("ff_gain_bp").value]
        if len(vs) != len(gs):
            _fail("speed_bp 와 gain_bp 길이가 다름")
            return
        if len(vs) < 2:
            _fail("점이 2개 미만")
            return
        if any(vs[i] <= vs[i - 1] for i in range(1, len(vs))):
            _fail("speed_bp 가 오름차순이 아님")
            return

        # 표도 ff_gain 과 같은 서보각 단위라 같은 배율로 옮긴다.
        gs = [g * self._steer_gain_rebase for g in gs]
        slopes = [
            (gs[i] - gs[i - 1]) / (vs[i] - vs[i - 1]) for i in range(1, len(vs))
        ]
        self._ff_gain_sched = (vs, gs, slopes)

    def _ff_gain_desc(self) -> str:
        if self._ff_gain_sched is None:
            return f"{self.ff_gain:.2f}(고정)"
        vs, gs, _ = self._ff_gain_sched
        pts = ", ".join(f"{v:.1f}m/s->{g:.2f}" for v, g in zip(vs, gs))
        return f"[{pts}]"

    def _ff_gain_at(self, speed: float) -> float:
        """현재 속도에 해당하는 FF 게인. 구간 밖은 끝값으로 고정."""
        sched = self._ff_gain_sched
        if sched is None:
            return self.ff_gain

        vs, gs, slopes = sched
        v = abs(speed)
        if v <= vs[0]:
            return gs[0]
        for i in range(1, len(vs)):
            if v <= vs[i]:
                return gs[i - 1] + slopes[i - 1] * (v - vs[i - 1])
        return gs[-1]

    def _heading_min_weight_at(self, speed: float) -> float:
        """헤딩항 억제의 하한. 고속으로 갈수록 1.0 (억제 없음) 에 붙는다.

        헤딩항은 복귀 감쇠다. 저속에서는 깎아도 안 넘지만, 고속에서는 그게
        그대로 라인 넘김이 된다. 근거 수치는 CFG 주석 참고.
        """
        lo = self.stanley_heading_weight_speed_lo
        hi = self.stanley_heading_weight_speed_hi
        base = self.stanley_heading_min_weight
        if hi <= lo:
            return base
        v = abs(speed)
        if v <= lo:
            return base
        if v >= hi:
            return 1.0
        return base + (1.0 - base) * (v - lo) / (hi - lo)

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
        for pub, value in zip(
            self._diag_pubs, (raw_norm, filtered_norm, cte, heading_err, kappa_used)
        ):
            if not has_listener(pub):
                continue
            msg.data = float(value)
            pub.publish(msg)

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
          16 veh_x [m], 17 veh_y [m], 18 veh_yaw [rad],
          19 longitudinal accel estimate [m/s²], 20 accel-hold intensity u [-],
          21 accel-hold extra steering [rad].

        19 번 이후는 뒤에 덧붙인 항목이다. 소비자(csv_logger_node)는 앞에서부터
        필요한 만큼만 잘라 쓰므로 기존 구독자를 깨지 않는다.
        """
        if not has_listener(self.stanley_debug_pub):
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
            float(self._accel_mps2),
            float(self._accel_hold_u),
            float(self._last_accel_extra),
        ]
        self.stanley_debug_pub.publish(msg)

    # ------------------------------------------------------------
    # 목표 속도 (CSV 3번째 열)
    # ------------------------------------------------------------
    def _setup_speed_profile(self, csv_points, csv_speeds) -> None:
        """웨이포인트별 목표 속도 배열을 만든다.

        CSV v 열(오프라인에서 곡률·가감속 한계로 계산됨)을 그대로 쓰되,
        앞쪽 선견구간의 최솟값을 취해 제어·측위 지연을 흡수한다.
        (감속은 조금 일찍, 가속은 늦게 → 안전측)

        선견구간은 웨이포인트마다 그 지점의 목표속도로 정한다. 여기서 한 번만
        계산해 두므로 주행 중 추가 연산은 없다.
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
        lookahead_t = max(
            0.0, float(self.get_parameter("speed_lookahead_time_sec").value)
        )
        lookahead_max = max(
            lookahead_m, float(self.get_parameter("speed_lookahead_max_m").value)
        )

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

        self._speed_lookahead_desc = "off"
        if lookahead_m > 0.0 and n >= 2 and self._speed_source == "csv":
            if lookahead_t > 0.0:
                # 웨이포인트별 선견거리 = clamp(그 지점 목표속도 × 시간, 하한, 상한).
                # _target_speed_at 과 같은 배율·상한을 먹여야 실제 주행속도
                # 기준이 된다 (전략 배율은 감속만 하므로 여기서는 무시 = 안전측).
                spans = [
                    min(
                        lookahead_max,
                        max(lookahead_m, self._effective_speed(v) * lookahead_t),
                    )
                    for v in profile
                ]
                self._speed_lookahead_desc = (
                    f"{lookahead_t:.2f}s [{min(spans):.2f}~{max(spans):.2f}]m"
                )
                profile = self._min_over_lookahead(csv_points, profile, spans)
            else:
                self._speed_lookahead_desc = f"{lookahead_m:.2f}m(고정)"
                profile = self._min_over_lookahead(csv_points, profile, lookahead_m)

        self._speed_profile = profile
        # 가속 문턱을 뽑을 기준값. 실제 명령될 속도(배율·상한 적용 후) 기준.
        self._profile_accel_ref = self._profile_peak_accel(
            csv_points, [self._effective_speed(v) for v in profile]
        )
        # 전략(회피·추월) 배율. local_planner 가 /planner/speed_scale 로 발행.
        self._strategy_speed_scale = 1.0
        scale_topic = str(self.get_parameter("speed_scale_topic").value).strip()
        if scale_topic:
            self.create_subscription(
                Float64, scale_topic, self._speed_scale_cb, 10
            )

    def _effective_speed(self, v: float) -> float:
        """CSV 속도에 정적 배율·상한만 먹인 값.

        전략 배율(_strategy_speed_scale)은 회피 때만 내려가는 동적 값이라
        여기서는 빼고 본다 (빼는 쪽이 안전측이다).
        """
        out = v * self._speed_scale_cfg
        if self._speed_max_mps > 0.0:
            out = min(out, self._speed_max_mps)
        return max(0.0, out)

    @staticmethod
    def _profile_peak_accel(points, speeds) -> float:
        """속도 프로파일이 의도하는 종가속의 피크 [m/s²].

        a = v·dv/ds 를 웨이포인트마다 구한 뒤 짧은 이동평균으로 다듬고
        최댓값을 취한다. 가속 보정 문턱을 이 값의 비율로 잡으면, v_ref 나
        speed_scale 을 바꿔 프로파일이 통째로 느려져도 문턱이 같이 따라간다.
        """
        n = len(points)
        if n < 3 or len(speeds) != n:
            return 0.0

        seg = [
            math.hypot(
                points[(i + 1) % n][0] - points[i][0],
                points[(i + 1) % n][1] - points[i][1],
            )
            for i in range(n)
        ]
        accel = []
        for i in range(n):
            ds = seg[i]
            if ds < 1e-9:
                accel.append(0.0)
                continue
            accel.append(speeds[i] * (speeds[(i + 1) % n] - speeds[i]) / ds)

        peak = 0.0
        for i in range(n):
            total = 0.0
            width = 0.0
            k = i
            while width < _PROFILE_ACCEL_SMOOTH_M:
                total += accel[k] * seg[k]
                width += seg[k]
                k = (k + 1) % n
                if k == i:
                    break
            if width > 1e-9:
                peak = max(peak, total / width)
        return peak

    @staticmethod
    def _min_over_lookahead(points, profile, lookahead_m):
        """각 점에서 앞쪽 선견구간 안에 있는 목표 속도의 최솟값.

        lookahead_m 은 스칼라(전 구간 같은 거리) 또는 웨이포인트별 배열
        (속도 연동)을 받는다.
        """
        n = len(points)
        if isinstance(lookahead_m, (int, float)):
            spans = [float(lookahead_m)] * n
        else:
            spans = [float(s) for s in lookahead_m]
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
            span = spans[i]
            while travelled < span:
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
        if len(self._path_poses) < 2 or not has_listener(self.tracked_path_pub):
            return

        # PoseStamped 를 매 틱 새로 만들면 140점 Path 하나에 2.6 ms 다 (실측).
        # 33 Hz 면 코어의 9 % 인데, 이건 순수 시각화 토픽이다. 객체는 풀에
        # 두고 좌표만 갈아 끼우면 0.15 ms 로 떨어진다.
        #
        # 재사용이 안전한 이유: `publish()` 는 그 자리에서 직렬화하고 돌아온다.
        # 반환 뒤에는 미들웨어가 이 파이썬 객체를 안 들고 있다. 노드들이 각자
        # 프로세스라 intra-process 로 넘어갈 일도 없다. 발행 후 이 메시지를
        # 붙들고 있는 곳도 없다 — 만들어서 보내고 끝이다.
        msg = self._tracked_path_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        pool = self._tracked_pose_pool
        n = len(self._path_poses)
        while len(pool) < n:
            ps = PoseStamped()
            # 헤더를 공유해 둔다 (예전 코드와 같다). 위에서 스탬프를 한 번
            # 갈면 전 포즈에 반영되므로 포즈마다 만질 일이 없다.
            ps.header = msg.header
            ps.pose.orientation.w = 1.0
            pool.append(ps)

        for i in range(n):
            pos = pool[i].pose.position
            px, py = self._path_poses[i]
            pos.x = float(px)
            pos.y = float(py)

        if len(msg.poses) != n:
            msg.poses = pool[:n]
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
        self._accel_hold_u = self._accel_hold_intensity(measured_speed_alive)

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
            steering_target = (
                self._esp_lag_compensate(steering_raw)
                if self._esp_lag_enable
                else self._smooth_steering(
                    steering_raw, alpha=self.local_path_steering_smooth_alpha
                )
            )
            steering_cmd = self._rate_limit_steering(
                self._limit_lateral_accel(steering_target, control_speed),
                rate_limit_radps=self.local_path_steering_rate_limit_radps,
            )
        else:
            # CSV 추종은 횡가속 상한을 걸지 않는다 — 곡선에서 곡률 FF 가 잘려
            # 오히려 언더스티어가 난다 (FF 는 언더스티어 방지용으로 넣은 항).
            steering_target = (
                self._esp_lag_compensate(steering_raw)
                if self._esp_lag_enable
                else self._smooth_steering(steering_raw)
            )
            steering_cmd = self._rate_limit_steering(steering_target)

        if self._esp_lag_enable:
            # 레이트 제한·포화를 다 거친 최종 명령으로 모델을 굴린다.
            self._esp_lag_update(steering_cmd)

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
