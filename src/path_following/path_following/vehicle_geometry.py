"""차량 기하 단일 정의.

지금까지 차폭·차길이 가정이 노드마다 흩어져 있었고 값도 서로 달랐다
(반폭이 0.075 / 0.17 / 0.21 / 0.22 / 0.25 로 제각각). 실측을 한 곳에 두고
전부 여기서 가져오게 한다. 치수를 다시 재면 이 파일만 고친다.

모든 길이는 [m], 기준 프레임은 base_link (뒷바퀴 축, +x 앞, +y 왼쪽).

2026-08-16 실측:
  폭        좌우 각각 0.15  -> 전폭 0.30
  앞        base_link 에서 0.50 까지 하드웨어
  뒤        base_link 에서 0.10 까지
  라이다    base_link 에서 x=0.31, z=0.20 (sensor_static_tf.cpp 와 일치해야 함)
"""
from __future__ import annotations

import math

# ---- 실측 치수 ----
HALF_WIDTH_M = 0.15
WIDTH_M = 2.0 * HALF_WIDTH_M

FRONT_M = 0.50   # base_link -> 앞끝
REAR_M = 0.10    # base_link -> 뒤끝
LENGTH_M = FRONT_M + REAR_M

WHEELBASE_M = 0.33

# 라이다 장착 위치. sensor_static_tf.cpp 의 base_link->laser translation 과
# 반드시 같아야 한다. 스캔 range 는 이 원점에서 재므로, "범퍼까지 몇 m" 를
# 따지는 값은 아래 LASER_TO_FRONT_M 을 더해서 라이다 기준으로 옮겨야 한다.
LASER_X_M = 0.31
LASER_Z_M = 0.20

# 라이다 원점에서 앞끝까지. 스캔이 보는 거리에서 이만큼을 빼야 실제 여유다.
LASER_TO_FRONT_M = FRONT_M - LASER_X_M  # 0.19

# 스윕 계산을 직선으로 취급하는 반경 상한. 이 위에서는 곡률 항이 mm 단위다.
_STRAIGHT_RADIUS_M = 1.0e4


def outer_half_width(radius_m: float) -> float:
    """base_link 가 반경 R 의 원호를 따를 때 차체가 경로 바깥으로 벗어나는 거리.

    차를 점으로 보면 반폭만 필요하지만, 앞이 0.50 m 나 되면 코너에서 앞
    외측 코너가 경로보다 더 바깥을 지난다. 회전중심에서 그 코너까지 거리에서
    경로 반경을 뺀 값이 실제로 필요한 여유다.

        d = hypot(FRONT, R + HALF_WIDTH) - R

    직선(R→∞)에서는 HALF_WIDTH 로 수렴한다. 실제 값:
        R=0.9  -> 0.263   R=1.5 -> 0.224   R=3.0 -> 0.186   직선 -> 0.150
    """
    r = abs(radius_m)
    if not math.isfinite(r) or r >= _STRAIGHT_RADIUS_M:
        return HALF_WIDTH_M
    if r <= 1e-6:
        return math.hypot(FRONT_M, HALF_WIDTH_M)
    return math.hypot(FRONT_M, r + HALF_WIDTH_M) - r


def outer_half_width_at_curvature(kappa_1pm: float) -> float:
    """곡률 κ [1/m] 로 받는 outer_half_width. κ=0 이면 직선."""
    k = abs(kappa_1pm)
    if k <= 1.0 / _STRAIGHT_RADIUS_M:
        return HALF_WIDTH_M
    return outer_half_width(1.0 / k)


# 경로 충돌검사(맵 팽창·장애물 disk)에서 쓸 반폭. 회피 경로가 낼 수 있는
# 가장 급한 코너를 기준으로 잡는다 — 매 점의 곡률을 따로 보기엔 검사 경로가
# 이미 곡률 제한을 거친 상태라 이득이 적고, 상수 하나가 훨씬 안전하다.
PATH_CHECK_MIN_RADIUS_M = 1.0
PATH_CHECK_HALF_WIDTH_M = round(outer_half_width(PATH_CHECK_MIN_RADIUS_M), 3)


def describe() -> str:
    return (
        f"폭 {WIDTH_M:.2f}m(반폭 {HALF_WIDTH_M:.2f}) "
        f"길이 {LENGTH_M:.2f}m(앞 {FRONT_M:.2f}/뒤 {REAR_M:.2f}) "
        f"L={WHEELBASE_M:.2f}m laser_x={LASER_X_M:.2f}m "
        f"(라이다→앞끝 {LASER_TO_FRONT_M:.2f}m)"
    )
