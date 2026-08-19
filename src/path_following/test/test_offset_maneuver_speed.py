#!/usr/bin/env python3
"""회피 기동이 **감속을 부르지 않는지** 검증.

    python3 -m pytest src/path_following/test/test_offset_maneuver_speed.py -q

레이싱에서 회피의 비용은 조향이 아니라 속도다. 옆으로 0.65 m 비키는 데 드는
조향은 6 m/s 에서 1.6° 뿐인데, 속도제한이 "정면에 장애물이 있다" 며 정지거리
기준으로 제동하면 5.7 → 2.9 m/s 로 반토막이 난다. 실제로 그랬다.

정지거리 한계를 면제받으려면 "이 장애물 옆을 지날 때 충분히 벌어져 있다" 를
보여야 하는데, 그 판정이 두 군데서 어긋나 있었다.
  1) 면제 판정이 FGM 식이라 "지금 d 와 목표 d 사이" 를 훑었다. 계획 기동은
     장애물에 닿기 전에 이미 오프셋에 올라와 있는데도 여유를 0 으로 봤다.
  2) 기동이 확보하는 여유(0.12)가 면제 기준(0.10+0.15=0.25)보다 좁았다.
둘 다 여기서 고정한다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.avoidance_safety import (  # noqa: E402
    AvoidSpeedParams,
    avoid_speed_limit,
)
from path_following.offset_maneuver import (  # noqa: E402
    ManeuverConfig,
    ObstacleSD,
    plan_maneuver,
)

P = AvoidSpeedParams(
    v_min=0.6,
    v_max=8.0,
    a_brake=3.0,
    ego_half_width_m=vg.HALF_WIDTH_M,
    ego_front_m=vg.FRONT_M,
    lateral_margin_m=0.10,
    pass_clear_extra_m=0.15,
    safety_factor=0.7,
)

#: 노드가 하는 것과 같은 유도 — 면제 기준을 기동 여유의 하한으로 삼는다.
MARGIN = P.lateral_margin_m + P.pass_clear_extra_m

CFG = ManeuverConfig(
    half_width_m=vg.HALF_WIDTH_M,
    lateral_margin_m=MARGIN,
    max_offset_m=0.70,
    a_lat_enter_mps2=3.0,
    a_lat_exit_mps2=2.0,
    a_lat_hard_mps2=4.5,
    enter_min_m=1.0,
    # 노드가 트랙 길이(37.4 m)의 18% 로 잘라서 쓰는 실제 값. 원래 파라미터
    # (9.0/12.0) 로 재면 실차에서 안 나오는 완만함을 테스트하게 된다.
    enter_max_m=6.7,
    exit_min_m=1.5,
    exit_max_m=6.7,
    hold_front_m=vg.FRONT_M + 0.20,
    hold_rear_m=vg.LENGTH_M + 0.30,
    merge_gap_m=3.0,
    v_plan_min_mps=1.5,
    max_steer_rad=0.60 * 0.3735,
    wheelbase_m=vg.WHEELBASE_M,
)

RACE_V = 6.0
R_CONE = 0.25


def _flat(x_base, y, r):
    """laser frame flat 배열. laser_to_base_x_m=0 으로 두어 x_base 를 그대로 쓴다."""
    return [0.0, x_base, y, r]


def _speed_with_plan(x_obs, v=RACE_V, y_obs=0.0, r=R_CONE):
    m = plan_maneuver(
        [ObstacleSD(x_obs, y_obs, r)], CFG, d_ego=0.0, d_ego_prime=0.0, v=v
    )
    if m is None:
        return None, None
    limit, reason = avoid_speed_limit(
        _flat(x_obs, y_obs, r),
        [],
        v,
        max(0.1, m.obstacle_s_first),
        m.d_pass,
        P,
        include_maneuver=False,
        passing=True,
        path_lat_at=lambda xb: m.d_at(max(0.0, xb)),
    )
    if m.speed_cap_mps is not None and m.speed_cap_mps < limit:
        return m.speed_cap_mps, "maneuver"
    return limit, reason


def _speed_without_plan(x_obs, v=RACE_V, y_obs=0.0, r=R_CONE):
    return avoid_speed_limit(
        _flat(x_obs, y_obs, r), [], v, 2.0, 0.0, P, include_maneuver=False
    )


# ----------------------------------------------------------------------
def test_the_planned_margin_is_exactly_what_earns_the_exemption():
    """기동 여유와 면제 기준이 어긋나면 계획대로 가도 제동이 안 풀린다."""
    assert CFG.lateral_margin_m >= MARGIN
    m = plan_maneuver(
        [ObstacleSD(10.0, 0.0, R_CONE)], CFG, d_ego=0.0, d_ego_prime=0.0, v=RACE_V
    )
    clearance = abs(m.d_at(10.0)) - R_CONE - vg.HALF_WIDTH_M
    need = R_CONE + P.ego_half_width_m + P.lateral_margin_m + P.pass_clear_extra_m
    assert abs(m.d_at(10.0)) >= need - 1e-9, (
        f"통과 시 여유 {clearance:.3f} m 로는 면제를 못 받는다"
    )


def test_no_braking_at_all_when_the_obstacle_is_seen_in_time():
    """레이싱에서 중요한 건 이 한 줄이다 — 제때 보면 안 줄인다."""
    for x in (8.0, 10.0, 12.0):
        v, reason = _speed_with_plan(x)
        assert reason == "clear", f"{x} m 에서 '{reason}' 로 감속한다"
        assert v >= RACE_V, f"{x} m 에서 {v:.2f} m/s 로 깎였다"


def test_this_is_a_real_improvement_over_no_plan():
    """계획이 없으면 같은 상황에서 얼마나 깎였는지 나란히 남긴다."""
    for x in (8.0, 10.0, 12.0):
        with_plan, _ = _speed_with_plan(x)
        without, _ = _speed_without_plan(x)
        assert without < RACE_V, "예전 거동이 재현되지 않는다"
        assert with_plan > without * 1.3


def test_late_detection_still_slows_down():
    """면제가 '아무 때나 안 줄인다' 가 되면 안 된다."""
    v, reason = _speed_with_plan(4.0)
    assert v < RACE_V and reason == "maneuver"


def test_speed_is_monotonic_in_detection_distance():
    """늦게 볼수록 느려야 한다 — 뒤집히면 튜닝이 불가능해진다."""
    speeds = []
    for x in (4.0, 5.0, 6.0, 8.0, 10.0):
        v, _ = _speed_with_plan(x)
        speeds.append(v)
    assert speeds == sorted(speeds), speeds


def test_exemption_is_refused_when_the_plan_does_not_clear():
    """계획이 비켜 가지 못하는 장애물에는 면제가 없어야 한다.

    다른 장애물을 피하려고 옆으로 갔는데 그 자리에 또 하나가 있는 상황이다.
    """
    m = plan_maneuver(
        [ObstacleSD(10.0, 0.0, R_CONE)], CFG, d_ego=0.0, d_ego_prime=0.0, v=RACE_V
    )
    intruder = _flat(10.0, m.d_pass, R_CONE)  # 계획한 통과 위치 위에 있다
    limit, reason = avoid_speed_limit(
        intruder,
        [],
        RACE_V,
        max(0.1, m.obstacle_s_first),
        m.d_pass,
        P,
        include_maneuver=False,
        passing=True,
        path_lat_at=lambda xb: m.d_at(max(0.0, xb)),
    )
    assert reason == "static" and limit < RACE_V


def test_the_old_fgm_style_check_was_the_blocker():
    """회귀 방지: path_lat_at 없이 같은 판정을 하면 면제가 안 나온다."""
    m = plan_maneuver(
        [ObstacleSD(10.0, 0.0, R_CONE)], CFG, d_ego=0.0, d_ego_prime=0.0, v=RACE_V
    )
    old, old_reason = avoid_speed_limit(
        _flat(10.0, 0.0, R_CONE),
        [],
        RACE_V,
        max(0.1, m.obstacle_s_first),
        m.d_pass,
        P,
        include_maneuver=False,
        passing=True,  # path_lat_at 없음 → FGM 식 근사
    )
    assert old_reason == "static" and old < RACE_V
    new, new_reason = _speed_with_plan(10.0)
    assert new_reason == "clear" and new > old


def test_exemption_needs_the_plan_not_just_the_passing_flag():
    """passing=True 만으로 면제가 나오면 안 된다 — 근거는 경로여야 한다."""
    _, reason = avoid_speed_limit(
        _flat(10.0, 0.0, R_CONE),
        [],
        RACE_V,
        2.0,
        0.0,
        P,
        include_maneuver=False,
        passing=True,
        path_lat_at=lambda xb: 0.0,  # 안 비킨다
    )
    assert reason == "static"


def test_across_the_race_envelope_a_timely_plan_costs_nothing():
    # 8~12 m 는 avoid_on 게이트(속도 연동, 상한 12 m)에서 실제로 잡히는 범위다.
    for v in (4.0, 5.0, 6.0, 7.0):
        for x in (8.0, 10.0, 12.0):
            speed, reason = _speed_with_plan(x, v=v)
            assert reason == "clear", f"v={v}, {x} m 에서 '{reason}' 로 감속"
            assert speed >= v


def test_steering_stays_small_across_the_race_envelope():
    """레이싱 속도대에서 회피 조향이 한 자릿수 초반이어야 한다."""
    from path_following.offset_maneuver import peak_abs_d2, steering_for_offset

    for v in (4.0, 5.0, 6.0, 7.0):
        m = plan_maneuver(
            [ObstacleSD(10.0, 0.0, R_CONE)], CFG, d_ego=0.0, d_ego_prime=0.0, v=v
        )
        k = max(
            peak_abs_d2(m.enter_coeff, m.enter_len_m),
            peak_abs_d2(m.exit_coeff, m.exit_len_m),
        )
        deg = math.degrees(steering_for_offset(k, vg.WHEELBASE_M))
        assert deg < 4.0, f"v={v} 에서 조향 {deg:.1f}°"
