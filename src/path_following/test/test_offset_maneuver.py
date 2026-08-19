#!/usr/bin/env python3
"""횡오프셋 회피 기동의 물리적 타당성을 고정한다.

    python3 -m pytest src/path_following/test/test_offset_maneuver.py -q

여기서 지키려는 것은 하나다: **고속에서 조향을 조금만 쓰고 지나간다.**
이전 구현은 진입 길이가 1.2 m 고정이라 6 m/s 에서 90 m/s² 를 요구했고,
낼 수 없는 값이라 차가 조향을 문 채로 벽에 갔다. 그 회귀를 막는다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.offset_maneuver import (  # noqa: E402
    D2_PEAK,
    ManeuverConfig,
    ObstacleSD,
    OffsetManeuver,
    choose_pass_offset,
    group_blocking,
    length_for_budget,
    peak_abs_d2,
    plan_maneuver,
    solve_quintic,
    speed_for_length,
    steering_for_offset,
)

CFG = ManeuverConfig(
    half_width_m=vg.HALF_WIDTH_M,
    lateral_margin_m=0.12,
    max_offset_m=0.65,
    a_lat_enter_mps2=3.0,
    a_lat_exit_mps2=2.0,
    a_lat_hard_mps2=4.5,
    enter_min_m=1.0,
    enter_max_m=9.0,
    exit_min_m=1.5,
    exit_max_m=12.0,
    hold_front_m=vg.FRONT_M + 0.20,
    hold_rear_m=vg.LENGTH_M + 0.30,
    merge_gap_m=3.0,
    v_plan_min_mps=1.5,
    max_steer_rad=0.3735,
    wheelbase_m=vg.WHEELBASE_M,
)

RACE_V = 6.0


def _plan(obstacles, v=RACE_V, d_ego=0.0, d_ego_prime=0.0, cfg=CFG):
    return plan_maneuver(
        obstacles, cfg, d_ego=d_ego, d_ego_prime=d_ego_prime, v=v
    )


def _peak_kappa(m: OffsetManeuver) -> float:
    return max(
        peak_abs_d2(m.enter_coeff, m.enter_len_m),
        peak_abs_d2(m.exit_coeff, m.exit_len_m),
    )


def _max_steer_deg(m: OffsetManeuver) -> float:
    return math.degrees(steering_for_offset(_peak_kappa(m), vg.WHEELBASE_M))


# ----------------------------------------------------------------------
# 수학
# ----------------------------------------------------------------------
def test_quintic_meets_its_boundary_conditions():
    c = solve_quintic(0.2, 0.05, 0.0, 0.6, 0.0, 0.0, 5.0)
    from path_following.offset_maneuver import (
        _eval_quintic,
        _eval_quintic_d1,
        _eval_quintic_d2,
    )

    assert _eval_quintic(c, 0.0) == pytest.approx(0.2)
    assert _eval_quintic_d1(c, 0.0) == pytest.approx(0.05)
    assert _eval_quintic(c, 5.0) == pytest.approx(0.6)
    assert _eval_quintic_d1(c, 5.0) == pytest.approx(0.0, abs=1e-9)
    assert _eval_quintic_d2(c, 5.0) == pytest.approx(0.0, abs=1e-9)


def test_length_formula_actually_hits_the_budget():
    """L = v·sqrt(5.7735·Δd/a) 가 실제로 그 횡가속을 낸다."""
    for v, dd, a in ((6.0, 0.5, 3.0), (4.0, 0.35, 3.0), (7.0, 0.8, 4.0)):
        L = length_for_budget(dd, v, a)
        c = solve_quintic(0.0, 0.0, 0.0, dd, 0.0, 0.0, L)
        assert v * v * peak_abs_d2(c, L) == pytest.approx(a, rel=0.01)
        assert speed_for_length(dd, L, a) == pytest.approx(v, rel=0.01)


def test_length_grows_linearly_with_speed():
    """같은 횡이동이면 진입 거리는 속도에 비례한다 — 이게 고속 대응의 전부다."""
    a, dd = 3.0, 0.5
    assert length_for_budget(dd, 6.0, a) == pytest.approx(
        2.0 * length_for_budget(dd, 3.0, a), rel=1e-9
    )


# ----------------------------------------------------------------------
# 사용자가 지목한 상황: 6 m/s 직선, 경로 위 장애물
# ----------------------------------------------------------------------
def test_race_speed_pass_uses_a_tiny_steering_angle():
    m = _plan([ObstacleSD(10.0, 0.0, 0.25)])
    assert m is not None
    assert _max_steer_deg(m) < 3.0, "고속 회피에 3° 넘게 쓰면 안 된다"
    assert m.peak_lateral_accel_mps2 <= CFG.a_lat_enter_mps2 + 1e-6
    assert m.speed_cap_mps is None, "여유 있게 만났으면 감속할 이유가 없다"


def test_race_speed_pass_starts_well_before_the_obstacle():
    """'멀리서부터 조금 틀기' 가 실제로 일어나는지."""
    m = _plan([ObstacleSD(10.0, 0.0, 0.25)])
    assert m is not None
    assert m.enter_len_m > 4.0, f"진입이 {m.enter_len_m:.1f}m 면 너무 급하다"
    assert m.enter_len_m / RACE_V > 0.7, "진입에 최소 0.7 초는 써야 한다"


def test_the_old_fixed_length_would_have_been_impossible():
    """회귀 방지: 예전 1.2 m 고정이 어떤 값을 요구했는지 숫자로 남긴다."""
    dd = 0.52
    c = solve_quintic(0.0, 0.0, 0.0, dd, 0.0, 0.0, 1.2)
    a_old = RACE_V**2 * peak_abs_d2(c, 1.2)
    assert a_old > 70.0, "옛 설정이 재현되지 않는다"
    steer_old = math.degrees(steering_for_offset(peak_abs_d2(c, 1.2), vg.WHEELBASE_M))
    assert steer_old > math.degrees(CFG.max_steer_rad), "풀락으로도 못 타는 경로였다"
    # 지금은 같은 상황을 훨씬 적은 조향으로 푼다.
    m = _plan([ObstacleSD(10.0, 0.0, 0.25)])
    assert _max_steer_deg(m) < steer_old / 10.0


def test_offset_actually_clears_the_obstacle():
    obs = ObstacleSD(10.0, 0.0, 0.25)
    m = _plan([obs])
    assert m is not None
    # 장애물 표면을 지나는 구간 내내 차체 옆면이 여유를 유지해야 한다.
    ds = obs.s - obs.r
    while ds <= obs.s + obs.r:
        gap = abs(m.d_at(ds) - obs.d) - obs.r - vg.HALF_WIDTH_M
        assert gap >= CFG.lateral_margin_m - 1e-6, f"ds={ds:.2f} 에서 여유 {gap:.3f}"
        ds += 0.05


def test_the_car_is_back_on_the_line_at_the_end():
    m = _plan([ObstacleSD(10.0, 0.0, 0.25)])
    assert m is not None
    assert m.d_at(m.total_length_m) == pytest.approx(0.0, abs=1e-9)
    assert m.d_at(m.total_length_m + 5.0) == pytest.approx(0.0, abs=1e-9)


def test_return_is_gentler_than_the_entry():
    """복귀는 급할 이유가 없다 — 진입보다 길게 나와야 한다."""
    m = _plan([ObstacleSD(10.0, 0.0, 0.25)])
    assert m is not None
    assert m.exit_len_m > m.enter_len_m


# ----------------------------------------------------------------------
# 속도별 거동
# ----------------------------------------------------------------------
def test_lateral_accel_never_exceeds_the_budget_across_the_envelope():
    for v in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0):
        m = _plan([ObstacleSD(12.0, 0.0, 0.25)], v=v)
        assert m is not None, f"v={v} 에서 계획 실패"
        assert m.peak_lateral_accel_mps2 <= CFG.a_lat_hard_mps2 + 1e-6
        assert m.speed_cap_mps is None


def test_faster_means_earlier_and_flatter():
    slow = _plan([ObstacleSD(12.0, 0.0, 0.25)], v=3.0)
    fast = _plan([ObstacleSD(12.0, 0.0, 0.25)], v=6.0)
    assert fast.enter_len_m > slow.enter_len_m
    assert _max_steer_deg(fast) < _max_steer_deg(slow)


# ----------------------------------------------------------------------
# 늦게 발견 → 조향이 아니라 속도로 답한다
# ----------------------------------------------------------------------
def test_late_detection_answers_with_a_speed_cap():
    m = _plan([ObstacleSD(4.0, 0.0, 0.25)])
    assert m is not None
    assert m.speed_cap_mps is not None and m.speed_cap_mps < RACE_V


def test_the_speed_cap_is_actually_sufficient():
    """돌려준 상한까지 줄이면 정말 예산 안으로 들어오는지."""
    m = _plan([ObstacleSD(4.0, 0.0, 0.25)])
    v_cap = m.speed_cap_mps
    assert v_cap**2 * _peak_kappa(m) == pytest.approx(CFG.a_lat_hard_mps2, rel=0.02)


def test_unsteerable_geometry_is_refused_not_published():
    """감속으로 안 풀리는 건 곡률이다. 그런 경로는 아예 안 만든다."""
    m = _plan([ObstacleSD(1.6, 0.0, 0.25)])
    assert m is None, "조향 한계를 넘는 경로를 만들면 그대로 벽으로 간다"


def test_every_published_plan_is_steerable():
    for s in [x * 0.1 for x in range(15, 150)]:
        for v in (2.0, 4.0, 6.0, 7.0):
            m = _plan([ObstacleSD(s, 0.0, 0.25)], v=v)
            if m is None:
                continue
            assert _max_steer_deg(m) <= math.degrees(CFG.max_steer_rad) + 1e-6


# ----------------------------------------------------------------------
# 방향 선택 / 연속 장애물
# ----------------------------------------------------------------------
def test_picks_the_cheaper_side():
    """장애물이 왼쪽으로 치우쳐 있으면 오른쪽으로 지나간다."""
    m = _plan([ObstacleSD(10.0, 0.30, 0.20)])
    assert m is not None and m.side == -1 and m.d_pass < 0.0


def test_does_not_flip_sides_once_committed_offset_exists():
    """이미 왼쪽에 나가 있으면 왼쪽 통과가 더 싸게 나와야 한다."""
    obs = [ObstacleSD(10.0, 0.0, 0.25)]
    left = _plan(obs, d_ego=+0.40)
    right = _plan(obs, d_ego=-0.40)
    assert left.side == +1
    assert right.side == -1


def test_consecutive_obstacles_are_one_maneuver():
    """하나 피하고 붙었다가 또 피하는 톱니를 막는다."""
    m = _plan([ObstacleSD(9.0, 0.10, 0.20), ObstacleSD(11.5, -0.10, 0.20)])
    assert m is not None
    assert m.obstacle_s_last >= 11.5, "뒤쪽 장애물까지 한 기동으로 덮어야 한다"
    for s, d, r in ((9.0, 0.10, 0.20), (11.5, -0.10, 0.20)):
        gap = abs(m.d_at(s) - d) - r - vg.HALF_WIDTH_M
        assert gap >= CFG.lateral_margin_m - 1e-6


def test_far_apart_obstacles_are_not_merged():
    g = group_blocking(
        [ObstacleSD(9.0, 0.0, 0.20), ObstacleSD(30.0, 0.0, 0.20)], CFG
    )
    assert g is not None and g.count == 1


def test_obstacle_outside_the_corridor_is_ignored():
    assert group_blocking([ObstacleSD(10.0, 1.20, 0.20)], CFG) is None
    assert _plan([ObstacleSD(10.0, 1.20, 0.20)]) is None


def test_obstacle_behind_is_ignored():
    assert _plan([ObstacleSD(-3.0, 0.0, 0.25)]) is None


def test_track_too_narrow_is_refused():
    """양쪽 다 오프셋 상한을 넘으면 계획하지 않는다 — 감속/정지로 넘겨야 한다."""
    narrow = ManeuverConfig(**{**CFG.__dict__, "max_offset_m": 0.20})
    assert choose_pass_offset(
        group_blocking([ObstacleSD(10.0, 0.0, 0.30)], narrow), narrow, 0.0
    ) is None
    assert plan_maneuver(
        [ObstacleSD(10.0, 0.0, 0.30)], narrow, d_ego=0.0, d_ego_prime=0.0, v=RACE_V
    ) is None


# ----------------------------------------------------------------------
# 연속성
# ----------------------------------------------------------------------
def test_plan_starts_from_the_current_lateral_state():
    """재계획해도 현재 위치에서 이어져야 경로가 튀지 않는다."""
    m = _plan([ObstacleSD(10.0, 0.0, 0.25)], d_ego=0.18, d_ego_prime=0.05)
    assert m is not None
    assert m.d_at(0.0) == pytest.approx(0.18)


def test_slope_is_carried_over_only_when_the_curve_starts_now():
    """리드 구간이 있으면 그동안 기준선과 나란히 가므로 시작 기울기는 0 이다.

    지금의 기울기를 진입 시작점에 붙이면 거기서 경로가 꺾인다.
    """
    # 남은 거리가 예산 길이보다 짧으면 리드 없이 지금 바로 꺾기 시작한다.
    late = _plan([ObstacleSD(3.95, 0.0, 0.25)], d_ego=0.18, d_ego_prime=0.05)
    assert late is not None and late.lead_len_m == pytest.approx(0.0, abs=1e-6)
    assert late.d_prime_at(0.0) == pytest.approx(0.05)

    early = _plan([ObstacleSD(10.0, 0.0, 0.25)], d_ego=0.18, d_ego_prime=0.05)
    assert early.lead_len_m > 0.5
    assert early.d_prime_at(0.0) == pytest.approx(0.0)
    assert early.d_prime_at(early.lead_len_m + 1e-9) == pytest.approx(0.0, abs=1e-6)


def test_lead_keeps_the_car_on_the_racing_line_until_it_must_move():
    """진입에 필요한 길이보다 멀리서 봤으면 그만큼 라인 위에 더 있어야 한다."""
    m = _plan([ObstacleSD(12.0, 0.0, 0.25)])
    assert m is not None and m.lead_len_m > 1.0
    assert m.d_at(m.lead_len_m) == pytest.approx(m.d_start)
    assert m.d_at(m.lead_len_m * 0.5) == pytest.approx(m.d_start)
    # 리드가 있어도 조향은 그대로 완만하다 — 진입 길이는 안 줄였다.
    assert _max_steer_deg(m) < 3.0


def test_offset_is_held_across_the_obstacle_span():
    obs = ObstacleSD(10.0, 0.0, 0.25)
    m = _plan([obs])
    assert m.d_at(m.enter_end_ds) == pytest.approx(m.d_pass, abs=1e-9)
    assert m.d_at(obs.s) == pytest.approx(m.d_pass, abs=1e-9)
    assert m.hold_end_ds >= obs.s + obs.r
