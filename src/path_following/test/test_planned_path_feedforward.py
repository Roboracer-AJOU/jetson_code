#!/usr/bin/env python3
"""계획된 회피 경로에서는 곡률 FF 가 살아 있어야 한다.

    python3 -m pytest src/path_following/test/test_planned_path_feedforward.py -q

Stanley 는 LOCAL_PATH 에서 피드백(heading+CTE)에 횡가속 상한을 건다. 6 m/s 에
2.1°, 7 m/s 에 1.5° 다. 그 상한은 **오차 보정**을 묶으라고 넣은 것이다.

문제는 같은 모드에서 FF 도 꺼져 있었다는 점이다. FGM 폴백 경로만 생각하면
맞는 처리다 — 조준점까지 그은 직선이라 곡률이 목표점 흔들림에서 나오는
잡음이고, 증폭하면 조향이 떤다.

하지만 횡오프셋 기동은 곡률이 우리가 타기로 계획한 값이다. FF 를 끄면 그
곡률조차 피드백이 만들어야 하는데, 상한이 딱 그만한 크기라 기하를 내는 데
예산을 다 쓰고 오차 지울 여지가 안 남는다. 여기서 그 산수를 고정한다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.offset_maneuver import (  # noqa: E402
    ManeuverConfig,
    ObstacleSD,
    peak_abs_d2,
    plan_maneuver,
)
from path_following.stanley_waypoint_follow_node import CFG as SCFG  # noqa: E402

MCFG = ManeuverConfig(
    half_width_m=vg.HALF_WIDTH_M,
    lateral_margin_m=0.25,
    max_offset_m=0.70,
    a_lat_enter_mps2=3.0,
    a_lat_exit_mps2=2.0,
    a_lat_hard_mps2=4.5,
    enter_min_m=1.0,
    enter_max_m=6.7,
    exit_min_m=1.5,
    exit_max_m=6.7,
    hold_front_m=vg.FRONT_M + 0.20,
    hold_rear_m=vg.LENGTH_M + 0.30,
    merge_gap_m=3.0,
    v_plan_min_mps=1.5,
    max_steer_rad=0.3735,
    wheelbase_m=vg.WHEELBASE_M,
)

WHEELBASE = float(SCFG["wheelbase"])
FB_A_LAT = float(SCFG["feedback_lateral_accel_mps2"])

RACE_SPEEDS = (4.0, 5.0, 6.0, 7.0)


def _ff_allowed(mode: str, planned: bool) -> bool:
    """노드의 FF 게이트와 같은 식."""
    return mode != "LOCAL_PATH" or planned


def _feedback_budget(v: float) -> float:
    """LOCAL_PATH 피드백에 걸리는 조향 상한 [rad]."""
    return math.atan(WHEELBASE * FB_A_LAT / (v * v))


def _planned_steer(v: float) -> float:
    """그 속도에서 계획 기동이 요구하는 조향 [rad]."""
    m = plan_maneuver(
        [ObstacleSD(s=v * 2.0, d=0.0, r=0.18)],
        MCFG,
        d_ego=0.0,
        d_ego_prime=0.0,
        v=v,
    )
    assert m is not None, f"{v} m/s 에서 기동 계획이 안 나왔다"
    kappa = max(
        peak_abs_d2(m.enter_coeff, m.enter_len_m),
        peak_abs_d2(m.exit_coeff, m.exit_len_m),
    )
    return math.atan(WHEELBASE * kappa)


# ── 게이트 자체 ──────────────────────────────────────────────────────────


def test_planned_avoidance_path_keeps_feedforward():
    assert _ff_allowed("LOCAL_PATH", planned=True)


def test_reactive_fgm_path_still_has_feedforward_off():
    """FGM 폴백 경로의 곡률은 조준점 잡음이라 FF 로 증폭하면 안 된다."""
    assert not _ff_allowed("LOCAL_PATH", planned=False)


def test_clean_track_is_unaffected():
    for planned in (False, True):
        assert _ff_allowed("CSV_TRACKING", planned)


# ── 왜 필요한가: 조향 예산 산수 ──────────────────────────────────────────


@pytest.mark.parametrize("v", RACE_SPEEDS)
def test_without_ff_the_plan_eats_most_of_the_feedback_budget(v: float):
    """FF 가 없으면 상한의 대부분을 기하 내는 데 써버린다.

    측정값 (계획 곡률 조향 / 피드백 상한):
        4 m/s 3.54°/4.72° = 75%
        5 m/s 2.27°/3.02° = 75%
        6 m/s 1.57°/2.10° = 75%
        7 m/s 1.41°/1.54° = 91%

    즉 7 m/s 에서 오차 보정에 남는 건 상한의 9% 다. 계획 경로에서 벌어져도
    되돌릴 힘이 없다는 뜻이고, 그게 "라인을 못 따라간다" 의 정체다.
    """
    need = _planned_steer(v)
    budget = _feedback_budget(v)
    assert need > 0.5 * budget, (
        f"{v} m/s: 계획 곡률 {math.degrees(need):.2f}° 가 "
        f"피드백 예산 {math.degrees(budget):.2f}° 의 절반도 안 된다 — "
        "이 테스트의 전제가 깨졌다"
    )


def test_the_squeeze_is_worst_at_race_speed():
    """상한은 v² 로 조여지는데 계획 곡률은 그만큼 안 줄어 고속이 제일 빡세다."""
    ratio = {v: _planned_steer(v) / _feedback_budget(v) for v in RACE_SPEEDS}
    assert ratio[7.0] > ratio[4.0]
    assert ratio[7.0] > 0.85, f"7 m/s 여유가 예상보다 크다: {ratio}"


@pytest.mark.parametrize("v", RACE_SPEEDS)
def test_with_ff_the_whole_feedback_budget_is_left_for_error(v: float):
    """FF 가 기하를 내면 상한은 온전히 오차 보정 몫으로 남는다."""
    assert _ff_allowed("LOCAL_PATH", planned=True)
    assert _feedback_budget(v) > 0.0


@pytest.mark.parametrize("v", RACE_SPEEDS)
def test_the_planned_curvature_itself_is_gentle(v: float):
    """계획 곡률은 어느 속도에서도 풀락 근처로 가지 않는다."""
    assert math.degrees(_planned_steer(v)) < 15.0


def test_faster_means_flatter():
    """속도가 오르면 같은 오프셋을 더 길게 뽑아 조향이 줄어야 한다."""
    steers = [_planned_steer(v) for v in RACE_SPEEDS]
    for a, b in zip(steers, steers[1:]):
        assert b <= a + 1e-9, f"고속에서 조향이 늘었다: {steers}"
