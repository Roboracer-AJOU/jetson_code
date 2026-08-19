"""회피 조향 중 "지나갈 수 있는" 장애물에 정지거리 한계를 걸지 않는지.

증상: FGM 회피 중 차가 0.49 m/s 까지 떨어졌다가(배율 0.17 = v_min 바닥)
장애물이 시야에서 빠지는 순간 다시 튀어 나갔다. 옆으로 비켜 지나가는
중인데도 정면 충돌 기준으로 제동거리를 계산한 게 원인이었다.

여기서 지키려는 두 가지가 서로 반대 방향이라 둘 다 테스트한다.
  - 비켜 가는 게 확실하면 감속하지 않는다 (레이싱)
  - 아직 진로에 있으면 반드시 감속한다 (안전)
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.avoidance_safety import (  # noqa: E402
    AvoidSpeedParams,
    avoid_speed_limit,
    passing_clearance_m,
)

P = AvoidSpeedParams(
    a_lat=4.0,
    a_brake=3.0,
    safety_factor=0.7,
    standoff_m=0.35,
    ego_half_width_m=0.15,
    ego_front_m=0.50,
    lateral_margin_m=0.10,
    pass_clear_extra_m=0.15,
    v_min=0.6,
    v_max=8.0,
)

R = 0.30
# 이 장애물을 지나가려면 필요한 횡여유: r + 반폭 + 마진 (+ 면제 추가분)
NEED = R + P.ego_half_width_m + P.lateral_margin_m       # 0.55
NEED_PASS = NEED + P.pass_clear_extra_m                  # 0.70


def _static(x: float, y: float, r: float = R) -> list[float]:
    return [1.0, x, y, r]


def _limit(x, y, *, lat, avoiding, r=R):
    """정적 장애물 하나에 대한 (목표속도, 사유)."""
    return avoid_speed_limit(
        _static(x, y, r), [], 3.0, 2.0, lat, P, laser_to_base_x_m=0.0,
        include_maneuver=avoiding,
    )


# --------------------------------------------------------- 기하 헬퍼 자체


def test_clearance_zero_when_we_cross_the_obstacle_line():
    """왼쪽으로 0.8 트는데 장애물이 그 사이(0.4)면 정면으로 지난다."""
    assert passing_clearance_m(0.4, 0.8) == 0.0
    assert passing_clearance_m(0.0, 0.8) == 0.0


def test_clearance_uses_the_nearer_endpoint_outside_the_span():
    # 왼쪽(+0.8)으로 트는데 장애물은 오른쪽(−0.5) → 멀어진다
    assert passing_clearance_m(-0.5, 0.8) == pytest.approx(0.5)
    # 왼쪽으로 트는데 장애물이 더 왼쪽(+1.2) → 다가간다 (1.2−0.8=0.4)
    assert passing_clearance_m(1.2, 0.8) == pytest.approx(0.4)


def test_clearance_is_symmetric_in_sign():
    assert passing_clearance_m(0.5, -0.8) == pytest.approx(0.5)


# ------------------------------------------------------------- 핵심 회귀


# x=1.0 은 기존 `beside` 예외(x < 0.85)의 밖이면서 gap 은 음수인 구간이다.
# 예전 코드가 무조건 v_min 을 돌려주던 바로 그 띠다.
PASS_X = 1.0


def test_passing_obstacle_no_longer_crawls():
    """핵심 회귀. 옆으로 확실히 비켜 가는데 v_min 으로 기어가면 안 된다.

    왼쪽으로 1.0 m 틀고 있고 장애물은 오른쪽 0.8 m — 지나간다.
    """
    v, reason = _limit(PASS_X, -0.8, lat=1.0, avoiding=True)
    assert reason == "maneuver", f"정지거리 한계가 아직 걸린다 ({reason})"
    assert v > P.v_min * 1.5, f"여전히 기어간다 ({v:.2f} m/s)"


def test_old_behaviour_was_v_min():
    """같은 상황에서 예전(회피 정보 미사용)에는 바닥이었음을 남겨둔다."""
    v, reason = _limit(PASS_X, -0.8, lat=1.0, avoiding=False)
    assert reason == "static"
    assert v == pytest.approx(P.v_min)


# --------------------------------------------------------------- 안전 쪽


def test_obstacle_in_our_swerve_path_still_brakes():
    """왼쪽으로 트는데 장애물도 왼쪽 진로 안 — 반드시 감속."""
    v, reason = _limit(PASS_X, 0.5, lat=1.0, avoiding=True)
    assert reason == "static"
    assert v == pytest.approx(P.v_min)


def test_dead_ahead_obstacle_still_brakes():
    """정면(y=0)은 아직 안 틀었다는 뜻이다 — 면제 대상이 아니다."""
    v, reason = _limit(PASS_X, 0.0, lat=1.0, avoiding=True)
    assert reason == "static"
    assert v == pytest.approx(P.v_min)


def test_marginal_clearance_is_not_exempted():
    """최소 여유는 넘지만 면제 추가분에 못 미치면 계속 감속한다."""
    y = -(NEED + 0.05)
    assert NEED <= abs(y) < NEED_PASS
    v, reason = _limit(2.0, y, lat=1.0, avoiding=True)
    assert reason == "static", "여유가 빠듯한데 면제됐다"


def test_exemption_requires_active_avoidance():
    """접근 구간(GLOBAL)에서는 레이싱라인이 굽어 |y| 가 커 보일 뿐이다.

    여기서 면제하면 코너 앞 콘을 CSV 전속으로 들이받는다.
    """
    v_go, r_go = _limit(2.5, -0.9, lat=1.0, avoiding=False)
    assert r_go == "static"
    v_av, r_av = _limit(2.5, -0.9, lat=1.0, avoiding=True)
    assert r_av == "maneuver"
    assert v_av > v_go


def test_maneuver_limit_still_caps_speed_when_exempted():
    """면제해도 무제한이 되지는 않는다 — 조향 횡가속도 한계가 남는다."""
    v, reason = _limit(PASS_X, -0.8, lat=1.0, avoiding=True)
    assert reason == "maneuver"
    assert v < P.v_max, "면제가 상한까지 풀어 버렸다"


def test_far_obstacle_outside_corridor_unaffected():
    """멀리 옆에 나란한 건 원래도 무시했다 — 동작이 바뀌면 안 된다."""
    v, reason = _limit(0.5, 1.5, lat=0.0, avoiding=False)
    assert reason == "clear"


@pytest.mark.parametrize("lat", [0.0, 0.3, 1.0, 2.0, -1.0])
def test_never_returns_below_v_min(lat):
    for x in (0.3, 0.8, 1.5, 3.0):
        for y in (-1.2, -0.5, 0.0, 0.5, 1.2):
            v, _ = _limit(x, y, lat=lat, avoiding=True)
            assert v >= P.v_min - 1e-9
            assert v <= P.v_max + 1e-9
