"""복귀는 지금 헤딩을 보고 길이를 정한다 — 라인을 넘어가면 안 된다.

회피 직후의 차는 라인과 나란하지 않다. 이미 라인을 향해 비스듬히 달리는
중이고, 복귀 경로는 C1 연속이라 그 헤딩에서 출발할 수밖에 없다.

예전 `_rejoin_length_for` 는 **이탈량만** 보고 길이를 정했다. 그래서 헤딩이
서 있어도 허용각(12.8°) 기준으로 길게 잡았고, 초기 기울기가 라인을 지나쳐
**반대편으로 넘어갔다가** 되돌아오는 경로가 나왔다. 레이스라인이 벽에 붙어
있으면 그 넘어간 양이 그대로 벽까지의 여유를 먹는다.

넘어가는 양은 `|d0|·F(p)`, `p = d0p·L/d0` 라 **길이에 비례해 커진다.**
"길게 잡으면 완만해진다" 는 직관이 여기서만 거꾸로다.

    python3 -m pytest src/path_following/test/test_rejoin_heading_aware.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path as FsPath

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))
sys.path.insert(0, str(FsPath(__file__).resolve().parent))

from path_following.local_planner_node import CFG, LocalPlannerNode  # noqa: E402
from test_rejoin_feasibility import _Corner  # noqa: E402

BUDGET = CFG["rejoin_merge_overshoot_m"]


class _Planner:
    """`_rejoin_length_for` 가 읽는 것만 갖춘 가짜 노드 (기준선 = 직선)."""

    _QUINTIC_D1_PEAK = LocalPlannerNode._QUINTIC_D1_PEAK
    _QUINTIC_D2_PEAK = LocalPlannerNode._QUINTIC_D2_PEAK

    _rejoin_heading_limit_rad = LocalPlannerNode._rejoin_heading_limit_rad
    _rejoin_length_for = LocalPlannerNode._rejoin_length_for
    _rejoin_line_crossing_m = LocalPlannerNode._rejoin_line_crossing_m
    _rejoin_crossing_cap_m = LocalPlannerNode._rejoin_crossing_cap_m
    _eval_quintic = staticmethod(LocalPlannerNode.__dict__["_eval_quintic"].__func__)
    _solve_quintic = staticmethod(LocalPlannerNode.__dict__["_solve_quintic"].__func__)
    _eval_quintic_d1 = staticmethod(
        LocalPlannerNode.__dict__["_eval_quintic_d1"].__func__
    )

    def __init__(self):
        for k in (
            "rejoin_min_length_m",
            "rejoin_max_length_m",
            "rejoin_time_sec",
            "rejoin_a_lat_mps2",
            "rejoin_merge_overshoot_m",
            "rejoin_track_lag_s",
        ):
            setattr(self, k, CFG[k])
        self._rejoin_heading_sin_hi = math.sin(
            math.radians(CFG["rejoin_max_heading_deg"])
        )
        self._rejoin_heading_sin_lo = math.sin(
            math.radians(CFG["rejoin_min_heading_deg"])
        )


def _slope(deg: float) -> float:
    return math.tan(math.radians(deg))


def _toward(d0: float, deg: float) -> float:
    """이탈 d0 인 차가 라인 쪽으로 deg 만큼 틀어져 있을 때의 d0p."""
    return -math.copysign(_slope(deg), d0)


def _crossing(p: _Planner, d0: float, d0p: float, L: float) -> float:
    coeff = p._solve_quintic(d0, d0p, 0.0, 0.0, 0.0, 0.0, L)
    return p._rejoin_line_crossing_m(coeff, L, d0, n=400)


def _peak_angle_deg(p: _Planner, d0: float, d0p: float, L: float) -> float:
    coeff = p._solve_quintic(d0, d0p, 0.0, 0.0, 0.0, 0.0, L)
    peak = max(abs(p._eval_quintic_d1(coeff, L * k / 399)) for k in range(400))
    return math.degrees(math.atan(peak))


# ------------------------------------------------- 넘어가지 않는다

DEVIATIONS = (0.6, 1.0, 1.5)
HEADINGS = (10.0, 20.0, 30.0, 40.0, 45.0)
SPEEDS = (2.0, 3.0, 4.5, 6.0)


def test_the_path_does_not_cross_the_line():
    """어떤 (이탈, 헤딩, 속도) 조합에서도 반대편으로 넘어가지 않는다."""
    p = _Planner()
    for d0 in DEVIATIONS:
        for deg in HEADINGS:
            for v in SPEEDS:
                d0p = _toward(d0, deg)
                L = p._rejoin_length_for(d0, v, d0p)
                assert _crossing(p, d0, d0p, L) <= BUDGET, (d0, deg, v)


def test_it_works_on_the_other_side_too():
    """부호만 뒤집은 상황도 같아야 한다."""
    p = _Planner()
    for deg in HEADINGS:
        left = p._rejoin_length_for(1.0, 3.0, _toward(1.0, deg))
        right = p._rejoin_length_for(-1.0, 3.0, _toward(-1.0, deg))
        assert left == right, deg


def test_regression_ignoring_the_heading_drove_us_over_the_line():
    """예전 동작(=d0p 를 빼고 부른 길이)은 실제로 예산을 넘겼다.

    이 조건이 안 넘어가게 되면 위 테스트들이 아무것도 안 지키는 것이다.
    """
    p = _Planner()
    d0, v = 1.0, 3.0
    d0p = _toward(d0, 30.0)
    before = p._rejoin_length_for(d0, v)  # d0p 없이 = 예전 식
    assert _crossing(p, d0, d0p, before) > BUDGET
    after = p._rejoin_length_for(d0, v, d0p)
    assert _crossing(p, d0, d0p, after) <= BUDGET
    assert after < before, "헤딩이 서 있으면 짧아져야 한다"


# ------------------------------------------- 길이가 길수록 더 넘어간다

def test_going_longer_makes_the_crossing_worse():
    """이 기동에서는 '길게 = 완만하게' 가 성립하지 않는다."""
    p = _Planner()
    d0, d0p = 1.0, _toward(1.0, 30.0)
    crossings = [_crossing(p, d0, d0p, L) for L in (3.0, 5.0, 7.0, 9.0)]
    assert crossings == sorted(crossings)
    assert crossings[0] <= BUDGET < crossings[-1]


def test_the_crossing_only_depends_on_the_shape_ratio():
    """넘어가는 양 / |d0| 는 p = d0p·L/d0 만의 함수다 — 스케일 불변."""
    p = _Planner()
    deg = 30.0
    for ratio in (2.0, 4.0, 5.0):
        seen = set()
        for d0 in DEVIATIONS:
            d0p = _toward(d0, deg)
            L = ratio * abs(d0) / _slope(deg)
            seen.add(round(_crossing(p, d0, d0p, L) / abs(d0), 4))
        assert len(seen) == 1, (ratio, seen)


# --------------------------------------------------- 회귀: 기존 동작 유지


def test_a_car_already_parallel_is_unchanged():
    """헤딩이 라인과 나란하면 예전 길이 그대로다."""
    p = _Planner()
    for d0 in DEVIATIONS:
        for v in SPEEDS:
            assert p._rejoin_length_for(d0, v, 0.0) == p._rejoin_length_for(d0, v)


def test_a_car_pointing_away_still_uses_the_allowed_angle():
    """라인에서 멀어지는 중이면 경로가 각을 먼저 되돌려야 한다.

    이때 '이미 선 각' 을 목표로 삼으면 되레 짧아져서 급해진다.
    """
    p = _Planner()
    for d0 in DEVIATIONS:
        for deg in HEADINGS:
            away = math.copysign(_slope(deg), d0)
            assert p._rejoin_length_for(d0, 3.0, away) == p._rejoin_length_for(d0, 3.0)


def test_a_gentle_heading_does_not_shorten_the_path():
    """허용각보다 완만하게 서 있으면 예전처럼 길게 간다."""
    p = _Planner()
    d0, v = 1.0, 3.0
    limit = math.degrees(p._rejoin_heading_limit_rad(v))
    gentle = _toward(d0, limit * 0.5)
    assert p._rejoin_length_for(d0, v, gentle) == p._rejoin_length_for(d0, v)


def test_the_path_never_bulges_past_the_heading_we_came_in_with():
    """짧게 잡으면 초기 각보다 더 서는 구간이 생긴다. 그러면 안 된다.

    이미 선 각보다 더 세우지 않는 게 조건이지, 무조건 눕히는 게 아니다.
    완만하게 들어온 경우엔 허용각까지는 써도 된다 — 거기까지가 예산이다.
    """
    p = _Planner()
    v = 3.0
    allowed = math.degrees(p._rejoin_heading_limit_rad(v))
    for d0 in DEVIATIONS:
        for deg in HEADINGS:
            d0p = _toward(d0, deg)
            L = p._rejoin_length_for(d0, v, d0p)
            ceiling = max(deg, allowed) + 1.0
            assert _peak_angle_deg(p, d0, d0p, L) <= ceiling, (d0, deg)


def test_the_length_stays_inside_the_configured_bounds():
    p = _Planner()
    for d0 in DEVIATIONS:
        for deg in HEADINGS:
            for v in SPEEDS:
                L = p._rejoin_length_for(d0, v, _toward(d0, deg))
                assert CFG["rejoin_min_length_m"] <= L <= CFG["rejoin_max_length_m"]


# ------------------------------------ 코너: 실제 진입점 `_plan_rejoin`
#
# 위까지는 기준선이 직선일 때의 길이 계산이다. 코너에서는 Frenet→직교
# 변환이 곡률을 바꾸므로 `_plan_rejoin` 이 후보를 다시 훑는다. 벽에 붙은
# 라인은 대개 코너이므로 여기가 진짜 현장이다.


def _corner():
    n = _Corner()
    n.rejoin_merge_overshoot_m = BUDGET  # 픽스처 기본(0.30)이 아닌 생산값으로
    return n


def _sweep(n):
    """(중심 s, 이탈, 헤딩, 속도) 를 훑으며 계획 결과를 모은다."""
    s_t, sgn = n.tightest_s()
    for s0 in (s_t, s_t - 3.0, s_t + 2.0, s_t + n._total_l / 2):
        for d0 in (0.4 * sgn, 0.6 * sgn, 1.0 * sgn, 1.5 * sgn, -0.8 * sgn, -1.2 * sgn):
            for deg in (0.0, 10.0, 20.0, 30.0, 40.0, 45.0):
                for v in (2.0, 3.0, 4.5, 6.0):
                    d0p = _toward(d0, deg)
                    L, coeff, kappa, sigma = n._plan_rejoin(s0, d0, d0p, 0.0, v)
                    yield (d0, deg, v), L, coeff, kappa


def test_no_planned_corner_path_crosses_the_line():
    """계획이 나왔다면 그 경로는 라인을 넘지 않는다. 예외 없다."""
    n = _corner()
    for case, L, coeff, kappa in _sweep(n):
        if not math.isfinite(kappa):
            continue  # 포기한 건 경로를 안 준 것이라 넘을 것도 없다
        assert n._rejoin_line_crossing_m(coeff, L, case[0], 400) <= BUDGET, case


def test_it_gives_up_rather_than_aim_at_the_wall():
    """급코너에 비스듬히 서면 성립하는 길이가 없다 — 그땐 CSV 를 유지한다.

    짧게 잡으면 조향 한계를 넘고 길게 잡으면 라인을 넘는다. 사이에 답이
    없으므로 무한대를 돌려 호출부가 포기하게 만든다.
    """
    n = _corner()
    s_t, sgn = n.tightest_s()
    _, _, kappa, _ = n._plan_rejoin(s_t, 0.4 * sgn, -math.copysign(1.0, sgn), 0.0, 2.0)
    assert not math.isfinite(kappa)


def test_giving_up_stays_rare():
    """포기는 예외여야 한다. 회피 후 복귀를 자주 접으면 그것대로 위험하다."""
    n = _corner()
    total = planned = 0
    for _, _, _, kappa in _sweep(n):
        total += 1
        planned += math.isfinite(kappa)
    # 이 픽스처는 반경 1.41 m 짜리 극단 코너다. 절반 이상은 나와야 한다.
    assert planned / total > 0.7, f"{planned}/{total}"


def test_a_car_parallel_to_the_line_is_unaffected_in_corners():
    """헤딩이 0 이면 넘어갈 일이 없으니 코너 판정도 예전 그대로다."""
    n = _corner()
    for case, L, coeff, kappa in _sweep(n):
        if case[1] != 0.0 or not math.isfinite(kappa):
            continue
        assert n._rejoin_line_crossing_m(coeff, L, case[0], 400) <= 1e-6, case
