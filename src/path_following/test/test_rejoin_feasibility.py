#!/usr/bin/env python3
"""고속 경로복귀가 물리적으로 따라갈 수 있는 기동인지 검증.

노드를 띄우지 않고 계산 메서드만 스텁 self 에 바인딩해 돌린다.

    python3 -m pytest src/path_following/test/test_rejoin_feasibility.py -q

배경: quintic d(s)=d0*(1-10u^3+15u^4-6u^5) 의 최대 |d''| 는 5.7735*|d0|/L^2 이고,
요구 횡가속도는 v^2 곱이다. 재합류 길이를 속도에만 연동하고 2.5m 로 자르면
7m/s / 1.5m 이탈에서 67.9 m/s^2 를 요구한다 — 타이어 한계의 7배다.
그런 경로는 추종 자체가 불가능해서 조향만 포화된 채 벽으로 밀린다.
"""
from __future__ import annotations

import ast
import inspect
import math
import sys
import textwrap
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.local_planner_node import LocalPlannerNode  # noqa: E402
from path_following.stanley_waypoint_follow_node import (  # noqa: E402
    StanleyWaypointFollowNode,
)

K_PEAK = 5.7735
# 실측 기준 접지력 한계. max_lateral_accel_mps2 주석 참고 (2.5~2.8m/s 코너에서
# v·ω 피크 4.84~5.59 이고 그 지점에서 이미 밀리고 있었다).
GRIP_LIMIT = 9.0
RACE_SPEEDS = (5.0, 6.0, 7.0)
DEVIATIONS = (0.5, 1.0, 1.5, 3.0)


def required_a_lat(d0: float, v: float, length: float) -> float:
    """길이 L 안에 d0 를 0 으로 만드는 quintic 이 요구하는 최대 횡가속도."""
    return v * v * K_PEAK * abs(d0) / (length * length)


def speed_cap_at(d0: float) -> float:
    return _Planner(cte=d0).speed_limit()


class _Planner:
    """_rejoin_length_for / _deviation_speed_limit 가 읽는 것만 갖춘 가짜 노드."""

    _QUINTIC_D1_PEAK = LocalPlannerNode._QUINTIC_D1_PEAK
    _QUINTIC_D2_PEAK = LocalPlannerNode._QUINTIC_D2_PEAK

    def __init__(self, **kw):
        self.rejoin_min_length_m = kw.get("rejoin_min_length_m", 0.50)
        self.rejoin_time_sec = kw.get("rejoin_time_sec", 0.8)
        self.rejoin_max_length_m = kw.get("rejoin_max_length_m", 10.0)
        self.rejoin_a_lat_mps2 = kw.get("rejoin_a_lat_mps2", 4.0)
        # 속도 연동 합류각. 기본은 실제 노드 기본값과 같게 둔다.
        self._rejoin_heading_sin_hi = math.sin(
            math.radians(kw.get("rejoin_max_heading_deg", 18.0))
        )
        self._rejoin_heading_sin_lo = math.sin(
            math.radians(kw.get("rejoin_min_heading_deg", 10.0))
        )
        self.rejoin_merge_overshoot_m = kw.get("rejoin_merge_overshoot_m", 0.30)
        self.rejoin_track_lag_s = kw.get("rejoin_track_lag_s", 0.30)
        # REJOIN 중 실측 초과곡률. 0 이면 "아직 안 쟀다" 라 길이 기반으로 뺀다.
        self._rejoin_kappa_max = kw.get("rejoin_kappa_max", 0.0)
        self.mode = kw.get("mode", "GLOBAL")
        self.deviation_speed_enable = kw.get("deviation_speed_enable", True)
        self.deviation_speed_free_m = kw.get("deviation_speed_free_m", 0.35)
        # 실제 노드는 최저속도를 avoid_speed_params.v_min 에 담는다. 스텁이
        # 평평한 속성으로 흉내내면 여기서 통과하고 실차에서 AttributeError 가
        # 난다 — 실제로 한 번 그렇게 죽었다. 구조를 그대로 맞춘다.
        self.avoid_speed_params = SimpleNamespace(
            v_min=kw.get("avoid_speed_min_mps", 0.6)
        )
        self._last_pose_for_speed = kw.get("pose", object())
        self._maneuver = None  # 계획 기동 없음 = 이탈 감속이 살아 있는 상태
        self._cte = kw.get("cte", 0.0)
        # 속도 상한은 REJOIN 중에는 실제 경로 길이로, 그 밖에서는 최대
        # 길이로 역산한다. 여기 기본은 "아직 경로 없음" 쪽이다.
        self._rejoin_length_m = kw.get("rejoin_length_m", 0.0)

    def _csv_cte_abs_m(self, _pose):
        return self._cte

    length_for = LocalPlannerNode._rejoin_length_for
    heading_limit = LocalPlannerNode._rejoin_heading_limit_rad
    _rejoin_heading_limit_rad = LocalPlannerNode._rejoin_heading_limit_rad
    _rejoin_speed_length_m = LocalPlannerNode._rejoin_speed_length_m
    speed_limit = LocalPlannerNode._deviation_speed_limit


class _Stanley:
    """_steering_for_lateral_accel 가 읽는 것만 갖춘 가짜 노드."""

    def __init__(self, wheelbase: float = 0.33):
        self.wheelbase = wheelbase

    delta_for = StanleyWaypointFollowNode._steering_for_lateral_accel


# ---------------------------------------------------------------- 재합류 길이


def test_rejoin_stays_within_grip_at_race_speed():
    """레이스 속도 × 큰 이탈 전 조합이 예산 안이어야 한다.

    이게 핵심 회귀 테스트다. 예전 로직(L=clip(0.8v, 0.5, 2.5))은 여기서
    전부 터진다.
    """
    p = _Planner()
    for v in RACE_SPEEDS:
        for d0 in DEVIATIONS:
            v_allowed = min(v, speed_cap_at(d0))
            length = p.length_for(d0, v_allowed)
            a = required_a_lat(d0, v_allowed, length)
            assert a <= p.rejoin_a_lat_mps2 + 1e-6, (
                f"v={v_allowed:.1f} d0={d0} L={length:.2f} → {a:.1f} m/s²"
            )


# ------------------------------------------------------------ 합류 헤딩 정렬


def merge_heading_deg(d0: float, length: float) -> float:
    """복귀 경로가 레이스라인과 이루는 최대 각. |d'| 최대는 u=0.5 에서 1.875*|d0|/L."""
    return math.degrees(math.atan(1.875 * abs(d0) / length))


def test_merge_angle_stays_shallow_across_race_envelope():
    """차가 라인에 비스듬히 꽂히지 않아야 한다.

    레이스라인이 벽에 붙어 있어서, 접근각이 서면 추종이 조금만 늦어도
    라인을 넘어 벽에 닿는다. 각을 눕히는 건 길이로만 가능하다.
    """
    p = _Planner()
    for d0 in (0.3, 0.5, 0.8, 1.0, 1.2, 1.5):
        # 이탈이 크면 최대 길이에 먼저 걸려 목표각을 못 지킨다. 트랙이 41 m 라
        # 길이를 더 못 늘리는 것이지 규칙이 틀린 게 아니다 — 그 경우의 각까지
        # 허용하되, 예전 33° 근처로는 절대 돌아가지 않아야 한다.
        capped = merge_heading_deg(d0, p.rejoin_max_length_m)
        assert capped < 20.0, f"d0={d0}: 최대 길이로도 {capped:.1f}° — 너무 서 있다"
        for v in RACE_SPEEDS + (2.5, 3.0):
            limit = max(math.degrees(p.heading_limit(v)), capped)
            ang = merge_heading_deg(d0, p.length_for(d0, v))
            assert ang <= limit + 1e-6, f"d0={d0} v={v} → {ang:.1f}° (한계 {limit:.1f}°)"


def test_old_length_rule_merged_at_a_steep_angle():
    """회귀 방지: 헤딩 제약 없이 시간·횡가속만 보면 얼마나 서 있었는지 남긴다.

    실측 구간(이탈 1.2 m, 2.6 m/s)에서 L=3.4 m 가 나왔고 그 경로는 라인과
    33° 를 이뤘다. 횡가속 예산은 통과하지만 붙는 모습이 전혀 자연스럽지 않다.
    """
    p = _Planner()
    d0, v = 1.2, 2.6
    old_len = min(
        p.rejoin_max_length_m,
        max(
            p.rejoin_min_length_m,
            p.rejoin_time_sec * v,
            math.sqrt(K_PEAK * d0 * v * v / p.rejoin_a_lat_mps2),
        ),
    )
    assert merge_heading_deg(d0, old_len) > 30.0
    assert merge_heading_deg(d0, p.length_for(d0, v)) <= math.degrees(
        p.heading_limit(v)
    ) + 1e-6


def test_time_budget_covers_the_planned_path():
    """포기 시한이 계획 통과시간보다 짧으면 안 된다.

    시한이 먼저 끊기면 override 가 내려가며 기준경로가 CSV 로 튀고, 막으려던
    급조향이 바로 그 순간 나온다. 헤딩 제약으로 경로가 길어졌으므로 고정
    5 초로는 저속·큰이탈에서 모자란다 (1.5 m / 2.5 m/s → 13.2 m, 5.3 초).
    """
    p = _Planner()
    cfg_ns = 5.0 * NS
    for d0, v in ((1.2, 2.6), (1.5, 2.5), (1.2, 6.0), (2.0, 6.0), (0.5, 7.0)):
        L = p.length_for(d0, v)
        budget = max(cfg_ns, 2.5 * (L / max(v, 1.0)) * NS)
        assert budget >= (L / v) * NS, f"d0={d0} v={v}: L={L:.1f}m 를 못 채우고 끊긴다"


def test_shallow_merge_does_not_cost_speed():
    """헤딩 제약으로 길어진 경로가 오히려 감속을 덜 요구한다.

    L 이 커지면 요구 횡가속도는 1/L^2 로 떨어지고 이탈 연동 감속 상한은
    L 에 비례해 올라간다. 완만하게 붙는 쪽이 더 빠르다.
    """
    p = _Planner()
    d0, v = 1.2, 6.0
    short = math.sqrt(K_PEAK * d0 * v * v / p.rejoin_a_lat_mps2)
    long = p.length_for(d0, v)
    assert long > short
    assert required_a_lat(d0, v, long) < required_a_lat(d0, v, short)


# ------------------------------------------------- 코너에서의 재합류 (실제 CSV)


def _load_raceline():
    import csv

    p = Path(__file__).resolve().parents[1] / "config" / "raceline.csv"
    pts = []
    for row in csv.reader(p.open()):
        if not row or row[0].lstrip().startswith("#"):
            continue
        try:
            pts.append((float(row[0]), float(row[1])))
        except (ValueError, IndexError):
            continue
    return pts


class _Corner:
    """실제 레이스라인을 얹고 코너 대응만 떼어낸 가짜 노드."""

    _CURV_BASELINE_M = LocalPlannerNode._CURV_BASELINE_M
    _REJOIN_SIGMA_FLOOR = LocalPlannerNode._REJOIN_SIGMA_FLOOR
    _REJOIN_LENGTH_CANDIDATES = LocalPlannerNode._REJOIN_LENGTH_CANDIDATES
    _QUINTIC_D1_PEAK = LocalPlannerNode._QUINTIC_D1_PEAK
    _QUINTIC_D2_PEAK = LocalPlannerNode._QUINTIC_D2_PEAK

    def __init__(self, straight: bool = False):
        import numpy as np

        pts = _load_raceline()
        self._n = len(pts)
        self._xs_np = np.asarray([p[0] for p in pts], dtype=float)
        self._ys_np = np.asarray([p[1] for p in pts], dtype=float)
        self._total_l = sum(
            math.hypot(
                pts[(i + 1) % self._n][0] - pts[i][0],
                pts[(i + 1) % self._n][1] - pts[i][1],
            )
            for i in range(self._n)
        )
        self.rejoin_min_length_m = 0.50
        self.rejoin_max_length_m = 10.0
        self.rejoin_time_sec = 0.8
        self.rejoin_a_lat_mps2 = 4.0
        self.rejoin_merge_overshoot_m = 0.30
        self.rejoin_track_lag_s = 0.30
        self.rejoin_max_path_curvature = 1.19  # tan(21.4°)/0.33
        self._rejoin_heading_sin_hi = math.sin(math.radians(18.0))
        self._rejoin_heading_sin_lo = math.sin(math.radians(10.0))
        self._build_curvature()
        if straight:  # 기준선이 직선일 때의 환원을 보기 위한 모드
            self._kappa_np = np.zeros(self._n)
            self._kappa_d_np = np.zeros(self._n)

    def tightest_s(self):
        import numpy as np

        i = int(np.argmax(np.abs(self._kappa_np)))
        return i * self._total_l / self._n, (1.0 if self._kappa_np[i] > 0 else -1.0)

    def max_kappa(self):
        import numpy as np

        return float(np.abs(self._kappa_np).max())

    _build_curvature = LocalPlannerNode._build_curvature
    _kappa_at_s = LocalPlannerNode._kappa_at_s
    _rejoin_path_curvature = LocalPlannerNode._rejoin_path_curvature
    _plan_rejoin = LocalPlannerNode._plan_rejoin
    _rejoin_length_for = LocalPlannerNode._rejoin_length_for
    _rejoin_heading_limit_rad = LocalPlannerNode._rejoin_heading_limit_rad
    _solve_quintic = staticmethod(LocalPlannerNode.__dict__["_solve_quintic"].__func__)
    _eval_quintic = staticmethod(LocalPlannerNode.__dict__["_eval_quintic"].__func__)
    _eval_quintic_d1 = staticmethod(
        LocalPlannerNode.__dict__["_eval_quintic_d1"].__func__
    )
    _eval_quintic_d2 = staticmethod(
        LocalPlannerNode.__dict__["_eval_quintic_d2"].__func__
    )


def test_straight_reference_reduces_to_the_closed_form():
    """기준선이 직선이면 초과곡률이 예전 `5.77·|d0|/L²` 로 돌아와야 한다.

    이게 어긋나면 곡률식이나 부호가 틀린 것이다.
    """
    n = _Corner(straight=True)
    for d0, L in ((1.2, 8.4), (1.5, 10.0), (0.5, 6.0)):
        coeff = n._solve_quintic(d0, 0.0, 0.0, 0.0, 0.0, 0.0, L)
        got, sigma = n._rejoin_path_curvature(0.0, coeff, L)
        assert sigma == pytest.approx(1.0)
        assert got == pytest.approx(K_PEAK * d0 / L**2, rel=0.05)


def test_reference_curvature_is_not_charged_to_the_manoeuvre():
    """기준선 곡률 자체는 초과분에서 빠져야 한다.

    레이스라인을 그냥 달려도 감당하는 몫이고 CSV 속도가 이미 그걸로 짜여
    있다. 총량으로 재면 중간 코너만 지나도 직선 복귀조차 4 m/s 아래로
    묶여서, 복귀 때마다 불필요하게 기어간다.
    """
    n = _Corner()
    s_t, _ = n.tightest_s()
    coeff = n._solve_quintic(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0)  # 이탈 0 = 라인 그대로
    excess, _ = n._rejoin_path_curvature(s_t - 3.0, coeff, 6.0)
    assert excess == pytest.approx(0.0, abs=1e-6), "라인 위에 있는데 초과곡률이 잡힌다"


def test_corner_inside_costs_more_than_outside():
    """코너 안쪽 복귀가 바깥쪽보다 훨씬 비싸다 (σ=1-d·κ 증폭).

    직선 가정은 둘을 똑같이 본다 — 그게 놓치던 것이다.

    시작점은 **정점 바로 앞** 이어야 한다. quintic 은 초반에 `d` 를 거의
    그대로 유지하므로, 그래야 이탈이 큰 구간이 정점을 지난다. 정점에서 멀면
    그 지점의 곡률이 지배해서 안/밖 구분 자체가 흐려진다 (트랙을 다시 뽑으면
    정점 위치가 바뀌므로 거리로 못 박으면 안 된다).
    """
    n = _Corner()
    s_t, sgn = n.tightest_s()
    s0 = s_t - 1.5
    _, _, k_in, sig_in = n._plan_rejoin(s0, 1.2 * sgn, 0.0, 0.0, 6.0)
    _, _, k_out, sig_out = n._plan_rejoin(s0, -1.2 * sgn, 0.0, 0.0, 6.0)
    assert sig_in < sig_out, f"안쪽 σ={sig_in:.2f} 가 바깥쪽 {sig_out:.2f} 보다 크다"
    # 안쪽이 아예 불가능(inf)해도 "더 비싸다" 는 성립한다.
    assert (not math.isfinite(k_in)) or k_in > k_out * 1.3, (
        f"안쪽 {k_in:.3f} vs 바깥쪽 {k_out:.3f}"
    )


def test_deep_inside_a_hairpin_is_abandoned_not_driven():
    """헤어핀 안쪽 깊숙이서는 감속이 아니라 포기해야 한다.

    σ→0 이면 오프셋 경로가 곡률 중심으로 뭉개져 어떤 속도로도 못 따라간다.
    그런 경로를 내보내느니 CSV 로 넘기는 게 안전하다 — Stanley 피드백은
    접지력 예산에 묶여 있다.
    """
    n = _Corner()
    s_t, sgn = n.tightest_s()
    r_min = 1.0 / n.max_kappa()
    # 정점에서 곡률반경만큼 안쪽 = 곡률 중심 위. 여기는 정의 자체가 안 된다.
    _, _, kappa, _ = n._plan_rejoin(s_t, r_min * sgn, 0.0, 0.0, 6.0)
    assert not math.isfinite(kappa), "곡률 중심에 있는데 경로를 만들어 버린다"


def test_unsteerable_path_is_abandoned_even_at_crawling_speed():
    """조향 한계를 넘는 곡률은 감속으로 해결되지 않는다.

    실측 전륜각 21.4° / 축거 0.33 m 라 낼 수 있는 최대 경로곡률이 1.19 1/m 다.
    초과곡률(κ_path−κ)만 보면 이게 안 걸린다 — 이미 급한 코너에서는 조금만
    더 휘어도 핸들이 끝까지 돌아간 상태가 되기 때문이다. 예전에는 이런
    경로가 "상한 0.7 m/s" 로 통과해서 차가 이상한 궤적 위를 기어갔다.
    """
    n = _Corner()
    s_t, sgn = n.tightest_s()
    for d0 in (1.2, 1.4, 1.6):
        _, _, kappa, _ = n._plan_rejoin(s_t - 0.5, d0 * sgn, 0.0, 0.0, 6.0)
        if math.isfinite(kappa):
            cap = math.sqrt(n.rejoin_a_lat_mps2 / kappa) if kappa > 1e-6 else 99.0
            assert cap >= 1.5, f"d0={d0}: {cap:.1f} m/s 로 기어간다 — 포기했어야 한다"


def test_normal_deviations_are_not_abandoned_around_the_track():
    """흔한 이탈량에서는 트랙 어디서든 복귀가 성립해야 한다.

    포기가 잦으면 override 가 자주 내려가 CSV 로 튀는 전환이 반복된다.
    """
    n = _Corner()
    step = max(1, n._n // 60)
    total = fails = 0
    for i in range(0, n._n, step):
        s0 = i * n._total_l / n._n
        for sgn in (1.0, -1.0):
            total += 1
            _, _, kappa, _ = n._plan_rejoin(s0, 1.0 * sgn, 0.0, 0.0, 6.0)
            if not math.isfinite(kappa):
                fails += 1
    assert fails / total < 0.05, f"{fails}/{total} 포기 — 너무 자주 접힌다"


def test_corner_rejoin_speed_cap_is_usable():
    """코너 복귀 상한이 기어가는 수준으로 떨어지면 안 된다."""
    n = _Corner()
    step = max(1, n._n // 60)
    caps = []
    for i in range(0, n._n, step):
        s0 = i * n._total_l / n._n
        for sgn in (1.0, -1.0):
            _, _, kappa, _ = n._plan_rejoin(s0, 1.0 * sgn, 0.0, 0.0, 6.0)
            if math.isfinite(kappa) and kappa > 1e-6:
                caps.append(math.sqrt(n.rejoin_a_lat_mps2 / kappa))
    caps.sort()
    median = caps[len(caps) // 2]
    assert median >= 4.0, f"중앙값 {median:.1f} m/s — 복귀마다 감속이 과하다"


def test_old_fixed_clamp_was_infeasible():
    """회귀 방지: 예전 2.5m 고정 상한이 왜 안 됐는지 남긴다."""
    for v in RACE_SPEEDS:
        old_len = min(2.50, max(0.50, 0.8 * v))
        assert required_a_lat(1.5, v, old_len) > 3 * GRIP_LIMIT


def test_length_grows_with_deviation():
    p = _Planner()
    assert p.length_for(1.5, 7.0) > p.length_for(0.5, 7.0)


def test_length_grows_with_speed_where_speed_constraints_bind():
    """속도 의존은 시간·횡가속 제약이 헤딩 제약을 넘어설 때만 나타난다.

    헤딩 제약 `L >= 1.875*|d0|/tan(θ)` 는 속도와 무관하다. 이탈이 크면
    그쪽이 지배해서 속도를 바꿔도 길이가 그대로인데, 그게 맞는 동작이다 —
    천천히 간다고 라인에 덜 비스듬히 붙는 건 아니다.
    """
    p = _Planner()
    assert p.length_for(0.3, 7.0) > p.length_for(0.3, 3.0)


def test_merge_angle_tightens_as_speed_rises():
    """빠를수록 합류각을 좁혀야 한다.

    합류각 ψ 로 붙으면 라인을 가로지르는 속도가 `v·sin(ψ)` 라, 추종 지연이
    같아도 **빠를수록 더 많이 넘어간다**. 각을 고정하면 고속에서 오버슈트가
    그대로 속도에 비례해 커진다.
    """
    p = _Planner()
    angs = [math.degrees(p.heading_limit(v)) for v in (2.5, 3.0, 4.0, 5.0, 6.0, 7.0)]
    assert angs == sorted(angs, reverse=True)
    assert angs[0] > angs[-1], "속도에 따라 전혀 안 좁혀진다"


def merge_overshoot_m(p, v: float) -> float:
    """합류 시 라인을 넘어가는 양 ≈ v·sin(ψ)·τ."""
    return v * math.sin(p.heading_limit(v)) * p.rejoin_track_lag_s


def test_merge_overshoot_budget_is_respected_until_the_floor_binds():
    """각이 아니라 '라인을 넘어가는 양' 이 묶여 있는지 본다.

    하한각(10°)이 걸리기 전까지는 예산을 정확히 지킨다. 저속에서는 상한각
    (18°)에 먼저 걸려 예산보다 덜 쓴다.
    """
    p = _Planner()
    floor = math.degrees(math.asin(p._rejoin_heading_sin_lo))
    for v in (3.0, 4.0, 5.0):
        assert math.degrees(p.heading_limit(v)) > floor, "이 속도는 하한 전이어야 한다"
        assert merge_overshoot_m(p, v) <= p.rejoin_merge_overshoot_m + 1e-9


def test_overshoot_overrun_at_the_floor_stays_bounded():
    """하한각이 예산을 이기는 건 의도된 양보다 — 대신 그 대가를 못 박아 둔다.

    41 m 트랙에서 10° 보다 눕히려면 만들 수 없는 길이가 필요하다. 그래서
    고속에서는 예산을 조금 넘기는데, 그 초과폭이 슬금슬금 커지면 안 된다.
    """
    p = _Planner()
    worst = max(merge_overshoot_m(p, v) for v in RACE_SPEEDS)
    assert worst <= 1.25 * p.rejoin_merge_overshoot_m, f"{worst:.2f} m 까지 넘어간다"


def test_merge_angle_never_collapses_to_an_unbuildable_length():
    """고속에서 각을 무한정 좁히면 트랙 둘레로는 못 만드는 길이를 요구한다."""
    p = _Planner()
    assert math.degrees(p.heading_limit(20.0)) >= 9.0


def test_length_respects_min_and_max():
    p = _Planner()
    assert p.length_for(0.0, 0.0) == p.rejoin_min_length_m
    assert p.length_for(50.0, 20.0) == p.rejoin_max_length_m


def test_time_based_length_still_applies_when_deviation_tiny():
    """이탈이 거의 없으면 횡가속 역산값이 0 이라 시간 연동이 이긴다."""
    p = _Planner()
    assert p.length_for(0.001, 6.0) == p.rejoin_time_sec * 6.0


# ------------------------------------------------------------ 이탈 연동 감속


def test_no_speed_penalty_during_normal_driving():
    """정상 주행의 작은 CTE 로는 절대 감속하지 않아야 한다."""
    for cte in (0.0, 0.1, 0.2, 0.35):
        assert _Planner(cte=cte).speed_limit() == float("inf")


def test_large_deviation_caps_speed_below_race_pace():
    assert _Planner(cte=5.0).speed_limit() < 7.0


def test_deviation_cap_is_exactly_the_grip_budget():
    """감속 상한은 임의의 숫자가 아니라 '최대 길이로 복귀할 때의 횡가속 예산'이다."""
    for cte in (1.0, 2.0, 3.0, 5.0):
        p = _Planner(cte=cte)
        v = p.speed_limit()
        assert required_a_lat(cte, v, p.rejoin_max_length_m) == pytest.approx(
            p.rejoin_a_lat_mps2
        )


def test_speed_limit_monotonic_in_deviation():
    caps = [_Planner(cte=d).speed_limit() for d in (0.5, 1.0, 2.0, 3.0, 5.0)]
    assert caps == sorted(caps, reverse=True)


def test_speed_limit_never_stalls_the_car():
    """아무리 멀어도 avoid_speed_min_mps 아래로는 안 내려간다 (갇힘 방지)."""
    assert _Planner(cte=500.0).speed_limit() == 0.6


def test_speed_limit_disabled_by_param():
    assert _Planner(cte=5.0, deviation_speed_enable=False).speed_limit() == float(
        "inf"
    )


def test_speed_limit_needs_pose():
    assert _Planner(cte=5.0, pose=None).speed_limit() == float("inf")


# ------------------------------------------------- 피드백 조향 접지력 상한


def test_feedback_cap_matches_requested_lateral_accel():
    """상한 각도로 달리면 정확히 예산만큼의 횡가속도가 나와야 한다."""
    s = _Stanley()
    for v in (2.0, 5.0, 7.0):
        delta = s.delta_for(4.0, v)
        a = v * v * math.tan(delta) / s.wheelbase
        assert abs(a - 4.0) < 1e-6


def test_feedback_cap_bites_only_at_speed():
    """저속에서는 max_steering(21.4°) 보다 커서 사실상 안 걸린다."""
    s = _Stanley()
    assert math.degrees(s.delta_for(4.0, 2.0)) > 15.0
    assert math.degrees(s.delta_for(4.0, 7.0)) < 3.0


def test_feedback_cap_off_when_disabled_or_crawling():
    s = _Stanley()
    assert s.delta_for(0.0, 7.0) is None
    assert s.delta_for(4.0, 0.2) is None


def test_uncapped_feedback_would_exceed_grip():
    """회귀 방지: 상한이 없을 때 7m/s 에서 10° 는 접지력을 크게 넘는다."""
    a = 7.0 * 7.0 * math.tan(math.radians(10.0)) / 0.33
    assert a > 2 * GRIP_LIMIT


# ------------------------------------------------------------- REJOIN 탈출

NS = 1_000_000_000


class _Clock:
    def __init__(self):
        self.t = 0
        self.nanoseconds = 0

    def now(self):
        return self

    def advance(self, sec: float):
        self.nanoseconds += int(sec * NS)


class _Rejoining:
    """REJOIN 유지/포기 판정에 필요한 것만 갖춘 가짜 노드."""

    def __init__(self, **kw):
        self.rejoin_stall_speed_mps = kw.get("rejoin_stall_speed_mps", 0.25)
        self.rejoin_stall_ns = int(kw.get("rejoin_stall_sec", 1.0) * NS)
        self.rejoin_max_active_ns = int(kw.get("rejoin_max_active_sec", 5.0) * NS)
        # 실제 시한은 경로 길이에 맞춰 늘어난다. 기본은 설정값 그대로.
        self._rejoin_budget_ns = int(
            kw.get("rejoin_budget_sec", self.rejoin_max_active_ns / NS) * NS
        )
        self._ego_speed_mps = kw.get("speed", 3.0)
        self._clock = _Clock()
        self._rejoin_start_ns = 0
        self._rejoin_moving_ns = 0
        self._rejoin_travel_m = 0.0
        self._rejoin_last_xy = None
        self._rejoin_progress_cycle = -1
        self._tf_cycle_id = 0

    def get_clock(self):
        return self._clock

    track = LocalPlannerNode._rejoin_track_progress
    abandon = LocalPlannerNode._rejoin_abandon_reason


def _drive(node: _Rejoining, seconds: float, step: float = 0.025) -> None:
    for _ in range(int(seconds / step)):
        node._clock.advance(step)
        node._tf_cycle_id += 1  # 주기당 1회 갱신 규약
        node.track(None)


def test_progress_tracking_is_once_per_cycle():
    """모드 갱신과 발행 양쪽에서 불려도 이동거리를 두 번 세면 안 된다."""
    n = _Rejoining()
    n._rejoin_last_xy = (0.0, 0.0)
    pose = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=2.0, y=0.0)))
    n._tf_cycle_id = 1
    n.track(pose)
    n.track(pose)
    assert n._rejoin_travel_m == pytest.approx(2.0)


def test_stopped_car_abandons_rejoin():
    """핵심 회귀: 정지하면 CTE 가 안 줄어 영원히 못 빠져나왔다.

    실측에서 정지 후 수 분간 REJOIN + override=true 가 유지됐고, Stanley 는
    수십 초 묵은 캐시 경로를 붙들고 있었다.
    """
    n = _Rejoining(speed=0.0)
    _drive(n, 0.5)
    assert n.abandon() == "", "0.5초 만에 포기하면 안 된다"
    _drive(n, 1.0)
    assert n.abandon() == "정지"


def test_moving_car_is_not_abandoned_for_stall():
    n = _Rejoining(speed=3.0)
    _drive(n, 3.0)
    assert n.abandon() == ""


def test_rejoin_has_absolute_time_cap():
    """계속 굴러가도 CTE 가 안 줄면(측위 바이어스 등) 상한에서 끊는다."""
    n = _Rejoining(speed=3.0)
    _drive(n, 6.0)
    assert "초과" in n.abandon()


def test_brief_stop_does_not_abandon():
    """신호 대기 수준의 짧은 감속으로는 포기하지 않는다."""
    n = _Rejoining(speed=3.0)
    _drive(n, 1.0)
    n._ego_speed_mps = 0.0
    _drive(n, 0.5)
    assert n.abandon() == ""


def test_rejoin_path_is_never_rebuilt():
    """회귀: 주기적 재생성이 기준경로를 갈아치워 복귀가 거칠어졌다.

    추종 오차를 경로를 바꿔서 없애면 안 된다 — 오차가 아니라 기준이
    지워진다. 한 번 그린 경로는 기동이 끝날 때까지 그대로여야 한다.
    """
    built = []

    class _N:
        _rejoin_path_msg = None
        _rejoin_progress_cycle = -1
        _tf_cycle_id = 0
        _ego_speed_mps = 3.0
        rejoin_stall_speed_mps = 0.25
        _rejoin_travel_m = 0.0
        _rejoin_last_xy = None
        _rejoin_moving_ns = 0

        def get_clock(self):
            return _Clock()

        def _build_frenet_quintic_rejoin_path(self, _pose):
            built.append(1)
            return SimpleNamespace(header=SimpleNamespace(stamp=None), poses=[1, 2])

        def _rejoin_reset_progress(self, _pose):
            pass

        refresh = LocalPlannerNode._refresh_rejoin_path
        _rejoin_track_progress = LocalPlannerNode._rejoin_track_progress

    n = _N()
    pose = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)))
    assert n.refresh(pose) is True
    first = n._rejoin_path_msg
    for i in range(200):  # 200 주기 = 수 미터 주행
        n._tf_cycle_id = i + 1
        pose.pose.position.x = 0.05 * i
        assert n.refresh(pose) is True
    assert len(built) == 1, f"경로를 {len(built)}번 다시 그렸다"
    assert n._rejoin_path_msg is first


# ----------------------------------------------------- 복귀 중 속도 회복


class _DevSpeed:
    """`_deviation_speed_limit` 만 떼어낸 가짜 노드."""

    _QUINTIC_D2_PEAK = LocalPlannerNode._QUINTIC_D2_PEAK

    def __init__(
        self, mode: str, cte: float, path_len: float = 0.0, kappa_max: float = 0.0
    ):
        self.mode = mode
        self.deviation_speed_enable = True
        self.deviation_speed_free_m = 0.35
        self.rejoin_a_lat_mps2 = 4.0
        self.rejoin_max_length_m = 10.0
        self._rejoin_length_m = path_len
        # 실측 초과곡률. 0 이면 아직 안 잰 것이라 길이 기반 역산으로 떨어진다.
        self._rejoin_kappa_max = kappa_max
        self.avoid_speed_params = SimpleNamespace(v_min=0.6)
        self._last_pose_for_speed = object()
        # 계획 기동이 살아 있으면 이탈 감속은 통째로 면제된다. 여기 테스트는
        # 그 반대쪽(기동 없이 라인에서 벗어난 상태)을 본다.
        self._maneuver = None
        self._cte = cte

    def _csv_cte_abs_m(self, _pose):
        return self._cte

    # 실물과 같은 이름으로 붙여야 내부 호출이 이어진다
    _rejoin_speed_length_m = LocalPlannerNode._rejoin_speed_length_m
    length = LocalPlannerNode._rejoin_speed_length_m
    limit = LocalPlannerNode._deviation_speed_limit


def test_rejoin_uses_actual_path_length_not_the_max():
    """회귀: 최대 길이(12 m)를 쓰면 상한이 안 걸려 CSV 전속으로 꽂힌다."""
    real = _DevSpeed("REJOIN", cte=1.2, path_len=4.0)
    naive = _DevSpeed("REJOIN", cte=1.2, path_len=0.0)  # 최대 길이로 폴백
    assert real.limit() < naive.limit()
    assert real.limit() < 3.5, f"복귀 초입인데 상한이 느슨하다 ({real.limit():.1f})"
    assert naive.limit() > 7.0, "예전 동작(사실상 무제한) 재현이 안 된다"


def test_speed_recovers_as_deviation_shrinks():
    """라인에 붙어 갈수록 상한이 커져야 한다 — 이게 '점진 복원' 이다."""
    prev = 0.0
    for cte in (1.6, 1.2, 0.8, 0.5, 0.4):
        v = _DevSpeed("REJOIN", cte=cte, path_len=4.0).limit()
        assert v > prev, f"cte={cte} 에서 상한이 안 늘었다"
        prev = v
    # 자유 구간에 들어가면 상한 자체가 사라진다
    assert _DevSpeed("REJOIN", cte=0.3, path_len=4.0).limit() == float("inf")


def test_outside_rejoin_still_uses_max_length():
    """아직 경로가 없는 상태(GLOBAL/AVOID)에서는 '최대한 길게 잡아도 되나'."""
    n = _DevSpeed("GLOBAL", cte=1.2, path_len=4.0)
    assert n.length() == pytest.approx(n.rejoin_max_length_m)


def test_deviation_limit_never_below_v_min():
    assert _DevSpeed("REJOIN", cte=9.0, path_len=1.0).limit() == pytest.approx(0.6)


# --------------------------------------------------------- 재합류 비활성화


class _Bridge:
    """`_publish_rejoin_bridge` 가 무엇을 발행하는지만 보는 가짜 노드."""

    def __init__(self, enable: bool, cte: float):
        self.rejoin_enable = enable
        self.rejoin_finish_lateral_m = 0.20
        self._cte = cte
        self._rejoin_path_msg = None
        self.published = []
        self.gate = []
        self.built = 0
        self.planned = []

    def _csv_cte_abs_m(self, _pose):
        return self._cte

    def _refresh_rejoin_path(self, _pose):
        self.built += 1
        self._rejoin_path_msg = SimpleNamespace(
            header=SimpleNamespace(stamp=None), poses=[1, 2]
        )
        return True

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "t"))

    pub_path = property(lambda self: SimpleNamespace(
        publish=lambda m: self.published.append(m)))
    pub_sent_dbg = None

    def _publish_override_gate(self, on):
        self.gate.append(on)

    def _set_path_planned(self, planned):
        self.planned.append(planned)

    bridge = LocalPlannerNode._publish_rejoin_bridge


def test_bridge_is_inert_when_rejoin_disabled():
    """재합류를 끄면 다리도 놓지 않는다 — 안 그러면 '꺼도 켜진' 꼴이 된다."""
    n = _Bridge(enable=False, cte=1.2)
    assert n.bridge(object()) is False
    assert n.built == 0 and not n.published and not n.gate


def test_bridge_skips_when_already_on_the_line():
    """이미 라인 위면 CSV 로 넘겨도 계단이 안 생기니 다리가 불필요하다."""
    n = _Bridge(enable=True, cte=0.05)
    assert n.bridge(object()) is False
    assert n.built == 0


def test_bridge_holds_override_when_off_the_line():
    n = _Bridge(enable=True, cte=1.2)
    assert n.bridge(object()) is True
    assert n.gate == [True], "라인 밖인데 override 를 놓쳤다"
    assert len(n.published) == 1
    # 재합류 경로는 계획 기하지만 게인이 FF 없이 맞춰져 있어 FF 는 끈 채로 준다.
    assert n.planned == [False]


def test_bridge_needs_a_pose():
    n = _Bridge(enable=True, cte=1.2)
    assert n.bridge(None) is False


# ------------------------------------------------------------- 속도 슬루


class _Slewing:
    """감속/가속 기울기 제한만 떼어낸 가짜 노드."""

    def __init__(self, a_brake=3.0, a_accel=4.0):
        self.avoid_speed_params = SimpleNamespace(a_brake=a_brake)
        self.avoid_a_accel_mps2 = a_accel
        self._clock = _Clock()
        self._slew_prev_v = None
        self._slew_prev_ns = 0

    def get_clock(self):
        return self._clock

    slew = LocalPlannerNode._slew_limit_speed


def test_recovery_acceleration_is_rate_limited():
    """회피 해제 순간 목표속도가 한 프레임에 튀면 안 된다.

    실측: 배율 0.17 → 1.00 이 한 프레임에 바뀌며 0.46 → 3.39 m/s (6 m/s^2).
    그 급가속이 재합류 조향과 겹치면서 라인 밖으로 밀렸다.
    """
    n = _Slewing(a_accel=4.0)
    n._clock.advance(0.025)
    n.slew(0.5)  # 이력 초기화
    n._clock.advance(0.025)
    v = n.slew(8.0)  # 위협 사라짐 → 정책 상한으로 점프 시도
    assert v == pytest.approx(0.5 + 4.0 * 0.025), "가속이 안 묶였다"

    # 0.5 → 3.4 까지 걸리는 시간이 물리적으로 말이 되어야 한다
    t = 0.0
    while n._slew_prev_v < 3.4 and t < 5.0:
        n._clock.advance(0.025)
        t += 0.025
        n.slew(8.0)
    assert 0.6 <= t <= 0.9, f"회복이 너무 느리거나 빠르다 ({t:.2f}s)"


def test_braking_is_still_allowed_to_be_immediate_within_a_brake():
    """가속 상한이 감속을 방해하면 안 된다."""
    n = _Slewing()
    n._clock.advance(0.025)
    n.slew(4.0)
    n._clock.advance(0.1)
    v = n.slew(0.0)
    assert v == pytest.approx(4.0 - 3.0 * 0.1)


def test_accel_limit_can_be_disabled():
    n = _Slewing(a_accel=0.0)
    n._clock.advance(0.025)
    n.slew(0.5)
    n._clock.advance(0.025)
    assert n.slew(8.0) == pytest.approx(8.0)


# ------------------------------------------------------- 스텁/실물 드리프트


def _self_attrs_read(func) -> set[str]:
    tree = ast.parse(textwrap.dedent(inspect.getsource(func)))
    return {
        n.attr
        for n in ast.walk(tree)
        if isinstance(n, ast.Attribute)
        and isinstance(n.value, ast.Name)
        and n.value.id == "self"
        and not isinstance(n.ctx, ast.Store)
    }


def _self_attrs_assigned(cls) -> set[str]:
    tree = ast.parse(textwrap.dedent(inspect.getsource(cls)))
    return {
        n.attr
        for n in ast.walk(tree)
        if isinstance(n, ast.Attribute)
        and isinstance(n.value, ast.Name)
        and n.value.id == "self"
        and isinstance(n.ctx, ast.Store)
    }


@pytest.mark.parametrize(
    "func",
    [
        LocalPlannerNode._rejoin_length_for,
        LocalPlannerNode._deviation_speed_limit,
        LocalPlannerNode._rejoin_reset_progress,
        LocalPlannerNode._rejoin_track_progress,
        LocalPlannerNode._rejoin_abandon_reason,
        LocalPlannerNode._refresh_rejoin_path,
        LocalPlannerNode._slew_limit_speed,
        LocalPlannerNode._rejoin_speed_length_m,
    ],
)
def test_methods_only_read_attributes_that_exist(func):
    """읽는 self.X 가 실제로 노드에 존재하는지 정적으로 확인한다.

    스텁 테스트만으로는 이걸 못 잡는다. 실제로 `self.avoid_speed_min_mps`
    (노드에는 없고 avoid_speed_params.v_min 에 들어있다) 를 읽다가 주행 중
    AttributeError 로 플래너가 통째로 죽은 적이 있다. 스텁이 그 이름을
    갖고 있어서 테스트는 전부 통과했었다.
    """
    known = _self_attrs_assigned(LocalPlannerNode) | set(dir(LocalPlannerNode))
    missing = sorted(_self_attrs_read(func) - known)
    assert not missing, f"{func.__name__} 가 없는 속성을 읽는다: {missing}"
