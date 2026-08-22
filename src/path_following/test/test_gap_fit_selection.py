"""FGM 은 차폭이 안 들어가는 갭을 후보에서 뺀다.

실측(20260822, 런치 로그)에서 회피가 AEB 까지 간 조우다.

    t=17.0s  v=4.0  장애물 4.09 m  AVOID
    gap 은 열렸지만 차폭이 안 들어감 — aim=-30° clear=0.57m < 1.00m

각도로만 고르면 "넓어 보이는데 못 지나가는" 갭이 이긴다. 그 뒤의 코리도
검사는 **이미 고른 갭 안에서** 각도를 옮길 뿐이라 되돌리지 못한다.
"""

from __future__ import annotations

import math

import numpy as np

from path_following.fgm_node import FGMNode


class _Stub:
    """갭 적합성 판정에 필요한 것만 들고 있는 껍데기."""

    def __init__(self, fit_min=1.0, samples=5, enable=True, fov_deg=90.0, inset_deg=3.0):
        self.corridor_half_width = 0.254
        self.corridor_stop_margin = 0.15
        self.preprocess_dist = 10.0
        self.gap_fit_check_enable = enable
        self.gap_fit_samples = samples
        self.gap_fit_min_m = fit_min
        self.gap_edge_inset_rad = math.radians(inset_deg)
        self.fov_angle = math.radians(90.0)
        self._fov_rad = math.radians(fov_deg)

    _corridor_clear_distance = FGMNode._corridor_clear_distance
    _clamp_to_cone = staticmethod(FGMNode._clamp_to_cone)
    _aim_range = FGMNode._aim_range
    _gap_best_clear_m = FGMNode._gap_best_clear_m
    _gaps_that_fit = FGMNode._gaps_that_fit

    def fit(self, gaps, rng, ang):
        return self._gaps_that_fit(gaps, rng, ang, ang, None, 0.0)


def _scan(blocked):
    """(각도, 거리) 격자. blocked = [(각도범위, 거리), ...] 는 벽이다."""
    ang = np.linspace(-math.pi / 2, math.pi / 2, 361)
    rng = np.full(ang.shape, 10.0)
    for (a0, a1), r in blocked:
        rng[(ang >= a0) & (ang <= a1)] = r
    return ang, rng


def test_a_wide_but_impassable_gap_is_dropped():
    """왼쪽은 좁아도 뚫려 있고, 오른쪽은 각도만 넓고 0.6 m 에서 막힌다."""
    d30 = math.radians(-30.0)
    ang, rng = _scan(
        [
            ((math.radians(-45.0), math.radians(-15.0)), 0.6),  # 오른쪽: 곧 막힘
            ((math.radians(-5.0), math.radians(5.0)), 1.2),  # 정면: 박스
        ]
    )
    right = np.where((ang >= math.radians(-45.0)) & (ang <= math.radians(-15.0)))[0]
    left = np.where(ang >= math.radians(15.0))[0]

    p = _Stub()
    kept = p.fit([right, left], rng, ang)

    assert len(kept) == 1
    assert kept[0] is left, "막힌 오른쪽 갭이 남았다"
    # 실측과 같은 자리에서 여유가 모자란 게 맞는지도 확인
    assert p._corridor_clear_distance(rng, ang, d30) < p.gap_fit_min_m


def test_both_open_gaps_survive():
    """둘 다 뚫려 있으면 아무것도 안 뺀다 — 판단은 기존 로직 몫이다."""
    ang, rng = _scan([((math.radians(-5.0), math.radians(5.0)), 1.2)])
    right = np.where(ang <= math.radians(-15.0))[0]
    left = np.where(ang >= math.radians(15.0))[0]
    kept = _Stub().fit([right, left], rng, ang)
    assert len(kept) == 2


def test_when_nothing_fits_the_list_is_untouched():
    """전부 막혔으면 원래 목록을 돌려준다. 못 지나갈 때는 감속·AEB 몫이다."""
    ang, rng = _scan([((-math.pi, math.pi), 0.5)])
    right = np.where(ang <= math.radians(-15.0))[0]
    left = np.where(ang >= math.radians(15.0))[0]
    kept = _Stub().fit([right, left], rng, ang)
    assert len(kept) == 2


def test_a_single_gap_is_never_filtered():
    """후보가 하나면 걸러 봐야 고를 게 없다."""
    ang, rng = _scan([((-math.pi, math.pi), 0.5)])
    only = np.where(ang >= math.radians(15.0))[0]
    assert _Stub().fit([only], rng, ang) == [only]


def test_the_check_can_be_turned_off():
    ang, rng = _scan([((math.radians(-45.0), math.radians(-15.0)), 0.6)])
    right = np.where((ang >= math.radians(-45.0)) & (ang <= math.radians(-15.0)))[0]
    left = np.where(ang >= math.radians(15.0))[0]
    kept = _Stub(enable=False).fit([right, left], rng, ang)
    assert len(kept) == 2


def test_the_scan_stops_early_once_a_gap_clearly_fits():
    """합격이 확인되면 남은 각도는 안 훑는다 — 스캔마다 도는 계산이라 중요하다."""
    ang, rng = _scan([])

    p = _Stub()
    calls = []
    real = p._corridor_clear_distance

    def counted(ranges, wrapped, angle):
        calls.append(angle)
        return real(ranges, wrapped, angle)

    p._corridor_clear_distance = counted
    got = p._gap_best_clear_m(rng, ang, -0.5, 0.5, p.gap_fit_min_m)
    assert got >= p.gap_fit_min_m
    assert len(calls) == 1


def test_the_fit_is_judged_on_the_range_we_can_actually_aim_at():
    """좁혀지기 전 범위로 재면 못 쓰는 각도로 합격시킨다.

    실측에서 갭 필터를 넣고도 차폭 경고가 그대로 나왔던 이유다. 갭 원래
    폭에는 뚫린 각도가 있는데, 가장자리 여유와 속도 연동 FOV 로 잘리고 나면
    그 각도가 범위 밖이라 조준할 수가 없다.
    """
    # 25° 밖만 뚫려 있고 안쪽은 막힌 갭
    ang, rng = _scan([((math.radians(-25.0), math.radians(25.0)), 0.6)])
    gap = np.where(np.ones(ang.shape, dtype=bool))[0]

    wide = _Stub(fov_deg=90.0)
    narrow = _Stub(fov_deg=20.0)  # ±20° 로 좁혀지면 뚫린 각도가 잘려 나간다

    lo_w, hi_w = wide._aim_range(ang[0], ang[-1], None, 0.0)
    lo_n, hi_n = narrow._aim_range(ang[0], ang[-1], None, 0.0)
    assert hi_n < hi_w

    assert wide._gap_best_clear_m(rng, ang, lo_w, hi_w, 1.0) >= 1.0
    assert narrow._gap_best_clear_m(rng, ang, lo_n, hi_n, 1.0) < 1.0
