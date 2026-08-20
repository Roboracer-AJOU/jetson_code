"""저속에서 로컬패스로 갈아타는 시점만 늦춘다.

FGM 은 그대로 켜 둔다 — 늦추는 건 CSV → local_path 전환뿐이다. 저속에서
일찍 갈아타 봐야 장애물이 아직 멀어 조준이 흔들리고, 그동안 레이스라인만
놓친다. 고속은 손대지 않는다. 거기서 늦추면 피할 거리가 안 나온다.

    python3 -m pytest src/path_following/test/test_avoid_on_late_lowspeed.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path as FsPath

import pytest

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following.local_planner_node import CFG, LocalPlannerNode  # noqa: E402


class _Gates:
    """게이트 계산만 떼어 온 가짜 플래너."""

    _avoid_on_late_factor = LocalPlannerNode._avoid_on_late_factor
    _speed_scaled_dist = LocalPlannerNode._speed_scaled_dist
    _effective_avoid_gates = LocalPlannerNode._effective_avoid_gates

    def __init__(self, v: float, *, late: bool = True):
        self._ego_speed_mps = v
        self.avoid_on_late_scale = CFG["avoid_on_late_scale"] if late else 1.0
        self.avoid_on_late_max_speed = CFG["avoid_on_late_max_speed"]
        self.avoid_on_late_blend_mps = CFG["avoid_on_late_blend_mps"]
        self.avoid_timing_margin = CFG["avoid_timing_margin"]
        self.avoid_timing_ref_mps = CFG["avoid_timing_ref_mps"]
        for name in (
            "avoid_on_m",
            "avoid_on_min_m",
            "avoid_on_max_m",
            "avoid_off_m",
            "avoid_off_min_m",
            "avoid_off_max_m",
            "fgm_enable_m",
            "fgm_enable_min_m",
            "fgm_enable_max_m",
        ):
            setattr(self, name, CFG[name])


def _on(v: float, **kw) -> float:
    return _Gates(v, **kw)._effective_avoid_gates()[0]


def _fgm(v: float, **kw) -> float:
    return _Gates(v, **kw)._effective_avoid_gates()[2]


def test_low_speed_switches_thirty_percent_later():
    for v in (1.0, 2.0, 3.0, 4.0):
        assert _on(v) == pytest.approx(0.7 * _on(v, late=False)), v


def test_high_speed_is_untouched():
    for v in (5.0, 6.0, 7.0):
        assert _on(v) == pytest.approx(_on(v, late=False)), v


def test_fgm_still_turns_on_at_the_same_distance():
    """켜지는 건 그대로 두라고 했다 — 늦추는 건 전환 시점뿐이다."""
    for v in (1.0, 3.0, 4.0, 6.0):
        assert _fgm(v) == pytest.approx(_fgm(v, late=False)), v


def test_the_switch_is_still_before_fgm_turns_on():
    """FGM 이 켜지기도 전에 갈아타면 목표가 없다."""
    for v in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0):
        on, _off, fgm = _Gates(v)._effective_avoid_gates()
        assert on <= fgm + 1e-9, v


def test_off_still_sits_outside_on():
    """이력 구간이 뒤집히면 AVOID 가 떤다."""
    for v in (1.0, 3.0, 4.5, 6.0):
        on, off, _ = _Gates(v)._effective_avoid_gates()
        assert off > on, v


def test_the_factor_comes_on_without_a_step():
    xs = [3.5 + 0.05 * k for k in range(40)]
    ys = [_Gates(v)._avoid_on_late_factor() for v in xs]
    assert max(abs(b - a) for a, b in zip(ys, ys[1:])) < 0.06
    assert ys[0] == pytest.approx(0.7)
    assert ys[-1] == pytest.approx(1.0)


def test_it_still_leaves_room_to_stop_at_low_speed():
    """늦춰도 그 거리에서 설 수는 있어야 한다 (a=3, 반응 0.15s)."""
    for v in (1.0, 2.0, 3.0, 4.0):
        assert _on(v) > v * 0.15 + v * v / 6.0, v
