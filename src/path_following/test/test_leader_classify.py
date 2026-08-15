#!/usr/bin/env python3
"""앞차 분류 (TRAILING 대상인가 AVOID 대상인가) 단위 테스트.

    python3 -m pytest src/path_following/test/test_leader_classify.py -q
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.local_planner_node import CFG, LocalPlannerNode  # noqa: E402


class _Stub:
    """_has_followable_leader / _leader_too_slow 가 쓰는 것만 갖춘 가짜 노드."""

    def __init__(self, gap: float, vs: float, v_csv: float, **kw):
        self._gap = gap
        self._vs = vs
        self._v_csv = v_csv
        self.trailing_min_leader_speed_mps = kw.get("min_leader", 0.5)
        self.trailing_speed_deficit_enable = kw.get("deficit_enable", False)
        self.trailing_max_speed_deficit_mps = kw.get("max_deficit", 1.5)

    def _forward_leader(self):
        return self._gap, self._vs

    def _csv_speed_now(self):
        return self._v_csv

    _leader_too_slow = LocalPlannerNode._leader_too_slow
    _has_followable_leader = LocalPlannerNode._has_followable_leader


def test_defaults():
    # 레이싱이므로 기본 ON — 우리보다 이만큼 느린 앞차는 따라가지 않고
    # 정적 장애물처럼 비켜 간다.
    assert CFG["trailing_speed_deficit_enable"] is True
    assert CFG["trailing_max_speed_deficit_mps"] == 0.5


def test_no_leader_when_nothing_ahead():
    assert _Stub(float("inf"), 3.0, 5.0)._has_followable_leader() is False


def test_stopped_and_oncoming_go_to_avoid():
    """서 있는 차·마주 오는 차는 따라갈 대상이 아니다 (플래그와 무관)."""
    for vs in (0.0, 0.3, -2.0):
        assert _Stub(2.0, vs, 5.0)._has_followable_leader() is False


def test_slow_leader_is_followed_when_flag_off():
    """기존 동작 — 0.5 m/s 만 넘으면 우리보다 한참 느려도 따라간다."""
    s = _Stub(2.0, 1.0, 5.0, deficit_enable=False)
    assert s._has_followable_leader() is True


def test_slow_leader_goes_to_avoid_when_flag_on():
    """켜면 같은 상황에서 AVOID 로 넘어간다 (5.0 - 1.0 = 4.0 > 1.5)."""
    s = _Stub(2.0, 1.0, 5.0, deficit_enable=True, max_deficit=1.5)
    assert s._has_followable_leader() is False


def test_similar_speed_leader_is_still_followed_when_flag_on():
    """비슷한 속도면 켜도 따라간다 — 이게 켰을 때의 의도다."""
    s = _Stub(2.0, 4.5, 5.0, deficit_enable=True, max_deficit=1.5)
    assert s._has_followable_leader() is True


def test_threshold_boundary():
    # deficit == 임계는 '넘지 않음' → 계속 따라간다
    assert _Stub(2.0, 3.5, 5.0, deficit_enable=True)._has_followable_leader() is True
    # 임계를 조금 넘으면 비켜 간다
    assert _Stub(2.0, 3.4, 5.0, deficit_enable=True)._has_followable_leader() is False


def test_faster_leader_is_never_too_slow():
    """앞차가 우리보다 빠르면 deficit 이 음수 — 당연히 따라간다."""
    s = _Stub(2.0, 6.0, 5.0, deficit_enable=True)
    assert s._leader_too_slow(6.0) is False
    assert s._has_followable_leader() is True
