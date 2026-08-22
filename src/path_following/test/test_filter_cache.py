#!/usr/bin/env python3
"""주기당 장애물 필터 캐시 — 재사용은 하되 낡은 값을 주면 안 된다.

캐시가 틀리면 차가 **지난 프레임의 장애물** 을 보고 판단한다. 그건 느려지는
게 아니라 잘못 가는 것이라, 속도보다 이쪽을 먼저 묶는다.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.local_planner_node import LocalPlannerNode  # noqa: E402


class _N:
    _filter_cached = LocalPlannerNode._filter_cached

    def __init__(self):
        self._tf_cycle_id = 0
        self._filter_cache = {}
        self._filter_cache_cycle = -1
        self.builds = 0

    def run(self, tag, raw, out):
        def build():
            self.builds += 1
            return out

        return self._filter_cached(tag, raw, build)


def test_the_same_input_inside_one_cycle_is_built_once():
    n = _N()
    raw = [1.0, 2.0, 3.0, 0.4]
    assert n.run("static_gate", raw, ["A"]) == ["A"]
    assert n.run("static_gate", raw, ["B"]) == ["A"], "두 번째가 캐시를 안 썼다"
    assert n.builds == 1


def test_a_new_cycle_rebuilds():
    """주기가 넘어갔는데 재사용하면 지난 프레임으로 주행한다."""
    n = _N()
    raw = [1.0, 2.0, 3.0, 0.4]
    n.run("static_gate", raw, ["A"])
    n._tf_cycle_id += 1
    assert n.run("static_gate", raw, ["B"]) == ["B"]
    assert n.builds == 2


def test_new_data_in_the_same_cycle_rebuilds():
    """구독 콜백이 새 리스트를 넣었으면 주기가 같아도 다시 만들어야 한다."""
    n = _N()
    n.run("static_gate", [1.0, 2.0, 3.0, 0.4], ["A"])
    assert n.run("static_gate", [9.0, 8.0, 7.0, 0.5], ["B"]) == ["B"]
    assert n.builds == 2


def test_equal_but_distinct_lists_are_not_confused():
    """내용이 같아도 다른 객체면 새 데이터로 본다 — 안전한 쪽으로 틀린다."""
    n = _N()
    a = [1.0, 2.0, 3.0, 0.4]
    b = [1.0, 2.0, 3.0, 0.4]
    n.run("static_gate", a, ["A"])
    assert n.run("static_gate", b, ["B"]) == ["B"]


def test_the_four_filters_do_not_share_a_slot():
    """게이트용과 해제용은 기준이 달라 결과도 다르다. 섞이면 안 된다."""
    n = _N()
    raw = [1.0, 2.0, 3.0, 0.4]
    assert n.run("static_gate", raw, ["gate"]) == ["gate"]
    assert n.run("static_exit", raw, ["exit"]) == ["exit"]
    assert n.run("dynamic_gate", raw, ["dgate"]) == ["dgate"]
    assert n.run("dynamic_exit", raw, ["dexit"]) == ["dexit"]
    assert n.builds == 4
    # 같은 주기에 다시 물어보면 각자 제 값이 나와야 한다
    assert n.run("static_gate", raw, ["x"]) == ["gate"]
    assert n.run("dynamic_exit", raw, ["x"]) == ["dexit"]
    assert n.builds == 4


def test_an_empty_result_is_cached_too():
    """[] 도 유효한 답이다 — falsy 라고 다시 만들면 캐시가 무의미해진다."""
    n = _N()
    raw = [1.0, 2.0, 3.0, 0.4]
    assert n.run("static_gate", raw, []) == []
    assert n.run("static_gate", raw, ["나중값"]) == []
    assert n.builds == 1


def test_the_cache_does_not_grow_across_cycles():
    n = _N()
    for i in range(200):
        n._tf_cycle_id += 1
        n.run("static_gate", [float(i)], ["x"])
    assert len(n._filter_cache) <= 4
