#!/usr/bin/env python3
"""장애물 마커 풀 — 객체를 돌려써도 화면은 그대로여야 한다."""
from __future__ import annotations

import sys
from pathlib import Path as FsPath

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from builtin_interfaces.msg import Duration as MsgDuration  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from path_following.integrated_obstacle_node import (  # noqa: E402
    IntegratedObstacleNode,
)


class _N:
    _track_marker = IntegratedObstacleNode._track_marker

    def __init__(self):
        self._laser_frame = "laser"
        self._marker_pool = []
        self._marker_lifetime = MsgDuration(sec=0, nanosec=200000000)


def test_the_invariant_fields_are_set_once():
    n = _N()
    m = n._track_marker(0)
    assert m.header.frame_id == "laser"
    assert m.ns == "integrated_obstacles"
    assert m.type == Marker.CUBE
    assert m.action == Marker.ADD
    assert m.scale.z == 0.2
    assert m.color.a == 0.8
    assert m.lifetime.nanosec == 200000000


def test_the_same_slot_hands_back_the_same_object():
    """이게 풀의 요점이다 — 새로 만들면 최적화가 없어진 것이다."""
    n = _N()
    assert n._track_marker(3) is n._track_marker(3)
    assert len(n._marker_pool) == 4


def test_different_slots_are_different_objects():
    """같은 객체를 두 번 넣으면 마커 하나만 그려진다."""
    n = _N()
    a, b = n._track_marker(0), n._track_marker(1)
    assert a is not b
    a.pose.position.x = 5.0
    assert b.pose.position.x == 0.0


def test_the_pool_grows_to_fit_and_keeps_earlier_slots():
    n = _N()
    first = n._track_marker(0)
    n._track_marker(0).id = 77
    n._track_marker(9)
    assert len(n._marker_pool) == 10
    assert n._track_marker(0) is first
    assert n._track_marker(0).id == 77, "풀이 커지면서 앞 슬롯이 날아갔다"


def test_every_marker_shares_one_lifetime_object():
    """수명은 상수다. 아무도 안 고치므로 나눠 써도 된다."""
    n = _N()
    assert n._track_marker(0).lifetime is n._track_marker(1).lifetime
