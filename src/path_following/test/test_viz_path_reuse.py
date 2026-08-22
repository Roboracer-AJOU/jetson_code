#!/usr/bin/env python3
"""시각화 Path 재사용 — 싸졌지만 **내용은 그대로**여야 한다.

포즈 객체를 돌려쓰는 최적화라, 길이가 줄었을 때 옛 꼬리가 남거나 좌표
갱신을 빠뜨리면 Foxglove 에 지난 경로가 그려진다. 그건 디버깅을 망친다.
"""
from __future__ import annotations

import sys
from pathlib import Path as FsPath

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from builtin_interfaces.msg import Time  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402

from path_following.local_planner_node import LocalPlannerNode  # noqa: E402
from path_following.stanley_waypoint_follow_node import (  # noqa: E402
    StanleyWaypointFollowNode,
)


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        # 발행 시점의 좌표를 떠 둔다 — 나중에 덮어써도 남게
        self.sent.append(
            (msg.header.stamp.sec, [(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        )


class _Clock:
    def __init__(self):
        self.sec = 0

    def now(self):
        self.sec += 1
        outer = self

        class _T:
            @staticmethod
            def to_msg():
                t = Time()
                t.sec = outer.sec
                return t

        return _T()


class _Stanley:
    _publish_tracked_path = StanleyWaypointFollowNode._publish_tracked_path

    def __init__(self):
        self.tracked_path_pub = _Pub()
        self.map_frame = "map"
        self._path_poses = []
        self._tracked_path_msg = Path()
        self._tracked_pose_pool = []
        self._clock = _Clock()

    def get_clock(self):
        return self._clock


def test_the_tracked_path_carries_the_current_window():
    s = _Stanley()
    s._path_poses = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]
    s._publish_tracked_path()
    assert s.tracked_path_pub.sent[0][1] == [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]


def test_a_new_window_fully_replaces_the_old_one():
    """풀을 돌려쓰므로 갱신을 빠뜨리면 지난 프레임 좌표가 남는다."""
    s = _Stanley()
    s._path_poses = [(1.0, 1.0)] * 5
    s._publish_tracked_path()
    s._path_poses = [(9.0, 9.0)] * 5
    s._publish_tracked_path()
    assert s.tracked_path_pub.sent[1][1] == [(9.0, 9.0)] * 5


def test_a_shorter_window_does_not_leave_a_tail():
    """길이가 줄었는데 리스트를 안 줄이면 옛 꼬리가 그대로 나간다."""
    s = _Stanley()
    s._path_poses = [(float(i), 0.0) for i in range(10)]
    s._publish_tracked_path()
    s._path_poses = [(100.0, 0.0), (101.0, 0.0), (102.0, 0.0)]
    s._publish_tracked_path()
    got = s.tracked_path_pub.sent[1][1]
    assert len(got) == 3, f"꼬리가 남았다: {len(got)}점"
    assert got == [(100.0, 0.0), (101.0, 0.0), (102.0, 0.0)]


def test_a_longer_window_grows_the_pool():
    s = _Stanley()
    s._path_poses = [(1.0, 0.0), (2.0, 0.0)]
    s._publish_tracked_path()
    s._path_poses = [(float(i), 1.0) for i in range(200)]
    s._publish_tracked_path()
    got = s.tracked_path_pub.sent[1][1]
    assert len(got) == 200
    assert got[0] == (0.0, 1.0) and got[-1] == (199.0, 1.0)


def test_every_pose_shares_the_path_header_stamp():
    """헤더 공유가 깨지면 포즈 스탬프가 첫 프레임에 얼어붙는다."""
    s = _Stanley()
    s._path_poses = [(1.0, 1.0), (2.0, 2.0)]
    s._publish_tracked_path()
    s._publish_tracked_path()
    msg = s._tracked_path_msg
    assert all(p.header is msg.header for p in msg.poses)
    assert msg.poses[0].header.stamp.sec == msg.header.stamp.sec


def test_too_short_a_path_publishes_nothing():
    s = _Stanley()
    s._path_poses = [(1.0, 1.0)]
    s._publish_tracked_path()
    assert s.tracked_path_pub.sent == []


class _Planner:
    _publish_csv_track_viz = LocalPlannerNode._publish_csv_track_viz

    def __init__(self, pts, stride=1):
        self.pub_csv_track = _Pub()
        self.map_frame = "map"
        self.points = pts
        self._csv_viz_stride = stride
        self._csv_viz_msg = None
        self._clock = _Clock()

    def get_clock(self):
        return self._clock


def test_the_csv_viz_matches_the_track():
    pts = [(float(i), float(i) * 0.5) for i in range(20)]
    p = _Planner(pts)
    p._publish_csv_track_viz()
    assert p.pub_csv_track.sent[0][1] == pts


def test_the_csv_viz_respects_the_stride():
    pts = [(float(i), 0.0) for i in range(20)]
    p = _Planner(pts, stride=5)
    p._publish_csv_track_viz()
    assert p.pub_csv_track.sent[0][1] == [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (15.0, 0.0)]


def test_the_csv_viz_is_built_once_but_restamped():
    """내용은 고정, 스탬프는 매번 새로 — 안 그러면 Foxglove 가 오래됐다고 버린다."""
    pts = [(float(i), 0.0) for i in range(10)]
    p = _Planner(pts)
    p._publish_csv_track_viz()
    first = p._csv_viz_msg
    p._publish_csv_track_viz()
    assert p._csv_viz_msg is first, "고정 경로를 다시 만들었다"
    s0, xy0 = p.pub_csv_track.sent[0]
    s1, xy1 = p.pub_csv_track.sent[1]
    assert xy0 == xy1
    assert s1 > s0, "스탬프가 안 갱신됐다"


def test_the_csv_viz_needs_at_least_two_points():
    p = _Planner([(0.0, 0.0)])
    p._publish_csv_track_viz()
    assert p.pub_csv_track.sent == []
