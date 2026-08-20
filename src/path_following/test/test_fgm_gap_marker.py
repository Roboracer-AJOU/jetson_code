"""Foxglove 에서 갭 V 가 보이게.

예전에 토픽은 40 Hz 로 나가는데 화면만 비어 있었다.

  1. pose.orientation = (0,0,0,0) — Foxglove 가 드롭, RViz 는 관대.
  2. 점이 z=0 — 맵 OccupancyGrid 평면에 묻힘. 큐브 장애물은 높이가 있어 보임.
  3. FGM OFF 때 DELETE — 같은 ns/id 의 다음 ADD 를 Foxglove 가 무시.
  4. 단일 Marker 토픽 — Scene 이 MarkerArray 만 그리는 레이아웃이 있음.

    python3 -m pytest src/path_following/test/test_fgm_gap_marker.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path as FsPath

from std_msgs.msg import Header
from visualization_msgs.msg import Marker

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following.fgm_node import FGMNode  # noqa: E402


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Viz:
    _fill_marker_pose = FGMNode._fill_marker_pose
    publish_gap_marker_angles = FGMNode.publish_gap_marker_angles

    def __init__(self):
        self._laser_frame = "laser"
        self._use_full_scan_fov = False
        self.fov_angle = math.radians(90.0)
        self.gap_marker_arm_scale = 1.5
        self.gap_marker_max_arm_m = 2.0
        self.gap_marker_z_m = 0.15
        self.preprocess_dist = 10.0
        self.gap_marker_pub = _Pub()
        self.gap_markers_pub = _Pub()
        self._last_gap_marker = None


def _stamp():
    return Header().stamp


def test_the_quaternion_is_valid():
    m = Marker()
    _Viz()._fill_marker_pose(m)
    assert m.pose.orientation.w == 1.0
    assert m.pose.orientation.x == 0.0


def test_the_marker_never_expires():
    """뷰어 시계가 젯슨과 어긋나면 lifetime 이 있는 마커는 즉시 만료된다."""
    m = Marker()
    _Viz()._fill_marker_pose(m)
    assert m.lifetime.sec == 0 and m.lifetime.nanosec == 0


def test_the_v_sits_above_the_map():
    v = _Viz()
    v.publish_gap_marker_angles(0.3, -0.3, 2.0, 2.0, _stamp())
    m = v.gap_marker_pub.msgs[-1]
    assert m.points
    assert all(abs(p.z - 0.15) < 1e-9 for p in m.points)


def test_it_is_a_strip_so_foxglove_will_draw_it():
    v = _Viz()
    v.publish_gap_marker_angles(0.4, -0.2, 1.5, 1.5, _stamp())
    m = v.gap_marker_pub.msgs[-1]
    assert m.type == Marker.LINE_STRIP
    assert len(m.points) == 3


def test_the_array_topic_gets_the_same_v():
    v = _Viz()
    v.publish_gap_marker_angles(0.1, -0.1, 1.0, 1.0, _stamp())
    assert v.gap_markers_pub.msgs
    arr = v.gap_markers_pub.msgs[-1]
    assert len(arr.markers) == 1
    assert arr.markers[0].type == Marker.LINE_STRIP


def test_the_arms_are_capped():
    v = _Viz()
    v.publish_gap_marker_angles(0.0, math.pi / 2, 20.0, 20.0, _stamp())
    pts = v.gap_marker_pub.msgs[-1].points
    # start, origin, end — origin at 0, arms ≤ 2 m
    assert math.hypot(pts[0].x, pts[0].y) <= 2.0 + 1e-9
    assert math.hypot(pts[2].x, pts[2].y) <= 2.0 + 1e-9
    assert abs(pts[1].x) < 1e-9 and abs(pts[1].y) < 1e-9
