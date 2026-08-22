"""시각화 게이트: 듣는 데가 없으면 발행을 건너뛰는가.

중요한 건 두 가지다.
  1) 구독자가 붙으면 예전과 **똑같은 내용** 이 나간다 (시각화가 죽으면 안 된다)
  2) 제어 경로 토픽에는 게이트가 안 걸려 있다 (거동이 달라지면 안 된다)
"""
from __future__ import annotations

import copy
import sys
import types

import pytest

from path_following.viz_gate import has_listener


class _Pub:
    """구독자 수를 마음대로 정할 수 있는 퍼블리셔 대역."""

    def __init__(self, subs: int = 0):
        self.subs = subs
        self.sent: list = []

    def get_subscription_count(self) -> int:
        return self.subs

    def publish(self, msg) -> None:
        # 실제 rclpy 는 이 자리에서 직렬화를 끝낸다. 노드들이 메시지 객체를
        # 재사용하므로(풀링) 참조를 그냥 담으면 나중에 값이 덮여 보인다.
        # 스냅샷을 떠야 미들웨어가 본 것과 같아진다.
        self.sent.append(copy.deepcopy(msg))


class _Broken:
    """get_subscription_count 가 터지는 퍼블리셔."""

    def __init__(self):
        self.sent: list = []

    def get_subscription_count(self):
        raise RuntimeError("미들웨어가 답을 안 준다")

    def publish(self, msg) -> None:
        self.sent.append(msg)


def test_the_gate_is_closed_when_nobody_listens():
    assert has_listener(_Pub(0)) is False


def test_the_gate_is_open_when_someone_listens():
    assert has_listener(_Pub(1)) is True
    assert has_listener(_Pub(7)) is True


def test_a_missing_publisher_is_treated_as_no_listener():
    assert has_listener(None) is False


def test_a_broken_count_falls_back_to_publishing():
    # 조회가 안 될 때 조용히 시각화를 끄면 원인을 못 찾는다. 그냥 낸다.
    assert has_listener(_Broken()) is True


# ------------------------------------------------------------------
# 노드에 실제로 걸린 게이트
# ------------------------------------------------------------------


def test_stanley_skips_the_tracked_path_when_nobody_listens():
    from path_following.stanley_waypoint_follow_node import StanleyWaypointFollowNode

    node = _stanley_stub()
    node.tracked_path_pub = _Pub(0)
    node._path_poses = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    StanleyWaypointFollowNode._publish_tracked_path(node)
    assert node.tracked_path_pub.sent == []
    # 풀도 안 만들었어야 한다 — 게이트가 조립 앞에 있다는 뜻이다
    assert node._tracked_pose_pool == []


def test_stanley_publishes_the_same_tracked_path_when_someone_listens():
    from path_following.stanley_waypoint_follow_node import StanleyWaypointFollowNode

    node = _stanley_stub()
    node.tracked_path_pub = _Pub(1)
    node._path_poses = [(0.0, 0.0), (1.0, 2.0), (3.0, 4.0)]
    StanleyWaypointFollowNode._publish_tracked_path(node)

    assert len(node.tracked_path_pub.sent) == 1
    msg = node.tracked_path_pub.sent[0]
    assert [(p.pose.position.x, p.pose.position.y) for p in msg.poses] == [
        (0.0, 0.0),
        (1.0, 2.0),
        (3.0, 4.0),
    ]
    assert msg.header.frame_id == "map"


def test_stanley_diagnostics_are_gated_per_topic():
    from path_following.stanley_waypoint_follow_node import StanleyWaypointFollowNode

    node = _stanley_stub()
    node.publish_control_diagnostics = True
    node.max_steering = 0.4
    quiet = [_Pub(0) for _ in range(5)]
    node._diag_pubs = tuple(quiet)
    StanleyWaypointFollowNode._publish_control_diagnostics(
        node, 0.1, 0.2, 0.3, 0.4, 0.5
    )
    assert all(p.sent == [] for p in quiet)

    # 가운데 하나(cte)만 듣고 있으면 그것만 나가야 한다
    pubs = [_Pub(0), _Pub(0), _Pub(1), _Pub(0), _Pub(0)]
    node._diag_pubs = tuple(pubs)
    StanleyWaypointFollowNode._publish_control_diagnostics(
        node, 0.1, 0.2, 0.3, 0.4, 0.5
    )
    assert [len(p.sent) for p in pubs] == [0, 0, 1, 0, 0]
    assert pubs[2].sent[0].data == pytest.approx(0.3)


def test_stanley_diagnostics_keep_their_order_and_values():
    """게이트를 달면서 발행 순서/값이 어긋나면 Foxglove 패널이 뒤바뀐다."""
    from path_following.stanley_waypoint_follow_node import StanleyWaypointFollowNode

    node = _stanley_stub()
    node.publish_control_diagnostics = True
    node.max_steering = 0.5
    pubs = [_Pub(1) for _ in range(5)]
    node._diag_pubs = tuple(pubs)
    StanleyWaypointFollowNode._publish_control_diagnostics(
        node, 0.25, -0.5, 1.5, -2.5, 0.75
    )
    got = [p.sent[0].data for p in pubs]
    # 0,1 은 max_steering 으로 정규화된 값, 나머지는 원값
    assert got == pytest.approx([0.5, -1.0, 1.5, -2.5, 0.75])


def test_aeb_gates_ttc_but_never_the_brake():
    from path_following.emergency_brake_node import EmergencyBrakeNode

    node = types.SimpleNamespace(ttc_pub=_Pub(0))
    EmergencyBrakeNode._publish_ttc(node, 1.5)
    assert node.ttc_pub.sent == []

    node.ttc_pub = _Pub(1)
    EmergencyBrakeNode._publish_ttc(node, 1.5)
    assert [m.data for m in node.ttc_pub.sent] == [1.5]

    # 무한대는 예전처럼 -1 로 바뀌어 나가야 한다
    node.ttc_pub = _Pub(1)
    EmergencyBrakeNode._publish_ttc(node, float("inf"))
    assert [m.data for m in node.ttc_pub.sent] == [-1.0]


def test_the_control_path_topics_have_no_gate():
    """제어 경로에 게이트가 새로 붙지 않았는지 소스에서 확인한다.

    이건 성능이 아니라 안전 검사다. 게이트가 하나라도 제어 토픽에 붙으면
    구독자가 늦게 붙는 순간의 첫 메시지를 잃는다.
    """
    import inspect

    from path_following import (
        emergency_brake_node,
        fgm_node,
        local_planner_node,
        stanley_waypoint_follow_node,
    )

    # (모듈, 게이트를 걸면 안 되는 퍼블리셔 속성 이름)
    forbidden = [
        (stanley_waypoint_follow_node, "drive_pub"),
        (local_planner_node, "pub_planner_speed_scale"),
        (local_planner_node, "pub_planner_mode"),
        (local_planner_node, "pub_override_gate"),
        (emergency_brake_node, "brake_pub"),
        (emergency_brake_node, "reverse_pub"),
        (fgm_node, "target_pub"),
    ]
    for mod, attr in forbidden:
        src = inspect.getsource(mod)
        assert f"has_listener(self.{attr})" not in src, (
            f"{mod.__name__}.{attr} 는 제어 경로다 — 게이트를 걸면 안 된다"
        )


def _stanley_stub():
    """__init__ 을 타지 않고 발행 함수만 떼어 쓰기 위한 최소 상태."""
    from nav_msgs.msg import Path

    class _Clock:
        def now(self):
            from builtin_interfaces.msg import Time

            class _T:
                @staticmethod
                def to_msg():
                    return Time()

            return _T()

    node = types.SimpleNamespace()
    node.map_frame = "map"
    node._tracked_path_msg = Path()
    node._tracked_pose_pool = []
    node.get_clock = lambda: _Clock()
    return node


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
