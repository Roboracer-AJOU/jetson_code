"""회피하기로 했으면 한 프레임 실패로 라인에 돌아가지 않는다.

회피 경로 생성은 FGM 조준·TF·잘림 판정이 겹쳐 있어 한 프레임쯤은 쉽게
실패한다. 그런데 그 한 번에 `avoid_retry_sec`(0.5 s) 래치가 걸리고, 그동안
모드가 GLOBAL 로 내려가 CSV 를 탔다. 5 m/s 면 장애물을 향해 2.5 m 직진이다.

실주행에서 "로컬패스로 갔다가 갑자기 글로벌패스로" 보이던 게 이것이고,
회피를 시작해 놓고 라인으로 돌아가니 오히려 박는다.

    python3 -m pytest src/path_following/test/test_avoid_commit.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path as FsPath

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following.local_planner_node import CFG, LocalPlannerNode  # noqa: E402

TH = CFG["avoid_blocked_frames_th"]


class _Clock:
    def __init__(self):
        self.t = 0

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return self.t


class _Path:
    def __init__(self, n=5):
        self.poses = list(range(n))


class _Node:
    _mark_avoid_blocked = LocalPlannerNode._mark_avoid_blocked
    _clear_avoid_blocked = LocalPlannerNode._clear_avoid_blocked
    _avoid_blocked = LocalPlannerNode._avoid_blocked
    _avoid_give_up = LocalPlannerNode._avoid_give_up
    _hold_last_avoid_path = LocalPlannerNode._hold_last_avoid_path

    def __init__(self, **kw):
        self._clock = _Clock()
        self.avoid_retry_ns = int(CFG["avoid_retry_sec"] * 1e9)
        self._avoid_blocked_until_ns = 0
        self.avoid_blocked_frames_th = kw.get("th", TH)
        self._avoid_blocked_frames = 0
        self._last_good_avoid_path = None
        self._last_good_avoid_ns = 0
        self.avoid_hold_max_ns = int(CFG["avoid_hold_max_sec"] * 1e9)
        self.published = []
        self.gate = []

    def get_clock(self):
        return self._clock

    class _Pub:
        def __init__(self, sink):
            self.sink = sink

        def publish(self, m):
            self.sink.append(m)

    @property
    def pub_path(self):
        return _Node._Pub(self.published)

    def _publish_override_gate(self, on):
        self.gate.append(bool(on))

    def tick(self, ms):
        self._clock.t += int(ms * 1e6)


# ------------------------------------------------- 한 프레임은 포기가 아니다


def test_one_bad_frame_does_not_give_up():
    n = _Node()
    n._mark_avoid_blocked()
    assert n._avoid_blocked(), "래치는 걸린다"
    assert not n._avoid_give_up(), "그렇다고 회피를 접지는 않는다"


def test_it_gives_up_only_after_enough_frames():
    n = _Node()
    for i in range(1, TH):
        n._mark_avoid_blocked()
        assert not n._avoid_give_up(), f"{i} 프레임에서 접으면 안 된다"
    n._mark_avoid_blocked()
    assert n._avoid_give_up()


def test_a_success_resets_the_count():
    n = _Node()
    for _ in range(TH - 1):
        n._mark_avoid_blocked()
    n._clear_avoid_blocked()
    assert n._avoid_blocked_frames == 0
    n._mark_avoid_blocked()
    assert not n._avoid_give_up()


def test_regression_the_old_rule_gave_up_immediately():
    """예전 조건(`_avoid_blocked()` 단독)은 첫 실패에 바로 참이었다."""
    n = _Node()
    n._mark_avoid_blocked()
    assert n._avoid_blocked() and not n._avoid_give_up()


# ------------------------------------------- 접기 전에는 직전 경로를 붙든다


def test_it_holds_the_last_path_through_a_glitch():
    n = _Node()
    n._last_good_avoid_path = _Path()
    n._last_good_avoid_ns = n._clock.t
    n._mark_avoid_blocked()
    assert n._hold_last_avoid_path() is True
    assert len(n.published) == 1
    assert n.gate == [True], "게이트를 내리면 Stanley 가 CSV 로 돌아간다"


def test_it_stops_holding_once_it_gives_up():
    n = _Node()
    n._last_good_avoid_path = _Path()
    n._last_good_avoid_ns = n._clock.t
    for _ in range(TH):
        n._mark_avoid_blocked()
    assert n._hold_last_avoid_path() is False
    assert n.published == []


def test_a_stale_path_is_not_held():
    """차가 이미 지나친 경로는 뒤를 가리킨다."""
    n = _Node()
    n._last_good_avoid_path = _Path()
    n._last_good_avoid_ns = n._clock.t
    n.tick(CFG["avoid_hold_max_sec"] * 1000 + 10)
    n._mark_avoid_blocked()
    assert n._hold_last_avoid_path() is False


def test_nothing_to_hold_before_the_first_good_path():
    n = _Node()
    n._mark_avoid_blocked()
    assert n._hold_last_avoid_path() is False


def test_the_hold_window_covers_the_give_up_window():
    """붙드는 시간이 포기 판정보다 짧으면 그 사이 CSV 로 튄다."""
    assert CFG["avoid_hold_max_sec"] >= TH / 40.0
