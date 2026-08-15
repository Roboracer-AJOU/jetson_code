#!/usr/bin/env python3
"""회피 막힘 래치 + AEB 탈출 모드 단위 테스트.

    python3 -m pytest src/path_following/test/test_avoid_escape.py -q
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from std_msgs.msg import Bool  # noqa: E402

from path_following.local_planner_node import CFG, LocalPlannerNode  # noqa: E402

HZ = 40.0
TICK_NS = int(1e9 / HZ)


class _Clock:
    def __init__(self):
        self.ns = 1_000_000_000

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return self.ns

    def tick(self, n: int = 1):
        self.ns += TICK_NS * n


class _Log:
    def __init__(self):
        self.lines: list[str] = []

    def _rec(self, m):
        self.lines.append(str(m))

    info = warn = error = _rec


class _Stub:
    """래치/탈출 로직이 건드리는 것만 갖춘 가짜 플래너."""

    def __init__(self, **kw):
        self._clock = _Clock()
        self._log = _Log()
        self.avoid_retry_ns = int(kw.get("retry_sec", 0.5) * 1e9)
        self._avoid_blocked_until_ns = 0
        self.aeb_escape_enable = kw.get("escape_enable", True)
        self.aeb_escape_arm_speed = kw.get("arm_speed", 0.20)
        self.aeb_escape_hold_ns = int(kw.get("hold_sec", 2.0) * 1e9)
        self.aeb_escape_speed_mps = kw.get("escape_speed", 0.8)
        self._aeb_escape_until_ns = 0
        self._aeb_escape_logged = False
        self._aeb_active = False
        self._aeb_count = 0
        self._ego_speed_mps = 0.0

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._log

    _avoid_blocked = LocalPlannerNode._avoid_blocked
    _mark_avoid_blocked = LocalPlannerNode._mark_avoid_blocked
    _clear_avoid_blocked = LocalPlannerNode._clear_avoid_blocked
    _cb_aeb = LocalPlannerNode._cb_aeb
    _aeb_escape_active = LocalPlannerNode._aeb_escape_active
    _log_aeb_escape = LocalPlannerNode._log_aeb_escape


# ------------------------------------------------------- 문제 1: 래치


def test_defaults():
    assert CFG["avoid_retry_sec"] == 0.5
    assert CFG["aeb_escape_enable"] is True
    assert CFG["aeb_escape_min_path_m"] < CFG["path_check_min_length_m"]


def test_blocked_latch_holds_for_retry_period():
    """막힌 뒤 재시도 주기 동안 계속 True 여야 한다 (모드 떨림 방지의 핵심)."""
    s = _Stub(retry_sec=0.5)
    s._mark_avoid_blocked()
    held = 0
    for _ in range(40):  # 1.0 s
        if not s._avoid_blocked():
            break
        held += 1
        s._clock.tick()
    # 0.5 s = 20 틱
    assert held == 20


def test_regression_bool_flag_would_flap():
    """예전 거동 재현 — bool 이면 한 프레임 뒤 이미 풀려 있다.

    래치가 없으면 AVOID→TRAILING 전이에서 리셋돼 다음 프레임에 바로
    TRAILING→AVOID 로 튕긴다. 그 왕복이 AEB 완화 기준까지 깜빡이게 했다.
    """
    s = _Stub(retry_sec=0.0)  # retry=0 이 곧 예전 bool 거동
    s._mark_avoid_blocked()
    assert s._avoid_blocked() is False


def test_latch_can_be_cleared_on_success():
    s = _Stub()
    s._mark_avoid_blocked()
    assert s._avoid_blocked() is True
    s._clear_avoid_blocked()
    assert s._avoid_blocked() is False


# ------------------------------------------------------- 문제 2: 탈출


def test_no_escape_before_aeb():
    assert _Stub()._aeb_escape_active() is False


def test_escape_waits_until_actually_stopped():
    """제동 중 고속에서는 경로를 바꾸지 않는다 (조향 급변 방지)."""
    s = _Stub(arm_speed=0.20)
    s._cb_aeb(Bool(data=True))
    s._ego_speed_mps = 2.0
    assert s._aeb_escape_active() is False
    s._ego_speed_mps = 0.05
    assert s._aeb_escape_active() is True


def test_escape_persists_after_aeb_release():
    """AEB 가 풀린 뒤에도 hold 동안 유지돼야 빠져나갈 시간이 있다."""
    s = _Stub(hold_sec=2.0)
    s._cb_aeb(Bool(data=True))
    s._ego_speed_mps = 0.0
    assert s._aeb_escape_active() is True

    s._cb_aeb(Bool(data=False))
    s._ego_speed_mps = 0.6  # 빠져나가는 중 — arm_speed 를 넘어도 유지된다
    held = 0
    for _ in range(120):  # 3 s
        if not s._aeb_escape_active():
            break
        held += 1
        s._clock.tick()
    assert held == 80  # 2.0 s = 80 틱


def test_escape_counts_aeb_and_logs_edges():
    s = _Stub()
    s._ego_speed_mps = 0.0
    s._cb_aeb(Bool(data=True))
    assert s._aeb_count == 1
    s._log_aeb_escape(s._aeb_escape_active())
    assert "탈출 모드 진입" in "".join(s._log.lines)

    s._cb_aeb(Bool(data=False))
    s._clock.tick(200)  # hold 경과
    s._log_aeb_escape(s._aeb_escape_active())
    assert "탈출 모드 종료" in "".join(s._log.lines)


def test_escape_disabled_flag_restores_old_behavior():
    s = _Stub(escape_enable=False)
    s._ego_speed_mps = 0.0
    s._cb_aeb(Bool(data=True))
    assert s._aeb_escape_active() is False
    s._cb_aeb(Bool(data=False))
    assert s._aeb_escape_until_ns == 0, "꺼져 있으면 창을 열지 않는다"
    assert s._aeb_escape_active() is False


def test_repeated_aeb_extends_escape():
    """탈출 중 AEB 가 또 걸려도 해제 시 창이 다시 열려야 한다."""
    s = _Stub(hold_sec=1.0)
    s._ego_speed_mps = 0.0
    s._cb_aeb(Bool(data=True))
    s._cb_aeb(Bool(data=False))
    first = s._aeb_escape_until_ns
    s._clock.tick(20)
    s._cb_aeb(Bool(data=True))
    s._cb_aeb(Bool(data=False))
    assert s._aeb_escape_until_ns > first
