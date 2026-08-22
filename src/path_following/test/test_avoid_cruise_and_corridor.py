"""회피 구간 순항속도 + "그냥 지나가도 되는 장애물" 무시.

두 가지를 지킨다.

1. 피해야 하는 장애물이면 진입 속도로 고속(3 m/s)/저속(2 m/s)을 가르고, 그
   값을 회피가 끝날 때까지 붙든다. 로컬패스로 갈아타는 시점부터 걸려서
   복귀가 끝날 때까지 유지되고, 글로벌패스로 돌아가면 CSV 속도로 복귀한다.
   상한이자 하한이라, 거리 기반 감속이나 이탈 한계가 더 낮게 불러도
   되돌린다 — 급감속은 AEB 뿐이다.

2. 레이스라인을 그대로 타고 가면 차폭에 안 걸리는 장애물은 무시한다. 회피도
   안 하고 감속도 안 한다. 기준은 반폭 0.15 + 슬립 여유 0.03.

    python3 -m pytest src/path_following/test/test_avoid_cruise_and_corridor.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path as FsPath

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.local_planner_node import CFG, LocalPlannerNode  # noqa: E402
from path_following.obstacle_filter import _outside_corridor  # noqa: E402

HIGH = CFG["avoid_cruise_speed_high_mps"]
LOW = CFG["avoid_cruise_speed_low_mps"]
TH = CFG["avoid_cruise_high_speed_th"]


# ------------------------------------------------------- (1) 회피 순항속도


REGRAB = CFG["avoid_cruise_regrab_sec"]


class _Clock:
    """`_avoid_cruise_target` 이 되쓰기 창을 재는 데만 쓴다."""

    def __init__(self):
        self.nanoseconds = 0

    def now(self):
        return self

    def advance(self, sec):
        self.nanoseconds += int(sec * 1e9)


class _Node:
    _avoid_speed_capped = LocalPlannerNode._avoid_speed_capped
    _avoid_cruise_target = LocalPlannerNode._avoid_cruise_target

    def __init__(self, mode="GLOBAL", obstacle_on=False, v=6.0, **kw):
        self.mode = mode
        self._obstacle_on = obstacle_on
        self._ego_speed_mps = v
        self._avoid_cruise_latched = None
        self._avoid_cruise_prev = None
        self._avoid_cruise_release_ns = 0
        self.avoid_cruise_regrab_ns = int(kw.get("regrab", REGRAB) * 1e9)
        self.avoid_cruise_speed_high_mps = kw.get("high", HIGH)
        self.avoid_cruise_speed_low_mps = kw.get("low", LOW)
        self.avoid_cruise_high_speed_th = kw.get("th", TH)
        self.clock = _Clock()

    def get_clock(self):
        return self.clock


def test_the_defaults_are_the_speeds_we_asked_for():
    assert (HIGH, LOW, TH) == (3.0, 2.0, 4.0)


def test_the_cap_is_on_while_avoiding():
    for mode in ("AVOID", "REJOIN"):
        assert _Node(mode)._avoid_speed_capped(), mode


def test_the_cap_starts_when_we_switch_to_the_local_path():
    """로컬패스 전환 게이트(`_obstacle_on`)와 같은 지점에서 걸린다."""
    assert _Node("GLOBAL", obstacle_on=True)._avoid_speed_capped()


def test_a_clear_track_is_not_capped():
    assert not _Node("GLOBAL")._avoid_speed_capped()
    assert not _Node("TRAILING")._avoid_speed_capped()


def test_the_entry_speed_picks_the_target():
    for v, want in ((8.0, HIGH), (4.5, HIGH), (3.9, LOW), (2.0, LOW)):
        assert _Node("AVOID", v=v)._avoid_cruise_target() == want, v


def test_the_threshold_itself_counts_as_slow():
    assert _Node("AVOID", v=TH)._avoid_cruise_target() == LOW


def test_going_back_to_global_releases_it():
    """CSV 속도로 돌아가야 한다."""
    assert _Node("GLOBAL")._avoid_cruise_target() == 0.0


def test_slowing_down_mid_avoidance_does_not_lower_the_target():
    """목표(3.0)가 문턱(4.0)보다 낮다 — 안 붙들면 회피 중에 또 내려간다."""
    n = _Node("AVOID", v=6.0)
    assert n._avoid_cruise_target() == HIGH
    n._ego_speed_mps = HIGH  # 목표까지 감속했다 = 문턱 아래다
    assert n._avoid_cruise_target() == HIGH


def test_the_whole_episode_holds_one_speed():
    """접근 → 회피 → 복귀가 전부 같은 값, 그 뒤 해제."""
    n = _Node("GLOBAL", obstacle_on=True, v=7.0)
    assert n._avoid_cruise_target() == HIGH
    for mode in ("AVOID", "REJOIN"):
        n.mode, n._obstacle_on, n._ego_speed_mps = mode, False, 2.5
        assert n._avoid_cruise_target() == HIGH, mode
    n.mode = "GLOBAL"
    assert n._avoid_cruise_target() == 0.0


def test_the_next_avoidance_gets_its_own_decision():
    """해제되고 한참 지났으면 다음 회피는 그때 속도로 다시 고른다."""
    n = _Node("AVOID", v=7.0)
    assert n._avoid_cruise_target() == HIGH
    n.mode = "GLOBAL"
    assert n._avoid_cruise_target() == 0.0
    n.clock.advance(REGRAB + 0.5)
    n.mode, n._ego_speed_mps = "AVOID", 2.0
    assert n._avoid_cruise_target() == LOW


# --------------------------------- 짧게 끊겼다 다시 켜지면 되쓴다
#
# 래치만으로는 부족하다. 접근 중(모드는 아직 GLOBAL) 검출이 한 프레임
# 깜빡이면 래치가 풀리고, 다음 프레임에 **이미 줄어든 속도로** 다시 고르게
# 되어 3.0 이 2.0 으로 떨어진다. 같은 장애물 하나를 피하는 중이다.


def test_a_dropped_frame_does_not_re_decide_the_target():
    n = _Node("GLOBAL", obstacle_on=True, v=6.0)
    assert n._avoid_cruise_target() == HIGH

    n._ego_speed_mps = 3.5  # 목표를 향해 감속하는 중 = 이제 문턱 아래다
    n.clock.advance(0.05)
    n._obstacle_on = False  # 한 프레임 유실
    assert n._avoid_cruise_target() == 0.0
    n.clock.advance(0.05)
    n._obstacle_on = True
    assert n._avoid_cruise_target() == HIGH, "되쓰지 않으면 2.0 으로 떨어진다"


def test_mode_bouncing_to_global_does_not_re_decide_either():
    """실측으로 나던 AVOID↔GLOBAL 진동에서도 목표는 그대로여야 한다."""
    n = _Node("AVOID", v=6.0)
    assert n._avoid_cruise_target() == HIGH
    n._ego_speed_mps = 3.0
    for _ in range(3):
        n.clock.advance(0.05)
        n.mode = "GLOBAL"
        n._avoid_cruise_target()
        n.clock.advance(0.05)
        n.mode = "AVOID"
        assert n._avoid_cruise_target() == HIGH


def test_the_regrab_window_expires():
    """진짜로 끝난 회피의 값을 무한정 들고 있으면 안 된다."""
    n = _Node("AVOID", v=7.0)
    assert n._avoid_cruise_target() == HIGH
    n.mode = "GLOBAL"
    n._avoid_cruise_target()
    n.clock.advance(REGRAB + 0.01)
    n.mode, n._ego_speed_mps = "AVOID", 2.0
    assert n._avoid_cruise_target() == LOW


def test_releasing_is_immediate_so_csv_speed_comes_back():
    """푸는 건 늦추지 않는다 — 늦추면 글로벌 복귀 후 속도 회복이 늦다."""
    n = _Node("AVOID", v=7.0)
    n._avoid_cruise_target()
    n.mode = "GLOBAL"
    assert n._avoid_cruise_target() == 0.0, "같은 프레임에 바로 풀려야 한다"


def test_zero_turns_it_off():
    assert _Node("AVOID", high=0.0, low=0.0)._avoid_cruise_target() == 0.0


# ------------------------------------------------- 상한이자 하한이다


def _apply(node: _Node, v: float, *, trailing=False, escaping=False):
    """`_planner_speed_scale` 의 순항속도 적용부를 그대로 옮겨 온 것."""
    cruise = node._avoid_cruise_target()
    if cruise > 0.0 and not trailing and not escaping:
        return cruise, "avoid_cruise"
    return v, "static"


def test_racing_speed_is_pulled_down():
    assert _apply(_Node("AVOID", v=7.0), 7.0) == (HIGH, "avoid_cruise")


def test_a_lower_limit_is_pulled_back_up():
    """거리 기반 감속이 더 낮게 불러도 되돌린다 — 급감속은 AEB 뿐이다."""
    assert _apply(_Node("AVOID", v=7.0), 1.2) == (HIGH, "avoid_cruise")
    assert _apply(_Node("AVOID", v=3.0), 0.9) == (LOW, "avoid_cruise")


def test_trailing_keeps_its_own_speed():
    """앞차를 따라갈 때는 앞차 속도가 기준이다."""
    assert _apply(_Node("AVOID"), 2.0, trailing=True) == (2.0, "static")


def test_the_escape_crawl_is_not_overridden():
    assert _apply(_Node("AVOID"), 0.8, escaping=True) == (0.8, "static")


def test_a_clear_track_keeps_the_csv_speed():
    assert _apply(_Node("GLOBAL"), 7.0) == (7.0, "static")


# ------------------------------------------- (2) 안 걸리는 장애물은 무시한다

CORRIDOR = CFG["corridor_max_lateral_from_raceline_m"]


def _blocks(lat: float, r: float, corridor: float = CORRIDOR) -> bool:
    """라인에서 lat 만큼 떨어진 반경 r 장애물이 회피 대상인가."""
    line = [(float(k), 0.0) for k in range(-10, 11)]
    return not _outside_corridor(
        1.0,
        0.0,
        r,
        corridor_max_lat_m=corridor,
        track_pts=line,
        laser_to_map=lambda x, y: (0.0, lat),
    )


def test_the_corridor_is_the_car_half_width_plus_slip_margin():
    assert CORRIDOR == round(vg.HALF_WIDTH_M + 0.03, 3)
    assert CORRIDOR == 0.18


def test_something_sitting_on_the_line_still_blocks():
    assert _blocks(0.0, 0.25)
    assert _blocks(0.1, 0.05)


def test_something_we_would_clear_is_ignored():
    """가까운 쪽 끝이 반폭+여유 밖 = 라인 그대로 가면 안 닿는다."""
    assert not _blocks(0.50, 0.25)  # 끝이 0.25 → 0.18 밖
    assert not _blocks(0.30, 0.05)  # 끝이 0.25


def test_the_three_centimetre_margin_is_there():
    """반폭 딱 15 cm 로 자르면 슬립으로 밀렸을 때 닿는다."""
    r = 0.10
    assert _blocks(vg.HALF_WIDTH_M + r + 0.02, r), "15~18 cm 구간은 아직 잡는다"
    assert not _blocks(vg.HALF_WIDTH_M + r + 0.04, r)


def test_a_wide_obstacle_reaches_the_line_even_from_far():
    """중심이 멀어도 크면 걸린다 — 반경을 빼는 이유."""
    assert _blocks(0.60, 0.45)


def test_regression_the_old_width_avoided_things_it_would_clear():
    """0.25 기준에서는 그냥 지나갈 물체도 회피 대상이었다."""
    lat, r = 0.45, 0.22  # 끝이 0.23 → 차 끝(0.15)에서 8 cm 여유
    assert _blocks(lat, r, corridor=0.25)
    assert not _blocks(lat, r)


def test_the_gate_is_a_clean_threshold():
    r = 0.20
    edge = CORRIDOR + r
    assert _blocks(edge - 0.01, r)
    assert not _blocks(edge + 0.01, r)


def test_it_reads_distance_not_side():
    """좌우 어느 쪽이든 같은 거리면 같은 판정이어야 한다."""
    for r in (0.05, 0.25):
        for lat in (0.10, 0.35, 0.90):
            assert _blocks(lat, r) == _blocks(-lat, r), (lat, r)
