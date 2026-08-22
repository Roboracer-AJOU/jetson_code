#!/usr/bin/env python3
"""최적화한 핫패스가 예전 구현과 **비트 단위로** 같은 답을 내는지 검증.

    python3 -m pytest src/path_following/test/test_hotpath_equivalence.py -q

성능 작업의 유일한 합격 조건이다. 근사적으로 같으면 안 된다 — 여기서 나온
세그먼트 인덱스가 다음 주기 앵커가 되고, 코리도 거리가 장애물 채택 여부를
가른다. 1 ulp 차이가 임계값 근처에서 다른 분기를 타면 궤적이 갈린다.

예전 구현을 이 파일 안에 그대로 박아 두고 같은 입력으로 돌려 비교한다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following import track_sliding as ts  # noqa: E402


def _load_track():
    for name in ("raceline.csv", "centerline.csv"):
        p = Path(__file__).resolve().parents[1] / "config" / name
        if p.is_file():
            return ts.load_csv_xy(str(p))
    pytest.skip("트랙 CSV 없음")


TRACK = _load_track()


# ---------------------------------------------------------------- 예전 구현
def _old_lateral(mx, my, pts):
    n = len(pts)
    if n < 2:
        return float("inf")
    xy = np.asarray(pts, dtype=np.float64)
    ax, ay = xy[:, 0], xy[:, 1]
    bx, by = np.roll(ax, -1), np.roll(ay, -1)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    t = np.divide(
        (mx - ax) * abx + (my - ay) * aby,
        ab2,
        out=np.zeros_like(ab2),
        where=ab2 >= 1e-14,
    )
    t = np.clip(t, 0.0, 1.0)
    qx = ax + t * abx
    qy = ay + t * aby
    d2 = (mx - qx) ** 2 + (my - qy) ** 2
    return float(math.sqrt(d2.min()))


class _OldLoop:
    """최적화 전 LoopTrackSliding.closest_projection_on_loop 그대로."""

    def __init__(self, points, half):
        self.points = points
        self.path_anchor_half_width = half
        self._track_anchor_seg = 0
        self._anchor_initialized = False

    def closest(self, mx, my):
        pts = self.points
        n = len(pts)
        half = self.path_anchor_half_width

        def eval_seg(i):
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % n]
            qx, qy, _t = ts._closest_point_on_segment(mx, my, ax, ay, bx, by)
            return qx, qy, (mx - qx) ** 2 + (my - qy) ** 2

        best_qx = best_qy = 0.0
        best_seg = 0
        best_d2 = float("inf")

        if not self._anchor_initialized:
            for i in range(n):
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2, best_qx, best_qy, best_seg = d2, qx, qy, i
            self._anchor_initialized = True
        else:
            for k in range(-half, half + 1):
                i = (self._track_anchor_seg + k) % n
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2, best_qx, best_qy, best_seg = d2, qx, qy, i

        if best_d2 > 100.0:
            best_d2 = float("inf")
            for i in range(n):
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2, best_qx, best_qy, best_seg = d2, qx, qy, i

        self._track_anchor_seg = best_seg
        return best_qx, best_qy, best_seg


# ---------------------------------------------------------------- 비교
def _probe_points(n=400, seed=7):
    """트랙 근처 + 트랙 밖 + 아주 먼 점을 섞는다."""
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        px, py = TRACK[rng.integers(0, len(TRACK))]
        out.append((px + rng.normal(0, 0.4), py + rng.normal(0, 0.4)))
    for _ in range(40):  # 100 m² 재탐색 분기를 태우는 먼 점
        out.append((float(rng.uniform(-60, 60)), float(rng.uniform(-60, 60))))
    return out


def test_lateral_distance_is_bit_identical():
    for mx, my in _probe_points():
        assert ts.lateral_distance_to_closed_polyline(mx, my, TRACK) == _old_lateral(
            mx, my, TRACK
        ), f"({mx}, {my}) 에서 갈렸다"


def test_lateral_distance_handles_a_degenerate_polyline():
    """길이 0 세그먼트가 섞여도 예전과 같아야 한다."""
    pts = [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 1.0), (0.0, 1.0)]
    for mx, my in ((0.5, 0.5), (2.0, 0.3), (-1.0, -1.0), (0.0, 0.0)):
        assert ts.lateral_distance_to_closed_polyline(mx, my, pts) == _old_lateral(
            mx, my, pts
        )


def test_a_short_polyline_is_still_infinite():
    assert ts.lateral_distance_to_closed_polyline(0.0, 0.0, [(1.0, 1.0)]) == float("inf")


@pytest.mark.parametrize("half", (30, 120, 200))
def test_the_projection_walks_the_same_track(half):
    """앵커가 상태를 이어받으므로 **한 점씩 순서대로** 따라가며 비교한다.

    한 번이라도 다른 세그먼트를 고르면 다음 주기 탐색창이 어긋나서 이후가
    전부 갈린다. 그래서 마지막 값만 보면 안 되고 전 구간을 봐야 한다.
    """
    new = ts.LoopTrackSliding(TRACK, 140, half)
    old = _OldLoop(TRACK, max(30, half))

    # 트랙을 따라 한 바퀴 돌면서 살짝씩 흔든다
    rng = np.random.default_rng(3)
    for i in range(0, len(TRACK), 3):
        px, py = TRACK[i]
        mx = px + float(rng.normal(0, 0.25))
        my = py + float(rng.normal(0, 0.25))
        got = new.closest_projection_on_loop(mx, my)
        want = old.closest(mx, my)
        assert got == want, f"i={i} 에서 갈렸다: {got} != {want}"


def test_the_projection_matches_on_the_first_uninitialized_call():
    new = ts.LoopTrackSliding(TRACK, 140, 120)
    old = _OldLoop(TRACK, 120)
    px, py = TRACK[300]
    assert new.closest_projection_on_loop(px + 0.1, py - 0.2) == old.closest(
        px + 0.1, py - 0.2
    )


def test_a_far_jump_triggers_the_same_full_research():
    """100 m² 밖으로 튀면 전 구간 재탐색 — 그 경로도 같아야 한다."""
    new = ts.LoopTrackSliding(TRACK, 140, 120)
    old = _OldLoop(TRACK, 120)
    px, py = TRACK[10]
    new.closest_projection_on_loop(px, py)
    old.closest(px, py)
    for mx, my in ((300.0, 300.0), TRACK[400], (-250.0, 80.0), TRACK[100]):
        assert new.closest_projection_on_loop(float(mx), float(my)) == old.closest(
            float(mx), float(my)
        )


def _old_closest_on_loop(ax, ay, bx, by, xp, yp):
    """최적화 전 LocalPlannerNode._closest_on_loop 그대로."""
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    t = np.divide(
        (xp - ax) * abx + (yp - ay) * aby,
        ab2,
        out=np.zeros_like(ab2),
        where=ab2 >= 1e-14,
    )
    t = np.clip(t, 0.0, 1.0)
    qx = ax + t * abx
    qy = ay + t * aby
    d2 = (xp - qx) ** 2 + (yp - qy) ** 2
    d2 = np.where(ab2 < 1e-14, np.inf, d2)
    i = int(np.argmin(d2))
    return float(qx[i]), float(qy[i]), i, float(t[i])


def _planner_stub(pts):
    from path_following.local_planner_node import LocalPlannerNode

    class _P:
        _closest_on_loop = LocalPlannerNode._closest_on_loop

    p = _P()
    p._xs_np = np.asarray([q[0] for q in pts], dtype=np.float64)
    p._ys_np = np.asarray([q[1] for q in pts], dtype=np.float64)
    p._bx_np = np.roll(p._xs_np, -1)
    p._by_np = np.roll(p._ys_np, -1)
    p._abx_np = p._bx_np - p._xs_np
    p._aby_np = p._by_np - p._ys_np
    p._ab2_np = p._abx_np * p._abx_np + p._aby_np * p._aby_np
    p._ab_ok_np = p._ab2_np >= 1e-14
    p._ab_bad_np = ~p._ab_ok_np
    p._ab_zeros_np = np.zeros_like(p._ab2_np)
    return p


def test_the_planner_frenet_projection_is_bit_identical():
    """Frenet 투영은 s·d 를 낳고 거기서 재합류·선감속이 전부 나온다."""
    p = _planner_stub(TRACK)
    for xp, yp in _probe_points(seed=11):
        assert p._closest_on_loop(xp, yp) == _old_closest_on_loop(
            p._xs_np, p._ys_np, p._bx_np, p._by_np, xp, yp
        ), f"({xp}, {yp}) 에서 갈렸다"


def test_the_planner_projection_survives_a_degenerate_segment():
    """중복점이 든 라인에서도 예전처럼 그 세그먼트를 배제해야 한다."""
    pts = [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    p = _planner_stub(pts)
    assert p._ab_bad_np.any(), "이 시험은 길이 0 세그먼트가 있어야 성립한다"
    for xp, yp in ((0.5, 0.5), (-1.0, 0.2), (0.0, 0.0), (3.0, 3.0)):
        assert p._closest_on_loop(xp, yp) == _old_closest_on_loop(
            p._xs_np, p._ys_np, p._bx_np, p._by_np, xp, yp
        )


def test_the_planner_projection_does_not_corrupt_its_cached_arrays():
    """스크래치를 재사용하다 정적 배열을 덮어쓰면 다음 호출부터 조용히 틀린다."""
    p = _planner_stub(TRACK)
    before = (p._ab2_np.copy(), p._abx_np.copy(), p._ab_zeros_np.copy())
    for xp, yp in _probe_points(n=50, seed=5):
        p._closest_on_loop(xp, yp)
    assert np.array_equal(p._ab2_np, before[0])
    assert np.array_equal(p._abx_np, before[1])
    assert np.array_equal(p._ab_zeros_np, before[2])


def _old_corridor(geom_ranges, wrapped, angle, half_w, margin, pre):
    """최적화 전 FGMNode._corridor_clear_distance 그대로.

    래핑은 반드시 노드와 같은 `arctan2(sin, cos)` 를 써야 한다. modulo 로
    바꾸면 마지막 자리가 달라져서, 멀쩡한 코드를 틀렸다고 잡는다.
    """
    from path_following.fgm_node import _wrap_pi_np

    d_ang = _wrap_pi_np(wrapped - angle)
    valid = (geom_ranges > 0.0) & (np.abs(d_ang) < math.pi * 0.5)
    if not np.any(valid):
        return pre
    r = geom_ranges[valid]
    da = d_ang[valid]
    along = r * np.cos(da)
    perp = np.abs(r * np.sin(da))
    blocking = (perp < half_w) & (along > 0.0)
    if not np.any(blocking):
        return pre
    return max(0.0, float(along[blocking].min()) - margin)


def _fgm_stub(half_w=0.254, margin=0.15, pre=10.0):
    from path_following.fgm_node import FGMNode

    class _F:
        _corridor_clear_distance = FGMNode._corridor_clear_distance
        _corridor_clear_reference = FGMNode._corridor_clear_reference
        _scan_xy = FGMNode._scan_xy

    f = _F()
    f.corridor_half_width = half_w
    f.corridor_stop_margin = margin
    f.preprocess_dist = pre
    f._scan_positive = None
    f._scan_cx = None
    f._scan_cy = None
    f._xy_src_ranges = None
    f._xy_src_wrapped = None
    return f


def _scans():
    """정상 스캔, 빈 스캔, 전부 0, 코리도에 아무것도 없는 스캔."""
    rng = np.random.default_rng(19)
    n = 1080
    ang = np.linspace(-2.356, 2.356, n)
    yield "일반", ang, 6.0 + 0.6 * np.cos(3.0 * ang) + rng.normal(0, 0.02, n)
    close = np.full(n, 8.0)
    close[np.abs(ang - 0.3) < 0.08] = 1.4
    yield "박스", ang, close
    yield "전부 0", ang, np.zeros(n)
    yield "일부 0", ang, np.where(np.abs(ang) < 0.5, 0.0, 5.0)
    yield "아주 가까움", ang, np.full(n, 0.05)
    yield "먼 벽만", ang, np.full(n, 10.0)


def test_the_corridor_distance_matches_the_old_formula():
    """코리도 거리는 **비트** 동일이 아니라 부동소수 잡음 이내로 같아야 한다.

    회전을 각도별 삼각함수에서 스칼라 회전으로 바꿨다. 수학적으로 같은 식이
    지만 반올림 순서가 달라서 마지막 자리가 흔들린다 (실측 ≤1 ulp).

    여기를 비트로 묶지 않는 이유는, 이 값이 **상태로 남지 않기** 때문이다.
    투영 인덱스처럼 다음 주기 앵커가 되는 값이면 1 ulp 가 누적되지만, 이건
    한 번 재서 문턱과 비교하고 버리는 거리다. 입력이 ±1 cm LiDAR 잡음인데
    1e-16 을 지킬 이유가 없다. 대신 **문턱 판정이 같은지** 를 따로 본다.
    """
    f = _fgm_stub()
    angles = np.linspace(-1.5, 1.5, 61)
    worst = 0.0
    for name, ang, rng_ in _scans():
        for a in angles:
            got = f._corridor_clear_distance(rng_, ang, float(a))
            want = _old_corridor(rng_, ang, float(a), 0.254, 0.15, 10.0)
            worst = max(worst, abs(got - want))
            assert got == pytest.approx(want, rel=1e-12, abs=1e-12), (
                f"{name} 스캔, angle={a:.3f}: {got} vs {want}"
            )
    assert worst < 1e-9, f"오차가 잡음 수준을 넘었다: {worst}"


def test_the_corridor_makes_the_same_pass_fail_decisions():
    """실제로 쓰이는 건 거리 자체가 아니라 문턱 비교 결과다."""
    f = _fgm_stub()
    angles = np.linspace(-2.0, 2.0, 121)
    for name, ang, rng_ in _scans():
        for thr in (0.5, 1.0, 1.5, 2.0):
            for a in angles:
                got = f._corridor_clear_distance(rng_, ang, float(a))
                want = _old_corridor(rng_, ang, float(a), 0.254, 0.15, 10.0)
                assert (got >= thr) == (want >= thr), (
                    f"{name}, angle={a:.3f}, 문턱 {thr}: 판정이 갈렸다"
                )


def test_the_scan_cache_is_rebuilt_when_the_scan_changes():
    """스캔이 바뀌었는데 옛 좌표를 쓰면 지난 프레임을 보고 조향한다."""
    f = _fgm_stub()
    ang = np.linspace(-2.356, 2.356, 400)
    near = np.full(400, 1.2)
    far = np.full(400, 9.0)
    a = f._corridor_clear_distance(near, ang, 0.0)
    b = f._corridor_clear_distance(far, ang, 0.0)
    assert a < b, "새 스캔인데 캐시를 그대로 썼다"
    assert b == pytest.approx(_old_corridor(far, ang, 0.0, 0.254, 0.15, 10.0), abs=1e-12)


def test_the_scan_cache_is_reused_within_one_scan():
    f = _fgm_stub()
    ang = np.linspace(-2.356, 2.356, 400)
    r = np.full(400, 3.0)
    f._corridor_clear_distance(r, ang, 0.0)
    cx = f._scan_cx
    f._corridor_clear_distance(r, ang, 0.4)
    assert f._scan_cx is cx, "같은 스캔인데 좌표를 다시 만들었다"


def test_the_reference_implementation_still_matches_the_old_code():
    """기준식을 남겨 둔 의미가 있으려면 그게 예전 코드 그대로여야 한다."""
    f = _fgm_stub()
    for name, ang, rng_ in _scans():
        for a in np.linspace(-1.5, 1.5, 21):
            assert f._corridor_clear_reference(rng_, ang, float(a)) == _old_corridor(
                rng_, ang, float(a), 0.254, 0.15, 10.0
            ), f"{name}, angle={a:.3f}"


def test_the_geometry_cache_returns_the_same_object_for_the_same_list():
    a = ts.segment_geometry(TRACK)
    b = ts.segment_geometry(TRACK)
    assert a is b


def test_a_different_list_gets_its_own_geometry():
    other = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert ts.segment_geometry(other) is not ts.segment_geometry(TRACK)
    assert ts.segment_geometry(other).n == 3


def test_the_cache_does_not_grow_without_bound():
    """임시 리스트를 계속 넘겨도 캐시가 불어나면 안 된다."""
    for i in range(50):
        ts.segment_geometry([(float(i), 0.0), (0.0, 1.0), (1.0, 1.0)])
    assert len(ts._GEOM_CACHE) <= ts._GEOM_CACHE_MAX
