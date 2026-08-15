#!/usr/bin/env python3
"""트래커 리팩터 동작 동일성 + KF 모드 테스트.

_update_tracks 는 dynamic/static 분류의 입력을 만든다. 여기가 어긋나면
플래너가 정지 장애물을 달려오는 차로 보거나 그 반대가 된다.

    python3 -m pytest src/path_following/test/test_obstacle_tracking.py -q
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.integrated_obstacle_node import (  # noqa: E402
    Detection,
    IntegratedObstacleNode,
    Track,
    resolve_keep_time,
)

DT = 0.025


class _Tracker:
    """_update_tracks* 가 쓰는 필드만 갖춘 가짜 노드."""

    def __init__(self, mode: str = "ema", consistent_centroid: bool = False):
        self._tracker_mode = mode
        self._consistent_centroid = consistent_centroid
        self.match_dist_m = 1.0
        self.vel_ema_alpha = 0.35
        self.max_track_speed_mps = 12.0
        self.track_keep_time_s = 0.12
        self._kf_sigma_accel = 3.0
        self._kf_sigma_meas = 0.06
        self._kf_gate_m2 = 0.0
        self._tracks: list[Track] = []
        self._next_id = 0

    _track_coords = IntegratedObstacleNode._track_coords
    _finish_track = IntegratedObstacleNode._finish_track
    _spawn_track = IntegratedObstacleNode._spawn_track
    _update_tracks = IntegratedObstacleNode._update_tracks
    _update_tracks_ema = IntegratedObstacleNode._update_tracks_ema
    _update_tracks_kf = IntegratedObstacleNode._update_tracks_kf


def _det(lx, ly, mx, my, r=0.15):
    return Detection(
        laser_x=lx,
        laser_y=ly,
        map_x=mx,
        map_y=my,
        radius=r,
        center_laser_x=lx,
        center_laser_y=ly,
        center_map_x=mx,
        center_map_y=my,
    )


def _legacy_ema(frames, alpha=0.35, match=1.0, max_v=12.0, keep=0.12):
    """리팩터 전 _update_tracks 그대로 옮긴 것. 기준값 생성용."""
    tracks, next_id = [], 0
    for dets in frames:
        for t in tracks:
            t["matched"] = False
        used = set()
        for t in tracks:
            best_i, best_d = -1, float("inf")
            for i, d in enumerate(dets):
                if i in used:
                    continue
                dist = math.hypot(d.map_x - t["map_x"], d.map_y - t["map_y"])
                if dist < best_d and dist <= match:
                    best_d, best_i = dist, i
            if best_i < 0:
                t["missed"] += DT
                continue
            d = dets[best_i]
            used.add(best_i)
            vx = (d.map_x - t["map_x"]) / DT
            vy = (d.map_y - t["map_y"]) / DT
            vlx = (d.laser_x - t["laser_x"]) / DT
            vly = (d.laser_y - t["laser_y"]) / DT
            if math.hypot(vx, vy) <= max_v:
                t["vx"] = alpha * vx + (1 - alpha) * t["vx"]
                t["vy"] = alpha * vy + (1 - alpha) * t["vy"]
                t["vlx"] = alpha * vlx + (1 - alpha) * t["vlx"]
                t["vly"] = alpha * vly + (1 - alpha) * t["vly"]
            t.update(
                map_x=d.map_x, map_y=d.map_y, laser_x=d.laser_x, laser_y=d.laser_y
            )
            t["speed"] = math.hypot(t["vx"], t["vy"])
            rng = math.hypot(t["laser_x"], t["laser_y"])
            t["closing"] = (
                -(t["laser_x"] * t["vlx"] + t["laser_y"] * t["vly"]) / rng
                if rng > 1e-3
                else 0.0
            )
            t["age"] += DT
            t["missed"] = 0.0
            t["matched"] = True
        for i, d in enumerate(dets):
            if i in used:
                continue
            tracks.append(
                dict(
                    id=next_id,
                    map_x=d.map_x,
                    map_y=d.map_y,
                    laser_x=d.laser_x,
                    laser_y=d.laser_y,
                    vx=0.0,
                    vy=0.0,
                    vlx=0.0,
                    vly=0.0,
                    speed=0.0,
                    closing=0.0,
                    age=DT,
                    missed=0.0,
                    matched=True,
                )
            )
            next_id += 1
        tracks = [t for t in tracks if t["missed"] <= keep]
    return tracks


def _scenario(n=40, vmap=1.2):
    """map 에서 +x 로 vmap 으로 가는 물체 하나. laser 에서는 다가온다."""
    frames = []
    for k in range(n):
        t = k * DT
        frames.append([_det(3.0 - 0.5 * t, 0.1, 10.0 + vmap * t, 5.0)])
    return frames


def test_ema_mode_matches_legacy_exactly():
    """기본 설정에서 리팩터 전후 결과가 완전히 같아야 한다."""
    frames = _scenario()
    ref = _legacy_ema(frames)

    trk = _Tracker(mode="ema", consistent_centroid=False)
    for dets in frames:
        trk._update_tracks(dets, DT)

    assert len(trk._tracks) == len(ref) == 1
    got, want = trk._tracks[0], ref[0]
    assert got.track_id == want["id"]
    for attr, key in [
        ("map_x", "map_x"),
        ("map_y", "map_y"),
        ("laser_x", "laser_x"),
        ("laser_y", "laser_y"),
        ("vx_map", "vx"),
        ("vy_map", "vy"),
        ("vx_laser", "vlx"),
        ("vy_laser", "vly"),
        ("speed", "speed"),
        ("closing_mps", "closing"),
        ("age_s", "age"),
    ]:
        assert getattr(got, attr) == want[key], attr


def test_ema_and_kf_agree_on_dynamic_classification():
    """같은 입력에서 두 모드 모두 map 속력을 맞춰야 한다 (0.45 m/s 임계 기준)."""
    frames = _scenario(n=60, vmap=1.2)
    out = {}
    for mode in ("ema", "kf"):
        trk = _Tracker(mode=mode)
        for dets in frames:
            trk._update_tracks(dets, DT)
        out[mode] = trk._tracks[0]
    for mode, t in out.items():
        assert abs(t.speed - 1.2) < 0.2, f"{mode} speed={t.speed}"
        assert t.speed >= 0.45, f"{mode} 는 dynamic 으로 분류돼야 한다"


def test_kf_reports_static_object_as_static():
    """노이즈만 있는 정지 물체를 움직인다고 보면 안 된다 (오분류 = 헛회피)."""
    import numpy as np

    rng = np.random.default_rng(11)
    trk = _Tracker(mode="kf")
    for _ in range(120):
        d = _det(
            2.0 + rng.normal(0, 0.02),
            0.0 + rng.normal(0, 0.02),
            8.0 + rng.normal(0, 0.02),
            4.0 + rng.normal(0, 0.02),
        )
        trk._update_tracks([d], DT)
    assert trk._tracks[0].speed < 0.45


def _run_with_gap(
    mode: str, gap: set[int], n: int = 40, vmap: float = 1.2, keep_s: float | None = None
):
    frames = _scenario(n=n, vmap=vmap)
    trk = _Tracker(mode=mode)
    if keep_s is not None:
        trk.track_keep_time_s = keep_s
    peak, ids = 0.0, set()
    for k, dets in enumerate(frames):
        trk._update_tracks([] if k in gap else dets, DT)
        if trk._tracks:
            ids.add(trk._tracks[0].track_id)
            if k > max(gap):
                peak = max(peak, trk._tracks[0].speed)
    return peak, ids, trk


def test_ema_speed_spikes_after_short_occlusion():
    """가림 후 재검출 프레임에서 EMA 속도가 폭주하는지.

    미검출 동안 위치를 얼려 두므로 4 프레임치 변위가 한 dt 에 몰린다.
    KF 는 그동안 predict 로 전진해 있어서 잔차가 작다.
    """
    gap = {20, 21, 22}  # 0.075 s — track_keep_time_s(0.12) 안이라 둘 다 살아남는다
    ema_peak, ema_ids, _ = _run_with_gap("ema", gap)
    kf_peak, kf_ids, _ = _run_with_gap("kf", gap)

    assert len(ema_ids) == 1 and len(kf_ids) == 1, "짧은 가림에선 트랙이 유지된다"
    assert ema_peak > 1.2 * 1.5, f"EMA 는 참값 1.2 대비 크게 튄다 (got {ema_peak:.2f})"
    assert kf_peak < 1.2 * 1.2, f"KF 는 거의 안 튄다 (got {kf_peak:.2f})"
    assert kf_peak < ema_peak


def test_keep_time_is_tied_to_tracker_mode():
    """유지 시간이 tracker_mode 에 묶여 있어야 한다.

    ema 에서 유지 시간을 늘리면 얼어붙은 위치가 그대로 오래 남아 오히려
    나빠진다. predict 로 위치를 밀어 주는 kf 에서만 늘린다. 플래그 하나
    (tracker_mode) 를 되돌리면 유지 시간도 같이 원복돼야 한다.
    """
    assert resolve_keep_time("ema", 0.12, 0.25) == 0.12
    assert resolve_keep_time("kf", 0.12, 0.25) == 0.25


def test_long_occlusion_kills_ema_track_but_kf_survives():
    """0.12 s 는 40 Hz 에서 5 프레임뿐이다.

    그보다 긴 가림이면 ema 트랙은 삭제되고 새 ID 로 태어난다. age_s 가 0 으로
    리셋되므로 dynamic_confirm_time_s 를 다시 세고, 그동안 달려오는 차가
    static 으로 분류된다. kf 는 늘린 유지 시간 덕에 같은 트랙을 이어 간다.
    """
    gap = set(range(20, 26))  # 0.15 s

    _, ema_ids, ema_trk = _run_with_gap("ema", gap, keep_s=0.12)
    assert len(ema_ids) > 1, "ema: 트랙이 삭제·재생성된다"

    _, kf_ids, kf_trk = _run_with_gap("kf", gap, keep_s=0.25)
    assert len(kf_ids) == 1, "kf: 같은 트랙을 유지한다"
    assert kf_trk._tracks[0].speed >= 0.45, "kf: dynamic 분류가 끊기지 않는다"

    # ema 는 가림 이후 구간만, kf 는 처음부터 누적한다.
    # age_s 가 dynamic_confirm_time_s 를 다시 세게 만드는 게 문제의 본질이다.
    # age_s 는 매칭된 프레임에서만 증가한다 → kf 는 (40-6)×0.025 = 0.85 s
    assert ema_trk._tracks[0].age_s < 0.5
    assert kf_trk._tracks[0].age_s > 0.8
    assert kf_trk._tracks[0].age_s > 2.0 * ema_trk._tracks[0].age_s


def test_consistent_centroid_keeps_published_point_as_nearest():
    """[A3] 를 켜도 발행 좌표(=거리 게이트 입력)는 최근접점이어야 한다."""
    trk = _Tracker(mode="ema", consistent_centroid=True)
    d = Detection(
        laser_x=1.90,
        laser_y=0.0,
        map_x=5.0,
        map_y=1.0,
        radius=0.2,
        center_laser_x=2.05,
        center_laser_y=0.02,
        center_map_x=5.15,
        center_map_y=1.02,
    )
    trk._update_tracks([d], DT)
    t = trk._tracks[0]
    assert (t.laser_x, t.laser_y) == (1.90, 0.0), "발행은 최근접점"
    assert (t.center_laser_x, t.center_laser_y) == (2.05, 0.02), "추적은 centroid"


def test_stale_tracks_are_dropped():
    trk = _Tracker(mode="ema")
    trk._update_tracks([_det(2.0, 0.0, 8.0, 4.0)], DT)
    assert len(trk._tracks) == 1
    for _ in range(10):  # 0.25 s > track_keep_time_s(0.12)
        trk._update_tracks([], DT)
    assert trk._tracks == []
