#!/usr/bin/env python3
"""[A1]~[A4] 순수 함수 단위 테스트. ROS 없이 돈다.

    python3 -m pytest src/path_following/test/test_scan_cluster.py -q
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.scan_cluster import (  # noqa: E402
    ClusterParams,
    adaptive_gap_threshold,
    cluster_scan_xy,
    min_points_for_range,
)
from path_following.track_kf import ConstantVelocityKF  # noqa: E402

ANGLE_INC = math.radians(0.25)  # 대표적인 2D 라이다 각분해능


def _arc(cx: float, cy: float, n: int, spread_m: float) -> tuple[np.ndarray, np.ndarray]:
    """(cx, cy) 중심으로 라이다에서 본 것처럼 늘어선 n 개의 점."""
    r = math.hypot(cx, cy)
    th0 = math.atan2(cy, cx)
    half = spread_m / (2.0 * max(r, 1e-6))
    th = np.linspace(th0 - half, th0 + half, n)
    return r * np.cos(th), r * np.sin(th)


def _legacy_cluster(px, py, gap, min_pts):
    """리팩터 전 _cluster_xy 그대로. 동작 동일성 확인용."""
    n = int(px.size)
    order = np.argsort(np.arctan2(py, px))
    px, py = px[order], py[order]
    out, start = [], 0
    for i in range(1, n):
        if math.hypot(px[i] - px[i - 1], py[i] - py[i - 1]) > gap:
            if i - start >= min_pts:
                out.append((start, i))
            start = i
    if n - start >= min_pts:
        out.append((start, n))
    return out


# ---------------------------------------------------------------- A1


def test_fixed_mode_matches_legacy():
    """리팩터가 기존 동작을 바꾸지 않았는지. 이게 깨지면 나머지는 볼 것도 없다."""
    rng = np.random.default_rng(7)
    for _ in range(20):
        xs, ys = [], []
        for cx, cy, n in [(2.0, 0.0, 20), (3.0, 1.5, 15), (6.0, -2.0, 8)]:
            ax, ay = _arc(cx, cy, n, 0.30)
            xs.append(ax + rng.normal(0, 0.005, n))
            ys.append(ay + rng.normal(0, 0.005, n))
        px = np.concatenate(xs)
        py = np.concatenate(ys)

        p = ClusterParams(mode="fixed", gap_threshold_m=0.28, min_points=10)
        got = cluster_scan_xy(px, py, angle_increment=ANGLE_INC, params=p)
        want = _legacy_cluster(px, py, 0.28, 10)
        assert len(got) == len(want)
        for c, (s, e) in zip(got, want):
            assert c.n_points == e - s


def test_adaptive_threshold_grows_with_range():
    p = ClusterParams(lambda_deg=10.0, sigma_r_m=0.02, min_gap_m=0.05, max_gap_m=0.35)
    near = adaptive_gap_threshold(1.0, ANGLE_INC, p)
    far = adaptive_gap_threshold(8.0, ANGLE_INC, p)
    assert p.min_gap_m <= near <= far <= p.max_gap_m
    assert far > near


def test_adaptive_splits_two_near_objects():
    """2 m 앞 0.20 m 간격의 두 물체. 고정 0.28 m 는 하나로 붙인다.

    λ=10° 에서 임계는 8 m 부근에서 0.28 과 만난다. 즉 고정값은 근거리에서
    지나치게 관대하고, 분리가 정작 중요한 건 가까운 쪽이다.
    """
    a = _arc(2.0, -0.20, 14, 0.20)
    b = _arc(2.0, 0.20, 14, 0.20)
    px = np.concatenate([a[0], b[0]])
    py = np.concatenate([a[1], b[1]])

    fixed = cluster_scan_xy(
        px,
        py,
        angle_increment=ANGLE_INC,
        params=ClusterParams(mode="fixed", gap_threshold_m=0.28, min_points=10),
    )
    adaptive = cluster_scan_xy(
        px,
        py,
        angle_increment=ANGLE_INC,
        params=ClusterParams(mode="adaptive", min_points=10),
    )
    assert len(fixed) == 1, "고정 임계는 근거리 두 물체를 병합한다 (기존 문제)"
    assert len(adaptive) == 2, "적응형은 분리해야 한다"


def test_adaptive_crossover_is_around_eight_meters():
    """고정 0.28 m 가 사실상 '8 m 용' 튜닝이었음을 못박아 둔다.

    이 관계가 깨지면 abd_lambda_deg 가 바뀐 것이고, 근거리 과분할 위험이
    같이 움직이므로 실차 재검증이 필요하다.
    """
    p = ClusterParams(lambda_deg=10.0, sigma_r_m=0.02, max_gap_m=0.35)
    assert adaptive_gap_threshold(2.0, ANGLE_INC, p) < 0.15   # 근거리는 훨씬 엄격
    assert abs(adaptive_gap_threshold(8.0, ANGLE_INC, p) - 0.28) < 0.02
    assert adaptive_gap_threshold(20.0, ANGLE_INC, p) == p.max_gap_m  # 상한에 눕는다


# ---------------------------------------------------------------- A2


def test_adaptive_min_points_recovers_far_object():
    """10 m 앞 0.3 m 물체는 ~7점. 고정 10점 기준에서는 통째로 사라진다."""
    px, py = _arc(10.0, 0.0, 7, 0.30)
    off = ClusterParams(min_points=10, adaptive_min_points=False)
    on = ClusterParams(min_points=10, adaptive_min_points=True, min_points_floor=3)
    assert len(cluster_scan_xy(px, py, angle_increment=ANGLE_INC, params=off)) == 0
    assert len(cluster_scan_xy(px, py, angle_increment=ANGLE_INC, params=on)) == 1


def test_min_points_clamped_both_ends():
    p = ClusterParams(min_points=10, adaptive_min_points=True, min_points_floor=3)
    assert min_points_for_range(0.5, ANGLE_INC, p) == 10   # 근거리는 상한 유지
    assert min_points_for_range(50.0, ANGLE_INC, p) == 3   # 원거리는 바닥
    # 단조 비증가 — 멀수록 기준이 느슨해져야 한다
    seq = [min_points_for_range(r, ANGLE_INC, p) for r in (0.5, 2.0, 5.0, 10.0, 50.0)]
    assert seq == sorted(seq, reverse=True)

    off = ClusterParams(min_points=10, adaptive_min_points=False)
    assert min_points_for_range(50.0, ANGLE_INC, off) == 10  # 끄면 항상 상한


# ---------------------------------------------------------------- A3


def test_centroid_radius_resists_outliers():
    """반지름이 프레임마다 튀면 그게 그대로 회피 폭 노이즈가 된다.

    본체 옆에 두어 점이 붙었다 떨어졌다 하는 상황을 만든다. bbox span/2 는
    한 점에 끌려 크게 뛰지만 분위수는 거의 안 움직여야 한다.
    """
    base_x, base_y = _arc(3.0, 0.0, 24, 0.30)
    stray_x, stray_y = _arc(3.0, 0.22, 2, 0.02)
    px = np.concatenate([base_x, stray_x])
    py = np.concatenate([base_y, stray_y])

    p = ClusterParams(min_points=10, gap_threshold_m=0.28)
    clean_bbox = cluster_scan_xy(base_x, base_y, angle_increment=ANGLE_INC, params=p)[0]
    dirty_bbox = cluster_scan_xy(px, py, angle_increment=ANGLE_INC, params=p)[0]
    clean_pct = cluster_scan_xy(
        base_x, base_y, angle_increment=ANGLE_INC, params=p, consistent_centroid=True
    )[0]
    dirty_pct = cluster_scan_xy(
        px, py, angle_increment=ANGLE_INC, params=p, consistent_centroid=True
    )[0]

    bbox_jump = abs(dirty_bbox.radius - clean_bbox.radius)
    pct_jump = abs(dirty_pct.radius - clean_pct.radius)
    assert pct_jump < bbox_jump
    assert clean_pct.radius >= 0.05


def test_radius_is_clamped_to_bounds():
    px, py = _arc(2.0, 0.0, 20, 0.02)  # 아주 작은 물체
    cl = cluster_scan_xy(
        px,
        py,
        angle_increment=ANGLE_INC,
        params=ClusterParams(min_points=10),
        consistent_centroid=True,
        radius_min_m=0.05,
        radius_max_m=0.425,
    )[0]
    assert cl.radius == 0.05


def test_center_and_near_are_distinct_and_sane():
    px, py = _arc(2.0, 0.0, 20, 0.40)
    cl = cluster_scan_xy(
        px,
        py,
        angle_increment=ANGLE_INC,
        params=ClusterParams(min_points=10),
        consistent_centroid=True,
    )[0]
    # 최근접점(발행/거리 게이트용)과 중심(추적용)은 서로 다른 점이다.
    # 원호는 자차 기준 오목이라 현(chord)의 중심이 원호보다 안쪽에 있다.
    assert math.hypot(cl.center_x, cl.center_y) < math.hypot(cl.near_x, cl.near_y)
    assert abs(cl.center_y) < 0.05
    assert abs(cl.near_y) < 0.05


# ---------------------------------------------------------------- A4


def test_kf_converges_to_true_velocity():
    """노이즈 있는 위치 관측에서 등속 KF 가 참값으로 수렴하는지."""
    rng = np.random.default_rng(0)
    dt, vx, vy = 0.025, 1.5, -0.4
    kf = ConstantVelocityKF(0.0, 0.0, sigma_accel=3.0, sigma_meas=0.06)
    px = py = 0.0
    for k in range(1, 200):
        px, py = vx * k * dt, vy * k * dt
        kf.predict(dt)
        kf.update(px + rng.normal(0, 0.06), py + rng.normal(0, 0.06))
    assert abs(kf.vx - vx) < 0.25
    assert abs(kf.vy - vy) < 0.25


def test_kf_beats_finite_difference_ema():
    """같은 노이즈 입력에서 KF 속도가 유한차분+EMA 보다 덜 흔들려야 한다."""
    rng = np.random.default_rng(3)
    dt, vx = 0.025, 1.0
    kf = ConstantVelocityKF(0.0, 0.0, sigma_accel=3.0, sigma_meas=0.06)
    ema_v, prev = 0.0, 0.0
    kf_err, ema_err = [], []
    for k in range(1, 300):
        z = vx * k * dt + rng.normal(0, 0.06)
        kf.predict(dt)
        kf.update(z, 0.0)
        ema_v = 0.35 * ((z - prev) / dt) + 0.65 * ema_v
        prev = z
        if k > 100:  # 수렴 구간만 비교
            kf_err.append(abs(kf.vx - vx))
            ema_err.append(abs(ema_v - vx))
    assert np.mean(kf_err) < np.mean(ema_err)


def test_kf_predicts_through_occlusion():
    """가려진 프레임에도 위치가 얼어붙지 않고 전진해야 한다."""
    dt, vx = 0.025, 2.0
    kf = ConstantVelocityKF(0.0, 0.0, sigma_accel=3.0, sigma_meas=0.06)
    for k in range(1, 60):
        kf.predict(dt)
        kf.update(vx * k * dt, 0.0)
    before = kf.px
    for _ in range(8):  # 8 프레임 가림
        kf.predict(dt)
    assert kf.px > before + vx * 8 * dt * 0.8


def test_kf_mahalanobis_gate_rejects_outlier():
    kf = ConstantVelocityKF(0.0, 0.0, sigma_accel=3.0, sigma_meas=0.06)
    for _ in range(40):
        kf.predict(0.025)
        kf.update(0.0, 0.0)
    assert kf.mahalanobis2(0.01, 0.01) < 9.0
    assert kf.mahalanobis2(5.0, 5.0) > 100.0
