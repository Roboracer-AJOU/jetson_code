#!/usr/bin/env python3
"""LaserScan 클러스터링 공용 모듈.

static_obstacle_node 와 integrated_obstacle_node 가 같은 알고리즘을 각자
복사해 갖고 있었다. 한쪽만 고치면 두 런치의 거동이 갈라지므로 여기로 모은다.

ROS 의존성이 없다 — numpy 배열만 받고 돌려준다. 그래서 합성 스캔으로
단위 테스트가 가능하다.

용어:
  breakpoint  인접한 두 스캔 점 사이를 "다른 물체" 로 끊는 지점
  ABD         Adaptive Breakpoint Detection. 끊는 임계를 거리에 비례시킨다
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ClusterParams:
    """클러스터링 설정. 두 노드가 같은 필드를 쓴다."""

    # "fixed" = 고정 임계(기존 동작) | "adaptive" = 거리 비례 임계(ABD)
    mode: str = "fixed"
    # fixed 모드에서 쓰는 고정 간격 임계 [m]
    gap_threshold_m: float = 0.28
    # --- ABD ---
    lambda_deg: float = 10.0     # 허용 입사각. 작을수록 임계가 커진다
    sigma_r_m: float = 0.02      # 거리 노이즈 1σ
    min_gap_m: float = 0.05
    max_gap_m: float = 0.35
    # --- 최소 점수 ---
    min_points: int = 10         # 상한 (기존 고정값)
    adaptive_min_points: bool = False
    min_points_floor: int = 3
    min_arc_m: float = 0.07      # 이 정도 호(arc)는 찍혀야 물체로 본다


@dataclass
class ClusterResult:
    """클러스터 하나. 인덱스는 호출부 원본 배열 기준이다."""

    idx: np.ndarray
    center_x: float
    center_y: float
    near_x: float
    near_y: float
    radius: float
    span_m: float
    range_m: float          # 중심까지의 거리
    n_points: int = 0


def adaptive_gap_threshold(
    r: np.ndarray | float, angle_increment: float, p: ClusterParams
) -> np.ndarray | float:
    """ABD 임계.

        d_max(r) = r · sin(Δφ) / sin(λ − Δφ) + 3σ

    λ 는 "이 각도보다 비스듬하게 놓인 면은 같은 물체로 안 본다" 는 허용
    입사각이다. 거리가 멀수록 같은 물체의 이웃 점 간격이 벌어지므로 임계도
    같이 커져야 한다. 고정 0.28 m 는 근거리에서는 관대하고 원거리에서는
    두 물체를 하나로 붙여 버린다.
    """
    lam = math.radians(max(1e-3, p.lambda_deg))
    denom = math.sin(lam - angle_increment)
    if denom <= 1e-6:
        # λ 가 각분해능보다 작으면 식이 발산한다. 상한으로 눕힌다.
        return p.max_gap_m
    d = r * (math.sin(angle_increment) / denom) + 3.0 * p.sigma_r_m
    return np.clip(d, p.min_gap_m, p.max_gap_m)


def min_points_for_range(
    r: float, angle_increment: float, p: ClusterParams
) -> int:
    """거리 r 에서 요구할 최소 점수.

    10 m 앞 0.3 m 물체는 각분해능상 7점 정도밖에 안 찍힌다. 고정 10점
    기준이면 그게 통째로 안 보인다. 기대 점수를 계산해 바닥까지만 낮춘다.
    """
    if not p.adaptive_min_points:
        return p.min_points
    if r <= 1e-3 or angle_increment <= 1e-9:
        return p.min_points
    expected = p.min_arc_m / (r * angle_increment)
    return int(min(p.min_points, max(p.min_points_floor, round(expected))))


def cluster_scan_xy(
    px: np.ndarray,
    py: np.ndarray,
    *,
    angle_increment: float,
    params: ClusterParams,
    radius_percentile: float = 90.0,
    radius_min_m: float = 0.05,
    radius_max_m: float = 0.425,
    consistent_centroid: bool = False,
) -> list[ClusterResult]:
    """각도 정렬 → breakpoint 분할 → 대표점/반지름 산출.

    px, py 는 laser frame 좌표다. 반환 idx 는 입력 배열 기준 인덱스라
    호출부가 map 좌표 등 병렬 배열을 같은 인덱스로 꺼내 쓸 수 있다.
    """
    n = int(px.size)
    if n == 0:
        return []

    order = np.argsort(np.arctan2(py, px))
    sx = px[order]
    sy = py[order]
    rr = np.hypot(sx, sy)

    step = np.hypot(np.diff(sx), np.diff(sy))
    if params.mode == "adaptive":
        # 두 점 중 가까운 쪽 기준 — 경계에서 관대해지지 않게 한다
        r_pair = np.minimum(rr[:-1], rr[1:])
        thr = adaptive_gap_threshold(r_pair, angle_increment, params)
    else:
        thr = np.full(step.shape, params.gap_threshold_m)
    breaks = np.nonzero(step > thr)[0] + 1

    out: list[ClusterResult] = []
    starts = np.concatenate(([0], breaks))
    ends = np.concatenate((breaks, [n]))

    for s, e in zip(starts, ends):
        if e <= s:
            continue
        cx = float(np.mean(sx[s:e]))
        cy = float(np.mean(sy[s:e]))
        # 최소 점수는 클러스터 거리에 따라 정한다
        r_rep = float(np.mean(rr[s:e])) if consistent_centroid else math.hypot(cx, cy)
        if (e - s) < min_points_for_range(r_rep, angle_increment, params):
            continue

        seg_x = sx[s:e]
        seg_y = sy[s:e]
        span = float(
            max(np.max(seg_x) - np.min(seg_x), np.max(seg_y) - np.min(seg_y))
        )
        kmin = int(np.argmin(seg_x * seg_x + seg_y * seg_y))

        if consistent_centroid:
            # bbox span/2 는 비스듬히 걸친 벽 조각에서 크게 부푼다.
            # 중심에서의 점 거리 분위수가 실제 물체 크기에 더 가깝다.
            d_from_c = np.hypot(seg_x - cx, seg_y - cy)
            radius = float(np.percentile(d_from_c, radius_percentile))
            radius = min(max(radius, radius_min_m), radius_max_m)
        else:
            radius = span / 2.0

        out.append(
            ClusterResult(
                idx=order[s:e].copy(),
                center_x=cx,
                center_y=cy,
                near_x=float(seg_x[kmin]),
                near_y=float(seg_y[kmin]),
                radius=radius,
                span_m=span,
                range_m=r_rep,
                n_points=int(e - s),
            )
        )
    return out
