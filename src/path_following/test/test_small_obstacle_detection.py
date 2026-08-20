#!/usr/bin/env python3
"""작은 장애물(50 cm 미만)이 쓸 만한 거리에서 잡히는지 — 실제 CFG 로 검증.

    python3 -m pytest src/path_following/test/test_small_obstacle_detection.py -q

배경: 대회 장애물은 최대 50×50 cm 라 그보다 작은 것도 봐야 한다. 그런데
검출은 두 게이트를 연달아 통과해야 하고, 둘 다 거리에 비례해 나빠진다.

  1. 점 수  — 폭 w 물체가 거리 r 에서 남기는 점은 w/(r·increment) 개다.
              고정 10 점이면 한계 거리가 w×23.8 m 라 20 cm 는 4.7 m.
  2. span   — 측정 span 은 양 끝 '맞은' 빔 사이라 실제 폭보다 짧다.
              얼마나 짧은지는 빔 격자와 물체의 위상에 달렸다.

한쪽만 풀면 다른 쪽이 그대로 잘라 낸다. 그래서 adaptive_min_points 와
min_obstacle_size_m 을 같이 잡아야 한다.

위상을 훑는 이유: 차가 다가가는 동안 격자 위상이 프레임마다 바뀌어서,
운 좋은 위상 하나만 재면 한계 거리가 크게 낙관적으로 나온다(15 cm 기준
1.6 m 를 7.1 m 로 봤다). 검출이 M-of-N(6 중 4)을 통과하려면 특정 위상이
아니라 위상 전반에서 떠야 하므로, 여기서는 모든 위상을 요구한다.

이 테스트는 라이브러리가 아니라 **배포되는 CFG 값** 을 고정한다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.integrated_obstacle_node import CFG as DYN_CFG  # noqa: E402
from path_following.scan_cluster import ClusterParams, cluster_scan_xy  # noqa: E402
from path_following.static_obstacle_node import CFG as STATIC_CFG  # noqa: E402

# scan 실측값 (Slamtec T1): 0.00421 rad = 0.241°
ANGLE_INC = 0.004210104234516621
PHASES = 8


def _params(cfg: dict) -> ClusterParams:
    """노드가 CFG 로 ClusterParams 를 만드는 방식 그대로."""
    return ClusterParams(
        mode=str(cfg["cluster_mode"]).strip().lower(),
        gap_threshold_m=float(cfg["cluster_gap_threshold_m"]),
        lambda_deg=float(cfg["abd_lambda_deg"]),
        sigma_r_m=float(cfg["abd_sigma_r_m"]),
        min_gap_m=float(cfg["abd_min_gap_m"]),
        max_gap_m=float(cfg["abd_max_gap_m"]),
        min_points=int(cfg["min_cluster_points"]),
        adaptive_min_points=bool(cfg["adaptive_min_points"]),
        min_points_floor=max(3, int(cfg["min_cluster_points_floor"])),
        min_arc_m=max(0.01, float(cfg["min_arc_m"])),
    )


def _seen(cfg: dict, width_m: float, range_m: float, phase: float) -> bool:
    """정면 range_m 의 폭 width_m 판을, 위상 phase 인 빔 격자로 본다."""
    half = math.atan2(width_m / 2.0, range_m)
    th = np.arange(-500, 501) * ANGLE_INC + phase
    th = th[np.abs(th) <= half]
    if th.size == 0:
        return False
    # 판이라 x 는 일정, y 만 벌어진다
    px = np.full(th.size, range_m)
    py = range_m * np.tan(th)
    clusters = cluster_scan_xy(
        px,
        py,
        angle_increment=ANGLE_INC,
        params=_params(cfg),
        radius_min_m=0.05,
        radius_max_m=float(cfg["max_obstacle_size_m"]) / 2.0,
    )
    lo = float(cfg["min_obstacle_size_m"])
    hi = float(cfg["max_obstacle_size_m"])
    return any(lo <= c.span_m <= hi for c in clusters)


def _seen_at_every_phase(cfg: dict, width_m: float, range_m: float) -> bool:
    return all(
        _seen(cfg, width_m, range_m, i * ANGLE_INC / PHASES) for i in range(PHASES)
    )


def _max_range(cfg: dict, width_m: float) -> float:
    """위상과 무관하게 검출이 유지되는 최대 거리 (사거리 상한에서 끊는다)."""
    cap = int(round(float(cfg["max_obstacle_range_m"]) * 10))
    best = 0.0
    for step in range(4, cap + 1):
        r = step / 10.0
        if _seen_at_every_phase(cfg, width_m, r):
            best = r
    return best


# ---------------------------------------------------------------- 배포 설정


def test_both_nodes_share_the_detection_gates():
    """정적/동적 노드가 갈리면 회피와 AEB 가 서로 다른 장애물을 본다."""
    for key in (
        "adaptive_min_points",
        "min_cluster_points",
        "min_cluster_points_floor",
        "min_arc_m",
        "min_obstacle_size_m",
        "max_obstacle_size_m",
    ):
        assert STATIC_CFG[key] == DYN_CFG[key], f"{key} 불일치"


def test_a_fifty_centimeter_box_is_seen_to_the_range_cap():
    """대회 최대 크기. 이건 원래도 됐고, 계속 돼야 한다."""
    assert _max_range(STATIC_CFG, 0.50) >= float(STATIC_CFG["max_obstacle_range_m"])


def test_smaller_boxes_are_seen_far_enough_to_avoid_at_speed():
    """6 m/s 로 달릴 때 회피 게이트(12 m)가 의미를 가지려면 필요한 거리.

    20 cm 가 4.7 m 에서야 보이면 충돌 0.8 초 전이라 회피가 아니라 AEB 다.
    """
    assert _max_range(STATIC_CFG, 0.20) >= 9.0
    assert _max_range(STATIC_CFG, 0.30) >= 10.5


def test_detection_range_grows_with_object_width():
    """폭이 커지는데 한계 거리가 줄면 게이트 조합이 뒤틀린 것이다."""
    ranges = [_max_range(STATIC_CFG, w) for w in (0.15, 0.20, 0.30, 0.50)]
    assert ranges == sorted(ranges)


# ---------------------------------------------------------------- 회귀


def test_regression_fixed_point_count_hid_small_boxes():
    """이 변경 전 설정(고정 10점)이 실제로 20 cm 를 근거리로 밀어냈는지.

    실패하면 한계가 다른 데 있다는 뜻이고, adaptive 를 켤 근거가 사라진다.
    """
    before = dict(STATIC_CFG, adaptive_min_points=False, min_obstacle_size_m=0.14)
    assert _max_range(before, 0.20) < 5.5
    assert _max_range(STATIC_CFG, 0.20) > _max_range(before, 0.20) + 3.0


def test_regression_the_span_gate_was_the_second_wall():
    """adaptive 만 켜고 span 게이트를 두면 15 cm 는 여전히 코앞이다.

    두 게이트를 같이 봐야 하는 이유. 한쪽만 되돌리면 조용히 반쯤 막힌다.
    """
    span_only = dict(STATIC_CFG, min_obstacle_size_m=0.14)
    assert _max_range(span_only, 0.15) < 3.0
    assert _max_range(STATIC_CFG, 0.15) > 4.5


def test_noise_sized_returns_are_still_rejected():
    """게이트를 낮춘 대가. 한두 점짜리 반사는 어떤 위상에서도 안 떠야 한다."""
    for width in (0.04, 0.08):
        for rng_m in (2.0, 6.0, 10.0):
            for i in range(PHASES):
                assert not _seen(
                    STATIC_CFG, width, rng_m, i * ANGLE_INC / PHASES
                ), f"{width * 100:.0f}cm @ {rng_m}m 가 장애물로 잡힌다"
