"""웨이포인트별 목표 속도 프로파일 (centerline / raceline 공용).

CSV 3번째 열 `v` 를 만드는 곳. 주행 중 "코너 감지" 를 하지 않고,
오프라인에서 경로 곡률과 차량 가감속 한계만으로 전 구간 속도를 미리 박는다.

  1) 곡률 한계   v_i = min(v_max, sqrt(a_lat / κ_i))
     반경이 큰 완만한 곡선은 자동으로 v_max 가 되고, U턴처럼 κ 가 큰 곳만
     느려진다. "90도 이상만 감속" 같은 분류 규칙이 필요 없는 이유다.
  2) 역방향 패스 v_i = min(v_i, sqrt(v_{i+1}² + 2·a_brake·ds))
     코너에 닿기 전부터 속도가 깎여 브레이킹 포인트가 저절로 앞당겨진다.
  3) 정방향 패스 v_i = min(v_i, sqrt(v_{i-1}² + 2·a_accel·ds))
     코너 탈출 후 속도가 순간이동하지 않는다.
"""
from __future__ import annotations

import argparse
import csv
import os

import numpy as np

try:
    from scipy.ndimage import gaussian_filter1d
except ImportError as exc:  # pragma: no cover
    raise SystemExit(f"Missing scipy: {exc}")

# ============================================================
# 차량 물리 파라미터 — 속도 프로파일의 근거
#
#   *** 아직 실측 전인 임시값입니다. 측정 후 여기를 채우세요. ***
#
#   a_lat   : 횡가속 한계 [m/s²]. 제일 중요. 반경 R 원을 돌다 미끄러지는
#             속도 v 로 a = v²/R 역산. 코너 속도가 곧 sqrt(a_lat / κ).
#   a_brake : 감속 한계 [m/s²]. 직선 풀브레이크 정지거리 d 에서 a = v²/(2d).
#   a_accel : 가속 한계 [m/s²]. 0→v 도달 거리 d 에서 a = v²/(2d).
#   v_max   : 직선 최고속도 캡 [m/s] (모터·기어비 또는 대회 규정).
#   safety  : 위 세 가속도에 곱하는 안전계수. 실차 검증 전엔 0.6~0.7 권장.
#   scale   : 프로파일 전체 배율. **직선·코너 비율은 그대로 두고 전 구간을
#             느리게** 만든다. 알고리즘 경향만 볼 때 이걸 쓴다.
#             (v_max 를 낮추면 직선만 느려지고 코너 속도는 물리값 그대로)
#   v_ref   : **기준 최고속도.** 여기만 바꾸면 전 구간이 비율 유지한 채 같이
#             움직인다. scale 을 직접 계산할 필요 없이 "최고속도를 이 값으로"
#             지정하면 알아서 역산한다. 0 이면 비활성(=물리값 그대로).
# ============================================================
VEHICLE = {
    "a_lat_mps2": 6.0,       # TODO 실측
    "a_brake_mps2": 4.0,     # TODO 실측
    "a_accel_mps2": 7.0,     # TODO 실측
    "v_max_mps": 8.0,        # TODO 실측
    "safety_factor": 1.0,    # TODO 실차 검증 전에는 0.6~0.7
    # ↓ 실험용 노브. 보통 여기(v_ref)만 만진다.
    "v_ref_mps": 6.0,        # 기준 최고속도 [m/s]. 0 = 비활성
    "speed_scale": 1.0,      # v_ref 대신 배율을 직접 줄 때 (0.4 = 40% 속도)
    "v_min_mps": 3.0,        # 하한 (완전 정지 방지)
    "measured": True,       # 실측값을 넣었으면 True → 경고 문구 사라짐
}

UNMEASURED_WARNING = (
    "\nWARNING: 차량 물리 파라미터가 아직 실측값이 아닙니다 (임시 플레이스홀더).\n"
    "  scripts/speed_profile.py 의 VEHICLE dict 를 실측값으로 채우고 "
    "measured=True 로 바꾸세요.\n"
    "  실차 첫 주행은 --safety-factor 0.6 --v-ref 2.0 정도로 시작하세요."
)


def path_curvature(points, resolution: float, smooth: float = 6.0):
    """[1/m] 곡률과 점 간격 ds [m]. points 는 픽셀 좌표."""
    pts = np.asarray(points, dtype=float)
    ds_px = float(np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1).mean())
    d1 = (np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)) / (2.0 * ds_px)
    d2 = (np.roll(pts, -1, axis=0) - 2.0 * pts + np.roll(pts, 1, axis=0)) / ds_px**2
    num = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    den = (d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5 + 1e-12
    kappa = num / den / resolution
    if smooth > 0:
        kappa = gaussian_filter1d(kappa, smooth, mode="wrap")
    return kappa, ds_px * resolution


def speed_profile(
    points,
    resolution,
    *,
    a_lat,
    a_accel,
    a_brake,
    v_max,
    v_min=0.0,
    scale=1.0,
):
    """웨이포인트별 목표 속도 [m/s]. 반환: (v, kappa, ds)

    scale 은 마지막에 곱하는 전체 배율. 균일 축소는 가·감속 제약을 항상
    만족하므로(양변이 s² 로 줄고 우변엔 여유가 생김) 안전하다.
    """
    kappa, ds = path_curvature(points, resolution)
    n = len(points)
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(kappa, 1e-9)))
    # 폐루프라 시작점 의존성이 남지 않도록 두 패스를 함께 여러 바퀴 돌린다.
    # 두 패스 모두 값을 낮추기만 하므로 단조 감소 → 수렴이 보장된다.
    for _ in range(4):
        before = v.copy()
        for i in range(n - 1, -1, -1):  # 역방향: 감속 한계
            j = (i - 1) % n
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * a_brake * ds))
        for i in range(n):              # 정방향: 가속 한계
            j = (i + 1) % n
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * a_accel * ds))
        if np.max(np.abs(before - v)) < 1e-9:
            break
    v = v * float(scale)
    if v_min > 0.0:
        v = np.maximum(v, float(v_min))
    return v, kappa, ds


def lap_time(v, ds) -> float:
    return float(np.sum(ds / np.maximum(v, 1e-6)))


def add_speed_args(parser: argparse.ArgumentParser) -> None:
    """--speed / 물리 파라미터 / 기준속도 노브를 CLI 에 추가."""
    parser.add_argument(
        "--speed",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="x,y,v 3열로 저장. --no-speed 면 x,y 만",
    )
    parser.add_argument(
        "--a-lat", type=float, default=VEHICLE["a_lat_mps2"], help="횡가속 한계 [m/s²]"
    )
    parser.add_argument(
        "--a-accel", type=float, default=VEHICLE["a_accel_mps2"], help="가속 한계 [m/s²]"
    )
    parser.add_argument(
        "--a-brake", type=float, default=VEHICLE["a_brake_mps2"], help="감속 한계 [m/s²]"
    )
    parser.add_argument(
        "--v-max", type=float, default=VEHICLE["v_max_mps"], help="직선 최고속도 캡 [m/s]"
    )
    parser.add_argument(
        "--safety-factor",
        type=float,
        default=VEHICLE["safety_factor"],
        help="가속도 3종에 곱하는 안전계수",
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=VEHICLE["speed_scale"],
        help="프로파일 전체 배율. 직선·코너 비율 유지한 채 느리게 (실험용)",
    )
    parser.add_argument(
        "--v-ref",
        type=float,
        default=VEHICLE["v_ref_mps"],
        help=(
            "기준 최고속도 [m/s]. 프로파일 최댓값이 이 값이 되도록 전체를 "
            "자동 스케일한다. 예: --v-ref 5 → 직선 5, 코너는 물리 비율대로 "
            "자동 축소. 0=비활성. 기본값은 VEHICLE['v_ref_mps']"
        ),
    )
    parser.add_argument(
        "--v-min", type=float, default=VEHICLE["v_min_mps"], help="속도 하한 [m/s]"
    )


def profile_kwargs_from_args(args, points, resolution, *, verbose=True) -> dict:
    """CLI args → speed_profile() kwargs. --v-ref 가 있으면 scale 을 역산."""
    safety = max(1e-3, args.safety_factor)
    kwargs = dict(
        a_lat=args.a_lat * safety,
        a_accel=args.a_accel * safety,
        a_brake=args.a_brake * safety,
        v_max=args.v_max,
        v_min=args.v_min,
        scale=args.speed_scale,
    )
    if getattr(args, "v_ref", 0.0) > 0.0:
        # 물리 프로파일을 먼저 뽑고, 최댓값이 v_ref 가 되도록 전체를 축소.
        # 직선/코너 속도 비율은 물리값 그대로 유지된다.
        probe, _, _ = speed_profile(
            points, resolution, **{**kwargs, "scale": 1.0, "v_min": 0.0}
        )
        kwargs["scale"] = float(args.v_ref) / max(float(probe.max()), 1e-9)
        if verbose:
            print(
                f"  v_ref={args.v_ref:.2f} m/s → speed_scale={kwargs['scale']:.3f} "
                f"(물리 최고속 {probe.max():.2f} m/s 기준 자동 계산)"
            )
    return kwargs


def report_profile(v, args, scale, label="speed profile") -> None:
    slow = float(np.mean(v < 0.6 * v.max()) * 100.0)
    print(
        f"  {label}: v min={v.min():.2f} max={v.max():.2f} "
        f"mean={v.mean():.2f} m/s (감속구간 {slow:.0f}% of lap)"
    )
    print(
        f"    a_lat={args.a_lat:.1f} a_accel={args.a_accel:.1f} "
        f"a_brake={args.a_brake:.1f} v_max={args.v_max:.1f} "
        f"safety={args.safety_factor:.2f} scale={scale:.3f}"
    )
    if args.v_min > 0.0:
        clipped = float(np.mean(v <= args.v_min + 1e-6))
        if clipped > 0.02:
            print(
                f"    NOTE: 웨이포인트 {100.0 * clipped:.0f}% 가 v_min={args.v_min} "
                "하한에 걸렸습니다. 그만큼은 물리 한계보다 빠른 지령이니 "
                "--v-min 을 낮추세요."
            )


def write_csv_xyv(path: str, rows) -> None:
    """x,y,v 3열 CSV. 기존 x,y 로더는 3번째 열을 무시하므로 하위호환된다."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "v"])
        for x, y, v in rows:
            writer.writerow([f"{float(x):.6f}", f"{float(y):.6f}", f"{float(v):.3f}"])
