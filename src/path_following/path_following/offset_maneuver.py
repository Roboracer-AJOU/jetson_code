"""레이스라인 기준 횡오프셋 회피 기동 (순수 계산 — ROS 의존 없음).

장애물을 "지금 보이는 방향으로 꺾는" 반응형(FGM)이 아니라, **미리 계획한
횡이동**으로 지나간다. 트랙 기준선(레이스라인)을 s 축으로 두고 옆으로 얼마나
비킬지 d(s) 를 한 번 그려 놓고 그대로 탄다.

    d
    ↑        ┌──────────────┐            ← 유지 (장애물 옆을 지나는 동안)
    │       ╱                ╲
    │      ╱                  ╲
    0 ────╯                    ╰────────  ← 복귀 (천천히 라인으로)
         └ 진입 ┘         └ 복귀 ┘
                    ↑ 장애물

핵심은 **진입 길이를 속도로 늘린다** 는 것 하나다. 횡이동 Δd 를 s 길이 L 에
걸쳐 하면 필요한 횡가속은

    a_lat ≈ v² · |d''|max = v² · 5.7735 · Δd / L²

라서, L 을 고정해 두면 속도의 제곱으로 폭발한다. 실제로 이전 구현은 L=1.2 m
고정이었고, 6 m/s 에서 Δd=0.65 m 를 요구하면 94 m/s²(9.5 g) 가 나왔다. 낼 수
없는 값이라 차는 그냥 조향을 물고 벽으로 갔다.

거꾸로 풀면 예산 a 를 정해 놓고 길이를 뽑을 수 있다.

    L = v · sqrt(5.7735 · Δd / a)

6 m/s, Δd=0.5 m, a=3 m/s² 면 L≈5.7 m — 즉 "장애물 6 m 앞에서부터 아주 조금
틀기 시작" 이다. 그때 최대 조향은 2° 남짓이고 감속할 이유가 없다. 이게 이
모듈이 하는 일 전부다.

늦게 발견해서 남은 거리가 모자라면 두 가지로 답한다.
  1) 남은 거리를 다 써서 최대한 완만하게 만들고,
  2) 그래도 예산을 넘으면 "이 속도까지 줄이면 된다" 는 상한을 같이 돌려준다.
치우는 대신 속도를 줄이는 쪽이 조향을 크게 넣는 것보다 언제나 안전하다.

지금까지 이 모듈은 기준선을 **직선으로 가정**했다. d'' 만으로 횡가속을 재고
있었다는 뜻이다. 코너에서는 그게 무너진다 — 기준선 자체가 v²·κ 를 이미 쓰고
있는데, 그 위에 복귀 곡선의 v²·d'' 를 얹으면 둘이 더해진다.

    a_total ≈ v²·(|κ| + |d''|)

R=6 m 코너를 6 m/s 로 돌면 코너만으로 6.0 m/s² 다. 접지력이 5~6 m/s²
(IMU 실측: 2.5~2.8 m/s 코너링에서 v·ω 피크 4.84~5.59 에서 이미 밀림) 이므로
복귀에 쓸 예산이 남아 있지 않다. 그런데 예산 검사는 d'' 만 봐서 통과했고,
감속도 계획 실패도 안 났다. 차는 조향을 물고 라인을 가로질러 바깥 벽으로 갔다.

그래서 `kappa_ref` 를 받는다. 주면 각 구간의 |κ| 를 예산에서 먼저 빼고
(남은 것으로 길이를 뽑고), 조향 한계·속도 상한도 |κ|+|d''| 로 판단한다.
안 주면 예전 그대로 직선 가정이다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Sequence, Tuple

# quintic d(s) = Δd·(10u³-15u⁴+6u⁵), u=s/L 의 미분 최대값 계수.
#   |d'|  최대: u=0.5 에서        1.875·Δd/L
#   |d''| 최대: u=(1±1/√3)/2 에서 5.7735·Δd/L²
D1_PEAK = 1.875
D2_PEAK = 5.7735

# d'' 실측 샘플 수. 시작 미분이 0 이 아니면 위 닫힌형이 안 맞아서 직접 훑는다.
_CURV_SAMPLES = 33


@dataclass(frozen=True)
class ManeuverConfig:
    """기동 계획에 필요한 예산과 한계."""

    half_width_m: float
    #: 장애물 표면에서 차체 옆면까지 추가로 띄울 거리
    lateral_margin_m: float
    #: |d| 상한. 트랙 폭에서 오는 물리적 한계
    max_offset_m: float
    #: 진입 횡가속 예산. 작을수록 멀리서부터 완만하게 시작한다
    a_lat_enter_mps2: float
    #: 복귀 횡가속 예산. 진입보다 작게 둬서 라인에 천천히 붙는다
    a_lat_exit_mps2: float
    #: 늦게 발견했을 때까지 허용하는 상한. 넘으면 감속으로 답한다
    a_lat_hard_mps2: float
    enter_min_m: float
    enter_max_m: float
    exit_min_m: float
    exit_max_m: float
    #: 장애물 표면 앞쪽으로 이만큼 전에는 오프셋에 올라와 있어야 한다
    hold_front_m: float
    #: 장애물 표면을 지난 뒤 이만큼 더 오프셋을 유지한다 (차체 길이 + 여유)
    hold_rear_m: float
    #: 이 간격 안에 있는 장애물은 한 덩어리로 본다 (연속 장애물)
    merge_gap_m: float
    #: 계획에 쓸 속도 하한. 정지 근처에서 L 이 0 으로 수렴하는 것을 막는다
    v_plan_min_mps: float
    #: 전륜 조향 한계 [rad]. 이걸 넘는 경로는 감속해도 못 탄다 — 계획 실패로 본다
    max_steer_rad: float
    #: 조향 한계 판정에 쓰는 축간거리 [m]
    wheelbase_m: float


@dataclass(frozen=True)
class ObstacleSD:
    """기준선 좌표계의 장애물. s 는 자차로부터의 전방 거리."""

    s: float
    d: float
    r: float


@dataclass(frozen=True)
class ObstacleGroup:
    """한 번의 기동으로 함께 지나갈 장애물 덩어리."""

    s_first: float
    s_last: float
    d_left: float   # 덩어리 왼쪽 끝 (표면)
    d_right: float  # 덩어리 오른쪽 끝 (표면)
    count: int


@dataclass(frozen=True)
class OffsetManeuver:
    """계획된 d(s). ds 는 기동 시작점(자차 위치)으로부터의 전방 거리."""

    d_start: float
    d_pass: float
    side: int  # +1 왼쪽으로 비킴, -1 오른쪽
    #: 진입 곡선을 시작하기 전에 현재 횡위치로 그냥 가는 구간.
    #: 진입에 필요한 길이보다 장애물이 멀면 그 차이만큼 라인 위에 더 머문다.
    lead_len_m: float
    enter_len_m: float
    hold_end_ds: float
    exit_len_m: float
    enter_coeff: Tuple[float, float, float, float, float, float]
    exit_coeff: Tuple[float, float, float, float, float, float]
    #: 계획 시 속도에서의 실제 최대 횡가속 [m/s²]
    peak_lateral_accel_mps2: float
    #: 이 속도 아래로 줄이면 예산 안에 들어온다. 여유가 있으면 None
    speed_cap_mps: float | None
    #: 진단용
    obstacle_s_first: float
    obstacle_s_last: float

    @property
    def total_length_m(self) -> float:
        return self.hold_end_ds + self.exit_len_m

    @property
    def enter_end_ds(self) -> float:
        return self.lead_len_m + self.enter_len_m

    def d_at(self, ds: float) -> float:
        """기동 시작점에서 ds 만큼 앞의 횡오프셋."""
        if ds <= self.lead_len_m:
            return self.d_start
        if ds < self.enter_end_ds:
            return _eval_quintic(self.enter_coeff, ds - self.lead_len_m)
        if ds < self.hold_end_ds:
            return self.d_pass
        t = ds - self.hold_end_ds
        if t >= self.exit_len_m:
            return 0.0
        return _eval_quintic(self.exit_coeff, t)

    def d_prime_at(self, ds: float) -> float:
        """d 의 s 미분 = 경로가 기준선과 이루는 각의 tan."""
        if self.lead_len_m > 0.0 and ds <= self.lead_len_m:
            return 0.0
        if ds < self.enter_end_ds:
            return _eval_quintic_d1(
                self.enter_coeff, max(0.0, ds - self.lead_len_m)
            )
        if ds < self.hold_end_ds:
            return 0.0
        t = ds - self.hold_end_ds
        if t >= self.exit_len_m:
            return 0.0
        return _eval_quintic_d1(self.exit_coeff, t)


# ----------------------------------------------------------------------
# quintic
# ----------------------------------------------------------------------
def solve_quintic(
    d0: float, d0p: float, d0pp: float, df: float, dfp: float, dfpp: float, L: float
) -> Tuple[float, float, float, float, float, float]:
    """양 끝의 값·1차·2차를 맞추는 5차 다항식 계수.

    3×3 을 직접 푼다 — numpy 없이도 되고, 해가 닫힌형이라 더 정확하다.
    """
    a0, a1, a2 = d0, d0p, 0.5 * d0pp
    if L < 1e-6:
        return a0, a1, a2, 0.0, 0.0, 0.0
    # 남은 경계조건 (끝점에서 이미 a0..a2 가 만드는 몫을 뺀 값)
    c0 = df - (a0 + a1 * L + a2 * L * L)
    c1 = dfp - (a1 + 2.0 * a2 * L)
    c2 = dfpp - 2.0 * a2
    L2, L3 = L * L, L * L * L
    # [L³ L⁴ L⁵; 3L² 4L³ 5L⁴; 6L 12L² 20L³] 의 역행렬을 정리한 결과
    a3 = (10.0 * c0 - 4.0 * c1 * L + 0.5 * c2 * L2) / L3
    a4 = (-15.0 * c0 + 7.0 * c1 * L - 1.0 * c2 * L2) / (L3 * L)
    a5 = (6.0 * c0 - 3.0 * c1 * L + 0.5 * c2 * L2) / (L3 * L2)
    return a0, a1, a2, a3, a4, a5


def _eval_quintic(c: Sequence[float], s: float) -> float:
    a0, a1, a2, a3, a4, a5 = c
    return a0 + s * (a1 + s * (a2 + s * (a3 + s * (a4 + s * a5))))


def _eval_quintic_d1(c: Sequence[float], s: float) -> float:
    _, a1, a2, a3, a4, a5 = c
    return a1 + s * (2 * a2 + s * (3 * a3 + s * (4 * a4 + s * 5 * a5)))


def _eval_quintic_d2(c: Sequence[float], s: float) -> float:
    _, _, a2, a3, a4, a5 = c
    return 2 * a2 + s * (6 * a3 + s * (12 * a4 + s * 20 * a5))


def peak_abs_d2(c: Sequence[float], L: float) -> float:
    """구간 [0,L] 에서 |d''| 최대. 시작 미분이 0 이 아닐 수 있어 훑어 본다."""
    if L <= 1e-6:
        return 0.0
    return max(
        abs(_eval_quintic_d2(c, L * k / (_CURV_SAMPLES - 1)))
        for k in range(_CURV_SAMPLES)
    )


# ----------------------------------------------------------------------
# 길이 산정
# ----------------------------------------------------------------------
def length_for_budget(delta_d: float, v: float, a_budget: float) -> float:
    """횡이동 Δd 를 예산 a 안에서 끝내는 데 필요한 최소 s 길이.

    a = v²·5.7735·Δd/L²  →  L = v·sqrt(5.7735·Δd/a)
    """
    dd = abs(float(delta_d))
    if dd < 1e-6 or a_budget <= 1e-6:
        return 0.0
    return abs(v) * math.sqrt(D2_PEAK * dd / a_budget)


def length_for_steer_limit(
    delta_d: float, max_steer_rad: float, wheelbase_m: float
) -> float:
    """조향 한계 안에서 Δd 를 끝내는 데 필요한 최소 s 길이.

    횡가속은 속도로 줄일 수 있지만 곡률은 못 줄인다. 그래서 이 길이는 속도와
    무관한 **기하학적 하한**이고, 저속에서 오히려 이쪽이 구속이 된다
    (예산 길이는 v 에 비례해 짧아지므로).

    tan(δ) = L_wb·κ = L_wb·5.7735·Δd/L²  →  L = sqrt(5.7735·Δd·L_wb/tan δ)
    """
    dd = abs(float(delta_d))
    if dd < 1e-6:
        return 0.0
    t = math.tan(max(1e-3, min(1.4, max_steer_rad)))
    return math.sqrt(D2_PEAK * dd * wheelbase_m / t)


def speed_for_length(delta_d: float, L: float, a_budget: float) -> float:
    """길이 L 안에서 Δd 를 예산 a 로 끝낼 수 있는 최대 속도. length_for_budget 의 역."""
    dd = abs(float(delta_d))
    if dd < 1e-6:
        return float("inf")
    if L <= 1e-6:
        return 0.0
    return L * math.sqrt(a_budget / (D2_PEAK * dd))


# ----------------------------------------------------------------------
# 장애물 묶기
# ----------------------------------------------------------------------
def group_blocking(
    obstacles: Sequence[ObstacleSD],
    cfg: ManeuverConfig,
    *,
    corridor_d: float = 0.0,
) -> ObstacleGroup | None:
    """기준선 주행을 막는 장애물들을 앞에서부터 한 덩어리로 묶는다.

    `corridor_d` 는 "막혔는지" 를 판정할 기준 횡위치다. 기본은 레이스라인(0).
    덩어리 안에 연속 장애물이 있으면 함께 넘겨서, 하나 피하고 라인에 붙었다가
    바로 다음 걸 또 피하는 톱니 거동을 막는다.
    """
    reach = cfg.half_width_m + cfg.lateral_margin_m
    blocking = [
        o
        for o in obstacles
        if o.s > 0.0 and abs(o.d - corridor_d) < (o.r + reach)
    ]
    if not blocking:
        return None
    blocking.sort(key=lambda o: o.s - o.r)

    first = blocking[0]
    s_first = first.s - first.r
    s_last = first.s + first.r
    d_left = first.d + first.r
    d_right = first.d - first.r
    count = 1
    for o in blocking[1:]:
        if (o.s - o.r) - s_last > cfg.merge_gap_m:
            break
        s_last = max(s_last, o.s + o.r)
        d_left = max(d_left, o.d + o.r)
        d_right = min(d_right, o.d - o.r)
        count += 1
    return ObstacleGroup(s_first, s_last, d_left, d_right, count)


def choose_pass_offset(
    group: ObstacleGroup,
    cfg: ManeuverConfig,
    d_ego: float,
    *,
    forbid_side: int = 0,
    max_left: float | None = None,
    max_right: float | None = None,
) -> Tuple[float, int] | None:
    """어느 쪽으로 얼마나 비킬지. 어느 쪽으로도 못 나가면 None.

    `max_left` / `max_right` 는 **그 방향으로 실제로 낼 수 있는 오프셋**이다.
    호출부가 점유맵에서 재서 넘긴다 (`_wall_budget_over`). 안 주면 `max_offset_m`
    로 좌우 같게 본다 — 트랙 경계를 모르는 상태이므로 테스트/폴백용이다.

    좌우를 따로 받는 게 중요하다. 레이스라인은 코너 안쪽에 붙으므로 한쪽은
    벽이고 반대쪽은 트랙이 통째로 남는다. 여유를 하나의 스칼라로 뭉개면
    "덜 움직이는 쪽" 이 곧 "벽 쪽" 인 경우를 못 걸러낸다.

    같은 조건이면 덜 움직이는 쪽을 고른다. 이미 한쪽으로 나가 있으면 그쪽이
    자연히 싸게 나오므로, 지나가는 중에 반대로 뒤집히지 않는다.

    `forbid_side` 는 "그쪽은 이미 해 봤는데 막히더라" 를 넣는 자리다.
    """
    cap_l = cfg.max_offset_m if max_left is None else min(max_left, cfg.max_offset_m)
    cap_r = cfg.max_offset_m if max_right is None else min(max_right, cfg.max_offset_m)

    reach = cfg.half_width_m + cfg.lateral_margin_m
    left = group.d_left + reach
    right = group.d_right - reach

    cands: List[Tuple[float, float, int]] = []
    if left <= cap_l and forbid_side != +1:
        cands.append((abs(left - d_ego), left, +1))
    if right >= -cap_r and forbid_side != -1:
        cands.append((abs(right - d_ego), right, -1))
    if not cands:
        return None
    cands.sort(key=lambda c: c[0])
    _, d_pass, side = cands[0]
    return d_pass, side


# ----------------------------------------------------------------------
# 기동 계획
# ----------------------------------------------------------------------
#: `kappa_ref` 를 훑는 간격 [m]. 코너 진입을 놓치지 않을 만큼만 촘촘하면 된다.
_KAPPA_SCAN_STEP_M = 0.25

#: 코너가 예산을 다 먹어도 길이 계산이 0 으로 나눠지지 않게 남겨 두는 바닥.
#: 여기에 걸렸다는 건 어차피 speed_cap 이 잡는다는 뜻이다.
_A_BUDGET_FLOOR = 0.30


def _kappa_span_max(
    kappa_ref: Callable[[float], float] | None, s_from: float, s_to: float
) -> float:
    """[s_from, s_to] 구간에서 기준선 |κ| 의 최대값."""
    if kappa_ref is None or s_to <= s_from:
        return 0.0
    out = 0.0
    n = max(2, int((s_to - s_from) / _KAPPA_SCAN_STEP_M) + 1)
    for i in range(n + 1):
        s = s_from + (s_to - s_from) * i / n
        out = max(out, abs(float(kappa_ref(s))))
    return out


def _budget_after_corner(a_budget: float, v: float, kappa: float) -> float:
    """코너가 이미 쓰고 있는 v²κ 를 뺀 나머지 횡가속 예산."""
    return max(_A_BUDGET_FLOOR, a_budget - v * v * abs(kappa))


def plan_maneuver(
    obstacles: Sequence[ObstacleSD],
    cfg: ManeuverConfig,
    *,
    d_ego: float,
    d_ego_prime: float,
    v: float,
    corridor_d: float = 0.0,
    forbid_side: int = 0,
    max_left: float | None = None,
    max_right: float | None = None,
    kappa_ref: Callable[[float], float] | None = None,
) -> OffsetManeuver | None:
    """횡오프셋 회피 기동 하나를 계획한다. 못 만들면 None.

    None 은 "지나갈 길이 트랙 안에 없다" 는 뜻이므로, 호출부는 감속·정지나
    다른 수단으로 넘겨야 한다.

    `max_left` / `max_right` 로 방향별 여유를 넘기면 벽 쪽 계획이 아예 안
    나온다. 안 주면 `cfg.max_offset_m` 만 보므로 트랙 경계를 무시한다.

    `kappa_ref(ds)` 는 기동 시작점에서 전방 `ds` 인 지점의 **기준선 곡률**을
    돌려준다. 주면 코너가 쓰는 v²κ 를 예산에서 먼저 빼고, 조향 한계와 속도
    상한도 |κ|+|d''| 로 본다. 안 주면 직선 가정(예전 동작)이다.
    """
    group = group_blocking(obstacles, cfg, corridor_d=corridor_d)
    if group is None:
        return None
    picked = choose_pass_offset(
        group,
        cfg,
        d_ego,
        forbid_side=forbid_side,
        max_left=max_left,
        max_right=max_right,
    )
    if picked is None:
        return None
    d_pass, side = picked

    v_plan = max(abs(v), cfg.v_plan_min_mps)
    delta_in = d_pass - d_ego

    # 오프셋에 올라와 있어야 하는 지점까지 남은 거리.
    runway = group.s_first - cfg.hold_front_m
    if runway <= 0.05:
        # 이미 장애물 옆이다 — 계획으로 풀 수 있는 상황이 아니다.
        return None

    # 길이의 하한은 둘이다. 횡가속 예산에서 오는 것(속도에 비례)과 조향
    # 한계에서 오는 것(속도 무관). 저속에서는 후자가 구속이 된다.
    steer_min_in = length_for_steer_limit(
        delta_in, cfg.max_steer_rad, cfg.wheelbase_m
    )
    # 진입 곡선은 [0, runway] 안 어딘가에 놓인다. 그 구간의 코너가 쓰는 만큼을
    # 예산에서 먼저 뺀다 — 남은 것으로 길이를 뽑아야 합이 접지력 안에 든다.
    kappa_in = _kappa_span_max(kappa_ref, 0.0, runway)
    a_enter = _budget_after_corner(cfg.a_lat_enter_mps2, v_plan, kappa_in)
    need = max(
        length_for_budget(delta_in, v_plan, a_enter),
        steer_min_in,
        cfg.enter_min_m,
    )
    if steer_min_in > runway:
        # 남은 거리로는 어떤 속도에서도 못 꺾는다. 제동으로 넘겨야 한다.
        return None
    # 남은 거리를 다 쓰는 쪽이 언제나 더 완만하다. 다만 한없이 길게 잡으면
    # 장애물과 무관한 구간까지 라인을 벗어나므로 상한을 둔다.
    enter_len = min(need, max(cfg.enter_max_m, steer_min_in), runway)
    enter_len = max(enter_len, 1e-3)

    # 진입에 필요한 길이보다 장애물이 멀면, 남는 만큼은 **라인 위에서** 간다.
    # 여기서 곧장 비키기 시작해도 조향은 같지만(길이가 같으니), 레이스라인을
    # 일찍 떠날 이유가 없다. 그쪽이 빠른 선이다. 진입 시점만 미루는 것이라
    # "멀리서부터 완만하게" 는 그대로다.
    lead = max(0.0, runway - enter_len)

    # 리드 구간을 지나는 동안 차는 기준선과 나란히 가므로, 진입 곡선이
    # 시작되는 지점의 기울기는 0 이다. 지금의 기울기를 거기 붙이면 꺾인다.
    enter_d0p = d_ego_prime if lead <= 1e-6 else 0.0
    enter_coeff = solve_quintic(
        d_ego, enter_d0p, 0.0, d_pass, 0.0, 0.0, enter_len
    )
    peak_in = peak_abs_d2(enter_coeff, enter_len)

    hold_end = max(lead + enter_len, group.s_last + cfg.hold_rear_m)

    # 복귀 구간의 코너도 같은 방식으로 뺀다. 길이가 정해져야 구간을 알고,
    # 구간을 알아야 길이가 정해지는 순환이라 두 번 돌린다 — 처음에는
    # exit_max_m 까지 넓게 보고, 나온 길이로 한 번 더 좁혀 잡는다.
    steer_min_out = length_for_steer_limit(
        d_pass, cfg.max_steer_rad, cfg.wheelbase_m
    )
    exit_len = cfg.exit_max_m
    kappa_out = 0.0
    for _ in range(2):
        kappa_out = _kappa_span_max(kappa_ref, hold_end, hold_end + exit_len)
        a_exit = _budget_after_corner(cfg.a_lat_exit_mps2, v_plan, kappa_out)
        exit_len = min(
            max(
                length_for_budget(d_pass, v_plan, a_exit),
                steer_min_out,
                cfg.exit_min_m,
            ),
            cfg.exit_max_m,
        )
    exit_coeff = solve_quintic(d_pass, 0.0, 0.0, 0.0, 0.0, 0.0, exit_len)
    peak_out = peak_abs_d2(exit_coeff, exit_len)

    # 접지력이 보는 건 기준선 곡률과 기동 곡률의 **합**이다. 구간별로 더한 뒤
    # 그중 최대를 쓴다 — 진입의 d'' 를 복귀 구간 코너에 더하면 없는 부하다.
    peak_kappa = max(kappa_in + peak_in, kappa_out + peak_out)
    peak_a = v_plan * v_plan * peak_kappa

    # 조향 한계를 넘는 경로는 감속해도 못 탄다. 횡가속은 속도로 줄일 수 있지만
    # 곡률은 속도와 무관하기 때문이다. 이런 건 계획 실패로 돌려서 호출부가
    # 제동으로 넘기게 한다 — 못 타는 경로를 발행하면 그대로 벽으로 간다.
    if steering_for_offset(peak_kappa, cfg.wheelbase_m) > cfg.max_steer_rad:
        return None

    # 예산을 넘으면 조향을 더 넣는 대신 속도로 답한다.
    speed_cap = None
    if peak_a > cfg.a_lat_hard_mps2 and peak_kappa > 1e-9:
        speed_cap = math.sqrt(cfg.a_lat_hard_mps2 / peak_kappa)

    return OffsetManeuver(
        d_start=d_ego,
        d_pass=d_pass,
        side=side,
        lead_len_m=lead,
        enter_len_m=enter_len,
        hold_end_ds=hold_end,
        exit_len_m=exit_len,
        enter_coeff=enter_coeff,
        exit_coeff=exit_coeff,
        peak_lateral_accel_mps2=peak_a,
        speed_cap_mps=speed_cap,
        obstacle_s_first=group.s_first,
        obstacle_s_last=group.s_last,
    )


def steering_for_offset(peak_d2: float, wheelbase_m: float) -> float:
    """|d''| 가 요구하는 전륜 조향각 [rad]. 기준선이 직선일 때의 값이다."""
    return math.atan(wheelbase_m * abs(peak_d2))
