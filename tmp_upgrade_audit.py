#!/usr/bin/env python3
"""path_following 업그레이드 [1]~[5] 오프라인 검증."""
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

# install/ 사본이 아니라 반드시 src 를 검증한다. colcon 이 symlink-install 이
# 아니면 install 쪽은 빌드 시점에 얼어붙어 있어서, 고친 걸 안 고쳤다고 한다.
sys.path.insert(0, str(Path(__file__).resolve().parent / "src" / "path_following"))

from path_following.local_planner_node import LocalPlannerNode  # noqa: E402

CSV = "/home/nvidia/f1tenth_ajou/src/path_following/config/raceline.csv"

FAILS = []


def check(name, ok, info=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}  {info}")
    if not ok:
        FAILS.append(name)


def make_node(**overrides):
    params = [Parameter("csv_path", Parameter.Type.STRING, CSV)]
    for k, v in overrides.items():
        if isinstance(v, bool):
            t = Parameter.Type.BOOL
        elif isinstance(v, int):
            t = Parameter.Type.INTEGER
        elif isinstance(v, float):
            t = Parameter.Type.DOUBLE
        else:
            t = Parameter.Type.STRING
        params.append(Parameter(k, t, v))
    rclpy.init(args=["--ros-args"] + sum(
        [["-p", f"{p.name}:={p.value}"] for p in params], []
    ))
    return LocalPlannerNode()


def pose_at(node, s, d=0.0):
    x, y, yaw = node._xy_yaw_at_s(s)
    p = PoseStamped()
    p.pose.position.x = x - d * math.sin(yaw)
    p.pose.position.y = y + d * math.cos(yaw)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def main():
    # TF 가 없는 오프라인 검증이므로 lookup 타임아웃을 0 으로 둔다.
    # (남겨두면 호출마다 블로킹돼 _update_mode 한 번이 0.6초까지 늘어난다)
    n = make_node(
        path_check_enable=False, verbose_logs=True, obstacle_tf_timeout_sec=0.0
    )
    L = n._total_l
    print(f"track length = {L:.2f} m, points = {len(n.points)}\n")

    # --- [1] Frenet ---
    check("_delta_s 랩어라운드 (+)", abs(n._delta_s(0.5, L - 0.5) - 1.0) < 1e-6,
          f"={n._delta_s(0.5, L - 0.5):.4f}")
    check("_delta_s 랩어라운드 (-)", abs(n._delta_s(L - 0.5, 0.5) + 1.0) < 1e-6,
          f"={n._delta_s(L - 0.5, 0.5):.4f}")
    check("_delta_s 범위 [-L/2, L/2)",
          all(-L / 2 <= n._delta_s(a, 0.0) < L / 2
              for a in [0.0, L * 0.25, L * 0.5, L * 0.75, L - 1e-3]))

    errs = []
    for frac in [0.0, 0.13, 0.37, 0.5, 0.66, 0.91]:
        s_t, d_t = frac * L, 0.3
        x, y, yaw = n._xy_yaw_at_s(s_t)
        px, py = x - d_t * math.sin(yaw), y + d_t * math.cos(yaw)
        s_b, d_b = n._frenet_xy(px, py)
        errs.append((abs(n._delta_s(s_b, s_t)), abs(d_b - d_t)))
    check("_frenet_xy 왕복 정확도", max(e[0] for e in errs) < 0.10
          and max(e[1] for e in errs) < 0.05,
          f"max ds={max(e[0] for e in errs):.4f}m dd={max(e[1] for e in errs):.4f}m")

    # --- [1]/[2] 전방 갭 ---
    n._s_ego, n._d_ego = 10.0, 0.0
    n._dynamic_sd = [(12.0, 0.0, 0.2, 1.0, 0.5)]
    gap = n._forward_gap_s_m()
    check("전방 갭 = ds - r - front_safety",
          abs(gap - (2.0 - 0.2 - n.ego_front_safety_m)) < 1e-6, f"={gap:.3f}m")

    n._dynamic_sd = [(8.0, 0.0, 0.2, 1.0, -0.5)]
    check("후방 장애물은 갭에서 제외", not math.isfinite(n._forward_gap_s_m()))

    n._s_ego = L - 1.0
    n._dynamic_sd = [(1.0, 0.0, 0.2, 1.0, 0.5)]
    g2 = n._forward_gap_s_m()
    check("결승선 넘어가는 전방차도 잡음",
          abs(g2 - (2.0 - 0.2 - n.ego_front_safety_m)) < 1e-6, f"={g2:.3f}m")

    # --- [5] 예측 s ---
    n._s_ego = 10.0
    n._dynamic_sd = [(12.0, 0.0, 0.2, 3.0, 0.0)]   # vs=+3 m/s (멀어짐)
    n._use_predicted_s = False
    g_now = n._forward_gap_s_m()
    n._use_predicted_s = True
    n._pred_horizon_sec = 1.0
    g_pred = n._forward_gap_s_m()
    check("예측 s: 멀어지는 앞차는 갭이 커짐",
          g_pred > g_now + 2.9, f"now={g_now:.2f} pred={g_pred:.2f}")
    n._dynamic_sd = [(12.0, 0.0, 0.2, -1.0, 0.0)]  # 마주 옴
    g_pred2 = n._forward_gap_s_m()
    check("예측 s: 다가오는 앞차는 갭이 줄어듦",
          g_pred2 < g_now - 0.9, f"now={g_now:.2f} pred={g_pred2:.2f}")
    n._dynamic_sd = [(12.0, 0.0, 0.2, -5.0, 0.0)]  # 예측상 자차를 지나침
    check("예측이 자차를 지나쳐도 사라지지 않음 (갭 0 으로 클램프)",
          n._forward_gap_s_m() == 0.0, f"={n._forward_gap_s_m()}")
    n._use_predicted_s = False

    # --- [2] TRAILING 추종 속도 (기준 = 앞차 속도) ---
    V_LEAD = 2.0          # 따라갈 만한 앞차 (min 0.5, deficit 조건도 통과)
    V_CSV = 3.0

    def leader(s_gap, vs=V_LEAD, r=0.2):
        """갭이 s_gap 이 되는 앞차 하나."""
        return [(10.0 + s_gap + r + n.ego_front_safety_m, 0.0, r, vs, 0.0)]

    n._reset_trailing_state()
    n._s_ego = 10.0
    n._dynamic_sd = leader(n.trailing_target_gap_m)
    v_on = n._trailing_target_speed(V_CSV)
    check("갭=목표면 앞차 속도로 수렴", abs(v_on - V_LEAD) < 1e-6,
          f"={v_on:.3f} (앞차 {V_LEAD})")

    n._reset_trailing_state()
    n._dynamic_sd = leader(0.05)
    v_close = n._trailing_target_speed(V_CSV)
    check("갭이 좁으면 앞차보다 느리게", v_close < V_LEAD, f"={v_close:.3f}")
    check("음수 속도는 안 나옴", v_close >= 0.0, f"={v_close:.3f}")

    n._reset_trailing_state()
    n._dynamic_sd = leader(6.0)
    v_far = n._trailing_target_speed(V_CSV)
    check("갭이 넓어도 CSV 속도를 넘지 않음", v_far <= V_CSV + 1e-9,
          f"={v_far:.3f} <= {V_CSV}")

    n._reset_trailing_state()
    n._dynamic_sd = []
    check("앞차 없으면 CSV 속도 그대로",
          n._trailing_target_speed(V_CSV) == V_CSV)

    n._reset_trailing_state()
    n._dynamic_sd = leader(0.0)
    check("갭 0 이면 제동거리 상한이 앞차 속도로 묶음",
          n._trailing_target_speed(V_CSV) <= V_LEAD + 1e-9)

    # --- [2] 상태 전이 ---
    def upd(node, **kw):
        args = dict(d_closest=float("inf"), d_gate=float("inf"), filtered=[],
                    current_pose=None, filtered_dynamic=[],
                    d_dyn_closest=float("inf"), d_dyn_gate=float("inf"),
                    rel_speed=0.0)
        args.update(kw)
        node._update_mode(**args)

    def enter_trailing():
        """앞차 래치가 서려면 leader_enter_count_th 프레임이 필요하다."""
        n._go_global()
        n._clear_avoid_blocked()
        # 포즈가 없으면 CSV 속도 = avoid_speed_ref(2.0) 로 잡혀서 V_LEAD 가
        # speed_deficit 조건을 통과한다. 포즈가 남아 있으면 CSV 3 m/s 기준이
        # 돼 "너무 느린 앞차" 로 분류되고 AVOID 로 간다.
        n._last_pose_for_speed = None
        n._leader_latched = False
        n._leader_seen_count = 0
        n._s_ego = 10.0
        n._dynamic_sd = leader(1.5)          # < trailing_enter_m 3.0
        for _ in range(n.leader_enter_count_th + 1):
            upd(n)

    enter_trailing()
    check("GLOBAL -> TRAILING", n.mode == "TRAILING", f"mode={n.mode}")

    n._dynamic_sd = leader(20.0)
    for _ in range(n.trailing_exit_count_th):
        upd(n)
    check("TRAILING -> GLOBAL (히스테리시스)", n.mode == "GLOBAL", f"mode={n.mode}")

    enter_trailing()
    assert n.mode == "TRAILING", n.mode
    upd(n, d_closest=0.5, filtered=[0.0, 0.5, 0.0, 0.2])
    check("TRAILING -> AVOID", n.mode == "AVOID", f"mode={n.mode}")

    n._mark_avoid_blocked()
    upd(n, d_closest=0.5, filtered=[0.0, 0.5, 0.0, 0.2])
    check("AVOID -> TRAILING (경로 막힘)", n.mode == "TRAILING", f"mode={n.mode}")
    check("전이 후에도 래치 유지 (떨림 방지)", n._avoid_blocked())

    # --- 모드 떨림 회귀: 막힌 정적 장애물 앞에서 40Hz 로 2초 ---
    # 예전엔 bool 이 전이마다 리셋돼 AVOID↔TRAILING 을 2프레임 주기로 왕복했다.
    # 그때마다 AEB 완화 기준이 깜빡이고 TRAILING PID 미분항이 리셋됐다.
    blocked_obs = dict(d_closest=0.5, filtered=[0.0, 0.5, 0.0, 0.2])
    n._go_global()
    n._s_ego = 10.0
    n._dynamic_sd = []
    seq = []
    for _ in range(80):                      # 2 s @ 40 Hz
        if n.mode == "AVOID":
            n._mark_avoid_blocked()          # 경로 생성 실패를 매번 재현
        upd(n, **blocked_obs)
        seq.append(n.mode)
        time.sleep(0.025)
    switches = sum(1 for i in range(1, len(seq)) if seq[i] != seq[i - 1])
    # retry_sec=0.5 → 2초에 AVOID 재시도 4번 = 전이 8회 이내
    check("모드 떨림 억제 (정적, 2초간 전이 횟수)", switches <= 10,
          f"{switches}회 (예전 bool 방식이면 ~78회)")
    check("따라갈 앞차 없는 정적 장애물은 GLOBAL 로 수렴 (AEB 완화 안 함)",
          n.mode == "GLOBAL", f"mode={n.mode}")
    check("GLOBAL 로 내려가도 감속은 유지 (속도 정책은 모드 무관)",
          not n._override_active)

    # 같은 상황인데 따라갈 앞차가 있으면 TRAILING 으로 붙어 있어야 한다
    n._clear_avoid_blocked()
    enter_trailing()
    n._dynamic_sd = leader(1.5)
    seq = []
    for _ in range(80):
        if n.mode == "AVOID":
            n._mark_avoid_blocked()
        upd(n, **blocked_obs)
        seq.append(n.mode)
        time.sleep(0.025)
    sw2 = sum(1 for i in range(1, len(seq)) if seq[i] != seq[i - 1])
    check("모드 떨림 억제 (앞차 있음)", sw2 <= 10, f"{sw2}회")
    check("앞차 있으면 TRAILING 비중이 지배적",
          seq.count("TRAILING") > len(seq) * 0.6,
          f"TRAILING {seq.count('TRAILING')}/{len(seq)} 프레임")
    n._clear_avoid_blocked()
    n._dynamic_sd = []

    # --- AVOID 탈출 교착 회귀 ---
    # rejoin 을 기본 켜면서 "pose 가 없다" 와 "pose 는 있는데 CTE 가 크다" 가
    # 한 조건으로 묶였고, 둘 다 pass 로 빠져 AVOID 에 영구히 갇혔다.
    # 그러면 장애물이 0 개인데 /planner/fgm_enable 이 계속 True 로 나간다.
    n.mode = "AVOID"
    n._avoid_off_count = 0
    for _ in range(n.avoid_off_count_th + 3):
        upd(n)                                   # 장애물 없음 + pose 없음
    check("pose 없으면 AVOID 즉시 탈출 (fgm_enable 고착 방지)",
          n.mode == "GLOBAL", f"mode={n.mode}")

    # pose 는 있는데 차가 서 있으면 CTE 가 줄지 않는다. 대기에 상한이 없으면
    # 이쪽으로도 똑같이 갇힌다.
    wait_frames = int(n.rejoin_wait_max_ns * n.publish_hz / 1e9)
    off_pose = pose_at(n, 10.0, d=1.0)           # CTE 1 m > 0.20 m
    n.mode = "AVOID"
    n._avoid_off_count = 0
    for _ in range(n.avoid_off_count_th + wait_frames - 2):
        upd(n, current_pose=off_pose)
    check("상한 안에서는 CTE 줄기를 기다린다", n.mode == "AVOID", f"mode={n.mode}")
    for _ in range(4):
        upd(n, current_pose=off_pose)
    check("정지 + CTE 큼도 상한 뒤 AVOID 탈출", n.mode != "AVOID", f"mode={n.mode}")
    n._go_global()
    n._rejoin_path_msg = None

    # --- AEB 탈출: 정적 장애물 정면에 멈춤 → AVOID 강제 ---
    n._go_global()
    n._ego_speed_mps = 3.0
    n._cb_aeb(Bool(data=True))
    upd(n, **blocked_obs)
    check("제동 중(고속)엔 탈출 미개입", not n._aeb_escape_active(),
          f"mode={n.mode}, v={n._ego_speed_mps}")

    n._ego_speed_mps = 0.0
    n._mark_avoid_blocked()                  # 경로가 막혀 있어도
    upd(n, **blocked_obs)
    check("멈춘 뒤 탈출 모드 = AVOID 강제", n.mode == "AVOID", f"mode={n.mode}")
    check("탈출 진입 시 막힘 래치 해제 (경로 재시도)", not n._avoid_blocked())

    for _ in range(20):                      # AEB 가 계속 걸린 채 정지
        n._mark_avoid_blocked()
        upd(n, **blocked_obs)
    check("탈출 중 TRAILING 으로 안 빠짐", n.mode == "AVOID", f"mode={n.mode}")

    n._cb_aeb(Bool(data=False))
    n._ego_speed_mps = 0.6                   # 빠져나가는 중
    upd(n, **blocked_obs)
    check("AEB 해제 후에도 hold 동안 AVOID 유지",
          n.mode == "AVOID" and n._aeb_escape_active(), f"mode={n.mode}")

    # 탈출 중 속도 상한
    n._last_pose_for_speed = pose_at(n, 30.0)
    n._speed_static_obs = []
    n._speed_dynamic_obs = []
    n._dynamic_sd = []
    n._override_active = True
    n._slew_prev_v = None
    sc_esc = n._planner_speed_scale()
    v_csv = n._csv_speed_now()
    check("탈출 중 속도 상한 적용",
          sc_esc * v_csv <= n.aeb_escape_speed_mps + 1e-6,
          f"v={sc_esc * v_csv:.2f} <= {n.aeb_escape_speed_mps}m/s (csv={v_csv:.2f})")
    check("탈출 속도가 0 은 아님 (기어 나갈 수 있음)", sc_esc * v_csv > 0.1,
          f"v={sc_esc * v_csv:.2f}m/s")

    n._aeb_escape_until_ns = 0               # 탈출 창 강제 종료
    n._aeb_active = False
    n._ego_speed_mps = 0.0
    n._override_active = False
    n._slew_prev_v = None

    # pose 없는 프레임이 TRAILING 을 풀면 안 된다
    enter_trailing()
    assert n.mode == "TRAILING", n.mode
    n._s_ego = None
    for _ in range(n.trailing_exit_count_th * 3):
        upd(n)
    check("pose 없는 프레임은 TRAILING 유지", n.mode == "TRAILING", f"mode={n.mode}")
    n._s_ego = 10.0

    n._go_global()
    n.trailing_enable = False
    n._dynamic_sd = leader(1.5)
    for _ in range(n.leader_enter_count_th + 1):
        upd(n)
    check("trailing_enable=False 면 미진입", n.mode == "GLOBAL", f"mode={n.mode}")
    n.trailing_enable = True
    n._go_global()
    n._dynamic_sd = []

    # --- [3] REJOIN 길이 속도 연동 ---
    check("rejoin 기본 활성화", n.rejoin_enable is True)
    for v, expect in [(0.0, n.rejoin_min_length_m),
                      (3.0, min(n.rejoin_max_length_m, 0.8 * 3.0)),
                      (8.0, n.rejoin_max_length_m)]:
        n._ego_speed_mps = v
        p = pose_at(n, 20.0, d=0.35)
        path = n._build_frenet_quintic_rejoin_path(p)
        got = n._delta_s(n._rejoin_target_s, n._frenet_xy(
            p.pose.position.x, p.pose.position.y)[0])
        check(f"rejoin L @ v={v}m/s", abs(got - expect) < 0.05,
              f"L={got:.2f} (기대 {expect:.2f}), poses={len(path.poses)}")

    n._ego_speed_mps = 3.0
    p = pose_at(n, 20.0, d=0.35)
    path = n._build_frenet_quintic_rejoin_path(p)
    d_end = abs(n._frenet_xy(path.poses[n.rejoin_sample_count - 1].pose.position.x,
                             path.poses[n.rejoin_sample_count - 1].pose.position.y)[1])
    check("rejoin 끝에서 |d|≈0", d_end < 0.05, f"|d|={d_end:.3f}m")

    # --- [4] frenet AVOID 경로 ---
    n.avoid_path_mode = "frenet"
    p = pose_at(n, 30.0, d=0.0)
    xr, yr, yawr = n._xy_yaw_at_s(31.5)
    tgt = (xr - 0.5 * math.sin(yawr), yr + 0.5 * math.cos(yawr))
    fp = n._build_avoid_path_frenet(p, tgt[0], tgt[1])
    check("frenet 경로 생성됨", fp is not None and len(fp.poses) > 10,
          f"poses={len(fp.poses) if fp else 0}")

    ds_list = [abs(n._frenet_xy(q.pose.position.x, q.pose.position.y)[1])
               for q in fp.poses]
    check("|d| <= max_offset", max(ds_list) <= n.avoid_frenet_max_offset_m + 1e-3,
          f"max|d|={max(ds_list):.3f} <= {n.avoid_frenet_max_offset_m}")
    check("피크 오프셋이 목표에 도달", max(ds_list) > 0.45, f"peak={max(ds_list):.3f}")
    check("경로 끝에서 레이스라인 복귀", ds_list[-1] < 0.05, f"|d|_end={ds_list[-1]:.3f}")

    steps = [math.hypot(fp.poses[i].pose.position.x - fp.poses[i - 1].pose.position.x,
                        fp.poses[i].pose.position.y - fp.poses[i - 1].pose.position.y)
             for i in range(1, len(fp.poses))]
    check("점 간격 연속 (튐 없음)", max(steps) < 3.0 * n.avoid_frenet_step_m,
          f"max step={max(steps):.3f}m")

    xr, yr, yawr = n._xy_yaw_at_s(31.5)
    far = (xr - 2.0 * math.sin(yawr), yr + 2.0 * math.cos(yawr))
    fp2 = n._build_avoid_path_frenet(p, far[0], far[1])
    d2 = max(abs(n._frenet_xy(q.pose.position.x, q.pose.position.y)[1])
             for q in fp2.poses)
    check("과한 목표 오프셋 클램프",
          d2 <= n.avoid_frenet_max_offset_m + 1e-3, f"max|d|={d2:.3f}")

    n.avoid_path_mode = "straight"
    sp = n._build_avoid_path(p, tgt[0], tgt[1], merge_csv_tail=False)
    check("straight 모드 회귀 정상", len(sp.poses) >= n.avoid_forward_num_points,
          f"poses={len(sp.poses)}")

    # --- 속도 우선순위 회귀 ---
    n._go_global()
    n._last_pose_for_speed = pose_at(n, 30.0)
    n._speed_static_obs = []
    n._speed_dynamic_obs = []
    n._dynamic_sd = []
    n._override_active = False
    sc_free = n._planner_speed_scale()
    check("장애물 없을 때 배율 1.0", abs(sc_free - 1.0) < 1e-6, f"={sc_free:.3f}")

    # 20Hz 로 1초간 돌려서 slew 를 통과한 정상 상태를 본다
    n.mode = "TRAILING"
    n._reset_trailing_state()
    n._s_ego = 30.0
    n._dynamic_sd = [(30.3, 0.0, 0.1, 1.0, 0.0)]
    hist = []
    for _ in range(20):
        hist.append(n._planner_speed_scale())
        time.sleep(0.05)
    sc_trail = hist[-1]
    check("TRAILING 이 배율을 낮춤", sc_trail < 1.0, f"={sc_trail:.3f}")
    check("TRAILING 배율이 음수가 아님", sc_trail >= 0.0, f"={sc_trail:.3f}")
    check("감속이 slew 로 완만함 (스텝 급락 없음)",
          all(hist[i] >= hist[i - 1] - 0.35 for i in range(1, len(hist))),
          f"최대 하강폭={max(hist[i-1]-hist[i] for i in range(1,len(hist))):.3f}/틱")

    n._dynamic_sd = [(40.0, 0.0, 0.1, 0.0, 0.0)]
    for _ in range(10):
        sc_back = n._planner_speed_scale()
        time.sleep(0.05)
    check("갭이 벌어지면 배율 회복", sc_back > sc_trail, f"{sc_trail:.3f} -> {sc_back:.3f}")

    n.destroy_node()
    rclpy.shutdown()

    print()
    if FAILS:
        print(f"FAIL {len(FAILS)}건: {FAILS}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
