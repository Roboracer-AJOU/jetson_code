"""시나리오 러너. 스택은 밖에서 띄워두고 이 스크립트가 차량/센서를 돌린다.

  python3 sim/run_scenario.py <scenario> [duration_s]
"""
from __future__ import annotations

import math
import sys

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/sim")

import rclpy
from race_sim import MAP_YAML as MAP
from race_sim import GridMap, Obstacle, PathObstacle, RaceSim

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/src/path_following")
from path_following import track_sliding

CSV = "/home/nvidia/f1tenth_ajou/src/path_following/config/raceline.csv"


def load_line(path):
    pts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line[0] in "#xX":
                continue
            c = line.split(",")
            pts.append((float(c[0]), float(c[1]), float(c[2]) if len(c) > 2 else 0.0))
    return pts


LINE = load_line(CSV)
# 스택은 track_sliding.DEFAULT_REVERSE_TRACK 방향으로 달린다. 시뮬 시작 자세와
# 장애물 배치도 같은 방향을 써야 한다.
if track_sliding.DEFAULT_REVERSE_TRACK:
    LINE = list(reversed(LINE))


def pose_at(i: int):
    """웨이포인트 i 의 (x, y, yaw)."""
    n = len(LINE)
    x, y, _ = LINE[i % n]
    x2, y2, _ = LINE[(i + 4) % n]
    return x, y, math.atan2(y2 - y, x2 - x)


def straight_start(need_m: float = 14.0) -> int:
    """앞으로 need_m 동안 가장 곧은 구간의 시작 웨이포인트."""
    n = len(LINE)
    best_i, best_turn = 0, 1e9
    for i in range(n):
        acc, j, turn = 0.0, i, 0.0
        while acc < need_m:
            a, b, c = LINE[j % n], LINE[(j + 1) % n], LINE[(j + 2) % n]
            acc += math.hypot(b[0] - a[0], b[1] - a[1])
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            turn += abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
            j += 1
        if turn < best_turn:
            best_turn, best_i = turn, i
    return best_i


def ahead(i: int, dist_m: float, lateral: float = 0.0):
    """웨이포인트 i 에서 경로 따라 dist_m 앞, 횡으로 lateral 이동한 지점."""
    return LINE_xy(index_ahead(i, dist_m), lateral)


def index_ahead(i: int, dist_m: float) -> int:
    """웨이포인트 i 에서 경로 호길이로 dist_m 앞의 인덱스."""
    n = len(LINE)
    acc, j = 0.0, i
    while acc < dist_m:
        a, b = LINE[j % n], LINE[(j + 1) % n]
        acc += math.hypot(b[0] - a[0], b[1] - a[1])
        j += 1
    return j % n


def index_back(i: int, dist_m: float) -> int:
    """웨이포인트 i 에서 경로 호길이로 dist_m 뒤의 인덱스.

    웨이포인트 개수로 세면 안 된다 — raceline 은 곡률에 따라 점 간격이
    2배 넘게 차이나서, 130점 뒤가 코너에선 5m 도 안 된다. corner_cone 이
    "반응할 거리를 준다" 고 해놓고 실제로는 5m 앞에 콘을 두고 있었다.
    """
    n = len(LINE)
    acc, j = 0.0, i
    while acc < dist_m:
        a, b = LINE[(j - 1) % n], LINE[j % n]
        acc += math.hypot(b[0] - a[0], b[1] - a[1])
        j -= 1
    return j % n


def LINE_xy(j: int, lateral: float = 0.0):
    """웨이포인트 j 를 횡으로 lateral 만큼 민 좌표."""
    n = len(LINE)
    x, y, _ = LINE[j % n]
    x2, y2, _ = LINE[(j + 3) % n]
    yaw = math.atan2(y2 - y, x2 - x)
    return x - lateral * math.sin(yaw), y + lateral * math.cos(yaw)


def pose_off(i: int, lateral: float = 0.0, heading_err_deg: float = 0.0):
    """웨이포인트 i 에서 횡으로 lateral, 헤딩을 heading_err 만큼 틀어 시작.

    모든 기존 시나리오는 라인 위에서 정확히 정렬된 채 출발한다. 실차는
    절대 그렇게 시작하지 않는다 — 수동으로 놓은 위치·초기 로컬라이제이션
    오차가 항상 있고, 추종기가 거기서 수렴하는지가 진짜 문제다.
    """
    x, y, yaw = pose_at(i)
    ox, oy = LINE_xy(i, lateral)
    return (ox, oy), yaw + math.radians(heading_err_deg)


_GMAP = None


def gmap():
    global _GMAP
    if _GMAP is None:
        _GMAP = GridMap(MAP)
    return _GMAP


def narrowest_index() -> int:
    """레이스라인에서 벽 여유가 가장 좁은 웨이포인트."""
    m = gmap()
    best_i, best_c = 0, 1e9
    for i, (x, y, _v) in enumerate(LINE):
        c = m.wall_clearance(x, y)
        if c < best_c:
            best_c, best_i = c, i
    return best_i


def sharpest_index() -> int:
    """곡률이 가장 큰 웨이포인트."""
    n = len(LINE)
    best_i, best_k = 0, 0.0
    for i in range(n):
        a, b, c = LINE[(i - 6) % n], LINE[i], LINE[(i + 6) % n]
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        cr = abs(v1[0] * v2[1] - v1[1] * v2[0])
        d = math.hypot(*v1) * math.hypot(*v2) + 1e-9
        if cr / d > best_k:
            best_k, best_i = cr / d, i
    return best_i


# ---------------------------------------------------------------- 시나리오
def scen_clean():
    x, y, yaw = pose_at(0)
    return (x, y), yaw, [], "무장애물 클린랩"


def scen_cone_straight():
    i = straight_start()
    x, y, yaw = pose_at(i)
    ox, oy = ahead(i, 11.0)
    return (x, y), yaw, [Obstacle(ox, oy, 0.16, name="cone")], "직선 정면 콘 (9m 앞)"


def scen_cone_offset():
    i = straight_start()
    x, y, yaw = pose_at(i)
    ox, oy = ahead(i, 11.0, lateral=0.18)
    return (x, y), yaw, [Obstacle(ox, oy, 0.16, name="cone")], "직선 살짝 치우친 콘"


def scen_two_cones():
    i = straight_start()
    x, y, yaw = pose_at(i)
    o1 = ahead(i, 10.0, lateral=0.20)
    o2 = ahead(i, 14.0, lateral=-0.25)
    return (
        (x, y),
        yaw,
        [Obstacle(*o1, 0.16, name="c1"), Obstacle(*o2, 0.16, name="c2")],
        "연속 콘 2개 (지그재그)",
    )


def scen_corner_cone():
    """곡률이 가장 큰 지점 부근에 콘. 접근거리는 호길이 16m 로 고정."""
    best_i = sharpest_index()
    start = index_back(best_i, 16.0)
    x, y, yaw = pose_at(start)
    ox, oy = LINE[best_i][0], LINE[best_i][1]
    return (x, y), yaw, [Obstacle(ox, oy, 0.16, name="corner")], "코너 정점 콘 (16m 접근)"


def scen_dynamic_slow():
    """같은 방향으로 느리게 가는 앞차."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    ox, oy = ahead(i, 8.0)
    ox2, oy2 = ahead(i, 8.5)
    h = math.atan2(oy2 - oy, ox2 - ox)
    v = 1.0
    return (
        (x, y),
        yaw,
        [Obstacle(ox, oy, 0.20, v * math.cos(h), v * math.sin(h), "slowcar")],
        "느린 앞차 (1.0 m/s, 6m 앞)",
    )


def scen_dynamic_cross():
    """옆에서 갑자기 끼어드는 물체."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    ox, oy = ahead(i, 10.0, lateral=2.2)
    return (
        (x, y),
        yaw,
        [Obstacle(ox, oy, 0.18, 0.0, -1.2, "crosser")],
        "측면 횡단 물체 (1.2 m/s)",
    )


def scen_sudden_wall():
    """바로 앞 2.5m 에 갑자기 나타난 큰 장애물 — AEB 영역."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    ox, oy = ahead(i, 4.0)
    return (
        (x, y),
        yaw,
        [Obstacle(ox, oy, 0.45, name="sudden")],
        "근거리 대형 장애물 (3m, AEB)",
    )


def scen_blocked():
    """트랙을 거의 다 막은 경우 — 세워야 한다."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    o1 = ahead(i, 9.0, lateral=0.55)
    o2 = ahead(i, 9.0, lateral=-0.55)
    o3 = ahead(i, 9.0, lateral=0.0)
    return (
        (x, y),
        yaw,
        [
            Obstacle(*o1, 0.30, name="b1"),
            Obstacle(*o2, 0.30, name="b2"),
            Obstacle(*o3, 0.30, name="b3"),
        ],
        "트랙 완전 봉쇄 (정지 기대)",
    )


# ------------------------------------------------- 추가: 경로추종 (장애물 없음)
def scen_offset_start():
    """라인에서 0.45m 벗어난 채 출발 — Stanley 수렴 확인."""
    i = straight_start()
    start, yaw = pose_off(i, lateral=0.45)
    return start, yaw, [], "횡오차 0.45m 로 출발"


def scen_heading_err_start():
    """헤딩이 35도 틀어진 채 출발."""
    i = straight_start()
    start, yaw = pose_off(i, heading_err_deg=35.0)
    return start, yaw, [], "헤딩오차 +35deg 로 출발"


def scen_bad_start():
    """횡 0.6m + 헤딩 -40도. 실차 수동 배치 최악값."""
    i = straight_start()
    start, yaw = pose_off(i, lateral=-0.60, heading_err_deg=-40.0)
    return start, yaw, [], "횡 -0.6m + 헤딩 -40deg 로 출발"


def scen_two_laps():
    """장애물 없이 길게 — 다랩 추종오차 통계용."""
    x, y, yaw = pose_at(0)
    return (x, y), yaw, [], "2랩 클린 (추종오차 통계)"


def scen_corner_entry():
    """가장 급한 코너에 장애물 없이 진입 — 속도 프로파일/횡가속 확인."""
    i = index_back(sharpest_index(), 20.0)
    x, y, yaw = pose_at(i)
    return (x, y), yaw, [], "최급코너 진입 (무장애물)"


# ------------------------------------------------- 추가: 회피 경계조건
def scen_corridor_edge():
    """레이스라인에서 0.45m 옆 — 코리도 필터(0.40m) 바로 바깥.

    필터가 "내 진로 밖" 이라고 버리는데 반경 0.22m 라 실제로는 0.45-0.22
    =0.23m 까지 들어와 있다. 차 반폭 0.15m 면 여유가 0.08m 밖에 없다.
    필터 기준이 장애물 반경을 안 보는지 확인하는 시나리오.
    """
    i = straight_start()
    x, y, yaw = pose_at(i)
    ox, oy = ahead(i, 11.0, lateral=0.45)
    return (x, y), yaw, [Obstacle(ox, oy, 0.22, name="edge")], "코리도 경계 콘 (0.45m 옆)"


def scen_narrow_cone():
    """벽 여유가 가장 좁은 곳에 콘 — 피할 공간이 없는 구간."""
    j = narrowest_index()
    start = index_back(j, 14.0)
    x, y, yaw = pose_at(start)
    ox, oy = LINE[j][0], LINE[j][1]
    c = gmap().wall_clearance(ox, oy)
    return (
        (x, y),
        yaw,
        [Obstacle(ox, oy, 0.16, name="narrow")],
        f"최협구간 콘 (벽여유 {c:.2f}m)",
    )


def scen_three_cones():
    """4m 간격 3연속 지그재그 — 재합류가 끝나기 전에 다음 장애물."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    return (
        (x, y),
        yaw,
        [
            Obstacle(*ahead(i, 9.0, lateral=0.22), 0.16, name="c1"),
            Obstacle(*ahead(i, 13.0, lateral=-0.24), 0.16, name="c2"),
            Obstacle(*ahead(i, 17.0, lateral=0.22), 0.16, name="c3"),
        ],
        "4m 간격 3연속 지그재그",
    )


def scen_chicane_tight():
    """2.5m 간격 시케인 — 회피-재합류 사이클보다 짧다."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    return (
        (x, y),
        yaw,
        [
            Obstacle(*ahead(i, 10.0, lateral=0.26), 0.16, name="s1"),
            Obstacle(*ahead(i, 12.5, lateral=-0.26), 0.16, name="s2"),
        ],
        "2.5m 간격 타이트 시케인",
    )


def scen_corner_exit():
    """코너 정점 3m 뒤 — 탈출가속 구간에 콘."""
    j = index_ahead(sharpest_index(), 3.0)
    start = index_back(j, 18.0)
    x, y, yaw = pose_at(start)
    return (
        (x, y),
        yaw,
        [Obstacle(LINE[j][0], LINE[j][1], 0.16, name="exit")],
        "코너 탈출부 콘",
    )


# ------------------------------------------------- 추가: 동적
def _xy():
    return [(p[0], p[1]) for p in LINE]


def scen_lead_stops():
    """앞차가 트랙을 따라 가다가 급정지 — trailing 에서 정지까지."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    return (
        (x, y),
        yaw,
        [PathObstacle(_xy(), index_ahead(i, 9.0), 2.0, 0.20, "leadstop",
                      brake_after_s=4.0, brake_decel=4.0)],
        "앞차 트랙주행 2.0m/s → 4초에 급정지",
    )


def scen_lead_slow():
    """트랙을 따라 계속 느리게 가는 앞차 — trailing 유지 확인."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    return (
        (x, y),
        yaw,
        [PathObstacle(_xy(), index_ahead(i, 8.0), 1.2, 0.20, "leadslow")],
        "앞차 트랙주행 1.2m/s 지속 (trailing)",
    )


def scen_head_on():
    """트랙을 역주행해 마주 오는 차."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    return (
        (x, y),
        yaw,
        [PathObstacle(_xy(), index_ahead(i, 16.0), 1.5, 0.20, "oncoming",
                      reverse=True)],
        "역주행 정면 접근 (1.5 m/s)",
    )


def scen_dyn_cross_late():
    """기존 dyn_cross 보다 늦게·빠르게 끼어든다 (2.5 m/s)."""
    i = straight_start()
    x, y, yaw = pose_at(i)
    ox, oy = ahead(i, 12.0, lateral=3.0)
    return (
        (x, y),
        yaw,
        [Obstacle(ox, oy, 0.18, 0.0, -2.5, "fastcross")],
        "고속 측면 횡단 (2.5 m/s)",
    )


SCENARIOS = {
    "clean": scen_clean,
    "two_laps": scen_two_laps,
    "offset_start": scen_offset_start,
    "heading_err": scen_heading_err_start,
    "bad_start": scen_bad_start,
    "corner_entry": scen_corner_entry,
    "corridor_edge": scen_corridor_edge,
    "narrow_cone": scen_narrow_cone,
    "three_cones": scen_three_cones,
    "chicane": scen_chicane_tight,
    "corner_exit": scen_corner_exit,
    "lead_stops": scen_lead_stops,
    "lead_slow": scen_lead_slow,
    "head_on": scen_head_on,
    "dyn_cross_late": scen_dyn_cross_late,
    "cone": scen_cone_straight,
    "cone_offset": scen_cone_offset,
    "two_cones": scen_two_cones,
    "corner_cone": scen_corner_cone,
    "dyn_slow": scen_dynamic_slow,
    "dyn_cross": scen_dynamic_cross,
    "sudden": scen_sudden_wall,
    "blocked": scen_blocked,
}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "clean"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
    seed = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    start, yaw, obs, desc = SCENARIOS[name]()

    rclpy.init()
    sim = RaceSim(gmap(), start, yaw, obs, ref_line=LINE, seed=seed)
    for _ in range(5):
        sim.publish_map()
    import time

    time.sleep(2.0)  # 스택이 맵/TF 를 받을 시간

    # 시뮬이 내는 센서/상태 토픽을 남이 같이 발행하면 결과가 통째로 거짓이 된다.
    # 특히 /vehicle/speed_mps 는 Stanley 의 조향 분모라, 남이 0.0 을 섞으면
    # 무장애물 직선에서도 조향이 폭주한다. 발행자는 우리 자신 하나여야 한다.
    for topic in ("/vehicle/speed_mps", "/scan"):
        n_pub = sim.count_publishers(topic)
        if n_pub > 1:
            print(
                f"[중단] {topic} 발행자가 {n_pub}개입니다 (시뮬 자신 포함). "
                f"실차 노드가 떠 있으면 결과를 믿을 수 없습니다.\n"
                f"        ros2 topic info {topic} 로 확인 후 종료하세요."
            )
            sim.destroy_node()
            rclpy.shutdown()
            sys.exit(3)

    print(f"[{name}] {desc}")
    print(f"  시작 ({start[0]:.2f}, {start[1]:.2f}) yaw={math.degrees(yaw):.0f}deg")
    r = sim.run(dur)

    print(f"  주행 {r.distance_m:.1f} m, 최고 {r.max_speed:.2f} m/s")
    print(f"  최소 벽 여유 {r.min_wall_clear_m:.3f} m", end="")
    if r.min_obs_clear_m == r.min_obs_clear_m:
        print(f", 최소 장애물 여유 {r.min_obs_clear_m:.3f} m")
    else:
        print()
    g = r.cte_stats("global")
    if g["n"]:
        print(
            f"  추종오차(GLOBAL n={g['n']}): 평균 {g['mean']:.3f} m, "
            f"95% {g['p95']:.3f} m, 최대 {g['max']:.3f} m"
        )
    print(f"  AEB {r.aeb_events}회, 모드 {r.modes}")
    if r.collided:
        print(f"  ### 충돌: {r.collision_kind} @ {r.collision_pos}")
    elif r.stalled:
        print("  ### 정지(스톨)")
    else:
        print("  OK 완주")
    import json

    with open(f"/tmp/trace_{name}.json", "w") as fh:
        json.dump(
            {
                "trace": r.speed_trace,
                "obstacles": [[o.x, o.y, o.r, o.vx, o.vy] for o in obs],
                "collided": r.collided,
            },
            fh,
        )
    from render import render

    png = render(
        trace=r.speed_trace, obstacles=obs, out=f"/tmp/view_{name}.png"
    )
    print(f"  view: {png}")
    sim.destroy_node()
    rclpy.shutdown()
    sys.exit(2 if r.collided else 0)


if __name__ == "__main__":
    main()
