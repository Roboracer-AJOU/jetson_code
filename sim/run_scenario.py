"""시나리오 러너. 스택은 밖에서 띄워두고 이 스크립트가 차량/센서를 돌린다.

  python3 sim/run_scenario.py <scenario> [duration_s]
"""
from __future__ import annotations

import math
import sys

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/sim")

import rclpy
from race_sim import GridMap, Obstacle, RaceSim

sys.path.insert(0, "/home/nvidia/f1tenth_ajou/src/path_following")
from path_following import track_sliding

MAP = "/home/nvidia/f1tenth_ajou/maps/cartographer_map_20260814_232850_rosmap.yaml"
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
    n = len(LINE)
    acc = 0.0
    j = i
    while acc < dist_m:
        a, b = LINE[j % n], LINE[(j + 1) % n]
        acc += math.hypot(b[0] - a[0], b[1] - a[1])
        j += 1
    x, y, _ = LINE[j % n]
    x2, y2, _ = LINE[(j + 3) % n]
    yaw = math.atan2(y2 - y, x2 - x)
    return x - lateral * math.sin(yaw), y + lateral * math.cos(yaw)


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
    """곡률이 가장 큰 지점 부근에 콘."""
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
    start = (best_i - 130) % n
    x, y, yaw = pose_at(start)
    ox, oy = LINE[best_i][0], LINE[best_i][1]
    return (x, y), yaw, [Obstacle(ox, oy, 0.16, name="corner")], "코너 정점 콘"


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


SCENARIOS = {
    "clean": scen_clean,
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
    start, yaw, obs, desc = SCENARIOS[name]()

    rclpy.init()
    gmap = GridMap(MAP)
    sim = RaceSim(gmap, start, yaw, obs)
    for _ in range(5):
        sim.publish_map()
    import time

    time.sleep(2.0)  # 스택이 맵/TF 를 받을 시간

    print(f"[{name}] {desc}")
    print(f"  시작 ({start[0]:.2f}, {start[1]:.2f}) yaw={math.degrees(yaw):.0f}deg")
    r = sim.run(dur)

    print(f"  주행 {r.distance_m:.1f} m, 최고 {r.max_speed:.2f} m/s")
    print(f"  최소 벽 여유 {r.min_wall_clear_m:.3f} m", end="")
    if r.min_obs_clear_m == r.min_obs_clear_m:
        print(f", 최소 장애물 여유 {r.min_obs_clear_m:.3f} m")
    else:
        print()
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
