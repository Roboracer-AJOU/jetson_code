"""실제 맵 기반 폐루프 시뮬레이터.

path_following 스택(integrated_obstacle / fgm / local_planner / emergency_brake /
stanley) 을 그대로 띄운 채 차량 + LiDAR + 장애물을 흉내낸다.
control_node 는 시리얼이 필요해 못 띄우므로 여기서 그 역할까지 대신한다.

  map PNG ─┐
           ├─ raycast ─> /scan ─> [스택] ─> /drive ─> 자전거모델 ─> pose ─> TF
  장애물 ──┘                                  /emergency_brake ─> 역토크
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String
from tf2_ros import TransformBroadcaster

# ---------------------------------------------------------------- 차량 파라미터
WHEELBASE_M = 0.33
MAX_STEER_RAD = 0.6981
STEER_RATE_RADPS = 6.0
# 조향 실효율. 순수 기구학이면 1.0 이지만 실차는 서보 유격·링키지 손실·언더스티어
# 때문에 명령한 각도만큼 안 돈다. stanley 의 곡률 피드포워드가
# ff_gain=2.3 (= 기구학 정답의 2.3배) 로 튜닝돼 있다는 건, 실차 응답이 기구학의
# 약 1/2.3 이라는 뜻이다. 시뮬을 그 실차에 맞춰 놓지 않으면 코너마다 2.3배
# 과조향이 나서 있지도 않은 버그를 쫓게 된다.
STEER_EFFECTIVENESS = 1.0 / 2.3
A_ACCEL = 3.0          # 구동 가속 한계 [m/s^2]
A_BRAKE = 4.0          # 일반 감속 한계
A_AEB = 6.0            # 역토크 제동 (control_node emergency_brake_duty 상당)
CAR_HALF_WIDTH = 0.15
CAR_FRONT_M = 0.33
CAR_REAR_M = 0.12
LASER_OFFSET_X = 0.275

# ---------------------------------------------------------------- LiDAR
SCAN_FOV_DEG = 270.0
SCAN_BEAMS = 541
SCAN_MAX_M = 12.0
SCAN_MIN_M = 0.12
SCAN_HZ = 20.0
SCAN_NOISE_M = 0.01

# ---------------------------------------------------------------- 맵
# 시뮬 전체가 쓰는 단일 출처. render.py / check_map.py / run_scenario.py 가
# 각자 문자열을 들고 있다가 맵이 지워지면 셋 다 따로 깨진다.
# raceline.csv 는 이 맵에서 뽑은 것이라 둘은 반드시 같이 움직인다.
MAP_YAML = (
    "/home/nvidia/f1tenth_ajou/maps/cartographer_map_20260814_232850_rosmap.yaml"
)


@dataclass
class Obstacle:
    """원형 장애물. vx/vy 가 0 이 아니면 동적."""

    x: float
    y: float
    r: float = 0.15
    vx: float = 0.0
    vy: float = 0.0
    name: str = "obs"
    # t 초 뒤부터 decel [m/s^2] 로 감속해 정지. 앞차가 갑자기 서는 상황용.
    brake_after_s: float = -1.0
    brake_decel: float = 3.0

    def __post_init__(self) -> None:
        self._t = 0.0

    def step(self, dt: float) -> None:
        self._t += dt
        if self.brake_after_s >= 0.0 and self._t >= self.brake_after_s:
            sp = math.hypot(self.vx, self.vy)
            if sp > 1e-6:
                new = max(0.0, sp - self.brake_decel * dt)
                k = new / sp
                self.vx *= k
                self.vy *= k
        self.x += self.vx * dt
        self.y += self.vy * dt


class PathObstacle(Obstacle):
    """레이스라인을 따라 달리는 장애물 (앞차/역주행차).

    Obstacle 의 등속 직진 모델은 곡선 트랙에서 몇 초 만에 코스를 벗어난다.
    실제로 lead/oncoming 시나리오를 돌려 보면 장애물이 레이스라인에서
    3~30m 떨어진 곳으로 날아가 버려서, 자차는 아무것도 안 만나고 완주한다.
    "통과" 가 나오지만 아무것도 검증하지 않은 것이다. 앞차는 트랙을 따라
    달려야 앞차다.
    """

    def __init__(self, path, s_index: int, speed: float, r: float = 0.20,
                 name: str = "pathobs", reverse: bool = False,
                 brake_after_s: float = -1.0, brake_decel: float = 3.0):
        self._path = [(float(a), float(b)) for a, b in path]
        self._i = int(s_index) % len(self._path)
        self._speed = float(speed)
        self._rev = bool(reverse)
        x, y = self._path[self._i]
        super().__init__(x, y, r, 0.0, 0.0, name,
                         brake_after_s=brake_after_s, brake_decel=brake_decel)

    def step(self, dt: float) -> None:
        self._t += dt
        if self.brake_after_s >= 0.0 and self._t >= self.brake_after_s:
            self._speed = max(0.0, self._speed - self.brake_decel * dt)

        n = len(self._path)
        remain = self._speed * dt
        while remain > 1e-9:
            j = (self._i - 1) % n if self._rev else (self._i + 1) % n
            ax, ay = self._path[self._i]
            bx, by = self._path[j]
            seg = math.hypot(bx - ax, by - ay)
            if seg <= 1e-9:
                self._i = j
                continue
            if remain < seg:
                k = remain / seg
                self.x = ax + (bx - ax) * k
                self.y = ay + (by - ay) * k
                # 진행 방향 속도 — 플래너의 상대속도 계산이 이걸 본다
                self.vx = (bx - ax) / seg * self._speed
                self.vy = (by - ay) / seg * self._speed
                return
            remain -= seg
            self._i = j
            self.x, self.y = bx, by
        self.vx = self.vy = 0.0


@dataclass
class SimResult:
    collided: bool = False
    collision_kind: str = ""
    collision_pos: tuple = (0.0, 0.0)
    min_wall_clear_m: float = 9e9
    min_obs_clear_m: float = 9e9
    max_speed: float = 0.0
    distance_m: float = 0.0
    elapsed_s: float = 0.0
    stalled: bool = False
    aeb_events: int = 0
    modes: dict = field(default_factory=dict)
    speed_trace: list = field(default_factory=list)
    # 경로추종 오차. 회피 중에는 일부러 라인을 벗어나므로 GLOBAL 모드일 때만
    # 모은 값을 따로 둔다 — 이게 "추종이 정상인가" 의 판정 대상이다.
    cte_all: list = field(default_factory=list)
    cte_global: list = field(default_factory=list)

    def cte_stats(self, which: str = "global") -> dict:
        import numpy as _np

        a = _np.asarray(self.cte_global if which == "global" else self.cte_all)
        if a.size == 0:
            return {"n": 0, "mean": float("nan"), "p95": float("nan"),
                    "max": float("nan")}
        return {
            "n": int(a.size),
            "mean": float(a.mean()),
            "p95": float(_np.percentile(a, 95)),
            "max": float(a.max()),
        }


class GridMap:
    def __init__(self, yaml_path: str):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img = Image.open(meta["image"]).convert("L")
        a = np.array(img)
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        # ROS map convention: 밝음=free. 이미지 위쪽이 y 최대라 뒤집는다.
        occ = np.flipud(a)
        self.free = occ > 250          # 확실한 free
        self.wall = occ < 100          # 확실한 occupied
        self.h, self.w = occ.shape
        self._occ_u8 = occ

    def world_to_idx(self, x, y):
        cx = ((np.asarray(x) - self.ox) / self.res).astype(np.int32)
        cy = ((np.asarray(y) - self.oy) / self.res).astype(np.int32)
        return cx, cy

    def is_wall(self, x, y) -> bool:
        cx, cy = self.world_to_idx(x, y)
        if cx < 0 or cy < 0 or cx >= self.w or cy >= self.h:
            return True
        return bool(self.wall[cy, cx])

    def to_occupancy_grid(self, frame="map") -> OccupancyGrid:
        g = OccupancyGrid()
        g.header.frame_id = frame
        g.info.resolution = self.res
        g.info.width = self.w
        g.info.height = self.h
        g.info.origin.position.x = self.ox
        g.info.origin.position.y = self.oy
        g.info.origin.orientation.w = 1.0
        data = np.full((self.h, self.w), -1, dtype=np.int8)
        data[self.free] = 0
        data[self.wall] = 100
        g.data = data.ravel().tolist()
        return g

    def wall_clearance(self, x: float, y: float, cap: float = 3.0) -> float:
        """가장 가까운 벽까지 거리 (근사, cap 까지만)."""
        if not hasattr(self, "_dt"):
            from scipy.ndimage import distance_transform_edt

            self._dt = distance_transform_edt(~self.wall) * self.res
        cx, cy = self.world_to_idx(x, y)
        if cx < 0 or cy < 0 or cx >= self.w or cy >= self.h:
            return 0.0
        return float(min(cap, self._dt[cy, cx]))


class RayCaster:
    """맵 + 원형 장애물에 대한 벡터화 레이캐스트."""

    def __init__(self, gmap: GridMap, seed: int = 0):
        # 시드를 고정한다. 예전엔 np.random 전역을 써서 매 실행 스캔 노이즈가
        # 달라졌고, 같은 시나리오가 완주와 충돌 사이를 오갔다. 그러면 A/B 를
        # 못 한다. 노이즈 민감도를 보고 싶으면 시드를 바꿔 여러 번 돌린다.
        self._rng = np.random.default_rng(seed)
        self.m = gmap
        self.step = gmap.res * 0.5
        self.n_steps = int(SCAN_MAX_M / self.step)
        self.t = np.arange(1, self.n_steps + 1, dtype=np.float32) * self.step
        self.angles = np.linspace(
            -math.radians(SCAN_FOV_DEG) / 2.0,
            math.radians(SCAN_FOV_DEG) / 2.0,
            SCAN_BEAMS,
        ).astype(np.float32)

    def scan(self, lx: float, ly: float, yaw: float, obstacles) -> np.ndarray:
        ang = self.angles + yaw
        ca, sa = np.cos(ang)[:, None], np.sin(ang)[:, None]
        xs = lx + ca * self.t[None, :]
        ys = ly + sa * self.t[None, :]
        cx = ((xs - self.m.ox) / self.m.res).astype(np.int32)
        cy = ((ys - self.m.oy) / self.m.res).astype(np.int32)
        oob = (cx < 0) | (cy < 0) | (cx >= self.m.w) | (cy >= self.m.h)
        cx = np.clip(cx, 0, self.m.w - 1)
        cy = np.clip(cy, 0, self.m.h - 1)
        hit = self.m.wall[cy, cx] | oob
        first = np.argmax(hit, axis=1)
        any_hit = hit.any(axis=1)
        rng = np.where(any_hit, self.t[np.clip(first, 0, self.n_steps - 1)], np.inf)

        # 장애물은 해석적으로 (ray-circle)
        for ob in obstacles:
            dx, dy = ob.x - lx, ob.y - ly
            proj = dx * np.cos(ang) + dy * np.sin(ang)
            perp2 = dx * dx + dy * dy - proj * proj
            r2 = ob.r * ob.r
            ok = (proj > 0) & (perp2 < r2)
            d = np.where(ok, proj - np.sqrt(np.maximum(r2 - perp2, 0.0)), np.inf)
            rng = np.minimum(rng, d)

        rng = np.where(np.isfinite(rng), rng, SCAN_MAX_M + 1.0)
        rng += self._rng.normal(0.0, SCAN_NOISE_M, rng.shape)
        return np.clip(rng, SCAN_MIN_M, SCAN_MAX_M + 1.0).astype(np.float32)


class RaceSim(Node):
    def __init__(self, gmap: GridMap, start_xy, start_yaw, obstacles=None,
                 ref_line=None, seed: int = 0):
        super().__init__("race_sim")
        self.m = gmap
        self.rc = RayCaster(gmap, seed=seed)
        self.obstacles = list(obstacles or [])
        # 추종오차 기준선 (x, y[, v]) 리스트. 없으면 CTE 를 안 잰다.
        self._ref = (
            np.asarray([[p[0], p[1]] for p in ref_line], dtype=np.float64)
            if ref_line
            else None
        )

        self.x, self.y = start_xy
        self.yaw = start_yaw
        self.v = 0.0
        self.steer = 0.0

        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.last_cmd_t = 0.0
        self.aeb = False
        self.aeb_seen = False
        self.mode = "?"

        self.res = SimResult()
        self._aeb_prev = False

        self.tfb = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.spd_pub = self.create_publisher(Float64, "/vehicle/speed_mps", 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self._grid = gmap.to_occupancy_grid()

        self.create_subscription(
            AckermannDriveStamped, "/drive", self._drive_cb, 10
        )
        self.create_subscription(Bool, "/emergency_brake", self._aeb_cb, 10)
        self.create_subscription(String, "/planner/mode", self._mode_cb, 10)

    # ------------------------------------------------------------ callbacks
    def _drive_cb(self, msg: AckermannDriveStamped) -> None:
        self.cmd_speed = float(msg.drive.speed)
        self.cmd_steer = float(msg.drive.steering_angle)
        self.last_cmd_t = time.time()

    def _aeb_cb(self, msg: Bool) -> None:
        self.aeb = bool(msg.data)
        self.aeb_seen = True

    def _mode_cb(self, msg: String) -> None:
        self.mode = msg.data
        self.res.modes[msg.data] = self.res.modes.get(msg.data, 0) + 1

    # ------------------------------------------------------------ publishing
    def publish_map(self) -> None:
        self._grid.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self._grid)

    def _publish_tf(self) -> None:
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "laser"
        t2.transform.translation.x = LASER_OFFSET_X
        t2.transform.rotation.w = 1.0
        self.tfb.sendTransform([t, t2])

    def _publish_scan(self) -> None:
        lx = self.x + LASER_OFFSET_X * math.cos(self.yaw)
        ly = self.y + LASER_OFFSET_X * math.sin(self.yaw)
        rng = self.rc.scan(lx, ly, self.yaw, self.obstacles)
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = "laser"
        s.angle_min = float(self.rc.angles[0])
        s.angle_max = float(self.rc.angles[-1])
        s.angle_increment = float(self.rc.angles[1] - self.rc.angles[0])
        s.range_min = SCAN_MIN_M
        s.range_max = SCAN_MAX_M
        s.ranges = rng.tolist()
        self.scan_pub.publish(s)

    # ------------------------------------------------------------ plant
    def step_vehicle(self, dt: float) -> None:
        """control_node 역할(AEB 우선) + 자전거 모델."""
        now = time.time()
        stale = (now - self.last_cmd_t) > 0.25 if self.last_cmd_t > 0 else True

        if self.aeb:
            # control_node: AEB 중에도 _auto_steer (=/drive 조향) 는 계속 반영된다.
            # duty 만 역토크로 바뀐다.
            target = 0.0
            decel = A_AEB
            steer_t = self.cmd_steer
        elif stale:
            target = 0.0
            decel = A_BRAKE
            steer_t = 0.0
        else:
            target = max(0.0, self.cmd_speed)
            decel = A_BRAKE
            steer_t = self.cmd_steer

        if target > self.v:
            self.v = min(target, self.v + A_ACCEL * dt)
        else:
            self.v = max(target, self.v - decel * dt)
        self.v = max(0.0, self.v)

        steer_t = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, steer_t))
        d = steer_t - self.steer
        lim = STEER_RATE_RADPS * dt
        self.steer += max(-lim, min(lim, d))

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        eff = math.tan(self.steer) * STEER_EFFECTIVENESS
        self.yaw += self.v / WHEELBASE_M * eff * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        for ob in self.obstacles:
            ob.step(dt)

        self.res.distance_m += self.v * dt
        self.res.max_speed = max(self.res.max_speed, self.v)
        self.res.speed_trace.append(
            (
                round(self.x, 2),
                round(self.y, 2),
                round(math.degrees(self.yaw)),
                round(self.v, 2),
                round(math.degrees(self.steer)),
                round(self.cmd_speed, 2),
                int(self.aeb),
            )
        )
        if self.aeb and not self._aeb_prev:
            self.res.aeb_events += 1
        self._aeb_prev = self.aeb

        if self._ref is not None:
            e = self._cte()
            self.res.cte_all.append(e)
            # 회피/재합류 중 라인 이탈은 오차가 아니라 의도된 동작이다.
            if self.mode in ("GLOBAL", "?"):
                self.res.cte_global.append(e)

    def _cte(self) -> float:
        """레퍼런스 폐폴리라인까지 최단거리 [m] (세그먼트 투영)."""
        p = self._ref
        ax, ay = p[:, 0], p[:, 1]
        bx, by = np.roll(ax, -1), np.roll(ay, -1)
        abx, aby = bx - ax, by - ay
        ab2 = abx * abx + aby * aby
        t = np.divide(
            (self.x - ax) * abx + (self.y - ay) * aby,
            ab2,
            out=np.zeros_like(ab2),
            where=ab2 >= 1e-14,
        )
        np.clip(t, 0.0, 1.0, out=t)
        dx = self.x - (ax + t * abx)
        dy = self.y - (ay + t * aby)
        return float(np.sqrt((dx * dx + dy * dy).min()))

    def check_collision(self) -> bool:
        """차량 외곽 코너로 벽/장애물 충돌 확인."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        pts = []
        for fx in (CAR_FRONT_M, CAR_FRONT_M * 0.5, 0.0, -CAR_REAR_M):
            for fy in (-CAR_HALF_WIDTH, 0.0, CAR_HALF_WIDTH):
                pts.append((self.x + fx * c - fy * s, self.y + fx * s + fy * c))
        for px, py in pts:
            if self.m.is_wall(px, py):
                self.res.collided = True
                self.res.collision_kind = "wall"
                self.res.collision_pos = (round(px, 2), round(py, 2))
                return True
            for ob in self.obstacles:
                if math.hypot(px - ob.x, py - ob.y) < ob.r:
                    self.res.collided = True
                    self.res.collision_kind = f"obstacle:{ob.name}"
                    self.res.collision_pos = (round(px, 2), round(py, 2))
                    return True

        self.res.min_wall_clear_m = min(
            self.res.min_wall_clear_m, self.m.wall_clearance(self.x, self.y)
        )
        for ob in self.obstacles:
            d = math.hypot(self.x - ob.x, self.y - ob.y) - ob.r
            self.res.min_obs_clear_m = min(self.res.min_obs_clear_m, d)
        return False

    # ------------------------------------------------------------ main loop
    def run(self, duration_s: float, realtime_factor: float = 1.0) -> SimResult:
        dt = 1.0 / SCAN_HZ
        n = int(duration_s * SCAN_HZ)
        stall_ticks = 0
        t0 = time.time()
        for _ in range(n):
            self._publish_tf()
            self._publish_scan()
            self.spd_pub.publish(Float64(data=float(self.v)))
            # 스택이 반응할 시간을 준다
            end = time.time() + dt / max(realtime_factor, 1e-3)
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.001)
            self.step_vehicle(dt)
            if self.check_collision():
                break
            if self.v < 0.05:
                stall_ticks += 1
                if stall_ticks > SCAN_HZ * 6:
                    self.res.stalled = True
                    break
            else:
                stall_ticks = 0
        self.res.elapsed_s = time.time() - t0
        if self.res.min_obs_clear_m > 8e9:
            self.res.min_obs_clear_m = float("nan")
        return self.res
