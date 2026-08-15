#!/usr/bin/env python3
"""AEB 탈출 모드 라이브 검증.

local_planner_node 를 실제로 띄우고, 정적 장애물이 CSV 정면을 막은 상태에서
AEB 상승/하강엣지를 넣어 본다. 확인하는 것:

  1. 제동 중(고속)에는 경로를 바꾸지 않는다
  2. 멈춘 뒤 mode=AVOID + /local_path 발행 + override=True
  3. 속도가 aeb_escape_speed_mps 로 묶인다
  4. AEB 해제 후에도 hold 동안 유지되고, 그 뒤 정상 판정으로 복귀

    python3 tmp_escape_live.py
"""
from __future__ import annotations

import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Float32MultiArray, Float64, String
from tf2_ros import TransformBroadcaster

from path_following.local_planner_node import LocalPlannerNode

CSV = "/home/nvidia/f1tenth_ajou/src/path_following/config/raceline.csv"
S_EGO = 12.0          # 자차를 놓을 트랙 위치
OBS_AHEAD_M = 0.9     # 정면 장애물까지 (레이저 기준)

FAILS: list[str] = []


def check(name, ok, info=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}  {info}", flush=True)
    if not ok:
        FAILS.append(name)


class Rig:
    """플래너에 필요한 입력을 전부 흉내 내는 보조 노드."""

    def __init__(self, planner: LocalPlannerNode):
        self.n = rclpy.create_node("escape_rig")
        self.p = planner
        self.tfb = TransformBroadcaster(self.n)
        self.pub_obs = self.n.create_publisher(
            Float32MultiArray, "/static_obstacles", 10
        )
        self.pub_speed = self.n.create_publisher(Float64, "/vehicle/speed_mps", 10)
        self.pub_aeb = self.n.create_publisher(Bool, "/emergency_brake", 10)
        self.pub_fgm = self.n.create_publisher(PointStamped, "/fgm_target", 10)

        self.mode = ""
        self.override = None
        self.scale = None
        self.path_len = 0
        self.n.create_subscription(String, "/planner/mode", self._cb_mode, 10)
        self.n.create_subscription(
            Bool, "/planner_path_override_active", self._cb_ovr, 10
        )
        self.n.create_subscription(
            Float64, "/planner/speed_scale", self._cb_scale, 10
        )
        self.n.create_subscription(Path, "/local_path", self._cb_path, 10)

        self.ego_speed = 0.0
        self.aeb = False
        self.obs = [0.0, OBS_AHEAD_M, 0.0, 0.18]
        self.fgm_xy = (1.0, 0.9)   # 왼쪽으로 열린 틈
        x, y, yaw = planner._xy_yaw_at_s(S_EGO)
        self.ex, self.ey, self.eyaw = x, y, yaw

    def _cb_mode(self, m):
        self.mode = m.data

    def _cb_ovr(self, m):
        self.override = bool(m.data)

    def _cb_scale(self, m):
        self.scale = float(m.data)

    def _cb_path(self, m):
        self.path_len = len(m.poses)

    def _tf(self, child):
        t = TransformStamped()
        t.header.stamp = self.n.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = child
        t.transform.translation.x = self.ex
        t.transform.translation.y = self.ey
        t.transform.rotation.z = math.sin(self.eyaw / 2.0)
        t.transform.rotation.w = math.cos(self.eyaw / 2.0)
        return t

    def tick(self):
        """한 프레임 분의 입력을 밀어 넣는다."""
        self.tfb.sendTransform([self._tf("base_link"), self._tf("laser")])
        # 장애물 (laser frame). [id, x, y, r, ...]
        self.pub_obs.publish(Float32MultiArray(data=list(self.obs)))
        self.pub_speed.publish(Float64(data=self.ego_speed))
        self.pub_aeb.publish(Bool(data=self.aeb))
        # FGM 은 안 띄우므로 목표점을 직접 준다
        pt = PointStamped()
        pt.header.frame_id = "laser"
        pt.header.stamp = self.n.get_clock().now().to_msg()
        pt.point.x, pt.point.y = self.fgm_xy
        self.pub_fgm.publish(pt)


def main():
    rclpy.init(
        args=[
            "--ros-args",
            "-p", f"csv_path:={CSV}",
            # 맵이 없으니 벽 검사는 자동으로 빠지고 장애물 원판 검사만 돈다.
            # 이걸 켜 둬야 "경로가 막혀서 기각" 경로를 실제로 밟는다.
            "-p", "path_check_enable:=true",
            "-p", "verbose_logs:=true",
            "-p", "obstacle_tf_timeout_sec:=0.02",
        ]
    )
    planner = LocalPlannerNode()
    rig = Rig(planner)

    ex = SingleThreadedExecutor()
    ex.add_node(planner)
    ex.add_node(rig.n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def run(sec: float, label=""):
        """sec 동안 40 Hz 로 입력을 밀어 넣는다 (ROS clock 대신 time.sleep)."""
        end = time.monotonic() + sec
        seen = []
        while time.monotonic() < end:
            rig.tick()
            if rig.mode:
                seen.append(rig.mode)
            time.sleep(0.025)
        if label:
            uniq = sorted(set(seen))
            print(
                f"    ({label}: modes={uniq}, override={rig.override}, "
                f"scale={rig.scale}, path_poses={rig.path_len})",
                flush=True,
            )
        return seen

    # ---------- 1. 평상시: 장애물 정면 → AVOID 로 회피 ----------
    rig.ego_speed = 2.0
    run(1.0, "정상 접근")
    check("정면 장애물이면 AVOID 로 회피 시도", rig.mode in ("AVOID", "GLOBAL"),
          f"mode={rig.mode}")
    base_scale = rig.scale

    # ---------- 2. AEB 가 고속 제동 중 ----------
    rig.aeb = True
    rig.ego_speed = 2.0
    run(0.6)
    check("제동 중(고속)에는 탈출 개입 없음",
          not planner._aeb_escape_active(),
          f"v={rig.ego_speed}, arm={planner.aeb_escape_arm_speed}")

    # ---------- 3. 멈췄다 → 탈출 모드 ----------
    rig.ego_speed = 0.0
    rig.path_len = 0
    run(0.8, "정지 후 탈출")
    check("멈춘 뒤 탈출 모드 진입", planner._aeb_escape_active())
    check("탈출 중 mode=AVOID 발행", rig.mode == "AVOID", f"mode={rig.mode}")
    check("탈출 경로 /local_path 발행", rig.path_len >= 2,
          f"poses={rig.path_len}")
    check("override=True (Stanley 가 탈출 경로를 탄다)", rig.override is True,
          f"override={rig.override}")

    v_csv = planner._csv_speed_now()
    v_cmd = (rig.scale or 0.0) * v_csv
    check("탈출 속도 상한 준수",
          v_cmd <= planner.aeb_escape_speed_mps + 1e-3,
          f"{v_cmd:.2f} <= {planner.aeb_escape_speed_mps}m/s (csv={v_csv:.2f})")
    check("탈출 속도가 0 이 아님", v_cmd > 0.05, f"{v_cmd:.2f}m/s")

    # ---------- 4. AEB 해제 → hold 동안 유지 ----------
    rig.aeb = False
    rig.ego_speed = 0.5
    run(0.5, "해제 직후")
    check("AEB 해제 후에도 hold 동안 탈출 유지",
          planner._aeb_escape_active() and rig.mode == "AVOID",
          f"mode={rig.mode}")

    # ---------- 5. hold 만료 → 정상 판정 복귀 ----------
    run(2.0, "hold 만료 후")
    check("hold 만료 후 탈출 종료", not planner._aeb_escape_active())

    # ---------- 6. 장애물 치우면 GLOBAL 복귀 ----------
    rig.ego_speed = 2.0
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        rig.tfb.sendTransform([rig._tf("base_link"), rig._tf("laser")])
        rig.pub_obs.publish(Float32MultiArray(data=[]))
        rig.pub_speed.publish(Float64(data=rig.ego_speed))
        rig.pub_aeb.publish(Bool(data=False))
        time.sleep(0.025)
    check("장애물 사라지면 GLOBAL/REJOIN 복귀",
          rig.mode in ("GLOBAL", "REJOIN"), f"mode={rig.mode}")
    check("복귀 후 override 해제", rig.override is False,
          f"override={rig.override}")
    check("복귀 후 속도 배율 회복", (rig.scale or 0.0) > 0.9,
          f"scale={rig.scale}")

    # ---------- 7. 회피 경로가 실제로 막힌 경우 (문제 1·2 의 핵심) ----------
    # FGM 목표를 정면으로 줘서 경로가 장애물 원판을 관통하게 만든다.
    # → _truncate_path_at_collision 기각 → 막힘 래치 → 따라갈 앞차 없으니 GLOBAL
    rig.obs = [0.0, OBS_AHEAD_M, 0.0, 0.18]
    rig.fgm_xy = (1.2, 0.0)
    rig.ego_speed = 1.0
    rig.aeb = False
    seq = run(3.0, "경로 막힘")
    sw = sum(1 for i in range(1, len(seq)) if seq[i] != seq[i - 1])
    check("경로 막히면 override 안 냄", rig.override is False,
          f"override={rig.override}")
    check("모드 떨림 억제 (3초, 40Hz)", sw <= 14, f"{sw}회 전이")
    check("따라갈 앞차 없으면 GLOBAL 쪽으로 수렴 (AEB 완화 안 함)",
          seq.count("GLOBAL") > len(seq) * 0.5,
          f"GLOBAL {seq.count('GLOBAL')}/{len(seq)} 프레임")
    check("막혀 있어도 감속은 걸린다", (rig.scale or 1.0) < 0.9,
          f"scale={rig.scale}")

    # 여기서 AEB 가 걸리고 멈췄다 → 탈출이 AVOID 를 강제해야 한다
    rig.aeb = True
    rig.ego_speed = 0.0
    run(0.8, "막힌 채 AEB")
    check("막힌 상태에서도 탈출이 AVOID 를 강제", rig.mode == "AVOID",
          f"mode={rig.mode}")

    # FGM 이 옆으로 열린 틈을 찍으면 그때 탈출 경로가 나온다
    rig.path_len = 0
    rig.fgm_xy = (1.0, 0.9)
    run(1.0, "탈출 방향 확보")
    check("탈출 방향이 열리면 경로 발행", rig.path_len >= 2,
          f"poses={rig.path_len}")
    check("탈출 경로로 override", rig.override is True,
          f"override={rig.override}")

    print(flush=True)
    if FAILS:
        print(f"FAIL {len(FAILS)}건: {FAILS}", flush=True)
        code = 1
    else:
        print("ALL PASS", flush=True)
        code = 0
    # executor 스레드가 물고 있으면 정상 종료가 걸리는 경우가 있다.
    # 검증 결과는 이미 다 찍었으므로 그냥 끊는다.
    os._exit(code)


if __name__ == "__main__":
    main()
