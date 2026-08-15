"""/static_obstacles vs /dynamic_obstacles 분류가 얼마나 안정적인지 측정.

시나리오를 돌리는 동안 같이 띄워서, 같은 물체가 static/dynamic 사이를
몇 번 오가는지, dynamic 으로 볼 때 트랙방향 속도(vs)가 얼마나 나오는지 본다.
"정적 콘이 앞차로 보인다" 는 추측을 숫자로 확인하기 위한 것.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("probe_classify")
        self.create_subscription(
            Float32MultiArray, "/static_obstacles", self._static_cb, 10
        )
        self.create_subscription(
            Float32MultiArray, "/dynamic_obstacles", self._dyn_cb, 10
        )
        self.n_static_msg = 0
        self.n_dyn_msg = 0
        self.static_frames = 0   # 정적 장애물이 1개 이상인 프레임
        self.dyn_frames = 0
        self.dyn_speeds: list[float] = []
        self.create_timer(1.0, self._report)

    def _static_cb(self, msg: Float32MultiArray) -> None:
        self.n_static_msg += 1
        if len(msg.data) >= 4:
            self.static_frames += 1

    def _dyn_cb(self, msg: Float32MultiArray) -> None:
        self.n_dyn_msg += 1
        d = msg.data
        if len(d) >= 6:
            self.dyn_frames += 1
            for k in range(0, len(d) - 5, 6):
                vx, vy = float(d[k + 3]), float(d[k + 4])
                self.dyn_speeds.append((vx * vx + vy * vy) ** 0.5)

    def _report(self) -> None:
        sp = self.dyn_speeds[-40:]
        avg = sum(sp) / len(sp) if sp else 0.0
        self.get_logger().info(
            f"static msg={self.n_static_msg} (물체있음 {self.static_frames}) | "
            f"dynamic msg={self.n_dyn_msg} (물체있음 {self.dyn_frames}) | "
            f"dyn 상대속력 최근평균={avg:.2f} m/s"
        )


def main() -> None:
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(
            f"\n[요약] static 물체프레임={node.static_frames} "
            f"dynamic 물체프레임={node.dyn_frames}",
            file=sys.stderr,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
