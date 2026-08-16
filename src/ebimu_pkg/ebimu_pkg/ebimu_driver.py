import glob
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import serial

from sensor_msgs.msg import Imu
from std_msgs.msg import Header

STANDARD_GRAVITY = 9.80665

# 칩 축(뒤에서 앞): +X 왼쪽, +Y 앞, +Z 아래.
#
# 원래 의도는 +X 오른쪽 / +Y 뒤 / +Z 아래였다. 그런데 IMU 를 물리적으로 다시
# 달면서 수평면에서 180° 돌아간 상태가 됐다 (+X 가 왼쪽으로). Z 는 그대로
# 아래다 — 정지 상태 가속도 실측 (-0.007, -0.052, -0.989) g 에서 az ≈ -1 g 이
# 나오는 건 +Z 가 중력 방향(아래)일 때뿐이다. 오른손계를 유지하려면
# Y = Z × X = 아래 × 왼쪽 = 앞 이므로, 결국 Z축 기준 180° 회전이다.
#
# 이걸 반영하지 않으면 yaw 가 정확히 180° 뒤집힌 값으로 나간다 (실측:
# 같은 자세에서 128.27° vs -51.73°). Cartographer 가 그걸 그대로 믿으므로
# 맵이 뒤집히고 /scan 이 occupancy grid 에 안 붙는다.
#
# ROS imu_link: +X 앞, +Y 왼쪽, +Z 위.
# v_ros = M v_chip, M = M^{-1} (180° 회전이라 스스로가 역행렬)
_ROS_FROM_CHIP_Q = (
    math.sqrt(0.5),  # x
    math.sqrt(0.5),  # y   ← 예전 값 -sqrt(0.5) (+X 오른쪽 가정)
    0.0,  # z
    0.0,  # w
)


def _chip_vec_to_ros(x: float, y: float, z: float) -> tuple[float, float, float]:
    # 앞 = +Y_chip, 왼쪽 = +X_chip, 위 = -Z_chip.  예전: (-y, -x, -z)
    return (y, x, -z)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def gravity_from_orientation(roll_deg: float, pitch_deg: float) -> tuple[float, float, float]:
    """orientation-only 스트림용 중력 벡터 (칩 축, 비력 규약).

    가속도계는 비력(specific force)을 재므로 정지 시 -g 를 읽는다.
    칩 자세 R(roll, pitch, yaw) 에 대해

        a_chip = Rᵀ (0, 0, +g) = g·(−sin p, cos p·sin r, cos p·cos r)

    az 부호가 반대로 들어가 있었다. 실측(정지)이 az ≈ −0.99 g 인데 예전 식은
    +0.99 g 를 내고, 그게 _chip_vec_to_ros 를 통과하면서 ROS 에서 −1 g 가 됐다
    — 정지 상태에서 위쪽으로 1 g 가 아니라 아래쪽으로 1 g 를 보고하는 셈이다.
    9필드 스트림에서는 이 함수가 안 쓰이므로 지금까지 드러나지 않았다.
    """
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    ax = -math.sin(pitch) * STANDARD_GRAVITY
    ay = math.sin(roll) * math.cos(pitch) * STANDARD_GRAVITY
    az = math.cos(roll) * math.cos(pitch) * STANDARD_GRAVITY  # 예전: 부호 반대
    return ax, ay, az


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return (qx, qy, qz, qw)


class EbimuDriver(Node):

    def __init__(self):
        super().__init__('ebimu_driver')

        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("accel_in_g", True)
        # EBIMU 실측 평균 주기(초). 한 타이머 틱에 프레임이 여러 개 몰려 들어왔을 때
        # 각 프레임에 이 간격만큼 역산한 timestamp를 매겨 burst를 보정하는 데 씀.
        self.declare_parameter("imu_sample_period_s", 0.01)
        # Cartographer tracking_frame Z는 위여야 함. 칩 Z가 아래라 그대로 내면
        # 맵이 기존 맵 대비 뒤집히고 /scan 이 occupancy grid에 안 붙음.
        self.declare_parameter("publish_ros_axes", True)

        requested_port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self.accel_in_g = bool(self.get_parameter("accel_in_g").value)
        self.nominal_period_s = float(self.get_parameter("imu_sample_period_s").value)
        # 마지막으로 발행한 stamp. 배치 역산이 이 값보다 과거로 내려가지 않게
        # 붙잡는 데 쓴다. Cartographer 의 imu_tracker 는 IMU stamp 가 한 번만
        # 뒤로 가도 CHECK 실패로 즉시 abort 한다.
        self._last_stamp_ns: int | None = None
        # PLL 상태: stamp 를 벽시계에 매번 앵커링하지 않고 자체 시계로 생성한다.
        self._next_stamp_ns: int | None = None
        self._period_ns: int = max(1, int(
            float(self.get_parameter("imu_sample_period_s").value) * 1e9))
        self._sync_t0_ns: int = 0
        self._sync_count: int = 0
        self.publish_ros_axes = bool(self.get_parameter("publish_ros_axes").value)
        port = self.resolve_port(requested_port)
        self.seen_full_imu_frame = False
        self.serial_buffer = ""
        self.bad_frame_count = 0
        self.seen_orientation_only_frame = False
        self.last_orientation_time = None
        self.last_roll = None
        self.last_pitch = None
        self.last_yaw = None

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as exc:
            self.log_serial_open_error(port, exc)
            raise

        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.timer = self.create_timer(0.01, self.read_serial)

        axes = (
            "ROS(+X fwd, +Y left, +Z up)"
            if self.publish_ros_axes
            else "chip(+X left, +Y fwd, +Z down)"
        )
        self.get_logger().info(
            f"EBIMU Driver Started on {port} @ {baud} baud, imu_link axes={axes}"
        )

    def resolve_port(self, requested_port: str) -> str:
        if requested_port and requested_port != "auto":
            return requested_port

        candidates = []
        for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyTHS*"):
            candidates.extend(sorted(glob.glob(pattern)))

        if not candidates:
            message = (
                "No serial port found. Set the port manually, for example "
                "'ros2 launch ebimu_pkg ebimu.launch.py port:=/dev/ttyUSB0'."
            )
            self.get_logger().error(message)
            raise RuntimeError(message)

        selected_port = candidates[0]
        self.get_logger().info(
            f"Auto-detected serial port {selected_port}. "
            f"Candidates: {', '.join(candidates)}"
        )
        return selected_port

    def log_serial_open_error(self, port: str, exc: Exception) -> None:
        error_text = str(exc)

        if isinstance(exc, serial.SerialException) and "Permission denied" in error_text:
            self.get_logger().error(
                f"Permission denied opening {port}. "
                "Run: 'sudo usermod -aG dialout $USER', "
                f"'sudo chmod 666 {port}', "
                "and if using Jetson UART also run "
                "'sudo systemctl stop nvgetty && sudo systemctl disable nvgetty'."
            )
            return

        if isinstance(exc, serial.SerialException) and (
            "No such file" in error_text or "could not open port" in error_text
        ):
            self.get_logger().error(
                f"Could not open serial port {port}. "
                "Check the EBIMU wiring and confirm the correct tty device."
            )
            return

        self.get_logger().error(f"Failed to open serial port {port}: {error_text}")

    def read_serial(self):
        if self.ser.in_waiting == 0:
            return

        chunk = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="ignore")
        if not chunk:
            return

        self.serial_buffer += chunk

        # Keep only the newest tail if we lost synchronization for too long.
        if len(self.serial_buffer) > 4096:
            last_star = self.serial_buffer.rfind("*")
            self.serial_buffer = self.serial_buffer[last_star:] if last_star != -1 else ""

        if "*" not in self.serial_buffer:
            self.process_line_frames()
            return

        first_star = self.serial_buffer.find("*")
        if first_star > 0:
            self.serial_buffer = self.serial_buffer[first_star:]

        frames = self.serial_buffer.split("*")
        complete_frames = frames[1:-1]
        tail_frame = frames[-1]

        # 한 틱에 여러 프레임이 밀려 들어와도 하나도 안 버리고 전부 publish하되,
        # 실제 샘플링 간격만큼 역산한 timestamp를 매겨 burst를 보정한다.
        self._process_frame_batch(complete_frames)

        if self.serial_buffer.endswith("\n") or self.serial_buffer.endswith("\r"):
            self._process_frame_batch([tail_frame])
            self.serial_buffer = ""
        else:
            self.serial_buffer = "*" + tail_frame

    def process_line_frames(self):
        lines = self.serial_buffer.splitlines(keepends=True)

        complete_lines = []
        self.serial_buffer = ""

        for line in lines:
            if line.endswith("\n") or line.endswith("\r"):
                complete_lines.append(line)
            else:
                self.serial_buffer = line

        # "*" 구분자 없는 스트림도 동일하게 batch 보정 적용.
        self._process_frame_batch(complete_lines)

    def _process_frame_batch(self, frames: list[str]) -> None:
        """한 틱에 몰려 들어온 프레임들에 위상동기(PLL) timestamp를 매겨 전부 publish.

        예전 방식은 배치마다 벽시계 now 에 다시 앵커링하고 거기서 역산했다.
        그래서 시리얼 버퍼링/스케줄러 지터가 그대로 stamp 에 실렸고, 역행 방지에
        걸리면 프레임 간격이 주기(10ms)가 아니라 step(1ms)으로 찍혀서
        간격 표준편차가 평균보다 커졌다(cartographer 로그: 1.05e-2 s +/- 1.85e-2 s).
        회전 중에는 이 시각 오차가 곧바로 각도 오차가 된다(70dps에서 18ms = 1.3deg).

        지금은 stamp 를 자체 시계로 만든다:
          stamp(k) = anchor + k * period
        - period 는 실측에서 학습한다(파라미터 0.01s = 100Hz 였지만 실측 95.2Hz).
        - 벽시계와의 위상차는 매 배치 아주 약하게(1/8) 슬루해서 따라간다.
          점프가 아니라 슬루라서 간격이 흔들리지 않는다.
        - 큰 정체(0.5s 이상) 뒤에는 하드 리싱크한다.
        """
        if not frames:
            return

        now_ns = self.get_clock().now().nanoseconds
        n = len(frames)

        if self._next_stamp_ns is None:      # 최초 1회
            self._period_ns = max(1, int(self.nominal_period_s * 1e9))
            self._next_stamp_ns = now_ns - (n - 1) * self._period_ns
            self._sync_t0_ns = self._next_stamp_ns
            self._sync_count = 0

        # 마지막 프레임이 있어야 할 자리와 실제 now 의 차이 = 위상 오차
        predicted_last = self._next_stamp_ns + (n - 1) * self._period_ns
        err_ns = now_ns - predicted_last

        if abs(err_ns) > 500_000_000:        # 0.5s 이상 어긋나면 스트림이 끊겼던 것
            self.get_logger().warn(
                f'IMU stamp 재동기화 (위상차 {err_ns * 1e-6:.0f} ms)')
            self._next_stamp_ns = now_ns - (n - 1) * self._period_ns
            self._sync_t0_ns = self._next_stamp_ns
            self._sync_count = 0
            err_ns = 0

        for frame in frames:
            stamp_ns = self._next_stamp_ns
            # 안전망: 어떤 경로로도 stamp 가 뒤로 가면 cartographer 의
            # imu_tracker.cc 가 CHECK 로 즉시 abort 한다.
            if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
                stamp_ns = self._last_stamp_ns + 1
            self._last_stamp_ns = stamp_ns
            self._next_stamp_ns = stamp_ns + self._period_ns
            self._sync_count += 1
            self.process_frame(frame, stamp=Time(nanoseconds=stamp_ns))

        # 위상 슬루: 주기의 1/4 이내로 제한하고 그중 1/8 만 반영.
        # 점프가 아니라 서서히 당겨야 간격 지터가 안 생긴다.
        cap = self._period_ns // 4
        self._next_stamp_ns += max(-cap, min(cap, err_ns)) // 8

        # 주기 학습: 충분히 쌓인 뒤 장기 평균으로 갱신(파라미터 값보다 실측 우선).
        if self._sync_count >= 200:
            measured = (now_ns - self._sync_t0_ns) // self._sync_count
            if 1_000_000 <= measured <= 100_000_000:      # 10Hz~1000Hz 사이만
                self._period_ns = (self._period_ns * 7 + measured) // 8
            if self._sync_count >= 2000:                  # 창을 굴려 최신성 유지
                self._sync_t0_ns = self._last_stamp_ns
                self._sync_count = 0

    def process_frame(self, frame: str, stamp=None):
        cleaned_line = frame.strip()

        if not cleaned_line:
            return

        try:
            data = [value.strip() for value in cleaned_line.split(",") if value.strip()]

            if len(data) < 3:
                return

            roll = float(data[0])
            pitch = float(data[1])
            yaw = float(data[2])

            if len(data) >= 9:
                gx = float(data[3])
                gy = float(data[4])
                gz = float(data[5])
                ax = float(data[6])
                ay = float(data[7])
                az = float(data[8])
                has_gyro_accel = True
            else:
                gx = gy = gz = 0.0
                ax = ay = az = 0.0
                has_gyro_accel = False

            if not self.seen_full_imu_frame:
                if has_gyro_accel:
                    self.seen_full_imu_frame = True
                    self.get_logger().info(
                        "Full IMU stream detected: orientation + gyro + accel"
                    )
                elif not self.seen_orientation_only_frame:
                    self.seen_orientation_only_frame = True
                    self.get_logger().info(
                        "Orientation-only IMU stream detected"
                    )

            self.publish_imu(roll, pitch, yaw, gx, gy, gz, ax, ay, az, has_gyro_accel, stamp=stamp)
        except Exception as exc:
            self.bad_frame_count += 1
            if self.bad_frame_count % 20 == 1:
                self.get_logger().warn(
                    f"Dropped malformed IMU frame #{self.bad_frame_count}: "
                    f"'{cleaned_line[:120]}' ({exc})"
                )

    def publish_imu(self, roll, pitch, yaw, gx, gy, gz, ax, ay, az, has_gyro_accel, stamp=None):
        imu_msg = Imu()
        imu_msg.header = Header()
        now = stamp if stamp is not None else self.get_clock().now()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = "imu_link"

        q = quaternion_from_euler(
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw),
        )
        if self.publish_ros_axes:
            q = _quat_mul(q, _ROS_FROM_CHIP_Q)

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        if has_gyro_accel:
            wx, wy, wz = math.radians(gx), math.radians(gy), math.radians(gz)
            if self.accel_in_g:
                ax *= STANDARD_GRAVITY
                ay *= STANDARD_GRAVITY
                az *= STANDARD_GRAVITY
        else:
            ax, ay, az = gravity_from_orientation(roll, pitch)
            wx = wy = wz = 0.0
            if self.last_orientation_time is not None:
                dt = (now - self.last_orientation_time).nanoseconds * 1e-9
                if dt > 1e-4 and self.last_roll is not None:
                    wx = math.radians(roll - self.last_roll) / dt
                    wy = math.radians(pitch - self.last_pitch) / dt
                    wz = math.radians(yaw - self.last_yaw) / dt
            self.last_orientation_time = now
            self.last_roll = roll
            self.last_pitch = pitch
            self.last_yaw = yaw

        if self.publish_ros_axes:
            wx, wy, wz = _chip_vec_to_ros(wx, wy, wz)
            ax, ay, az = _chip_vec_to_ros(ax, ay, az)

        imu_msg.angular_velocity.x = wx
        imu_msg.angular_velocity.y = wy
        imu_msg.angular_velocity.z = wz
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EbimuDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
