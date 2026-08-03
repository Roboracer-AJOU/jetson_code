import glob
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import serial

from sensor_msgs.msg import Imu
from std_msgs.msg import Header

STANDARD_GRAVITY = 9.80665


def gravity_from_orientation(roll_deg: float, pitch_deg: float) -> tuple[float, float, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    ax = -math.sin(pitch) * STANDARD_GRAVITY
    ay = math.sin(roll) * math.cos(pitch) * STANDARD_GRAVITY
    az = -math.cos(roll) * math.cos(pitch) * STANDARD_GRAVITY
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

        requested_port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self.accel_in_g = bool(self.get_parameter("accel_in_g").value)
        self.nominal_period_s = float(self.get_parameter("imu_sample_period_s").value)
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

        self.get_logger().info(f"EBIMU Driver Started on {port} @ {baud} baud")

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
        """한 틱에 몰려 들어온 프레임들에 균등 간격 timestamp를 매겨 전부 publish.

        프레임을 버리지 않고, 가장 오래된 프레임일수록 과거 시각을,
        가장 최신 프레임은 지금 시각을 갖도록 역산한다.
        """
        if not frames:
            return

        now = self.get_clock().now()
        n = len(frames)
        for i, frame in enumerate(frames):
            offset = Duration(seconds=self.nominal_period_s * (n - 1 - i))
            self.process_frame(frame, stamp=now - offset)

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

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        if has_gyro_accel:
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)

            if self.accel_in_g:
                ax *= STANDARD_GRAVITY
                ay *= STANDARD_GRAVITY
                az *= STANDARD_GRAVITY

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
        else:
            ax, ay, az = gravity_from_orientation(roll, pitch)
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            if self.last_orientation_time is not None:
                dt = (now - self.last_orientation_time).nanoseconds * 1e-9
                if dt > 1e-4 and self.last_roll is not None:
                    imu_msg.angular_velocity.x = math.radians(roll - self.last_roll) / dt
                    imu_msg.angular_velocity.y = math.radians(pitch - self.last_pitch) / dt
                    imu_msg.angular_velocity.z = math.radians(yaw - self.last_yaw) / dt

            self.last_orientation_time = now
            self.last_roll = roll
            self.last_pitch = pitch
            self.last_yaw = yaw

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
