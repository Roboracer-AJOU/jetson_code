#!/usr/bin/env python3
"""Latest-value CSV telemetry logger for the F1TENTH vehicle."""

from __future__ import annotations

import csv
import math
import os
import time
from datetime import datetime
from typing import Dict, Iterable, List

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


CSV_COLUMNS = [
    "time_sec",
    "ros_time_sec",
    "data_valid",
    "vesc_valid",
    "vehicle_valid",
    "rc_valid",
    "safety_valid",
    "imu_valid",
    "vesc_age_ms",
    "vehicle_age_ms",
    "rc_age_ms",
    "safety_age_ms",
    "imu_age_ms",
    "mode",
    "estop_active",
    "manual_override",
    "timeout_active",
    "rc_ch1",
    "rc_ch2",
    "rc_ch5",
    "rc_ch6",
    "target_duty",
    "limited_duty",
    "final_duty",
    "vesc_duty",
    "target_steering",
    "final_steering",
    "speed_mps",
    "speed_kph",
    "accel_mps2",
    "erpm",
    "motor_rpm",
    "wheel_rpm",
    "battery_voltage",
    "voltage_drop",
    "input_current",
    "motor_current",
    "vesc_temp",
    "motor_temp",
    "fault_code",
    "duty_limit_active",
    "limit_reason",
    "cross_track_error_m",
    "heading_error_rad",
    "heading_term_rad",
    "cross_track_term_rad",
    "stanley_steering_sum_rad",
    "raw_steering_cmd_rad",
    "filtered_or_limited_steering_cmd_rad",
    "stanley_speed_mps",
    "closest_path_index",
    "path_curvature_1pm",
    "steering_ff_rad",
    "steering_before_saturation_rad",
    "active_path_lateral_error_m",
    "active_path_x_m",
    "active_path_y_m",
    "csv_path_lateral_error_m",
    "vehicle_x_m",
    "vehicle_y_m",
    "vehicle_yaw_rad",
    "imu_stamp_sec",
    "yaw_rate_radps",
    "imu_accel_x_mps2",
    "lateral_accel_mps2",
    "imu_accel_z_mps2",
]

STANLEY_COLUMNS = (
    "cross_track_error_m",
    "heading_error_rad",
    "heading_term_rad",
    "cross_track_term_rad",
    "stanley_steering_sum_rad",
    "raw_steering_cmd_rad",
    "filtered_or_limited_steering_cmd_rad",
    "stanley_speed_mps",
    "closest_path_index",
    "path_curvature_1pm",
    "steering_ff_rad",
    "steering_before_saturation_rad",
    "active_path_lateral_error_m",
    "active_path_x_m",
    "active_path_y_m",
    "csv_path_lateral_error_m",
    "vehicle_x_m",
    "vehicle_y_m",
    "vehicle_yaw_rad",
)

TOPIC_KEYS = ("vesc", "vehicle", "rc", "safety", "imu")
LIMIT_REASONS = {
    0: "NONE",
    1: "ESTOP",
    2: "TIMEOUT",
    3: "MANUAL_OVERRIDE",
    4: "DUTY_LIMIT",
}


class CsvLoggerNode(Node):
    """Cache newest messages and write one row at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("csv_logger_node")

        self.declare_parameter("log_hz", 20.0)
        self.declare_parameter("output_root", "~/f1tenth_ajou/logs")
        self.declare_parameter("flush_every_rows", 20)
        self.declare_parameter("valid_timeout_sec", 0.5)
        self.declare_parameter("waiting_log_period_sec", 5.0)
        self.declare_parameter("pole_pairs", 2.0)
        self.declare_parameter("gear_ratio", 12.0)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("nominal_voltage", 14.4)
        self.declare_parameter("subscribe_legacy_telemetry", True)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_sample_hz", 20.0)

        self._log_hz = max(0.1, float(self.get_parameter("log_hz").value))
        self._flush_every_rows = max(
            1, int(self.get_parameter("flush_every_rows").value)
        )
        self._valid_timeout_sec = max(
            0.0, float(self.get_parameter("valid_timeout_sec").value)
        )
        self._waiting_log_period_sec = max(
            0.5, float(self.get_parameter("waiting_log_period_sec").value)
        )
        self._pole_pairs = max(
            1e-6, float(self.get_parameter("pole_pairs").value)
        )
        self._gear_ratio = max(
            1e-6, float(self.get_parameter("gear_ratio").value)
        )
        self._wheel_radius = max(
            1e-6, float(self.get_parameter("wheel_radius").value)
        )
        self._nominal_voltage = float(
            self.get_parameter("nominal_voltage").value
        )
        self._imu_sample_hz = max(
            0.1, float(self.get_parameter("imu_sample_hz").value)
        )
        self._imu_sample_period = 1.0 / self._imu_sample_hz
        self._last_imu_sample_monotonic: float | None = None

        self._latest: Dict[str, object] = {
            column: math.nan for column in CSV_COLUMNS
        }
        self._latest.update(
            {
                "mode": "UNKNOWN",
                "estop_active": False,
                "manual_override": False,
                "timeout_active": False,
                "duty_limit_active": False,
                "limit_reason": "NONE",
                "fault_code": 0,
                "rc_ch6": -1,
            }
        )
        self._received_monotonic: Dict[str, float | None] = {
            key: None for key in TOPIC_KEYS
        }
        self._start_monotonic = time.monotonic()
        self._last_waiting_log = 0.0
        self._row_count = 0
        self._last_vehicle_speed = None
        self._last_vehicle_stamp = None
        # Replaced as one object by the callback; the writer never sees a
        # partially updated Stanley control-cycle snapshot.
        self._latest_stanley: Dict[str, float] | None = None

        output_root = os.path.abspath(
            os.path.expanduser(str(self.get_parameter("output_root").value))
        )
        session_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._session_dir = os.path.join(output_root, session_name)
        os.makedirs(self._session_dir, exist_ok=False)
        self._csv_path = os.path.join(self._session_dir, "telemetry.csv")
        self._metadata_path = os.path.join(self._session_dir, "metadata.yaml")
        self._csv_file = open(
            self._csv_path, "w", newline="", encoding="utf-8-sig"
        )
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._csv_file.flush()
        self._write_metadata()

        self.create_subscription(
            Float64MultiArray, "/vesc/status", self._on_vesc_status, 10
        )
        self.create_subscription(
            Float64MultiArray, "/vehicle/state", self._on_vehicle_state, 10
        )
        self.create_subscription(
            Float64MultiArray, "/rc/state", self._on_rc_state, 10
        )
        self.create_subscription(
            Float64MultiArray, "/control/state", self._on_control_state, 10
        )
        self.create_subscription(
            Float64MultiArray, "/safety/state", self._on_safety_state, 10
        )
        self.create_subscription(
            Float64MultiArray, "/stanley/debug", self._on_stanley_debug, 10
        )
        imu_topic = str(self.get_parameter("imu_topic").value)
        self.create_subscription(Imu, imu_topic, self._on_imu, 20)
        if bool(self.get_parameter("subscribe_legacy_telemetry").value):
            self.create_subscription(
                Float64MultiArray,
                "/vehicle/telemetry",
                self._on_legacy_telemetry,
                10,
            )
            self.create_subscription(
                AckermannDriveStamped, "/drive", self._on_drive, 10
            )

        self.create_timer(1.0 / self._log_hz, self._write_row)
        self.get_logger().info(
            f"CSV logger started: {self._csv_path} ({self._log_hz:.1f} Hz)"
        )

    @staticmethod
    def _value(data: Iterable[float], index: int, default=None):
        values = data if isinstance(data, (list, tuple)) else list(data)
        if index >= len(values):
            return default
        value = float(values[index])
        return value if math.isfinite(value) else default

    def _update(self, **values: object) -> None:
        for key, value in values.items():
            if key in self._latest and value is not None:
                self._latest[key] = value

    def _mark_received(self, topic_key: str) -> None:
        self._received_monotonic[topic_key] = time.monotonic()

    def _on_vesc_status(self, msg: Float64MultiArray) -> None:
        # [stamp, voltage, input_current, motor_current, duty, erpm,
        #  vesc_temp, motor_temp, fault_code]
        d = msg.data
        self._mark_received("vesc")
        voltage = self._value(d, 1)
        self._update(
            battery_voltage=voltage,
            voltage_drop=(
                self._nominal_voltage - voltage
                if voltage is not None
                else None
            ),
            input_current=self._value(d, 2),
            motor_current=self._value(d, 3),
            vesc_duty=self._value(d, 4),
            erpm=self._value(d, 5),
            vesc_temp=self._value(d, 6),
            motor_temp=self._value(d, 7),
            fault_code=self._value(d, 8),
        )

    def _on_vehicle_state(self, msg: Float64MultiArray) -> None:
        # [stamp, speed_mps, speed_kph, accel, erpm, motor_rpm,
        #  wheel_rpm, battery_voltage, voltage_drop]
        d = msg.data
        self._mark_received("vehicle")
        stamp = self._value(d, 0)
        speed = self._value(d, 1)
        accel = self._value(d, 3)
        if accel is None and speed is not None and stamp is not None:
            if (
                self._last_vehicle_speed is not None
                and self._last_vehicle_stamp is not None
            ):
                dt = stamp - self._last_vehicle_stamp
                if dt > 1e-6:
                    accel = (speed - self._last_vehicle_speed) / dt
            self._last_vehicle_speed = speed
            self._last_vehicle_stamp = stamp
        self._update(
            speed_mps=speed,
            speed_kph=self._value(
                d, 2, speed * 3.6 if speed is not None else None
            ),
            accel_mps2=accel,
            erpm=self._value(d, 4),
            motor_rpm=self._value(d, 5),
            wheel_rpm=self._value(d, 6),
            battery_voltage=self._value(d, 7),
            voltage_drop=self._value(d, 8),
        )

    def _on_rc_state(self, msg: Float64MultiArray) -> None:
        # [stamp, ch1, ch2, ch5, ch6, manual_active, auto_active,
        #  estop_active]
        d = msg.data
        self._mark_received("rc")
        manual = bool(self._value(d, 5, 0.0))
        auto = bool(self._value(d, 6, 0.0))
        self._update(
            rc_ch1=self._value(d, 1),
            rc_ch2=self._value(d, 2),
            rc_ch5=self._value(d, 3),
            rc_ch6=self._value(d, 4),
            manual_override=manual,
            mode="AUTO" if auto else ("MANUAL" if manual else "UNKNOWN"),
            estop_active=bool(self._value(d, 7, 0.0)),
        )

    def _on_control_state(self, msg: Float64MultiArray) -> None:
        # [stamp, target_duty, target_steering]
        self._update(
            target_duty=self._value(msg.data, 1),
            target_steering=self._value(msg.data, 2),
        )

    def _on_safety_state(self, msg: Float64MultiArray) -> None:
        # [stamp, estop, manual_override, timeout, duty_limit, input_duty,
        #  limited_duty, final_duty, final_steering, limit_reason_code]
        d = msg.data
        self._mark_received("safety")
        reason = self._value(d, 9)
        self._update(
            estop_active=bool(self._value(d, 1, 0.0)),
            manual_override=bool(self._value(d, 2, 0.0)),
            timeout_active=bool(self._value(d, 3, 0.0)),
            duty_limit_active=bool(self._value(d, 4, 0.0)),
            target_duty=self._value(d, 5),
            limited_duty=self._value(d, 6),
            final_duty=self._value(d, 7),
            final_steering=self._value(d, 8),
            limit_reason=(
                LIMIT_REASONS.get(int(reason), f"CODE_{int(reason)}")
                if reason is not None
                else "NONE"
            ),
        )

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        # Legacy command fallback. Speed is not duty, so do not mix them.
        self._update(target_steering=float(msg.drive.steering_angle))

    def _on_stanley_debug(self, msg: Float64MultiArray) -> None:
        """Atomically cache one /stanley/debug control-cycle snapshot.

        The publisher provides 19 coherent values; see STANLEY_COLUMNS.
        """
        if len(msg.data) < len(STANLEY_COLUMNS):
            self.get_logger().warning(
                "Ignoring /stanley/debug with "
                f"{len(msg.data)} values; expected {len(STANLEY_COLUMNS)}"
            )
            return
        values = [
            float(value) for value in msg.data[: len(STANLEY_COLUMNS)]
        ]
        if not all(math.isfinite(value) for value in values):
            return
        self._latest_stanley = dict(zip(STANLEY_COLUMNS, values))

    def _on_imu(self, msg: Imu) -> None:
        """Cache ROS-frame IMU data (+X forward, +Y left, +Z up)."""
        now_monotonic = time.monotonic()
        if (
            self._last_imu_sample_monotonic is not None
            and now_monotonic - self._last_imu_sample_monotonic
            < self._imu_sample_period
        ):
            return

        values = (
            float(msg.angular_velocity.z),
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )
        if not all(math.isfinite(value) for value in values):
            return
        self._last_imu_sample_monotonic = now_monotonic
        self._mark_received("imu")
        stamp_sec = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        self._update(
            imu_stamp_sec=stamp_sec,
            yaw_rate_radps=values[0],
            imu_accel_x_mps2=values[1],
            lateral_accel_mps2=values[2],
            imu_accel_z_mps2=values[3],
        )

    def _on_legacy_telemetry(self, msg: Float64MultiArray) -> None:
        """Adapt control_node's existing /vehicle/telemetry array."""
        d: List[float] = list(msg.data)
        auto = bool(self._value(d, 6, 0.0))
        estop = bool(self._value(d, 7, 0.0))
        speed = self._value(d, 10)
        erpm = self._value(d, 15)
        voltage = self._value(d, 18)
        motor_rpm = erpm / self._pole_pairs if erpm is not None else None
        wheel_rpm = (
            motor_rpm / self._gear_ratio
            if motor_rpm is not None
            else None
        )
        self._update(
            mode="AUTO" if auto else "MANUAL",
            estop_active=estop,
            manual_override=not auto,
            rc_ch1=self._value(d, 8),
            rc_ch2=self._value(d, 9),
            rc_ch5=self._value(d, 5),
            target_duty=self._value(d, 14),
            limited_duty=self._value(d, 2),
            final_duty=self._value(d, 2),
            vesc_duty=self._value(d, 2),
            target_steering=self._value(d, 1),
            final_steering=self._value(d, 4),
            speed_mps=speed,
            speed_kph=speed * 3.6 if speed is not None else None,
            erpm=erpm,
            motor_rpm=motor_rpm,
            wheel_rpm=wheel_rpm,
            battery_voltage=voltage,
            voltage_drop=(
                self._nominal_voltage - voltage
                if voltage is not None
                else None
            ),
            input_current=self._value(d, 16),
            motor_current=self._value(d, 17),
        )

    def _write_row(self) -> None:
        now_monotonic = time.monotonic()
        required_received = (
            self._received_monotonic["vesc"] is not None
            and self._received_monotonic["vehicle"] is not None
        )
        if not required_received:
            waiting_log_due = (
                now_monotonic - self._last_waiting_log
                >= self._waiting_log_period_sec
            )
            if waiting_log_due:
                self._last_waiting_log = now_monotonic
                missing = [
                    f"/{key}/status" if key == "vesc" else f"/{key}/state"
                    for key in ("vesc", "vehicle")
                    if self._received_monotonic[key] is None
                ]
                message = "waiting for required telemetry topics: "
                self.get_logger().info(message + ", ".join(missing))
            return

        row = dict(self._latest)
        stanley = self._latest_stanley
        if stanley is not None:
            row.update(stanley)
        row["time_sec"] = now_monotonic - self._start_monotonic
        row["ros_time_sec"] = self.get_clock().now().nanoseconds * 1e-9
        for key in TOPIC_KEYS:
            received_at = self._received_monotonic[key]
            if received_at is None:
                age_ms = math.nan
                valid = False
            else:
                age_ms = max(0.0, (now_monotonic - received_at) * 1000.0)
                valid = age_ms <= self._valid_timeout_sec * 1000.0
            row[f"{key}_age_ms"] = age_ms
            row[f"{key}_valid"] = valid
        row["data_valid"] = bool(row["vesc_valid"] and row["vehicle_valid"])
        try:
            self._writer.writerow(row)
            self._row_count += 1
            if self._row_count % self._flush_every_rows == 0:
                self._csv_file.flush()
        except Exception as exc:
            self.get_logger().error(f"CSV write failed: {exc}")

    def _write_metadata(self) -> None:
        lines = [
            f'created_at: "{datetime.now().astimezone().isoformat()}"',
            f'csv_file: "{os.path.basename(self._csv_path)}"',
            f"log_hz: {self._log_hz}",
            f"valid_timeout_sec: {self._valid_timeout_sec}",
            f"imu_sample_hz: {self._imu_sample_hz}",
            "latest_value_logging: true",
            "subscribe_only: true",
            "vehicle_parameters:",
            f"  pole_pairs: {self._pole_pairs}",
            f"  gear_ratio: {self._gear_ratio}",
            f"  wheel_radius: {self._wheel_radius}",
            f"  nominal_voltage: {self._nominal_voltage}",
            "topics:",
            "  - /vesc/status",
            "  - /vehicle/state",
            "  - /rc/state",
            "  - /control/state",
            "  - /safety/state",
            "  - /stanley/debug       # 19-value coherent control snapshot",
            f"  - {self.get_parameter('imu_topic').value}",
            "  - /vehicle/telemetry  # legacy fallback",
            "  - /drive              # legacy fallback",
        ]
        with open(self._metadata_path, "w", encoding="utf-8") as metadata_file:
            metadata_file.write("\n".join(lines) + "\n")

    def destroy_node(self) -> None:
        if hasattr(self, "_csv_file") and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CsvLoggerNode()
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
