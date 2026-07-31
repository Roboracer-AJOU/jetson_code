#!/usr/bin/env python3
"""
No-load VESC speed PI test utility.

This file does not include steering, RC, CH5, or MANUAL mode logic.
It independently verifies the AUTO speed feedback control from the current
control_node.py. After validation, the relevant final logic is intended to be
merged back into control_node.py.
"""
from __future__ import annotations

import csv
import math
import select
import struct
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


# ============================================================
# MERGE_START: AUTO SPEED FEEDBACK CONTROL
# ============================================================
CFG = {
    "vesc_port": "/dev/ttyACM0",
    "vesc_baud": 115200,
    "timer_period_sec": 0.02,
    "serial_open_delay_sec": 2.0,
    "debug_log_hz": 1.0,
    "test_enabled": True,
    "target_speed_mps": 8.0,
    "min_move_duty": 0.06,
    "min_move_speed_mps": 0.08,
    "max_auto_duty": 0.70,
    "max_target_speed_mps": 10.0,
    "auto_duty_output_sign": 1.0,
    "speed_ff_duty_per_mps": 0.076,
    "speed_kp": 0.15,
    "speed_ki": 0.03,
    "i_enable_error_mps": 0.5,
    "target_decrease_i_scale": 0.5,
    "target_change_threshold_mps": 0.1,
    "duty_rate_limit_per_sec": 0.60,
    "vesc_telemetry_timeout_sec": 0.3,
    "vesc_poll_period_sec": 0.05,
    "invert_speed_sign": False,
    "pole_pairs": 2,
    "gear_ratio": 12.0,
    "wheel_diameter": 0.10,
}


class VescSpeedPITestNode(Node):
    def __init__(self) -> None:
        super().__init__("vesc_speed_pi_test_node")

        self._debug_log_hz = max(0.0, float(CFG["debug_log_hz"]))
        self._min_move_duty = max(0.0, float(CFG["min_move_duty"]))
        self._min_move_speed_mps = max(0.0, float(CFG["min_move_speed_mps"]))
        self._max_auto_duty = max(0.0, float(CFG.get("max_auto_duty", 0.30)))
        self._max_auto_brake_duty = self.clamp(
            max(0.0, float(CFG.get("max_auto_brake_duty", self._max_auto_duty))),
            0.0,
            self._max_auto_duty,
        )
        self._max_target_speed_mps = max(0.0, float(CFG.get("max_target_speed_mps", 3.0)))
        self._test_enabled = bool(CFG["test_enabled"])
        self._configured_target_speed_mps = self.clamp(
            float(CFG["target_speed_mps"]),
            0.0,
            self._max_target_speed_mps,
        )
        self._auto_duty_output_sign = -1.0 if float(
            CFG.get("auto_duty_output_sign", -1.0)
        ) < 0.0 else 1.0
        self._speed_ff_duty_per_mps = float(CFG["speed_ff_duty_per_mps"])
        self._speed_kp = float(CFG.get("speed_kp", 0.04))
        self._speed_ki = float(CFG.get("speed_ki", 0.015))
        self._i_enable_error_mps = max(0.0, float(CFG["i_enable_error_mps"]))
        self._target_decrease_i_scale = self.clamp(
            float(CFG["target_decrease_i_scale"]), 0.0, 1.0
        )
        self._target_change_threshold_mps = max(
            0.0, float(CFG["target_change_threshold_mps"])
        )
        self._duty_rate_limit_per_sec = max(
            0.0, float(CFG.get("duty_rate_limit_per_sec", 0.15))
        )
        self._vesc_telemetry_timeout = max(
            0.0, float(CFG.get("vesc_telemetry_timeout_sec", 0.3))
        )
        self._vesc_poll_period = max(0.0, float(CFG.get("vesc_poll_period_sec", 0.05)))
        self._invert_speed_sign = bool(CFG.get("invert_speed_sign", True))
        self._pole_pairs = max(1e-6, float(CFG.get("pole_pairs", 2)))
        self._gear_ratio = max(1e-6, float(CFG.get("gear_ratio", 12.0)))
        self._wheel_diameter = max(1e-6, float(CFG.get("wheel_diameter", 0.10)))

        self._estop_lock = threading.Lock()
        self._estop_latched = False
        self._keyboard_running = False
        self._keyboard_thread: threading.Thread | None = None
        self._stdin_termios_old = None

        self._last_timer_time = time.time()
        self._target_speed_mps = 0.0
        self._previous_target_speed_mps = 0.0
        self._previous_target_speed_for_log = 0.0
        self._speed_error = 0.0
        self._speed_ff_term = 0.0
        self._speed_integral_duty = 0.0
        self._speed_p_term = 0.0
        self._speed_i_term = 0.0
        self._raw_speed_duty_cmd = 0.0
        self._limited_speed_duty_cmd = 0.0
        self._speed_duty_cmd = 0.0
        self._auto_duty_limiter_active = False
        self._i_integration_active = False
        self._target_decrease_detected = False
        self._last_auto_duty_cmd = 0.0
        self.current_duty = 0.0
        self._vesc_duty_now = 0.0
        self._last_duty_int = None
        self._last_duty_packet = None
        self._last_debug_log_time = 0.0
        self._last_vesc_poll_time = 0.0
        self._last_vesc_telemetry_time = 0.0
        self._vesc_rx_buffer = bytearray()
        self._erpm = 0.0
        self._measured_speed_mps = 0.0
        self._current_motor = 0.0
        self._current_in = 0.0
        self._input_voltage = 0.0

        self._csv_file = None
        self._csv_writer = None
        self._last_csv_flush_time = 0.0
        self._start_time = time.time()
        self._open_csv_log()

        self.get_logger().info(f"Opening VESC serial: {CFG['vesc_port']}")
        self.vesc = serial.Serial(
            str(CFG["vesc_port"]), int(CFG["vesc_baud"]), timeout=0.0
        )
        time.sleep(float(CFG["serial_open_delay_sec"]))

        self.set_vesc_duty(0.0)
        self._reset_speed_controller()

        self.create_timer(float(CFG["timer_period_sec"]), self.timer_callback)
        self._speed_pub = self.create_publisher(
            Float64, "/vesc_speed_test/speed_mps", 10
        )
        self._telemetry_pub = self.create_publisher(
            Float64MultiArray, "/vesc_speed_test/telemetry", 10
        )

        self._start_keyboard_estop()

        self.get_logger().info("VESC speed PI test node started")
        self.get_logger().info(
            f"test_enabled={self._test_enabled}, "
            f"configured_target_speed_mps={self._configured_target_speed_mps:.2f}, "
            f"max_target_speed_mps={self._max_target_speed_mps:.2f}, "
            f"speed_kp={self._speed_kp:.3f}, "
            f"speed_ki={self._speed_ki:.3f}, "
            f"duty_rate_limit_per_sec={self._duty_rate_limit_per_sec:.2f}, "
            f"max_auto_duty={self._max_auto_duty:.2f}"
        )
        self.get_logger().info(
            "Speed controller: control = feedforward + P + I, "
            f"speed_ff_duty_per_mps={self._speed_ff_duty_per_mps:.3f}, "
            f"speed_kp={self._speed_kp:.2f}, "
            f"speed_ki={self._speed_ki:.2f}, "
            f"i_enable_error_mps={self._i_enable_error_mps:.1f}, "
            f"target_decrease_i_scale={self._target_decrease_i_scale:.1f}, "
            "negative duty disabled"
        )
        if self._test_enabled:
            self.get_logger().warn(
                "No-load speed PI test enabled. Configured target speed will be "
                "applied after valid VESC telemetry is received"
            )
        else:
            self.get_logger().warn("Test disabled: VESC duty will remain zero")
        if sys.stdin.isatty():
            self.get_logger().info("Keyboard E-stop: Space=stop(latch), R=reset")

    def _compute_speed_from_erpm(self, erpm: float) -> float:
        motor_rpm = float(erpm) / self._pole_pairs
        wheel_rpm = motor_rpm / self._gear_ratio
        raw_speed_mps = wheel_rpm / 60.0 * math.pi * self._wheel_diameter
        if self._invert_speed_sign:
            return -raw_speed_mps
        return raw_speed_mps

    def _reset_speed_controller(self) -> None:
        self._previous_target_speed_mps = 0.0
        self._previous_target_speed_for_log = 0.0
        self._speed_error = 0.0
        self._speed_ff_term = 0.0
        self._speed_integral_duty = 0.0
        self._speed_p_term = 0.0
        self._speed_i_term = 0.0
        self._raw_speed_duty_cmd = 0.0
        self._limited_speed_duty_cmd = 0.0
        self._speed_duty_cmd = 0.0
        self._auto_duty_limiter_active = False
        self._i_integration_active = False
        self._target_decrease_detected = False
        self._last_auto_duty_cmd = 0.0

    def _apply_duty_rate_limit(self, target_duty: float, dt: float) -> float:
        if self._duty_rate_limit_per_sec <= 0.0:
            return target_duty
        dt = self.clamp(dt, 1e-4, 0.1)
        max_step = self._duty_rate_limit_per_sec * dt
        diff = target_duty - self._last_auto_duty_cmd
        if abs(diff) <= max_step:
            return target_duty
        return self._last_auto_duty_cmd + math.copysign(max_step, diff)

    def _update_speed_controller(
        self, target_speed: float, measured_speed: float, dt: float
    ) -> float:
        target_speed = self.clamp(target_speed, 0.0, self._max_target_speed_mps)
        dt = self.clamp(dt, 1e-4, 0.1)

        if target_speed <= 1e-6:
            self._reset_speed_controller()
            return 0.0

        self._previous_target_speed_for_log = self._previous_target_speed_mps
        self._target_decrease_detected = (
            self._previous_target_speed_mps - target_speed
            > self._target_change_threshold_mps
        )
        if self._target_decrease_detected:
            self._speed_integral_duty *= self._target_decrease_i_scale
        self._previous_target_speed_mps = target_speed

        self._speed_error = target_speed - measured_speed
        if target_speed <= self._min_move_speed_mps:
            self._speed_ff_term = 0.0
        else:
            self._speed_ff_term = self._speed_ff_duty_per_mps * target_speed

        # Feedforward estimates steady-state duty, P corrects transients, and I
        # corrects only the remaining model and load error near the target.
        self._speed_p_term = self._speed_kp * self._speed_error

        self._i_integration_active = (
            abs(self._speed_error) <= self._i_enable_error_mps
        )
        if self._i_integration_active:
            candidate_i_term = self._speed_integral_duty + (
                self._speed_ki * self._speed_error * dt
            )
        else:
            candidate_i_term = self._speed_integral_duty

        raw_candidate_effort = (
            self._speed_ff_term + self._speed_p_term + candidate_i_term
        )
        lower_limit = 0.0
        upper_limit = self._max_auto_duty

        accept_integral = (
            lower_limit <= raw_candidate_effort <= upper_limit
            or (raw_candidate_effort > upper_limit and self._speed_error < 0.0)
            or (raw_candidate_effort < lower_limit and self._speed_error > 0.0)
        )
        if self._i_integration_active and accept_integral:
            self._speed_integral_duty = self.clamp(
                candidate_i_term,
                -self._max_auto_duty,
                upper_limit,
            )

        self._speed_i_term = self._speed_integral_duty
        raw_control_effort = (
            self._speed_ff_term + self._speed_p_term + self._speed_i_term
        )
        limited_control_effort = self.clamp(
            raw_control_effort,
            lower_limit,
            upper_limit,
        )
        self._auto_duty_limiter_active = (
            abs(raw_control_effort - limited_control_effort) > 1e-6
        )

        if (
            target_speed > self._min_move_speed_mps
            and measured_speed < self._min_move_speed_mps
            and 0.0 < limited_control_effort < self._min_move_duty
        ):
            limited_control_effort = min(self._min_move_duty, self._max_auto_duty)

        duty_cmd = limited_control_effort * self._auto_duty_output_sign
        duty_cmd = self._apply_duty_rate_limit(duty_cmd, dt)
        duty_cmd = self.clamp(duty_cmd, -self._max_auto_duty, self._max_auto_duty)

        self._last_auto_duty_cmd = duty_cmd
        self._raw_speed_duty_cmd = raw_control_effort
        self._limited_speed_duty_cmd = limited_control_effort
        self._speed_duty_cmd = duty_cmd
        return duty_cmd

    # ============================================================
    # MERGE_END: AUTO SPEED FEEDBACK CONTROL
    # ============================================================

    def _is_estop_latched(self) -> bool:
        with self._estop_lock:
            return self._estop_latched

    def _set_estop_latched(self, latched: bool) -> None:
        with self._estop_lock:
            changed = self._estop_latched != latched
            self._estop_latched = latched
        if not changed:
            return
        self._target_speed_mps = 0.0
        self.current_duty = 0.0
        self._reset_speed_controller()
        self._last_duty_int = None
        self._last_duty_packet = None
        try:
            self.set_vesc_duty(0.0)
        except Exception as exc:
            self.get_logger().error(f"E-stop zero duty send failed: {exc}")
        if latched:
            self.get_logger().warn("E-stop latched: target=0, PI reset, duty=0")
        else:
            self.get_logger().info(
                "E-stop cleared: control will resume according to test_enabled "
                "and the configured target speed"
            )

    def _start_keyboard_estop(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warn(
                "Keyboard E-stop disabled: stdin is not a TTY"
            )
            return

        self._keyboard_running = True
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_listener,
            name="vesc_speed_pi_test_keyboard_estop",
            daemon=True,
        )
        self._keyboard_thread.start()

    def _keyboard_listener(self) -> None:
        fd = sys.stdin.fileno()
        self._stdin_termios_old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._keyboard_running:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                ch = sys.stdin.read(1)
                if ch == " ":
                    self._set_estop_latched(True)
                elif ch.lower() == "r":
                    self._set_estop_latched(False)
        except Exception as exc:
            self.get_logger().error(f"Keyboard E-stop thread failed: {exc}")
        finally:
            if self._stdin_termios_old is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, self._stdin_termios_old)

    def _stop_keyboard_estop(self) -> None:
        self._keyboard_running = False
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=0.5)
            self._keyboard_thread = None

    def timer_callback(self) -> None:
        now = time.time()
        dt = now - self._last_timer_time
        self._last_timer_time = now

        try:
            self._poll_vesc_telemetry(now)

            vesc_fresh = self._vesc_telemetry_fresh(now)

            if self._is_estop_latched():
                self._target_speed_mps = 0.0
                self.current_duty = 0.0
                self._reset_speed_controller()
            elif not self._test_enabled:
                self._target_speed_mps = 0.0
                self.current_duty = 0.0
                self._reset_speed_controller()
            elif self._configured_target_speed_mps <= 1e-6:
                self._target_speed_mps = 0.0
                self.current_duty = 0.0
                self._reset_speed_controller()
            elif not vesc_fresh:
                self._target_speed_mps = self._configured_target_speed_mps
                self.current_duty = 0.0
                self._reset_speed_controller()
            else:
                self._target_speed_mps = self._configured_target_speed_mps
                self.current_duty = self._update_speed_controller(
                    self._target_speed_mps, self._measured_speed_mps, dt
                )

            self.set_vesc_duty(self.current_duty)
            self._publish_speed()
            self._publish_telemetry(vesc_fresh)
            self._write_csv_row(now, vesc_fresh)
            self._maybe_log_debug()
        except Exception as exc:
            self._target_speed_mps = 0.0
            self.current_duty = 0.0
            self._reset_speed_controller()
            try:
                self.set_vesc_duty(0.0)
            except Exception:
                pass
            self.get_logger().error(f"Control loop error: {exc}")

    def _request_vesc_values(self) -> None:
        payload = bytearray([4])
        self.vesc.write(self.make_vesc_packet(payload))

    def _poll_vesc_telemetry(self, now: float) -> None:
        if self._vesc_poll_period > 0.0 and (
            now - self._last_vesc_poll_time >= self._vesc_poll_period
        ):
            self._last_vesc_poll_time = now
            self._request_vesc_values()

        waiting = self.vesc.in_waiting
        if waiting <= 0:
            return

        self._vesc_rx_buffer.extend(self.vesc.read(waiting))
        if len(self._vesc_rx_buffer) > 2048:
            del self._vesc_rx_buffer[:-2048]
        self._parse_vesc_rx_buffer(now)

    def _parse_vesc_rx_buffer(self, now: float) -> None:
        while True:
            start = self._vesc_rx_buffer.find(b"\x02")
            if start < 0:
                self._vesc_rx_buffer.clear()
                return
            if start > 0:
                del self._vesc_rx_buffer[:start]
            if len(self._vesc_rx_buffer) < 5:
                return

            payload_len = self._vesc_rx_buffer[1]
            packet_len = payload_len + 5
            if len(self._vesc_rx_buffer) < packet_len:
                return
            packet = self._vesc_rx_buffer[:packet_len]
            del self._vesc_rx_buffer[:packet_len]

            if packet[-1] != 0x03:
                continue
            payload = packet[2 : 2 + payload_len]
            rx_crc = (packet[2 + payload_len] << 8) | packet[3 + payload_len]
            if self.crc16_ccitt(payload) != rx_crc:
                continue
            self._handle_vesc_payload(payload, now)

    def _handle_vesc_payload(self, payload: bytes | bytearray, now: float) -> None:
        if not payload or payload[0] != 4:
            return
        if len(payload) < 29:
            return

        try:
            current_motor = struct.unpack(">i", payload[5:9])[0] / 100.0
            current_in = struct.unpack(">i", payload[9:13])[0] / 100.0
            erpm = float(struct.unpack(">i", payload[23:27])[0])
            input_voltage = struct.unpack(">h", payload[27:29])[0] / 10.0
        except struct.error:
            return

        self._erpm = erpm
        self._current_motor = current_motor
        self._current_in = current_in
        self._input_voltage = input_voltage
        self._measured_speed_mps = self._compute_speed_from_erpm(erpm)
        self._last_vesc_telemetry_time = now

    def _vesc_telemetry_fresh(self, now: float) -> bool:
        return (
            self._last_vesc_telemetry_time > 0.0
            and now - self._last_vesc_telemetry_time <= self._vesc_telemetry_timeout
        )

    def set_vesc_duty(self, duty: float) -> None:
        duty_limit = abs(self._max_auto_duty)
        duty = self.clamp(duty, -duty_limit, duty_limit)
        self._vesc_duty_now = duty
        duty_int = int(duty * 100000)

        if duty_int == self._last_duty_int and self._last_duty_packet is not None:
            self.vesc.write(self._last_duty_packet)
            return

        payload = bytearray()
        payload.append(5)
        payload.extend(struct.pack(">i", duty_int))

        packet = self.make_vesc_packet(payload)
        self._last_duty_int = duty_int
        self._last_duty_packet = packet
        self.vesc.write(packet)

    def _publish_speed(self) -> None:
        msg = Float64()
        msg.data = float(self._measured_speed_mps)
        self._speed_pub.publish(msg)

    def _publish_telemetry(self, vesc_fresh: bool) -> None:
        msg = Float64MultiArray()
        # Order:
        # 0 target_speed_mps
        # 1 measured_speed_mps
        # 2 speed_error
        # 3 p_term
        # 4 i_term
        # 5 raw_control_effort
        # 6 limited_control_effort
        # 7 duty_cmd
        # 8 erpm
        # 9 current_in
        # 10 current_motor
        # 11 input_voltage
        # 12 duty_limiter_active
        # 13 estop_latched
        msg.data = [
            float(self._target_speed_mps),
            float(self._measured_speed_mps),
            float(self._speed_error),
            float(self._speed_p_term),
            float(self._speed_i_term),
            float(self._raw_speed_duty_cmd),
            float(self._limited_speed_duty_cmd),
            float(self._speed_duty_cmd),
            float(self._erpm),
            float(self._current_in),
            float(self._current_motor),
            float(self._input_voltage),
            1.0 if self._auto_duty_limiter_active else 0.0,
            1.0 if self._is_estop_latched() else 0.0,
        ]
        self._telemetry_pub.publish(msg)

    def _open_csv_log(self) -> None:
        log_dir = Path.cwd() / "vesc_speed_test_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._csv_path = log_dir / f"vesc_speed_test_{stamp}.csv"
        self._csv_file = self._csv_path.open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            [
                "timestamp",
                "elapsed_sec",
                "target_speed_mps",
                "measured_speed_mps",
                "speed_error",
                "p_term",
                "i_term",
                "raw_control_effort",
                "limited_control_effort",
                "duty_cmd",
                "vesc_duty_sent",
                "erpm",
                "current_in",
                "current_motor",
                "input_voltage",
                "duty_limiter_active",
                "vesc_telemetry_fresh",
                "vesc_telemetry_age_sec",
                "drive_command_age_sec",
                "estop_latched",
                "ff_term",
                "i_integration_active",
                "target_decrease_detected",
                "previous_target_speed_mps",
            ]
        )
        self.get_logger().info(f"CSV log: {self._csv_path}")

    def _write_csv_row(self, now: float, vesc_fresh: bool) -> None:
        if self._csv_writer is None:
            return
        vesc_age = (
            now - self._last_vesc_telemetry_time
            if self._last_vesc_telemetry_time > 0.0
            else -1.0
        )
        self._csv_writer.writerow(
            [
                f"{now:.6f}",
                f"{now - self._start_time:.6f}",
                f"{self._target_speed_mps:.6f}",
                f"{self._measured_speed_mps:.6f}",
                f"{self._speed_error:.6f}",
                f"{self._speed_p_term:.6f}",
                f"{self._speed_i_term:.6f}",
                f"{self._raw_speed_duty_cmd:.6f}",
                f"{self._limited_speed_duty_cmd:.6f}",
                f"{self._speed_duty_cmd:.6f}",
                f"{self._vesc_duty_now:.6f}",
                f"{self._erpm:.6f}",
                f"{self._current_in:.6f}",
                f"{self._current_motor:.6f}",
                f"{self._input_voltage:.6f}",
                1 if self._auto_duty_limiter_active else 0,
                1 if vesc_fresh else 0,
                f"{vesc_age:.6f}",
                "-1.000000",
                1 if self._is_estop_latched() else 0,
                f"{self._speed_ff_term:.6f}",
                1 if self._i_integration_active else 0,
                1 if self._target_decrease_detected else 0,
                f"{self._previous_target_speed_for_log:.6f}",
            ]
        )
        if now - self._last_csv_flush_time >= 1.0:
            self._last_csv_flush_time = now
            self._csv_file.flush()

    def _maybe_log_debug(self) -> None:
        if self._debug_log_hz <= 0.0:
            return
        now = time.time()
        if now - self._last_debug_log_time < 1.0 / self._debug_log_hz:
            return
        self._last_debug_log_time = now
        self.get_logger().info(
            f"target={self._target_speed_mps:.3f} "
            f"measured={self._measured_speed_mps:.3f} "
            f"error={self._speed_error:.3f} "
            f"FF={self._speed_ff_term:.4f} "
            f"P={self._speed_p_term:.4f} "
            f"I={self._speed_i_term:.4f} "
            f"raw duty={self._raw_speed_duty_cmd:.4f} "
            f"limited duty={self._limited_speed_duty_cmd:.4f} "
            f"sent duty={self._vesc_duty_now:.4f} "
            f"ERPM={self._erpm:.0f} "
            f"input voltage={self._input_voltage:.1f} "
            f"current_in={self._current_in:.2f} "
            f"estop={self._is_estop_latched()}"
        )

    @staticmethod
    def make_vesc_packet(payload: bytearray) -> bytearray:
        packet = bytearray()
        packet.append(0x02)
        packet.append(len(payload))
        packet.extend(payload)

        crc = VescSpeedPITestNode.crc16_ccitt(payload)
        packet.append((crc >> 8) & 0xFF)
        packet.append(crc & 0xFF)
        packet.append(0x03)
        return packet

    @staticmethod
    def crc16_ccitt(data: bytearray) -> int:
        crc = 0
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    def destroy_node(self) -> None:
        self._stop_keyboard_estop()
        self.get_logger().info("Stopping VESC speed PI test...")
        self._target_speed_mps = 0.0
        self.current_duty = 0.0
        self._reset_speed_controller()
        try:
            self.set_vesc_duty(0.0)
            time.sleep(0.1)
            self.set_vesc_duty(0.0)
        except Exception as exc:
            self.get_logger().error(f"Stop failed: {exc}")

        try:
            if self._csv_file is not None:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception:
            pass

        try:
            self.vesc.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VescSpeedPITestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"Fatal error: {exc}")
        else:
            print(f"Fatal error before node startup: {exc}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
