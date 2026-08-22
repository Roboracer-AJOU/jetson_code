#!/usr/bin/env python3
"""
실차 하드웨어 제어: /drive (AckermannDriveStamped) → ESP32 조향 + VESC duty.

CH5 (PPM index [4] ONLY) 로 수동/자율:
  - CH5 <= 1300 (1000, 수동): CH1→ESP, CH2→VESC
  - CH5 >= 1700 (2000, 자율): /drive.speed 목표속도 PI→VESC + S:→ESP

ESP → Jetson: RC,ch1_us,ch2_us,ch5_us,0  (raw PWM us, 1000~2000)

터미널에서 Space → 비상정지(래치), r → 해제.
"""
from __future__ import annotations

import math
import os
import select
import struct
import sys
import termios
import threading
import time
import tty

import rclpy
import serial
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, Float64MultiArray


# ============================================================
# USER TUNING — vehicle control (여기만 수정)
# ============================================================
CFG = {
    "drive_topic": "/drive",
    # ESP: USB-C (C to C). auto = /dev/serial/by-id 에서 ESP 보드를 찾는다.
    "esp_port": "auto",
    "vesc_port": "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00",
    "esp_baud": 115200,         # USB-C (RC 텔레메트리 + S:)
    "vesc_baud": 115200,
    # Stanley max_drive_speed / max_steering_angle 과 맞추면 1:1 스케일
    "max_speed_mps": 10.0,
    # /drive 의 steering_angle [rad] 을 S∈[-1,1] 로 정규화할 때 쓰는 풀스케일.
    #
    # Stanley 는 steer_scale_calibrated=True 에서 게인을 ×0.428 재환산해
    # **실전륜각 [rad]** 을 내보낸다 (풀스케일 0.3735 = 21.4°). 그러니 여기
    # 분모도 0.3735 여야 S=±1 이 실제 풀락에 대응한다.
    #
    # 0.8726646 은 ESP normToAngle 의 서보 혼 이동각(40°/140°)이지 전륜각이
    # 아니다. 그걸 분모로 쓰면 실제 21.4° 를 요구해도 S=0.428 밖에 안 나가
    # 조향이 42.8% 로 깎인다. 저속에서는 피드백이 메워서 티가 안 나고,
    # 고속에서 요구 조향각이 커지면 풀락에 걸려 라인을 벗어난다.
    #
    # 한 번 0.8726 으로 되돌린 적이 있는데, 그때 "2.34배 세져서 벽에 박았다"
    # 고 본 건 오진이었다. 그 주행은 raceline CSV 감김방향이 뒤집혀 헤딩오차가
    # 180° 였던 구간이다.
    #
    # 이 값은 Stanley 의 max_steering_angle_real_rad 와 **반드시 같아야** 한다.
    # 한쪽만 바꾸면 정규화가 어긋난다.
    "max_steering_angle_rad": 0.3735,  # ±21.4° 실측 전륜각
    "max_duty": 0.3,           # MANUAL mode CH2 duty limit
    "speed_scale": 1.0,         # 추가 감쇠 (1.0=끔)
    "min_move_duty": 0.06,      # 정지마찰 극복용 최소 duty (speed>threshold 일 때)
    "manual_duty_rise_rate_per_sec": 0.25,
    "manual_duty_fall_rate_per_sec": 0.10,
    "min_move_speed_mps": 0.08,
    "status_log_hz": 2.0,         # 터미널 속도/제어 STATUS (0=끔)
    "max_steer": 1.0,          # ESP 조향 명령 범위. 1.0이면 서보 ±50°까지 사용
    # 0 = 끔. 조향 변화율은 Stanley 쪽 steering_rate_limit_radps 에서만 건다
    # (여기서 한 번 더 자르면 회피용으로 올려둔 응답속도가 무효화된다).
    "steer_rate_limit_per_sec": 0.0,
    "steer_cmd_format": "prefixed",  # plain: "0.500\n" | prefixed: "S:0.500\n"
    "invert_speed": False,      # legacy AUTO sign flag; prefer auto_duty_output_sign
    # ESP normToAngle: S:-1→좌(40°), S:+1→우(140°) — INVERT_RC_STEER 미적용
    # Stanley +steer=좌 → S:- 로 보내야 함 (False면 AUTO 조향 반대 → 옆으로 밀림)
    "invert_steer": False,
    "cmd_timeout_sec": 0.25,
    "timer_period_sec": 0.02,     # ESP loop 20ms — AUTO 조향 응답
    "serial_open_delay_sec": 0.0,
    "enable_keyboard_estop": True,
    "estop_reset_key": "r",
    # RC (ESP -> RC,ch1_us,ch2_us,mode_us,0)  raw PWM us
    "ch5_manual_us": 1300,        # CH5 <= 1300 수동(1000)
    "ch5_auto_us": 1700,          # CH5 >= 1700 자율(2000)
    "rc_center_ch2": 1500,
    "rc_min_val": 0,
    "rc_max_val": 3000,
    "rc_deadzone": 30,
    "rc_timeout_sec": 0.30,
    # ESP validUs() 와 같은 범위. 밖이면 그 채널은 손실로 본다.
    "rc_valid_min_us": 800,
    "rc_valid_max_us": 2200,
    # ESP ESTOP_THRESHOLD_US 와 같은 값이어야 한다. 젯슨이 더 높으면 ESP 는 서보를
    # 중앙에 고정하는데 젯슨은 duty 를 계속 내보내는 구간이 생긴다.
    # CH340 DTR/RTS 자동 리셋 회로 때문에 포트를 여는 순간 ESP 가 재부팅된다
    # (rst:0x1 POWERON_RESET). 리눅스 tty 가 open 시 DTR 을 올려서 pyserial 로는 못 막는다.
    # 부팅 + PPM 재동기가 끝날 때까지 RC 를 손실로 두고 duty 를 끊는 유예 시간.
    "esp_boot_settle_sec": 1.5,
    # ESP 가 리셋 후 PPM 락에 실패했을 때 다시 리셋을 걸기까지 기다리는 시간과 횟수.
    # 실측상 리셋 1회당 락 성공률이 약 2/3 이라 5회면 사실상 확실히 붙는다.
    "esp_recovery_wait_sec": 2.0,
    "esp_recovery_max_tries": 5,
    # ESP ESTOP_THRESHOLD_US / ESTOP_RELEASE_US 와 같은 값이어야 한다.
    # 1500 은 3단 스위치 중립값과 겹쳐서 가만히 있어도 ESTOP 이 걸렸다.
    # 실측 정상값 CH6=1000, ESTOP 위치=2000.
    "ch6_estop_us": 1700,         # CH6 >= 1700 ESTOP latch
    "ch6_estop_release_us": 1400,  # CH6 <= 1400 이면 RC 로 건 latch 해제 (히스테리시스)
    "invert_rc_throttle": False,  # 송신기 CH2 전후진 duty 부호 (True면 반전)
    "auto_duty_ramp_sec": 1.0,    # AUTO: /drive duty → VESC (1초에 목표까지)
    "telemetry_topic": "/vehicle/telemetry",  # drive_monitor.py 구독
    "speed_topic": "/vehicle/speed_mps",
    # AUTO closed-loop speed control (max_target_speed_mps is target speed [m/s])
    "max_auto_duty": 0.70,        # AUTO final safety duty limit
    "max_target_speed_mps": 10.0,  # AUTO 목표 속도 하드 상한 (안전 클램프)
    # AUTO 목표 속도를 어디서 받을지.
    #   True  : /drive.speed (stanley 가 CSV v 열에서 뽑아 보내는 값) — 구간별 가변
    #   False : 아래 target_speed_mps 로 전 구간 정속 (구동계 튜닝용)
    # 어느 쪽이든 /drive 가 cmd_timeout_sec 이상 끊기면 duty·steer 0 으로 떨어진다.
    "use_drive_speed_command": True,
    "target_speed_mps": 2.0,       # use_drive_speed_command=False 일 때만 사용
    # ---- 비상 제동 (emergency_brake_node) ----
    # AUTO 에서만 동작. 키보드 Space / RC CH6 ESTOP 과 달리 자동으로 풀린다.
    # AUTO 속도 PI 는 duty 하한이 0 이라 타력주행밖에 못 한다. 여기서만 역토크를
    # 걸어 실제로 세운다.
    "emergency_brake_topic": "/emergency_brake",
    "emergency_brake_duty": 0.15,      # 역방향 duty 크기. 크면 잠기고 미끄러진다
    # 이 속도 아래로 떨어지면 역토크 해제 (계속 걸면 후진한다)
    "emergency_brake_release_speed_mps": 0.15,
    # 신호가 오다가 끊기면 제동 (노드가 죽은 것 → fail-safe).
    # 한 번도 못 받았으면 노드 미사용으로 보고 무시한다.
    "emergency_brake_stale_sec": 0.5,
    # ---- 후진 탈출 (emergency_brake_node 가 요청) ----
    # 장애물 코앞에 서면 최대 조향으로도 못 나간다. 그때만 곧게 물러난다.
    # 제동 역토크와 같은 하드웨어 경로라 새로 검증할 것은 없고, 다른 점은
    # "멈출 때까지" 가 아니라 "정해진 만큼 뒤로" 라는 것뿐이다.
    #
    # 제동과 달리 이건 차를 **움직이는** 명령이라 신호가 없을 때의 기본값이
    # 반대다. 제동은 끊기면 걸고(fail-safe), 후진은 끊기면 푼다.
    "escape_reverse_topic": "/aeb/escape_reverse",
    "escape_reverse_duty": 0.12,        # 제동(0.15)보다 약하게 — 기어가는 수준
    "escape_reverse_stale_sec": 0.3,
    # 요청이 붙박이로 True 가 돼도 이 시간이면 끊는다. 요청이 한 번 False 로
    # 떨어져야 다시 걸린다 — 노드가 이상해져도 계속 뒤로 가지는 않는다.
    "escape_reverse_max_sec": 2.5,
    # 이보다 빨리 물러나면 duty 를 끊고 타력으로 둔다.
    "escape_reverse_max_speed_mps": 0.6,
    "auto_duty_output_sign": 1.0,
    "speed_ff_duty_per_mps": 0.076,
    "speed_kp": 0.15,
    "speed_ki": 0.03,
    "i_enable_error_mps": 0.5,
    "target_decrease_i_scale": 0.5,
    "target_change_threshold_mps": 0.1,
    "duty_rate_limit_per_sec": 0.60,
    "vesc_telemetry_timeout_sec": 0.3,
    # 20Hz(0.05)면 /vehicle/speed_mps 알맹이가 20Hz라 odom이 zero-order hold로
    # 같은 속도를 2~3번 재사용하고, 응답도 최대 50ms 묵은 값이 된다(3m/s에서 ~15cm
    # 뒤처진 prior). VESC는 /dev/ttyACM0 = USB CDC라 vesc_baud는 이름뿐이고
    # GET_VALUES 응답 ~80B에 실제 대역폭 제약이 없어서 매 틱 폴링으로 올림.
    # 폴링은 timer_period_sec(0.02) 틱에서만 일어나므로 상한이 곧 50Hz다.
    # _last_vesc_poll_time을 now로 리셋하는 구조라 이 값을 틱 주기와 같은 0.02로
    # 두면 타이머 지터로 한 틱씩 건너뛰어 50/25Hz를 오간다. 틱보다 짧게 잡아
    # 매 틱 확실히 폴링되게 함.
    "vesc_poll_period_sec": 0.015,
    "invert_speed_sign": False,
    "pole_pairs": 2,
    "gear_ratio": 12.0,
    "wheel_diameter": 0.10,
}


def _serial_by_id_ports() -> list[str]:
    root = "/dev/serial/by-id"
    if not os.path.isdir(root):
        return []
    return sorted(os.path.join(root, name) for name in os.listdir(root))


def _is_vesc_id(path: str) -> bool:
    name = os.path.basename(path)
    return "ChibiOS" in name or "STMicroelectronics" in name


def _is_imu_id(path: str) -> bool:
    return "CP2102_USB_to_UART_Bridge_Controller_0001" in os.path.basename(path)


def _is_esp_id(path: str) -> bool:
    name = os.path.basename(path)
    return any(
        key in name
        for key in (
            "Espressif",
            "USB_JTAG",
            "USB_SERIAL",
            "CH340",
            "CH910",
            "QinHeng",
            "1a86",
            "USB_Serial",
        )
    )


def resolve_serial_port(role: str, configured: str) -> str:
    configured = str(configured).strip()
    if configured and configured != "auto" and os.path.exists(configured):
        return configured

    by_id = _serial_by_id_ports()
    if role == "vesc":
        for path in by_id:
            if _is_vesc_id(path):
                return path
        raise RuntimeError(
            "VESC USB를 못 찾음. 현재 by-id: " + (", ".join(by_id) or "없음")
        )

    for path in by_id:
        if _is_vesc_id(path) or _is_imu_id(path):
            continue
        if _is_esp_id(path):
            return path
    extras = [
        path for path in by_id if not _is_vesc_id(path) and not _is_imu_id(path)
    ]
    if extras:
        return extras[0]
    raise RuntimeError(
        "ESP USB 시리얼을 못 찾음. /dev/ttyTHS1 UART는 쓰지 않음. "
        "데이터 케이블로 ESP USB-C를 꽂은 뒤 ls -l /dev/serial/by-id 확인. "
        "현재: " + (", ".join(by_id) or "없음")
    )


def _set_usb_latency_timer(port: str, latency_ms: int = 1) -> None:
    name = os.path.basename(os.path.realpath(port))
    for path in (
        f"/sys/class/tty/{name}/device/latency_timer",
        f"/sys/bus/usb-serial/devices/{name}/latency_timer",
    ):
        try:
            with open(path, "w", encoding="ascii") as handle:
                handle.write(str(int(latency_ms)))
            return
        except OSError:
            continue


def _open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = int(baud)
    ser.timeout = 0.0
    ser.write_timeout = 0.0
    ser.dsrdtr = False
    ser.rtscts = False
    # open() 전에 내려야 한다. pyserial 기본값이 True 라서 나중에 내리면
    # 여는 순간 DTR 이 한 번 올라가고, ESP32 자동 리셋 회로가 이걸 리부팅으로 받는다.
    ser.dtr = False
    ser.rts = False
    ser.open()
    _set_usb_latency_timer(port, 1)
    return ser


class VehicleControlNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_control_node")

        self._drive_topic = str(CFG["drive_topic"])
        self._max_speed_mps = max(float(CFG["max_speed_mps"]), 1e-3)
        self._max_steering_angle_rad = max(
            float(CFG["max_steering_angle_rad"]), 1e-3
        )
        self._warn_if_steer_scale_mismatch()
        self._max_duty = float(CFG["max_duty"])
        self._speed_scale = self.clamp(float(CFG["speed_scale"]), 0.0, 1.0)
        self._min_move_duty = max(0.0, float(CFG["min_move_duty"]))
        self._manual_duty_rise_rate = max(
            0.0, float(CFG.get("manual_duty_rise_rate_per_sec", 0.25))
        )
        self._manual_duty_fall_rate = max(
            0.0, float(CFG.get("manual_duty_fall_rate_per_sec", 0.10))
        )
        self._min_move_speed_mps = max(0.0, float(CFG["min_move_speed_mps"]))
        self._status_log_hz = max(0.0, float(CFG.get("status_log_hz", 2.0)))
        self._status_log_period = (
            1.0 / self._status_log_hz if self._status_log_hz > 0.0 else 0.0
        )
        self._status_log_accum = 0.0
        self._max_steer = float(CFG["max_steer"])
        self._steer_rate_limit_per_sec = max(
            0.0, float(CFG.get("steer_rate_limit_per_sec", 1.5))
        )
        self._steer_cmd_format = str(CFG.get("steer_cmd_format", "plain")).lower()
        self._invert_speed = bool(CFG["invert_speed"])
        self._invert_steer = bool(CFG["invert_steer"])
        self._cmd_timeout = float(CFG["cmd_timeout_sec"])
        self._estop_reset_key = str(CFG["estop_reset_key"]).lower()[:1] or "r"
        self._ch5_manual_us = int(CFG["ch5_manual_us"])
        self._ch5_auto_us = int(CFG["ch5_auto_us"])
        self._mode_auto_latched = False
        self._rc_center_ch2 = int(CFG["rc_center_ch2"])
        self._rc_min_val = int(CFG["rc_min_val"])
        self._rc_max_val = int(CFG["rc_max_val"])
        self._rc_deadzone = int(CFG["rc_deadzone"])
        self._rc_timeout = float(CFG["rc_timeout_sec"])
        self._rc_valid_min_us = int(CFG.get("rc_valid_min_us", 800))
        self._rc_valid_max_us = int(CFG.get("rc_valid_max_us", 2200))
        self._ch6_estop_us = int(CFG.get("ch6_estop_us", 1500))
        self._ch6_estop_release_us = min(
            int(CFG.get("ch6_estop_release_us", 1400)), self._ch6_estop_us
        )
        self._invert_rc_throttle = bool(CFG.get("invert_rc_throttle", False))
        self._auto_duty_ramp_sec = max(0.0, float(CFG.get("auto_duty_ramp_sec", 1.0)))
        self._max_auto_duty = max(0.0, float(CFG.get("max_auto_duty", 0.30)))
        self._max_auto_brake_duty = self.clamp(
            max(0.0, float(CFG.get("max_auto_brake_duty", self._max_auto_duty))),
            0.0,
            self._max_auto_duty,
        )
        self._max_target_speed_mps = max(0.0, float(CFG.get("max_target_speed_mps", 3.0)))
        self._configured_target_speed_mps = self.clamp(
            float(CFG["target_speed_mps"]),
            0.0,
            self._max_target_speed_mps,
        )
        self._use_drive_speed_command = bool(
            CFG.get("use_drive_speed_command", False)
        )
        self._emergency_brake_duty = abs(
            float(CFG.get("emergency_brake_duty", 0.15))
        )
        self._emergency_brake_release_speed = max(
            0.0, float(CFG.get("emergency_brake_release_speed_mps", 0.15))
        )
        self._emergency_brake_stale = max(
            0.0, float(CFG.get("emergency_brake_stale_sec", 0.5))
        )
        self._emergency_brake_cmd = False
        self._emergency_brake_recv_time = 0.0
        self._emergency_brake_engaged = False
        self._escape_reverse_duty = abs(float(CFG.get("escape_reverse_duty", 0.12)))
        self._escape_reverse_stale = max(
            0.0, float(CFG.get("escape_reverse_stale_sec", 0.3))
        )
        self._escape_reverse_max_sec = max(
            0.0, float(CFG.get("escape_reverse_max_sec", 2.5))
        )
        self._escape_reverse_max_speed = max(
            0.0, float(CFG.get("escape_reverse_max_speed_mps", 0.6))
        )
        self._escape_reverse_cmd = False
        self._escape_reverse_recv_time = 0.0
        self._escape_reverse_since = 0.0
        self._escape_reverse_spent = False
        self._escape_reverse_engaged = False
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
        self._estop_source: str | None = None
        self._keyboard_running = False
        self._keyboard_thread: threading.Thread | None = None
        self._stdin_termios_old = None

        self.last_cmd_time = time.time()
        self._last_timer_time = time.time()
        self._auto_duty = 0.0
        self._auto_duty_applied = 0.0
        self._auto_steer = 0.0
        self._auto_steer_applied = 0.0
        self._target_speed_mps = 0.0
        # [TEMP] straight_drive_publisher — MANUAL 전용 /drive.speed 목표
        self._manual_drive_target_mps = 0.0
        self._manual_drive_speed_active = False
        self._manual_drive_invert_duty = False
        self._last_manual_drive_cmd_time = 0.0
        self._previous_auto_target_speed_mps = 0.0
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
        self._vesc_duty_feedback = 0.0
        self.current_steer = 0.0
        self._last_duty_int = None
        self._last_duty_packet = None
        self._manual_duty_applied = 0.0
        self._last_speed_mps = 0.0
        self._last_steering_rad = 0.0
        self._last_vesc_poll_time = 0.0
        self._last_vesc_telemetry_time = 0.0
        self._vesc_rx_buffer = bytearray()
        self._erpm = 0.0
        self._measured_speed_mps = 0.0
        self._current_motor = 0.0
        self._current_in = 0.0
        self._input_voltage = 0.0

        self._rc_ch1 = 1497
        self._rc_ch2 = 1497
        self._rc_ch5 = 1000
        self._rc_ch6 = 0
        self._last_rc_time = 0.0
        self._rc_signal_ok = True
        self.esp32_target_angle_deg = None
        self.esp32_servo_command_deg = None
        self.last_esp32_packet_time = None
        self.last_esp32_steering_time = None
        self._esp_rx_buffer = bytearray()
        self._control_mode = "INIT"

        # CH340 자동 리셋 때문에 포트를 열면 ESP 가 반드시 한 번 재부팅된다.
        # 부팅 + PPM 재동기가 끝날 때까지는 RC 를 손실로 취급해야 안전하다.
        self._esp_boot_settle_sec = float(CFG.get("esp_boot_settle_sec", 1.5))
        self._esp_boot_time = time.time()
        self._esp_boot_log_time = 0.0
        self._esp_boot_count = 0
        self._esp_unparsed_count = 0
        self._esp_unparsed_log_time = 0.0

        # PPM 락 실패 시 ESP 를 다시 리셋해서 되살리는 우회책
        self._esp_recovery_wait_sec = float(CFG.get("esp_recovery_wait_sec", 2.0))
        self._esp_recovery_max_tries = int(CFG.get("esp_recovery_max_tries", 5))
        self._esp_recovery_tries = 0
        self._esp_rc_dead_since = 0.0

        esp_port = resolve_serial_port("esp", CFG["esp_port"])
        vesc_port = resolve_serial_port("vesc", CFG["vesc_port"])
        self.get_logger().info(f"Opening ESP32 USB-C: {esp_port}")
        self.esp = _open_serial(esp_port, int(CFG["esp_baud"]))
        self.get_logger().info(f"Opening VESC serial: {vesc_port}")
        self.vesc = _open_serial(vesc_port, int(CFG["vesc_baud"]))

        time.sleep(float(CFG["serial_open_delay_sec"]))

        self.create_subscription(
            AckermannDriveStamped,
            self._drive_topic,
            self.drive_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(CFG.get("emergency_brake_topic", "/emergency_brake")),
            self._emergency_brake_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(CFG.get("escape_reverse_topic", "/aeb/escape_reverse")),
            self._escape_reverse_callback,
            10,
        )
        self.create_timer(float(CFG["timer_period_sec"]), self.timer_callback)
        tel_topic = str(CFG.get("telemetry_topic", "/vehicle/telemetry"))
        self._telemetry_pub = self.create_publisher(Float64MultiArray, tel_topic, 10)
        speed_topic = str(CFG.get("speed_topic", "/vehicle/speed_mps"))
        self._speed_pub = self.create_publisher(Float64, speed_topic, 10)
        self.esp32_target_angle_pub = self.create_publisher(
            Float32, "/esp32/target_angle_deg", 10
        )
        self.esp32_servo_command_pub = self.create_publisher(
            Float32, "/esp32/servo_command_deg", 10
        )

        if bool(CFG["enable_keyboard_estop"]):
            self._start_keyboard_estop()

        self.get_logger().info("Vehicle control node started")
        self.get_logger().info(f"Subscribing: {self._drive_topic} (AckermannDriveStamped)")
        self.get_logger().info(
            f"Scale: speed≤{self._max_speed_mps} m/s × scale={self._speed_scale} "
            f"→ duty±{self._max_duty}, min_move_duty={self._min_move_duty}, "
            f"steer≤{self._max_steering_angle_rad} rad → cmd±{self._max_steer}"
        )
        self.get_logger().info("Output: ESP32 steering + VESC duty (CH5 mode switch)")
        self.get_logger().info(
            f"RC manual: CH5<={self._ch5_manual_us} -> CH2->VESC, "
            f"CH5>={self._ch5_auto_us} -> /drive->VESC + S:->ESP"
        )
        self.get_logger().info(
            f"max_duty = MANUAL CH2 duty limit: {self._max_duty:.2f}"
        )
        self.get_logger().info(
            f"max_auto_duty = AUTO final safety limit: {self._max_auto_duty:.2f}"
        )
        self.get_logger().info(
            f"max_target_speed_mps = AUTO 목표 속도 상한: "
            f"{self._max_target_speed_mps:.2f}"
        )
        if self._use_drive_speed_command:
            self.get_logger().info(
                "AUTO target speed = /drive.speed (stanley CSV v열, 구간별 가변). "
                f"상한 {self._max_target_speed_mps:.2f} m/s 로 클램프"
            )
        else:
            self.get_logger().info(
                f"AUTO target speed = target_speed_mps 정속: "
                f"{self._configured_target_speed_mps:.2f} m/s "
                "(구간별 속도를 쓰려면 use_drive_speed_command=True)"
            )
        self.get_logger().info(
            f"AEB: {CFG.get('emergency_brake_topic')} 수신 시 역토크 duty "
            f"{self._emergency_brake_duty:.2f} (AUTO 전용, 자동 해제). "
            "신호가 오다 끊기면 제동"
        )
        self.get_logger().info(
            f"AUTO speed FF+PI: target≤{self._max_target_speed_mps:.2f} m/s, "
            f"duty≤{self._max_auto_duty:.2f}, ff={self._speed_ff_duty_per_mps:.3f}, "
            f"kp={self._speed_kp:.3f}, ki={self._speed_ki:.3f}, "
            f"rate≤{self._duty_rate_limit_per_sec:.2f}/s"
        )
        if bool(CFG["enable_keyboard_estop"]) and sys.stdin.isatty():
            self.get_logger().info(
                f"Keyboard ESTOP: Space=stop(latch), {self._estop_reset_key.upper()}=reset"
            )

    def _is_estop_latched(self) -> bool:
        with self._estop_lock:
            return self._estop_latched

    def _set_estop_latched(self, latched: bool, source: str = "key") -> None:
        """source: latch 를 건 주체. 'rc' 로 건 것은 'rc' 로만, 'key' 로 건 것은
        키보드로만 풀린다. 조종기 글리치로 걸린 latch 가 키보드 없이는 안 풀려서
        모든 명령이 먹통이 되던 문제 때문이다."""
        with self._estop_lock:
            if latched:
                changed = not self._estop_latched
                self._estop_latched = True
                if changed:
                    self._estop_source = source
            else:
                # 키보드는 마스터 리셋이라 무엇으로 걸렸든 푼다. RC 는 RC 로 건
                # latch 만 풀 수 있어서, 사람이 스페이스로 건 정지를 조종기가
                # 임의로 해제하지 못한다.
                if (
                    self._estop_latched
                    and source == "rc"
                    and self._estop_source != "rc"
                ):
                    return
                changed = self._estop_latched
                self._estop_latched = False
                self._estop_source = None
            held_by = self._estop_source
        if not changed:
            return
        self.current_duty = 0.0
        self.current_steer = 0.0
        self._reset_speed_controller()
        self._last_duty_int = None
        self._last_duty_packet = None
        if latched:
            how = (
                "CH6 를 내리거나 "
                if held_by == "rc"
                else ""
            )
            self.get_logger().warn(
                f"ESTOP latched ({held_by}) — output forced to zero "
                f"({how}{self._estop_reset_key.upper()} 키로 해제)"
            )
        else:
            self.get_logger().info("ESTOP cleared — /drive commands accepted again")

    def _start_keyboard_estop(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warn(
                "Keyboard ESTOP disabled: stdin is not a TTY "
                "(run `ros2 run path_following control_node` in a terminal)"
            )
            return

        self._keyboard_running = True
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_listener,
            name="control_node_keyboard_estop",
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
                elif ch.lower() == self._estop_reset_key:
                    self._set_estop_latched(False)
        except Exception as e:
            self.get_logger().error(f"Keyboard ESTOP thread failed: {e}")
        finally:
            if self._stdin_termios_old is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, self._stdin_termios_old)

    def _stop_keyboard_estop(self) -> None:
        self._keyboard_running = False
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=0.5)
            self._keyboard_thread = None

    def drive_callback(self, msg: AckermannDriveStamped) -> None:
        if self._is_estop_latched():
            return

        cmd_speed = float(msg.drive.speed)
        steering_rad = float(msg.drive.steering_angle)

        if self._is_autonomous_mode():
            self._manual_drive_speed_active = False
            if not math.isfinite(cmd_speed):
                return

            self.last_cmd_time = time.time()
            if self._use_drive_speed_command:
                # stanley 가 CSV v 열에서 뽑아 보낸 구간별 목표 속도
                target_speed_mps = self.clamp(
                    abs(cmd_speed), 0.0, self._max_target_speed_mps
                )
            else:
                target_speed_mps = self._configured_target_speed_mps

            if self._invert_steer:
                steering_rad = -steering_rad

            steer_norm = self.clamp(
                steering_rad / self._max_steering_angle_rad, -1.0, 1.0
            )

            self._target_speed_mps = target_speed_mps
            self._last_speed_mps = target_speed_mps
            self._last_steering_rad = steering_rad
            self._auto_steer = steer_norm * self._max_steer
            return

        # MANUAL: [TEMP] straight_drive — AUTO용 last_cmd_time / _auto_steer 건드리지 않음
        if math.isfinite(cmd_speed) and abs(cmd_speed) > 1e-6:
            self._manual_drive_target_mps = self.clamp(
                abs(cmd_speed), 0.0, self._max_target_speed_mps
            )
            self._manual_drive_invert_duty = cmd_speed < 0.0
            self._manual_drive_speed_active = True
            self._last_manual_drive_cmd_time = time.time()
        else:
            self._manual_drive_speed_active = False
            self._manual_drive_invert_duty = False

    def _emergency_brake_callback(self, msg: Bool) -> None:
        self._emergency_brake_cmd = bool(msg.data)
        self._emergency_brake_recv_time = time.time()

    def _escape_reverse_callback(self, msg: Bool) -> None:
        want = bool(msg.data)
        if not want:
            # 요청이 내려가야 시간 예산이 되살아난다. 이게 붙박이 True 에
            # 대한 방어다.
            self._escape_reverse_since = 0.0
            self._escape_reverse_spent = False
        self._escape_reverse_cmd = want
        self._escape_reverse_recv_time = time.time()

    def _escape_reverse_requested(self, now: float) -> bool:
        """후진 탈출 요청 여부.

        제동과 정반대의 실패 규칙을 쓴다. 제동은 신호가 끊기면 거는 게
        안전하지만, 후진은 차를 움직이는 명령이라 끊기면 푸는 게 안전하다.
        """
        if not self._escape_reverse_cmd:
            return False
        if self._escape_reverse_recv_time <= 0.0:
            return False
        if now - self._escape_reverse_recv_time > self._escape_reverse_stale:
            return False
        if self._escape_reverse_spent:
            return False
        if self._escape_reverse_since <= 0.0:
            self._escape_reverse_since = now
        elif now - self._escape_reverse_since > self._escape_reverse_max_sec:
            self._escape_reverse_spent = True
            self.get_logger().warn(
                f"후진 탈출 시간 상한 {self._escape_reverse_max_sec:.1f}s 초과 — "
                f"끊는다. 요청이 한 번 내려가야 다시 건다"
            )
            return False
        return True

    def _escape_reverse_output_duty(self) -> float:
        """후진 duty. 너무 빨라지면 끊고 타력으로 둔다."""
        if abs(self._measured_speed_mps) >= self._escape_reverse_max_speed > 0.0:
            return 0.0
        return -self._escape_reverse_duty * self._auto_duty_output_sign

    def _emergency_brake_requested(self, now: float) -> bool:
        """AEB 제동 요청 여부.

        한 번도 수신한 적 없으면 노드를 안 띄운 것으로 보고 무시한다.
        수신하다 끊기면 노드가 죽은 것이므로 제동한다 (fail-safe).
        """
        if self._emergency_brake_recv_time <= 0.0:
            return False
        if now - self._emergency_brake_recv_time > self._emergency_brake_stale:
            return True
        return self._emergency_brake_cmd

    def _emergency_brake_output_duty(self) -> float:
        """역토크 duty. 앞으로 가고 있을 때만 건다.

        `_measured_speed_mps` 는 ERPM 에서 온 **부호 있는** 값이다. 예전에는
        `abs()` 로 해제를 판정했는데, 그러면 이미 뒤로 구르는 중일 때
        |속도| 가 다시 임계를 넘어 역토크가 되살아난다 — 뒤로 갈수록 더
        세게 미는 폭주다. 감속 중에 해제 구간(±0.15)을 한 틱에 건너뛰면
        (20 Hz 에서 충분히 일어난다) 그대로 후진으로 넘어갔다.

        부호를 그대로 보면 전진 중일 때만 걸리고, 멈췄거나 이미 뒤로
        구르면 0 이다. 의도한 후진은 `_escape_reverse_output_duty` 가 따로
        낸다.
        """
        if self._measured_speed_mps <= self._emergency_brake_release_speed:
            return 0.0
        return -self._emergency_brake_duty * self._auto_duty_output_sign

    @staticmethod
    def _parse_rc_kv_line(line: str):
        """USB 디버그 포맷: COUNT=8 CH1=1505 CH2=1499 CH5=1000 CH6=1000 ... servo_cmd=90"""
        fields = {}
        for token in line.replace(",", " ").split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            fields[key.strip().upper()] = value.strip()
        if "CH1" not in fields or "CH2" not in fields or "CH5" not in fields:
            return None
        try:
            ch1 = int(float(fields["CH1"]))
            ch2 = int(float(fields["CH2"]))
            ch5 = int(float(fields["CH5"]))
            ch6 = int(float(fields.get("CH6", "0")))
        except ValueError:
            return None
        target_angle_deg = None
        servo_command_deg = None
        try:
            if "TARGET" in fields:
                target_angle_deg = float(fields["TARGET"])
            if "SERVO_CMD" in fields:
                servo_command_deg = float(fields["SERVO_CMD"])
        except ValueError:
            target_angle_deg = None
            servo_command_deg = None
        return (ch1, ch2, ch5, ch6, target_angle_deg, servo_command_deg)

    @staticmethod
    def _parse_rc_line(line: str):
        parts = [item.strip() for item in line.strip().split(",")]
        if len(parts) < 5 or parts[0] != "RC":
            return VehicleControlNode._parse_rc_kv_line(line)
        try:
            ch1 = int(parts[1])
            ch2 = int(parts[2])
            ch5 = int(parts[3])
            ch6 = int(parts[4])
        except (ValueError, IndexError):
            return None

        target_angle_deg = None
        servo_command_deg = None
        if len(parts) >= 7:
            try:
                parsed_target_angle_deg = float(parts[5])
                parsed_servo_command_deg = float(parts[6])
                if (
                    math.isfinite(parsed_target_angle_deg)
                    and math.isfinite(parsed_servo_command_deg)
                ):
                    target_angle_deg = parsed_target_angle_deg
                    servo_command_deg = parsed_servo_command_deg
            except (ValueError, IndexError):
                pass

        return (
            ch1,
            ch2,
            ch5,
            ch6,
            target_angle_deg,
            servo_command_deg,
        )

    def _valid_us(self, value: int) -> int:
        """ESP validUs() 와 같은 판정. 범위 밖이면 0(손실)으로 표시한다."""
        if self._rc_valid_min_us <= value <= self._rc_valid_max_us:
            return value
        return 0

    # ESP32 ROM 부트로더가 리셋 직후 115200 으로 뱉는 문자열들.
    # CH340 의 DTR/RTS 자동 리셋 회로 때문에 포트를 열 때마다 리셋이 걸린다.
    _ESP_BOOT_MARKERS = (
        "rst:0x",
        "ets ",
        "boot:0x",
        "SPI_FAST_FLASH_BOOT",
        "entry 0x",
        "ESP32 RC + Jetson",
    )

    def _looks_like_esp_boot(self, line: str) -> bool:
        return any(marker in line for marker in self._ESP_BOOT_MARKERS)

    def _note_esp_reboot(self, line: str) -> None:
        """ESP 재부팅 감지. 서보는 90도로 돌아갔고 PPM 은 재동기 전이다."""
        now = time.time()
        self._esp_boot_time = now
        # 부팅 직후 값은 신뢰할 수 없다. RC 를 손실로 떨어뜨려 duty 를 끊는다.
        self._rc_ch1 = 0
        self._rc_ch2 = 0
        self._rc_ch5 = 0
        self._rc_ch6 = 0
        self._last_rc_time = 0.0
        self._mode_auto_latched = False
        if now - self._esp_boot_log_time > 1.0:
            self._esp_boot_log_time = now
            self._esp_boot_count += 1
            self.get_logger().warn(
                f"ESP32 재부팅 감지 (#{self._esp_boot_count}): {line.strip()[:60]!r} "
                "— 서보 중앙 복귀, PPM 재동기 대기. MANUAL 강제, duty 0"
            )

    def _note_unparsed_esp_line(self, line: str) -> None:
        text = line.strip()
        if not text:
            return
        if self._looks_like_esp_boot(text):
            self._note_esp_reboot(text)
            return
        self._esp_unparsed_count += 1
        now = time.time()
        if now - self._esp_unparsed_log_time > 5.0:
            self._esp_unparsed_log_time = now
            self.get_logger().warn(
                f"ESP 시리얼 해석 불가 (누적 {self._esp_unparsed_count}): {text[:60]!r}"
            )

    def _pulse_esp_reset(self) -> None:
        """DTR/RTS 로 ESP32 를 하드 리셋한다.

        CH340 자동 리셋 회로는 RTS 가 EN, DTR 이 GPIO0 에 물려 있다.
        GPIO0 를 놓아둔 채 EN 만 잠깐 내렸다 올리면 일반 부팅으로 재시작한다.
        """
        self.esp.dtr = False
        self.esp.rts = True
        time.sleep(0.05)
        self.esp.rts = False
        self.esp.reset_input_buffer()
        self._esp_rx_buffer.clear()

    def _esp_ppm_recovery_check(self, now: float) -> None:
        """PPM 락에 실패한 ESP 를 다시 리셋해서 되살린다.

        ESP 는 리셋 후 약 1/3 확률로 PPM 캡처가 살아나지 못하고, 한 번 그렇게
        되면 스스로 복구되지 않는다 (텔레메트리는 계속 RC,0,0,0,0). 젯슨이 USB 를
        열 때마다 CH340 이 ESP 를 리셋시키므로 주행 시작 때마다 재현된다.

        펌웨어 쪽 재무장 워치독이 올라가기 전까지의 우회책이다. 리셋을 다시
        걸면 다음 시도에서 락이 걸릴 확률이 그만큼 생긴다.
        """
        if self._esp_recovery_tries >= self._esp_recovery_max_tries:
            return
        if self._esp_booting(now):
            return
        # 링크 자체가 죽었으면(텔레메트리 없음) 리셋해도 의미가 없다.
        if self.last_esp32_packet_time is None:
            return
        if now - self.last_esp32_packet_time > 0.5:
            return
        # RC 가 살아 있으면 할 일이 없다.
        if self._rc_fresh(now):
            self._esp_recovery_tries = 0
            self._esp_rc_dead_since = 0.0
            return
        # 움직이는 중에는 건드리지 않는다.
        if abs(self._measured_speed_mps) > 0.05:
            return

        if self._esp_rc_dead_since <= 0.0:
            self._esp_rc_dead_since = now
            return
        if now - self._esp_rc_dead_since < self._esp_recovery_wait_sec:
            return

        self._esp_recovery_tries += 1
        self._esp_rc_dead_since = 0.0
        self.get_logger().warn(
            f"ESP PPM 락 실패 — ESP 재리셋 시도 "
            f"{self._esp_recovery_tries}/{self._esp_recovery_max_tries}"
        )
        try:
            self._pulse_esp_reset()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"ESP 재리셋 실패: {exc}")

    def _esp_booting(self, now: float) -> bool:
        """부팅 직후 PPM 재동기 구간. 이 동안은 무조건 안전 상태로 둔다."""
        if self._esp_boot_time <= 0.0:
            return False
        if now - self._esp_boot_time <= self._esp_boot_settle_sec:
            return True
        self._esp_boot_time = 0.0
        return False

    # 조향 체인의 실제 배율. Stanley 게인 재환산과 아래 분모가 상쇄된 뒤
    # 남는 값으로, "운동학적으로 정확한 조향의 몇 배를 내보내는가" 를 뜻한다.
    # 1.0 = 요구한 전륜각이 그대로 나간다.
    _STEER_DELIVERY_TUNED = 1.0

    def _warn_if_steer_scale_mismatch(self) -> None:
        """조향 체인의 **실효 배율**이 1.0 에서 벗어나면 경고한다.

        (보정ON + 0.3735) 와 (보정OFF + 0.8727) 은 물리 출력이 같고 둘 다
        1.0 이다. 반대로 (보정ON + 0.8727) 은 0.428 로, 요구 조향의 43% 만
        나간다. 값이 두 파일에 나뉘어 있어서 한쪽만 옮기면 이렇게 어긋난다.

        그래서 비교해야 할 것은 분모 자체가 아니라 상쇄 후 남는 실효 배율이다.
        """
        try:
            from path_following.stanley_waypoint_follow_node import (  # noqa: PLC0415
                CFG as STANLEY_CFG,
            )
        except Exception:  # 패키지 구성이 달라도 제어는 계속돼야 한다
            return

        servo_full = float(STANLEY_CFG.get("max_steering_angle", 0.0))
        if servo_full <= 1e-6:
            return
        rebase = 1.0
        if STANLEY_CFG.get("steer_scale_calibrated", False):
            real = float(STANLEY_CFG.get("max_steering_angle_real_rad", 0.0))
            if real <= 1e-6:
                return
            rebase = real / servo_full
        # 분모까지 반영한 실효 배율을 "운동학 정확 = 1.0" 기준으로 환산한다.
        delivery = (rebase / self._max_steering_angle_rad) * servo_full

        self.get_logger().info(
            f"조향 실효 배율 {delivery:.3f} "
            f"(게인재환산 {rebase:.3f} / 분모 {self._max_steering_angle_rad:.4f})"
        )
        if abs(delivery - self._STEER_DELIVERY_TUNED) <= 0.02:
            return
        self.get_logger().error(
            f"조향 실효 배율이 튜닝값에서 벗어났다: "
            f"{delivery:.3f} vs {self._STEER_DELIVERY_TUNED:.3f} "
            f"(조향이 {delivery / self._STEER_DELIVERY_TUNED:.2f}배). "
            f"Stanley 게인과 control_node 분모를 같이 옮겼는지 확인할 것."
        )

    def _read_esp_rc(self) -> None:
        waiting = self.esp.in_waiting
        if waiting <= 0:
            return

        self._esp_rx_buffer.extend(self.esp.read(waiting))
        if len(self._esp_rx_buffer) > 512:
            del self._esp_rx_buffer[:-512]

        while True:
            nl = self._esp_rx_buffer.find(b"\n")
            if nl < 0:
                break
            line = self._esp_rx_buffer[:nl].decode(errors="ignore")
            del self._esp_rx_buffer[: nl + 1]
            parsed = self._parse_rc_line(line)
            if parsed is None:
                self._note_unparsed_esp_line(line)
                continue
            (
                ch1,
                ch2,
                ch5,
                ch6,
                target_angle_deg,
                servo_command_deg,
            ) = parsed
            packet_time = time.time()
            # ESP 는 PPM 을 잃으면 RC,0,0,0,0 을 계속 보낸다. 범위 검증 없이 받으면
            # 이 프레임이 '최신 정상 RC' 로 취급돼서, 조종기가 끊겨도 AUTO 가 유지되고
            # VESC duty 가 계속 나간다. 값이 깨진 프레임도 같은 경로로 모드를 뒤집었다.
            ch1 = self._valid_us(ch1)
            ch2 = self._valid_us(ch2)
            ch5 = self._valid_us(ch5)
            ch6 = self._valid_us(ch6)

            self._rc_ch1 = ch1
            self._rc_ch2 = ch2
            self._rc_ch5 = ch5
            self._rc_ch6 = ch6
            self.last_esp32_packet_time = packet_time

            # 링크(USB)는 살아 있어도 조종기 신호가 없으면 RC 는 stale 로 둔다.
            rc_link_ok = ch5 > 0 or ch1 > 0 or ch2 > 0 or ch6 > 0
            if rc_link_ok:
                self._last_rc_time = packet_time
                if not self._rc_signal_ok:
                    self._rc_signal_ok = True
                    self.get_logger().info("RC 신호 복구 — 조종기 프레임 정상")
            elif self._rc_signal_ok:
                self._rc_signal_ok = False
                self.get_logger().warn(
                    "RC 신호 손실 (ESP PPM failsafe) — MANUAL 강제, duty 0"
                )

            if target_angle_deg is not None and servo_command_deg is not None:
                self.esp32_target_angle_deg = target_angle_deg
                self.esp32_servo_command_deg = servo_command_deg
                self.last_esp32_steering_time = packet_time

                target_msg = Float32()
                target_msg.data = float(target_angle_deg)
                self.esp32_target_angle_pub.publish(target_msg)

                servo_msg = Float32()
                servo_msg.data = float(servo_command_deg)
                self.esp32_servo_command_pub.publish(servo_msg)

            if ch6 > 0:
                if ch6 >= self._ch6_estop_us:
                    self._set_estop_latched(True, source="rc")
                elif ch6 <= self._ch6_estop_release_us:
                    self._set_estop_latched(False, source="rc")

    def _rc_fresh(self, now: float | None = None) -> bool:
        now = time.time() if now is None else now
        if self._esp_booting(now):
            return False
        if self._last_rc_time <= 0.0:
            return False
        if self._rc_timeout <= 0.0:
            return True
        return (now - self._last_rc_time) <= self._rc_timeout

    def _is_autonomous_mode(self) -> bool:
        # RC 가 끊기면 AUTO 를 유지하지 않는다. 예전에는 latch 를 그대로 돌려줘서
        # 조종기가 죽어도 자율 duty 가 계속 나갔다.
        if not self._rc_fresh():
            self._mode_auto_latched = False
            return False
        ch5 = self._rc_ch5
        if ch5 <= 0:
            # 프레임 한두 개가 깨진 경우. 위 stale 검사가 지속 손실을 잡으므로
            # 여기서는 직전 모드를 유지해 순간 글리치로 모드가 튀지 않게 한다.
            return self._mode_auto_latched
        if ch5 <= self._ch5_manual_us:
            self._mode_auto_latched = False
            return False
        if ch5 >= self._ch5_auto_us:
            self._mode_auto_latched = True
            return True
        return self._mode_auto_latched

    def _rc_ch2_to_duty(self, ch2: int) -> float:
        if ch2 <= 0:
            return 0.0
        # ESP raw us (PPM index 1)
        if 800 <= ch2 <= 2200:
            center = self._rc_center_ch2
            error = ch2 - center
            if abs(error) < self._rc_deadzone:
                return 0.0
            duty = (error / 500.0) * self._max_duty
            if self._invert_rc_throttle:
                duty = -duty
            return self.clamp(duty, -self._max_duty, self._max_duty)

        # legacy 0..3000 scale fallback
        ch2 = int(self.clamp(float(ch2), self._rc_min_val, self._rc_max_val))
        error = ch2 - self._rc_center_ch2
        if abs(error) < self._rc_deadzone:
            return 0.0

        if error > 0:
            span = max(self._rc_max_val - self._rc_center_ch2, 1)
        else:
            span = max(self._rc_center_ch2 - self._rc_min_val, 1)

        duty = (error / span) * self._max_duty
        if self._invert_rc_throttle:
            duty = -duty
        return self.clamp(duty, -self._max_duty, self._max_duty)

    def _slew_auto_duty(self, target: float, dt: float) -> float:
        """AUTO duty를 ramp_sec 동안 선형으로 목표까지 올림/내림."""
        if self._auto_duty_ramp_sec <= 0.0:
            return target

        dt = self.clamp(dt, 1e-4, 0.1)
        max_step = (self._max_duty / self._auto_duty_ramp_sec) * dt
        diff = target - self._auto_duty_applied
        if abs(diff) <= max_step:
            return target
        return self._auto_duty_applied + math.copysign(max_step, diff)

    def _compute_speed_from_erpm(self, erpm: float) -> float:
        motor_rpm = float(erpm) / self._pole_pairs
        wheel_rpm = motor_rpm / self._gear_ratio
        raw_speed_mps = wheel_rpm / 60.0 * math.pi * self._wheel_diameter
        if self._invert_speed_sign:
            return -raw_speed_mps
        return raw_speed_mps

    def _reset_speed_controller(self) -> None:
        self._previous_auto_target_speed_mps = 0.0
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
        self._auto_duty = 0.0
        self._auto_duty_applied = 0.0
        self._last_auto_duty_cmd = 0.0

    def _reset_auto_steer(self) -> None:
        self._auto_steer = 0.0
        self._auto_steer_applied = 0.0

    def _apply_duty_rate_limit(self, target_duty: float, dt: float) -> float:
        if self._duty_rate_limit_per_sec <= 0.0:
            return target_duty
        dt = self.clamp(dt, 1e-4, 0.1)
        max_step = self._duty_rate_limit_per_sec * dt
        diff = target_duty - self._last_auto_duty_cmd
        if abs(diff) <= max_step:
            return target_duty
        return self._last_auto_duty_cmd + math.copysign(max_step, diff)

    def _apply_manual_duty_rate_limit(self, target_duty: float, dt: float) -> float:
        dt = self.clamp(dt, 1e-4, 0.1)
        target_duty = self.clamp(target_duty, -self._max_duty, self._max_duty)

        if self._manual_duty_rise_rate <= 0.0 and self._manual_duty_fall_rate <= 0.0:
            self._manual_duty_applied = target_duty
            return target_duty

        diff = target_duty - self._manual_duty_applied
        if diff > 0.0:
            max_step = self._manual_duty_rise_rate * dt
        elif diff < 0.0:
            max_step = self._manual_duty_fall_rate * dt
        else:
            return self._manual_duty_applied

        if abs(diff) <= max_step:
            self._manual_duty_applied = target_duty
        else:
            self._manual_duty_applied += math.copysign(max_step, diff)

        return self._manual_duty_applied

    def _apply_steer_rate_limit(self, target_steer: float, dt: float) -> float:
        target_steer = self.clamp(target_steer, -self._max_steer, self._max_steer)
        if self._steer_rate_limit_per_sec <= 0.0:
            self._auto_steer_applied = target_steer
            return target_steer

        dt = self.clamp(dt, 1e-4, 0.1)
        max_step = self._steer_rate_limit_per_sec * dt
        diff = target_steer - self._auto_steer_applied
        if abs(diff) <= max_step:
            self._auto_steer_applied = target_steer
        else:
            self._auto_steer_applied += math.copysign(max_step, diff)
        return self._auto_steer_applied

    def _update_auto_speed_controller(
        self, target_speed: float, measured_speed: float, dt: float
    ) -> float:
        target_speed = self.clamp(target_speed, 0.0, self._max_target_speed_mps)
        dt = self.clamp(dt, 1e-4, 0.1)

        if target_speed <= 1e-6:
            self._reset_speed_controller()
            return 0.0

        self._target_decrease_detected = (
            self._previous_auto_target_speed_mps - target_speed
            > self._target_change_threshold_mps
        )
        if self._target_decrease_detected:
            self._speed_integral_duty *= self._target_decrease_i_scale
        self._previous_auto_target_speed_mps = target_speed

        self._speed_error = target_speed - measured_speed
        if target_speed <= self._min_move_speed_mps:
            self._speed_ff_term = 0.0
        else:
            self._speed_ff_term = self._speed_ff_duty_per_mps * target_speed

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
        self._auto_duty = duty_cmd
        self._auto_duty_applied = duty_cmd
        return duty_cmd

    def _request_vesc_values(self) -> None:
        payload = bytearray([4])  # COMM_GET_VALUES
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
        if not payload or payload[0] != 4:  # COMM_GET_VALUES response
            return
        if len(payload) < 29:
            return

        try:
            current_motor = struct.unpack(">i", payload[5:9])[0] / 100.0
            current_in = struct.unpack(">i", payload[9:13])[0] / 100.0
            duty_feedback = struct.unpack(">h", payload[21:23])[0] / 1000.0
            erpm = float(struct.unpack(">i", payload[23:27])[0])
            input_voltage = struct.unpack(">h", payload[27:29])[0] / 10.0
        except struct.error:
            return

        self._erpm = erpm
        self._current_motor = current_motor
        self._current_in = current_in
        self._input_voltage = input_voltage
        self._vesc_duty_feedback = duty_feedback
        self._measured_speed_mps = self._compute_speed_from_erpm(erpm)
        self._last_vesc_telemetry_time = now

    def _vesc_telemetry_fresh(self, now: float) -> bool:
        return (
            self._last_vesc_telemetry_time > 0.0
            and now - self._last_vesc_telemetry_time <= self._vesc_telemetry_timeout
        )

    def timer_callback(self) -> None:
        now = time.time()
        dt = now - self._last_timer_time
        self._last_timer_time = now

        self._read_esp_rc()
        self._esp_ppm_recovery_check(now)
        self._poll_vesc_telemetry(now)
        autonomous = self._is_autonomous_mode()
        self._control_mode = "AUTO" if autonomous else "MANUAL"

        aeb = (
            autonomous
            and not self._is_estop_latched()
            and self._emergency_brake_requested(now)
        )
        if aeb != self._emergency_brake_engaged:
            self._emergency_brake_engaged = aeb
            if aeb:
                self.get_logger().warn(
                    f"AEB 제동 — v={self._measured_speed_mps:.2f} m/s"
                )
            else:
                self.get_logger().info("AEB 해제 — 정상 주행 복귀")

        reversing = (
            autonomous
            and not self._is_estop_latched()
            and not aeb
            and self._escape_reverse_requested(now)
        )
        if reversing != self._escape_reverse_engaged:
            self._escape_reverse_engaged = reversing
            if reversing:
                self.get_logger().warn("후진 탈출 — 조향 중립으로 곧게 물러난다")
            else:
                self.get_logger().info("후진 탈출 종료")

        if self._is_estop_latched():
            self.current_duty = 0.0
            self.current_steer = 0.0
            self._reset_speed_controller()
            self._reset_auto_steer()
        elif aeb:
            # 조향은 마지막 명령을 유지한다 (직진으로 되돌리면 오히려 위험).
            self._manual_drive_speed_active = False
            self.current_duty = self._emergency_brake_output_duty()
            self.current_steer = self._apply_steer_rate_limit(self._auto_steer, dt)
            self._reset_speed_controller()
            self.send_steering(self.current_steer)
        elif reversing:
            # 제동과 반대로 조향을 중립으로 되돌린다. 꺾인 채 물러나면 뒤가
            # 어디로 갈지 예측이 안 되는데, 뒤 여유는 곧게 간다는 가정으로
            # 쟀다. 한 번에 꺾지 않고 평소 레이트 제한을 그대로 태운다.
            self._manual_drive_speed_active = False
            self.current_duty = self._escape_reverse_output_duty()
            self.current_steer = self._apply_steer_rate_limit(0.0, dt)
            self._reset_speed_controller()
            self.send_steering(self.current_steer)
        elif autonomous:
            self._manual_drive_speed_active = False
            if now - self.last_cmd_time > self._cmd_timeout:
                self._reset_speed_controller()
                self._reset_auto_steer()
                self.current_steer = 0.0
                self.current_duty = 0.0
            else:
                target_speed = self.clamp(
                    self._target_speed_mps, 0.0, self._max_target_speed_mps
                )
                self.current_steer = self._apply_steer_rate_limit(self._auto_steer, dt)
                if target_speed <= 1e-6:
                    self.current_duty = 0.0
                    self._reset_speed_controller()
                elif not self._vesc_telemetry_fresh(now):
                    self.current_duty = 0.0
                    self._reset_speed_controller()
                else:
                    self.current_duty = self._update_auto_speed_controller(
                        target_speed, self._measured_speed_mps, dt
                    )
            self.send_steering(self.current_steer)
        else:
            self._reset_auto_steer()
            self._reset_speed_controller()
            target_duty = (
                self._rc_ch2_to_duty(self._rc_ch2) if self._rc_fresh(now) else 0.0
            )
            self.current_duty = self._apply_manual_duty_rate_limit(target_duty, dt)
            self.current_steer = 0.0

        self.set_vesc_duty(self.current_duty)
        self._publish_telemetry(autonomous)
        self._publish_speed()
        self._maybe_log_status(dt)

    def _maybe_log_status(self, dt: float) -> None:
        if self._status_log_period <= 0.0:
            return
        self._status_log_accum += max(0.0, dt)
        if self._status_log_accum < self._status_log_period:
            return
        self._status_log_accum = 0.0

        v_act = self._measured_speed_mps
        vesc_fresh = self._vesc_telemetry_fresh(time.time())

        if self._control_mode == "AUTO":
            v_tgt = self.clamp(
                self._target_speed_mps, 0.0, self._max_target_speed_mps
            )
            speed_part = (
                f"v_tgt={v_tgt:.2f} v_act={v_act:.2f} "
                f"err={self._speed_error:+.2f} "
                f"ff={self._speed_ff_term:+.3f} "
                f"p={self._speed_p_term:+.3f} i={self._speed_i_term:+.3f} "
                f"raw={self._raw_speed_duty_cmd:+.3f} lim={self._limited_speed_duty_cmd:+.3f}"
            )
        else:
            speed_part = f"v_act={v_act:.2f} | RC CH2"

        duty_vesc = (
            f" duty_vesc={self._vesc_duty_feedback:+.3f}"
            if vesc_fresh
            else " duty_vesc=—"
        )
        # VESC 가 0.1 V 단위로 주므로 소수 한 자리가 곧 분해능이다.
        batt = f"{self._input_voltage:.1f}V" if vesc_fresh else "—"
        if (
            self.esp32_target_angle_deg is not None
            and self.esp32_servo_command_deg is not None
            and self.last_esp32_steering_time is not None
        ):
            esp32_steering_age_ms = max(
                0.0, (time.time() - self.last_esp32_steering_time) * 1000.0
            )
            esp32_steering = (
                f"ESP32 RC: ch1={self._rc_ch1} ch2={self._rc_ch2} "
                f"ch5={self._rc_ch5} ch6={self._rc_ch6} "
                f"target_angle_deg={self.esp32_target_angle_deg:.2f} "
                f"servo_command_deg={self.esp32_servo_command_deg:.2f} "
                f"steering_age_ms={esp32_steering_age_ms:.0f}"
            )
        else:
            esp32_steering = (
                f"ESP32 RC: ch1={self._rc_ch1} ch2={self._rc_ch2} "
                f"ch5={self._rc_ch5} ch6={self._rc_ch6} "
                "target_angle_deg=— servo_command_deg=— steering_age_ms=—"
            )

        self.get_logger().info(
            f"STATUS | {self._control_mode} | "
            f"duty={self.current_duty:+.3f}{duty_vesc} | "
            f"{speed_part} m/s | "
            f"erpm={self._erpm:.0f} batt={batt} CH5={self._rc_ch5} | "
            f"{esp32_steering}"
        )

    def _publish_telemetry(self, autonomous: bool) -> None:
        msg = Float64MultiArray()
        msg.data = [
            float(self._last_speed_mps),
            float(self._last_steering_rad),
            float(self.current_duty),
            float(self._auto_duty),
            float(self.current_steer),
            float(self._rc_ch5),
            1.0 if autonomous else 0.0,
            1.0 if self._is_estop_latched() else 0.0,
            float(self._rc_ch1),
            float(self._rc_ch2),
            float(self._measured_speed_mps),
            float(self._target_speed_mps),
            float(self._speed_error),
            0.0,
            float(self._speed_duty_cmd),
            float(self._erpm),
            float(self._current_in),
            float(self._current_motor),
            float(self._input_voltage),
            float(self._speed_p_term),
            float(self._speed_i_term),
            float(self._raw_speed_duty_cmd),
            float(self._limited_speed_duty_cmd),
            1.0 if self._auto_duty_limiter_active else 0.0,
            float(self._max_auto_duty),
            float(self._vesc_duty_now),
            float(self._target_speed_mps),
            float(self._measured_speed_mps),
            float(self._speed_error),
            float(self._speed_ff_term),
            float(self._speed_p_term),
            float(self._speed_i_term),
            float(self._raw_speed_duty_cmd),
            float(self._limited_speed_duty_cmd),
            float(self._speed_duty_cmd),
            1.0 if self._auto_duty_limiter_active else 0.0,
            1.0 if self._i_integration_active else 0.0,
            1.0 if self._target_decrease_detected else 0.0,
        ]
        self._telemetry_pub.publish(msg)

    def _publish_speed(self) -> None:
        msg = Float64()
        msg.data = float(self._measured_speed_mps)
        self._speed_pub.publish(msg)

    def send_steering(self, steer: float) -> None:
        steer = self.clamp(steer, -self._max_steer, self._max_steer)
        if self._steer_cmd_format == "prefixed":
            line = f"S:{steer:.3f}\n"
        else:
            # jetson_steer_send.py 와 동일: 숫자만 + 줄바꿈
            line = f"{steer:.3f}\n"
        self.esp.write(line.encode())

    def set_vesc_duty(self, duty: float) -> None:
        duty_limit = max(abs(self._max_duty), abs(self._max_auto_duty))
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

    def stop_vehicle(self) -> None:
        self.set_vesc_duty(0.0)
        self.send_steering(0.0)

    @staticmethod
    def make_vesc_packet(payload: bytearray) -> bytearray:
        packet = bytearray()
        packet.append(0x02)
        packet.append(len(payload))
        packet.extend(payload)

        crc = VehicleControlNode.crc16_ccitt(payload)
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
        self.get_logger().info("Stopping vehicle...")
        self._reset_speed_controller()
        try:
            self.stop_vehicle()
            time.sleep(0.1)
            self.stop_vehicle()
        except Exception as e:
            self.get_logger().error(f"Stop failed: {e}")

        try:
            self.esp.close()
            self.vesc.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleControlNode()
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
