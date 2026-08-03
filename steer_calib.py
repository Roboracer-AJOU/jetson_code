#!/usr/bin/env python3
"""앞바퀴 조향각 실측용 — ESP32 에 S: 명령을 직접 보내고 그 값을 유지한다.

control_node 를 끄고 실행할 것 (같은 시리얼 포트를 점유한다).
VESC 는 건드리지 않으므로 구동 모터는 절대 돌지 않는다.

각 단계에서 앞바퀴 실제 각도를 각도기로 재고 Enter 로 다음 단계로 넘어간다.
"""
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial 이 없습니다: pip3 install pyserial")

PORT = "/dev/ttyTHS1"
BAUD = 115200
SEND_HZ = 20.0
FULL_SCALE_DEG = 40.0  # S:1.0 이 뜻한다고 소프트웨어가 믿는 앞바퀴 각도

STEPS = [0.0, 0.25, 0.50, 0.75, 1.00, 0.0, -0.25, -0.50, -0.75, -1.00, 0.0]


class SteerHolder:
    def __init__(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.value = 0.0
        self.servo_deg = None
        self.target_deg = None
        self._stop = threading.Event()
        self._tx = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)

    def start(self) -> None:
        self._tx.start()
        self._rx.start()

    def stop(self) -> None:
        self._stop.set()
        time.sleep(0.15)
        try:
            self.ser.write(b"S:0.000\n")
            self.ser.flush()
        except Exception:
            pass
        self.ser.close()

    def _tx_loop(self) -> None:
        period = 1.0 / SEND_HZ
        while not self._stop.is_set():
            try:
                self.ser.write(f"S:{self.value:.3f}\n".encode())
            except Exception as exc:
                print(f"\n[송신 오류] {exc}")
                return
            time.sleep(period)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                return
            if not raw.startswith("RC,"):
                continue
            parts = raw.split(",")
            if len(parts) < 7:
                continue
            try:
                self.target_deg = float(parts[5])
                self.servo_deg = float(parts[6])
            except ValueError:
                continue


def main() -> None:
    print(__doc__)
    print(f"포트 {PORT} @ {BAUD}\n")
    print("차량을 들어올려 앞바퀴가 바닥에 닿지 않게 한 뒤 시작하세요.")
    input("준비되면 Enter: ")

    try:
        holder = SteerHolder(PORT, BAUD)
    except serial.SerialException as exc:
        sys.exit(f"포트를 열 수 없습니다: {exc}\ncontrol_node 가 떠 있으면 먼저 끄세요.")

    holder.start()
    results = []

    try:
        for s in STEPS:
            holder.value = s
            time.sleep(0.6)  # ESP 스무딩 + 서보 이동 대기
            expect = s * FULL_SCALE_DEG
            servo = holder.servo_deg
            servo_txt = f"{servo:.0f}°" if servo is not None else "수신없음"
            print(
                f"\n  S:{s:+.3f}  →  소프트웨어 기대 바퀴각 {expect:+.1f}°  "
                f"|  ESP 서보각 {servo_txt}"
            )
            ans = input("    실측 바퀴각(도, 부호 포함) 입력 후 Enter [건너뛰려면 그냥 Enter]: ").strip()
            if ans:
                try:
                    results.append((s, expect, float(ans)))
                except ValueError:
                    print("    숫자가 아니라 건너뜁니다.")
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        holder.value = 0.0
        time.sleep(0.3)
        holder.stop()

    if not results:
        print("\n입력된 측정값이 없습니다.")
        return

    print("\n=== 결과 ===")
    print("  S 값     기대각     실측각    전달비율")
    ratios = []
    for s, expect, meas in results:
        if abs(expect) < 1e-6:
            print(f"  {s:+.2f}   {expect:+7.1f}°  {meas:+7.1f}°       (중립)")
            continue
        r = meas / expect
        ratios.append(r)
        print(f"  {s:+.2f}   {expect:+7.1f}°  {meas:+7.1f}°     {r:6.3f}")

    if ratios:
        avg = sum(ratios) / len(ratios)
        print(f"\n평균 전달비율 = {avg:.3f}")
        print(f"실제 최대 조향각 ≈ {avg * FULL_SCALE_DEG:.1f}°")
        print(f"→ max_steering_angle_rad 권장값 = {avg * FULL_SCALE_DEG * 3.14159265 / 180.0:.4f} rad")


if __name__ == "__main__":
    main()
