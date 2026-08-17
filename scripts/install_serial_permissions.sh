#!/usr/bin/env bash
# USB/UART 시리얼 권한 설치 후 재부팅
# 사용: sudo bash /home/nvidia/f1tenth_ajou/scripts/install_serial_permissions.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "  sudo bash $0"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cp "${SCRIPT_DIR}/99-f1tenth-serial.rules" /etc/udev/rules.d/99-f1tenth-serial.rules
chmod 644 /etc/udev/rules.d/99-f1tenth-serial.rules

usermod -aG dialout,tty nvidia

udevadm control --reload-rules || true
udevadm trigger || true

echo "udev 설치 완료. 재부팅합니다."
reboot
