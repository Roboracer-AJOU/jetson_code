#!/usr/bin/env bash
# ESP USB-C (CH340) → /dev/ttyUSB*
# 사용: sudo bash /home/nvidia/f1tenth_ajou/scripts/install_ch340.sh
set -u

if [[ "${EUID}" -ne 0 ]]; then
  echo "  sudo bash $0"
  exit 1
fi

KO="$(cd "$(dirname "$0")" && pwd)/ch341.ko"
KVER="$(uname -r)"

echo "=== brltty 끄기 ==="
systemctl stop brltty.service brltty-udev.service 2>/dev/null || true
systemctl disable brltty.service brltty-udev.service 2>/dev/null || true
systemctl mask brltty.service brltty-udev.service 2>/dev/null || true
pkill -9 brltty 2>/dev/null || true

echo "=== ch341 모듈 ==="
if lsmod | awk '{print $1}' | grep -qx ch341; then
  echo "ch341 already loaded"
else
  insmod "${KO}" || modprobe ch341 || true
fi

mkdir -p "/lib/modules/${KVER}/extra"
cp -f "${KO}" "/lib/modules/${KVER}/extra/ch341.ko"
depmod -a "${KVER}" 2>/dev/null || true
echo ch341 > /etc/modules-load.d/ch341.conf

# brltty / ModemManager 가 CH340 를 다시 안 잡게
cat > /etc/udev/rules.d/99-f1tenth-ch340.rules <<'EOF'
ACTION=="add", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{ID_MM_DEVICE_IGNORE}="1"
ACTION=="add", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{BRLTTY}="0"
EOF
udevadm control --reload-rules 2>/dev/null || true

bind_ch340() {
  local dev iface name
  for dev in /sys/bus/usb/devices/*; do
    [[ -f "${dev}/idVendor" ]] || continue
    [[ "$(cat "${dev}/idVendor" 2>/dev/null || true)" == "1a86" ]] || continue
    [[ "$(cat "${dev}/idProduct" 2>/dev/null || true)" == "7523" ]] || continue
    iface="${dev}:1.0"
    [[ -d "${iface}" ]] || continue
    name="$(basename "${iface}")"
    echo "CH340 iface ${name}"
    if [[ -e "/sys/bus/usb/drivers/usbfs/${name}" ]]; then
      echo "  unbind usbfs"
      echo -n "${name}" > /sys/bus/usb/drivers/usbfs/unbind || true
    fi
    if [[ -L "${iface}/driver" ]]; then
      local cur
      cur="$(basename "$(readlink "${iface}/driver")")"
      echo "  current driver=${cur}"
      if [[ "${cur}" != "ch341" ]]; then
        echo -n "${name}" > "/sys/bus/usb/drivers/${cur}/unbind" || true
      fi
    fi
    echo "  bind ch341"
    echo -n "${name}" > /sys/bus/usb/drivers/ch341/bind 2>/dev/null || true
    if [[ -d /sys/bus/usb-serial/drivers/ch341-uart ]]; then
      echo -n "${name}" > /sys/bus/usb-serial/drivers/ch341-uart/bind 2>/dev/null || true
    fi
  done
}

echo "=== bind CH340 ==="
bind_ch340

# IMU CP2102
for dev in /sys/bus/usb/devices/*; do
  [[ -f "${dev}/idVendor" ]] || continue
  [[ "$(cat "${dev}/idVendor" 2>/dev/null || true)" == "10c4" ]] || continue
  iface="${dev}:1.0"
  [[ -d "${iface}" ]] || continue
  name="$(basename "${iface}")"
  if [[ ! -L "${iface}/driver" && -d /sys/bus/usb-serial/drivers/cp210x ]]; then
    echo "bind cp210x ${name}"
    echo -n "${name}" > /sys/bus/usb-serial/drivers/cp210x/bind 2>/dev/null || true
  fi
done

sleep 1
echo
echo "=== /dev/serial/by-id ==="
ls -l /dev/serial/by-id || true
echo "=== ttyUSB/ttyACM ==="
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "(none)"

if ls /dev/serial/by-id/*1a86* >/dev/null 2>&1; then
  echo
  echo "OK: ESP CH340 시리얼 준비됨"
  exit 0
fi
echo
echo "FAIL: CH340 tty 가 안 생겼음. lsusb / dmesg 확인"
lsusb | grep -i 1a86 || true
exit 1
