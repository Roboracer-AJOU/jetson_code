#!/usr/bin/env bash
# SSH에서 rviz2 / DISPLAY=:0 사용 전, Jetson X11 권한을 한 번 열어 줍니다.
# (GDM 로그인 화면만 떠 있어도 동작)
set -euo pipefail

REAL_USER="${SUDO_USER:-nvidia}"

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "Run once with sudo:"
  echo "  sudo bash $(readlink -f "$0" 2>/dev/null || echo "$0")"
  exit 1
fi

if [[ ! -S /tmp/.X11-unix/X0 ]]; then
  echo "ERROR: X display :0 is not running. Connect monitor and power on Jetson GUI first."
  exit 1
fi

opened=0
for auth in /run/user/*/gdm/Xauthority; do
  [[ -f "${auth}" ]] || continue
  if DISPLAY=:0 XAUTHORITY="${auth}" xhost +SI:localuser:"${REAL_USER}" 2>/dev/null; then
    echo "[fix_rviz_x11] OK via ${auth}"
    opened=1
    break
  fi
done

if [[ "${opened}" -eq 0 ]]; then
  for auth in /run/user/"$(id -u "${REAL_USER}")"/.Xauthority; do
    [[ -f "${auth}" ]] || continue
    if DISPLAY=:0 XAUTHORITY="${auth}" xhost +SI:localuser:"${REAL_USER}" 2>/dev/null; then
      echo "[fix_rviz_x11] OK via ${auth}"
      opened=1
      break
    fi
  done
fi

if [[ "${opened}" -eq 0 ]]; then
  echo "ERROR: could not open X11 for ${REAL_USER}. Log in at the Jetson monitor once, then retry."
  exit 1
fi

# Quick test as the real user (no cookie file needed after xhost)
if command -v runuser >/dev/null 2>&1; then
  if runuser -u "${REAL_USER}" -- env DISPLAY=:0 xdpyinfo >/dev/null 2>&1; then
    echo "[fix_rviz_x11] xdpyinfo OK — RViz should work now:"
    echo "  unset XAUTHORITY"
    echo "  export DISPLAY=:0"
    echo "  rviz2"
  else
    echo "[fix_rviz_x11] xhost set, but xdpyinfo still failed. Reboot or log in at monitor."
  fi
fi
