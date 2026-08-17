#!/usr/bin/env bash
# Called by systemd at boot — allow nvidia to use :0 from SSH (even on GDM greeter).
set -euo pipefail

TARGET_USER="${1:-nvidia}"
sleep 2

[[ -S /tmp/.X11-unix/X0 ]] || exit 0

for auth in /run/user/*/gdm/Xauthority /run/user/*/.Xauthority; do
  [[ -f "${auth}" ]] || continue
  if DISPLAY=:0 XAUTHORITY="${auth}" xhost +SI:localuser:"${TARGET_USER}" 2>/dev/null; then
    logger -t rviz-xhost "allowed localuser:${TARGET_USER} via ${auth}"
    exit 0
  fi
done

logger -t rviz-xhost "failed to allow localuser:${TARGET_USER}"
exit 1
