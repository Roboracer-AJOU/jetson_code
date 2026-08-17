#!/usr/bin/env bash
# Jetson Orin NX (Ubuntu 22.04 / aarch64) — ROS 2 Humble + Cartographer
# 사용: bash scripts/install_ros2_humble.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "root 권한이 필요합니다. 다시 실행:"
  echo "  sudo bash $0"
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive
export LANG=en_US.UTF-8

echo "=== locale ==="
apt-get update -y
apt-get install -y locales curl gnupg lsb-release software-properties-common ca-certificates
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "=== ROS 2 apt repo ==="
mkdir -p /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  > /etc/apt/sources.list.d/ros2.list

apt-get update -y

echo "=== ROS Humble + Cartographer ==="
apt-get install -y \
  ros-humble-ros-base \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-cartographer-ros-msgs \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete

USER_HOME="$(getent passwd nvidia | cut -d: -f6)"
BASHRC="${USER_HOME}/.bashrc"
if [[ -f "${BASHRC}" ]] && ! grep -q 'source /opt/ros/humble/setup.bash' "${BASHRC}"; then
  cat >> "${BASHRC}" <<'EOF'

# ROS 2 Humble
source /opt/ros/humble/setup.bash
EOF
  chown nvidia:nvidia "${BASHRC}"
fi

echo
echo "설치 완료."
echo "  ROS_DISTRO=humble"
ls /opt/ros/humble/lib/cartographer_ros | sed 's/^/  cartographer_ros: /'
echo
echo "새 터미널에서 확인:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 pkg list | grep cartographer"
