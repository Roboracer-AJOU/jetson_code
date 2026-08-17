#!/usr/bin/env bash
# f1tenth_ajou Python + ROS apt 의존성 일괄 설치
# 사용: sudo bash scripts/install_python_deps.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "root 권한이 필요합니다:"
  echo "  sudo bash $0"
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

echo "=== apt update ==="
apt-get update -y

echo "=== Python / ROS 패키지 ==="
# scipy/skimage 는 apt 로 깐다. `pip install scikit-image` 는 numpy 2.x 를
# ~/.local 에 끌고 들어와 시스템 numpy 1.21 을 가리고, 그 ABI 로 빌드된
# scipy 가 "numpy.core.multiarray failed to import" 로 죽는다.
apt-get install -y \
  python3-serial \
  python3-pip \
  python3-pytest \
  python3-numpy \
  python3-scipy \
  python3-skimage \
  python3-yaml \
  python3-pil \
  ros-humble-ackermann-msgs

echo "=== rosdep 초기화 (최초 1회) ==="
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init || true
fi
sudo -u nvidia bash -lc 'source /opt/ros/humble/setup.bash && rosdep update'

echo "=== 워크스페이스 rosdep 설치 ==="
cd /home/nvidia/f1tenth_ajou
sudo -u nvidia bash -lc '
  source /opt/ros/humble/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
'

echo "=== import 확인 ==="
sudo -u nvidia python3 - <<'PY'
import serial
import numpy
import scipy.ndimage
import yaml
from PIL import Image
from skimage.morphology import skeletonize
print(f"OK: serial, numpy {numpy.__version__}, scipy, skimage, yaml, PIL")
PY

echo
echo "설치 완료. 다음:"
echo "  cd /home/nvidia/f1tenth_ajou"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --symlink-install"
