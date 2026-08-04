#!/usr/bin/env bash
# control_node 를 패스 코어(5~6)에 묶어서 실행. 종료 후에도 정책 스크립트가 nice 를 맞춤.
set -euo pipefail
ROOT="/home/nvidia/f1tenth_ajou"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"
# 백그라운드로 정책 한 번 적용 (control 기동 직후)
(sleep 1; bash "${ROOT}/scripts/apply_cpu_policy.sh" --once >/tmp/f1tenth_cpu_policy_control.log 2>&1 || true) &
exec taskset -c 4-5 ros2 run path_following control_node "$@"
