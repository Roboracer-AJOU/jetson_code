#!/usr/bin/env bash
# CPU 정책 적용 (재시작 후에도 런치/데몬이 이걸 호출)
#
#   로컬(위치추정): 코어1~4 = CPU 0-3
#   패스(경로추종): 코어5~6 = CPU 4-5 + nice 우선순위
#
# 사용:
#   sudo bash scripts/apply_cpu_policy.sh --once
#   bash scripts/apply_cpu_policy.sh --once --affinity-only   # nice 없이 코어만
#   bash scripts/apply_cpu_policy.sh --daemon                 # 5초마다 재적용
set -uo pipefail

AFFINITY_ONLY=0
DAEMON=0
ONCE=1
INTERVAL=5
LOCK=/tmp/f1tenth_cpu_policy.lock

for arg in "$@"; do
  case "$arg" in
    --affinity-only) AFFINITY_ONLY=1 ;;
    --daemon) DAEMON=1; ONCE=0 ;;
    --once) ONCE=1 ;;
    --help|-h)
      sed -n '2,12p' "$0"
      exit 0
      ;;
  esac
done

pin_threads() {
  local cpus="$1" pid="$2"
  taskset -pc "${cpus}" "${pid}" >/dev/null 2>&1 || return 1
  for tid in /proc/"${pid}"/task/*; do
    taskset -pc "${cpus}" "${tid##*/}" >/dev/null 2>&1 || true
  done
  return 0
}

set_nice_all() {
  local n="$1" pid="$2"
  [[ "${AFFINITY_ONLY}" -eq 1 ]] && return 0
  if [[ "${EUID}" -eq 0 ]]; then
    renice -n "${n}" -p "${pid}" >/dev/null 2>&1 || true
    for tid in /proc/"${pid}"/task/*; do
      renice -n "${n}" -p "${tid##*/}" >/dev/null 2>&1 || true
    done
  else
    # root 없으면 우선순위 올리기(음수)는 실패할 수 있음. 내리기만 시도.
    if [[ "${n}" -ge 0 ]]; then
      renice -n "${n}" -p "${pid}" >/dev/null 2>&1 || true
      for tid in /proc/"${pid}"/task/*; do
        renice -n "${n}" -p "${tid##*/}" >/dev/null 2>&1 || true
      done
    elif sudo -n true 2>/dev/null; then
      sudo -n renice -n "${n}" -p "${pid}" >/dev/null 2>&1 || true
      for tid in /proc/"${pid}"/task/*; do
        sudo -n renice -n "${n}" -p "${tid##*/}" >/dev/null 2>&1 || true
      done
    fi
  fi
}

apply_one() {
  local cpus="$1" nice="$2" pid="$3" label="$4"
  [[ -n "${pid}" && -d "/proc/${pid}" ]] || return 0
  pin_threads "${cpus}" "${pid}" || return 0
  set_nice_all "${nice}" "${pid}"
  if [[ "${DAEMON}" -eq 0 ]]; then
    local aff cur
    aff=$(taskset -pc "${pid}" 2>/dev/null | awk -F: '{print $2}' | xargs)
    cur=$(ps -p "${pid}" -o nice= | tr -d ' ')
    echo "OK  aff=${aff} nice=${cur}  ${label}  $(ps -p "${pid}" -o comm=)  pid=${pid}"
  fi
}

pids_matching() {
  # stdout: pid list, one per line. Skip our own script / grep noise.
  local pat="$1"
  pgrep -f "${pat}" 2>/dev/null | while read -r pid; do
    [[ -d "/proc/${pid}" ]] || continue
    local cmd
    cmd=$(tr '\0' ' ' < /proc/"${pid}"/cmdline 2>/dev/null || true)
    echo "${cmd}" | grep -q 'apply_cpu_policy' && continue
    echo "${pid}"
  done
}

apply_policy() {
  # --- 로컬: CPU 0-3, nice 유지(0) ---
  local lp
  for pat in \
    'cartographer_ros/cartographer_node' \
    'sllidar_node' \
    'ebimu_driver' \
    'vesc_wheel_odom\.py' \
    'sensor_static_tf' \
    'static_map_publisher' \
    'localization_initial_pose_setter' \
    'rviz2' \
    'foxglove_bridge'
  do
    while read -r lp; do
      apply_one "0-3" 0 "${lp}" "local"
    done < <(pids_matching "${pat}")
  done
  # localization launch parent
  while read -r lp; do
    apply_one "0-3" 0 "${lp}" "local-launch"
  done < <(pgrep -af 'ros2 launch localization_layer' | awk '/localization_layer/ {print $1}')

  # --- 패스: CPU 4-5 + nice ---
  # control (ros2 run / python .py 둘 다)
  while read -r lp; do
    local cmd
    cmd=$(ps -p "${lp}" -o args= 2>/dev/null || true)
    echo "${cmd}" | grep -q 'ros2 run path_following control_node' && continue
    apply_one "4-5" -15 "${lp}" "control"
  done < <(pids_matching 'path_following/lib/path_following/control_node|python[0-9.]* control_node\.py')

  while read -r lp; do apply_one "4-5" -10 "${lp}" "stanley"; done < <(pids_matching 'stanley_waypoint_follow_node')
  while read -r lp; do apply_one "4-5" -5 "${lp}" "planner"; done < <(pids_matching 'local_planner_node')
  while read -r lp; do apply_one "4-5" 0 "${lp}" "obstacle"; done < <(pids_matching 'integrated_obstacle_node|static_obstacle_node')
  while read -r lp; do apply_one "4-5" 5 "${lp}" "fgm"; done < <(pids_matching 'fgm_node')
  while read -r lp; do apply_one "4-5" 10 "${lp}" "path-launch"; done < <(pgrep -af 'ros2 launch path_following' | awk '/path_follow_/ {print $1}')
  while read -r lp; do apply_one "4-5" 19 "${lp}" "viz"; done < <(pids_matching 'stack_status_node|drive_monitor')
}

run_once() {
  if [[ "${AFFINITY_ONLY}" -eq 0 && "${EUID}" -ne 0 ]]; then
    if sudo -n "${BASH_SOURCE[0]}" --once 2>/dev/null; then
      return
    fi
    if sudo -n true 2>/dev/null; then
      sudo -n bash "${BASH_SOURCE[0]}" --once
      return
    fi
    # sudo 없으면 코어만이라도
    AFFINITY_ONLY=1
    apply_policy
    echo "NOTE: nice(음수)는 sudo 필요. 한 번만: sudo bash scripts/install_cpu_policy_sudoers.sh"
    return
  fi
  apply_policy
}

if [[ "${DAEMON}" -eq 1 ]]; then
  exec 9>"${LOCK}"
  if ! flock -n 9; then
    echo "cpu_policy daemon already running"
    exit 0
  fi
  echo "cpu_policy daemon start (interval=${INTERVAL}s)"
  while true; do
    run_once >/dev/null 2>&1 || true
    sleep "${INTERVAL}"
  done
else
  echo "=== apply_cpu_policy ==="
  run_once
fi
