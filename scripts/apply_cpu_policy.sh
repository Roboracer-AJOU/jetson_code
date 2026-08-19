#!/usr/bin/env bash
# CPU 정책 적용 (재시작 후에도 런치/데몬이 이걸 호출)
#
#   로컬 처리(위치추정): CPU 0-4  — cartographer/odom/map/rviz, 스케줄러가 분배
#   로컬 센서:           CPU 5    — LiDAR/IMU 드라이버. 패스와 공유
#   패스(경로추종):      CPU 5-7  + nice 우선순위
#
# 8코어 전제다. nvpmodel 15W 는 CPU 4-7 을 오프라인으로 내리므로 MAXN 이어야 한다:
#   sudo nvpmodel -m 0 && sudo jetson_clocks
# 코어가 4개뿐이면 taskset 이 실패하고 커널 기본 스케줄링으로 돌아간다.
#
# 코어 2개일 때 패스 그룹이 195/200% 까지 차서 5번을 공유하도록 넓혔다. 로컬
# 처리(0-4)와는 여전히 분리돼 있지만, 5번에서는 센서 드라이버와 패스 노드가
# 경합한다. 패스 노드 nice 가 음수라 그대로 두면 센서가 밀리므로 센서에도 음수
# nice 를 줘서 control 다음 순위를 지키게 했다. 음수 nice 는 root 가 필요하다:
#   한 번만: sudo bash scripts/install_cpu_policy_sudoers.sh
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
      sed -n '2,20p' "$0"
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
  # --- 로컬 센서: CPU 5 (패스 그룹과 공유) ---
  # 스캔/IMU 주기가 밀리면 그 뒤 스택 전체가 같이 밀린다. 같은 코어에 nice 가
  # 음수인 패스 노드들이 올라오므로, 센서도 -12 를 줘서 control(-15) 바로 다음
  # 순위를 지키게 한다. 드라이버 자체는 가벼워서(합쳐 30% 미만) 나머지를 패스가
  # 그대로 쓴다.
  local lp
  for pat in \
    'sllidar_node' \
    'ebimu_driver'
  do
    while read -r lp; do
      apply_one "5" -12 "${lp}" "sensor"
    done < <(pids_matching "${pat}")
  done

  # --- 로컬 처리 1순위: cartographer (CPU 0-4, nice -10) ---
  # 스캔 매칭이 파이프라인 최상류다. 여기가 밀리면 stanley/control 이 낡은 pose 로
  # 계산하므로 뒤 노드 우선순위를 아무리 높여도 소용이 없다. 실측상 실시간 상관
  # 스캔매처 스레드 하나가 코어 하나를 67% 점유하고 나머지 18개 스레드는 1~2% 라,
  # 코어를 더 줘도 안 풀리고 우선순위로만 지켜줄 수 있다.
  while read -r lp; do
    apply_one "0-4" -10 "${lp}" "cartographer"
  done < <(pids_matching 'cartographer_ros/cartographer_node')

  # --- 나머지 로컬 처리: CPU 0-4, nice 유지(0) ---
  for pat in \
    'vesc_wheel_odom\.py' \
    'sensor_static_tf' \
    'static_map_publisher' \
    'localization_initial_pose_setter' \
    'rviz2' \
    'foxglove_bridge'
  do
    while read -r lp; do
      apply_one "0-4" 0 "${lp}" "local"
    done < <(pids_matching "${pat}")
  done
  # localization launch parent
  while read -r lp; do
    apply_one "0-4" 0 "${lp}" "local-launch"
  done < <(pgrep -af 'ros2 launch localization_layer' | awk '/localization_layer/ {print $1}')

  # --- 패스: CPU 5-7 + nice (5 번은 센서와 공유) ---
  # control (ros2 run / python .py 둘 다)
  while read -r lp; do
    local cmd
    cmd=$(ps -p "${lp}" -o args= 2>/dev/null || true)
    echo "${cmd}" | grep -q 'ros2 run path_following control_node' && continue
    apply_one "5-7" -15 "${lp}" "control"
  done < <(pids_matching 'path_following/lib/path_following/control_node|python[0-9.]* control_node\.py')

  # FGM enable 중이면 FGM을 패스 코어 1순위 (nice -20). 플래그: /tmp/f1tenth_fgm_boost
  local fgm_nice=5
  local fgm_boost=0
  if [[ -f /tmp/f1tenth_fgm_boost ]]; then
    case "$(tr -d '[:space:]' < /tmp/f1tenth_fgm_boost 2>/dev/null || true)" in
      1|true|TRUE|on|ON) fgm_boost=1; fgm_nice=-20 ;;
    esac
  fi

  while read -r lp; do apply_one "5-7" -10 "${lp}" "stanley"; done < <(pids_matching 'stanley_waypoint_follow_node')
  while read -r lp; do apply_one "5-7" -5 "${lp}" "planner"; done < <(pids_matching 'local_planner_node')
  while read -r lp; do apply_one "5-7" -8 "${lp}" "aeb"; done < <(pids_matching 'emergency_brake_node')
  while read -r lp; do apply_one "5-7" 0 "${lp}" "obstacle"; done < <(pids_matching 'integrated_obstacle_node|static_obstacle_node')
  local fgm_label="fgm"
  [[ "${fgm_boost}" -eq 1 ]] && fgm_label="fgm-boost"
  while read -r lp; do
    apply_one "5-7" "${fgm_nice}" "${lp}" "${fgm_label}"
  done < <(pids_matching 'fgm_node')
  while read -r lp; do apply_one "5-7" 10 "${lp}" "path-launch"; done < <(pgrep -af 'ros2 launch path_following' | awk '/path_follow_/ {print $1}')
  while read -r lp; do apply_one "5-7" 19 "${lp}" "viz"; done < <(pids_matching 'stack_status_node|drive_monitor')
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
