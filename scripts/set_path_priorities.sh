#!/usr/bin/env bash
# 패스팔로잉: 코어5~6 고정 + nice 우선순위
# 로컬(위치추정) 프로세스는 절대 건드리지 않음
# 사용: sudo bash scripts/set_path_priorities.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "root 로 실행하세요: sudo bash $0"
  exit 1
fi

# 패스팔로잉으로 볼 커맨드 패턴 (로컬 패턴은 여기 넣지 말 것)
PATH_PATTERNS=(
  'path_following/lib/path_following/control_node'
  'python control_node.py'
  'stanley_waypoint_follow_node'
  'local_planner_node'
  'static_obstacle_node'
  'integrated_obstacle_node'
  'fgm_node'
  'drive_monitor'
  'stack_status_node'
  'ros2 launch path_following'
)

is_local_process() {
  local cmd="$1"
  echo "$cmd" | grep -Eqi \
    'cartographer_node|sllidar_node|ebimu_driver|vesc_wheel_odom|sensor_static_tf|static_map_publisher|localization_initial|rviz2|foxglove|localization_layer'
}

pin_and_nice() {
  local n="$1" pid="$2" label="$3"
  if [[ -z "${pid}" || ! -d "/proc/${pid}" ]]; then
    echo "SKIP ${label}: 프로세스 없음"
    return 0
  fi
  local cmd
  cmd=$(ps -p "${pid}" -o args= 2>/dev/null || true)
  if is_local_process "${cmd}"; then
    echo "SKIP ${label} pid=${pid}: 로컬 프로세스로 판단 (보호)"
    return 0
  fi

  # 코어5~6 = CPU 4,5
  taskset -pc 4-5 "${pid}" >/dev/null
  for tid in /proc/"${pid}"/task/*; do
    taskset -pc 4-5 "${tid##*/}" >/dev/null 2>&1 || true
  done

  renice -n "${n}" -p "${pid}" >/dev/null
  for tid in /proc/"${pid}"/task/*; do
    renice -n "${n}" -p "${tid##*/}" >/dev/null 2>&1 || true
  done

  local nice name aff
  nice=$(ps -p "${pid}" -o nice= | tr -d ' ')
  name=$(ps -p "${pid}" -o comm=)
  aff=$(taskset -pc "${pid}" 2>/dev/null | awk -F: '{print $2}' | xargs)
  local ok="OK"
  [[ "${nice}" == "${n}" ]] || ok="MISMATCH want=${n}"
  echo "${ok}  nice=${nice}  aff=${aff}  ${label}  ${name}  pid=${pid}"
}

first_pid() {
  local pat="$1"
  pgrep -f "${pat}" | head -1 || true
}

# control 은 실행 방식이 두 가지:
#   1) python control_node.py
#   2) ros2 run path_following control_node
#      → .../lib/path_following/control_node  (확장자 .py 없음)
ctrl_pid() {
  local pid
  pid=$(pgrep -f 'path_following/lib/path_following/control_node' | head -1 || true)
  [[ -n "${pid}" ]] || pid=$(pgrep -f 'python[0-9.]* control_node\.py' | head -1 || true)
  [[ -n "${pid}" ]] || pid=$(pgrep -f '[p]ython control_node\.py' | head -1 || true)
  # ros2 run 래퍼는 제외하고 실제 노드만
  if [[ -n "${pid}" ]]; then
    local cmd
    cmd=$(ps -p "${pid}" -o args= 2>/dev/null || true)
    if echo "${cmd}" | grep -q 'ros2 run path_following control_node'; then
      pid=$(pgrep -f 'path_following/lib/path_following/control_node' | head -1 || true)
    fi
  fi
  echo "${pid}"
}

echo "=== 패스쪽: 코어5~6 + nice 적용 ==="
pin_and_nice -15 "$(ctrl_pid)" "1 control"
pin_and_nice -10 "$(first_pid 'stanley_waypoint_follow_node')" "2 stanley"
pin_and_nice  -5 "$(first_pid 'local_planner_node')" "3 local_planner"
# integrated 우선, 없으면 static
OBS_PID=$(first_pid 'integrated_obstacle_node')
[[ -n "${OBS_PID}" ]] || OBS_PID=$(first_pid 'static_obstacle_node')
pin_and_nice   0 "${OBS_PID}" "4 obstacle"
pin_and_nice   5 "$(first_pid 'fgm_node')" "5 fgm"
pin_and_nice  10 "$(pgrep -af 'ros2 launch path_following' | awk '/path_follow_/ {print $1; exit}')" "6 launch"

while read -r pid; do
  [[ -n "${pid}" ]] || continue
  pin_and_nice 19 "${pid}" "viz/monitor"
done < <(pgrep -af 'drive_monitor|stack_status_node' | awk '{print $1}' || true)

echo
echo "=== 로컬쪽 확인 (변경되면 안 됨) ==="
for pat in cartographer_node sllidar_node ebimu_driver vesc_wheel_odom sensor_static_tf static_map_publisher localization_initial rviz2; do
  while read -r pid; do
    [[ -n "${pid}" && -d "/proc/${pid}" ]] || continue
    printf "nice=%-4s aff=%-5s %s\n" \
      "$(ps -p "${pid}" -o nice= | tr -d ' ')" \
      "$(taskset -pc "${pid}" 2>/dev/null | awk -F: '{print $2}' | xargs)" \
      "$(ps -p "${pid}" -o comm=)"
  done < <(pgrep -f "${pat}" || true)
done

echo
echo "완료. (재시작하면 풀림 — 패스 런치 후 이 스크립트를 다시 실행)"
