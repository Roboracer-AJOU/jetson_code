#!/bin/bash
# 스택 기동 -> 시나리오 실행 -> 정리
cd /home/nvidia/f1tenth_ajou || exit 1
source install/setup.bash

SCEN="${1:-clean}"
DUR="${2:-25}"
EXTRA="${3:-}"

# 이 실행이 띄운 노드의 PID 만 정리한다.
#
# 예전엔 pkill -f "lib/path_following/<node>" 로 패턴 학살을 했는데, 그러면
# ROS_DOMAIN_ID 를 나눠 병렬로 돌릴 때 서로의 노드를 죽여 버린다 (실제로
# 3개 병렬이 전부 즉시 스톨했다). 패턴이 아니라 자기 자식만 죽여야 한다.
STACK_PIDS=()
cleanup() {
  for pid in "${STACK_PIDS[@]}"; do
    kill "$pid" 2>/dev/null
  done
  wait 2>/dev/null
}
trap cleanup EXIT

# ------------------------------------------------------------------ 프리플라이트
# control_node 는 여기서 절대 죽이지 않는다 — 실차 시리얼(VESC/ESP32)을 쥐고
# 있어서 시뮬이 남의 하드웨어를 끄면 안 된다. 대신 떠 있으면 거부한다.
#
# 이게 없으면 두 가지가 동시에 조용히 벌어진다:
#  1. control_node 가 /drive 를 구독한다 → 시뮬 명령이 실차 모터로 나간다.
#  2. control_node 가 /vehicle/speed_mps 를 0.0 으로 발행한다 → 시뮬이 내는
#     같은 토픽과 섞인다. Stanley 의 cte 항이 atan(k*cte/(v+soft)) 라
#     v=0 이 섞이면 조향이 폭주해서, 멀쩡한 코드가 무장애물 직선에서
#     벽에 박는다. 로그 어디에도 원인이 안 남는다.
# 패턴에 [c] 를 쓰는 이유: pgrep 자기 자신의 명령줄이 패턴을 포함해서
# 오탐한다. 병렬로 여러 개가 동시에 뜨면 서로를 control_node 로 착각해
# 멀쩡한 실행이 "[중단]" 으로 죽는다. [c]ontrol 은 실제 프로세스에만 맞는다.
if pgrep -f "lib/path_following/[c]ontrol_node" > /dev/null; then
  cat >&2 <<'EOF'
[중단] control_node 가 실행 중입니다.

  시뮬레이션과 충돌합니다:
    - control_node 가 /drive 를 구독 → 시뮬 명령이 실차 모터/서보로 나갑니다
    - /vehicle/speed_mps 를 시뮬과 동시에 발행 → Stanley 조향이 폭주합니다

  control_node 를 띄운 터미널에서 Ctrl+C 로 종료한 뒤 다시 실행하세요.
  (하드웨어를 쥔 프로세스라 이 스크립트가 대신 죽이지 않습니다)
EOF
  exit 3
fi

LOG=/tmp/sim_${SCEN}
ros2 run path_following integrated_obstacle_node --ros-args --log-level warn \
  > ${LOG}_obs.log 2>&1 &
STACK_PIDS+=($!)
ros2 run path_following fgm_node --ros-args --log-level warn \
  > ${LOG}_fgm.log 2>&1 &
STACK_PIDS+=($!)
ros2 run path_following local_planner_node --ros-args --log-level info \
  ${EXTRA} > ${LOG}_plan.log 2>&1 &
STACK_PIDS+=($!)
ros2 run path_following emergency_brake_node --ros-args --log-level info \
  > ${LOG}_aeb.log 2>&1 &
STACK_PIDS+=($!)
ros2 run path_following stanley_waypoint_follow_node --ros-args --log-level warn \
  -p status_log_hz:=0.0 > ${LOG}_stanley.log 2>&1 &
STACK_PIDS+=($!)

# 고정 sleep 대신 실제 기동을 기다린다. 부하가 없으면 더 빨리 넘어가고,
# 병렬로 CPU 가 밀릴 때는 더 기다린다 — 고정값은 양쪽 다 틀린다.
for _ in $(seq 1 60); do
  n=$(ros2 node list 2>/dev/null | grep -c "local_planner_node\|stanley_waypoint\|emergency_brake")
  [ "${n:-0}" -ge 3 ] && break
  sleep 0.5
done

python3 sim/run_scenario.py "$SCEN" "$DUR"
RC=$?
cleanup
exit $RC
