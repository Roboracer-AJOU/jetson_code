#!/bin/bash
# 스택 기동 -> 시나리오 실행 -> 정리
cd /home/nvidia/f1tenth_ajou || exit 1
source install/setup.bash

SCEN="${1:-clean}"
DUR="${2:-25}"
EXTRA="${3:-}"

cleanup() {
  for n in stanley_waypoint_follow_node local_planner_node fgm_node \
           integrated_obstacle_node emergency_brake_node; do
    pkill -f "lib/path_following/${n}" 2>/dev/null
  done
  sleep 0.5
}
cleanup

LOG=/tmp/sim_${SCEN}
ros2 run path_following integrated_obstacle_node --ros-args --log-level warn \
  > ${LOG}_obs.log 2>&1 &
ros2 run path_following fgm_node --ros-args --log-level warn \
  > ${LOG}_fgm.log 2>&1 &
ros2 run path_following local_planner_node --ros-args --log-level info \
  ${EXTRA} > ${LOG}_plan.log 2>&1 &
ros2 run path_following emergency_brake_node --ros-args --log-level info \
  > ${LOG}_aeb.log 2>&1 &
ros2 run path_following stanley_waypoint_follow_node --ros-args --log-level warn \
  -p status_log_hz:=0.0 > ${LOG}_stanley.log 2>&1 &

sleep 5
python3 sim/run_scenario.py "$SCEN" "$DUR"
RC=$?
cleanup
exit $RC
