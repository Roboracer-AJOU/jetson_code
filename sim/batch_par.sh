#!/bin/bash
# 시나리오 병렬 배치. 시나리오마다 ROS_DOMAIN_ID 를 달리해 토픽을 격리한다.
#
#   bash sim/batch_par.sh [-j N] [시나리오...]
#
# 주의: 병렬도를 올리면 CPU 가 모자라 스택이 실시간을 못 따라간다. 그러면
# 스캔→판정→조향 지연이 실제보다 커져서, 코드는 멀쩡한데 시뮬에서만 실패가
# 난다. 순차 결과와 다르면 -j 를 낮춰라. sim/verify_parallel.sh 로 확인한다.
cd /home/nvidia/f1tenth_ajou || exit 1

JOBS=3
if [ "$1" = "-j" ]; then JOBS="$2"; shift 2; fi

DEFAULT="
clean two_laps:70
offset_start heading_err bad_start corner_entry
cone cone_offset corridor_edge two_cones three_cones chicane
corner_cone corner_exit narrow_cone
dyn_slow dyn_cross dyn_cross_late lead_slow lead_stops head_on
sudden blocked
"
SCENS="${*:-$DEFAULT}"

OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

run_one() {
  local item="$1" slot="$2"
  local s="${item%%:*}"
  local d="${item#*:}"; [ "$d" = "$s" ] && d=28
  # 도메인 ID 0 은 실차/개발용으로 비워 둔다
  ROS_DOMAIN_ID=$((40 + slot)) bash sim/run_all.sh "$s" "$d" \
    > "$OUT/$s.log" 2>&1
}

echo "=================== 병렬 배치 (-j $JOBS) ==================="
i=0
slot=0
pids=()
names=()
for item in $SCENS; do
  run_one "$item" "$slot" &
  pids+=($!)
  names+=("${item%%:*}")
  slot=$(( (slot + 1) % JOBS ))
  i=$((i + 1))
  if [ $((i % JOBS)) -eq 0 ]; then wait; fi
done
wait

pass=0; fail=0; failed=""
for n in "${names[@]}"; do
  [ -f "$OUT/$n.log" ] || continue
  sed 's/^/  /' "$OUT/$n.log"
  echo "  -----------------------------------------------"
  if grep -q "### 충돌" "$OUT/$n.log"; then
    fail=$((fail+1)); failed="$failed $n"
  else
    pass=$((pass+1))
  fi
done
echo "=================== 요약 ==================="
echo "  통과 $pass / 실패 $fail"
[ -n "$failed" ] && echo "  실패:$failed"
