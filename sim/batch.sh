#!/bin/bash
# 여러 시나리오 연속 실행 후 요약.
#   bash sim/batch.sh                  # 전체 회귀
#   bash sim/batch.sh cone chicane     # 골라서
#   bash sim/batch.sh two_laps:70      # 시나리오별 시간 지정
cd /home/nvidia/f1tenth_ajou || exit 1

DEFAULT="
clean two_laps:70
offset_start heading_err bad_start corner_entry
cone cone_offset corridor_edge two_cones three_cones chicane
corner_cone corner_exit narrow_cone
dyn_slow dyn_cross dyn_cross_late lead_stops head_on
sudden blocked
"
SCENS="${*:-$DEFAULT}"

pass=0; fail=0; failed=""
echo "=================== 시나리오 배치 ==================="
for item in $SCENS; do
  s="${item%%:*}"
  d="${item#*:}"; [ "$d" = "$s" ] && d=28
  out=$(bash sim/run_all.sh "$s" "$d" 2>&1)
  echo "$out" | grep -v "^    t=" | sed 's/^/  /'
  # 판정은 "충돌 없음". blocked 처럼 정지가 정답인 시나리오도 있어서
  # 완주 여부로는 못 가른다.
  if ! echo "$out" | grep -q "### 충돌"; then
    pass=$((pass+1))
  else
    fail=$((fail+1)); failed="$failed $s"
  fi
  echo "  -----------------------------------------------"
done
echo "=================== 요약 ==================="
echo "  통과 $pass / 실패 $fail"
[ -n "$failed" ] && echo "  실패:$failed"
