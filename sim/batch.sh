#!/bin/bash
# 여러 시나리오 연속 실행 후 요약
cd /home/nvidia/f1tenth_ajou || exit 1
SCENS="${*:-cone cone_offset two_cones corner_cone dyn_slow dyn_cross sudden blocked}"
echo "=================== 시나리오 배치 ==================="
for s in $SCENS; do
  out=$(bash sim/run_all.sh "$s" 28 2>&1)
  echo "$out" | grep -v "^    t=" | sed 's/^/  /'
  echo "  -----------------------------------------------"
done
