#!/usr/bin/env bash
# 코어별 사용률 측정 (mpstat 불필요, /proc/stat 직접 읽음)
#   bash scripts/cpu_watch.sh 0.2 | tee /tmp/cpu.log
INTERVAL="${1:-0.2}"
awk -v interval="$INTERVAL" '
BEGIN {
  printf "%-10s", "time"
  while (("nproc" | getline n) > 0) ncpu = n
  close("nproc")
  for (i = 0; i < ncpu; i++) printf "%6s", "cpu" i
  printf "%9s%9s\n", "loc(0-4)", "path(5-7)"
  while (1) {
    cmd = "cat /proc/stat"
    while ((cmd | getline line) > 0) {
      split(line, f, " ")
      if (f[1] !~ /^cpu[0-9]+$/) continue
      c = substr(f[1], 4) + 0
      tot = 0
      for (j = 2; j <= 11; j++) tot += f[j]
      idle = f[5] + f[6]
      busy = tot - idle
      dt = tot - ptot[c]
      db = busy - pbusy[c]
      if (dt > 0 && ptot[c] > 0) pct[c] = 100 * db / dt
      ptot[c] = tot
      pbusy[c] = busy
    }
    close(cmd)
    if (first++ > 0) {
      "date +%H:%M:%S" | getline ts; close("date +%H:%M:%S")
      printf "%-10s", ts
      loc = 0; path = 0
      for (i = 0; i < ncpu; i++) {
        printf "%6.0f", pct[i]
        if (i <= 4) loc += pct[i]; else path += pct[i]
      }
      printf "%9.0f%9.0f\n", loc, path
      fflush()
    }
    system("sleep " interval)
  }
}'
