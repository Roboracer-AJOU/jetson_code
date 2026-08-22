#!/usr/bin/env python3
"""ROS 노드별 CPU 사용량을 창(window) 동안 적분해서 보여 준다.

    python3 debug/cpu_by_node.py [초]

`top` 한 장면은 주기적으로 도는 노드에서 튄다. 여기서는 /proc/<pid>/stat 의
utime+stime 를 앞뒤로 읽어 그 구간의 실제 소비만 센다. 스레드 수도 같이
보여 준다 — 코어 하나를 넘는 값은 대개 executor 가 여럿인 것이다.
"""
from __future__ import annotations

import os
import sys
import time

CLK = os.sysconf("SC_CLK_TCK")
KEYS = (
    "path_following",
    "cartographer",
    "sllidar",
    "ebimu",
    "vesc",
    "foxglove",
    "robot_state",
    "static_transform",
)


def procs():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="ignore").strip()
            if not cmd or not any(k in cmd for k in KEYS):
                continue
            if "cpu_by_node" in cmd:
                continue
            out[int(pid)] = cmd
        except OSError:
            continue
    return out


def jiffies(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            parts = f.read().rsplit(") ", 1)[1].split()
        return int(parts[11]) + int(parts[12])  # utime + stime
    except (OSError, IndexError):
        return None


def nthreads(pid):
    try:
        return len(os.listdir(f"/proc/{pid}/task"))
    except OSError:
        return 0


def label(cmd):
    for tok in cmd.split():
        if "/path_following/" in tok:
            return tok.rsplit("/", 1)[-1]
    for k in KEYS:
        if k in cmd:
            for tok in cmd.split():
                if k in tok:
                    return tok.rsplit("/", 1)[-1]
    return cmd.split()[0].rsplit("/", 1)[-1]


def main():
    win = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    ps = procs()
    if not ps:
        print("대상 프로세스가 없다 — 런치가 떠 있는지 확인할 것")
        return

    t0 = time.time()
    a = {p: jiffies(p) for p in ps}
    time.sleep(win)
    b = {p: jiffies(p) for p in ps}
    dt = time.time() - t0

    rows = []
    for p, cmd in ps.items():
        if a.get(p) is None or b.get(p) is None:
            continue
        pct = 100.0 * (b[p] - a[p]) / CLK / dt
        rows.append((pct, label(cmd), p, nthreads(p)))
    rows.sort(reverse=True)

    ncpu = os.cpu_count() or 1
    total = sum(r[0] for r in rows)
    print(f"창 {dt:.1f} s, 코어 {ncpu}개\n")
    print(f"{'CPU%':>7}  {'스레드':>5}  {'pid':>7}  노드")
    for pct, name, pid, nt in rows:
        print(f"{pct:7.1f}  {nt:5d}  {pid:7d}  {name}")
    print(f"\n합계 {total:.1f}%  (코어 환산 {total/100.0:.2f} / {ncpu})")


if __name__ == "__main__":
    main()
