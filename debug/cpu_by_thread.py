#!/usr/bin/env python3
"""노드 안에서 **어느 스레드** 가 CPU 를 쓰는지 본다.

    python3 debug/cpu_by_thread.py <노드이름조각> [초]

파이썬 콜백은 메인 스레드에서만 돈다 (rclpy.spin 은 단일 스레드). 나머지는
DDS 수신/송신/이벤트 스레드다. 둘의 비율이 곧 "코드를 더 깎아서 얻을 게
있는가" 의 답이다 — DDS 쪽이 대부분이면 파이썬을 아무리 줄여도 안 내려간다.

py-spy 와 달리 ptrace 권한이 필요 없다. /proc 만 읽는다.
"""
from __future__ import annotations

import os
import sys
import time

CLK = os.sysconf("SC_CLK_TCK")


def tids(pid):
    try:
        return sorted(int(t) for t in os.listdir(f"/proc/{pid}/task"))
    except OSError:
        return []


def tstat(pid, tid):
    try:
        with open(f"/proc/{pid}/task/{tid}/stat") as f:
            body = f.read()
        name = body[body.index("(") + 1 : body.rindex(")")]
        parts = body[body.rindex(")") + 2 :].split()
        return name, int(parts[11]) + int(parts[12])
    except (OSError, ValueError, IndexError):
        return None, None


def find_pid(frag):
    for p in os.listdir("/proc"):
        if not p.isdigit():
            continue
        try:
            with open(f"/proc/{p}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="ignore")
        except OSError:
            continue
        if frag in cmd and "cpu_by_thread" not in cmd and "/proc" not in cmd:
            return int(p), cmd.strip()
    return None, None


def main():
    if len(sys.argv) < 2:
        sys.exit("사용법: cpu_by_thread.py <노드이름조각> [초]")
    frag = sys.argv[1]
    win = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

    pid, cmd = find_pid(frag)
    if pid is None:
        sys.exit(f"'{frag}' 에 맞는 프로세스가 없다")

    ts = tids(pid)
    a = {t: tstat(pid, t) for t in ts}
    t0 = time.time()
    time.sleep(win)
    b = {t: tstat(pid, t) for t in ts}
    dt = time.time() - t0

    rows = []
    for t in ts:
        na, ja = a.get(t, (None, None))
        nb, jb = b.get(t, (None, None))
        if ja is None or jb is None:
            continue
        rows.append((100.0 * (jb - ja) / CLK / dt, nb or na, t))
    rows.sort(reverse=True)

    print(f"pid {pid}  창 {dt:.1f} s  스레드 {len(ts)}개\n")
    print(f"{'CPU%':>7}  {'tid':>7}  스레드 이름")
    main_pct = 0.0
    for pct, name, t in rows:
        if pct < 0.05:
            continue
        mark = ""
        if t == pid:
            mark = "   <- 메인 (파이썬 콜백이 여기서 돈다)"
            main_pct = pct
        print(f"{pct:7.1f}  {t:7d}  {name}{mark}")

    total = sum(r[0] for r in rows)
    other = total - main_pct
    print(f"\n합계 {total:.1f}%")
    print(f"  메인(파이썬) {main_pct:.1f}%")
    print(f"  그 외(DDS/전송) {other:.1f}%")
    if total > 1e-6:
        print(f"  → 파이썬 비중 {100.0*main_pct/total:.0f}%")


if __name__ == "__main__":
    main()
