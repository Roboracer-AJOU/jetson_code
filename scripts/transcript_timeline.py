#!/usr/bin/env python3
"""대화 기록에서 "언제 어떤 파일을 고쳤나" 만 뽑아 시간순으로 낸다.

    python3 scripts/transcript_timeline.py <jsonl> [시작줄]

복원 지점이 없는 시각으로 되돌려야 할 때, 그 시각 이후의 편집만 골라 역순으로
지우기 위한 것이다. 기록에는 tool_use 가 통째로 남아 있어서 어떤 파일의 어떤
문자열을 무엇으로 바꿨는지까지 그대로 나온다.
"""
import json
import re
import sys

TS = re.compile(r"<timestamp>[^,]+, ([^<]+?) \(UTC")
Q = re.compile(r"<user_query>\n(.*?)\n</user_query>", re.S)
EDIT = {"StrReplace", "Write", "Delete", "EditNotebook"}


def main() -> None:
    path = sys.argv[1]
    start = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    stamp = "?"
    with open(path, encoding="utf-8") as fh:
        for n, line in enumerate(fh, 1):
            if n < start:
                continue
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            role = ev.get("role")
            content = (ev.get("message") or {}).get("content") or []
            if not isinstance(content, list):
                continue

            for blk in content:
                if not isinstance(blk, dict):
                    continue
                if blk.get("type") == "text" and role == "user":
                    txt = blk.get("text", "")
                    m = TS.search(txt)
                    if m:
                        stamp = m.group(1)
                    q = Q.search(txt)
                    if q:
                        one = " ".join(q.group(1).split())[:90]
                        print(f"\n[{stamp}] (줄 {n}) 사용자: {one}")
                elif blk.get("type") == "tool_use" and blk.get("name") in EDIT:
                    inp = blk.get("input") or {}
                    p = str(inp.get("path", "?")).replace(
                        "/home/nvidia/f1tenth_ajou/", ""
                    )
                    old = " ".join(str(inp.get("old_string", "")).split())[:60]
                    tag = blk["name"]
                    print(f"    {tag:<10} {p}" + (f"   « {old}" if old else ""))


main()
