#!/usr/bin/env python3
"""대화 기록의 편집을 역순으로 되감아 특정 시각의 코드로 되돌린다.

    python3 scripts/rewind_edits.py <jsonl> <cutoff_line> [--apply]

복원 스냅샷이 없는 시각으로 가야 할 때 쓴다. 기록에는 StrReplace 의
`old_string`/`new_string` 이 통째로 남아 있으므로, cutoff 이후의 편집을
**역순으로** 뒤집으면(new → old) 그 시각의 파일이 그대로 나온다. 기억으로
재구성하는 것과 달리 추측이 없다.

기본은 시험 실행이다. `--apply` 를 줘야 파일을 건드린다. 되감다 한 건이라도
안 맞으면 거기서 멈추고 아무것도 안 쓴다 — 절반만 되감긴 파일이 제일 나쁘다.
"""
import json
import sys
from pathlib import Path

ROOT = Path("/home/nvidia/f1tenth_ajou")
SCOPE = "src/path_following/"
EDIT = {"StrReplace", "Write", "Delete"}


def collect(jsonl: str, cutoff: int, upto: int):
    """[cutoff, upto) 구간의 편집을 시간순으로 모은다.

    상한이 필요한 이유: 파일에 따라 이미 다른 방법으로(스냅샷 복원 등) 되돌린
    구간이 있다. 그런 편집까지 되감으려 들면 "못 찾음" 이 쏟아져 진짜 실패와
    구분이 안 된다.
    """
    out = []
    with open(jsonl, encoding="utf-8") as fh:
        for n, line in enumerate(fh, 1):
            if n < cutoff or n >= upto:
                continue
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            content = (ev.get("message") or {}).get("content") or []
            if not isinstance(content, list):
                continue
            for blk in content:
                if not isinstance(blk, dict) or blk.get("type") != "tool_use":
                    continue
                if blk.get("name") not in EDIT:
                    continue
                inp = blk.get("input") or {}
                p = str(inp.get("path", ""))
                if SCOPE not in p:
                    continue
                out.append((n, blk["name"], inp))
    return out


def main() -> None:
    jsonl, cutoff = sys.argv[1], int(sys.argv[2])
    upto = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3].isdigit() else 10**9
    apply = "--apply" in sys.argv

    edits = collect(jsonl, cutoff, upto)
    print(f"[{cutoff}, {upto}) 구간 편집 {len(edits)} 건\n")

    # 파일별 현재 내용을 메모리에 올려 되감는다. 전부 성공해야 쓴다.
    buf: dict[Path, str] = {}
    created: set[Path] = set()
    fails: list[str] = []
    # 한 건이라도 못 되감은 파일은 통째로 건드리지 않는다. 절반만 되감긴
    # 파일은 되감기 전보다 나쁘다.
    dirty: set[Path] = set()

    for n, kind, inp in reversed(edits):
        path = Path(inp["path"])
        if path not in buf:
            buf[path] = path.read_text(encoding="utf-8") if path.exists() else ""

        if kind == "StrReplace":
            new, old = inp.get("new_string", ""), inp.get("old_string", "")
            if new and buf[path].count(new) >= 1:
                buf[path] = buf[path].replace(new, old, 1)
            else:
                fails.append(f"  줄 {n} {path.name}: new_string 못 찾음")
                dirty.add(path)
        elif kind == "Write":
            # 그 턴에 새로 만든 파일이면 지워야 그 시각이 된다.
            created.add(path)
        elif kind == "Delete":
            fails.append(f"  줄 {n} {path.name}: Delete 는 자동 복구 불가")
            dirty.add(path)

    print("되돌릴 파일:")
    for p in sorted(buf):
        rel = str(p).replace(str(ROOT) + "/", "")
        cur = p.read_text(encoding="utf-8") if p.exists() else ""
        mark = " (삭제 예정)" if p in created else ""
        d = len(buf[p].splitlines()) - len(cur.splitlines())
        print(f"  {rel}{mark}   줄수 {d:+d}")

    if fails:
        print(f"\n못 되감은 것 {len(fails)} 건:")
        for f in fails[:20]:
            print(f)

    if dirty:
        print("\n건너뛸 파일 (실패가 섞여 절반만 되감김):")
        for p in sorted(dirty):
            print(f"  {str(p).replace(str(ROOT) + '/', '')}")

    if not apply:
        print("\n시험 실행이다. 실제로 쓰려면 --apply 를 붙인다.")
        return

    done = 0
    for p, text in buf.items():
        if p in dirty:
            continue
        if p in created:
            p.unlink(missing_ok=True)
        else:
            p.write_text(text, encoding="utf-8")
        done += 1
    print(f"\n{done} 개 파일 적용 완료, {len(dirty)} 개 건너뜀.")


main()
