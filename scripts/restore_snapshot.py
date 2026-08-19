#!/usr/bin/env python3
"""에디터 로컬 히스토리 스냅샷으로 path_following 을 통째로 되돌린다.

    python3 scripts/restore_snapshot.py "08-20 00:13" [--apply]

편집 기록을 역순으로 되감는 방식(rewind_edits.py)은 중간에 스냅샷 복원이
한 번이라도 끼면 사슬이 끊겨 못 쓴다. 이건 그 시각에 통째로 찍힌 파일을
그대로 덮어쓰므로 그런 문제가 없다.

그 시각 이후 새로 생긴 파일은 지운다. 단 "스냅샷에 없다" 는 것만으로 지우면
그때도 있었지만 그 빌드에서 안 건드린 파일까지 날아간다. 그래서 git 이 아는
파일이거나 그 이전 스냅샷이 있는 파일은 남긴다.
"""
import datetime
import glob
import json
import os
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

ROOT = Path("/home/nvidia/f1tenth_ajou")
PKG = ROOT / "src/path_following"
HIST = Path("/home/nvidia/.cursor-server/data/User/History")
KST = datetime.timezone(datetime.timedelta(hours=9))
SKIP_DIRS = {"__pycache__", ".pytest_cache", "build", "install", "log"}


def load_history():
    """{상대경로: [(epoch_ms, 스냅샷파일), ...]} 를 만든다."""
    out: dict[str, list[tuple[int, Path]]] = {}
    for d in glob.glob(str(HIST / "*")):
        ej = Path(d) / "entries.json"
        if not ej.exists():
            continue
        try:
            meta = json.loads(ej.read_text(encoding="utf-8"))
        except Exception:
            continue
        res = meta.get("resource", "")
        if "src/path_following/" not in res:
            continue
        rel = res.split("src/path_following/", 1)[1]
        for e in meta.get("entries", []):
            out.setdefault(rel, []).append((e.get("timestamp", 0), Path(d) / e.get("id", "")))
    for v in out.values():
        v.sort()
    return out


def git_tracked() -> set[str]:
    try:
        r = subprocess.run(
            ["git", "ls-files", "src/path_following"],
            cwd=ROOT, capture_output=True, text=True, check=True,
        )
    except Exception:
        return set()
    return {ln.split("src/path_following/", 1)[1] for ln in r.stdout.splitlines() if "src/path_following/" in ln}


def current_files() -> set[str]:
    out = set()
    for p in PKG.rglob("*"):
        if not p.is_file():
            continue
        if any(part in SKIP_DIRS for part in p.relative_to(PKG).parts):
            continue
        out.add(str(p.relative_to(PKG)))
    return out


def main() -> None:
    label = sys.argv[1]
    apply = "--apply" in sys.argv

    hist = load_history()
    snap: dict[str, Path] = {}
    cutoff = 0
    for rel, entries in hist.items():
        for ts, f in entries:
            if datetime.datetime.fromtimestamp(ts / 1000, KST).strftime("%m-%d %H:%M") == label:
                snap[rel] = f
                cutoff = max(cutoff, ts)
    if not snap:
        sys.exit(f"'{label}' 시각의 스냅샷이 없다.")

    cur = current_files()
    tracked = git_tracked()

    restore, unchanged = [], []
    for rel, src in sorted(snap.items()):
        dst = PKG / rel
        want = src.read_text(encoding="utf-8", errors="replace")
        have = dst.read_text(encoding="utf-8", errors="replace") if dst.exists() else None
        (unchanged if have == want else restore).append((rel, have, want))

    # 그 시각 이후 새로 생긴 파일만 삭제 후보로 올린다.
    delete = []
    for rel in sorted(cur - set(snap)):
        if rel in tracked:
            continue
        earlier = [ts for ts, _ in hist.get(rel, []) if ts < cutoff]
        if earlier:
            continue
        delete.append(rel)

    print(f"기준 시각: {label}  (스냅샷 {len(snap)} 파일)\n")
    print(f"되돌릴 파일 {len(restore)} 개:")
    for rel, have, want in restore:
        d = len(want.splitlines()) - len((have or "").splitlines())
        print(f"  {rel:52s} 줄수 {d:+5d}")
    print(f"\n이미 같은 파일 {len(unchanged)} 개")
    print(f"\n삭제할 파일 {len(delete)} 개 (그 이후 새로 생김):")
    for rel in delete:
        print(f"  {rel}")

    if not apply:
        print("\n시험 실행이다. 실제로 쓰려면 --apply 를 붙인다.")
        return

    stamp = datetime.datetime.now(KST).strftime("%m%d_%H%M%S")
    backup = ROOT / f".before_restore_{stamp}.tar.gz"
    with tarfile.open(backup, "w:gz") as tar:
        tar.add(PKG, arcname="path_following", filter=lambda ti: None if any(
            s in ti.name for s in SKIP_DIRS) else ti)
    print(f"\n현재 상태 백업: {backup}")

    for rel, _, want in restore:
        dst = PKG / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_text(want, encoding="utf-8")
    for rel in delete:
        (PKG / rel).unlink(missing_ok=True)
    for p in PKG.rglob("__pycache__"):
        shutil.rmtree(p, ignore_errors=True)
    print(f"{len(restore)} 개 복원, {len(delete)} 개 삭제 완료.")


main()
