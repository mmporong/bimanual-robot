#!/usr/bin/env python3
"""ggao50 그리퍼 STL 을 받아 출력 방향을 잡고 K1 Max G-code 까지 만든다.

상류 저장소에 라이선스 표기가 없어 STL 을 이 저장소에 벤더링하지 않는다.
대신 필요할 때 받아서 준비하는 경로를 남긴다. 받은 파일과 생성한 G-code 는
`--out` 아래에만 두고 커밋하지 않는다.

    https://github.com/ggao50/SO101-Parallel-Gripper

사용법:
  python3 design/gripper/prepare_ggao50_print.py --out "$HOME/gcode/ggao50"
  python3 design/gripper/prepare_ggao50_print.py --out ... --skip-slice
"""

from __future__ import annotations

import argparse
import base64
import json
import subprocess
import sys
import urllib.parse
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_REPO = "ggao50/SO101-Parallel-Gripper"
SLICER = REPO_ROOT / "design/cad/slice_k1max_petg.py"
ANALYZER = Path.home() / ".claude/skills/3d/scripts/analyze_stl.py"

# 실측 서포트 판정 결과. 회전으로 없앨 수 있는 것만 회전한다.
PARTS = {
    "backplate": {"rotate": "Y+90", "support": "tree", "note": "레일. 회전해도 서포트 필요"},
    "leftgripper": {"rotate": None, "support": "tree", "note": "죠. 파지면이 수직이라 원본 유지"},
    "rightgripper": {"rotate": "X180", "support": "tree", "note": "죠 대칭"},
    "connectorplate": {"rotate": None, "support": "tree", "note": "손목 연결판"},
    "cameraplate": {"rotate": None, "support": "tree", "note": "손목캠 마운트. 안 쓰면 생략"},
    "pinion": {"rotate": "X+90", "support": "off", "note": "구동 기어. 회전하면 서포트 불필요"},
}
INFILL = "25%"


def fetch(name: str, target: Path) -> None:
    path = urllib.parse.quote(f"STL/Parallel Jaw Gripper - {name}.stl")
    url = f"https://api.github.com/repos/{SOURCE_REPO}/contents/{path}"
    with urllib.request.urlopen(url, timeout=60) as response:
        payload = json.load(response)
    target.write_bytes(base64.b64decode(payload["content"]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True, help="받은 STL 과 G-code 를 둘 위치")
    parser.add_argument("--skip-slice", action="store_true")
    parser.add_argument("--no-camera", action="store_true", help="cameraplate 를 건너뛴다")
    args = parser.parse_args()

    out = args.out.expanduser().resolve()
    raw = out / "raw"
    ready = out / "print_ready"
    raw.mkdir(parents=True, exist_ok=True)
    ready.mkdir(parents=True, exist_ok=True)

    names = [name for name in PARTS if not (args.no_camera and name == "cameraplate")]
    for name in names:
        target = raw / f"{name}.stl"
        if not target.exists():
            print(f"받는 중 {name}")
            fetch(name, target)

    rotate_names = [name for name in names if PARTS[name]["rotate"]]
    if rotate_names:
        if not ANALYZER.exists():
            raise SystemExit(f"방향 분석기를 찾지 못했습니다: {ANALYZER}")
        subprocess.run(
            [sys.executable, str(ANALYZER), "--rotate-out", str(ready)]
            + [str(raw / f"{name}.stl") for name in rotate_names],
            check=True, capture_output=True,
        )
    for name in names:
        destination = ready / f"{name}.stl"
        if not destination.exists():
            destination.write_bytes((raw / f"{name}.stl").read_bytes())

    print(f"\n출력 준비 완료 -> {ready}")
    for name in names:
        info = PARTS[name]
        turn = info["rotate"] or "원본"
        print(f"  {name:16s} 방향 {turn:6s} 서포트 {info['support']:5s}  {info['note']}")

    if args.skip_slice:
        return

    for support in ("off", "tree"):
        targets = [ready / f"{name}.stl" for name in names if PARTS[name]["support"] == support]
        if not targets:
            continue
        command = [sys.executable, str(SLICER), "--out", str(out), "--infill", INFILL,
                   "--support", support]
        if support == "tree":
            command.append("--plate-only")
        command += [str(path) for path in targets]
        result = subprocess.run(command, capture_output=True, text=True)
        report = json.loads(result.stdout) if result.stdout.startswith("{") else {"ok": False}
        for entry in report.get("results", []):
            if entry.get("ok"):
                info = entry["verify"]["info"]
                print(f"  {entry['part']:16s} {info['filament used [cm3]']:>7s} cm3 "
                      f"{info['estimated printing time (normal mode)']:>12s}")
            else:
                print(f"  {entry.get('part')} 실패: {entry.get('error')}")


if __name__ == "__main__":
    main()
