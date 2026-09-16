#!/usr/bin/env python3
"""기계 단일 원본에서 README 최상단의 자동 상태 표를 생성한다."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


START = "<!-- AUTO:PROJECT-STATUS:START -->"
END = "<!-- AUTO:PROJECT-STATUS:END -->"


def render(spec: dict, gripper: dict) -> str:
    footprint_x, footprint_y = spec["chassis"]["frame_outer_footprint"]
    left = gripper["selected_finger"]
    navigation = spec["navigation"]
    return "\n".join(
        [
            START,
            "## 단일 원본 자동 요약",
            "",
            "> 아래 표는 기계 YAML과 왼손 그리퍼 선정 매니페스트에서 생성한다. "
            "수정하려면 표가 아니라 연결된 원본 파일을 고친다.",
            "",
            "| 항목 | 현행 값 |",
            "|---|---|",
            f"| 기계 개정 | `{spec['revision']}` |",
            f"| 상·하부 프레임 | {footprint_y:.0f}×{footprint_x:.0f} mm |",
            f"| 상판 높이 | {spec['chassis']['plates']['tabletop']['z_top']:.0f} mm |",
            f"| 왼손 | SO-101 스톡 구동부 + {left['finger_length_mm']:.0f} mm·립 {left['tip_lip_mm']:.0f} mm {left['material'].replace('_', ' ')} FinRay 2개 |",
            f"| 오른손 | `{spec['right_parallel_gripper']['model']}` |",
            f"| RGB-D | {spec['camera']['model']}, 상판 위 {spec['camera']['height_above_tabletop']:.0f} mm |",
            f"| 베이스 | {navigation['drive_type']}, {navigation['motor']['model'] if 'motor' in navigation else navigation['reuse_source']['motor']} |",
            f"| 모델 상태 | `{spec['status']}` |",
            "",
            "원본: [`hold_flow_mechanical_v0_3.yaml`](design/mechanical/hold_flow_mechanical_v0_3.yaml), "
            "[`finray_80mm_lip3_selection.yaml`](design/gripper/finray_80mm_lip3_selection.yaml)",
            END,
        ]
    )


def synchronize(readme: str, generated: str) -> str:
    if START in readme or END in readme:
        if readme.count(START) != 1 or readme.count(END) != 1:
            raise ValueError("README 자동 요약 마커가 불완전하거나 중복됐다")
        prefix, remainder = readme.split(START, 1)
        _, suffix = remainder.split(END, 1)
        return prefix.rstrip() + "\n\n" + generated + suffix
    anchor = "## 다음 세션 최우선 작업"
    if anchor not in readme:
        raise ValueError(f"README 삽입 기준 제목을 찾지 못했다: {anchor}")
    before, after = readme.split(anchor, 1)
    return before.rstrip() + "\n\n" + generated + "\n\n" + anchor + after


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    root = args.repo.resolve()
    readme_path = root / "README.md"
    spec_path = root / "design/mechanical/hold_flow_mechanical_v0_3.yaml"
    gripper_path = root / "design/gripper/finray_80mm_lip3_selection.yaml"
    spec = yaml.safe_load(spec_path.read_text(encoding="utf-8"))
    gripper = yaml.safe_load(gripper_path.read_text(encoding="utf-8"))
    current = readme_path.read_text(encoding="utf-8")
    expected = synchronize(current, render(spec, gripper))
    if current == expected:
        print("README automatic status OK")
        return 0
    if args.write:
        readme_path.write_text(expected, encoding="utf-8")
        print("README automatic status updated")
        return 0
    print("README automatic status is stale; run: python tools/ci/sync_readme_status.py --write")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
