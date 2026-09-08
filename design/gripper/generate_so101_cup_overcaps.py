#!/usr/bin/env python3
"""기본 SO-101 회전식 죠에 묶어 쓰는 컵 접촉 오버캡을 생성한다.

실물 죠 홀 패턴을 확정하기 전 P0용이다. 두 죠를 하나의 강체로 연결하지 않고
같은 부품 두 개를 각각 스트랩으로 고정한다. STL의 원래 자세가 곧 출력 자세다.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import cadquery as cq
from cadquery import exporters


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = REPO_ROOT / "design/gripper/exports/cup_overcaps"
STEP_TIMESTAMP_PATTERN = re.compile(r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+(')")
PETG_DENSITY_G_CM3 = 1.27


def cup_overcap(cup_diameter_mm: float, radial_clearance_mm: float) -> cq.Workplane:
    if not 60.0 <= cup_diameter_mm <= 82.0:
        raise ValueError("컵 외경은 현재 검증 범위 60~82 mm 안이어야 합니다.")
    if not 0.2 <= radial_clearance_mm <= 2.0:
        raise ValueError("반경 유격은 0.2~2.0 mm 안이어야 합니다.")

    width = 44.0
    radial_thickness = 16.0
    height = 30.0
    contact_depth = 3.0
    cup_radius = cup_diameter_mm / 2.0 + radial_clearance_mm

    body = cq.Workplane("XY").box(
        radial_thickness,
        width,
        height,
        centered=(False, True, False),
    )
    cup_center_x = radial_thickness + cup_radius - contact_depth
    cup_cut = (
        cq.Workplane("XY")
        .center(cup_center_x, 0.0)
        .circle(cup_radius)
        .extrude(height)
    )
    body = body.cut(cup_cut)

    # 4 mm 폭 재사용 스트랩 두 줄. 실물 죠 홀 패턴을 몰라도 비파괴 장착할 수 있다.
    for z_center in (8.0, 22.0):
        slot = (
            cq.Workplane("XY")
            .box(radial_thickness + 2.0, 24.0, 4.4, centered=(False, True, True))
            .translate((-1.0, 0.0, z_center))
        )
        body = body.cut(slot)

    # 컵이 젖었을 때 아래로 빠지는 것을 늦추는 얕은 받침. 파지력 대신 쓰지 않는다.
    lip = (
        cq.Workplane("XY")
        .box(9.0, 30.0, 3.0, centered=(False, True, False))
        .translate((radial_thickness - 0.5, 0.0, 0.0))
    )
    return body.union(lip).edges("|Z").fillet(1.0)


def normalize_step_header(step_path: Path) -> None:
    source = step_path.read_text(encoding="utf-8")
    normalized, replacements = STEP_TIMESTAMP_PATTERN.subn(
        r"\g<1>1970-01-01T00:00:00\2",
        source,
        count=1,
    )
    if replacements != 1:
        raise ValueError(f"STEP 헤더 생성 시각을 찾지 못했습니다: {step_path}")
    step_path.write_text(
        "\n".join(line.rstrip() for line in normalized.splitlines()) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cup-diameter", type=float, default=70.0)
    parser.add_argument("--radial-clearance", type=float, default=0.8)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    output = args.output.resolve()
    step_dir = output / "step"
    stl_dir = output / "stl"
    step_dir.mkdir(parents=True, exist_ok=True)
    stl_dir.mkdir(parents=True, exist_ok=True)

    shape = cup_overcap(args.cup_diameter, args.radial_clearance)
    step_path = step_dir / "so101_cup_overcap.step"
    stl_path = stl_dir / "so101_cup_overcap.stl"
    exporters.export(shape, str(step_path))
    normalize_step_header(step_path)
    exporters.export(shape, str(stl_path), tolerance=0.05, angularTolerance=0.1)

    box = shape.val().BoundingBox()
    volume_cm3 = shape.val().Volume() / 1000.0
    manifest = {
        "schema_version": "0.1",
        "generator": str(Path(__file__).relative_to(REPO_ROOT)),
        "design": "SO-101 기본 회전식 죠용 독립 스트랩 컵 오버캡",
        "quantity": 2,
        "cup_outer_diameter_mm": args.cup_diameter,
        "radial_clearance_mm": args.radial_clearance,
        "bounds_mm": [round(box.xlen, 3), round(box.ylen, 3), round(box.zlen, 3)],
        "valid_brep": bool(shape.val().isValid()),
        "volume_cm3_each": round(volume_cm3, 3),
        "solid_petg_mass_g_each": round(volume_cm3 * PETG_DENSITY_G_CM3, 1),
        "print_orientation": "as_exported_no_rotation",
        "support_expected": True,
        "support_reason": "24mm strap slots bridge in original orientation; analyzer suggested Y-90 but rotation was not applied",
        "step": str(step_path.relative_to(REPO_ROOT)),
        "stl": str(stl_path.relative_to(REPO_ROOT)),
        "status": "fit_coupon_and_simulation_preview_not_wet_payload_qualified",
        "measurement_gates": [
            "컵 실제 외경·테이퍼",
            "기본 SO-101 고정·가동 죠 접촉면과 스트랩 간섭",
            "젖은 컵의 미끄럼·변형·낙하 시험",
        ],
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
