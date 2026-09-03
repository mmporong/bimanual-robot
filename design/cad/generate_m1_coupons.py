#!/usr/bin/env python3
"""인계 문서 6절 M1 공차 쿠폰의 출력용 STEP/STL을 생성한다.

`docs/20260901_설계팀_제작인계패키지_P0.md`의 M1 표가 후보 치수만 정의하고
모델을 남기지 않아, 출력 순서 1번을 실행할 수 없었다. 이 스크립트가 그 표를
그대로 형상으로 옮긴다. 치수 단위는 mm다.

차체 14종과 같은 원칙을 지킨다. 모든 형상은 수직 프리즘이라 45° 기준에서
서포트가 필요 없고, 수평 원형공을 만들지 않는다. 각인은 윗면을 파 내려가므로
오버행을 만들지 않는다.

사용법:
  "$HOME/.cache/bimanual-cad-venv/bin/python" design/cad/generate_m1_coupons.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cadquery as cq

from generate_hold_flow_cad import (
    K1_MAX_BUILD_MM,
    PETG_DENSITY_G_CM3,
    REPO_ROOT,
    Part,
    export_part,
    rounded_box_xy,
)

DEFAULT_OUTPUT = REPO_ROOT / "design/cad/exports"
DOC_PATH = REPO_ROOT / "docs/20260901_설계팀_제작인계패키지_P0.md"

# 인계 문서 6절 M1 표
M3_CLEARANCE = (3.2, 3.4, 3.6)
M4_CLEARANCE = (4.2, 4.4, 4.6)
ROD_6MM = (6.0, 6.1, 6.2, 6.3)
INSERT_M3 = (3.8, 4.0, 4.2)          # 열압입 인서트 제조사 기준 4.0 전후 3종
FIT_CLEARANCE = (0.20, 0.25, 0.30, 0.35)  # 면당 유격
MAST_FACE_CLEARANCE = 0.4            # 마스트 이음 현재값

PLUG_NOMINAL = 12.0
INSERT_DEPTH = 5.0
LABEL_HEIGHT = 4.0
LABEL_DEPTH = 0.6
STUB_LENGTH = 40.0                   # 30 mm 삽입 시험 + 파지 여유 10 mm


def engrave(shape: cq.Workplane, text: str, x: float, y: float, top_z: float) -> cq.Workplane:
    """윗면에 글자를 파 넣는다. 파 내려가는 방향이라 오버행이 생기지 않는다."""
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=top_z - LABEL_DEPTH)
        .center(x, y)
        .text(text, LABEL_HEIGHT, LABEL_DEPTH + 0.1)
    )
    return shape.cut(cutter)


def hole_coupon() -> cq.Workplane:
    """M3·M4 여유홀, 6 mm 로드, 인서트 홀을 한 판에 모은 쿠폰."""
    thickness = 8.0
    plate = rounded_box_xy(140.0, 95.0, thickness, 4.0)

    rows: list[tuple[str, float, list[tuple[float, float]], float | None]] = [
        # (행 이름, 행 중심 y, [(지름, x)], 블라인드 깊이)
        ("M3", 30.0, [(d, x) for d, x in zip(M3_CLEARANCE, (-45.0, -30.0, -15.0))], None),
        ("M4", 30.0, [(d, x) for d, x in zip(M4_CLEARANCE, (20.0, 35.0, 50.0))], None),
        ("ROD", 0.0, [(d, x) for d, x in zip(ROD_6MM, (-42.0, -22.0, -2.0, 18.0))], None),
        ("INS", -30.0, [(d, x) for d, x in zip(INSERT_M3, (-42.0, -22.0, -2.0))], INSERT_DEPTH),
    ]

    for name, y, holes, depth in rows:
        for diameter, x in holes:
            if depth is None:
                cutter = cq.Workplane("XY").center(x, y).circle(diameter / 2.0).extrude(thickness)
            else:
                cutter = (
                    cq.Workplane("XY")
                    .workplane(offset=thickness - depth)
                    .center(x, y)
                    .circle(diameter / 2.0)
                    .extrude(depth + 0.1)
                )
            plate = plate.cut(cutter)
            plate = engrave(plate, f"{diameter:.1f}", x, y - 9.0, thickness)
        label_x = holes[0][1] - 14.0 if name != "M4" else holes[0][1] - 14.0
        plate = engrave(plate, name, label_x, y + 9.0, thickness)

    return plate


def fit_socket_coupon() -> cq.Workplane:
    """면당 유격 4종을 관통 포켓으로 시험한다."""
    thickness = 12.0
    block = rounded_box_xy(100.0, 34.0, thickness, 3.0)
    for clearance, x in zip(FIT_CLEARANCE, (-37.5, -12.5, 12.5, 37.5)):
        side = PLUG_NOMINAL + 2.0 * clearance
        pocket = (
            cq.Workplane("XY")
            .center(x, 2.0)
            .box(side, side, thickness + 2.0, centered=(True, True, False))
            .translate((0.0, 0.0, -1.0))
        )
        block = block.cut(pocket)
        block = engrave(block, f"{clearance:.2f}", x, -12.0, thickness)
    return block


def fit_plug_coupon() -> cq.Workplane:
    """소켓 4종에 차례로 넣어 보는 기준 각기둥."""
    base = rounded_box_xy(34.0, 26.0, 3.0, 3.0)
    post = (
        cq.Workplane("XY")
        .workplane(offset=3.0)
        .box(PLUG_NOMINAL, PLUG_NOMINAL, 20.0, centered=(True, True, False))
    )
    return base.union(post)


def mast_stub() -> cq.Workplane:
    """마스트 세그먼트와 같은 단면의 짧은 시험편."""
    outer = rounded_box_xy(45.0, 35.0, STUB_LENGTH, 2.0)
    inner = rounded_box_xy(39.0, 29.0, STUB_LENGTH + 2.0, 1.0).translate((0.0, 0.0, -1.0))
    return outer.cut(inner)


def coupler_stub() -> cq.Workplane:
    """이음관과 같은 단면의 짧은 시험편. 면당 0.4 mm 유격을 그대로 쓴다."""
    outer = rounded_box_xy(
        39.0 - 2.0 * MAST_FACE_CLEARANCE,
        29.0 - 2.0 * MAST_FACE_CLEARANCE,
        STUB_LENGTH,
        1.2,
    )
    inner = rounded_box_xy(32.2, 22.2, STUB_LENGTH + 2.0, 0.8).translate((0.0, 0.0, -1.0))
    return outer.cut(inner)


def parts() -> list[Part]:
    return [
        Part(
            "coupon_m1_holes",
            hole_coupon(),
            1,
            "M3 3.2/3.4/3.6, M4 4.2/4.4/4.6 여유홀, 6 mm 로드 6.0~6.3, M3 인서트 3.8/4.0/4.2 블라인드홀",
            "관통볼트가 층을 깨지 않고 흔들림이 최소인 조합을 골라 차체 CAD의 홀 지름을 확정",
        ),
        Part(
            "coupon_m1_fit_socket",
            fit_socket_coupon(),
            1,
            f"면당 유격 {', '.join(f'{c:.2f}' for c in FIT_CLEARANCE)} mm 관통 포켓 4종",
            "손 조립이 가능하고 횡유격 0.5 mm 이하인 유격을 골라 이음·끼움부에 적용",
        ),
        Part(
            "coupon_m1_fit_plug",
            fit_plug_coupon(),
            1,
            f"소켓 시험용 {PLUG_NOMINAL:g} mm 기준 각기둥",
            None,
        ),
        Part(
            "coupon_m1_mast_stub",
            mast_stub(),
            1,
            f"45×35×3 mm 마스트 단면 {STUB_LENGTH:g} mm 시험편",
            "이음관을 30 mm 삽입해 비틀림과 미끄럼을 확인",
        ),
        Part(
            "coupon_m1_coupler_stub",
            coupler_stub(),
            1,
            f"면당 {MAST_FACE_CLEARANCE:g} mm 유격 이음관 {STUB_LENGTH:g} mm 시험편",
            "삽입이 뻑뻑하거나 헐거우면 이음관 외곽을 0.1 mm 단위로 재생성",
        ),
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    records = [export_part(part, output) for part in parts()]
    total_mass_g = sum(r["estimated_petg_mass_g_each"] * r["quantity"] for r in records)

    manifest = {
        "schema_version": "0.1",
        "generator": str(Path(__file__).relative_to(REPO_ROOT)),
        "source_doc": str(DOC_PATH.relative_to(REPO_ROOT)),
        "units": {"length": "mm", "volume": "cm3", "mass": "g"},
        "printer": "Creality K1 Max 300×300×300 mm",
        "material_assumption": f"PETG {PETG_DENSITY_G_CM3} g/cm3, 100% solid CAD volume",
        "estimated_total_petg_mass_g": round(total_mass_g, 1),
        "all_breps_valid": all(r["valid_brep"] for r in records),
        "all_parts_fit_k1_max": all(r["k1_max_fit"] for r in records),
        "build_volume_mm": list(K1_MAX_BUILD_MM),
        "parts": records,
    }
    manifest_path = output / "manifest_m1_coupons.json"
    with manifest_path.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"{len(records)}종 생성, PETG 상한 {total_mass_g:.1f} g -> {manifest_path}")


if __name__ == "__main__":
    main()
