#!/usr/bin/env python3
"""ggao50 SO101 평행 그리퍼 죠에 볼트로 붙이는 교체형 파지 인서트를 생성한다.

죠 본체는 그대로 두고 물체에 닿는 면만 갈아 끼운다. 세 형상을 같은 부착
치수로 만들어 실물 기울임 시험으로 고를 수 있게 한다. 치수 단위는 mm다.

부착면 기준값은 `STL/Parallel Jaw Gripper - leftgripper.stl` 실측이다.
  파지 평면 x = -49.4, Y -84.2~-11.7 (72.5), Z -29.3~28.2 (57.5)

사용법:
  "$HOME/.cache/bimanual-cad-venv/bin/python" design/gripper/generate_jaw_inserts.py
  ... --bottle-d 60 --v-angle 90
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import cadquery as cq

REPO_ROOT = Path(__file__).resolve().parents[2]
STEP_TIMESTAMP_PATTERN = re.compile(r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+(')")
DEFAULT_OUTPUT = REPO_ROOT / "design/gripper/exports"

# ggao50 죠 파지면 실측 (STL 을 평면으로 잘라 측정)
# 파지면은 손끝으로 갈수록 좁아지는 하나의 테이퍼다.
#   높이 = 0.682 x Y + 67.5   (R^2 0.958)
#   손끝 Y=-84 -> 8.7 mm, 뿌리 Y=-16 -> 57.5 mm
JAW_FACE_LENGTH = 72.5          # 손가락 방향 전체 (Y -84.2 ~ -11.5)
JAW_FACE_HEIGHT_MAX = 57.5      # 뿌리 쪽 최대 높이
JAW_TAPER_SLOPE = 0.682
JAW_TAPER_OFFSET = 67.5

# 인서트는 높이가 충분한 뿌리 쪽에만 붙인다. 손끝은 8.7 mm 라 병을 못 받친다.
MOUNT_Y_NEAR = -14.0            # 뿌리 쪽 끝 (높이 약 57 mm)
MOUNT_Y_FAR = -42.0             # 손끝 쪽 끝 (높이 약 39 mm)
EDGE_INSET = 2.0                # 죠 외곽에서 물러나는 양
PETG_DENSITY_G_CM3 = 1.27
TPU_DENSITY_G_CM3 = 1.21

def jaw_height_at(y: float) -> float:
    """파지면의 병 축 방향 높이. 실측 선형 근사."""
    return min(JAW_TAPER_SLOPE * y + JAW_TAPER_OFFSET, JAW_FACE_HEIGHT_MAX)


# 인서트 공통 부착 규격
INSERT_W = MOUNT_Y_NEAR - MOUNT_Y_FAR                       # 손가락 방향 폭 28.0
INSERT_H_NEAR = jaw_height_at(MOUNT_Y_NEAR) - 2 * EDGE_INSET   # 뿌리 쪽 높이
INSERT_H_FAR = jaw_height_at(MOUNT_Y_FAR) - 2 * EDGE_INSET     # 손끝 쪽 높이
BACK_THICKNESS = 3.0            # 죠에 닿는 등판 두께
BOLT_D = 3.4                    # M3 관통 여유홀
BOLT_HEAD_D = 6.5               # M3 소켓캡 머리 자리파기
BOLT_HEAD_DEPTH = 1.6
BOLT_SPACING = 20.0             # 볼트 2개 중심거리 (손가락 방향, 홈 밖 두꺼운 자리)
CORNER_R = 3.0


def base_plate(pad_thickness: float) -> cq.Workplane:
    """죠 테이퍼를 따르는 사다리꼴 블록. Z가 병 축, Y가 손가락 방향, X가 개폐 방향."""
    total = BACK_THICKNESS + pad_thickness
    half_w = INSERT_W / 2.0
    near, far = INSERT_H_NEAR / 2.0, INSERT_H_FAR / 2.0
    return (
        cq.Workplane("YZ")
        .polyline([
            (half_w, near), (half_w, -near), (-half_w, -far), (-half_w, far),
        ])
        .close()
        .extrude(total)
        .edges("|X")
        .fillet(1.5)
    )


def bolt_holes(shape: cq.Workplane, total_thickness: float) -> cq.Workplane:
    """죠 쪽에서 관통, 파지면 쪽에 머리 자리파기. 머리가 물체에 닿지 않게 묻는다."""
    for y in (-BOLT_SPACING / 2.0, BOLT_SPACING / 2.0):
        shape = shape.cut(
            cq.Workplane("YZ").workplane(offset=-1.0).center(y, 0.0)
            .circle(BOLT_D / 2.0).extrude(total_thickness + 2.0)
        )
        shape = shape.cut(
            cq.Workplane("YZ").workplane(offset=total_thickness - BOLT_HEAD_DEPTH)
            .center(y, 0.0).circle(BOLT_HEAD_D / 2.0).extrude(BOLT_HEAD_DEPTH + 1.0)
        )
    return shape


def flat_pad(pad_thickness: float = 4.0) -> cq.Workplane:
    """A안 - 평면 패드. TPU로 뽑아 마찰만 올린다."""
    total = BACK_THICKNESS + pad_thickness
    return bolt_holes(base_plate(pad_thickness), total)


def v_groove(v_angle_deg: float, groove_width: float, pad_extra: float = 3.0) -> cq.Workplane:
    """B안 - V홈. 지름과 무관하게 병 중심을 잡는다.

    홈은 병 축 방향(Z)으로 길게 파고, 손가락 방향(Y)으로 벌어진다. 홈 깊이는
    각도와 폭에서 결정되며, 깊을수록 유효 개폐가 줄어 큰 병이 안 들어간다.
    """
    import math

    half = math.radians(v_angle_deg / 2.0)
    depth = (groove_width / 2.0) / math.tan(half)
    pad_thickness = depth + pad_extra
    total = BACK_THICKNESS + pad_thickness
    body = base_plate(pad_thickness)

    wedge = (
        cq.Workplane("XY")
        .polyline([
            (total + 1.0, groove_width / 2.0),
            (total + 1.0, -groove_width / 2.0),
            (total - depth, 0.0),
        ])
        .close()
        .extrude(INSERT_H_NEAR + 4.0, both=True)
    )
    body = body.cut(wedge)
    return bolt_holes(body, total)


def trapezoid_groove(v_angle_deg: float, groove_width: float,
                     flat_width: float = 14.0, pad_extra: float = 3.0) -> cq.Workplane:
    """C안 - 사다리꼴 홈. 가운데 평면이 납작한 물체도 받는다."""
    import math

    half = math.radians(v_angle_deg / 2.0)
    depth = ((groove_width - flat_width) / 2.0) / math.tan(half)
    pad_thickness = depth + pad_extra
    total = BACK_THICKNESS + pad_thickness
    body = base_plate(pad_thickness)

    groove = (
        cq.Workplane("XY")
        .polyline([
            (total + 1.0, groove_width / 2.0),
            (total + 1.0, -groove_width / 2.0),
            (total - depth, -flat_width / 2.0),
            (total - depth, flat_width / 2.0),
        ])
        .close()
        .extrude(INSERT_H_NEAR + 4.0, both=True)
    )
    body = body.cut(groove)
    return bolt_holes(body, total)


def normalize_step(step_path: Path) -> None:
    """생성 시각과 행 끝 공백을 제거해 재생성 diff를 고정한다."""
    source = step_path.read_text(encoding="utf-8")
    normalized, _ = STEP_TIMESTAMP_PATTERN.subn(r"\g<1>1970-01-01T00:00:00\2", source, count=1)
    normalized = "\n".join(line.rstrip() for line in normalized.splitlines()) + "\n"
    step_path.write_text(normalized, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--groove-width", type=float, default=24.0,
                        help="홈 폭. 인서트 폭 28 안쪽")
    parser.add_argument("--v-angle", type=float, default=135.0, help="V 끼인각(도)")
    args = parser.parse_args()

    out = args.output.resolve()
    (out / "stl").mkdir(parents=True, exist_ok=True)
    (out / "step").mkdir(parents=True, exist_ok=True)

    variants = {
        "insert_A_flat_pad": (flat_pad(), TPU_DENSITY_G_CM3, "평면 패드. TPU 95A 권장"),
        "insert_B_v_groove": (v_groove(args.v_angle, args.groove_width), TPU_DENSITY_G_CM3,
                              f"V홈 {args.v_angle:g}도. 지름 무관 자기중심"),
        "insert_C_trapezoid": (trapezoid_groove(args.v_angle, args.groove_width), TPU_DENSITY_G_CM3,
                               f"사다리꼴 홈 {args.v_angle:g}도. 원통과 평면 겸용"),
    }

    records = []
    for name, (shape, density, note) in variants.items():
        step = out / "step" / f"{name}.step"
        stl = out / "stl" / f"{name}.stl"
        cq.exporters.export(shape, str(step))
        normalize_step(step)
        cq.exporters.export(shape, str(stl), tolerance=0.05, angularTolerance=0.1)
        bb = shape.val().BoundingBox()
        vol = shape.val().Volume() / 1000.0
        records.append({
            "name": name,
            "quantity": 4,
            "trapezoid_height_mm": [round(INSERT_H_FAR, 1), round(INSERT_H_NEAR, 1)],
            "valid_brep": bool(shape.val().isValid()),
            "bounds_mm": [round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
            "volume_cm3_each": round(vol, 3),
            "estimated_mass_g_each": round(vol * density, 1),
            "note": note,
        })

    manifest = {
        "schema_version": "0.1",
        "generator": str(Path(__file__).relative_to(REPO_ROOT)),
        "target_gripper": "ggao50/SO101-Parallel-Gripper",
        "jaw_face_measured_mm": {
            "length": JAW_FACE_LENGTH,
            "height_max": JAW_FACE_HEIGHT_MAX,
            "taper": "height = 0.682 * Y + 67.5",
        },
        "mount_window_y_mm": [MOUNT_Y_FAR, MOUNT_Y_NEAR],
        "groove_width_mm": args.groove_width,
        "v_angle_deg": args.v_angle,
        "fastening": {
            "bolt": "M3", "quantity_per_insert": 2,
            "spacing_mm": BOLT_SPACING,
            "clearance_hole_mm": BOLT_D,
            "counterbore": {"diameter_mm": BOLT_HEAD_D, "depth_mm": BOLT_HEAD_DEPTH},
            "note": "죠 쪽에서 너트, 파지면 쪽에 머리를 묻는다. 죠에 Ø3.4 관통 2개 필요",
        },
        "parts": records,
    }
    path = out / "manifest_inserts.json"
    with path.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    for r in records:
        print(f"{r['name']:22s} {r['bounds_mm']}  {r['estimated_mass_g_each']:5.1f} g  valid={r['valid_brep']}")
    print(f"-> {path}")


if __name__ == "__main__":
    main()
