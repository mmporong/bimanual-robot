#!/usr/bin/env python3
"""HOLD THE FLOW 기구 부품의 STEP/STL을 재현 가능하게 생성한다.

치수 단위는 mm이다. 이 파일은 K1 Max에서 출력할 수 있는 차체판, 팔 어댑터,
카메라 마스트, Astra 거치대, 주행 모터 캐리어와 캐스터 어댑터를 생성한다.
SO-101 본체와 ggao50 평행 그리퍼는 공개 원본 형상을 사용하므로 여기서 다시
모델링하지 않는다.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
import yaml
from cadquery import exporters


REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_2.yaml"
DEFAULT_OUTPUT = REPO_ROOT / "design/cad/exports"
ROS_CAD_MESH_DIR = REPO_ROOT / "src/hold_flow_description/meshes/cad"
PETG_DENSITY_G_CM3 = 1.27
K1_MAX_BUILD_MM = (300.0, 300.0, 300.0)
ROS_MESH_PARTS = {
    "arm_adapter",
    "astra_cradle",
    "camera_backing",
    "camera_mast_segment",
    "chassis_bottom",
    "chassis_middle",
    "chassis_top",
    "lidar_riser",
}
STEP_TIMESTAMP_PATTERN = re.compile(
    r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+(')"
)


@dataclass(frozen=True)
class Part:
    name: str
    shape: cq.Workplane
    quantity: int
    note: str
    measurement_gate: str | None = None


def rounded_box_xy(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def through_holes(
    shape: cq.Workplane,
    points: list[tuple[float, float]],
    diameter: float,
    depth: float,
) -> cq.Workplane:
    cutters = [
        cq.Workplane("XY")
        .center(x, y)
        .circle(diameter / 2.0)
        .extrude(depth)
        for x, y in points
    ]
    for cutter in cutters:
        shape = shape.cut(cutter)
    return shape


def lightened_plate(
    thickness: float,
    cutouts: bool,
    cable_holes: list[tuple[float, float, float]],
) -> cq.Workplane:
    plate = rounded_box_xy(250.0, 250.0, thickness, 12.0)
    plate = through_holes(
        plate,
        [(-105.0, -105.0), (-105.0, 105.0), (105.0, -105.0), (105.0, 105.0)],
        4.5,
        thickness,
    )

    if cutouts:
        for x in (-58.0, 58.0):
            for y in (-58.0, 58.0):
                pocket = rounded_box_xy(80.0, 80.0, thickness + 2.0, 8.0).translate((x, y, -1.0))
                plate = plate.cut(pocket)

    for x, y, diameter in cable_holes:
        plate = plate.cut(
            cq.Workplane("XY")
            .center(x, y)
            .circle(diameter / 2.0)
            .extrude(thickness)
        )
    return plate


def bottom_plate() -> cq.Workplane:
    plate = lightened_plate(6.0, True, [(0.0, 0.0, 28.0)])
    # 주행 모터의 차체측 지지벽. 바퀴만 차체 밖으로 나가고 지지벽은 250 mm 안에 둔다.
    for side in (-1.0, 1.0):
        wall = (
            cq.Workplane("XY")
            .box(70.0, 8.0, 48.0, centered=(True, True, False))
            .translate((90.0, side * 121.0, 0.0))
        )
        plate = plate.union(wall)
        # 축 중심은 조립 시 바닥에서 32.9 mm, 부품 바닥은 6 mm이므로 z=26.9 mm.
        # 원형 수평공은 내부 서포트와 처짐을 만들므로 위가 열린 U 슬롯으로 가공한다.
        axle_slot = (
            cq.Workplane("XY")
            .box(18.2, 12.0, 30.2, centered=(True, True, False))
            .translate((90.0, side * 121.0, 17.8))
        )
        plate = plate.cut(axle_slot)
        # 측면 캐리어 자체를 드릴 지그로 사용해 M4 구멍 4개를 출력 후 천공한다.
    return plate


def middle_plate() -> cq.Workplane:
    return lightened_plate(
        6.0,
        True,
        [(-45.0, 0.0, 42.0), (55.0, 0.0, 32.0)],
    )


def top_plate() -> cq.Workplane:
    plate = lightened_plate(
        8.0,
        False,
        [(25.0, 70.0, 25.0), (25.0, -70.0, 25.0), (-80.0, 0.0, 22.0)],
    )

    arm_adapter_points: list[tuple[float, float]] = []
    for arm_y in (-70.0, 70.0):
        for dx in (-55.0, 55.0):
            for dy in (-35.0, 35.0):
                arm_adapter_points.append((25.0 + dx, arm_y + dy))
    plate = through_holes(plate, arm_adapter_points, 4.5, 8.0)

    mast_points = [(-107.5, -27.5), (-107.5, 27.5), (-52.5, -27.5), (-52.5, 27.5)]
    plate = through_holes(plate, mast_points, 4.5, 8.0)
    lidar_points = [(67.5, -17.0), (67.5, 17.0), (112.5, -17.0), (112.5, 17.0)]
    return through_holes(plate, lidar_points, 3.4, 8.0)


def arm_adapter_plate() -> cq.Workplane:
    plate = rounded_box_xy(130.0, 90.0, 6.0, 8.0)
    plate = through_holes(
        plate,
        [(-55.0, -35.0), (-55.0, 35.0), (55.0, -35.0), (55.0, 35.0)],
        4.5,
        6.0,
    )
    plate = plate.cut(rounded_box_xy(36.0, 28.0, 8.0, 5.0).translate((0.0, 0.0, -1.0)))
    # SO-101 실물 홀 패턴을 재기 전에도 조립 시험이 가능한 4개의 장공.
    for x in (-31.0, 31.0):
        slot = cq.Workplane("XY").center(x, 0.0).slot2D(28.0, 4.5, 90.0).extrude(6.0)
        plate = plate.cut(slot)
    for y in (-25.0, 25.0):
        slot = cq.Workplane("XY").center(0.0, y).slot2D(24.0, 4.5, 0.0).extrude(6.0)
        plate = plate.cut(slot)
    return plate


def camera_backing_plate() -> cq.Workplane:
    plate = rounded_box_xy(75.0, 75.0, 8.0, 7.0)
    plate = through_holes(
        plate,
        [(-27.5, -27.5), (-27.5, 27.5), (27.5, -27.5), (27.5, 27.5)],
        4.5,
        8.0,
    )
    socket = rounded_box_xy(37.0, 27.0, 5.0, 2.0).translate((0.0, 0.0, 3.0))
    return plate.union(socket)


def rectangular_tube(length: float) -> cq.Workplane:
    outer = rounded_box_xy(45.0, 35.0, length, 2.0)
    inner = rounded_box_xy(39.0, 29.0, length + 2.0, 1.0).translate((0.0, 0.0, -1.0))
    # 수평 원형공은 세워 출력할 때 내부 서포트를 요구하므로 모델링하지 않는다.
    # 조립 시 양 끝 20 mm 위치를 4.2 mm로 천공한다.
    return outer.cut(inner)


def mast_coupler() -> cq.Workplane:
    # 마스트 내부 39×29 mm에 면당 0.4 mm 조립 유격을 둔다.
    outer = rounded_box_xy(38.2, 28.2, 60.0, 1.2)
    inner = rounded_box_xy(32.2, 22.2, 62.0, 0.8).translate((0.0, 0.0, -1.0))
    # 마스트와 함께 체결한 상태에서 4.2 mm 관통 드릴링한다.
    return outer.cut(inner)


def astra_cradle() -> cq.Workplane:
    # URDF 카메라 좌표: +X 깊이, +Y 폭, +Z 높이. 165 mm 본체 폭은 Y축이다.
    base = rounded_box_xy(65.0, 182.0, 6.0, 6.0)
    base = base.cut(cq.Workplane("XY").slot2D(20.0, 6.6, 0.0).extrude(6.0))
    # 카메라 310 g을 포함한 장착부 400 g 게이트를 지키는 관통 경량 슬롯.
    # 중앙 M6 조절 슬롯과 양끝 측벽의 하중 경로는 남긴다.
    for y in (-53.0, 53.0):
        pocket = cq.Workplane("XY").center(0.0, y).slot2D(60.0, 28.0, 90.0).extrude(6.0)
        base = base.cut(pocket)
    for side in (-1.0, 1.0):
        wall = (
            cq.Workplane("XY")
            .box(53.0, 6.0, 24.0, centered=(True, True, False))
            .translate((0.0, side * 87.0, 6.0))
        )
        base = base.union(wall)
    return base


def lidar_riser() -> cq.Workplane:
    """LDS-03 스캔면을 약 165 mm로 올리는 짧고 강성 높은 받침대."""
    riser = rounded_box_xy(65.0, 50.0, 29.0, 6.0)
    riser = through_holes(
        riser,
        [(-22.5, -17.0), (-22.5, 17.0), (22.5, -17.0), (22.5, 17.0)],
        3.4,
        29.0,
    )
    # USB/UART 케이블 통과공과 수직 경량공은 출력 방향을 바꾸지 않아도 된다.
    riser = riser.cut(cq.Workplane("XY").circle(8.0).extrude(29.0))
    for x in (-14.0, 14.0):
        riser = riser.cut(cq.Workplane("XY").center(x, 0.0).circle(5.0).extrude(29.0))
    # LDS-03 상세 홀 패턴을 실측하기 전 사용할 상단 조절 장공.
    for y in (-13.0, 13.0):
        riser = riser.cut(cq.Workplane("XY").center(0.0, y).slot2D(20.0, 3.4, 0.0).extrude(29.0))
    return riser


def drive_side_carrier() -> cq.Workplane:
    carrier = rounded_box_xy(82.0, 58.0, 8.0, 6.0)
    carrier = carrier.cut(cq.Workplane("XY").circle(8.05).extrude(8.0))
    carrier = through_holes(
        carrier,
        [(-28.0, -14.5), (-28.0, 14.5), (28.0, -14.5), (28.0, 14.5)],
        4.5,
        8.0,
    )
    # C018 라벨/혼 치수 실측 전 조절 가능한 서보 체결 장공.
    for x in (-18.0, 18.0):
        carrier = carrier.cut(cq.Workplane("XY").center(x, 0.0).slot2D(18.0, 3.6, 90.0).extrude(8.0))
    return carrier


def rear_caster_adapter(thickness: float = 8.0) -> cq.Workplane:
    adapter = rounded_box_xy(70.0, 52.0, thickness, 6.0)
    for x in (-24.0, 24.0):
        adapter = adapter.cut(cq.Workplane("XY").center(x, 0.0).slot2D(24.0, 4.5, 90.0).extrude(thickness))
    adapter = adapter.cut(cq.Workplane("XY").circle(15.0).extrude(thickness))
    return adapter


def parts() -> list[Part]:
    return [
        Part("chassis_bottom", bottom_plate(), 1, "바퀴측 모터 지지벽이 통합된 하판"),
        Part("chassis_middle", middle_plate(), 1, "배터리·제어기 층의 경량 중판"),
        Part("chassis_top", top_plate(), 1, "양팔·마스트·라이다가 체결되는 8 mm 상판"),
        Part(
            "arm_adapter",
            arm_adapter_plate(),
            2,
            "SO-101 베이스와 상판 사이의 교체형 보강 어댑터",
            "SO-101 실물 베이스 체결공 중심거리 측정 후 장공을 원형공으로 확정",
        ),
        Part("camera_backing", camera_backing_plate(), 1, "마스트 하단 보강판"),
        Part("camera_mast_segment", rectangular_tube(267.0), 3, "45×35×3 mm 중공 마스트"),
        Part("camera_mast_coupler", mast_coupler(), 2, "마스트 60 mm 내부 이음관"),
        Part(
            "astra_cradle",
            astra_cradle(),
            1,
            "Astra S 170×55×45 mm 설계 포락선용 거치대",
            "보유 Astra의 하단 M6 위치와 실제 외곽 치수 측정 후 측면 유격 확정",
        ),
        Part(
            "lidar_riser",
            lidar_riser(),
            1,
            "LDS-03 중심 x=90 mm, 스캔면 z≈165 mm용 29 mm 저상 받침대",
            "LDS-03 실물 광학 스캔면 높이와 하부 체결공 측정 후 상단 장공 확정",
        ),
        Part(
            "drive_side_carrier",
            drive_side_carrier(),
            2,
            "625급 외부 베어링과 C018 장공을 가진 측면 캐리어",
            "JD-AMR 휠 허브·C018 혼·625 베어링 축 조합 실측 후 중심공 공차 확정",
        ),
        Part(
            "rear_caster_adapter",
            rear_caster_adapter(),
            1,
            "JD-AMR 볼 캐스터용 높이 조절 어댑터",
            "캐스터 볼 지름·플랜지 홀 패턴·접촉 높이 실측 후 최종 홀 확정",
        ),
        Part("rear_caster_shim_1mm", rear_caster_adapter(1.0), 1, "캐스터 높이 1 mm 보정판"),
        Part("rear_caster_shim_2mm", rear_caster_adapter(2.0), 1, "캐스터 높이 2 mm 보정판"),
        Part("rear_caster_shim_3mm", rear_caster_adapter(3.0), 1, "캐스터 높이 3 mm 보정판"),
    ]


def bbox_mm(shape: cq.Workplane) -> tuple[float, float, float]:
    box = shape.val().BoundingBox()
    return (box.xlen, box.ylen, box.zlen)


def normalize_step_header(step_path: Path) -> None:
    """OpenCascade 생성 시각을 제거해 동일 CAD의 Git diff를 고정한다."""
    source = step_path.read_text(encoding="utf-8")
    normalized, replacements = STEP_TIMESTAMP_PATTERN.subn(
        r"\g<1>1970-01-01T00:00:00\2",
        source,
        count=1,
    )
    if replacements != 1:
        raise ValueError(f"STEP 헤더 생성 시각을 찾지 못했습니다: {step_path}")
    normalized = "\n".join(line.rstrip() for line in normalized.splitlines()) + "\n"
    step_path.write_text(normalized, encoding="utf-8")


def export_part(part: Part, output: Path) -> dict[str, object]:
    step_dir = output / "step"
    stl_dir = output / "stl"
    step_dir.mkdir(parents=True, exist_ok=True)
    stl_dir.mkdir(parents=True, exist_ok=True)
    step_path = step_dir / f"{part.name}.step"
    stl_path = stl_dir / f"{part.name}.stl"
    exporters.export(part.shape, str(step_path))
    normalize_step_header(step_path)
    exporters.export(part.shape, str(stl_path), tolerance=0.08, angularTolerance=0.15)

    bounds = bbox_mm(part.shape)
    volume_mm3 = part.shape.val().Volume()
    return {
        "name": part.name,
        "quantity": part.quantity,
        "valid_brep": bool(part.shape.val().isValid()),
        "bounds_mm": [round(value, 3) for value in bounds],
        "volume_cm3_each": round(volume_mm3 / 1000.0, 3),
        "estimated_petg_mass_g_each": round(volume_mm3 / 1000.0 * PETG_DENSITY_G_CM3, 1),
        "k1_max_fit": all(size <= limit for size, limit in zip(bounds, K1_MAX_BUILD_MM)),
        "step": str(step_path.relative_to(REPO_ROOT)),
        "stl": str(stl_path.relative_to(REPO_ROOT)),
        "note": part.note,
        "measurement_gate": part.measurement_gate,
    }


def export_assembly(items: list[Part], output: Path) -> None:
    assembly = cq.Assembly(name="hold_flow_printed_structure")
    transforms = {
        "chassis_bottom": (0.0, 0.0, 6.0),
        "chassis_middle": (0.0, 0.0, 62.0),
        "chassis_top": (0.0, 0.0, 111.0),
        "camera_backing": (-80.0, 0.0, 119.0),
        "lidar_riser": (90.0, 0.0, 119.0),
    }
    for part in items:
        if part.name not in transforms:
            continue
        x, y, z = transforms[part.name]
        assembly.add(
            part.shape,
            name=part.name,
            loc=cq.Location(cq.Vector(x, y, z)),
        )
    for arm_y, name in ((70.0, "left_arm_adapter"), (-70.0, "right_arm_adapter")):
        adapter = next(item.shape for item in items if item.name == "arm_adapter")
        assembly.add(adapter, name=name, loc=cq.Location(cq.Vector(25.0, arm_y, 119.0)))
    assembly_path = output / "step/hold_flow_printed_structure.step"
    assembly.save(str(assembly_path))
    normalize_step_header(assembly_path)


def sync_ros_cad_meshes(output: Path) -> None:
    ROS_CAD_MESH_DIR.mkdir(parents=True, exist_ok=True)
    for part_name in sorted(ROS_MESH_PARTS):
        shutil.copyfile(
            output / "stl" / f"{part_name}.stl",
            ROS_CAD_MESH_DIR / f"{part_name}.stl",
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    with SPEC_PATH.open(encoding="utf-8") as stream:
        spec = yaml.safe_load(stream)
    if spec["chassis"]["footprint"] != [250.0, 250.0]:
        raise ValueError("현재 CAD 생성기는 검토가 끝난 250×250 mm 차체만 허용합니다.")

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    generated_parts = parts()
    records = [export_part(part, output) for part in generated_parts]
    export_assembly(generated_parts, output)
    sync_ros_cad_meshes(output)
    total_mass_g = sum(
        record["estimated_petg_mass_g_each"] * record["quantity"] for record in records
    )
    manifest = {
        "schema_version": "0.1",
        "generator": str(Path(__file__).relative_to(REPO_ROOT)),
        "source_spec": str(SPEC_PATH.relative_to(REPO_ROOT)),
        "units": {"length": "mm", "volume": "cm3", "mass": "g"},
        "printer": "Creality K1 Max 300×300×300 mm",
        "material_assumption": f"PETG {PETG_DENSITY_G_CM3} g/cm3, 100% solid CAD volume",
        "estimated_total_petg_mass_g": round(total_mass_g, 1),
        "all_breps_valid": all(record["valid_brep"] for record in records),
        "all_parts_fit_k1_max": all(record["k1_max_fit"] for record in records),
        "parts": records,
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
