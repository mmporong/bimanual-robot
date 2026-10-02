#!/usr/bin/env python3
"""HOLD THE FLOW 기구 부품의 STEP/STL을 재현 가능하게 생성한다.

치수 단위는 mm이다. 하부와 같은 340(x)×450(y) mm 상부 프레임, 720 mm 작업
높이, 2020 기둥, 4분할 출력 상판, 배터리·라이다 장착판, 팔 어댑터와 카메라
부품을 생성한다. 상판 네 조각은 K1 Max에서 평면 출력할 수 있다.
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
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_3.yaml"
DEFAULT_OUTPUT = REPO_ROOT / "design/cad/exports"
ROS_CAD_MESH_DIR = REPO_ROOT / "src/hold_flow_description/meshes/cad"
PETG_DENSITY_G_CM3 = 1.27
ALUMINUM_DENSITY_G_CM3 = 2.70
K1_MAX_BUILD_MM = (300.0, 300.0, 300.0)
K1_MAX_SAFE_XY_MM = 296.0
ROS_MESH_PARTS = {
    "arm_adapter",
    "astra_cradle",
    "camera_backing",
    "camera_mast_segment",
    "battery_mount_plate",
    "frame_column",
    "lidar_mount_plate",
    "tabletop_front_left",
    "tabletop_front_right",
    "tabletop_rear_left",
    "tabletop_rear_right",
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
    material: str = "PETG"
    density_g_cm3: float = PETG_DENSITY_G_CM3
    fdm_part: bool = True


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
    length: float,
    width: float,
    thickness: float,
    corner_radius: float,
    column_centers: list[tuple[float, float]],
    cutouts: bool,
    cable_holes: list[tuple[float, float, float]],
) -> cq.Workplane:
    plate = rounded_box_xy(length, width, thickness, corner_radius)
    plate = through_holes(
        plate,
        column_centers,
        4.5,
        thickness,
    )

    if cutouts:
        for x in (-72.0, 72.0):
            for y in (-72.0, 72.0):
                pocket = rounded_box_xy(104.0, 104.0, thickness + 2.0, 8.0).translate((x, y, -1.0))
                plate = plate.cut(pocket)

    for x, y, diameter in cable_holes:
        plate = plate.cut(
            cq.Workplane("XY")
            .center(x, y)
            .circle(diameter / 2.0)
            .extrude(thickness)
        )
    return plate


def full_top_plate(spec: dict) -> cq.Workplane:
    length, width = spec["chassis"]["tabletop_footprint"]
    thickness = float(spec["chassis"]["plates"]["tabletop"]["thickness"])
    arm_adapter_x, arm_adapter_y = spec["arm_mounts"]["adapter"]["footprint"]
    arm_mounts = (spec["arm_mounts"]["left"]["xyz"], spec["arm_mounts"]["right"]["xyz"])
    mast_x, mast_y = spec["camera"]["mount"]["mast_center_xy"]
    backing_x, backing_y = spec["camera"]["mount"]["backing_footprint"]
    plate = lightened_plate(
        length,
        width,
        thickness,
        float(spec["chassis"]["corner_radius"]),
        [tuple(point) for point in spec["chassis"]["vertical_frame"]["centers_xy"]],
        False,
        [
            (float(arm[0]), float(arm[1]), 25.0)
            for arm in arm_mounts
        ]
        + [(float(mast_x), float(mast_y), 22.0)],
    )

    arm_adapter_points: list[tuple[float, float]] = []
    for arm in arm_mounts:
        for dx in (-(arm_adapter_x / 2.0 - 10.0), arm_adapter_x / 2.0 - 10.0):
            for dy in (-(arm_adapter_y / 2.0 - 10.0), arm_adapter_y / 2.0 - 10.0):
                arm_adapter_points.append((float(arm[0]) + dx, float(arm[1]) + dy))
    plate = through_holes(plate, arm_adapter_points, 4.5, thickness)

    mast_points = [
        (mast_x + dx, mast_y + dy)
        for dx in (-(backing_x / 2.0 - 7.5), backing_x / 2.0 - 7.5)
        for dy in (-(backing_y / 2.0 - 7.5), backing_y / 2.0 - 7.5)
    ]
    plate = through_holes(plate, mast_points, 4.5, thickness)
    split_x = float(spec["chassis"]["plates"]["tabletop"]["split_x"])
    seam_fasteners = (
        [(split_x + dx, y) for dx in (-12.0, 12.0) for y in (-165.0, -65.0, 65.0, 165.0)]
        + [(x, y) for x in (-125.0, -75.0, 20.0, 105.0) for y in (-12.0, 12.0)]
    )
    plate = through_holes(plate, seam_fasteners, 4.5, thickness)
    return plate


def tabletop_quadrants(spec: dict) -> dict[str, cq.Workplane]:
    """완성 외곽과 체결공을 보존하며 상판을 출력 가능한 네 판으로 자른다."""
    length, width = map(float, spec["chassis"]["tabletop_footprint"])
    tabletop = spec["chassis"]["plates"]["tabletop"]
    thickness = float(tabletop["thickness"])
    split_x = float(tabletop["split_x"])
    gap = float(tabletop["seam_gap"])
    x_ranges = {
        "front": (split_x + gap / 2.0, length / 2.0),
        "rear": (-length / 2.0, split_x - gap / 2.0),
    }
    y_ranges = {
        "left": (gap / 2.0, width / 2.0),
        "right": (-width / 2.0, -gap / 2.0),
    }
    source = full_top_plate(spec)
    result: dict[str, cq.Workplane] = {}
    for fore_aft, (x0, x1) in x_ranges.items():
        for side, (y0, y1) in y_ranges.items():
            center_x = (x0 + x1) / 2.0
            center_y = (y0 + y1) / 2.0
            clip = (
                cq.Workplane("XY")
                .box(x1 - x0, y1 - y0, thickness + 2.0, centered=(True, True, False))
                .translate((center_x, center_y, -1.0))
            )
            result[f"tabletop_{fore_aft}_{side}"] = source.intersect(clip).translate(
                (-center_x, -center_y, 0.0)
            )
    return result


def battery_mount_plate(spec: dict) -> cq.Workplane:
    mount = spec["printed_mounts"]["battery_plate"]
    length, width = map(float, mount["footprint"])
    thickness = float(mount["thickness"])
    plate = rounded_box_xy(length, width, thickness, 8.0)
    plate = through_holes(
        plate,
        [(dx, dy) for dx in (-75.0, 75.0) for dy in (-45.0, 45.0)],
        4.5,
        thickness,
    )
    for x in (-45.0, 45.0):
        plate = plate.cut(cq.Workplane("XY").center(x, 0).slot2D(42.0, 5.0, 90).extrude(thickness))
    return plate


def lidar_mount_plate(spec: dict) -> cq.Workplane:
    mount = spec["printed_mounts"]["lidar_plate"]
    length, width = map(float, mount["footprint"])
    thickness = float(mount["thickness"])
    plate = rounded_box_xy(length, width, thickness, 7.0)
    plate = through_holes(
        plate,
        [(dx, dy) for dx in (-35.0, 35.0) for dy in (-27.5, 27.5)],
        3.4,
        thickness,
    )
    for y in (-13.0, 13.0):
        plate = plate.cut(cq.Workplane("XY").center(0.0, y).slot2D(20.0, 3.4, 0.0).extrude(thickness))
    plate = plate.cut(cq.Workplane("XY").circle(8.0).extrude(thickness))
    return plate


def arm_adapter_plate(spec: dict) -> cq.Workplane:
    length, width = spec["arm_mounts"]["adapter"]["footprint"]
    thickness = float(spec["arm_mounts"]["adapter"]["thickness"])
    plate = rounded_box_xy(length, width, thickness, 8.0)
    plate = through_holes(
        plate,
        [
            (dx, dy)
            for dx in (-(length / 2.0 - 10.0), length / 2.0 - 10.0)
            for dy in (-(width / 2.0 - 10.0), width / 2.0 - 10.0)
        ],
        4.5,
        thickness,
    )
    plate = plate.cut(rounded_box_xy(36.0, 28.0, thickness + 2.0, 5.0).translate((0.0, 0.0, -1.0)))
    # SO-101 실물 홀 패턴을 재기 전에도 조립 시험이 가능한 4개의 장공.
    for x in (-31.0, 31.0):
        slot = cq.Workplane("XY").center(x, 0.0).slot2D(28.0, 4.5, 90.0).extrude(thickness)
        plate = plate.cut(slot)
    for y in (-(width / 2.0 - 10.0), width / 2.0 - 10.0):
        slot = cq.Workplane("XY").center(0.0, y).slot2D(24.0, 4.5, 0.0).extrude(thickness)
        plate = plate.cut(slot)
    return plate


def camera_backing_plate(spec: dict) -> cq.Workplane:
    length, width = spec["camera"]["mount"]["backing_footprint"]
    thickness = float(spec["camera"]["mount"]["backing_thickness"])
    plate = rounded_box_xy(length, width, thickness, 7.0)
    hole_dx = length / 2.0 - 7.5
    hole_dy = width / 2.0 - 7.5
    plate = through_holes(
        plate,
        [(dx, dy) for dx in (-hole_dx, hole_dx) for dy in (-hole_dy, hole_dy)],
        4.5,
        thickness,
    )
    socket = rounded_box_xy(21.0, 21.0, 5.0, 2.0).translate((0.0, 0.0, thickness - 5.0))
    return plate.union(socket)


def rectangular_tube(length: float, outer_xy: float = 20.0, wall: float = 2.0) -> cq.Workplane:
    outer = rounded_box_xy(outer_xy, outer_xy, length, 1.5)
    inner = rounded_box_xy(
        outer_xy - 2.0 * wall,
        outer_xy - 2.0 * wall,
        length + 2.0,
        0.8,
    ).translate((0.0, 0.0, -1.0))
    # 수평 원형공은 세워 출력할 때 내부 서포트를 요구하므로 모델링하지 않는다.
    # 조립 시 양 끝 20 mm 위치를 4.2 mm로 천공한다.
    return outer.cut(inner)


def mast_coupler() -> cq.Workplane:
    # 마스트 내부 39×29 mm에 면당 0.4 mm 조립 유격을 둔다.
    outer = rounded_box_xy(38.2, 28.2, 60.0, 1.2)
    inner = rounded_box_xy(32.2, 22.2, 62.0, 0.8).translate((0.0, 0.0, -1.0))
    # 마스트와 함께 체결한 상태에서 4.2 mm 관통 드릴링한다.
    return outer.cut(inner)


def astra_cradle(spec: dict) -> cq.Workplane:
    # URDF 카메라 좌표: +X 깊이, +Y 폭, +Z 높이. 165 mm 본체 폭은 Y축이다.
    camera_depth, camera_width, _ = spec["camera"]["design_envelope"]
    base_depth = camera_depth + 25.0
    base_width = camera_width + 17.0
    base = rounded_box_xy(base_depth, base_width, 6.0, 6.0)
    base = base.cut(cq.Workplane("XY").slot2D(20.0, 6.6, 0.0).extrude(6.0))
    # 카메라 310 g을 포함한 장착부 400 g 게이트를 지키는 관통 경량 슬롯.
    # 중앙 M6 조절 슬롯과 양끝 측벽의 하중 경로는 남긴다.
    for y in (-53.0, 53.0):
        pocket = cq.Workplane("XY").center(0.0, y).slot2D(60.0, 28.0, 90.0).extrude(6.0)
        base = base.cut(pocket)
    for side in (-1.0, 1.0):
        wall = (
            cq.Workplane("XY")
            .box(camera_depth + 13.0, 6.0, 24.0, centered=(True, True, False))
            .translate((0.0, side * (camera_width + 9.0) / 2.0, 6.0))
        )
        base = base.union(wall)
    return base


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


def parts(spec: dict) -> list[Part]:
    frame = spec["chassis"]["vertical_frame"]
    mast = spec["camera"]["mount"]
    quadrants = tabletop_quadrants(spec)
    generated = [
        Part(
            name,
            shape,
            1,
            "340×450 mm 상판을 K1 Max용으로 나눈 PETG 판",
            "프로파일 체결공·SO101 체결부 인서트와 상판 평탄도 실측",
        )
        for name, shape in quadrants.items()
    ]
    generated += [
        Part(
            "frame_column",
            rectangular_tube(float(frame["length"]), float(frame["section"][0])),
            4,
            "상부 프레임을 720 mm 작업 높이로 지지하는 2020 중공 기둥 후보",
            "선정 압출재 카탈로그 단면·선형 질량·체결 브래킷 확정",
            "aluminum_6063_nominal",
            ALUMINUM_DENSITY_G_CM3,
            False,
        ),
        Part(
            "frame_rail_x",
            rectangular_tube(float(spec["chassis"]["profile_rings"]["x_rail_length"])),
            5,
            "하부·상부 프레임과 중앙 이음부를 받치는 x축 2020 프로파일",
            "실물 하부 프레임 체결 길이와 브래킷 간섭 측정",
            "aluminum_6063_nominal",
            ALUMINUM_DENSITY_G_CM3,
            False,
        ),
        Part(
            "frame_rail_y",
            rectangular_tube(float(spec["chassis"]["profile_rings"]["y_rail_length"])),
            5,
            "하부·상부 프레임과 중앙 이음부를 받치는 y축 2020 프로파일",
            "실물 하부 프레임 체결 길이와 브래킷 간섭 측정",
            "aluminum_6063_nominal",
            ALUMINUM_DENSITY_G_CM3,
            False,
        ),
        Part(
            "battery_mount_plate",
            battery_mount_plate(spec),
            1,
            "배터리 스트랩 슬롯과 M4 체결공을 가진 하단 출력판",
            "배터리 실물 외곽·커넥터 방향·스트랩 폭 측정",
        ),
        Part(
            "lidar_mount_plate",
            lidar_mount_plate(spec),
            1,
            "LDS-03 조절 장공과 케이블 구멍을 가진 출력판",
            "LDS-03 하부 체결공과 광학 스캔면 높이 측정",
        ),
        Part(
            "arm_adapter",
            arm_adapter_plate(spec),
            2,
            "SO-101 베이스와 상판 사이의 교체형 보강 어댑터",
            "SO-101 실물 베이스 체결공 중심거리 측정 후 장공을 원형공으로 확정",
        ),
        Part("camera_backing", camera_backing_plate(spec), 1, "20 mm 카메라 마스트 하단 보강판"),
        Part("camera_mast_segment", rectangular_tube(float(mast["mast_length"]), float(mast["mast_section"][0])), 1, "20×20×2 mm 카메라 마스트"),
        Part(
            "astra_cradle",
            astra_cradle(spec),
            1,
            "Astra S 40×165×48 mm 설계 포락선용 거치대",
            "보유 Astra의 하단 M6 위치와 실제 외곽 치수 측정 후 측면 유격 확정",
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
            2,
            "전·후 JD-AMR 볼 캐스터용 높이 조절 어댑터",
            "캐스터 볼 지름·플랜지 홀 패턴·접촉 높이 실측 후 최종 홀 확정",
        ),
        Part("rear_caster_shim_1mm", rear_caster_adapter(1.0), 1, "캐스터 높이 1 mm 보정판"),
        Part("rear_caster_shim_2mm", rear_caster_adapter(2.0), 1, "캐스터 높이 2 mm 보정판"),
        Part("rear_caster_shim_3mm", rear_caster_adapter(3.0), 1, "캐스터 높이 3 mm 보정판"),
    ]
    return generated


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
        "material": part.material,
        "estimated_material_mass_g_each": round(volume_mm3 / 1000.0 * part.density_g_cm3, 1),
        "fdm_part": part.fdm_part,
        "k1_max_fit": all(size <= limit for size, limit in zip(bounds, K1_MAX_BUILD_MM)),
        "k1_max_safe_fit": (
            part.fdm_part
            and bounds[0] <= K1_MAX_SAFE_XY_MM
            and bounds[1] <= K1_MAX_SAFE_XY_MM
            and bounds[2] <= K1_MAX_BUILD_MM[2]
        ),
        "step": str(step_path.relative_to(REPO_ROOT)),
        "stl": str(stl_path.relative_to(REPO_ROOT)),
        "note": part.note,
        "measurement_gate": part.measurement_gate,
    }


def assembly_placements(spec: dict) -> dict[str, object]:
    plates = spec["chassis"]["plates"]
    mast = spec["camera"]["mount"]
    frame = spec["chassis"]["vertical_frame"]
    ring = spec["chassis"]["profile_rings"]
    return {
        "tabletop_front_left": [60.15, 112.65, float(plates["tabletop"]["z_bottom"])],
        "tabletop_front_right": [60.15, -112.65, float(plates["tabletop"]["z_bottom"])],
        "tabletop_rear_left": [-110.15, 112.65, float(plates["tabletop"]["z_bottom"])],
        "tabletop_rear_right": [-110.15, -112.65, float(plates["tabletop"]["z_bottom"])],
        "battery_mount_plate": list(map(float, spec["printed_mounts"]["battery_plate"]["origin_xyz"])),
        "lidar_mount_plate": list(map(float, spec["printed_mounts"]["lidar_plate"]["origin_xyz"])),
        "camera_backing": [*map(float, mast["mast_center_xy"]), float(plates["tabletop"]["z_top"])],
        "camera_mast_segment": [*map(float, mast["mast_center_xy"]), float(mast["mast_z_bottom"])],
        "arm_adapters": [
            [float(arm[0]), float(arm[1]), float(plates["tabletop"]["z_top"])]
            for arm in (spec["arm_mounts"]["left"]["xyz"], spec["arm_mounts"]["right"]["xyz"])
        ],
        "frame_columns": [
            [float(x), float(y), float(frame["z_bottom"])]
            for x, y in frame["centers_xy"]
        ],
        "frame_rail_x": [
            [0.0, y, z]
            for z in (float(ring["lower_center_z"]), float(ring["upper_center_z"]))
            for y in (-float(ring["x_rail_center_y"]), float(ring["x_rail_center_y"]))
        ] + [[0.0, 0.0, float(ring["upper_center_z"])]],
        "frame_rail_y": [
            [x, 0.0, z]
            for z in (float(ring["lower_center_z"]), float(ring["upper_center_z"]))
            for x in (-float(ring["y_rail_center_x"]), float(ring["y_rail_center_x"]))
        ] + [[float(plates["tabletop"]["split_x"]), 0.0, float(ring["upper_center_z"])]],
    }


def export_assembly(items: list[Part], output: Path, spec: dict) -> dict[str, object]:
    assembly = cq.Assembly(name="hold_flow_frame_structure")
    placements = assembly_placements(spec)
    for part in items:
        if part.name not in placements or part.name in {"frame_rail_x", "frame_rail_y"}:
            continue
        x, y, z = placements[part.name]
        assembly.add(
            part.shape,
            name=part.name,
            loc=cq.Location(cq.Vector(x, y, z)),
        )
    for location, name in zip(placements["arm_adapters"], ("left_arm_adapter", "right_arm_adapter")):
        adapter = next(item.shape for item in items if item.name == "arm_adapter")
        assembly.add(adapter, name=name, loc=cq.Location(cq.Vector(*location)))
    column = next(item.shape for item in items if item.name == "frame_column")
    for index, location in enumerate(placements["frame_columns"], start=1):
        assembly.add(column, name=f"frame_column_{index}", loc=cq.Location(cq.Vector(*location)))
    ring = spec["chassis"]["profile_rings"]
    rail_x = next(item.shape for item in items if item.name == "frame_rail_x")
    rail_y = next(item.shape for item in items if item.name == "frame_rail_y")
    rail_x_length = float(ring["x_rail_length"])
    rail_y_length = float(ring["y_rail_length"])
    for index, location in enumerate(placements["frame_rail_x"], start=1):
        x, y, z = location
        assembly.add(
            rail_x,
            name=f"frame_rail_x_{index}",
            loc=cq.Location(cq.Vector(x - rail_x_length / 2.0, y, z), cq.Vector(0, 1, 0), 90),
        )
    for index, location in enumerate(placements["frame_rail_y"], start=1):
        x, y, z = location
        assembly.add(
            rail_y,
            name=f"frame_rail_y_{index}",
            loc=cq.Location(cq.Vector(x, y - rail_y_length / 2.0, z), cq.Vector(1, 0, 0), -90),
        )
    assembly_path = output / "step/hold_flow_frame_structure.step"
    assembly.save(str(assembly_path))
    normalize_step_header(assembly_path)
    return placements


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
    if spec["chassis"]["tabletop_footprint"] != [340.0, 450.0]:
        raise ValueError("현재 CAD 생성기는 340(x)×450(y) mm 4분할 상판만 허용합니다.")

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    generated_parts = parts(spec)
    records = [export_part(part, output) for part in generated_parts]
    placements = export_assembly(generated_parts, output, spec)
    sync_ros_cad_meshes(output)
    total_mass_g = sum(
        record["estimated_material_mass_g_each"] * record["quantity"] for record in records
    )
    manifest = {
        "schema_version": "0.1",
        "generator": str(Path(__file__).relative_to(REPO_ROOT)),
        "source_spec": str(SPEC_PATH.relative_to(REPO_ROOT)),
        "units": {"length": "mm", "volume": "cm3", "mass": "g"},
        "printer": "Creality K1 Max 300×300×300 mm",
        "material_assumption": "부품별 명목 밀도, 중공 CAD 체적; 실제 출력 infill·판재·압출재 질량은 실측으로 교체",
        "assembly_placements_mm": placements,
        "estimated_total_material_mass_g": round(total_mass_g, 1),
        "all_breps_valid": all(record["valid_brep"] for record in records),
        "all_parts_fit_k1_max": all(record["k1_max_fit"] for record in records),
        "all_fdm_parts_safe_fit_k1_max": all(
            record["k1_max_safe_fit"] for record in records if record["fdm_part"]
        ),
        "parts": records,
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
