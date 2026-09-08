#!/usr/bin/env python3
"""300 mm 상판·720 mm 작업 높이 모델의 재현 가능한 1차 기하/정역학 검사.

이 계산은 URDF 작성에 쓰는 설계 가정의 자기 일관성을 확인한다. 실물 질량,
체결 강도, 전도 동역학, 바퀴/캐스터 하중 시험을 대체하지 않는다.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


SPEC_PATH = Path(__file__).with_name("hold_flow_mechanical_v0_3.yaml")
REPO_ROOT = SPEC_PATH.parents[2]
URDF_PATH = REPO_ROOT / "src/hold_flow_description/urdf/hold_flow.urdf"
sys.path.insert(0, str(URDF_PATH.parents[1] / "scripts"))
from validate_description import link_transforms  # noqa: E402


def urdf_zero_joint_mass_properties() -> tuple[float, tuple[float, float, float], list[dict]]:
    """URDF 링크 관성 중심을 zero-joint FK로 옮겨 전체 질량과 COM을 계산한다."""
    root = ET.parse(URDF_PATH).getroot()
    transforms = link_transforms(root)
    components: list[dict] = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").attrib["value"])
        origin = inertial.find("origin")
        local_xyz = [0.0, 0.0, 0.0]
        if origin is not None:
            local_xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()]
        transform = transforms[link.attrib["name"]]
        world_xyz = tuple(
            sum(transform[axis][column] * local_xyz[column] for column in range(3))
            + transform[axis][3]
            for axis in range(3)
        )
        components.append(
            {
                "name": link.attrib["name"],
                "mass_kg": mass,
                "com_xyz_m": [round(value, 9) for value in world_xyz],
                "provenance": "URDF inertial at zero joint configuration",
            }
        )
    total = sum(component["mass_kg"] for component in components)
    com = tuple(
        sum(component["mass_kg"] * component["com_xyz_m"][axis] for component in components)
        / total
        for axis in range(3)
    )
    return total, com, components


def polygon_area(points: list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    )


def signed_edge_margin(point: tuple[float, float], polygon: list[tuple[float, float]]) -> float:
    """반시계 볼록 다각형 내부 점에서 가장 가까운 변까지의 거리를 반환한다."""
    px, py = point
    margins = []
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = x2 - x1, y2 - y1
        margins.append((dx * (py - y1) - dy * (px - x1)) / math.hypot(dx, dy))
    return min(margins)


def arm_mount_clearances_mm(
    tabletop_xy: tuple[float, float],
    base_envelope_xy: tuple[float, float],
    center_spacing: float,
) -> dict[str, float]:
    _, tabletop_y = tabletop_xy
    base_x, base_y = base_envelope_xy
    return {
        "front_rear_margin_each": (tabletop_xy[0] - base_x) / 2.0,
        "side_margin_each": (tabletop_y - center_spacing - base_y) / 2.0,
        "between_arm_gap": center_spacing - base_y,
    }


def main() -> None:
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    tabletop = spec["chassis"]["plates"]["tabletop"]
    camera = spec["camera"]
    arms = spec["arm_mounts"]
    navigation = spec["navigation"]

    tabletop_xy = tuple(spec["chassis"]["tabletop_footprint"])
    base_envelope_xy = tuple(arms["source_base_envelope"][:2])
    clearances = arm_mount_clearances_mm(
        tabletop_xy,
        base_envelope_xy,
        float(arms["center_spacing"]),
    )

    mass_kg, com_m, components = urdf_zero_joint_mass_properties()

    support_polygon_m = [
        (float(x) / 1000.0, float(y) / 1000.0)
        for x, y in navigation["support_polygon_xy"]
    ]
    if polygon_area(support_polygon_m) < 0.0:
        support_polygon_m.reverse()
    static_margin_m = signed_edge_margin(com_m[:2], support_polygon_m)

    camera_height = float(camera["optical_center_xyz"][2]) - float(tabletop["z_top"])
    report = {
        "source_spec": str(SPEC_PATH.relative_to(SPEC_PATH.parents[2])),
        "geometry": {
            "tabletop_xy_mm": list(tabletop_xy),
            "tabletop_top_z_mm": tabletop["z_top"],
            "arm_base_xyz_mm": [arms["left"]["xyz"], arms["right"]["xyz"]],
            "arm_mount_clearances_mm": {key: round(value, 3) for key, value in clearances.items()},
            "camera_optical_xyz_mm": camera["optical_center_xyz"],
            "camera_height_above_tabletop_mm": camera_height,
            "wheel_center_separation_mm": navigation["wheel"]["geometric_center_separation"],
            "ball_caster_count": len(navigation["ball_casters"]["centers_xyz"]),
        },
        "provisional_mass_model": {
            "total_kg": round(mass_kg, 3),
            "exact_total_kg": mass_kg,
            "com_mm": [round(value * 1000.0, 1) for value in com_m],
            "static_support_margin_mm": round(static_margin_m * 1000.0, 1),
            "components": components,
            "configuration": "URDF articulated joint positions are zero",
            "scope": "URDF 명목 관성의 정적 검사이며 동적 전도·실물 하중 검증이 아님",
        },
        "gates": {
            "tabletop_is_exactly_300_square": tabletop_xy == (300.0, 300.0),
            "tabletop_is_720_high": math.isclose(float(tabletop["z_top"]), 720.0),
            "camera_is_20_to_30cm_above_tabletop": 200.0 <= camera_height <= 300.0,
            "both_arms_fit_tabletop_without_envelope_overlap": min(clearances.values()) > 0.0,
            "two_drive_wheels": len(
                [key for key in navigation["wheel"] if key.startswith("center_xyz_")]
            ) == 2,
            "two_ball_casters": len(navigation["ball_casters"]["centers_xyz"]) == 2,
            "provisional_com_inside_support_polygon": static_margin_m > 0.0,
        },
    }
    failed = [name for name, passed in report["gates"].items() if not passed]
    if failed:
        raise AssertionError(f"설계 게이트 실패: {failed}")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
