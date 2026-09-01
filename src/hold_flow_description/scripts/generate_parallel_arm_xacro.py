#!/usr/bin/env python3
"""벤더링한 Robonine URDF에서 prefix 가능한 양팔 Xacro 매크로를 생성한다."""

from __future__ import annotations

import copy
from pathlib import Path

from lxml import etree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE_ROOT / "third_party/robonine/so_101.urdf.xacro"
OUTPUT = PACKAGE_ROOT / "urdf/so101_parallel_arm.generated.xacro"
XACRO_NS = "http://www.ros.org/wiki/xacro"
CORRECTED_MASSES_KG = {
    "base_link": 0.147,
    "link1_1": 0.100006,
    "link2_1": 0.103,
    "link3_1": 0.104,
    "link4_1": 0.079,
    "link5_1": 0.140,
    "clamp_1": 0.015,
    "clamp_2": 0.015,
}


def local_name(element: etree._Element) -> str:
    return etree.QName(element).localname


def prefix_references(element: etree._Element) -> None:
    tag = local_name(element)
    if tag in {"link", "joint"} and "name" in element.attrib:
        element.attrib["name"] = "${prefix}" + element.attrib["name"]
    if tag in {"parent", "child"} and "link" in element.attrib:
        element.attrib["link"] = "${prefix}" + element.attrib["link"]
    if tag == "mimic" and "joint" in element.attrib:
        element.attrib["joint"] = "${prefix}" + element.attrib["joint"]
    if tag == "mesh" and "filename" in element.attrib:
        element.attrib["filename"] = element.attrib["filename"].replace(
            "package://so_arm_101_description/meshes/",
            "package://hold_flow_description/meshes/robonine/",
        )
    for child in element:
        if isinstance(child.tag, str):
            prefix_references(child)


def correct_inertia(link: etree._Element, original_name: str) -> None:
    target_mass = CORRECTED_MASSES_KG[original_name]
    inertial = link.find("inertial")
    if inertial is None:
        raise ValueError(f"{original_name}: inertial 누락")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        raise ValueError(f"{original_name}: mass/inertia 누락")
    source_mass = float(mass.attrib["value"])
    scale = target_mass / source_mass
    mass.attrib["value"] = f"{target_mass:.9g}"
    for key in ("ixx", "iyy", "izz", "ixy", "iyz", "ixz"):
        inertia.attrib[key] = f"{float(inertia.attrib[key]) * scale:.9g}"


def main() -> None:
    parser = etree.XMLParser(remove_blank_text=True)
    source_root = etree.parse(str(SOURCE), parser).getroot()
    etree.register_namespace("xacro", XACRO_NS)
    output_root = etree.Element("robot", nsmap={"xacro": XACRO_NS})
    output_root.append(
        etree.Comment(
            " GENERATED FILE: scripts/generate_parallel_arm_xacro.py로 재생성. "
            "형상·관절축은 Robonine 공개 모델, 질량은 SO-101 실측급 총질량에 맞춰 보정. "
        )
    )
    macro = etree.SubElement(
        output_root,
        f"{{{XACRO_NS}}}macro",
        name="so101_parallel_arm",
        params="prefix parent xyz rpy",
    )
    mount_joint = etree.SubElement(macro, "joint", name="${prefix}mount_joint", type="fixed")
    etree.SubElement(mount_joint, "parent", link="${parent}")
    etree.SubElement(mount_joint, "child", link="${prefix}base_link")
    etree.SubElement(mount_joint, "origin", xyz="${xyz}", rpy="${rpy}")

    for source_child in source_root:
        if not isinstance(source_child.tag, str):
            continue
        tag = local_name(source_child)
        name = source_child.attrib.get("name", "")
        if tag == "link" and name == "world":
            continue
        if tag == "joint" and name == "world_to_base":
            continue
        if tag not in {"link", "joint"}:
            continue
        copied = copy.deepcopy(source_child)
        if tag == "link":
            correct_inertia(copied, name)
        prefix_references(copied)
        macro.append(copied)

    OUTPUT.write_bytes(
        etree.tostring(output_root, xml_declaration=True, encoding="UTF-8", pretty_print=True)
    )
    print(f"generated: {OUTPUT}")


if __name__ == "__main__":
    main()
