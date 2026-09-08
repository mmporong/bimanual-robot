#!/usr/bin/env python3
"""검증된 SO-101 팔 체인에서 prefix 가능한 양팔 Xacro 매크로를 생성한다.

상류 형상은 TheRobotStudio SO-ARM100/101(Apache-2.0)이고, 관절 오리진과 한계는
로컬 JD-AMR 실기에서 검증된 값을 그대로 쓴다. 그리퍼는 포함하지 않는다.
왼팔은 기본 회전식 죠, 오른팔은 ggao50 평행 그리퍼를 별도 Xacro로 붙인다.

사용법:
  python3 scripts/generate_so101_arm_xacro.py [--source <jdamr urdf>]
"""

from __future__ import annotations

import argparse
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    Path.home() / "jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_description/urdf/jdamr_cube2.urdf"
)
OUTPUT = PACKAGE_ROOT / "third_party/so_arm_101/so101_arm.urdf.xacro"
EXPECTED_SOURCE_SHA256 = "d80275f4b4313229001a55836529fdf2747917bb49fc6c8490195ff42f0d7634"

KEEP_LINKS = [
    "arm_base_link",
    "arm_shoulder_link",
    "arm_upper_arm_link",
    "arm_lower_arm_link",
    "arm_wrist_link",
    "arm_gripper_link",
]
KEEP_JOINTS = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
]

HEADER = """<?xml version="1.0"?>
<!--
  SO-ARM101 팔 체인. 상류 형상은 TheRobotStudio SO-ARM100/101(Apache-2.0)이고,
  관절 오리진과 한계는 로컬 JD-AMR 실기에서 검증된 값이다. 출처는 같은
  디렉터리의 NOTICE 를 본다.

  TheRobotStudio/SO-ARM100 commit: eecbe3e0a9ebb23e25ad7b2759b03884c6660903
  upstream SO101 URDF sha256: 3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c
  local calibrated input sha256: d80275f4b4313229001a55836529fdf2747917bb49fc6c8490195ff42f0d7634

  이 파일은 scripts/generate_so101_arm_xacro.py 가 생성한다. 직접 편집하지 않는다.
  그리퍼는 여기에 없다. 왼팔 기본 회전식 죠와 오른팔 ggao50 평행 그리퍼를
  별도 Xacro가 ${prefix}gripper_link 에 이어 붙인다.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="so101_arm" params="prefix parent *origin">
    <joint name="${prefix}mount_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="${prefix}base_link"/>
      <xacro:insert_block name="origin"/>
    </joint>

"""


def strip_prefix(name: str) -> str:
    return name.replace("arm_", "", 1)


def rewrite(element: ET.Element) -> ET.Element:
    for node in element.iter():
        if node.tag in ("link", "joint") and "name" in node.attrib:
            node.attrib["name"] = "${prefix}" + strip_prefix(node.attrib["name"])
        if node.tag in ("parent", "child") and "link" in node.attrib:
            node.attrib["link"] = "${prefix}" + strip_prefix(node.attrib["link"])
        if node.tag == "mesh" and "filename" in node.attrib:
            node.attrib["filename"] = node.attrib["filename"].replace(
                "package://jdamr_cube_description/meshes/so101",
                "package://hold_flow_description/meshes/so101",
            )
    if element.tag == "joint" and element.attrib.get("type") == "revolute":
        if element.find("dynamics") is None:
            ET.SubElement(element, "dynamics", damping="0.08", friction="0.02")
        limit = element.find("limit")
        if limit is not None and element.find("safety_controller") is None:
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
            margin = min(0.05, (upper - lower) / 4.0)
            ET.SubElement(
                element,
                "safety_controller",
                soft_lower_limit=f"{lower + margin:.8g}",
                soft_upper_limit=f"{upper - margin:.8g}",
                k_position="30",
                k_velocity="3",
            )
    return element


def serialize(element: ET.Element, indent: str = "    ") -> str:
    ET.indent(element, space="  ")
    text = ET.tostring(element, encoding="unicode")
    return "\n".join(indent + line for line in text.rstrip().splitlines())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()

    if not args.source.exists():
        raise SystemExit(
            f"원본 URDF를 찾지 못했습니다: {args.source}\n"
            "JD-AMR 워크스페이스가 없는 기기라면 생성된 결과물이 이미 커밋돼 있으니 그대로 쓴다."
        )

    source_sha256 = hashlib.sha256(args.source.read_bytes()).hexdigest()
    if source_sha256 != EXPECTED_SOURCE_SHA256:
        raise SystemExit(
            "검증되지 않은 SO-101 생성 원본입니다. "
            f"sha256={source_sha256}, expected={EXPECTED_SOURCE_SHA256}. "
            "입력을 검토하고 provenance 상수를 함께 갱신해야 합니다."
        )

    root = ET.parse(args.source).getroot()
    blocks = []
    for name in KEEP_LINKS:
        element = next(item for item in root.findall("link") if item.get("name") == name)
        blocks.append(serialize(rewrite(element)))
    for name in KEEP_JOINTS:
        element = next(item for item in root.findall("joint") if item.get("name") == name)
        blocks.append(serialize(rewrite(element)))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        HEADER + "\n\n".join(blocks) + "\n  </xacro:macro>\n</robot>\n", encoding="utf-8"
    )
    print(
        f"{args.output.relative_to(PACKAGE_ROOT)} 작성, 링크 {len(KEEP_LINKS)} "
        f"관절 {len(KEEP_JOINTS)}, source sha256 {source_sha256}"
    )


if __name__ == "__main__":
    main()
