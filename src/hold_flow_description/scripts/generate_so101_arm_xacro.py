#!/usr/bin/env python3
"""검증된 SO-101 팔 체인에서 prefix 가능한 양팔 Xacro 매크로를 생성한다.

상류 형상은 TheRobotStudio SO-ARM100/101(Apache-2.0)이고, 관절 오리진과 한계는
로컬 JD-AMR 실기에서 검증된 값을 그대로 쓴다. 그리퍼는 포함하지 않는다.
상류 회전식 죠 대신 ggao50 평행 그리퍼를 쓰므로 `urdf/ggao50_gripper.xacro`가
`${prefix}gripper_link`에 이어 붙는다.

사용법:
  python3 scripts/generate_so101_arm_xacro.py [--source <jdamr urdf>]
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    Path.home() / "jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_description/urdf/jdamr_cube2.urdf"
)
OUTPUT = PACKAGE_ROOT / "third_party/so_arm_101/so101_arm.urdf.xacro"

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

  이 파일은 scripts/generate_so101_arm_xacro.py 가 생성한다. 직접 편집하지 않는다.
  그리퍼는 여기에 없다. 상류 회전식 죠 대신 ggao50 평행 그리퍼를 쓰므로
  urdf/ggao50_gripper.xacro 가 ${prefix}gripper_link 에 이어 붙는다.
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
    print(f"{args.output.relative_to(PACKAGE_ROOT)} 작성, 링크 {len(KEEP_LINKS)} 관절 {len(KEEP_JOINTS)}")


if __name__ == "__main__":
    main()
