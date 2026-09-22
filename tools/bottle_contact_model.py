"""순정 평면 ggao 죠의 병 접촉 실험. 패드·홈·테이프를 추가하지 않는다."""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import cup_contact_model as common
from workcell_preview_inputs import materialize_urdf
from solve_task_poses import rpy_matrix

DEFAULT_CONFIG = common.ROOT / "config/simulation/bottle_contact_experiment.json"
SIDE = "right"
OBJECT_LABEL = "Bottle"
CONTACT_FRAME = "right_contact_center"
FINGER_LINKS = ("right_finger1_link", "right_finger2_link")
GRIPPER_JOINTS = ("right_finger1_joint", "right_finger2_joint")
GRIPPER_KEY = "gripper_m"
GRIPPER_UNIT = "m"


def load_config(path=DEFAULT_CONFIG):
    config = json.loads(Path(path).read_text())
    if config.get("schema") != "bottle_contact_experiment_v1" or config.get("gripper_unit") != "m":
        raise ValueError("병 실험 스키마와 그리퍼 m 단위가 필요합니다")
    # 공통 판정기의 기존 필드명만 매핑한다. 수치의 단위나 뜻을 변환하지 않는다.
    for suffix in ("center_m", "radius_m", "height_m", "mass_kg", "spawn_offset_m"):
        config["cup_"+suffix] = config.pop("bottle_"+suffix)
    for key in ("reorient_backoff_m", "reorient_height_m", "pregrasp_backoff_m", "pregrasp_height_m"):
        value = config["approach"][key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"approach.{key}: 유한한 양수 필요")
    common.validate_config(config)
    assembly = config.get("assembly_hypothesis")
    if assembly is not None:
        if not isinstance(assembly, dict) or assembly.get("mode") != "replacement_proxy":
            raise ValueError("지원하지 않는 조립 가설")
        for key in ("mount_xyz_m", "mount_rpy_rad", "servo_center_base_m", "servo_size_base_m"):
            value = np.asarray(assembly[key], dtype=float)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError(f"assembly.{key}: 유한한 3벡터 필요")
            if key == "servo_size_base_m" and np.any(value <= 0):
                raise ValueError("서보 가정 크기는 양수여야 합니다")
    if config["cup_radius_m"] >= config["gripper_open_m"]:
        raise ValueError("병 반지름이 순정 죠의 열림 간격보다 작아야 합니다")
    return config


def prepare_model(output, config, source=common.URDF_PATH, *, source_is_materialized=False):
    replacement = config.get("assembly_hypothesis") is not None
    target = output / ("replacement_hypothesis.urdf" if replacement else "stock_ggao_contact_proxy.urdf")
    if source_is_materialized:
        source_tree = ET.parse(source)
        for mesh in source_tree.findall(".//mesh"):
            path = Path(mesh.get("filename", ""))
            if not path.is_absolute() or not path.is_file():
                raise ValueError("결합 모델에는 존재하는 절대 메쉬 경로가 필요합니다")
        source_info = {"materialized_source": str(source),
                       "sha256": hashlib.sha256(Path(source).read_bytes()).hexdigest()}
        source_tree.write(target, encoding="utf-8", xml_declaration=True)
    else:
        source_info = materialize_urdf(source, target,
            base_contract=common.ROOT / "config/navigation/jdamr_migration.json")
    tree = ET.parse(target)
    root = tree.getroot()
    # 원본 관성·관절 한계·좌표계를 유지하고, 로그 사본의 충돌 박스만 시각 박스에 맞춘다.
    for name in ("right_gripper_base_link", *FINGER_LINKS):
        link = common._required(root, f"./link[@name='{name}']")
        visuals = link.findall("visual")
        if not visuals or any(v.find("geometry/box") is None for v in visuals):
            raise ValueError(f"순정 박스 모델 계약 불일치: {name}")
        for collision in list(link.findall("collision")):
            link.remove(collision)
        for visual in visuals:
            collision = ET.SubElement(link, "collision")
            for tag in ("origin", "geometry"):
                collision.append(copy.deepcopy(common._required(visual, tag)))
    if config.get("assembly_hypothesis") is not None:
        apply_replacement_hypothesis(root, config["assembly_hypothesis"])
    ET.SubElement(root, "link", name=CONTACT_FRAME)
    joint = ET.SubElement(root, "joint", name=CONTACT_FRAME+"_joint", type="fixed")
    ET.SubElement(joint, "parent", link="right_tool0")
    ET.SubElement(joint, "child", link=CONTACT_FRAME)
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, config["contact_center_tool_m"])), rpy="0 0 0")
    ET.indent(root)
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return target, {"source": source_info, "proxy_file": target.name,
                    "proxy_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
                    "status": "replacement_hypothesis_uncalibrated" if replacement else "project_stock_flat_jaw_boxes_not_calibrated_stl",
                    "assembly_mode": "replacement_hypothesis_proxy" if replacement else "legacy_overlay",
                    "pads_added": False, "grooves_added": False, "tape_added": False}


def apply_replacement_hypothesis(root, assembly):
    """교체 조립 가설의 로그 사본만 변경. 실물 장착 확정값으로 사용하지 않는다."""
    mount = common._required(root, "./joint[@name='right_gripper_mount_joint']/origin")
    mount.set("xyz", " ".join(map(str, assembly["mount_xyz_m"])))
    mount.set("rpy", " ".join(map(str, assembly["mount_rpy_rad"])))
    link = common._required(root, "./link[@name='right_gripper_link']")
    for tag in ("visual", "collision"):
        for element in list(link.findall(tag)):
            link.remove(element)
    # 기존 조립체를 지운 자리에 서보 몸체를 남긴다. 빈 공간으로 만들어 통과시키지 않는다.
    rotation = rpy_matrix(*assembly["mount_rpy_rad"])
    center = np.asarray(assembly["mount_xyz_m"])+rotation @ assembly["servo_center_base_m"]
    for tag in ("visual", "collision"):
        element = ET.SubElement(link, tag, name="nominal_replacement_servo")
        ET.SubElement(element, "origin", xyz=" ".join(map(str, center)),
                      rpy=" ".join(map(str, assembly["mount_rpy_rad"])))
        ET.SubElement(ET.SubElement(element, "geometry"), "box", size=" ".join(map(str, assembly["servo_size_base_m"])))
        if tag == "visual":
            ET.SubElement(ET.SubElement(element, "material", name="nominal_servo"), "color", rgba="0.2 0.2 0.23 1")
    inertial = common._required(link, "inertial")
    common._required(inertial, "origin").set("xyz", " ".join(map(str, center)))
    common._required(inertial, "origin").set("rpy", "0 0 0")
    mass = float(common._required(inertial, "mass").get("value"))
    sx, sy, sz = assembly["servo_size_base_m"]
    tensor = rotation @ np.diag([mass*(sy*sy+sz*sz)/12, mass*(sx*sx+sz*sz)/12,
                                mass*(sx*sx+sy*sy)/12]) @ rotation.T
    inertia = common._required(inertial, "inertia")
    for key, row, col in (("ixx", 0, 0), ("iyy", 1, 1), ("izz", 2, 2), ("ixy", 0, 1), ("ixz", 0, 2), ("iyz", 1, 2)):
        inertia.set(key, str(tensor[row, col]))


class ContactChain(common.Chain):
    def transforms(self, positions):
        transforms = super().transforms(positions)
        transforms["right_tool0"] = transforms[CONTACT_FRAME]
        return transforms


def make_plan(model, config, start_joint_deg=None, place=False):
    return common.make_plan(model, config, start_joint_deg, place,
                            side=SIDE, chain_factory=ContactChain)


def sample_plan(plan, elapsed_s):
    return common.sample_plan(plan, elapsed_s, GRIPPER_KEY)


def non_gripping_contact_failure(forces, config):
    if any(not math.isfinite(value) for value in forces.values()):
        return "nonfinite_physics_state"
    if max(forces.values(), default=0.) >= config["minimum_contact_force_n"]:
        return "object_contact_with_non_gripping_robot_link"
    return None
