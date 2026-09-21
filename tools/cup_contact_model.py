"""접촉 실험의 가정 모델·IK·판정. 실물 FinRay 보정값으로 사용하지 않는다."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from plan_body_side_grasp import (
    Chain, URDF_PATH, solve_horizontal_endpoint, measure_stage,
    validate_measurement,
)
from workcell_preview_inputs import materialize_urdf
from workcell_preview_motion import build_demo
from cup_contact_recovery import validate_recovery
from cup_contact_place import validate_placement

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/simulation/cup_contact_experiment.json"
SIDE = "left"
OBJECT_LABEL = "Cup"
CONTACT_FRAME = "left_contact_center"
FINGER_LINKS = ("left_gripper_link", "left_moving_jaw_link")
GRIPPER_JOINTS = ("left_gripper",)
GRIPPER_KEY = "gripper_rad"
GRIPPER_UNIT = "rad"


def effective_config_sha256(config):
    """기본값 보완·CLI 적용 후 설정의 순서 독립적인 지문."""
    canonical = json.dumps(config, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def load_config(path=DEFAULT_CONFIG):
    config = json.loads(Path(path).read_text())
    if config.get("schema") != "cup_contact_experiment_v1" or config.get("gripper_unit", "rad") != "rad":
        raise ValueError("지원하지 않는 접촉 실험 설정")
    if "recovery" not in config:
        config["recovery"] = json.loads(DEFAULT_CONFIG.read_text())["recovery"]
    validate_recovery(config["recovery"])
    if "placement" not in config:
        config["placement"] = json.loads(DEFAULT_CONFIG.read_text())["placement"]
    return validate_config(config)


def validate_config(config):
    config.setdefault("mesh_collision_mode", "convexHull")
    if config["mesh_collision_mode"] not in {"convexHull", "convexDecomposition"}:
        raise ValueError("지원하지 않는 메쉬 충돌 근사")
    unit = config.get("gripper_unit", "rad")
    if unit not in {"rad", "m"}:
        raise ValueError("그리퍼 단위는 rad 또는 m이어야 합니다")
    validate_placement(config["placement"], unit)
    vectors = ["table_center_m", "table_size_m", "cup_center_m", "contact_center_tool_m", "cup_spawn_offset_m"]
    if unit == "rad":
        vectors += ["pad_size_m", "fixed_pad_center_tool_m", "moving_pad_center_jaw_m"]
    for key in vectors:
        value = np.asarray(config[key], dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError(f"{key}: 유한한 3벡터 필요")
        if "size" in key and np.any(value <= 0):
            raise ValueError(f"{key}: 크기는 양수")
    positive = ["cup_radius_m", "cup_height_m", "cup_mass_kg",
                "physics_dt_s", "lift_distance_m", "minimum_lift_m", "maximum_tracking_error_rad",
                "minimum_contact_force_n", "maximum_midbody_height_error_m",
                "maximum_contact_center_error_m", "maximum_cup_lateral_drift_m", "maximum_cup_tilt_deg",
                "maximum_preclose_displacement_m"]
    positive += ["moving_pad_mass_kg", "gripper_max_effort_nm"] if unit == "rad" else ["gripper_max_effort_n", "gripper_kp", "gripper_kd"]
    for key in positive:
        if type(config[key]) not in (int, float) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key}: 유한한 양수 필요")
    for key in ("friction", f"gripper_open_{unit}", f"gripper_close_{unit}", "table_surface_z_m"):
        if type(config[key]) not in (int, float) or not math.isfinite(config[key]) or config[key] < 0:
            raise ValueError(f"{key}: 유한한 음이 아닌 값 필요")
    lower, upper = (0., 1.74533) if unit == "rad" else (.010, .0433)
    if not lower <= config[f"gripper_close_{unit}"] < config[f"gripper_open_{unit}"] <= upper:
        raise ValueError("그리퍼 열림/닫힘 범위 오류")
    if not 0 < config["minimum_lift_m"] <= config["lift_distance_m"]:
        raise ValueError("상승 판정 임계값 오류")
    bottom = config["cup_center_m"][2] - config["cup_height_m"] / 2
    top = config["table_center_m"][2] + config["table_size_m"][2] / 2
    if not math.isclose(bottom, top, abs_tol=1e-6) or not math.isclose(top, config["table_surface_z_m"], abs_tol=1e-6):
        raise ValueError("컵 바닥·책상 윗면 좌표 불일치")
    return config


def preclose_failure(sample, config):
    """의도한 닫힘 전 접촉·컵 이동을 파지 접촉과 구분한다."""
    if sample["phase"] not in {"RESET", "REORIENT_ABOVE", "PREGRASP_ABOVE", "ALIGN_MIDDLE", "APPROACH"}:
        return None
    if sample["cup_displacement_from_start_m"] > config["maximum_preclose_displacement_m"]:
        return "cup_displaced_before_close"
    if max(sample["contact_force_n"]) >= config["minimum_contact_force_n"]:
        return "premature_cup_contact"
    return None


def _box(link, name, center_m, size_m):
    for kind in ("visual", "collision"):
        element = ET.SubElement(link, kind, name=name)
        ET.SubElement(element, "origin", xyz=" ".join(map(str, center_m)), rpy="0 0 0")
        ET.SubElement(ET.SubElement(element, "geometry"), "box", size=" ".join(map(str, size_m)))
        if kind == "visual":
            ET.SubElement(ET.SubElement(element, "material", name="assumed_rigid_pad"),
                          "color", rgba="0.05 0.65 0.85 1")


def _required(parent: ET.Element, xpath: str) -> ET.Element:
    element = parent.find(xpath)
    if element is None:
        raise ValueError(f"접촉 모델의 필수 URDF 요소 누락: {xpath}")
    return element


def prepare_model(output: Path, config, source=URDF_PATH):
    """원본을 보존하고 로그 폴더에만 가정 URDF를 만든다."""
    target = output / "contact_proxy.urdf"
    provenance = materialize_urdf(source, target,
        base_contract=ROOT / "config/navigation/jdamr_migration.json")
    tree = ET.parse(target)
    root = tree.getroot()
    fixed = _required(root, "./link[@name='left_gripper_link']")
    moving = _required(root, "./link[@name='left_moving_jaw_link']")
    inertial = _required(moving, "inertial")
    inertia_origin = _required(inertial, "origin")
    mass_element = _required(inertial, "mass")
    inertia = _required(inertial, "inertia")
    for kind in ("visual", "collision"):
        for item in list(fixed.findall(kind)):
            mesh = item.find("geometry/mesh")
            if mesh is not None and "wrist_roll_follower" in mesh.get("filename", ""):
                fixed.remove(item)
        for item in list(moving.findall(kind)):
            moving.remove(item)
    transforms = Chain(source).transforms({})
    fixed_from_tool = np.linalg.inv(transforms["left_gripper_link"]) @ transforms["left_tool0"]
    center = (fixed_from_tool @ np.r_[config["fixed_pad_center_tool_m"], 1])[:3]
    _box(fixed, "assumed_fixed_pad", center, config["pad_size_m"])
    # 원래 회전식 관절을 유지하고 이동측 손가락의 길이축을 jaw -Y에 둔다.
    sx, sy, sz = config["pad_size_m"]
    _box(moving, "assumed_moving_pad", config["moving_pad_center_jaw_m"], [sx, sz, sy])
    inertia_origin.set("xyz", " ".join(map(str, config["moving_pad_center_jaw_m"])))
    mass = config["moving_pad_mass_kg"]
    mass_element.set("value", str(mass))
    for key, value in zip(("ixx", "iyy", "izz"),
                          (mass*(sz*sz+sy*sy)/12, mass*(sx*sx+sy*sy)/12, mass*(sx*sx+sz*sz)/12)):
        inertia.set(key, str(value))
    for key in ("ixy", "ixz", "iyz"):
        inertia.set(key, "0")
    ET.SubElement(root, "link", name="left_contact_center")
    joint = ET.SubElement(root, "joint", name="left_contact_center_joint", type="fixed")
    ET.SubElement(joint, "parent", link="left_tool0")
    ET.SubElement(joint, "child", link="left_contact_center")
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, config["contact_center_tool_m"])), rpy="0 0 0")
    ET.indent(root)
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return target, {"source": provenance, "proxy_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
                    "status": "assumed_rigid_pads_stock_revolute_joint_not_finray_twin"}


class ContactChain(Chain):
    """기존 5구속 솔버에 명시한 접촉 프레임을 전달한다."""
    def transforms(self, positions):
        transforms = super().transforms(positions)
        transforms["left_tool0"] = transforms["left_contact_center"]
        return transforms


def make_plan(model: Path, config, start_joint_deg=None, place=False, *, side="left", chain_factory=ContactChain,
              ik_seed_joint_deg=None):
    chain = chain_factory(model)
    home = build_demo(Chain(model))[0]
    center = np.asarray(config["cup_center_m"])
    gripper_key = "gripper_rad" if side == "left" else "gripper_m"
    unit = "rad" if side == "left" else "m"
    open_position = config[f"gripper_open_{unit}"]
    close_position = config[f"gripper_close_{unit}"]
    approach = config.get("approach", {"reorient_backoff_m": .07, "reorient_height_m": .13,
                                      "pregrasp_backoff_m": .025, "pregrasp_height_m": .08})
    # 작업대 가정 위치에 대한 경로이며 카메라 검출 결과가 아니다.
    targets = [
        ("REORIENT_ABOVE", center + [-approach["reorient_backoff_m"], 0, approach["reorient_height_m"]], 3.0),
        ("PREGRASP_ABOVE", center + [-approach["pregrasp_backoff_m"], 0, approach["pregrasp_height_m"]], 2.0),
        ("ALIGN_MIDDLE", center + [-approach["pregrasp_backoff_m"], 0, 0], 2.0),
        ("APPROACH", center, 2.0),
        ("LIFT", center + [0, 0, config["lift_distance_m"]], 3.0),
    ]
    start = home[f"{side}_joint_deg"] if start_joint_deg is None else start_joint_deg
    if np.asarray(start).shape != (5,) or not np.isfinite(start).all():
        raise ValueError("시작 관절은 유한한 5축 각도여야 합니다")
    start = np.asarray(start, dtype=float).tolist()
    if start_joint_deg is not None:
        targets = targets[1:]
    seed_degrees = start if ik_seed_joint_deg is None else ik_seed_joint_deg
    if np.asarray(seed_degrees).shape != (5,) or not np.isfinite(seed_degrees).all():
        raise ValueError("IK 초기값은 유한한 5축 각도여야 합니다")
    seed = np.radians(seed_degrees)
    stages = []
    for name, target, duration_s in targets:
        seed = solve_horizontal_endpoint(chain, side, target, seed, restarts=1, iterations=240)
        measured = measure_stage(chain, side, seed, target, np.array([1, 0, 0]))
        reasons = validate_measurement(measured)
        if reasons:
            raise ValueError(f"{name} IK 미통과: {reasons}")
        stages.append({"name": name, "joint_deg": np.degrees(seed).tolist(), "target_m": target.tolist(),
                       "duration_s": duration_s, "measurement": measured})
    grasp = stages[-2]
    poses = [{"name": "RESET", "joint_deg": list(start), "duration_s": 1.0}, *stages[:-1],
             {"name": "CLOSE", "joint_deg": grasp["joint_deg"], "duration_s": 2.0},
             {"name": "CONTACT_HOLD", "joint_deg": grasp["joint_deg"], "duration_s": 1.0}, stages[-1],
             {"name": "LIFT_HOLD", "joint_deg": stages[-1]["joint_deg"], "duration_s": 2.0}]
    for pose in poses:
        pose[gripper_key] = close_position if pose["name"] in {
            "CLOSE", "CONTACT_HOLD", "LIFT", "LIFT_HOLD"} else open_position
    if place:
        lower_target = center + [0, 0, -config["placement"]["lowering_offset_m"]]
        lower = solve_horizontal_endpoint(chain, side, lower_target, seed, restarts=1, iterations=240)
        measured = measure_stage(chain, side, lower, lower_target, np.array([1, 0, 0]))
        reasons = validate_measurement(measured)
        if reasons:
            raise ValueError(f"LOWER IK 미통과: {reasons}")
        lower_pose = {"joint_deg": np.degrees(lower).tolist(), "target_m": lower_target.tolist(),
                      "measurement": measured, gripper_key: close_position}
        retreat_stages = []
        retreat_seed = lower
        for name, target in (
            ("WITHDRAW", center + [-config["placement"]["withdraw_distance_m"], 0, 0]),
            ("CLEAR_ABOVE", center + [-config["placement"]["withdraw_distance_m"], 0,
                                      config["placement"]["clearance_height_m"]]),
        ):
            retreat_seed = solve_horizontal_endpoint(chain, side, target, retreat_seed, restarts=1, iterations=240)
            measured = measure_stage(chain, side, retreat_seed, target, np.array([1, 0, 0]))
            reasons = validate_measurement(measured)
            if reasons:
                raise ValueError(f"{name} IK 미통과: {reasons}")
            retreat_stages.append({"name": name, "joint_deg": np.degrees(retreat_seed).tolist(),
                                   "target_m": target.tolist(), "measurement": measured,
                                   "duration_s": 2.0, gripper_key: open_position})
        poses.extend([
            {**lower_pose, "name": "LOWER", "duration_s": 3.0},
            {**lower_pose, "name": "TABLE_SETTLE", "duration_s": 1.0},
            {**lower_pose, "name": "OPEN", "duration_s": 2.0, gripper_key: open_position},
            {**lower_pose, "name": "RELEASE_HOLD", "duration_s": 1.0, gripper_key: open_position},
            *retreat_stages,
            {**retreat_stages[-1], "name": "PLACE_HOLD", "duration_s": 2.0},
        ])
    return {"poses": poses, "right_parked_deg": home["right_joint_deg"],
            "parked_joint_deg": home["right_joint_deg" if side == "left" else "left_joint_deg"],
            "active_side": side, "gripper_unit": unit,
            "placement_enabled": place,
            "hardware_accessed": False, "il_dataset_used": False, "tcp_frame": f"{side}_contact_center",
            "continuous_collision_validated": False, "model_calibrated": False}


def sample_plan(plan, elapsed_s, gripper_key="gripper_rad"):
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError("경과 시간 오류")
    poses = plan["poses"]
    previous = poses[0]
    for pose in poses:
        if elapsed_s < pose["duration_s"]:
            t = elapsed_s / pose["duration_s"]
            weight = t*t*(3-2*t)
            return {"phase": pose["name"], "done": False,
                    "joint_deg": (np.array(previous["joint_deg"])*(1-weight)+np.array(pose["joint_deg"])*weight).tolist(),
                    gripper_key: previous[gripper_key]*(1-weight)+pose[gripper_key]*weight}
        elapsed_s -= pose["duration_s"]
        previous = pose
    return {"phase": previous["name"], "joint_deg": previous["joint_deg"],
            gripper_key: previous[gripper_key], "done": True}


def evaluate_lift(samples, config, completed, reason):
    hold = [s for s in samples if s["phase"] == "LIFT_HOLD"]
    minimum_samples = max(2, math.ceil(1.0 / config["physics_dt_s"]))
    terminal = hold[-minimum_samples:]
    enough = len(terminal) == minimum_samples
    valid = enough and all(
        np.isfinite([*s["cup_position_m"], *s["contact_force_n"], s["arm_error_rad"],
                     s["contact_center_error_m"], s["cup_tilt_deg"]]).all()
        and s["cup_position_m"][2]-config["cup_center_m"][2] >= config["minimum_lift_m"]
        and min(s["contact_force_n"]) >= config["minimum_contact_force_n"]
        and s["arm_error_rad"] <= config["maximum_tracking_error_rad"]
        and abs(s["midbody_height_error_m"]) <= config["maximum_midbody_height_error_m"]
        and s["contact_center_error_m"] <= config["maximum_contact_center_error_m"]
        and np.linalg.norm(np.array(s["cup_position_m"][:2])-config["cup_center_m"][:2])
            <= config["maximum_cup_lateral_drift_m"]
        and s["cup_tilt_deg"] <= config["maximum_cup_tilt_deg"]
        for s in terminal)
    return {"completed": bool(completed), "stop_reason": reason,
            "rigid_proxy_lift_pass": bool(completed and valid), "hold_window_samples": len(terminal),
            "real_finray_grasp_verified": False, "hardware_accessed": False,
            "object_attachment_used": False, "il_dataset_used": False}


def ready_to_lift(samples, config):
    """닫힌 뒤 마지막 0.5초 동안 양쪽 접촉과 중간 높이가 유지돼야 상승한다."""
    count = max(2, math.ceil(.5 / config["physics_dt_s"]))
    hold = [s for s in samples if s["phase"] == "CONTACT_HOLD"][-count:]
    return len(hold) == count and all(
        np.isfinite([*s["contact_force_n"], s["midbody_height_error_m"]]).all()
        and min(s["contact_force_n"]) >= config["minimum_contact_force_n"]
        and abs(s["midbody_height_error_m"]) <= config["maximum_midbody_height_error_m"]
        for s in hold)
