#!/usr/bin/env python3
"""준비한 6축 raw 목표를 Isaac 순정 죠 접촉 시험에 연결한다.

실물 포트에 접근하지 않는다. 물리 드라이브 시간·힘은 실물 추종의 보증이 아니다.
"""
import argparse
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import cup_contact_model as contact
import ik_pick_place as pick
from simulate_cup_contact import run


def simulation_plan(packet, calibration_bytes):
    config = packet["config"]
    cal = json.loads(calibration_bytes)
    start_deg, start_rad = pick.decode_raw(packet["start_raw"], cal, config, calibration_bytes)
    poses = [{"name": "RESET", "joint_deg": start_deg, "gripper_rad": start_rad, "duration_s": 1.0}]
    holds = {"CLOSE": ("CONTACT_HOLD", 1.0), "LIFT": ("LIFT_HOLD", 2.0),
             "LOWER": ("TABLE_SETTLE", 1.0), "OPEN": ("RELEASE_HOLD", 1.0),
             "CLEAR_ABOVE": ("PLACE_HOLD", 2.0)}
    for index, pose in enumerate(packet["poses"]):
        q_deg, angle_rad = pick.decode_raw(pose["raw_ticks"], cal, config, calibration_bytes)
        previous = poses[-1]
        duration_s = max(.2, float(np.max(np.abs(np.array(q_deg)-previous["joint_deg"]))) / 20,
                         abs(angle_rad-previous["gripper_rad"])/.65)
        if pose["phase"] in {"REORIENT_ABOVE", "PREGRASP_ABOVE", "ALIGN_MIDDLE", "APPROACH"}:
            duration_s = max(duration_s, 3.0 if pose["phase"] == "REORIENT_ABOVE" else 2.0)
        poses.append({"name": pose["phase"], "joint_deg": q_deg, "gripper_rad": angle_rad,
                      "duration_s": duration_s, "target_m": pose["target_m"]})
        if pose["phase"] in holds and (index == len(packet["poses"])-1 or
                                       packet["poses"][index+1]["phase"] != pose["phase"]):
            name, duration_s = holds[pose["phase"]]
            poses.append({**poses[-1], "name": name, "duration_s": duration_s})
    return {"poses": poses, "right_parked_deg": config["right_parked_deg"],
            "right_gripper_m": config["right_gripper_m"],
            "parked_joint_deg": config["right_parked_deg"], "active_side": "left",
            "placement_enabled": True, "gripper_unit": "rad", "tcp_frame": "left_contact_center",
            "model_calibrated": False, "continuous_collision_validated": False,
            "hardware_accessed": False, "il_dataset_used": False,
            "execution_packet_sha256": pick.digest(packet), "command_source": "quantized_raw_targets"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, default=pick.DEFAULT_CALIBRATION)
    parser.add_argument("--right-calibration", type=Path, default=pick.RIGHT_CALIBRATION)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    packet = json.loads(args.plan.read_text())
    fingerprint = packet.pop("packet_sha256", None)
    if fingerprint != pick.digest(packet):
        parser.error("변경된 실행 packet")
    calibration_bytes = args.calibration.read_bytes()
    pick.verify_packet(packet, calibration_bytes, args.right_calibration.read_bytes())
    config = contact.load_config(contact.ROOT / "config/simulation/cup_stock_so101_experiment.json")
    for key in ("cup_center_m", "table_surface_z_m", "cup_height_m", "cup_radius_m",
                "contact_center_tool_m", "lift_distance_m", "gripper_open_rad", "gripper_close_rad"):
        config[key] = copy.deepcopy(packet["config"][key])
    config["table_center_m"][2] = config["table_surface_z_m"]-config["table_size_m"][2]/2
    config["placement_center_m"] = packet["config"]["place_center_m"]
    config["description"] = "Fixed IK packet raw goals; stock jaw; assumed contact frame/friction; no hardware"
    backend = SimpleNamespace(**{key: getattr(contact, key) for key in (
        "SIDE", "GRIPPER_UNIT", "OBJECT_LABEL", "FINGER_LINKS", "GRIPPER_JOINTS", "GRIPPER_KEY", "CONTACT_FRAME",
        "prepare_model", "sample_plan")})
    backend.__file__ = __file__
    backend.EXTRA_TOOL_FILES = ("ik_pick_place.py",)
    backend.load_config = lambda path: copy.deepcopy(config)
    backend.make_plan = lambda model, config, **kwargs: simulation_plan(packet, calibration_bytes)
    options = SimpleNamespace(config=args.plan, output_dir=args.output_dir,
        headless=args.headless, record=False, stay_open=False, plan_only=False,
        cup_mass_kg=None, spawn_y_offset_mm=None, spawn_x_offset_mm=None, recover=False, place=True)
    run(options, backend)


if __name__ == "__main__":
    main()
