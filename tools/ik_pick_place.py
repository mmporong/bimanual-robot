#!/usr/bin/env python3
"""고정 작업셀 왼팔 픽앤플레이스 준비·모의 실행·명시 실행.

기본 경로는 포트를 열지 않는다. 물리 성공은 모터 도달과 별도로 기록한다.
"""
from __future__ import annotations

import argparse
from datetime import date
import hashlib
import json
import math
from pathlib import Path
import signal
import select
import threading
import time

import numpy as np

from plan_body_side_grasp import (Chain, URDF_PATH, solve_horizontal_endpoint,
                                  measure_stage, validate_measurement)
from workcell_preview_motion import leveled_pose
from servo.execute_safe_recovery import (JOINTS, DEFAULT_CALIBRATION, calibration_binding,
    interpolate_raw, read_required, require_write)
from servo.sts_bus import A_POS, A_GOAL, A_SPEED, A_ACCEL, A_TORQUE, A_OFFSET, decode_offset
from dashboard_stop import open_stop_bus
from servo.small_raw_jog import dashboard_monitor
from servo.joint_reference import (joint_deg_to_raw, raw_to_joint_deg,
                                   validate_reference, joint_limits_deg)

PHASES = ["REORIENT_ABOVE", "PREGRASP_ABOVE", "ALIGN_MIDDLE", "APPROACH", "CLOSE", "LIFT",
          "TRANSFER", "LOWER", "OPEN", "WITHDRAW", "CLEAR_ABOVE"]
NAMES = [*JOINTS, "gripper"]
RIGHT_CALIBRATION = DEFAULT_CALIBRATION.with_name("arms_right.json")
NOMINAL_JOINT_CONVENTION = "lerobot_degrees_so101_new_calib"


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def template():
    return {"schema": "fixed_ik_pick_place_v1", "frame": "base_footprint",
        "provenance": {"status": "unmeasured", "measured_at": None, "valid_for": None},
        **{key: None for key in ("cup_center_m", "place_center_m", "table_surface_z_m",
            "cup_height_m", "cup_radius_m", "contact_center_tool_m", "start_joint_deg",
            "right_parked_deg", "right_gripper_m", "right_gripper_percent", "start_gripper_percent",
            "open_percent", "close_percent", "gripper_open_rad", "gripper_close_rad",
            "lift_distance_m", "pregrasp_backoff_m", "pregrasp_height_m", "withdraw_distance_m",
            "reorient_backoff_m", "reorient_height_m", "lowering_offset_m")}}


def validate_config(config):
    if config.get("schema") != "fixed_ik_pick_place_v1" or config.get("frame") != "base_footprint":
        raise ValueError("schema/frame 오류")
    convention = config.get("joint_convention")
    if convention is not None:
        if convention != NOMINAL_JOINT_CONVENTION:
            raise ValueError("지원하지 않는 joint_convention")
        if config.get("joint_reference") is not None:
            raise ValueError("표준 관절 규약과 수동 영점을 함께 적용할 수 없습니다")
    for key in template():
        if key not in config or config[key] is None:
            raise ValueError(f"실측/입력 필요: {key}")
    for key, length in (("cup_center_m", 3), ("place_center_m", 3), ("contact_center_tool_m", 3),
                        ("start_joint_deg", 5), ("right_parked_deg", 5)):
        values = config[key]
        if not isinstance(values, list) or len(values) != length or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError(f"{key}: 유한한 {length}벡터 필요")
    for key in set(template()) - {"schema", "frame", "provenance", "cup_center_m", "place_center_m",
                                  "contact_center_tool_m", "start_joint_deg", "right_parked_deg"}:
        v = config[key]
        if type(v) not in (int, float) or not math.isfinite(v) or v < 0:
            raise ValueError(f"{key}: 유한한 음이 아닌 숫자 필요")
    for key in ("cup_height_m", "cup_radius_m", "lift_distance_m", "pregrasp_backoff_m",
                "pregrasp_height_m", "withdraw_distance_m", "reorient_backoff_m", "reorient_height_m"):
        if config[key] <= 0:
            raise ValueError(f"{key}: 양수 필요")
    if not 0 <= config["close_percent"] < config["open_percent"] <= 100 or not all(
            0 <= config[k] <= 100 for k in ("start_gripper_percent", "right_gripper_percent")):
        raise ValueError("그리퍼 percentage 범위 오류")
    if not 0 <= config["gripper_close_rad"] < config["gripper_open_rad"] <= 1.74533:
        raise ValueError("URDF 그리퍼 rad 범위 오류")
    if not .010 <= config["right_gripper_m"] <= .0433:
        raise ValueError("오른손 gripper m 범위 오류")
    if config["lowering_offset_m"] > .003:
        raise ValueError("감독형 하강 여유는 3 mm 이하여야 합니다")
    for key in ("cup_center_m", "place_center_m"):
        if not math.isclose(config[key][2] - config["cup_height_m"] / 2,
                            config["table_surface_z_m"], abs_tol=1e-6):
            raise ValueError(f"{key}: 컵 중간·작업대 높이 불일치")
    provenance = config["provenance"]
    if not isinstance(provenance, dict) or provenance.get("status") not in {"user_measured", "simulation_fixture"}:
        raise ValueError("provenance.status는 user_measured 또는 simulation_fixture")
    if not provenance.get("valid_for") or not provenance.get("measured_at"):
        raise ValueError("provenance 출처 필요")
    date.fromisoformat(provenance["measured_at"])
    return config


class FixedContactChain(Chain):
    def __init__(self, config):
        super().__init__(URDF_PATH)
        self.offset_m = np.asarray(config["contact_center_tool_m"])

    def transforms(self, positions):
        transforms = super().transforms(positions)
        tool = transforms["left_tool0"].copy()
        tool[:3, 3] += tool[:3, :3] @ self.offset_m
        transforms["left_tool0"] = tool
        return transforms


def apply_joint_reference_limits(chain, calibration_bytes, reference):
    """현재 chain에 URDF와 선택한 각도 규약의 raw 한계 교집합을 적용한다."""
    if reference is None:
        calibration = json.loads(calibration_bytes)
        raw_for([0] * 5, 50, calibration_bytes)
        hardware_limits = [
            [-(calibration[n]["range_max"] - calibration[n]["range_min"]) * 180 / 4095,
             (calibration[n]["range_max"] - calibration[n]["range_min"]) * 180 / 4095]
            for n in JOINTS]
    else:
        hardware_limits = joint_limits_deg(calibration_bytes, reference)
    for name, hardware_deg in zip(JOINTS, hardware_limits):
        key = f"left_{name}"
        nominal, hardware = chain.limits[key], np.radians(hardware_deg)
        lower, upper = max(nominal[0], hardware[0]), min(nominal[1], hardware[1])
        if lower >= upper:
            raise ValueError(f"{name}: URDF와 실물 가동 범위의 교집합이 없습니다")
        chain.limits[key] = (lower, upper)


def raw_for(q_deg, percent, calibration_bytes, joint_reference=None):
    if type(percent) not in (int, float) or not math.isfinite(percent) or not 0 <= percent <= 100:
        raise ValueError("그리퍼 percentage 범위 오류")
    plan = {"start_joint_deg": q_deg, "recovery": {"target_joint_deg": q_deg}}
    if joint_reference is None:
        raw = calibration_binding(plan, calibration_bytes)["target_raw"]
    else:
        validate_reference(joint_reference, calibration_bytes, hashlib.sha256(URDF_PATH.read_bytes()).hexdigest())
        raw = joint_deg_to_raw(q_deg, calibration_bytes, joint_reference)
    cal = json.loads(calibration_bytes)
    item = cal["gripper"]
    if any(type(item[k]) is not int for k in ("id", "range_min", "range_max", "drive_mode")):
        raise ValueError("gripper calibration 정수 오류")
    if not 1 <= item["id"] <= 253 or not 0 <= item["range_min"] < item["range_max"] <= 4095 or item["drive_mode"] not in (0, 1):
        raise ValueError("gripper calibration 범위 오류")
    if item["id"] in [cal[name]["id"] for name in JOINTS]:
        raise ValueError("gripper ID 중복")
    normalized = (100 - percent if item["drive_mode"] else percent) / 100
    return [*raw, int(normalized * (item["range_max"] - item["range_min"]) + item["range_min"])]


def decode_raw(raw, cal, config, calibration_bytes=None):
    reference = config.get("joint_reference")
    if reference is None:
        q_deg = [(value - (cal[name]["range_min"] + cal[name]["range_max"]) / 2) * 360 / 4095
                 for name, value in zip(JOINTS, raw)]
    else:
        if calibration_bytes is None:
            raise ValueError("joint_reference 대조에는 원본 calibration bytes 필요")
        validate_reference(reference, calibration_bytes, hashlib.sha256(URDF_PATH.read_bytes()).hexdigest())
        q_deg = raw_to_joint_deg(raw[:5], calibration_bytes, reference)
    item = cal["gripper"]
    # 물리 열림과 URDF rad의 두 대응점은 설정에서 제공한다. 무보정 자동 등치는 하지 않는다.
    def endpoint(percent):
        normalized = (100-percent if item["drive_mode"] else percent)/100
        return int(normalized*(item["range_max"]-item["range_min"])+item["range_min"])
    close_ticks, open_ticks = endpoint(config["close_percent"]), endpoint(config["open_percent"])
    if close_ticks == open_ticks:
        raise ValueError("그리퍼 대응점이 같은 raw tick입니다")
    fraction = (raw[5]-close_ticks)/(open_ticks-close_ticks)
    angle_rad = config["gripper_close_rad"] + fraction * (config["gripper_open_rad"] - config["gripper_close_rad"])
    return q_deg, angle_rad


def require_supported_physical_mapping(config):
    """지원하는 표준 규약의 명시 선택과 정밀도 보증을 구분한다.

    LeRobot degrees와 SO101 new-calib은 가동 범위 중점 영점을 사용한다.
    이 선택은 실측 정밀도 인증이 아니다. 경로 감사·작업셀 관측·현재 보정
    readback은 별도로 요구한다. 수동 영점은 계속 오프라인 비교에만 쓴다.
    """
    if config.get("joint_reference") is not None:
        raise ValueError("수동 영점은 실물 실행에 사용할 수 없습니다")
    if config.get("joint_convention") != NOMINAL_JOINT_CONVENTION:
        raise ValueError("실행 계획에 joint_convention을 명시해야 합니다")


def prepare(config, calibration_bytes, right_calibration_bytes=None):
    validate_config(config)
    chain = FixedContactChain(config)
    apply_joint_reference_limits(chain, calibration_bytes, config.get("joint_reference"))
    cup_m, place_m = np.array(config["cup_center_m"]), np.array(config["place_center_m"])
    above_m = np.array([0, 0, config["lift_distance_m"]])
    back_m = np.array([-config["pregrasp_backoff_m"], 0, 0])
    targets = [cup_m + [-config["reorient_backoff_m"], 0, config["reorient_height_m"]],
               cup_m + back_m + [0, 0, config["pregrasp_height_m"]], cup_m + back_m,
               cup_m, cup_m, cup_m + above_m, place_m + above_m,
               place_m + [0, 0, -config["lowering_offset_m"]],
               place_m + [0, 0, -config["lowering_offset_m"]],
               place_m + [-config["withdraw_distance_m"], 0, 0],
               place_m + [-config["withdraw_distance_m"], 0, config["lift_distance_m"]]]
    # IK 초기값에서만 손목 수평 해를 선택한다. 실제 시작 자세/명령은 바꾸지 않는다.
    seed = np.radians(leveled_pose(chain, "left", config["start_joint_deg"]))
    start_raw = raw_for(config["start_joint_deg"], config["start_gripper_percent"], calibration_bytes, config.get("joint_reference"))
    poses = []
    for index, (name, target_m) in enumerate(zip(PHASES, targets)):
        # 첫 접근에서는 접힌 시작 자세를 수평 끝점으로 재정렬한다.
        # 이후 구간은 Cartesian 중간점을 풀어 수평 운반·하강을 유지한다.
        if name in {"LIFT", "TRANSFER", "LOWER", "WITHDRAW", "CLEAR_ABOVE"}:
            previous_m = chain.transforms(dict(zip([f"left_{n}" for n in JOINTS], seed)))["left_tool0"][:3, 3]
            steps = max(1, math.ceil(float(np.linalg.norm(target_m - previous_m)) / .005))
            points = [previous_m + (target_m - previous_m) * i / steps for i in range(1, steps + 1)]
        else:
            points = [target_m]
        for point_m in points:
            # 정수 tick 반올림 뒤에도 기존 3도 관절 여유를 유지한다.
            seed = solve_horizontal_endpoint(chain, "left", point_m, seed, restarts=1, iterations=240,
                                             position_tolerance_mm=.1, axes_tolerance_deg=.1,
                                             joint_margin_extra_deg=360 / 4095 / 2 + 1e-6)
            measured = measure_stage(chain, "left", seed, point_m, np.array([1, 0, 0]))
            reasons = validate_measurement(measured)
            if reasons:
                raise ValueError(f"{name}: IK 미통과 {reasons}")
            percent = config["close_percent"] if name in {"CLOSE", "LIFT", "TRANSFER", "LOWER"} else config["open_percent"]
            q_deg = np.degrees(seed).tolist()
            raw_ticks = raw_for(q_deg, percent, calibration_bytes, config.get("joint_reference"))
            quantized_deg, _ = decode_raw(raw_ticks, json.loads(calibration_bytes), config, calibration_bytes)
            raw_measurement = measure_stage(chain, "left", np.radians(quantized_deg), point_m, np.array([1, 0, 0]))
            if validate_measurement(raw_measurement):
                raise ValueError(f"{name}: 양자화 후 FK 미통과")
            poses.append({"phase": name, "joint_deg": q_deg, "target_m": point_m.tolist(),
                          "gripper_percent": percent, "raw_ticks": raw_ticks,
                          "measurement": measured, "raw_measurement": raw_measurement})
    cal = json.loads(calibration_bytes)
    right_calibration_bytes = right_calibration_bytes or RIGHT_CALIBRATION.read_bytes()
    return {"schema": "fixed_ik_execution_v1", "config": config, "config_sha256": digest(config),
        "calibration_sha256": hashlib.sha256(calibration_bytes).hexdigest(),
        "urdf_sha256": hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(),
        "servo_ids": [cal[n]["id"] for n in NAMES], "start_raw": start_raw, "poses": poses,
        "right_calibration_sha256": hashlib.sha256(right_calibration_bytes).hexdigest(),
        "right_expected_raw": raw_for(config["right_parked_deg"], config["right_gripper_percent"], right_calibration_bytes),
        "motion_command_emitted": False, "hardware_accessed": False,
        "physical_grasp_verified": False, "collision_audit": None}


def audit(plan, calibration_bytes):
    from audit_pick_trajectory import (UrdfScene, audit_state, collision_polydata, collision_hits,
        nonadjacent_self_pairs, MOVING_LEFT_LINKS, STATIC_CRITICAL_LINKS, joint_margin_deg)
    scene, chain = UrdfScene(URDF_PATH), FixedContactChain(plan["config"])
    cal, config = json.loads(calibration_bytes), plan["config"]
    apply_joint_reference_limits(chain, calibration_bytes, config.get("joint_reference"))
    pairs = nonadjacent_self_pairs(scene) + [(a, b) for a in MOVING_LEFT_LINKS for b in STATIC_CRITICAL_LINKS]
    previous = plan["start_raw"]
    failures, count = [], 0
    # 시작 자세도 검사한다. 샘플 사이의 충돌 부재를 보증하지 않는다.
    for pose in [{"phase": "START", "raw_ticks": previous}, *plan["poses"]]:
        for raw in interpolate_raw(previous, pose["raw_ticks"], 10):
            count += 1
            q_deg, gripper_rad = decode_raw(raw, cal, config, calibration_bytes)
            positions = dict(zip([f"left_{n}" for n in JOINTS], np.radians(q_deg)))
            positions.update(zip([f"right_{n}" for n in JOINTS], np.radians(config["right_parked_deg"])))
            positions.update(left_gripper=gripper_rad, right_finger1_joint=config["right_gripper_m"],
                             right_finger2_joint=config["right_gripper_m"])
            transforms = scene.link_transforms(positions)
            polys = {n: collision_polydata(scene, n, transforms) for n in set(MOVING_LEFT_LINKS + STATIC_CRITICAL_LINKS)}
            if any(p is None for p in polys.values()):
                raise ValueError("충돌 형상 누락")
            hits = collision_hits(polys, pairs)
            clearance = audit_state(scene, positions)
            margin_deg = joint_margin_deg(chain, np.radians(q_deg))
            if hits or not clearance["passes"] or margin_deg < 3 or not 0 <= gripper_rad <= 1.74533:
                failures.append({"sample": count, "phase": pose["phase"], "pairs": hits,
                    "cross_arm_pairs": clearance["triangle_collision_pairs_cross_arm"], "margin_deg": margin_deg})
                return {"robot_sampled_collision_pass": False, "samples": count,
                        "max_raw_step_ticks": 10, "failures": failures, "fully_audited": False,
                        "object_collision_validated": False, "continuous_collision_validated": False}
        previous = pose["raw_ticks"]
    return {"robot_sampled_collision_pass": not failures, "samples": count,
            "max_raw_step_ticks": 10, "failures": failures,
            "fully_audited": True,
            "object_collision_validated": False, "continuous_collision_validated": False}


def verify_packet(plan, calibration_bytes, right_calibration_bytes=None):
    config = validate_config(plan["config"])
    if plan.get("schema") != "fixed_ik_execution_v1" or plan.get("motion_command_emitted") is not False:
        raise ValueError("실행 계획 schema 오류")
    if plan["calibration_sha256"] != hashlib.sha256(calibration_bytes).hexdigest() or plan["urdf_sha256"] != hashlib.sha256(URDF_PATH.read_bytes()).hexdigest() or plan["config_sha256"] != digest(config):
        raise ValueError("준비 뒤 calibration/URDF/config 변경")
    cal = json.loads(calibration_bytes)
    if plan["servo_ids"] != [cal[n]["id"] for n in NAMES] or plan["start_raw"] != raw_for(config["start_joint_deg"], config["start_gripper_percent"], calibration_bytes, config.get("joint_reference")):
        raise ValueError("시작 상태/ID 불일치")
    phases = [p["phase"] for i, p in enumerate(plan["poses"]) if i == 0 or p["phase"] != plan["poses"][i-1]["phase"]]
    if phases != PHASES:
        raise ValueError("픽앤플레이스 단계 누락/순서 오류")
    for pose in plan["poses"]:
        if pose["raw_ticks"] != raw_for(pose["joint_deg"], pose["gripper_percent"], calibration_bytes, config.get("joint_reference")):
            raise ValueError("raw 목표 불일치")
    expected = prepare(config, calibration_bytes, right_calibration_bytes)
    if any(plan[key] != expected[key] for key in ("right_calibration_sha256", "right_expected_raw")) or len(plan["poses"]) != len(expected["poses"]):
        raise ValueError("IK 중간점/오른팔 기준이 준비 입력과 다릅니다")
    for actual, reference in zip(plan["poses"], expected["poses"]):
        if any(actual[key] != reference[key] for key in ("phase", "raw_ticks", "gripper_percent")) or not np.allclose(
                actual["joint_deg"], reference["joint_deg"], rtol=0, atol=1e-5) or not np.allclose(
                actual["target_m"], reference["target_m"], rtol=0, atol=1e-9):
            raise ValueError("IK 중간점이 준비 입력과 다릅니다")
    return cal


class MockBus:
    def __init__(self, ids, raw):
        self.positions = dict(zip(ids, raw))
        self.reg = {(sid, address): value for sid, pos in zip(ids, raw)
                    for address, value in ((A_POS, pos), (A_GOAL, pos), (A_TORQUE, 0), (A_SPEED, 300), (A_ACCEL, 10))}
        self.writes = []

    def read(self, sid, address, size=1):
        if address == A_POS and self.reg[sid, A_TORQUE]:
            delta = self.reg[sid, A_GOAL] - self.positions[sid]
            self.positions[sid] += max(-8, min(8, delta))
        return self.positions[sid] if address == A_POS else self.reg.get((sid, address))

    def write(self, sid, address, value, size=1):
        self.writes.append((sid, address, value))
        self.reg[sid, address] = value
        return True

    def close(self):
        pass


def stop_restore(bus, ids, profiles):
    errors = []
    # 먼저 모든 축에 OFF를 전송한다. 한 축 실패가 나머지 축의 정지를 막지 않는다.
    for sid in ids:
        try:
            require_write(bus, sid, A_TORQUE, 0)
        except BaseException as exc:
            errors.append(str(exc))
    # 해제한 뒤 현재 위치를 goal로 고정하고 기존 프로파일을 복원한다.
    for sid in ids:
        try:
            pos = read_required(bus, sid, A_POS, 2)
            require_write(bus, sid, A_GOAL, pos, 2)
        except BaseException as exc:
            errors.append(str(exc))
    for sid, (speed, accel) in zip(ids, profiles):
        for address, value, size in ((A_SPEED, speed, 2), (A_ACCEL, accel, 1)):
            try:
                require_write(bus, sid, address, value, size)
                if read_required(bus, sid, address, size) != value:
                    raise RuntimeError(f"ID {sid}: 복원 readback 실패 {address}")
            except BaseException as exc:
                errors.append(str(exc))
    # 복원 쓰기 뒤 OFF를 최대 3회 다시 쓰고 매회 readback한다.
    pending = list(ids)
    last_errors = {}
    for _ in range(3):
        for sid in list(pending):
            try:
                require_write(bus, sid, A_TORQUE, 0)
            except BaseException as exc:
                last_errors[sid] = str(exc)
        for sid in list(pending):
            try:
                if read_required(bus, sid, A_TORQUE) == 0:
                    pending.remove(sid)
                    last_errors.pop(sid, None)
                else:
                    last_errors[sid] = f"ID {sid}: torque off readback 실패"
            except BaseException as exc:
                last_errors[sid] = str(exc)
        if not pending:
            break
    errors.extend(last_errors[sid] for sid in ids if sid in pending)
    return errors


def run_sequence(bus, plan, confirm, *, sleep=time.sleep, clock=time.monotonic,
                 monitor=lambda: None, preflight=lambda: None,
                 raw_step_ticks=10, arrival_tolerance_ticks=15):
    if type(raw_step_ticks) is not int or not 1 <= raw_step_ticks <= 35:
        raise ValueError("raw_step_ticks: 1..35 정수가 필요합니다")
    if type(arrival_tolerance_ticks) is not int or not 1 <= arrival_tolerance_ticks <= 15:
        raise ValueError("arrival_tolerance_ticks: 1..15 정수가 필요합니다")
    ids = plan["servo_ids"]
    profiles, events = [], []
    result = {"sequence_completed": False, "physical_grasp_verified": False,
              "physical_task_verified": False, "phases_completed": [], "events": events,
              "temperature_load_read": False, "failure": None, "command_write_attempted": False}
    result["physical_mapping_precision_verified"] = False
    result.update(raw_step_ticks=raw_step_ticks,
                  arrival_tolerance_ticks=arrival_tolerance_ticks)
    result.update(plan_sha256=digest(plan), config_sha256=plan["config_sha256"],
                  calibration_sha256=plan["calibration_sha256"], urdf_sha256=plan["urdf_sha256"],
                  failure_phase=None)
    phase = "PREFLIGHT"
    mutated = False
    try:
        preflight()
        monitor()
        positions = [read_required(bus, sid, A_POS, 2) for sid in ids]
        if max(abs(a-b) for a, b in zip(positions, plan["start_raw"])) > 15:
            raise RuntimeError("start_state_changed")
        if any(read_required(bus, sid, A_TORQUE) for sid in ids):
            raise RuntimeError("start_torque_not_off")
        profiles = [(read_required(bus, sid, A_SPEED, 2), read_required(bus, sid, A_ACCEL)) for sid in ids]
        mutated = True
        result["command_write_attempted"] = True
        for sid, pos in zip(ids, positions):
            require_write(bus, sid, A_GOAL, pos, 2)
            require_write(bus, sid, A_SPEED, 150, 2)
            require_write(bus, sid, A_ACCEL, 10)
        for sid in ids:
            require_write(bus, sid, A_TORQUE, 1)
        if [read_required(bus, sid, A_TORQUE) for sid in ids] != [1]*6:
            raise RuntimeError("torque_enable_failed")
        previous = positions
        for index, pose in enumerate(plan["poses"]):
            phase = pose["phase"]
            if index == 0 or phase != plan["poses"][index-1]["phase"]:
                if not confirm(phase):
                    raise RuntimeError(f"observation_rejected:{phase}")
            for goals in interpolate_raw(previous, pose["raw_ticks"], raw_step_ticks):
                monitor()
                before = [read_required(bus, sid, A_POS, 2) for sid in ids]
                baseline = [abs(a-b) for a, b in zip(goals, before)]
                for sid, goal in zip(ids, goals):
                    require_write(bus, sid, A_GOAL, goal, 2)
                if [read_required(bus, sid, A_GOAL, 2) for sid in ids] != goals:
                    raise RuntimeError(f"goal_readback_failed:{phase}")
                result["last_goal_raw"] = goals.copy()
                last, last_motion_s = before, [clock()]*6
                deadline_s = clock() + 8
                while True:
                    sleep(.03)
                    monitor()
                    if [read_required(bus, sid, A_TORQUE) for sid in ids] != [1]*len(ids):
                        raise RuntimeError(f"torque_released:{phase}")
                    now = [read_required(bus, sid, A_POS, 2) for sid in ids]
                    errors = [abs(a-b) for a, b in zip(goals, now)]
                    result["last_observed_raw"] = now.copy()
                    result["max_position_error_ticks"] = max(errors)
                    if any(e > b + 10 for e, b in zip(errors, baseline)):
                        raise RuntimeError(f"position_diverged:{phase}")
                    for i, (a, b) in enumerate(zip(now, last)):
                        if abs(a-b) > 2:
                            last_motion_s[i], last[i] = clock(), a
                    if max(errors) <= arrival_tolerance_ticks:
                        break
                    if any(e > arrival_tolerance_ticks and clock()-t > .5 for e, t in zip(errors, last_motion_s)):
                        raise RuntimeError(f"position_stall:{phase}")
                    if clock() > deadline_s:
                        raise RuntimeError(f"arrival_timeout:{phase}")
                events.append({"phase": phase, "raw_ticks": now, "timestamp_monotonic_s": clock()})
            previous = pose["raw_ticks"]
            if index == len(plan["poses"])-1 or phase != plan["poses"][index+1]["phase"]:
                result["phases_completed"].append(phase)
        result["sequence_completed"] = True
    except BaseException as exc:
        result["failure"] = f"{type(exc).__name__}:{exc}"
        result["failure_phase"] = phase
    finally:
        result["stop_errors"] = stop_restore(bus, ids, profiles) if mutated else []
        if result["stop_errors"]:
            result["sequence_completed"] = False
    return result


def snapshot(bus, calibration_bytes, joint_reference=None):
    cal = json.loads(calibration_bytes)
    raw_for([0]*5, 50, calibration_bytes)
    raw, torque = [], []
    for name in NAMES:
        item = cal[name]
        sid = item["id"]
        if decode_offset(read_required(bus, sid, A_OFFSET, 2)) != item["homing_offset"]:
            raise ValueError(f"{name}: 실물 offset과 저장 calibration 불일치")
        value = read_required(bus, sid, A_POS, 2)
        if not item["range_min"] <= value <= item["range_max"]:
            raise ValueError(f"{name}: raw range 밖")
        raw.append(value)
        torque.append(read_required(bus, sid, A_TORQUE))
    q_deg = [(v - (cal[n]["range_min"] + cal[n]["range_max"])/2)*360/4095 for n, v in zip(JOINTS, raw)]
    if joint_reference is not None:
        validate_reference(joint_reference, calibration_bytes, hashlib.sha256(URDF_PATH.read_bytes()).hexdigest())
        q_deg = raw_to_joint_deg(raw[:5], calibration_bytes, joint_reference)
    item = cal["gripper"]
    percent = (raw[5]-item["range_min"])*100/(item["range_max"]-item["range_min"])
    return {"raw_ticks": raw, "torque": torque, "joint_deg": q_deg,
            **({"joint_reference": joint_reference} if joint_reference is not None else {}),
            "gripper_percent": 100-percent if item["drive_mode"] else percent,
            "hardware_accessed": True, "motion_command_emitted": False,
            "calibration_sha256": hashlib.sha256(calibration_bytes).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("template", "prepare", "check", "mock", "snapshot", "execute"))
    parser.add_argument("--config", type=Path)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--right-calibration", type=Path, default=RIGHT_CALIBRATION)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port")
    parser.add_argument("--right-port")
    parser.add_argument("--joint-reference", type=Path, help="snapshot/prepare에 적용할 실물 정렬 기준")
    parser.add_argument("--joint-convention", choices=(NOMINAL_JOINT_CONVENTION,),
                        help="prepare에서 표준 LeRobot degrees/SO101 new-calib 규약을 명시")
    parser.add_argument("--ik-only", action="store_true", help="prepare에서 IK packet만 만들고 CPU 감사를 미룸")
    parser.add_argument("--observed-workcell", action="store_true",
                        help="컵·작업대 경로를 관측하며 단계 확인 방식으로 실행")
    parser.add_argument("--stop-status-url", default="http://127.0.0.1:8770/api/stop/status",
                        help="execute 중 합성할 대시보드 정지 상태 API")
    args = parser.parse_args()
    protected = {p.resolve() for p in (args.config, args.plan, args.calibration, args.right_calibration, args.joint_reference, URDF_PATH) if p}
    if args.joint_reference and args.mode not in {"snapshot", "prepare"}:
        parser.error("joint-reference는 snapshot/prepare에서만 지정합니다. 실행은 packet에 묶인 기준을 사용합니다")
    if args.joint_convention and args.mode != "prepare":
        parser.error("joint-convention은 prepare에서만 지정합니다. 실행은 packet의 규약을 사용합니다")
    if args.output.resolve() in protected or args.output.exists():
        parser.error("출력은 기존 파일/입력을 덮어쓸 수 없습니다")
    if args.mode == "template":
        result = template()
    else:
        calibration_bytes = args.calibration.read_bytes()
        right_calibration_bytes = args.right_calibration.read_bytes()
        joint_reference = json.loads(args.joint_reference.read_text()) if args.joint_reference else None
        if joint_reference is not None:
            validate_reference(joint_reference, calibration_bytes, hashlib.sha256(URDF_PATH.read_bytes()).hexdigest())
        if args.mode == "snapshot":
            if not args.port:
                parser.error("snapshot에는 --port 필요")
            bus = open_stop_bus(args.port)
            try:
                result = snapshot(bus, calibration_bytes, joint_reference)
            finally:
                bus.close()
        elif args.mode == "prepare":
            if args.config is None:
                parser.error("--config 필요")
            config = json.loads(args.config.read_text())
            if args.joint_convention:
                if config.get("joint_convention", args.joint_convention) != args.joint_convention:
                    parser.error("config와 CLI joint_convention 불일치")
                config["joint_convention"] = args.joint_convention
            if joint_reference is not None:
                if "joint_reference" in config and config["joint_reference"] != joint_reference:
                    raise ValueError("config와 CLI joint_reference 불일치")
                config["joint_reference"] = joint_reference
            result = prepare(config, calibration_bytes, right_calibration_bytes)
            result["collision_audit"] = ({"robot_sampled_collision_pass": False,
                "fully_audited": False, "reason": "not_run", "object_collision_validated": False,
                "continuous_collision_validated": False} if args.ik_only else audit(result, calibration_bytes))
            result["packet_sha256"] = digest(result)
        else:
            if args.plan is None:
                parser.error("--plan 필요")
            plan = json.loads(args.plan.read_text())
            fingerprint = plan.pop("packet_sha256", None)
            if fingerprint != digest(plan):
                parser.error("계획 또는 감사 결과가 준비 뒤 변경됐습니다")
            verify_packet(plan, calibration_bytes, right_calibration_bytes)
            if args.mode == "check":
                result = {"offline_binding_pass": True, "hardware_accessed": False,
                          "collision_audit": audit(plan, calibration_bytes)}
            else:
                mock = args.mode == "mock"
                if not mock:
                    try:
                        require_supported_physical_mapping(plan["config"])
                    except ValueError as exc:
                        parser.error(str(exc))
                if not mock and (not args.port or not args.right_port or
                    Path(args.port).resolve() == Path(args.right_port).resolve() or not args.observed_workcell or
                    plan["config"]["provenance"]["status"] != "user_measured" or
                    not audit(plan, calibration_bytes)["robot_sampled_collision_pass"]):
                    parser.error("실행에는 실측 설정·좌우 별도 포트·작업셀 관측·로봇 경로 감사 통과 필요")
                right_bus = None if mock else open_stop_bus(args.right_port)
                if right_bus is not None:
                    try:
                        right_state = snapshot(right_bus, right_calibration_bytes)
                        if max(abs(a-b) for a, b in zip(right_state["raw_ticks"], plan["right_expected_raw"])) > 15:
                            raise ValueError("오른팔 주차 자세 불일치")
                    except BaseException:
                        right_bus.close()
                        raise
                try:
                    bus = MockBus(plan["servo_ids"], plan["start_raw"]) if mock else open_stop_bus(args.port)
                except BaseException:
                    if right_bus is not None:
                        right_bus.close()
                    raise
                interrupted = threading.Event()
                def monitor():
                    if interrupted.is_set():
                        raise KeyboardInterrupt("실행 중단")
                    if mock:
                        return
                    dashboard_monitor(args.stop_status_url)
                    state = snapshot(right_bus, right_calibration_bytes)
                    if max(abs(a-b) for a, b in zip(state["raw_ticks"], plan["right_expected_raw"])) > 15 or state["torque"] != right_state["torque"]:
                        raise RuntimeError("right_parked_state_changed")
                    if interrupted.is_set():
                        raise KeyboardInterrupt("실행 중단")
                def confirm(phase):
                    if mock:
                        return True
                    monitor()
                    print(f"{phase}: 컵·책상·반대 팔 경로 확인 후 Enter, 중단 q (30초 제한):", flush=True)
                    import sys
                    deadline = time.monotonic() + 30
                    while not interrupted.is_set():
                        monitor()
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            return False
                        available, _, _ = select.select([sys.stdin], [], [], min(.2, remaining))
                        if available:
                            value = sys.stdin.readline()
                            return value != "" and value.strip() == ""
                    raise KeyboardInterrupt("실행 중단")
                def interrupt(_signum, _frame):
                    # cleanup을 신호 예외로 끊지 않고 실행 루프에 취소를 전달한다.
                    interrupted.set()
                old_handlers = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
                try:
                    result = run_sequence(bus, plan, confirm, sleep=(lambda _: None) if mock else time.sleep,
                        monitor=monitor, preflight=(lambda: None) if mock else lambda: snapshot(bus, calibration_bytes))
                    result.update(hardware_accessed=not mock,
                                  motion_command_emitted=not mock and result["command_write_attempted"], mock=mock)
                finally:
                    bus.close()
                    if right_bus is not None:
                        right_bus.close()
                    for s, handler in old_handlers.items():
                        signal.signal(s, handler)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    print(args.output)
    return 2 if result.get("failure") or result.get("stop_errors") or (
        "collision_audit" in result and not result["collision_audit"]["robot_sampled_collision_pass"]
        and not (args.mode == "prepare" and args.ik_only)) else 0


if __name__ == "__main__":
    raise SystemExit(main())
