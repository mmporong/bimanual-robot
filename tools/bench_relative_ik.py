#!/usr/bin/env python3
"""관측한 왼손 방향을 잠정 모델 변환에 묶는 소규모 상대 IK 실험.

모터·카메라를 열지 않는다. 보정 JSON/URDF를 바꾸지 않고, 같은 raw↔모델
변환을 시작 FK·IK·관절 범위·양자화 후 FK·경로 표본에 사용한다.
잠정 방향 대응을 실물 좌표 정밀도 검증이나 파지 성공으로 표시하지 않는다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from ik_pick_place import FixedContactChain, JOINTS
from plan_body_side_grasp import (
    URDF_PATH, base_positions, grasp_axes, measure_stage,
    solve_horizontal_endpoint, validate_measurement,
)
from servo.execute_safe_recovery import interpolate_raw


def vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length or any(
            type(item) not in (int, float) or not np.isfinite(item) for item in value):
        raise ValueError(f"{label}: 유한한 숫자 {length}개가 필요합니다")
    return np.array(value, dtype=float)


class BenchJointMap:
    """q_model = q_lerobot + offset. offset은 측정 인증이 아닌 실험 입력이다."""

    def __init__(self, calibration_bytes, mapping):
        self.calibration = json.loads(calibration_bytes)
        if not isinstance(mapping, dict):
            raise ValueError("잠정 변환 객체가 필요합니다")
        if mapping.get("schema") != "bench_provisional_joint_map_v1":
            raise ValueError("잠정 관절 변환 schema 오류")
        if mapping.get("scope") != "left_stock_cup_bench_only" or mapping.get("joint_order") != JOINTS:
            raise ValueError("왼팔 책상 실험 전용 변환이 필요합니다")
        if mapping.get("physical_mapping_precision_verified") is not False:
            raise ValueError("잠정 변환을 실물 정밀도 검증으로 표시할 수 없습니다")
        if mapping.get("physical_grasp_verified") is not False or any(
                value is not False for key, value in mapping.items()
                if key.endswith(("verified", "ready"))):
            raise ValueError("잠정 변환에 파지 성공이나 실행 인증을 기록할 수 없습니다")
        if mapping.get("calibration_sha256") != hashlib.sha256(calibration_bytes).hexdigest():
            raise ValueError("변환과 calibration 해시 불일치")
        if mapping.get("urdf_sha256") != hashlib.sha256(URDF_PATH.read_bytes()).hexdigest():
            raise ValueError("변환과 URDF 해시 불일치")
        provenance = mapping.get("provenance")
        if not isinstance(provenance, dict) or not all(
                isinstance(provenance.get(key), str) and provenance[key].strip()
                for key in ("observed_at", "evidence", "assumptions")):
            raise ValueError("방향 관측 근거와 미검증 가정이 필요합니다")
        keys = ("schema", "scope", "joint_order", "offset_deg", "calibration_sha256",
                "urdf_sha256", "physical_mapping_precision_verified", "physical_grasp_verified")
        self._mapping = json.loads(json.dumps({key: mapping[key] for key in keys}, allow_nan=False))
        self._mapping["provenance"] = {key: provenance[key] for key in ("observed_at", "evidence", "assumptions")}
        self.offset = vector(mapping.get("offset_deg"), 5, "offset_deg")
        if self.offset[:4].tolist() != [0] * 4 or self.offset[4] not in (0, 90):
            raise ValueError("이 실험은 손목 roll 0/90도 후보만 지원합니다")
        self.offset.setflags(write=False)
        ids, ranges = [], []
        for name in JOINTS:
            item = self.calibration[name]
            if any(type(item.get(key)) is not int for key in ("id", "range_min", "range_max")):
                raise ValueError(f"{name}: calibration 정수 오류")
            if not 1 <= item["id"] <= 253 or not 0 <= item["range_min"] < item["range_max"] <= 4095:
                raise ValueError(f"{name}: calibration 범위 오류")
            ids.append(item["id"])
            ranges.append([item["range_min"], item["range_max"]])
        if len(set(ids)) != 5:
            raise ValueError("서보 ID 중복")
        self.ranges = np.array(ranges, dtype=float)
        self.midpoint = self.ranges.mean(axis=1)

    @property
    def mapping(self):
        return json.loads(json.dumps(self._mapping, allow_nan=False))

    def to_model_deg(self, raw):
        values = vector(raw, 5, "raw")
        if np.any(values != np.rint(values)) or np.any(values < self.ranges[:, 0]) or np.any(values > self.ranges[:, 1]):
            raise ValueError("raw는 저장 범위 안의 정수여야 합니다")
        return (values - self.midpoint) * 360 / 4095 + self.offset

    def to_raw(self, model_deg):
        q = vector(model_deg, 5, "model_deg")
        values = np.rint((q - self.offset) * 4095 / 360 + self.midpoint).astype(int).tolist()
        self.to_model_deg(values)
        return values

    def apply_limits(self, chain):
        lower = (self.ranges[:, 0] - self.midpoint) * 360 / 4095 + self.offset
        upper = (self.ranges[:, 1] - self.midpoint) * 360 / 4095 + self.offset
        for name, lo, hi in zip(JOINTS, lower, upper):
            key = f"left_{name}"
            a, b = chain.limits[key]
            bounds = max(a, np.radians(lo)), min(b, np.radians(hi))
            if bounds[0] >= bounds[1]:
                raise ValueError(f"{name}: URDF와 raw 범위의 교집합이 없습니다")
            chain.limits[key] = bounds


def plan_delta(start_raw, joint_map, contact_center_tool_m, delta_base_m):
    """50 mm 이내 상대 목표를 같은 변환으로 푼다. 절대 작업셀 보정은 아니다."""
    delta = vector(delta_base_m, 3, "delta_base_m")
    if np.linalg.norm(delta) > .05:
        raise ValueError("한 번의 상대 목표는 50 mm 이내여야 합니다")
    contact = vector(contact_center_tool_m, 3, "contact_center_tool_m")
    chain = FixedContactChain({"contact_center_tool_m": contact.tolist()})
    joint_map.apply_limits(chain)
    seed = np.radians(joint_map.to_model_deg(start_raw))
    for name, value in zip(JOINTS, seed):
        lower, upper = chain.limits[f"left_{name}"]
        if not lower + np.radians(3) <= value <= upper - np.radians(3):
            raise ValueError(f"{name}: 시작 자세가 관절 한계 3도 여유 밖입니다")
    start = chain.transforms(base_positions("left", seed))["left_tool0"]
    heading, _ = grasp_axes(start, "left")
    heading[2] = 0
    if np.linalg.norm(heading) < 1e-6:
        raise ValueError("현재 모델에서 수평 접근 방향을 구할 수 없습니다")
    heading /= np.linalg.norm(heading)
    target = start[:3, 3] + delta
    q = solve_horizontal_endpoint(
        chain, "left", target, seed, restarts=4,
        position_tolerance_mm=.1, axes_tolerance_deg=.1,
        joint_margin_extra_deg=360 / 4095 / 2 + 1e-6,
    )
    target_raw = joint_map.to_raw(np.degrees(q).tolist())
    quantized = np.radians(joint_map.to_model_deg(target_raw))
    measured = measure_stage(chain, "left", quantized, target, heading)
    if validate_measurement(measured):
        raise ValueError(f"상대 IK의 양자화 후 FK 실패: {measured}")
    waypoints = interpolate_raw(start_raw, target_raw, 5)
    samples = []
    for raw in [start_raw, *waypoints]:
        sample_q = np.radians(joint_map.to_model_deg(raw))
        transforms = chain.transforms(base_positions("left", sample_q))
        tool = transforms["left_tool0"]
        samples.append({"raw_ticks": raw, "model_joint_deg": np.degrees(sample_q).tolist(),
                        "contact_center_m": tool[:3, 3].tolist()})
    return {"schema": "bench_relative_ik_v1", "mapping": joint_map.mapping,
            "start_raw": start_raw, "target_raw": target_raw,
            "contact_center_tool_m": contact.tolist(), "delta_base_m": delta.tolist(),
            "start_contact_m": start[:3, 3].tolist(), "target_contact_m": target.tolist(),
            "endpoint_measurement": measured, "waypoints_raw": waypoints, "samples": samples,
            "hardware_accessed": False, "motion_command_emitted": False,
            "physical_mapping_precision_verified": False, "physical_grasp_verified": False,
            "collision_audit": None, "physical_execution_ready": False}


def audit_self_collision(plan, joint_map, gripper_rad):
    """같은 mapped raw로 왼팔 비인접 메시 충돌을 표본 검사한다."""
    from audit_pick_trajectory import (
        UrdfScene, MOVING_LEFT_LINKS, collision_polydata, collision_hits,
        nonadjacent_self_pairs,
    )
    if type(gripper_rad) not in (int, float) or not np.isfinite(gripper_rad) or not 0 <= gripper_rad <= 1.74533:
        raise ValueError("그리퍼 모델 개구는 0..1.74533 rad여야 합니다")
    if not isinstance(plan, dict) or plan.get("schema") != "bench_relative_ik_v1" or plan.get("mapping") != joint_map.mapping:
        raise ValueError("감사할 계획과 관절 변환이 다릅니다")
    for flag in ("hardware_accessed", "motion_command_emitted", "physical_mapping_precision_verified",
                 "physical_grasp_verified", "physical_execution_ready"):
        if plan.get(flag) is not False:
            raise ValueError("오프라인 상대 IK 계획만 검사합니다")
    samples = plan.get("samples")
    expected = [plan["start_raw"], *interpolate_raw(plan["start_raw"], plan["target_raw"], 5)]
    if not isinstance(samples, list) or len(samples) != len(expected) or not samples:
        raise ValueError("시작부터 목표까지 경로 표본이 필요합니다")
    chain = FixedContactChain({"contact_center_tool_m": vector(plan["contact_center_tool_m"], 3, "contact_center_tool_m").tolist()})
    for sample, raw in zip(samples, expected):
        if not isinstance(sample, dict) or sample.get("raw_ticks") != raw:
            raise ValueError("경로 raw 표본이 명령 보간과 다릅니다")
        q_deg = joint_map.to_model_deg(raw)
        if not np.allclose(vector(sample.get("model_joint_deg"), 5, "model_joint_deg"), q_deg, rtol=0, atol=1e-9):
            raise ValueError("경로 각도가 관절 변환과 다릅니다")
        tcp = chain.transforms(base_positions("left", np.radians(q_deg)))["left_tool0"][:3, 3]
        if not np.allclose(vector(sample.get("contact_center_m"), 3, "contact_center_m"), tcp, rtol=0, atol=1e-9):
            raise ValueError("경로 FK가 관절 변환과 다릅니다")
    scene = UrdfScene(URDF_PATH)
    pairs = nonadjacent_self_pairs(scene)
    hits = []
    for index, sample in enumerate(samples):
        q = np.radians(joint_map.to_model_deg(sample["raw_ticks"]))
        positions = base_positions("left", q)
        positions["left_gripper"] = gripper_rad
        transforms = scene.link_transforms(positions)
        polys = {name: collision_polydata(scene, name, transforms) for name in MOVING_LEFT_LINKS}
        if any(poly is None for poly in polys.values()):
            raise ValueError("왼팔 충돌 메시가 누락됐습니다")
        collisions = collision_hits(polys, pairs)
        if collisions:
            hits.append({"sample": index, "pairs": collisions})
    return {"sampled_self_collision_clear": not hits, "sample_count": len(plan["samples"]),
            "hits": hits, "gripper_rad_assumption": gripper_rad,
            "continuous_collision_verified": False, "external_world_collision_verified": False,
            "physical_execution_ready": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--start-raw", type=int, nargs=5, required=True)
    parser.add_argument("--contact-center-tool-m", type=float, nargs=3, required=True)
    parser.add_argument("--delta-base-m", type=float, nargs=3, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    joint_map = BenchJointMap(args.calibration.read_bytes(), json.loads(args.mapping.read_text()))
    result = plan_delta(args.start_raw, joint_map, args.contact_center_tool_m, args.delta_base_m)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(args.output)


if __name__ == "__main__":
    main()
