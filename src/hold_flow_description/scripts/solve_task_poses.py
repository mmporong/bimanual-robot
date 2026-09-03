#!/usr/bin/env python3
"""현재 URDF의 FK로 운반·붓기 자세를 다시 푼다.

기존 YAML의 관절값은 폐기한 이전 팔 체인용 legacy 값이라 현재 SO-101 체인에서는
양팔이 서로 닿는다. 여기서는 별도 기구 모델을 두지 않고 **커밋된 URDF를 그대로
읽어** FK를 만들고, 감쇠 최소자승 IK로 tool0 목표를 푼다. 모델이 하나뿐이라
FK와 IK가 어긋날 수 없다.

결과는 JSON 으로 출력한다. YAML 반영은 사람이 검토한 뒤 수동으로 한다.

사용법:
  python3 scripts/solve_task_poses.py
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf/hold_flow.urdf"
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_2.yaml"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
FINGER_OPEN_M = 0.0325


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class Chain:
    """URDF에서 base_footprint 기준 링크 변환을 계산한다."""

    def __init__(self, urdf_path: Path) -> None:
        root = ET.parse(urdf_path).getroot()
        self.joints = []
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
                rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
            axis = joint.find("axis")
            limit = joint.find("limit")
            self.joints.append({
                "name": joint.attrib["name"],
                "type": joint.attrib["type"],
                "parent": joint.find("parent").attrib["link"],
                "child": joint.find("child").attrib["link"],
                "xyz": np.array(xyz),
                "rot": rpy_matrix(*rpy),
                "axis": np.array([float(v) for v in axis.attrib["xyz"].split()]) if axis is not None else None,
                "lower": float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None,
                "upper": float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None,
            })
        self.limits = {j["name"]: (j["lower"], j["upper"]) for j in self.joints if j["lower"] is not None}

    def transforms(self, positions: dict[str, float]) -> dict[str, np.ndarray]:
        out = {"base_footprint": np.eye(4)}
        pending = list(self.joints)
        while pending:
            progressed = False
            rest = []
            for joint in pending:
                parent = out.get(joint["parent"])
                if parent is None:
                    rest.append(joint)
                    continue
                local = np.eye(4)
                local[:3, :3] = joint["rot"]
                local[:3, 3] = joint["xyz"]
                value = positions.get(joint["name"], 0.0)
                if joint["type"] == "revolute" and joint["axis"] is not None:
                    axis = joint["axis"] / np.linalg.norm(joint["axis"])
                    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                    r = np.eye(3) + math.sin(value) * k + (1 - math.cos(value)) * (k @ k)
                    motion = np.eye(4)
                    motion[:3, :3] = r
                    local = local @ motion
                elif joint["type"] == "prismatic" and joint["axis"] is not None:
                    motion = np.eye(4)
                    motion[:3, 3] = joint["axis"] * value
                    local = local @ motion
                out[joint["child"]] = parent @ local
                progressed = True
            if not progressed:
                raise AssertionError(f"연결되지 않은 관절: {[j['name'] for j in rest]}")
            pending = rest
        return out


def arm_names(side: str) -> list[str]:
    return [f"{side}_{name}" for name in ARM_JOINTS]


def base_positions(side: str, q: np.ndarray) -> dict[str, float]:
    values = dict(zip(arm_names(side), q))
    values[f"{side}_finger1_joint"] = FINGER_OPEN_M
    values[f"{side}_finger2_joint"] = FINGER_OPEN_M
    return values


def residual(chain: Chain, side: str, q: np.ndarray, frame: str, target: np.ndarray,
             axis_frame: str | None, axis_target: np.ndarray | None) -> np.ndarray:
    """위치 오차 3성분 + (선택) 공구 Z축 방향 오차 2성분."""
    transforms = chain.transforms(base_positions(side, q))
    error = list(target - transforms[frame][:3, 3])
    if axis_target is not None and axis_frame is not None:
        current_axis = transforms[axis_frame][:3, 2]
        difference = axis_target - current_axis
        basis = np.eye(3) - np.outer(axis_target, axis_target)
        eigenvalues, eigenvectors = np.linalg.eigh(basis)
        tangent = eigenvectors[:, eigenvalues > 0.5].T
        error.extend(tangent @ difference)
    return np.array(error)


def solve_ik(chain: Chain, side: str, frame: str, target: np.ndarray,
             seed: np.ndarray, iterations: int = 400,
             axis_frame: str | None = None,
             axis_target: np.ndarray | None = None) -> tuple[np.ndarray, float]:
    """감쇠 최소자승 IK. 5축이라 위치 3 + 축방향 2 = 5 구속이 정확히 맞는다."""
    names = arm_names(side)
    lower = np.array([chain.limits[n][0] for n in names])
    upper = np.array([chain.limits[n][1] for n in names])
    q = np.clip(seed.copy(), lower, upper)
    step = 1e-5
    for _ in range(iterations):
        error = residual(chain, side, q, frame, target, axis_frame, axis_target)
        if np.linalg.norm(error[:3]) < 2e-4 and (len(error) == 3 or np.linalg.norm(error[3:]) < 2e-3):
            break
        jac = np.zeros((len(error), len(names)))
        for index in range(len(names)):
            probe = q.copy()
            probe[index] += step
            jac[:, index] = (residual(chain, side, probe, frame, target, axis_frame, axis_target) - error) / step
        damping = 1e-4
        delta = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(len(error)), error)
        q = np.clip(q + 0.5 * delta, lower, upper)
    final = residual(chain, side, q, frame, target, axis_frame, axis_target)
    return q, float(np.linalg.norm(final[:3]))


def solve_with_restarts(chain: Chain, side: str, frame: str, target: np.ndarray,
                        restarts: int = 24, seed: int = 7,
                        axis_frame: str | None = None,
                        axis_target: np.ndarray | None = None) -> tuple[np.ndarray, float]:
    names = arm_names(side)
    lower = np.array([chain.limits[n][0] for n in names])
    upper = np.array([chain.limits[n][1] for n in names])
    rng = np.random.default_rng(seed)
    best_q, best_err = None, float("inf")
    for index in range(restarts):
        start = np.zeros(len(names)) if index == 0 else rng.uniform(lower * 0.7, upper * 0.7)
        q, err = solve_ik(chain, side, frame, target, start,
                          axis_frame=axis_frame, axis_target=axis_target)
        if err < best_err:
            best_q, best_err = q, err
    return best_q, best_err


def main() -> None:
    argparse.ArgumentParser(description=__doc__).parse_args()

    chain = Chain(URDF_PATH)
    report = {"urdf": str(URDF_PATH.relative_to(REPO_ROOT)), "candidates": {}}

    # 운반: 두 팔을 몸통 위로 모아 낮게 든다. 전방 도달을 줄여 COM 을 뒤로 당긴다.
    # 격자 탐색 결과 링크 원점 최소거리가 가장 큰 조합. 전방 도달을 억제해
    # COM 을 뒤에 두면서도 좌우 팔을 벌린다.
    transport_targets = {
        "left": np.array([0.06, 0.10, 0.26]),
        "right": np.array([0.06, -0.10, 0.26]),
    }
    # 붓기: 컵은 오른팔, 병은 왼팔. 병 주둥이가 컵 림 위 24.5 mm 에 오게 한다.
    cup_rim = np.array([0.150, 0.047, 0.470])
    bottle_cap = cup_rim + np.array([0.0, 0.0, 0.0245])
    # 병은 수직에서 112.3도 기울고, 그리퍼는 컵 반대쪽(+Y)에 남는다.
    tilt = math.radians(112.3)
    bottle_axis = np.array([0.0, -math.sin(tilt), math.cos(tilt)])
    # 컵은 세워서 든다.
    cup_axis = np.array([0.0, 0.0, 1.0])

    for label, targets, frames in (
        ("transport", transport_targets, {"left": "left_tool0", "right": "right_tool0"}),
        ("pour", {"left": bottle_cap, "right": cup_rim},
         {"left": "left_bottle_tcp", "right": "right_cup_tcp"}),
    ):
        entry = {}
        for side, target in targets.items():
            if label == "transport":
                # zero seed 를 유지해 좌우가 같은 분기(팔꿈치 위)로 풀리게 한다.
                q, err = solve_ik(chain, side, frames[side], target, np.zeros(len(ARM_JOINTS)))
            elif label == "pour":
                axis = bottle_axis if side == "left" else cup_axis
                q, err = solve_with_restarts(
                    chain, side, frames[side], target,
                    axis_frame=f"{side}_tool0", axis_target=axis,
                )
            else:
                q, err = solve_with_restarts(chain, side, frames[side], target)
            tool_axis = chain.transforms(base_positions(side, q))[f"{side}_tool0"][:3, 2]
            entry[side] = {
                "frame": frames[side],
                "target_m": [round(float(v), 4) for v in target],
                "tool0_axis": [round(float(v), 3) for v in tool_axis],
                "tilt_from_upright_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, tool_axis[2])))), 1),
                "joint_deg": [round(math.degrees(v), 1) for v in q],
                "position_error_mm": round(err * 1000.0, 2),
                "within_limits": True,
            }
        report["candidates"][label] = entry

    print(json.dumps(report, ensure_ascii=False, indent=2))
    worst = max(
        side["position_error_mm"]
        for state in report["candidates"].values()
        for side in state.values()
    )
    if worst > 5.0:
        print(f"경고: 최대 위치 오차 {worst} mm. 목표가 작업공간 밖일 수 있다.", file=sys.stderr)


if __name__ == "__main__":
    main()
