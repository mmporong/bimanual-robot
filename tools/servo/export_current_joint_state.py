#!/usr/bin/env python3
"""SO101 현재 엔코더를 LeRobot degrees로 환산해 URDF 한계와 대조한다.

Present_Position·Torque_Enable·Voltage·Temperature만 읽는다. 목표 위치나 EEPROM을
쓰지 않으며, 출력 JSON은 trajectory collision audit의 시작 자세 입력으로 쓴다.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from pathlib import Path
import sys


SERVO_DIR = Path(__file__).resolve().parent
REPO_ROOT = SERVO_DIR.parents[1]
IK_DIR = REPO_ROOT / "src/hold_flow_description/scripts"
for directory in (SERVO_DIR, IK_DIR):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from solve_task_poses import ARM_JOINTS, Chain, URDF_PATH  # noqa: E402
from sts_bus import A_POS, A_TEMP, A_TORQUE, A_VOLT, Bus, find_port  # noqa: E402


DEFAULT_CALIBRATION = REPO_ROOT / "calibration/bi_follower/arms_left.json"
MAX_RESOLUTION_VALUE = 4095


def raw_to_lerobot_degrees(raw: int, range_min: int, range_max: int) -> float:
    if range_min >= range_max:
        raise ValueError("calibration range_min은 range_max보다 작아야 합니다")
    midpoint = (range_min + range_max) / 2.0
    return (raw - midpoint) * 360.0 / MAX_RESOLUTION_VALUE


def read_state(port: str, calibration_path: Path) -> dict:
    calibration_path = calibration_path.resolve()
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    chain = Chain(URDF_PATH)
    bus = Bus(find_port(port))
    joints = {}
    try:
        for name in ARM_JOINTS:
            item = calibration[name]
            servo_id = int(item["id"])
            raw = bus.read(servo_id, A_POS, 2)
            torque = bus.read(servo_id, A_TORQUE)
            voltage_raw = bus.read(servo_id, A_VOLT)
            temperature = bus.read(servo_id, A_TEMP)
            if None in (raw, torque, voltage_raw, temperature):
                raise RuntimeError(f"{name}(ID {servo_id}) 상태 읽기 실패")
            degrees = raw_to_lerobot_degrees(raw, item["range_min"], item["range_max"])
            lower, upper = chain.limits[f"left_{name}"]
            lower_deg, upper_deg = math.degrees(lower), math.degrees(upper)
            joints[name] = {
                "servo_id": servo_id,
                "raw_position": raw,
                "lerobot_degrees": round(degrees, 4),
                "calibration_range": [item["range_min"], item["range_max"]],
                "within_calibration_range": item["range_min"] <= raw <= item["range_max"],
                "urdf_limit_deg": [round(lower_deg, 4), round(upper_deg, 4)],
                "within_urdf_limit": lower_deg <= degrees <= upper_deg,
                "urdf_limit_margin_deg": round(min(degrees - lower_deg, upper_deg - degrees), 4),
                "torque_enabled": bool(torque),
                "voltage_v": round(voltage_raw / 10.0, 2),
                "temperature_c": temperature,
            }
    finally:
        bus.close()
    blockers = []
    for name, item in joints.items():
        if not item["within_calibration_range"]:
            blockers.append(f"{name}: raw position이 calibration range 밖")
        if not item["within_urdf_limit"]:
            blockers.append(f"{name}: LeRobot degrees가 URDF hard limit 밖")
    try:
        calibration_label = str(calibration_path.relative_to(REPO_ROOT))
    except ValueError:
        calibration_label = str(calibration_path)
    return {
        "schema_version": "1.0",
        "mode": "read_only_current_joint_state",
        "motion_command_emitted": False,
        "side": "left",
        "port": port,
        "calibration": calibration_label,
        "degree_conversion": "(raw - (range_min + range_max)/2) * 360 / 4095",
        "joint_order": ARM_JOINTS,
        "joint_degrees": [joints[name]["lerobot_degrees"] for name in ARM_JOINTS],
        "joints": joints,
        "safe_for_trajectory_start": not blockers,
        "blocking_findings": blockers,
        "captured_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--strict", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = read_state(args.port, args.calibration)
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    if args.strict and not report["safe_for_trajectory_start"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
