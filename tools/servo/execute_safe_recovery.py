#!/usr/bin/env python3
"""검증된 SO-101 복귀 계획을 작은 동기 waypoint로 실행한다.

기본은 실물 상태를 읽고 계획과 대조하는 dry-run이다. ``--execute``를 명시해야만
목표 위치와 토크를 쓴다. 기본 실행은 부하·온도·스톨·계획 시작점 불일치를 감시하고
끝에 토크를 해제한다. ``--position-only``는 부하·온도를 읽지 않으며,
``--hold-torque-on-success``는 승인된 다음 연속 단계까지 성공 토크를 유지한다.
실패·예외·인터럽트에서는 토크 해제 경로를 실행하고 기존 속도/가속도를 복원한다.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

SERVO_DIR = Path(__file__).resolve().parent
if str(SERVO_DIR) not in sys.path:
    sys.path.insert(0, str(SERVO_DIR))

from sts_bus import A_ACCEL, A_GOAL, A_LOAD, A_POS, A_SPEED, A_TEMP, A_TORQUE, Bus, find_port


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CALIBRATION = REPO_ROOT / "calibration/bi_follower/arms_left.json"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
RESOLUTION_MAX = 4095


def degrees_to_raw(degrees: float, range_min: int, range_max: int) -> int:
    midpoint = (range_min + range_max) / 2.0
    return int(round(degrees * RESOLUTION_MAX / 360.0 + midpoint))


def interpolate_raw(start: list[int], target: list[int], max_step_ticks: int) -> list[list[int]]:
    if max_step_ticks <= 0:
        raise ValueError("max_step_ticks는 양수여야 합니다")
    steps = max(1, math.ceil(max(abs(b - a) for a, b in zip(start, target)) / max_step_ticks))
    return [
        [int(round(a + (b - a) * index / steps)) for a, b in zip(start, target)]
        for index in range(1, steps + 1)
    ]


def load_inputs(plan_path: Path, calibration_path: Path) -> tuple[dict, dict, list[int]]:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    if plan.get("motion_command_emitted") is not False:
        raise ValueError("motion_command_emitted=false인 미리보기 계획만 허용합니다")
    if not plan.get("ready_for_explicit_motion_approval", False):
        raise ValueError("복귀 계획이 실행 승인 전 게이트를 통과하지 못했습니다")
    target_deg = plan["recovery"]["target_joint_deg"]
    if len(target_deg) != len(JOINTS):
        raise ValueError("복귀 목표 관절 수가 5개가 아닙니다")
    target_raw = [
        degrees_to_raw(target_deg[index], calibration[name]["range_min"], calibration[name]["range_max"])
        for index, name in enumerate(JOINTS)
    ]
    for name, raw in zip(JOINTS, target_raw):
        item = calibration[name]
        if not item["range_min"] <= raw <= item["range_max"]:
            raise ValueError(f"{name}: 복귀 목표 {raw}가 calibration range 밖입니다")
    return plan, calibration, target_raw


def read_all(bus, calibration: dict, position_only: bool = False) -> dict[str, list[int]]:
    values = {"position": [], "torque": [], "temperature": [], "load": []}
    for name in JOINTS:
        sid = int(calibration[name]["id"])
        readings = (bus.read(sid, A_POS, 2), bus.read(sid, A_TORQUE))
        if any(value is None for value in readings):
            raise RuntimeError(f"{name}(ID {sid}) 상태 읽기 실패")
        position, torque = readings
        values["position"].append(int(position))
        values["torque"].append(int(torque))
        if not position_only:
            temperature = bus.read(sid, A_TEMP)
            load = bus.read(sid, A_LOAD, 2)
            if temperature is None or load is None:
                raise RuntimeError(f"{name}(ID {sid}) 상태 읽기 실패")
            values["temperature"].append(int(temperature))
            values["load"].append(int(load) & 0x3FF)
    return values


def validate_start(plan: dict, calibration: dict, state: dict, target_raw: list[int],
                   start_tolerance_ticks: int, max_delta_ticks: int,
                   max_waypoint_step_ticks: int = 220,
                   allow_torque_enabled: bool = False) -> list[list[int]]:
    torque = state["torque"]
    if any(torque) and not (allow_torque_enabled and all(torque)):
        raise RuntimeError("시작 토크는 전부 꺼져 있거나 승인된 연속 단계에서 전부 켜져 있어야 합니다")
    expected = [
        degrees_to_raw(plan["start_joint_deg"][index], calibration[name]["range_min"],
                       calibration[name]["range_max"])
        for index, name in enumerate(JOINTS)
    ]
    drift = [actual - reference for actual, reference in zip(state["position"], expected)]
    if max(abs(value) for value in drift) > start_tolerance_ticks:
        raise RuntimeError(f"계획 생성 뒤 자세가 바뀌었습니다: raw drift={drift}")
    delta = [target - actual for target, actual in zip(target_raw, state["position"])]
    if max(abs(value) for value in delta) > max_delta_ticks:
        raise RuntimeError(f"복귀 이동량이 상한 {max_delta_ticks} tick을 넘습니다: {delta}")
    return interpolate_raw(state["position"], target_raw, max_step_ticks=max_waypoint_step_ticks)


def release_and_restore(bus, ids: list[int], previous: list[tuple[int, int]]) -> None:
    for sid in ids:
        position = bus.read(sid, A_POS, 2)
        if position is not None:
            bus.write(sid, A_GOAL, position, 2)
    for sid in ids:
        bus.write(sid, A_TORQUE, 0)
    for sid, (speed, accel) in zip(ids, previous):
        bus.write(sid, A_SPEED, speed, 2)
        bus.write(sid, A_ACCEL, accel)


def require_write(bus, sid: int, address: int, value: int, size: int = 1) -> None:
    if not bus.write(sid, address, value, size):
        raise RuntimeError(f"ID {sid}: register {address} 쓰기 실패")


def read_required(bus, sid: int, address: int, size: int = 1, retries: int = 3) -> int:
    for _ in range(retries):
        value = bus.read(sid, address, size)
        if value is not None:
            return int(value)
    raise RuntimeError(f"ID {sid}: register {address} 읽기 실패")


def confirmed_temperatures(bus, ids: list[int], limit: int) -> list[int]:
    first = [read_required(bus, sid, A_TEMP) for sid in ids]
    if not any(value > limit for value in first):
        return first
    return [read_required(bus, sid, A_TEMP) for sid in ids]


def execute(bus, calibration: dict, waypoints: list[list[int]], speed: int, acceleration: int,
            load_limit: int, temperature_limit: int, arrival_tolerance_ticks: int = 15,
            hold_torque_on_success: bool = False, position_only: bool = False) -> dict:
    ids = [int(calibration[name]["id"]) for name in JOINTS]
    previous = [(read_required(bus, sid, A_SPEED, 2), read_required(bus, sid, A_ACCEL)) for sid in ids]
    initial = [read_required(bus, sid, A_POS, 2) for sid in ids]
    peak_load = None if position_only else [0] * len(ids)
    loaded_final: list[int] | None = None
    reached = False
    try:
        for sid, position in zip(ids, initial):
            require_write(bus, sid, A_GOAL, position, 2)
            require_write(bus, sid, A_ACCEL, acceleration)
            require_write(bus, sid, A_SPEED, speed, 2)
        for sid in ids:
            require_write(bus, sid, A_TORQUE, 1)
        time.sleep(0.05)
        enabled = [read_required(bus, sid, A_TORQUE) for sid in ids]
        if enabled != [1] * len(ids):
            raise RuntimeError(f"토크 활성화 확인 실패: {enabled}")

        for waypoint_index, waypoint in enumerate(waypoints, start=1):
            before = [read_required(bus, sid, A_POS, 2) for sid in ids]
            for sid, goal in zip(ids, waypoint):
                require_write(bus, sid, A_GOAL, goal, 2)
            written_goals = [read_required(bus, sid, A_GOAL, 2) for sid in ids]
            if written_goals != waypoint:
                raise RuntimeError(
                    f"waypoint {waypoint_index}: 목표 레지스터 대조 실패, "
                    f"expected={waypoint}, actual={written_goals}"
                )
            initial_error = [abs(goal - value) for goal, value in zip(waypoint, before)]
            budget = max(8.0, max(initial_error) / max(speed, 1) * 1.5 + 2.0)
            deadline = time.monotonic() + budget
            last_motion = [time.monotonic()] * len(ids)
            last = before
            while True:
                time.sleep(0.03)
                now = [read_required(bus, sid, A_POS, 2) for sid in ids]
                loads = None if position_only else [
                    read_required(bus, sid, A_LOAD, 2) & 0x3FF for sid in ids
                ]
                temperatures = None if position_only else confirmed_temperatures(
                    bus, ids, temperature_limit
                )
                if loads is not None:
                    peak_load = [max(old, new) for old, new in zip(peak_load, loads)]
                errors = [abs(goal - value) for goal, value in zip(waypoint, now)]
                if any(error > baseline + 10 for error, baseline in zip(errors, initial_error)):
                    raise RuntimeError(
                        f"waypoint {waypoint_index}: 목표에서 멀어짐, "
                        f"initial_error={initial_error}, error={errors}"
                    )
                if loads is not None and max(loads) > load_limit:
                    raise RuntimeError(f"waypoint {waypoint_index}: 부하 상한 초과 {loads}")
                if temperatures is not None and max(temperatures) > temperature_limit:
                    raise RuntimeError(f"waypoint {waypoint_index}: 온도 상한 초과 {temperatures}")
                for index, (value, old) in enumerate(zip(now, last)):
                    if abs(value - old) > 2:
                        last_motion[index] = time.monotonic()
                        last[index] = value
                if max(errors) <= arrival_tolerance_ticks:
                    break
                stalled = [
                    JOINTS[index]
                    for index, error in enumerate(errors)
                    if error > arrival_tolerance_ticks and time.monotonic() - last_motion[index] > 0.5
                ]
                if stalled:
                    raise RuntimeError(
                        f"waypoint {waypoint_index}: 0.5초 스톨 {stalled}, "
                        f"position={now}, error={errors}, load={loads}"
                    )
                if time.monotonic() > deadline:
                    raise RuntimeError(f"waypoint {waypoint_index}: 도달 시간 초과, position={now}")
        loaded_final = [read_required(bus, sid, A_POS, 2) for sid in ids]
        loaded_error = [abs(goal - value) for goal, value in zip(waypoints[-1], loaded_final)]
        reached = max(loaded_error) <= arrival_tolerance_ticks
    except BaseException:
        release_and_restore(bus, ids, previous)
        raise
    if reached and hold_torque_on_success:
        try:
            for sid, (previous_speed, previous_accel) in zip(ids, previous):
                require_write(bus, sid, A_SPEED, previous_speed, 2)
                require_write(bus, sid, A_ACCEL, previous_accel)
            held_torque = [read_required(bus, sid, A_TORQUE) for sid in ids]
            if held_torque != [1] * len(ids):
                raise RuntimeError(f"토크 유지 확인 실패: {held_torque}")
        except BaseException:
            release_and_restore(bus, ids, previous)
            raise
        return {
            "completed": True,
            "target_reached_with_torque": True,
            "persistent_after_torque_release": None,
            "torque_held": True,
            "loaded_final_raw": loaded_final,
            "released_final_raw": None,
            "released_target_error_ticks": None,
            "peak_load": peak_load,
        }
    release_and_restore(bus, ids, previous)
    time.sleep(0.5)
    released_final = [read_required(bus, sid, A_POS, 2) for sid in ids]
    assert loaded_final is not None
    loaded_error = [abs(goal - value) for goal, value in zip(waypoints[-1], loaded_final)]
    released_error = [abs(goal - value) for goal, value in zip(waypoints[-1], released_final)]
    persistent = max(released_error) <= arrival_tolerance_ticks
    return {
        "completed": persistent,
        "target_reached_with_torque": max(loaded_error) <= arrival_tolerance_ticks,
        "persistent_after_torque_release": persistent,
        "torque_held": False,
        "loaded_final_raw": loaded_final,
        "released_final_raw": released_final,
        "released_target_error_ticks": released_error,
        "peak_load": peak_load,
    }


class MultiJointFakeBus:
    def __init__(self, calibration: dict, positions: list[int]):
        self.positions = {int(calibration[name]["id"]): value for name, value in zip(JOINTS, positions)}
        self.goals = self.positions.copy()
        self.reg = {
            sid: {A_TORQUE: 0, A_SPEED: 300, A_ACCEL: 10, A_TEMP: 34, A_LOAD: 20}
            for sid in self.positions
        }

    def read(self, sid: int, address: int, size: int = 1):
        if address == A_POS:
            if self.reg[sid][A_TORQUE]:
                delta = self.goals[sid] - self.positions[sid]
                self.positions[sid] += max(-8, min(8, delta))
            return self.positions[sid]
        if address == A_GOAL:
            return self.goals[sid]
        return self.reg[sid].get(address, 0)

    def write(self, sid: int, address: int, value: int, size: int = 1):
        if address == A_GOAL:
            self.goals[sid] = value
        else:
            self.reg[sid][address] = value
        return True

    def close(self):
        return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--port")
    parser.add_argument("--speed", type=int, default=80)
    parser.add_argument("--acceleration", type=int, default=5)
    parser.add_argument("--load-limit", type=int, default=450)
    parser.add_argument("--temperature-limit", type=int, default=55)
    parser.add_argument("--arrival-tolerance-ticks", type=int, default=15,
                        help="P게인 16의 목표 앞 정지를 허용하는 도달 오차(15 tick≈1.32도)")
    parser.add_argument("--start-tolerance-ticks", type=int, default=12)
    parser.add_argument("--max-delta-ticks", type=int, default=220)
    parser.add_argument("--max-waypoint-step-ticks", type=int, default=220,
                        help="최종 목표 간격 상한. 실제 연속 경로 감사 간격은 별도 2도")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--hold-torque-on-success",
        action="store_true",
        help="도착 성공 시 팔 토크를 유지한다. 실패 시에는 항상 토크를 해제한다",
    )
    parser.add_argument(
        "--allow-torque-enabled",
        action="store_true",
        help="직전 연속 단계가 유지한 팔 토크 5개를 허용한다. 일부만 켜진 상태는 거부한다",
    )
    parser.add_argument("--position-only", action="store_true",
                        help="부하·온도를 읽지 않고 위치·스톨만 감시한다")
    parser.add_argument("--mock", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    plan, calibration, target_raw = load_inputs(args.plan, args.calibration)
    expected_start = [
        degrees_to_raw(plan["start_joint_deg"][index], calibration[name]["range_min"],
                       calibration[name]["range_max"])
        for index, name in enumerate(JOINTS)
    ]
    bus = MultiJointFakeBus(calibration, expected_start) if args.mock else Bus(find_port(args.port))
    try:
        state = read_all(bus, calibration, args.position_only)
        waypoints = validate_start(
            plan, calibration, state, target_raw, args.start_tolerance_ticks, args.max_delta_ticks,
            args.max_waypoint_step_ticks, args.allow_torque_enabled,
        )
        preview = {
            "mode": "safe_recovery_execute" if args.execute else "safe_recovery_dry_run",
            "motion_command_emitted": False,
            "current_raw": state["position"],
            "target_raw": target_raw,
            "raw_delta": [b - a for a, b in zip(state["position"], target_raw)],
            "waypoint_count": len(waypoints),
            "max_waypoint_step_ticks": max(
                max(abs(b - a) for a, b in zip(previous, current))
                for previous, current in zip([state["position"]] + waypoints[:-1], waypoints)
            ),
            "arrival_tolerance_ticks": args.arrival_tolerance_ticks,
        }
        if not args.execute:
            print(json.dumps(preview, ensure_ascii=False, indent=2))
            return 0
        previous_sigint = signal.getsignal(signal.SIGINT)
        previous_sigterm = signal.getsignal(signal.SIGTERM)

        def interrupt_motion(signum, _frame):
            raise KeyboardInterrupt(f"signal {signum}")

        signal.signal(signal.SIGINT, interrupt_motion)
        signal.signal(signal.SIGTERM, interrupt_motion)
        try:
            result = execute(
                bus, calibration, waypoints, args.speed, args.acceleration,
                args.load_limit, args.temperature_limit, args.arrival_tolerance_ticks,
                args.hold_torque_on_success, args.position_only,
            )
        except KeyboardInterrupt as exc:
            print(f"실행 중단: {exc}. 토크 해제 경로를 실행했습니다.", file=sys.stderr)
            return 130
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)
        preview.update(result)
        preview["motion_command_emitted"] = True
        print(json.dumps(preview, ensure_ascii=False, indent=2))
        return 0 if result["completed"] else 3
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
