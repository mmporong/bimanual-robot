#!/usr/bin/env python3
"""검증된 SO-101 복귀 계획을 작은 동기 waypoint로 실행한다.

기본은 실물 상태를 읽고 계획과 대조하는 dry-run이다. ``--execute``를 명시해야만
목표 위치와 토크를 쓴다. 기본 실행은 부하·온도·스톨·계획 시작점 불일치를 감시하고
끝에 자세를 유지한다. ``--position-only``는 부하·온도를 읽지 않는다.
실패·예외·인터럽트에서는 현재 위치로 목표를 동결하고 토크 상태를 바꾸지 않는다.
토크 해제는 팔이 지지된 상태에서 명시한 성공 후 해제 옵션으로만 수행한다.
"""

from __future__ import annotations

import argparse
import hashlib
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


def calibration_binding(plan: dict, calibration_bytes: bytes) -> dict:
    """파일 스냅샷과 같은 변환으로 실행 준비물의 관절 명령을 기록한다."""
    calibration = json.loads(calibration_bytes)
    ids = []
    for name in JOINTS:
        item = calibration[name]
        for key in ("id", "range_min", "range_max"):
            if type(item[key]) is not int:
                raise ValueError(f"{name}: {key}는 정수여야 합니다")
        if not 1 <= item["id"] <= 253:
            raise ValueError(f"{name}: servo ID 범위 오류")
        if not 0 <= item["range_min"] < item["range_max"] <= RESOLUTION_MAX:
            raise ValueError(f"{name}: calibration range 오류")
        ids.append(item["id"])
    if len(set(ids)) != len(ids):
        raise ValueError("servo ID가 중복됩니다")
    vectors = {
        "start_raw": plan["start_joint_deg"],
        "target_raw": plan["recovery"]["target_joint_deg"],
    }
    result = {
        "sha256": hashlib.sha256(calibration_bytes).hexdigest(),
        "joint_order": JOINTS.copy(),
        "servo_ids": ids,
    }
    for label, joint_deg in vectors.items():
        if not isinstance(joint_deg, list) or len(joint_deg) != len(JOINTS):
            raise ValueError(f"{label}: 관절 수는 5개여야 합니다")
        raw_ticks = []
        for name, value_deg in zip(JOINTS, joint_deg):
            if type(value_deg) not in (int, float) or not math.isfinite(value_deg):
                raise ValueError(f"{name}: 관절각은 유한한 숫자여야 합니다")
            item = calibration[name]
            raw = degrees_to_raw(value_deg, item["range_min"], item["range_max"])
            if not item["range_min"] <= raw <= item["range_max"]:
                raise ValueError(f"{name}: {label}가 calibration range 밖입니다")
            raw_ticks.append(raw)
        result[label] = raw_ticks
    return result


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
    calibration_bytes = calibration_path.read_bytes()
    calibration = json.loads(calibration_bytes)
    if plan.get("motion_command_emitted") is not False:
        raise ValueError("motion_command_emitted=false인 미리보기 계획만 허용합니다")
    if not plan.get("ready_for_explicit_motion_approval", False):
        raise ValueError("복귀 계획이 실행 승인 전 게이트를 통과하지 못했습니다")
    if "calibration_binding" in plan:
        expected = calibration_binding(plan, calibration_bytes)
        if plan["calibration_binding"] != expected:
            raise ValueError("계획 생성 뒤 calibration 또는 raw 목표가 변경됐습니다")
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
    prepared = hold_current(bus, ids)
    if prepared["errors"]:
        raise RuntimeError(f"토크 해제 전 목표 동결 실패: {prepared}")
    errors = []
    interrupted = None
    for sid in ids:
        try:
            require_write(bus, sid, A_TORQUE, 0)
        except BaseException as exc:
            errors.append(f"ID {sid}: {exc}")
            if not isinstance(exc, Exception) and interrupted is None:
                interrupted = exc
    torque = {}
    for sid in ids:
        try:
            torque[sid] = read_required(bus, sid, A_TORQUE)
        except BaseException as exc:
            torque[sid] = None
            errors.append(f"ID {sid}: {exc}")
            if not isinstance(exc, Exception) and interrupted is None:
                interrupted = exc
    if interrupted is not None:
        interrupted.add_note(f"토크 해제 중단: torque={torque}, errors={errors}")
        raise interrupted
    if errors or any(value != 0 for value in torque.values()):
        raise RuntimeError(f"토크 해제 미확인: torque={torque}, errors={errors}")
    for sid, (speed, accel) in zip(ids, previous):
        require_write(bus, sid, A_SPEED, speed, 2)
        require_write(bus, sid, A_ACCEL, accel)


def require_write(bus, sid: int, address: int, value: int, size: int = 1) -> None:
    if not bus.write(sid, address, value, size):
        raise RuntimeError(f"ID {sid}: register {address} 쓰기 실패")


def hold_current(bus, ids: list[int]) -> dict:
    """진행 목표를 취소한다. 토크를 끄거나 꺼진 축을 다시 켜지 않는다.

    통신 실패는 유지 성공이 아니다. 실패한 축이 있어도 나머지 축을 처리한다.
    """
    report = {"action": "hold_current", "torque_write_emitted": False,
              "servos": [], "errors": []}
    for sid in ids:
        item = {"id": sid, "goal": None, "torque": None, "goal_verified": False}
        report["servos"].append(item)
        try:
            position = read_required(bus, sid, A_POS, 2)
            item["goal"] = position
            require_write(bus, sid, A_GOAL, position, 2)
            item["goal_verified"] = read_required(bus, sid, A_GOAL, 2) == position
            item["torque"] = read_required(bus, sid, A_TORQUE)
            if not item["goal_verified"]:
                raise RuntimeError(f"ID {sid}: 정지 목표 판독 불일치")
        except BaseException as exc:
            report["errors"].append(f"ID {sid}: {type(exc).__name__}: {exc}")
    report["hold_command_verified"] = bool(ids) and not report["errors"] and all(
        item["goal_verified"] and item["torque"] == 1 for item in report["servos"]
    )
    return report


def stop_after_error(bus, ids: list[int], error: BaseException) -> None:
    report = hold_current(bus, ids)
    error.motion_stop_report = report
    error.add_note(json.dumps(report, ensure_ascii=False))


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
            hold_torque_on_success: bool = True, position_only: bool = False,
            monitor=lambda: None, waypoint_period_s: float | None = None,
            tracking_limit_ticks: int = 45) -> dict:
    ids = [int(calibration[name]["id"]) for name in JOINTS]
    peak_load = None if position_only else [0] * len(ids)
    loaded_final: list[int] | None = None
    reached = False
    try:
        monitor()
        if waypoint_period_s is not None and not (math.isfinite(waypoint_period_s)
                                                   and 0 < waypoint_period_s <= 1):
            raise ValueError("waypoint_period_s는 0 초과 1 이하이어야 합니다")
        if waypoint_period_s is not None and not arrival_tolerance_ticks <= tracking_limit_ticks <= 45:
            raise ValueError("도달 허용치 <= 추종 상한 <= 45 tick이어야 합니다")
        previous = [(read_required(bus, sid, A_SPEED, 2), read_required(bus, sid, A_ACCEL)) for sid in ids]
        initial = [read_required(bus, sid, A_POS, 2) for sid in ids]
        if waypoint_period_s is not None:
            for first, second in zip([initial] + waypoints[:-1], waypoints):
                if len(second) != len(ids) or max(abs(b-a) for a, b in zip(first, second)) > 5:
                    raise ValueError("연속 전달 경로의 관절별 간격은 5 tick 이하이어야 합니다")
        for name, sid, position in zip(JOINTS, ids, initial):
            item = calibration[name]
            seed_ticks = min(item["range_max"], max(item["range_min"], position))
            # 경계의 정수 판독 편차만 허용하며 명령 범위 자체는 넓히지 않는다.
            if abs(seed_ticks - position) > 2:
                raise RuntimeError(f"{name}: 현재 위치가 저장 범위에서 벗어남: {position}")
            require_write(bus, sid, A_GOAL, seed_ticks, 2)
            require_write(bus, sid, A_ACCEL, acceleration)
            require_write(bus, sid, A_SPEED, speed, 2)
        for sid in ids:
            require_write(bus, sid, A_TORQUE, 1)
        time.sleep(0.05)
        enabled = [read_required(bus, sid, A_TORQUE) for sid in ids]
        if enabled != [1] * len(ids):
            raise RuntimeError(f"토크 활성화 확인 실패: {enabled}")

        for waypoint_index, waypoint in enumerate(waypoints, start=1):
            monitor()
            before = [read_required(bus, sid, A_POS, 2) for sid in ids]
            if waypoint_period_s is not None and max(
                    abs(goal - value) for goal, value in zip(waypoint, before)) > tracking_limit_ticks:
                raise RuntimeError(f"waypoint {waypoint_index}: 명령 전 추종 상한 초과")
            for sid, goal in zip(ids, waypoint):
                require_write(bus, sid, A_GOAL, goal, 2)
            written_goals = [read_required(bus, sid, A_GOAL, 2) for sid in ids]
            if written_goals != waypoint:
                raise RuntimeError(
                    f"waypoint {waypoint_index}: 목표 레지스터 대조 실패, "
                    f"expected={waypoint}, actual={written_goals}"
                )
            initial_error = [abs(goal - value) for goal, value in zip(waypoint, before)]
            next_waypoint_s = time.monotonic() + (waypoint_period_s or 0)
            budget = max(8.0, max(initial_error) / max(speed, 1) * 1.5 + 2.0)
            deadline = time.monotonic() + budget
            last_motion = [time.monotonic()] * len(ids)
            last = before
            while True:
                time.sleep(0.03)
                monitor()
                if [read_required(bus, sid, A_TORQUE) for sid in ids] != [1] * len(ids):
                    raise RuntimeError("복귀 중 토크 해제 감지")
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
                if waypoint_period_s is not None and max(errors) > tracking_limit_ticks:
                    raise RuntimeError(f"waypoint {waypoint_index}: 추종 상한 초과, error={errors}")
                if waypoint_period_s is None and any(
                        error > baseline + 10 for error, baseline in zip(errors, initial_error)):
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
                streaming_midpoint = waypoint_period_s is not None and waypoint_index < len(waypoints)
                if streaming_midpoint and time.monotonic() >= next_waypoint_s:
                    break
                if not streaming_midpoint and max(errors) <= arrival_tolerance_ticks:
                    break
                stalled = [
                    JOINTS[index]
                    for index, error in enumerate(errors)
                    if error > arrival_tolerance_ticks and time.monotonic() - last_motion[index] > 0.5
                ]
                if stalled and not streaming_midpoint:
                    raise RuntimeError(
                        f"waypoint {waypoint_index}: 0.5초 스톨 {stalled}, "
                        f"position={now}, error={errors}, load={loads}"
                    )
                if time.monotonic() > deadline:
                    raise RuntimeError(f"waypoint {waypoint_index}: 도달 시간 초과, position={now}")
        loaded_final = [read_required(bus, sid, A_POS, 2) for sid in ids]
        loaded_error = [abs(goal - value) for goal, value in zip(waypoints[-1], loaded_final)]
        reached = max(loaded_error) <= arrival_tolerance_ticks
    except BaseException as exc:
        stop_after_error(bus, ids, exc)
        raise
    if hold_torque_on_success:
        try:
            for sid, (previous_speed, previous_accel) in zip(ids, previous):
                require_write(bus, sid, A_SPEED, previous_speed, 2)
                require_write(bus, sid, A_ACCEL, previous_accel)
            held_torque = [read_required(bus, sid, A_TORQUE) for sid in ids]
            if held_torque != [1] * len(ids):
                raise RuntimeError(f"토크 유지 확인 실패: {held_torque}")
        except BaseException as exc:
            stop_after_error(bus, ids, exc)
            raise
        return {
            "completed": reached,
            "target_reached_with_torque": reached,
            "persistent_after_torque_release": None,
            "torque_held": True,
            "loaded_final_raw": loaded_final,
            "released_final_raw": None,
            "released_target_error_ticks": None,
            "peak_load": peak_load,
        }
    if not reached:
        error = RuntimeError("최종 목표 미도달: 현재 위치에서 정지")
        stop_after_error(bus, ids, error)
        raise error
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


def execute_encoder_feedback(bus, calibration, target, *, monitor=lambda: None,
                             speed=60, acceleration=5, tolerance_ticks=10,
                             compensation_limit_ticks=40, tracking_limit_ticks=45,
                             sleep=time.sleep, clock=time.monotonic):
    """이미 켜진 팔의 위치 오차를 제한된 외부 피드백으로 보정한다.

    P/EEPROM/보호 설정과 토크 상태를 변경하지 않는다. 보낸 goal과 관측 위치,
    명목 IK 목표를 구분한다. 실패 시 진행 목표를 취소하고 토크를 유지한다.
    기본 보정/추종 상한은 40/45 tick이다. 명시한 책상 시험은 최대 50/60 tick까지
    허용하되 실제 위치 구간, 전달 간격과 실행 시간 상한은 바꾸지 않는다.
    """
    ids = [calibration[name]["id"] for name in JOINTS]
    if len(target) != 5 or any(type(v) is not int for v in target):
        raise ValueError("관절 목표는 정수 5개여야 합니다")
    if not 1 <= tolerance_ticks <= 15 or not 0 <= compensation_limit_ticks <= 50 or not 15 <= tracking_limit_ticks <= 60:
        raise ValueError("피드백 보정 범위 오류")
    samples, prior = [], []
    current = goals = bias = None
    try:
        monitor()
        if [read_required(bus, sid, A_TORQUE) for sid in ids] != [1] * 5:
            raise RuntimeError("연속 피드백은 전축 토크 ON 상태에서만 실행합니다")
        current = [read_required(bus, sid, A_POS, 2) for sid in ids]
        for name, value in zip(JOINTS, target):
            if not calibration[name]["range_min"] <= value <= calibration[name]["range_max"]:
                raise ValueError(f"{name}: 목표가 저장 범위 밖입니다")
        initial = current.copy()
        goals = [read_required(bus, sid, A_GOAL, 2) for sid in ids]
        if max(abs(a-b) for a, b in zip(goals, current)) > tracking_limit_ticks:
            raise RuntimeError("초기 유지 명령이 추종 범위 밖입니다")
        # 직전 피드백의 중력 보상 명령을 현재 위치 값으로 덮어쓰지 않는다.
        bias = [goal-value if abs(now-value) <= 15 and abs(goal-value) <= compensation_limit_ticks else 0
                for goal, now, value in zip(goals, current, target)]
        prior = [(read_required(bus, sid, A_SPEED, 2), read_required(bus, sid, A_ACCEL)) for sid in ids]
        for sid in ids:
            require_write(bus, sid, A_SPEED, speed, 2)
            require_write(bus, sid, A_ACCEL, acceleration)
        deadline = clock() + 10
        last_position, last_motion = current.copy(), [clock()] * 5
        consecutive = 0
        while True:
            monitor()
            if clock() > deadline:
                raise RuntimeError("제한된 피드백 보정으로 목표에 도달하지 못했습니다")
            if [read_required(bus, sid, A_TORQUE) for sid in ids] != [1] * 5:
                raise RuntimeError("피드백 중 토크 해제 감지")
            current = [read_required(bus, sid, A_POS, 2) for sid in ids]
            errors = [v-t for v, t in zip(current, target)]
            for i, value in enumerate(current):
                if abs(value-last_position[i]) > 2:
                    last_motion[i], last_position[i] = clock(), value
                if not min(initial[i], target[i])-10 <= value <= max(initial[i], target[i])+10:
                    raise RuntimeError("피드백 중 관측 위치가 목표 구간을 벗어났습니다")
            if max(abs(e) for e in errors) <= tolerance_ticks:
                consecutive += 1
                if consecutive == 3:
                    break
                sleep(.1)
                continue
            consecutive = 0
            for i, error in enumerate(errors):
                # 명목 목표에 명령이 도착한 뒤에만 작은 보정량을 누적한다.
                if abs(goals[i]-(target[i]+bias[i])) <= 2 and abs(error) > tolerance_ticks:
                    bias[i] = max(-compensation_limit_ticks, min(compensation_limit_ticks,
                                    bias[i] + (-2 if error > 0 else 2)))
                desired = target[i] + bias[i]
                candidate = goals[i] + max(-5, min(5, desired-goals[i]))
                candidate = max(current[i]-tracking_limit_ticks, min(current[i]+tracking_limit_ticks, candidate))
                if abs(candidate-goals[i]) > 5:
                    raise RuntimeError("현재 판독과 이전 명령의 차이로 5 tick 전달 제한을 유지할 수 없습니다")
                item = calibration[JOINTS[i]]
                if not item["range_min"] <= candidate <= item["range_max"]:
                    raise RuntimeError("보정 명령이 저장 범위를 벗어났습니다")
                if (abs(error) > tolerance_ticks and abs(candidate-current[i]) >= tracking_limit_ticks
                        and clock()-last_motion[i] > .8):
                    raise RuntimeError(f"{JOINTS[i]}: 피드백 추종 한계에서 위치 변화 없음")
                require_write(bus, ids[i], A_GOAL, candidate, 2)
                goals[i] = candidate
            if [read_required(bus, sid, A_GOAL, 2) for sid in ids] != goals:
                raise RuntimeError("피드백 목표 readback 불일치")
            samples.append({"actual_raw": current.copy(), "command_raw": goals.copy(),
                            "compensation_ticks": bias.copy()})
            if clock() > deadline:
                raise RuntimeError("제한된 피드백 보정으로 목표에 도달하지 못했습니다")
            sleep(.1)
        for sid, (old_speed, old_acceleration) in zip(ids, prior):
            require_write(bus, sid, A_SPEED, old_speed, 2)
            require_write(bus, sid, A_ACCEL, old_acceleration)
        monitor()
        if clock() > deadline:
            raise RuntimeError("종료 확인 중 피드백 실행 시간 상한을 넘었습니다")
        if [read_required(bus, sid, A_TORQUE) for sid in ids] != [1] * 5:
            raise RuntimeError("피드백 종료 토크 유지 미확인")
        if [read_required(bus, sid, A_GOAL, 2) for sid in ids] != goals:
            raise RuntimeError("피드백 종료 목표 유지 미확인")
        current = [read_required(bus, sid, A_POS, 2) for sid in ids]
        if max(abs(a-b) for a,b in zip(current, target)) > tolerance_ticks:
            raise RuntimeError("프로파일 복원 뒤 최종 위치가 목표 허용오차 밖입니다")
        return {"completed": True, "target_raw": target, "actual_raw": current,
                "command_raw": goals, "compensation_ticks": bias, "samples": samples,
                "torque_held": True, "temperature_load_read": False}
    except BaseException as exc:
        exc.encoder_feedback_report = {"target_raw": target, "actual_raw": current,
                                       "command_raw": goals, "compensation_ticks": bias,
                                       "samples": samples, "temperature_load_read": False}
        exc.add_note(json.dumps({"encoder_feedback": exc.encoder_feedback_report}, ensure_ascii=False))
        stop_after_error(bus, ids, exc)
        for sid, (old_speed, old_acceleration) in zip(ids, prior):
            for address, value, size in ((A_SPEED, old_speed, 2), (A_ACCEL, old_acceleration, 1)):
                try:
                    require_write(bus, sid, address, value, size)
                except BaseException as restore_error:
                    exc.add_note(f"ID {sid} 프로파일 복원 실패: {restore_error}")
        raise


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
        dest="hold_torque_on_success",
        action="store_true",
        default=True,
        help="도착 후 팔 토크를 유지한다(기본값). 오류 시 현재 위치에서 정지한다",
    )
    parser.add_argument("--release-torque-on-success", dest="hold_torque_on_success",
                        action="store_false", help="팔을 받친 상태에서만 사용: 성공 후 토크 해제")
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
            print(f"실행 중단: {exc}. 현재 위치 정지를 시도했습니다. "
                  f"결과={getattr(exc, 'motion_stop_report', None)}", file=sys.stderr)
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
