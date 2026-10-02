#!/usr/bin/env python3
"""현재 raw 기준 손목 왕복 또는 보정 대조 후 그리퍼 부분 개폐 시험."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import sys
import threading
import time
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dashboard_stop import open_stop_bus
from servo.sts_bus import (A_ACCEL, A_GOAL, A_MAX_ANGLE, A_MIN_ANGLE, A_OFFSET,
                          A_PHASE, A_POS, A_SPEED, A_TORQUE, decode_offset)
from servo.execute_safe_recovery import read_required, require_write
from servo.servo_record_ranges import validate_capture

# STS3215 Operating_Mode: LeRobot feetech/tables.py, position mode = 0.
OPERATING_MODE = 33
SID = 5


def dashboard_monitor(url):
    with urlopen(url, timeout=0.3) as response:
        state = json.load(response)
    if not state.get("enabled") or state.get("requested") or state.get("running"):
        raise RuntimeError("대시보드 정지 요청 또는 정지 연결 미확인")


def run(bus, delta=32, execute=False, monitor=lambda: None,
        sleep=time.sleep, clock=time.monotonic, *, calibration=None, expected_phase=None):
    gripper = calibration is not None
    if type(delta) is not int or delta != (128 if gripper else 32):
        raise ValueError("손목은 +32 tick, 그리퍼는 열림 방향 128 tick만 허용합니다")
    if gripper:
        validate_capture(calibration)
        if type(expected_phase) is not int or expected_phase not in (12, 76):
            raise ValueError("그리퍼 Phase 기대값은 12 또는 76이어야 합니다")
    sid = 6 if gripper else SID
    tolerance_ticks = 16 if gripper else 4
    result = {"schema": "partial_gripper_probe_v1" if gripper else "single_wrist_raw_jog_v1", "servo_id": sid,
              "motion_command_emitted": False, "roundtrip_reached": False,
              "calibration_used": gripper, "samples": [], "stop_errors": []}
    if gripper:
        result.update(partial_cycle_reached=False, tolerance_ticks=tolerance_ticks,
                      physical_jaw_motion_verified=False)
    previous = None
    mutated = False
    try:
        monitor()
        result["initial_raw"] = [read_required(bus, sid, A_POS, 2) for sid in range(1, 7)]
        torque = [read_required(bus, sid, A_TORQUE) for sid in range(1, 7)]
        if torque != [0] * 6:
            raise RuntimeError("다른 제어기가 켠 토크가 있습니다. 이 시험은 토크 OFF 상태에서 시작합니다")
        if read_required(bus, sid, OPERATING_MODE) != 0:
            raise RuntimeError("시험 축이 위치 제어 모드가 아닙니다")
        start = result["initial_raw"][sid - 1]
        lo, hi = read_required(bus, sid, A_MIN_ANGLE, 2), read_required(bus, sid, A_MAX_ANGLE, 2)
        if gripper:
            for name, c in calibration.items():
                motor = c["id"]
                observed = (decode_offset(read_required(bus, motor, A_OFFSET, 2)),
                            read_required(bus, motor, A_MIN_ANGLE, 2),
                            read_required(bus, motor, A_MAX_ANGLE, 2))
                if observed != (c["homing_offset"], c["range_min"], c["range_max"]):
                    raise RuntimeError(f"{name}: 보정 파일/EEPROM 불일치")
            if read_required(bus, sid, A_PHASE) != expected_phase:
                raise RuntimeError("그리퍼 Phase 불일치")
            delta *= -1 if calibration["gripper"]["drive_mode"] else 1
        target = start + delta
        return_goal_ticks = start + delta // 2 if gripper else start
        if not 0 <= lo < hi <= 4095 or not lo <= start <= hi:
            raise RuntimeError("현재 위치가 하드웨어 위치 제한 밖입니다")
        if not lo + 8 <= min(target, return_goal_ticks) <= max(target, return_goal_ticks) <= hi - 8:
            raise RuntimeError("현재 위치/목표가 하드웨어 위치 제한 여유 밖입니다")
        # 토크 OFF 상태에서 사용자가 자세를 계속 바꾸고 있는지 확인한다.
        sleep(0.05)
        if abs(read_required(bus, sid, A_POS, 2) - start) > 3:
            raise RuntimeError("현재 시험 축 위치가 변하고 있습니다")
        result.update(start_raw=start, target_raw=target, hardware_limits=[lo, hi])
        if gripper:
            result.update(partial_close_raw=return_goal_ticks,
                          gripper_drive_mode=calibration["gripper"]["drive_mode"], expected_phase=expected_phase)
        previous = (read_required(bus, sid, A_SPEED, 2), read_required(bus, sid, A_ACCEL))
        if not execute:
            return result
        monitor()
        mutated = True
        require_write(bus, sid, A_GOAL, start, 2)
        require_write(bus, sid, A_SPEED, 80, 2)
        require_write(bus, sid, A_ACCEL, 5)
        if read_required(bus, sid, A_GOAL, 2) != start:
            raise RuntimeError("토크 ON 전 현재 위치 목표 확인 실패")
        monitor()
        result["motion_command_emitted"] = True
        require_write(bus, sid, A_TORQUE, 1)
        if read_required(bus, sid, A_TORQUE) != 1:
            raise RuntimeError("토크 ON 확인 실패")
        phases = (("OPEN_PROBE", target), ("PARTIAL_CLOSE", return_goal_ticks)) if gripper else (("OUTBOUND", target), ("RETURN", start))
        for phase, goal in phases:
            monitor()
            require_write(bus, sid, A_GOAL, goal, 2)
            if read_required(bus, sid, A_GOAL, 2) != goal:
                raise RuntimeError("목표 readback 실패")
            now = clock()
            deadline, last_motion, last = now + (4.0 if gripper else 2.0), now, read_required(bus, sid, A_POS, 2)
            while True:
                monitor()
                position = read_required(bus, sid, A_POS, 2)
                result["samples"].append({"phase": phase, "raw": position, "goal": goal})
                if not max(lo, min(start, target) - 8) <= position <= min(hi, max(start, target) + 8):
                    raise RuntimeError("계획된 작은 범위 밖으로 위치가 벗어났습니다")
                if read_required(bus, sid, A_TORQUE) != 1:
                    raise RuntimeError("실행 중 토크 해제 관측")
                if abs(position - goal) <= tolerance_ticks:
                    break
                now = clock()
                if abs(position - last) >= 2:
                    last_motion, last = now, position
                if now >= deadline or now - last_motion >= 0.5:
                    raise RuntimeError(f"{phase}: 목표 미도달/스톨")
                sleep(0.025)
            # 잠깐 유지하는 동안에도 정지 요청을 감시한다.
            for _ in range(8):
                monitor()
                sleep(0.025)
        result["partial_cycle_reached" if gripper else "roundtrip_reached"] = True
    except (Exception, SystemExit, KeyboardInterrupt) as exc:
        result["error"] = str(exc) or type(exc).__name__
    finally:
        if mutated:
            # 실패 시 원위치 복귀를 시도하지 않는다. 현재 목표 정지 + OFF만 수행한다.
            try:
                require_write(bus, sid, A_TORQUE, 0)
            except (Exception, SystemExit) as exc:
                result["stop_errors"].append(str(exc))
            try:
                pos = read_required(bus, sid, A_POS, 2)
                require_write(bus, sid, A_GOAL, pos, 2)
            except (Exception, SystemExit) as exc:
                result["stop_errors"].append(str(exc))
            for address, value, size in ((A_SPEED, previous[0], 2), (A_ACCEL, previous[1], 1)):
                try:
                    require_write(bus, sid, address, value, size)
                    if read_required(bus, sid, address, size) != value:
                        raise RuntimeError("구동 설정 복원 확인 실패")
                except (Exception, SystemExit) as exc:
                    result["stop_errors"].append(str(exc))
            # 설정 복원 뒤 해제 상태를 다시 확인한다. ACK만으로 OFF라고 판정하지 않는다.
            result["release_attempts"] = []
            for _ in range(3):
                attempt = {}
                try:
                    require_write(bus, sid, A_TORQUE, 0)
                    sleep(0.05)
                    attempt["torque"] = read_required(bus, sid, A_TORQUE)
                except (Exception, SystemExit) as exc:
                    attempt["error"] = str(exc)
                result["release_attempts"].append(attempt)
                if attempt.get("torque") == 0:
                    break
            try:
                result["final_torque"] = [read_required(bus, sid, A_TORQUE) for sid in range(1, 7)]
                result["final_raw"] = [read_required(bus, sid, A_POS, 2) for sid in range(1, 7)]
                if result["final_torque"] != [0] * 6:
                    raise RuntimeError("최종 토크 OFF 확인 실패")
            except (Exception, SystemExit) as exc:
                result["stop_errors"].append(str(exc))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--delta-ticks", type=int, help="손목 기본 32; --calibration 그리퍼 기본 128")
    parser.add_argument("--calibration", type=Path, help="지정하면 ID 6 부분 개폐 시험")
    parser.add_argument("--expected-gripper-phase", type=int, choices=(12, 76))
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dashboard", default="http://127.0.0.1:8770/api/stop/status")
    args = parser.parse_args()
    calibration = None
    calibration_sha = None
    if args.calibration:
        if args.expected_gripper_phase is None:
            parser.error("그리퍼 시험에는 --expected-gripper-phase가 필요합니다")
        raw = args.calibration.read_bytes()
        calibration = json.loads(raw)
        calibration_sha = hashlib.sha256(raw).hexdigest()
    elif args.expected_gripper_phase is not None:
        parser.error("Phase 지정에는 --calibration이 필요합니다")
    delta = args.delta_ticks if args.delta_ticks is not None else (128 if calibration is not None else 32)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # 증거 파일을 만들 수 없는 경우에는 토크 ON 전에 거절한다.
    with args.output.open("x", encoding="utf-8") as handle:
        interrupted = threading.Event()
        def interrupt(_signum, _frame):
            # 정지·복원 도중에도 신호가 cleanup을 끊지 않도록 요청만 기록한다.
            interrupted.set()
        def monitor():
            if interrupted.is_set():
                raise KeyboardInterrupt("실행 중단")
            dashboard_monitor(args.dashboard)
            if interrupted.is_set():
                raise KeyboardInterrupt("실행 중단")
        signal.signal(signal.SIGINT, interrupt)
        signal.signal(signal.SIGTERM, interrupt)
        bus = open_stop_bus(args.port)
        try:
            result = run(bus, delta, args.execute, monitor=monitor,
                         calibration=calibration, expected_phase=args.expected_gripper_phase)
            result["port"] = args.port
            if calibration is not None:
                result.update(calibration_path=str(args.calibration.resolve()), calibration_sha256=calibration_sha)
        finally:
            bus.close()
        json.dump(result, handle, ensure_ascii=False, indent=2)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    if result.get("error") or result["stop_errors"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
