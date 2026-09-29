#!/usr/bin/env python3
"""현재 raw 위치 기준 손목 회전 한 축 왕복 시험. 보정 파일·EEPROM 변경 없음."""
import argparse
import json
from pathlib import Path
import signal
import sys
import threading
import time
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dashboard_stop import open_stop_bus
from servo.sts_bus import A_ACCEL, A_GOAL, A_MAX_ANGLE, A_MIN_ANGLE, A_POS, A_SPEED, A_TORQUE
from servo.execute_safe_recovery import read_required, require_write

# STS3215 Operating_Mode: LeRobot feetech/tables.py, position mode = 0.
OPERATING_MODE = 33
SID = 5


def dashboard_monitor(url):
    with urlopen(url, timeout=0.3) as response:
        state = json.load(response)
    if not state.get("enabled") or state.get("requested") or state.get("running"):
        raise RuntimeError("대시보드 정지 요청 또는 정지 연결 미확인")


def run(bus, delta=32, execute=False, monitor=lambda: None,
        sleep=time.sleep, clock=time.monotonic):
    if type(delta) is not int or delta != 32:
        raise ValueError("이 시험은 현재 위치 +32 tick 손목 회전만 허용합니다")
    result = {"schema": "single_wrist_raw_jog_v1", "servo_id": SID,
              "motion_command_emitted": False, "roundtrip_reached": False,
              "calibration_used": False, "samples": [], "stop_errors": []}
    previous = None
    mutated = False
    try:
        monitor()
        result["initial_raw"] = [read_required(bus, sid, A_POS, 2) for sid in range(1, 7)]
        torque = [read_required(bus, sid, A_TORQUE) for sid in range(1, 7)]
        if torque != [0] * 6:
            raise RuntimeError("다른 제어기가 켠 토크가 있습니다. 이 시험은 토크 OFF 상태에서 시작합니다")
        if read_required(bus, SID, OPERATING_MODE) != 0:
            raise RuntimeError("손목이 위치 제어 모드가 아닙니다")
        start = result["initial_raw"][SID - 1]
        lo, hi = read_required(bus, SID, A_MIN_ANGLE, 2), read_required(bus, SID, A_MAX_ANGLE, 2)
        target = start + delta
        if not 0 <= lo < hi <= 4095 or not lo + 8 <= min(start, target) <= max(start, target) <= hi - 8:
            raise RuntimeError("현재 위치/목표가 하드웨어 위치 제한 여유 밖입니다")
        # 토크 OFF 상태에서 사용자가 자세를 계속 바꾸고 있는지 확인한다.
        sleep(0.05)
        if abs(read_required(bus, SID, A_POS, 2) - start) > 3:
            raise RuntimeError("현재 손목 위치가 변하고 있습니다")
        result.update(start_raw=start, target_raw=target, hardware_limits=[lo, hi])
        previous = (read_required(bus, SID, A_SPEED, 2), read_required(bus, SID, A_ACCEL))
        if not execute:
            return result
        monitor()
        mutated = True
        require_write(bus, SID, A_GOAL, start, 2)
        require_write(bus, SID, A_SPEED, 80, 2)
        require_write(bus, SID, A_ACCEL, 5)
        if read_required(bus, SID, A_GOAL, 2) != start:
            raise RuntimeError("토크 ON 전 현재 위치 목표 확인 실패")
        monitor()
        result["motion_command_emitted"] = True
        require_write(bus, SID, A_TORQUE, 1)
        if read_required(bus, SID, A_TORQUE) != 1:
            raise RuntimeError("토크 ON 확인 실패")
        for phase, goal in (("OUTBOUND", target), ("RETURN", start)):
            monitor()
            require_write(bus, SID, A_GOAL, goal, 2)
            if read_required(bus, SID, A_GOAL, 2) != goal:
                raise RuntimeError("목표 readback 실패")
            now = clock()
            deadline, last_motion, last = now + 2.0, now, read_required(bus, SID, A_POS, 2)
            while True:
                monitor()
                position = read_required(bus, SID, A_POS, 2)
                result["samples"].append({"phase": phase, "raw": position, "goal": goal})
                if not min(start, target) - 8 <= position <= max(start, target) + 8:
                    raise RuntimeError("계획된 작은 범위 밖으로 위치가 벗어났습니다")
                if read_required(bus, SID, A_TORQUE) != 1:
                    raise RuntimeError("실행 중 토크 해제 관측")
                if abs(position - goal) <= 4:
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
        result["roundtrip_reached"] = True
    except (Exception, SystemExit, KeyboardInterrupt) as exc:
        result["error"] = str(exc) or type(exc).__name__
    finally:
        if mutated:
            # 실패 시 원위치 복귀를 시도하지 않는다. 현재 목표 정지 + OFF만 수행한다.
            try:
                require_write(bus, SID, A_TORQUE, 0)
            except (Exception, SystemExit) as exc:
                result["stop_errors"].append(str(exc))
            try:
                pos = read_required(bus, SID, A_POS, 2)
                require_write(bus, SID, A_GOAL, pos, 2)
            except (Exception, SystemExit) as exc:
                result["stop_errors"].append(str(exc))
            for address, value, size in ((A_SPEED, previous[0], 2), (A_ACCEL, previous[1], 1)):
                try:
                    require_write(bus, SID, address, value, size)
                    if read_required(bus, SID, address, size) != value:
                        raise RuntimeError("구동 설정 복원 확인 실패")
                except (Exception, SystemExit) as exc:
                    result["stop_errors"].append(str(exc))
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
    parser.add_argument("--delta-ticks", type=int, default=32)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dashboard", default="http://127.0.0.1:8770/api/stop/status")
    args = parser.parse_args()
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
            result = run(bus, args.delta_ticks, args.execute, monitor=monitor)
            result["port"] = args.port
        finally:
            bus.close()
        json.dump(result, handle, ensure_ascii=False, indent=2)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    if result.get("error") or result["stop_errors"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
