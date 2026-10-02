#!/usr/bin/env python3
"""기록 후보의 한계값만 적용한다. 오프셋·Phase·구동 목표는 쓰지 않는다.

기본은 읽기 대조다. --execute는 검증 후 한계값을 쓰며, 실패하면 변경을
시도한 한계값을 복구한다. 통신 단절 시 복구도 실패할 수 있으므로 결과를
확인하고 전원 재인가 후 읽기 대조를 마치기 전에는 이동하지 않는다.
"""
import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dashboard_stop import open_stop_bus
from servo.servo_record_ranges import SO101, validate_capture
from servo.sts_bus import (A_D, A_I, A_LOCK, A_MAX_ANGLE, A_MIN_ANGLE,
                          A_OFFSET, A_P, A_PHASE, A_POS, A_TORQUE, decode_offset)

FIELDS = {"min": (A_MIN_ANGLE, 2), "max": (A_MAX_ANGLE, 2),
          "offset_raw": (A_OFFSET, 2), "phase": (A_PHASE, 1),
          "p": (A_P, 1), "d": (A_D, 1), "i": (A_I, 1),
          "torque": (A_TORQUE, 1), "pos": (A_POS, 2), "lock": (A_LOCK, 1)}
PROTECTED = ("offset_raw", "phase", "p", "d", "i", "lock")


def snapshot(bus):
    rows = {}
    for sid in range(1, 7):
        row = {}
        for key, (addr, width) in FIELDS.items():
            value = bus.read(sid, addr, width)
            if type(value) is not int or not 0 <= value < (1 << (8 * width)):
                raise RuntimeError(f"ID {sid}: {key} 읽기 실패")
            row[key] = value
        rows[sid] = row
    return rows


def check_before(rows, candidate, expected, phases):
    validate_capture(candidate)
    if len(phases) != 6:
        raise ValueError("Phase 기대값은 6개여야 합니다")
    for name in SO101:
        c, old = candidate[name], expected[name]
        sid = c["id"]
        row = rows[sid]
        if old["id"] != sid or c["homing_offset"] != old["homing_offset"]:
            raise ValueError(f"{name}: 후보/백업 offset 또는 ID 불일치")
        if decode_offset(row["offset_raw"]) != old["homing_offset"]:
            raise ValueError(f"{name}: 현재 offset이 백업과 다릅니다")
        if (row["min"], row["max"]) != (old["range_min"], old["range_max"]):
            raise ValueError(f"{name}: 현재 한계값이 백업과 다릅니다")
        if row["phase"] != phases[sid - 1]:
            raise ValueError(f"{name}: Phase 불일치")
        if row["torque"] != 0 or row["lock"] != 1:
            raise ValueError(f"{name}: 토크 OFF/EEPROM 잠금 상태가 아닙니다")
        if not c["range_min"] <= row["pos"] <= c["range_max"]:
            raise ValueError(f"{name}: 현재 위치가 후보 범위 밖입니다")
        if max(row["min"], c["range_min"]) >= min(row["max"], c["range_max"]):
            raise ValueError(f"{name}: 기존/후보 범위가 겹치지 않습니다")


def write_limit(bus, sid, addr, value):
    if addr not in (A_MIN_ANGLE, A_MAX_ANGLE):
        raise ValueError("한계값 외 EEPROM 쓰기는 허용하지 않습니다")
    if bus.read(sid, A_TORQUE) != 0:
        raise RuntimeError(f"ID {sid}: 토크 OFF 확인 실패")
    try:
        if not bus.write(sid, A_LOCK, 0):
            raise RuntimeError(f"ID {sid}: 잠금 해제 ACK 없음")
        time.sleep(0.05)
        if not bus.write(sid, addr, value, 2):
            raise RuntimeError(f"ID {sid}: 한계 쓰기 ACK 없음")
        time.sleep(0.08)
    finally:
        if not bus.write(sid, A_LOCK, 1) or bus.read(sid, A_LOCK) != 1:
            raise RuntimeError(f"ID {sid}: EEPROM 재잠금 실패")
        time.sleep(0.05)
    if bus.read(sid, addr, 2) != value:
        raise RuntimeError(f"ID {sid}: 한계값 readback 불일치")


def verify_after(bus, before, candidate):
    after = snapshot(bus)
    for name in SO101:
        c = candidate[name]
        sid = c["id"]
        row = after[sid]
        if (row["min"], row["max"]) != (c["range_min"], c["range_max"]):
            raise RuntimeError(f"{name}: 최종 한계 대조 실패")
        if row["torque"] != 0 or any(row[k] != before[sid][k] for k in PROTECTED):
            raise RuntimeError(f"{name}: 보존 설정/토크 대조 실패")
    return after


def apply_ranges(bus, candidate, expected, phases, persist, cancelled=lambda: False):
    before = snapshot(bus)
    check_before(before, candidate, expected, phases)
    # 디스크 기록 실패 시 EEPROM을 건드리지 않는다.
    persist({"status": "prepared", "before": before, "candidate": candidate})
    touched = []
    try:
        for name in SO101:
            c = candidate[name]
            sid = c["id"]
            for key, addr in (("min", A_MIN_ANGLE), ("max", A_MAX_ANGLE)):
                if cancelled():
                    raise RuntimeError("중단 요청")
                value = c["range_" + key]
                if value == before[sid][key]:
                    continue
                # ACK 유실이어도 쓰기가 도착했을 수 있으므로 호출 전에 복구 목록에 추가.
                touched.append((sid, addr, before[sid][key]))
                write_limit(bus, sid, addr, value)
        if cancelled():
            raise RuntimeError("중단 요청")
        after = verify_after(bus, before, candidate)
        result = {"status": "applied_pending_power_cycle", "before": before,
                  "after": after, "candidate": candidate,
                  "motion_command_emitted": False}
        persist(result)
        return result
    except (Exception, SystemExit) as exc:
        errors = []
        for sid, addr, old in reversed(touched):
            try:
                write_limit(bus, sid, addr, old)
            except (Exception, SystemExit) as restore_exc:
                errors.append(f"ID {sid}, addr {addr}: {restore_exc}")
        try:
            restored = snapshot(bus)
            for sid in before:
                if any(restored[sid][k] != before[sid][k]
                       for k in ("min", "max", *PROTECTED, "torque")):
                    errors.append(f"ID {sid}: 복구 최종 대조 실패")
        except (Exception, SystemExit) as restore_exc:
            errors.append(str(restore_exc))
        result = {"status": "failed", "error": str(exc), "before": before,
                  "rollback_errors": errors, "rollback_verified": not errors,
                  "motion_command_emitted": False}
        try:
            persist(result)
        except Exception as log_exc:
            result["log_error"] = str(log_exc)
        return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--candidate", required=True)
    ap.add_argument("--expected", required=True, help="기록 시작 전 EEPROM 백업 JSON")
    ap.add_argument("--phases", required=True, nargs=6, type=int)
    ap.add_argument("--report-dir", help="--execute 시 새로 생성할 로컬 결과 폴더")
    ap.add_argument("--execute", action="store_true")
    args = ap.parse_args()
    if args.execute and not args.report_dir:
        ap.error("--execute에는 --report-dir가 필요합니다")
    candidate = json.loads(Path(args.candidate).expanduser().read_text())
    expected = json.loads(Path(args.expected).expanduser().read_text())
    validate_capture(candidate)
    cancelled = [False]
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda *_: cancelled.__setitem__(0, True))
    bus = None
    try:
        bus = open_stop_bus(args.port)
        if not args.execute:
            rows = snapshot(bus)
            check_before(rows, candidate, expected, args.phases)
            print(json.dumps({"status": "preflight_pass", "before": rows}, indent=2))
            return 0
        folder = Path(args.report_dir).expanduser()
        folder.mkdir(parents=True, exist_ok=False)
        with (folder / "journal.jsonl").open("x", encoding="utf-8") as handle:
            def persist(value):
                handle.write(json.dumps(value, ensure_ascii=False) + "\n")
                handle.flush()
                os.fsync(handle.fileno())
            result = apply_ranges(bus, candidate, expected, args.phases, persist,
                                  lambda: cancelled[0])
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["status"] == "applied_pending_power_cycle" else 1
    finally:
        if bus is not None:
            bus.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
