#!/usr/bin/env python3
"""LeRobot 공식 수동 보정에 백업·복원을 덧붙인다. 이동 목표는 보내지 않는다.

robot.connect/configure 대신 bus.connect와 robot.calibrate를 사용한다.
새 보정은 별도 디렉터리에 저장하며 저장소의 활성 JSON은 자동 교체하지 않는다.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import signal
import subprocess


PRESERVED = ("Operating_Mode", "Lock", "Phase", "P_Coefficient",
             "I_Coefficient", "D_Coefficient")


def save(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def calibrate_connected(robot, directory, source_bytes):
    """연결된 버스에서 공식 calibrate만 호출하고 실패하면 실제 보정 백업을 복원한다."""
    bus = robot.bus
    names = list(bus.motors)
    before = bus.read_calibration()
    torque = {n: bus.read("Torque_Enable", n, normalize=False) for n in names}
    if any(value != 0 for value in torque.values()):
        raise ValueError("시작 시 전축 토크 OFF가 필요합니다. 자동으로 해제하지 않습니다")
    settings = {n: {reg: bus.read(reg, n, normalize=False) for reg in PRESERVED} for n in names}
    save(directory / "before.json", {"calibration": {n: asdict(v) for n, v in before.items()},
                                    "settings": settings, "torque": torque})
    with (directory / "active_calibration.backup.json").open("xb") as stream:
        stream.write(source_bytes)
    result = {"calibration_completed": False, "motion_command_emitted": False,
              "physical_mapping_verified": False, "restoration_errors": []}
    failure = None
    try:
        # LeRobot의 두 입력 단계(중간 자세, 전체 범위 기록)를 그대로 사용한다.
        robot.calibrate()
        if bus.read_calibration() != robot.calibration:
            raise RuntimeError("공식 보정 결과와 보드 재판독 불일치")
        for name in names:
            item = robot.calibration[name]
            pos = bus.read("Present_Position", name, normalize=False)
            if not 0 <= item.range_min < item.range_max <= 4095:
                raise RuntimeError(f"{name}: 잘못된 보정 범위")
            if not item.range_min <= pos <= item.range_max:
                raise RuntimeError(f"{name}: 현재 위치가 새 범위 밖")
            for reg in PRESERVED[2:]:
                if bus.read(reg, name, normalize=False) != settings[name][reg]:
                    raise RuntimeError(f"{name}: 예상하지 않은 {reg} 변경")
        result["calibration_completed"] = True
    except BaseException as exc:
        failure = exc
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        for name in names:
            try:
                bus.write("Torque_Enable", name, 0)
                bus.write("Lock", name, 0)
                if failure is not None:
                    bus.write_calibration({name: before[name]}, cache=False)
                bus.write("Operating_Mode", name, settings[name]["Operating_Mode"])
                bus.write("Lock", name, settings[name]["Lock"])
                if bus.read("Torque_Enable", name, normalize=False) != 0:
                    raise RuntimeError("토크 OFF 재판독 실패")
                for reg in PRESERVED:
                    if bus.read(reg, name, normalize=False) != settings[name][reg]:
                        raise RuntimeError(f"{reg} 재판독 불일치")
            except BaseException as exc:
                result["restoration_errors"].append(f"{name}: {exc}")
        if failure is not None:
            try:
                if bus.read_calibration() != before:
                    raise RuntimeError("기존 보정 재판독 불일치")
            except BaseException as exc:
                result["restoration_errors"].append(str(exc))
        if result["restoration_errors"]:
            result["calibration_completed"] = False
        save(directory / "result.json", result)
    if result["restoration_errors"]:
        raise RuntimeError(f"복원/종료 확인 실패: {result['restoration_errors']}") from failure
    if failure is not None:
        raise failure
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    source = args.calibration.read_bytes()
    owners = subprocess.run(["fuser", args.port], capture_output=True, timeout=2)
    if owners.returncode != 1:
        parser.error("포트 점유 또는 점유 확인 실패")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    robot = SOFollower(SOFollowerRobotConfig(port=args.port, id="candidate",
                      calibration_dir=args.output_dir, use_degrees=True, cameras={}))
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    try:
        robot.bus.connect()
        robot.bus.port_handler.ser.exclusive = True
        calibrate_connected(robot, args.output_dir, source)
        print(f"보정 후보: {robot.calibration_fpath}. 활성 JSON은 아직 교체하지 않았습니다.")
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
