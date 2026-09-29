#!/usr/bin/env python3
"""왼팔 reset 자세에서 팔꿈치 -3도 이탈 한 단계의 후보 생성·제한 실행.

기본 동작은 스냅샷과 calibration을 묶은 JSON 후보만 새 파일에 저장한다.
실물 쓰기는 별도 모델 검토 JSON과 ``--execute``를 모두 지정한 경우에만 가능하다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import xml.etree.ElementTree as ET

import numpy as np

TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import ik_pick_place
from dashboard_stop import open_stop_bus
from plan_body_side_grasp import Chain, URDF_PATH, base_positions
from servo.execute_safe_recovery import degrees_to_raw, read_required
from servo.servo_record_ranges import validate_capture
from servo.small_raw_jog import dashboard_monitor
from servo.sts_bus import A_MAX_ANGLE, A_MIN_ANGLE, A_OFFSET, A_POS, A_TORQUE, decode_offset

JOINTS = ik_pick_place.JOINTS
NAMES = ik_pick_place.NAMES
ELBOW_INDEX = JOINTS.index("elbow_flex")
MAX_DELTA_TICKS = 35
START_TOLERANCE_TICKS = 3
SAMPLE_STEP_DEG = 0.5


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _finite_number(value, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label}: 유한한 숫자가 필요합니다")
    return float(value)


def _snapshot_joint_deg(raw: list[int], calibration: dict) -> list[float]:
    return [
        (value - (calibration[name]["range_min"] + calibration[name]["range_max"]) / 2) * 360 / 4095
        for name, value in zip(JOINTS, raw)
    ]


def _finger_local_corners() -> np.ndarray:
    root = ET.parse(URDF_PATH).getroot()
    link = root.find("./link[@name='left_moving_jaw_link']")
    if link is None:
        raise ValueError("URDF left_moving_jaw_link 누락")
    points = []
    for collision in link.findall("collision"):
        if not collision.get("name", "").startswith("left_moving_finger_"):
            continue
        origin, box = collision.find("origin"), collision.find("geometry/box")
        if origin is None or box is None:
            raise ValueError("URDF 손가락 collision box 누락")
        center = np.asarray([float(v) for v in origin.get("xyz", "").split()])
        size = np.asarray([float(v) for v in box.get("size", "").split()])
        if center.shape != (3,) or size.shape != (3,) or np.any(size <= 0):
            raise ValueError("URDF 손가락 collision 형상 오류")
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    points.append(center + size * np.array([sx, sy, sz]) / 2)
    if not points:
        raise ValueError("URDF 손가락 collision 형상 누락")
    return np.asarray(points)


def _fk_metrics(chain: Chain, raw: list[int], calibration: dict, baseline: tuple[float, float] | None = None):
    q_deg = _snapshot_joint_deg(raw[:5], calibration)
    q_rad = np.radians(q_deg)
    transforms = chain.transforms(base_positions("left", q_rad))
    tcp_z = float(transforms["left_tool0"][2, 3])
    jaw = transforms["left_moving_jaw_link"]
    corners = _finger_local_corners()
    world = (jaw[:3, :3] @ corners.T).T + jaw[:3, 3]
    finger_bottom_z = float(np.min(world[:, 2]))
    limits = [chain.limits[f"left_{name}"] for name in JOINTS]
    margin_deg = min(
        math.degrees(min(value - lower, upper - value))
        for value, (lower, upper) in zip(q_rad, limits)
    )
    if baseline is None:
        baseline = (tcp_z, finger_bottom_z)
    return {
        "tcp_z_relative_m": tcp_z - baseline[0],
        "finger_bottom_relative_m": finger_bottom_z - baseline[1],
        "joint_margin_deg": margin_deg,
    }, baseline


def prepare(snapshot: dict, calibration_bytes: bytes) -> dict:
    calibration = json.loads(calibration_bytes)
    validate_capture(calibration)
    if [calibration[name]["id"] for name in NAMES] != list(range(1, 7)):
        raise ValueError("이 제한 단계는 calibration ID 1..6 순서에만 적용됩니다")
    if calibration["elbow_flex"]["id"] != 3:
        raise ValueError("elbow_flex는 예상 ID 3이어야 합니다")
    if snapshot.get("calibration_sha256") != _sha(calibration_bytes):
        raise ValueError("스냅샷과 calibration 해시가 다릅니다")
    if snapshot.get("hardware_accessed") is not True or snapshot.get("motion_command_emitted") is not False:
        raise ValueError("ik_pick_place.snapshot 결과만 허용합니다")
    raw, torque = snapshot.get("raw_ticks"), snapshot.get("torque")
    if not isinstance(raw, list) or len(raw) != 6 or any(type(v) is not int for v in raw):
        raise ValueError("스냅샷 raw_ticks는 정수 6개여야 합니다")
    if torque != [0] * 6:
        raise ValueError("스냅샷의 모든 토크가 OFF여야 합니다")
    for name, value in zip(NAMES, raw):
        item = calibration[name]
        if not item["range_min"] <= value <= item["range_max"]:
            raise ValueError(f"{name}: 현재 raw가 calibration 범위 밖입니다")
    expected_q = _snapshot_joint_deg(raw[:5], calibration)
    supplied_q = snapshot.get("joint_deg")
    if not isinstance(supplied_q, list) or len(supplied_q) != 5 or not np.allclose(
        supplied_q, expected_q, rtol=0, atol=1e-9
    ):
        raise ValueError("스냅샷 joint_deg와 raw_ticks가 일치하지 않습니다")

    target = raw.copy()
    target[ELBOW_INDEX] = degrees_to_raw(
        expected_q[ELBOW_INDEX] - 3.0,
        calibration["elbow_flex"]["range_min"],
        calibration["elbow_flex"]["range_max"],
    )
    delta = [b - a for a, b in zip(raw, target)]
    if not -MAX_DELTA_TICKS <= delta[ELBOW_INDEX] < 0 or any(
        value != 0 for index, value in enumerate(delta) if index != ELBOW_INDEX
    ):
        raise ValueError(f"ID 3의 -3도 제한 단계가 아닙니다: raw delta={delta}")
    if not calibration["elbow_flex"]["range_min"] <= target[ELBOW_INDEX] <= calibration["elbow_flex"]["range_max"]:
        raise ValueError("ID 3 목표가 calibration 범위 밖입니다")

    snapshot_sha = ik_pick_place.digest(snapshot)
    return {
        "schema": "ik_reset_clear_step_v1",
        "servo_ids": list(range(1, 7)),
        "start_raw": raw,
        "config_sha256": snapshot_sha,
        "snapshot_sha256": snapshot_sha,
        "calibration_sha256": _sha(calibration_bytes),
        "urdf_sha256": _sha(URDF_PATH.read_bytes()),
        "poses": [{"phase": "CLEAR_RESET_STEP", "raw_ticks": target}],
        "max_raw_delta_ticks": max(abs(value) for value in delta),
        "motion_command_emitted": False,
        "hardware_accessed": False,
        "physical_cup_grasp_verified": False,
        "full_physical_geometry_verified": False,
    }


def expected_review_samples(plan: dict, calibration: dict) -> list[dict]:
    """리뷰 raw 표본의 FK를 시작 표본 대비 상승량(m)과 URDF 관절 여유로 반환한다."""
    start, end = plan["start_raw"], plan["poses"][0]["raw_ticks"]
    count = max(1, math.ceil(3.0 / SAMPLE_STEP_DEG))
    raws = [[int(round(a + (b - a) * i / count)) for a, b in zip(start, end)] for i in range(count + 1)]
    chain, baseline, result = Chain(URDF_PATH), None, []
    for raw in raws:
        metrics, baseline = _fk_metrics(chain, raw, calibration, baseline)
        result.append({"raw_ticks": raw, **metrics})
    return result


def validate_model_review(plan: dict, review: dict, calibration_bytes: bytes) -> None:
    calibration = json.loads(calibration_bytes)
    end = plan["poses"][0]["raw_ticks"]
    if review.get("schema") != "reset_clear_model_review_v1" or review.get("accepted") is not True:
        raise ValueError("승인된 reset clear 모델 검토가 필요합니다")
    for key, expected in (("start_raw", plan["start_raw"]), ("end_raw", end),
                          ("calibration_sha256", plan["calibration_sha256"]),
                          ("urdf_sha256", plan["urdf_sha256"])):
        if review.get(key) != expected:
            raise ValueError(f"모델 검토 {key} 바인딩 불일치")
    for key in ("physical_geometry_verified", "continuous_collision_validated",
                "initial_contact_escape_completed"):
        if review.get(key) is not False:
            raise ValueError(f"모델 검토 {key}=false가 필요합니다")
    if _finite_number(review.get("max_joint_step_deg"), "max_joint_step_deg") > SAMPLE_STEP_DEG:
        raise ValueError("모델 검토 샘플 간격이 0.5도를 넘습니다")
    maximum_x = _finite_number(review.get("maximum_arm_x_relative_m"), "maximum_arm_x_relative_m")
    cup_x = _finite_number(review.get("cup_near_face_x_relative_m"), "cup_near_face_x_relative_m")
    if maximum_x >= cup_x:
        raise ValueError("모델상 팔 최대 X가 컵 근접면을 넘습니다")

    expected_samples = expected_review_samples(plan, calibration)
    samples = review.get("samples")
    if not isinstance(samples, list) or len(samples) != len(expected_samples):
        raise ValueError("모델 검토 FK 샘플 수 불일치")
    initial_pairs = None
    previous_tcp = previous_finger = previous_margin = None
    review_baseline = None
    for index, (actual, expected) in enumerate(zip(samples, expected_samples)):
        if actual.get("raw_ticks") != expected["raw_ticks"]:
            raise ValueError(f"모델 검토 sample {index} raw 변조")
        for key in ("tcp_z_relative_m", "finger_bottom_relative_m", "joint_margin_deg"):
            _finite_number(actual.get(key), f"sample {index} {key}")
        if review_baseline is None:
            review_baseline = (float(actual["tcp_z_relative_m"]), float(actual["finger_bottom_relative_m"]))
        # 검토기는 작업셀 기준 상대 좌표를 기록한다. URDF FK로 그 좌표의 샘플 간
        # 상승량을 독립 재계산해, 좌표 원점 차이는 허용하되 궤적 변조는 거절한다.
        for key, baseline in zip(("tcp_z_relative_m", "finger_bottom_relative_m"), review_baseline):
            if not math.isclose(float(actual[key]) - baseline, expected[key], rel_tol=0, abs_tol=1e-6):
                raise ValueError(f"모델 검토 sample {index} FK 상승량 불일치: {key}")
        if not math.isclose(float(actual["joint_margin_deg"]), expected["joint_margin_deg"],
                            rel_tol=0, abs_tol=1e-6):
            raise ValueError(f"모델 검토 sample {index} FK 값 불일치: joint_margin_deg")
        pairs = actual.get("self_collision_pairs")
        if not isinstance(pairs, list) or any(not isinstance(pair, list) or len(pair) != 2 for pair in pairs):
            raise ValueError("self_collision_pairs 형식 오류")
        pair_set = {tuple(pair) for pair in pairs}
        initial_pairs = pair_set if initial_pairs is None else initial_pairs
        if not pair_set <= initial_pairs:
            raise ValueError("짧은 단계에서 새로운 self collision pair가 생겼습니다")
        tcp, finger, margin = (float(actual[k]) for k in
                               ("tcp_z_relative_m", "finger_bottom_relative_m", "joint_margin_deg"))
        if margin <= 0 or (previous_tcp is not None and
                (tcp < previous_tcp - 1e-12 or finger < previous_finger - 1e-12 or margin < previous_margin - 1e-12)):
            raise ValueError("TCP/손가락 상승 또는 관절 여유가 단조 비감소가 아닙니다")
        previous_tcp, previous_finger, previous_margin = tcp, finger, margin
    if (previous_tcp is None or previous_tcp - review_baseline[0] <= 0 or
            previous_finger - review_baseline[1] <= 0):
        raise ValueError("모델상 TCP와 손가락 최저점이 양의 방향으로 상승하지 않습니다")


def validate_hardware(bus, plan: dict, calibration_bytes: bytes) -> None:
    calibration = json.loads(calibration_bytes)
    validate_capture(calibration)
    positions, torque = [], []
    for name in NAMES:
        item, sid = calibration[name], calibration[name]["id"]
        observed = (
            decode_offset(read_required(bus, sid, A_OFFSET, 2)),
            read_required(bus, sid, A_MIN_ANGLE, 2),
            read_required(bus, sid, A_MAX_ANGLE, 2),
        )
        expected = (item["homing_offset"], item["range_min"], item["range_max"])
        if observed != expected:
            raise RuntimeError(f"{name}: calibration/EEPROM offset·limit 불일치")
        positions.append(read_required(bus, sid, A_POS, 2))
        torque.append(read_required(bus, sid, A_TORQUE))
    drift = [a - b for a, b in zip(positions, plan["start_raw"])]
    if max(abs(value) for value in drift) > START_TOLERANCE_TICKS:
        raise RuntimeError(f"실행 직전 시작 raw가 바뀌었습니다: {drift}")
    if torque != [0] * 6:
        raise RuntimeError("실행 직전 모든 토크가 OFF여야 합니다")


def execute(bus, plan: dict, calibration_bytes: bytes, monitor) -> dict:
    result = ik_pick_place.run_sequence(
        bus, plan, lambda phase: phase == "CLEAR_RESET_STEP", monitor=monitor,
        preflight=lambda: validate_hardware(bus, plan, calibration_bytes),
    )
    result.update(
        schema="ik_reset_clear_step_result_v1",
        snapshot_sha256=plan["snapshot_sha256"],
        calibration_sha256=plan["calibration_sha256"],
        urdf_sha256=plan["urdf_sha256"],
        physical_cup_grasp_verified=False,
        full_physical_geometry_verified=False,
        temperature_load_read=False,
    )
    try:
        result["final_torque"] = [read_required(bus, sid, A_TORQUE) for sid in plan["servo_ids"]]
        if result["final_torque"] != [0] * 6:
            result["stop_errors"].append("최종 전체 토크 OFF 확인 실패")
            result["sequence_completed"] = False
    except BaseException as exc:
        result["stop_errors"].append(f"최종 토크 확인 실패: {exc}")
        result["sequence_completed"] = False
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-file", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--stop-status-url")
    parser.add_argument("--model-review", type=Path)
    args = parser.parse_args()
    protected = {args.snapshot_file.resolve(), args.calibration.resolve()}
    if args.model_review is not None:
        protected.add(args.model_review.resolve())
    if args.output.exists() or args.output.resolve() in protected:
        parser.error("출력은 기존 파일이나 입력을 덮어쓸 수 없습니다")
    if args.execute and (not args.port or not args.stop_status_url or args.model_review is None):
        parser.error("실행에는 --port, --stop-status-url, --model-review가 모두 필요합니다")
    if not args.execute and (args.port or args.stop_status_url or args.model_review):
        parser.error("포트·정지 상태·모델 검토는 --execute에서만 사용합니다")

    calibration_bytes = args.calibration.read_bytes()
    plan = prepare(json.loads(args.snapshot_file.read_text()), calibration_bytes)
    result = plan
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # 동작 전에 기록 경로를 확보한다. 중간 종료 시 실행 미확정 pending 기록을 남긴다.
    with args.output.open("x", encoding="utf-8") as stream:
        initial = plan if not args.execute else {
            "schema": "ik_reset_clear_step_pending_v1", "status": "execution_pending",
            "motion_command_emitted": None, "hardware_accessed": None, "plan": plan,
            "physical_cup_grasp_verified": False,
        }
        json.dump(initial, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    interrupted = threading.Event()
    old_handlers = {}
    bus = None
    if args.execute:
        validate_model_review(plan, json.loads(args.model_review.read_text()), calibration_bytes)

        def interrupt(_signum, _frame):
            interrupted.set()

        def monitor():
            if interrupted.is_set():
                raise KeyboardInterrupt("실행 중단")
            dashboard_monitor(args.stop_status_url)
            if interrupted.is_set():
                raise KeyboardInterrupt("실행 중단")

        old_handlers = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            bus = open_stop_bus(args.port)
            result = execute(bus, plan, calibration_bytes, monitor)
            result.update(hardware_accessed=True, motion_command_emitted=result["command_write_attempted"],
                          model_review_path=str(args.model_review.resolve()))
        finally:
            if bus is not None:
                bus.close()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)

    with args.output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    print(args.output)
    return 2 if result.get("failure") or result.get("stop_errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
