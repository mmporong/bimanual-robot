#!/usr/bin/env python3
"""상단 RGB 컵 검출을 왼팔 DLS IK까지 연결하는 비동작 드라이런 도구.

이 도구는 카메라나 이미지에서 컵을 찾고, 보정된 작업대 평면 homography로
컵의 바닥 접점을 ``base_footprint`` XY로 변환한 뒤, 커밋된 URDF의
``left_cup_tcp``에 대해 pre-grasp와 grasp IK를 푼다. 서보 쓰기 기능은 없다.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
IK_SCRIPT_DIR = REPO_ROOT / "src/hold_flow_description/scripts"
if str(IK_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(IK_SCRIPT_DIR))

from solve_task_poses import (  # noqa: E402
    ARM_JOINTS,
    Chain,
    URDF_PATH,
    base_positions,
    residual,
    solve_with_restarts,
)


DEFAULT_TOP = "/dev/v4l/by-path/pci-0000:65:00.3-usb-0:2.1:1.0-video-index0"
DEFAULT_WORKSPACE = Path("calibration/cameras/top/workspace_plane.yaml")


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    xyxy_px: list[float]
    contact_px: list[float]


def read_frame(image_path: Path | None, camera: str, width: int, height: int) -> np.ndarray:
    if image_path is not None:
        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError(f"이미지를 읽을 수 없습니다: {image_path}")
        return image
    capture = cv2.VideoCapture(camera, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        raise RuntimeError(f"카메라를 열 수 없습니다: {camera}")
    try:
        for _ in range(5):
            ok, frame = capture.read()
        if not ok:
            raise RuntimeError(f"카메라 프레임을 읽을 수 없습니다: {camera}")
        return frame
    finally:
        capture.release()


def select_best_cup(result: Any, minimum_confidence: float) -> Detection | None:
    candidates: list[Detection] = []
    height, width = result.orig_shape
    for box in result.boxes:
        class_id = int(box.cls.item())
        label = result.names[class_id]
        confidence = float(box.conf.item())
        if label != "cup" or confidence < minimum_confidence:
            continue
        x1, y1, x2, y2 = [float(value) for value in box.xyxy[0]]
        # 컵 중심은 높이에 따른 시차를 포함한다. 작업대 평면과 만나는 바닥 중심을
        # homography 입력으로 사용해야 XY 편향이 작다.
        contact_x = min(max((x1 + x2) / 2.0, 0.0), width - 1.0)
        contact_y = min(max(y2, 0.0), height - 1.0)
        candidates.append(
            Detection(
                label=label,
                confidence=confidence,
                xyxy_px=[x1, y1, x2, y2],
                contact_px=[contact_x, contact_y],
            )
        )
    return max(candidates, key=lambda item: item.confidence, default=None)


def detect_cup(image: np.ndarray, model_path: str, minimum_confidence: float) -> Detection | None:
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "ultralytics를 불러오지 못했습니다. 이 호스트에서는 system python3를 사용하세요"
        ) from exc
    model = YOLO(model_path)
    result = model.predict(image, conf=minimum_confidence, verbose=False, device="cpu")[0]
    return select_best_cup(result, minimum_confidence)


def annotate(image: np.ndarray, detection: Detection | None, output: Path) -> None:
    shown = image.copy()
    if detection is None:
        cv2.putText(shown, "cup: not detected", (16, 32), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (30, 30, 220), 2, cv2.LINE_AA)
    else:
        x1, y1, x2, y2 = [round(value) for value in detection.xyxy_px]
        cx, cy = [round(value) for value in detection.contact_px]
        cv2.rectangle(shown, (x1, y1), (x2, y2), (30, 210, 90), 2)
        cv2.circle(shown, (cx, cy), 6, (20, 70, 240), -1)
        cv2.putText(
            shown,
            f"cup {detection.confidence:.3f} | floor contact",
            (x1, max(24, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (30, 120, 30),
            2,
            cv2.LINE_AA,
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), shown):
        raise RuntimeError(f"주석 이미지 저장 실패: {output}")


def load_workspace(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise RuntimeError(
            f"작업대 평면 보정값이 없습니다: {path}. "
            "컵 좌표를 추정해 관절값을 만들지 않습니다"
        )
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    image_points = np.asarray(data["image_points_px"], dtype=float)
    base_points = np.asarray(data["base_points_m"], dtype=float)
    if image_points.shape != base_points.shape or image_points.ndim != 2 or image_points.shape[1] != 2:
        raise RuntimeError("image_points_px와 base_points_m은 같은 N×2 배열이어야 합니다")
    if len(image_points) < 4:
        raise RuntimeError("평면 homography에는 대응점이 최소 4개 필요합니다")
    homography, _ = cv2.findHomography(image_points, base_points, method=0)
    if homography is None:
        raise RuntimeError("작업대 homography 계산에 실패했습니다")
    data["homography"] = homography
    return data


def pixel_to_base_xy(pixel: list[float], homography: np.ndarray) -> np.ndarray:
    point = np.array([pixel[0], pixel[1], 1.0], dtype=float)
    projected = homography @ point
    if abs(projected[2]) < 1e-12:
        raise RuntimeError("homography 투영점이 무한대입니다")
    return projected[:2] / projected[2]


def joint_limit_margin_deg(chain: Chain, side: str, q: np.ndarray) -> float:
    margins = []
    for name, value in zip([f"{side}_{joint}" for joint in ARM_JOINTS], q):
        lower, upper = chain.limits[name]
        margins.append(min(value - lower, upper - value))
    return math.degrees(min(margins))


def solve_pick_plan_at_xy(
    detection: Detection,
    base_xy: np.ndarray,
    surface_z: float,
    grasp_height: float,
    clearance: float,
    approach_pitch_deg: float,
) -> dict[str, Any]:
    grasp = np.array([base_xy[0], base_xy[1], surface_z + grasp_height])
    # 왼팔 장착점에서 컵 몸통을 향하는 수평 접근이다. 순정 죠를 컵 양옆으로
    # 넣고 닫는 방식이며, 수직 top-down 자세처럼 wrist limit에 붙지 않는다.
    left_mount_xy = np.array([0.020, 0.170])
    approach_xy = base_xy - left_mount_xy
    approach_norm = float(np.linalg.norm(approach_xy))
    if approach_norm < 1e-6:
        raise RuntimeError("컵 XY가 왼팔 베이스와 겹쳐 접근 방향을 정할 수 없습니다")
    horizontal = approach_xy / approach_norm
    pitch = math.radians(approach_pitch_deg)
    axis = np.array([
        horizontal[0] * math.cos(pitch),
        horizontal[1] * math.cos(pitch),
        math.sin(pitch),
    ])
    pregrasp = grasp - axis * clearance

    chain = Chain(URDF_PATH)
    stages = {}
    for index, (name, target) in enumerate((("pregrasp", pregrasp), ("grasp", grasp))):
        q, error = solve_with_restarts(
            chain,
            "left",
            "left_cup_tcp",
            target,
            restarts=16,
            seed=17 + index,
            axis_frame="left_tool0",
            axis_target=axis,
        )
        fk = chain.transforms(base_positions("left", q))["left_cup_tcp"][:3, 3]
        stages[name] = {
            "target_base_footprint_m": [round(float(value), 5) for value in target],
            "joint_deg": [round(math.degrees(float(value)), 2) for value in q],
            "fk_base_footprint_m": [round(float(value), 5) for value in fk],
            "position_error_mm": round(error * 1000.0, 2),
            "minimum_joint_limit_margin_deg": round(joint_limit_margin_deg(chain, "left", q), 2),
        }

    maximum_error = max(stage["position_error_mm"] for stage in stages.values())
    minimum_margin = min(stage["minimum_joint_limit_margin_deg"] for stage in stages.values())
    ready = maximum_error <= 5.0 and minimum_margin >= 5.0
    return {
        "mode": "dry_run_only",
        "motion_command_emitted": False,
        "detection": asdict(detection),
        "cup_floor_contact_base_xy_m": [round(float(value), 5) for value in base_xy],
        "approach_axis": [round(float(value), 5) for value in axis],
        "approach_pitch_deg": approach_pitch_deg,
        "grasp_height_above_table_m": grasp_height,
        "solver": "numerical_jacobian_damped_least_squares",
        "stages": stages,
        "ready_for_collision_review": ready,
        "blocking_findings": [] if ready else [
            f"IK 오차 또는 관절 여유 부족: max_error={maximum_error:.2f} mm, "
            f"min_margin={minimum_margin:.2f} deg"
        ],
    }


def solve_pick_plan(detection: Detection, workspace: dict[str, Any]) -> dict[str, Any]:
    base_xy = pixel_to_base_xy(detection.contact_px, workspace["homography"])
    surface_z = float(workspace["surface_z_m"])
    cup_height = float(workspace["cup_height_m"])
    grasp_fraction = float(workspace.get("grasp_height_fraction", 0.55))
    return solve_pick_plan_at_xy(
        detection,
        base_xy,
        surface_z,
        cup_height * grasp_fraction,
        float(workspace.get("pregrasp_clearance_m", 0.060)),
        float(workspace.get("approach_pitch_deg", 0.0)),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, help="카메라 대신 사용할 정지 이미지")
    parser.add_argument("--camera", default=DEFAULT_TOP)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--model", required=True, help="COCO cup 클래스가 있는 Ultralytics 모델")
    parser.add_argument("--confidence", type=float, default=0.40)
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument("--measured-forward-m", type=float,
                        help="왼팔 장착축 기준 컵 전방 거리. workspace homography 대신 단일 실측 좌표 사용")
    parser.add_argument("--measured-lateral-m", type=float, default=0.0,
                        help="왼팔 장착축 기준 컵 좌우 거리(+는 URDF +Y)")
    parser.add_argument("--table-surface-z-m", type=float, default=0.6931)
    parser.add_argument("--grasp-height-above-table-m", type=float, default=0.070)
    parser.add_argument("--approach-pitch-deg", type=float, default=-55.0,
                        help="0은 수평, 음수는 컵을 향해 아래로 기울인 접근")
    parser.add_argument("--pregrasp-clearance-m", type=float, default=0.060)
    parser.add_argument("--annotated", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--plan",
        action="store_true",
        help="보정된 작업대 좌표와 URDF로 pre-grasp/grasp IK까지 계산",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image = read_frame(args.image, args.camera, args.width, args.height)
    detection = detect_cup(image, args.model, args.confidence)
    if args.annotated:
        annotate(image, detection, args.annotated)
    report: dict[str, Any] = {
        "mode": "detect_only",
        "motion_command_emitted": False,
        "image_size": [int(image.shape[1]), int(image.shape[0])],
        "detection": None if detection is None else asdict(detection),
    }
    exit_code = 0
    if detection is None:
        report["blocking_findings"] = ["cup 클래스가 confidence 기준을 통과하지 못함"]
        exit_code = 2
    elif args.plan:
        try:
            if args.measured_forward_m is None:
                report = solve_pick_plan(detection, load_workspace(args.workspace))
            else:
                left_mount_xy = np.array([0.020, 0.170])
                base_xy = left_mount_xy + np.array(
                    [args.measured_forward_m, args.measured_lateral_m]
                )
                report = solve_pick_plan_at_xy(
                    detection,
                    base_xy,
                    args.table_surface_z_m,
                    args.grasp_height_above_table_m,
                    args.pregrasp_clearance_m,
                    args.approach_pitch_deg,
                )
                report["coordinate_source"] = {
                    "type": "single_measured_left_arm_relative",
                    "forward_m": args.measured_forward_m,
                    "lateral_m": args.measured_lateral_m,
                    "repeatable_pixel_mapping_available": False,
                }
            if not report["ready_for_collision_review"]:
                exit_code = 2
        except RuntimeError as exc:
            report["mode"] = "plan_blocked"
            report["blocking_findings"] = [str(exc)]
            exit_code = 2
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
