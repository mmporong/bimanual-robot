#!/usr/bin/env python3
"""저장된 관절 JSON·사진만 보존하고 현행 URDF FK로 다시 비교한다.

카메라·서보·ROS에 접속하지 않는다. 계산된 TCP는 모델 좌표이며 영상 측정이 아니다.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import shutil
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src/hold_flow_description/scripts"))
from solve_task_poses import ARM_JOINTS, Chain, URDF_PATH, base_positions  # noqa: E402


def vector(value: object, size: int, label: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{label}: {size}개 숫자가 필요합니다")
    if any(type(item) not in (int, float) or not math.isfinite(item) for item in value):
        raise ValueError(f"{label}: 유한 숫자만 허용합니다")
    return np.asarray(value, dtype=float)


def compare_state(state: dict, plan: dict, stage: str, chain: Chain) -> dict:
    if state.get("side") != "left" or state.get("joint_order") != ARM_JOINTS:
        raise ValueError("왼팔 5축의 명시된 관절 순서가 필요합니다")
    q_deg = vector(state.get("joint_degrees"), 5, "저장 자세")
    goal = plan["stages"][stage]
    target_deg = vector(goal.get("joint_deg"), 5, "계획 자세")
    target_m = vector(goal.get("target_base_footprint_m"), 3, "계획 TCP")

    def fk(deg: np.ndarray) -> np.ndarray:
        return chain.transforms(base_positions("left", np.radians(deg)))["left_cup_tcp"][:3, 3]

    actual_fk = fk(q_deg)
    planned_fk = fk(target_deg)
    return {
        "captured_at": state.get("captured_at"),
        "joint_degrees": q_deg.tolist(),
        "joint_delta_from_plan_deg": (q_deg - target_deg).tolist(),
        "max_abs_joint_delta_deg": float(np.max(np.abs(q_deg - target_deg))),
        "saved_state_model_tcp_m": actual_fk.tolist(),
        "planned_model_tcp_m": planned_fk.tolist(),
        "target_base_footprint_m": target_m.tolist(),
        "saved_vs_planned_model_tcp_distance_mm": float(np.linalg.norm(actual_fk - planned_fk) * 1000),
        "saved_model_tcp_vs_target_mm": float(np.linalg.norm(actual_fk - target_m) * 1000),
        "planned_solver_residual_mm": float(np.linalg.norm(planned_fk - target_m) * 1000),
        "real_cup_error_mm": None,
        "grasp_success": None,
    }


def archive_file(source: Path, destination: Path) -> dict:
    source = source.resolve(strict=True)
    if not source.is_file():
        raise ValueError(f"일반 파일만 허용합니다: {source}")
    shutil.copy2(source, destination)
    return {
        "source": str(source),
        "archived": destination.name,
        "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
        "source_mtime_ns": source.stat().st_mtime_ns,
        "mtime_is_capture_time": False,
    }


def write_report(plan_path: Path, state_paths: list[Path], image_paths: list[Path],
                 urdf_path: Path, stage: str, output: Path) -> dict:
    # 기존 증거를 덮어쓰지 않는다. 출력 디렉터리는 실행마다 새로 지정한다.
    if not state_paths:
        raise ValueError("비교할 저장 관절 상태가 필요합니다")
    for path in [plan_path, urdf_path, *state_paths, *image_paths]:
        if not path.is_file():
            raise ValueError(f"입력 파일 없음: {path}")
    for path in image_paths:
        if path.suffix.lower() not in {".jpg", ".jpeg", ".png"}:
            raise ValueError("저장 이미지는 JPG 또는 PNG만 허용합니다")
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    chain = Chain(urdf_path)
    rows = [compare_state(json.loads(p.read_text(encoding="utf-8")), plan, stage, chain)
            for p in state_paths]
    output.mkdir(parents=True, exist_ok=False)
    manifest = [archive_file(plan_path, output / "plan.json"),
                archive_file(urdf_path, output / "fk_model.urdf")]
    for index, (path, row) in enumerate(zip(state_paths, rows)):
        item = archive_file(path, output / f"state_{index:02d}.json")
        manifest.append(item)
        row["source"] = item["source"]
    photos = []
    for index, path in enumerate(image_paths):
        item = archive_file(path, output / f"image_{index:02d}{path.suffix.lower()}")
        manifest.append(item)
        photos.append(item)
    result = {
        "schema_version": "1.0", "mode": "offline_saved_state_fk_comparison",
        "hardware_accessed": False, "motion_command_emitted": False,
        "stage": stage, "states": rows, "manifest": manifest,
        "limitations": [
            "현행 stock jaw FK이며 순정 죠 실물 TCP·ID 6 재캘리브레이션 측정이 아니다",
            "사진과 관절값의 시간 동기화는 미확인이다",
            "카메라 외부 보정값이 없어 사진에 TCP를 투영하지 않는다",
            "이미지 손상·가림은 수동 검토하며 파지 성공을 자동 판정하지 않는다",
            "첨부 URDF는 FK 추적용이며 상대 메시 자산까지 묶은 시뮬레이터 패키지가 아니다",
        ],
    }
    (output / "report.json").write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                                  allow_nan=False) + "\n", encoding="utf-8")
    table_rows = "".join(
        f"<tr><td>{html.escape(Path(row['source']).name)}</td>"
        f"<td>{row['max_abs_joint_delta_deg']:.2f}</td>"
        f"<td>{row['saved_vs_planned_model_tcp_distance_mm']:.2f}</td></tr>" for row in rows)
    figures = "".join(f"<figure><img src='{p['archived']}' alt='저장 사진'>"
                      f"<figcaption>{html.escape(Path(p['source']).name)}"
                      " · 시간 동기화 미확인</figcaption></figure>" for p in photos)
    limitations = "".join(f"<li>{html.escape(s)}</li>" for s in result["limitations"])
    page = ("<!doctype html><html lang='ko'><meta charset='utf-8'>"
            "<title>컵 접근 오프라인 재생</title><style>body{font:16px sans-serif;max-width:1100px;"
            "margin:32px auto;padding:16px}table{border-collapse:collapse}td,th{padding:10px;"
            "border:1px solid #aaa}.photos{display:flex;flex-wrap:wrap}figure{margin:12px}"
            "img{max-width:480px;width:100%}figcaption{max-width:480px}</style>"
            "<h1>컵 접근 오프라인 재생</h1><p>실물 접속 없음. 아래 거리는 현행 모델상의 차이이며 "
            "실물 컵 위치 오차가 아닙니다.</p><table><tr><th>저장 자세</th>"
            "<th>계획 대비 최대 관절 차이 (deg)</th><th>계획 대비 모델 TCP 차이 (mm)</th></tr>"
            + table_rows + "</table><ul>" + limitations + "</ul><div class='photos'>"
            + figures + "</div></html>")
    (output / "index.html").write_text(page, encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--state", type=Path, action="append", required=True)
    parser.add_argument("--image", type=Path, action="append", default=[])
    parser.add_argument("--urdf", type=Path, default=URDF_PATH)
    parser.add_argument("--stage", choices=["pregrasp", "grasp"], default="pregrasp")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    write_report(args.plan, args.state, args.image, args.urdf, args.stage, args.output_dir)
    print(args.output_dir / "index.html")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
