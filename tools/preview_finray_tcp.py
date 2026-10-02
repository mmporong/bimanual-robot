#!/usr/bin/env python3
"""FinRay 손가락의 TCP 후보를 활성 모델과 분리해 오프라인 비교한다.

선택 STL은 해시와 외곽 치수만 읽는다. 라이선스가 확인되지 않은 메시를 저장소나
출력 URDF에 포함하지 않는다. 출력 URDF는 활성 모델의 복사본에 질량·관성·충돌·
시각 형상이 없는 후보 고정 프레임만 추가한다. 후보 오프셋은 모두 ``left_tool0``
로컬 좌표계의 가정값이며 실측 보정, 실행 승인 또는 실물 정확도를 뜻하지 않는다.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import numbers
import re
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SELECTION = REPO_ROOT / "design/gripper/finray_80mm_lip3_selection.yaml"
DEFAULT_URDF = REPO_ROOT / "src/hold_flow_description/urdf/hold_flow.urdf"
SOLVER_PATH = REPO_ROOT / "src/hold_flow_description/scripts/solve_task_poses.py"


@dataclass(frozen=True)
class Candidate:
    name: str
    offset_mm: tuple[float, float, float]


def _solver_module():
    spec = importlib.util.spec_from_file_location("hold_flow_solve_task_poses", SOLVER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"FK 모듈을 불러올 수 없습니다: {SOLVER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_selection(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("selected_finger"), dict):
        raise ValueError("FinRay 선택 YAML에 selected_finger 객체가 없습니다")
    selected = data["selected_finger"]
    required = {"source_filename", "source_sha256", "measured_stl_envelope_mm"}
    missing = sorted(required - selected.keys())
    if missing:
        raise ValueError(f"FinRay 선택 YAML 필드가 없습니다: {missing}")
    return data


def discover_stl(downloads: Path, filename: str, expected_sha256: str) -> Path | None:
    if not downloads.exists():
        return None
    matches = sorted(path for path in downloads.rglob(filename) if path.is_file())
    if not matches:
        return None
    matching_hash = [
        path for path in matches
        if hashlib.sha256(path.read_bytes()).hexdigest() == expected_sha256
    ]
    if len(matching_hash) == 1:
        return matching_hash[0]
    if len(matching_hash) > 1:
        return matching_hash[0]
    if len(matches) == 1:
        return matches[0]
    raise ValueError(f"이름이 같은 STL이 여러 개지만 선택 해시와 일치하지 않습니다: {matches}")


def _stl_vertices(data: bytes) -> np.ndarray:
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + count * 50 == len(data):
            vertices = np.empty((count * 3, 3), dtype=float)
            for index in range(count):
                values = struct.unpack_from("<12f", data, 84 + index * 50)
                vertices[index * 3:(index + 1) * 3] = np.asarray(values[3:]).reshape(3, 3)
            return vertices

    text = data.decode("utf-8", errors="ignore")
    values = re.findall(
        r"\bvertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", text
    )
    if not values:
        raise ValueError("지원되는 binary 또는 ASCII STL 꼭짓점을 찾지 못했습니다")
    return np.asarray([[float(value) for value in row] for row in values], dtype=float)


def inspect_stl(path: Path, expected_sha256: str, expected_envelope_mm: Sequence[float]) -> dict[str, Any]:
    data = path.read_bytes()
    vertices = _stl_vertices(data)
    if not np.isfinite(vertices).all():
        raise ValueError("STL에 유한하지 않은 꼭짓점 좌표가 있습니다")
    envelope = vertices.max(axis=0) - vertices.min(axis=0)
    expected = np.asarray(expected_envelope_mm, dtype=float)
    actual_hash = hashlib.sha256(data).hexdigest()
    return {
        "path": str(path),
        "sha256": actual_hash,
        "expected_sha256": expected_sha256,
        "sha256_matches_selection": actual_hash == expected_sha256,
        "envelope_mm": [round(float(value), 5) for value in envelope],
        "expected_envelope_mm": [float(value) for value in expected],
        "envelope_matches_selection": bool(np.allclose(envelope, expected, atol=0.02, rtol=0.0)),
        "inspection_only": True,
    }


def _finite_real(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(f"{label}은 bool이나 문자열이 아닌 실수여야 합니다")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label}은 유한한 값이어야 합니다")
    return result


def normalize_candidates(offsets_mm: Sequence[Sequence[float]]) -> list[Candidate]:
    values = list(offsets_mm) or [(0.0, 0.0, 0.0)]
    candidates = []
    for index, row in enumerate(values, start=1):
        if isinstance(row, (str, bytes)) or not isinstance(row, Sequence) or len(row) != 3:
            raise ValueError("TCP 후보 오프셋은 x y z 세 값이어야 합니다")
        offset = tuple(_finite_real(value, "TCP 후보 오프셋") for value in row)
        candidates.append(Candidate(f"left_finray_tcp_candidate_{index:03d}", offset))
    return candidates


def validate_candidates(candidates: Sequence[Candidate]) -> list[Candidate]:
    checked = []
    names = set()
    for candidate in candidates:
        if not isinstance(candidate, Candidate):
            raise ValueError("TCP 후보는 Candidate 객체여야 합니다")
        if not re.fullmatch(r"left_finray_tcp_candidate_[0-9]{3}", candidate.name):
            raise ValueError(f"TCP 후보 프레임 이름이 올바르지 않습니다: {candidate.name!r}")
        if candidate.name in names:
            raise ValueError(f"TCP 후보 프레임 이름이 중복됩니다: {candidate.name}")
        if (
            isinstance(candidate.offset_mm, (str, bytes))
            or not isinstance(candidate.offset_mm, Sequence)
            or len(candidate.offset_mm) != 3
        ):
            raise ValueError("TCP 후보 오프셋은 x y z 세 값이어야 합니다")
        offset = tuple(_finite_real(value, "TCP 후보 오프셋") for value in candidate.offset_mm)
        checked.append(Candidate(candidate.name, offset))
        names.add(candidate.name)
    if not checked:
        raise ValueError("TCP 후보가 하나 이상 필요합니다")
    return checked


def normalize_joint_deg(joint_deg: Sequence[float]) -> list[float]:
    if isinstance(joint_deg, (str, bytes)) or not isinstance(joint_deg, Sequence) or len(joint_deg) != 5:
        raise ValueError("--joint-deg에는 관절각 다섯 개가 필요합니다")
    return [_finite_real(value, "관절각") for value in joint_deg]


def validate_output_paths(
    selection: Path,
    input_urdf: Path,
    stl: Path | None,
    output_urdf: Path,
    output_json: Path | None,
) -> None:
    protected = {selection.resolve(), input_urdf.resolve(), DEFAULT_URDF.resolve()}
    if stl is not None:
        protected.add(stl.resolve())
    outputs = [output_urdf] + ([output_json] if output_json is not None else [])
    resolved_outputs = [path.resolve() for path in outputs]
    if len(set(resolved_outputs)) != len(resolved_outputs):
        raise ValueError("URDF와 JSON 출력 경로는 서로 달라야 합니다")
    for path, resolved in zip(outputs, resolved_outputs):
        if resolved in protected:
            raise ValueError(f"입력 또는 활성 모델 경로를 출력으로 사용할 수 없습니다: {path}")
        if path.exists():
            raise FileExistsError(f"기존 증거 파일을 덮어쓰지 않습니다: {path}")


def write_preview_urdf(source: Path, output: Path, candidates: Sequence[Candidate]) -> None:
    candidates = validate_candidates(candidates)
    validate_output_paths(DEFAULT_SELECTION, source, None, output, None)
    tree = ET.parse(source)
    root = tree.getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    if "left_tool0" not in links or "left_cup_tcp" not in links:
        raise ValueError("활성 URDF에 left_tool0 또는 left_cup_tcp가 없습니다")
    root.append(ET.Comment(
        " Offline assumed TCP markers only: no visual, collision, inertial, calibration, or motion approval. "
    ))
    for candidate in candidates:
        if candidate.name in links:
            raise ValueError(f"preview 프레임 이름이 이미 있습니다: {candidate.name}")
        ET.SubElement(root, "link", name=candidate.name)
        joint = ET.SubElement(root, "joint", name=f"{candidate.name}_joint", type="fixed")
        ET.SubElement(joint, "parent", link="left_tool0")
        ET.SubElement(joint, "child", link=candidate.name)
        xyz_m = [value / 1000.0 for value in candidate.offset_mm]
        ET.SubElement(joint, "origin", xyz=" ".join(f"{value:.9g}" for value in xyz_m), rpy="0 0 0")
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    with output.open("xb") as stream:
        stream.write(payload)


def compare_fk(urdf: Path, candidates: Sequence[Candidate], joint_deg: Sequence[float]) -> dict[str, Any]:
    candidates = validate_candidates(candidates)
    checked_joint_deg = normalize_joint_deg(joint_deg)
    solver = _solver_module()
    chain = solver.Chain(urdf)
    q = np.radians(np.asarray(checked_joint_deg, dtype=float))
    transforms = chain.transforms(solver.base_positions("left", q))
    active = transforms["left_cup_tcp"][:3, 3]
    rows = []
    for candidate in candidates:
        position = transforms[candidate.name][:3, 3]
        rows.append({
            "frame": candidate.name,
            "assumed_tool0_offset_mm": list(candidate.offset_mm),
            "base_position_m": [round(float(value), 9) for value in position],
            "delta_from_active_left_cup_tcp_mm": [
                round(float(value), 6) for value in (position - active) * 1000.0
            ],
        })
    return {
        "joint_deg": checked_joint_deg,
        "active_left_cup_tcp_base_position_m": [round(float(value), 9) for value in active],
        "candidates": rows,
    }


def build_result(
    selection: dict[str, Any],
    stl_report: dict[str, Any] | None,
    preview_urdf: Path,
    fk_report: dict[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": "1.0",
        "mode": "offline_finray_tcp_assumption_preview",
        "motion_command_emitted": False,
        "active_urdf_modified": False,
        "preview_urdf": str(preview_urdf),
        "selection_status": selection.get("status"),
        "license_status": selection.get("selected_finger", {}).get("license_status"),
        "selected_stl_inspection": stl_report or {"status": "not_found_under_downloads"},
        "fk_comparison": fk_report,
        "missing_measurements_before_active_model_update": [
            "left_tool0에서 FinRay 기준점까지의 장착 변환 xyz/rpy",
            "컵 접촉 상태에서 사용할 TCP의 실측 위치와 정의",
            "출력한 손가락 한 쌍의 질량과 무게중심",
            "열림·닫힘 전 구간의 충돌 외곽과 유효 접촉 폭",
        ],
        "semantics": [
            "후보 오프셋은 left_tool0 로컬 좌표계의 가정값이다",
            "STL은 해시와 외곽 치수 확인에만 사용하며 preview URDF에 포함하지 않는다",
            "복사된 기존 모델 요소는 유지되며 새 preview 프레임에만 visual, collision, inertial 요소가 없다",
            "결과는 실측 보정, 실행 승인 또는 실물 정확도 검증이 아니다",
        ],
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", type=Path, default=DEFAULT_SELECTION)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--stl", type=Path, help="선택 STL 경로. 생략하면 ~/Downloads 아래에서 찾습니다")
    parser.add_argument(
        "--offset-mm", action="append", nargs=3, type=float, metavar=("X", "Y", "Z"),
        help="left_tool0 로컬 TCP 후보. 여러 번 지정할 수 있으며 기본값은 0 0 0입니다",
    )
    parser.add_argument(
        "--joint-deg", nargs=5, type=float, default=[0.0] * 5,
        metavar=("J1", "J2", "J3", "J4", "J5"),
    )
    parser.add_argument("--output-urdf", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    selection = load_selection(args.selection)
    selected = selection["selected_finger"]
    stl_path = args.stl
    if stl_path is None:
        stl_path = discover_stl(
            Path.home() / "Downloads", selected["source_filename"], selected["source_sha256"]
        )
    elif not stl_path.is_file():
        raise FileNotFoundError(f"STL을 찾지 못했습니다: {stl_path}")

    stl_report = None
    if stl_path is not None:
        stl_report = inspect_stl(
            stl_path, selected["source_sha256"], selected["measured_stl_envelope_mm"]
        )
        if not stl_report["sha256_matches_selection"] or not stl_report["envelope_matches_selection"]:
            raise ValueError("STL 해시 또는 외곽 치수가 선택 YAML과 다릅니다")

    candidates = normalize_candidates(args.offset_mm or [])
    joint_deg = normalize_joint_deg(args.joint_deg)
    validate_output_paths(
        args.selection, args.urdf, stl_path, args.output_urdf, args.output_json
    )
    write_preview_urdf(args.urdf, args.output_urdf, candidates)
    result = build_result(
        selection,
        stl_report,
        args.output_urdf,
        compare_fk(args.output_urdf, candidates, joint_deg),
    )
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        with args.output_json.open("x", encoding="utf-8") as stream:
            stream.write(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
