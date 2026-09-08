#!/usr/bin/env python3
"""실패 분석 레코드를 JSON Schema로 검증한다.

사용법:
    python3 tools/validate_failure_record.py <실패레코드.json 또는 디렉터리>

인자를 주지 않으면 data/schema/example_failure_record.json 을 검증한다.
스키마: data/schema/failure_record.schema.json
"""

from __future__ import annotations

import json
import re
import sys
from argparse import ArgumentParser
from datetime import datetime
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
SCHEMA_PATH = REPO_ROOT / "data" / "schema" / "failure_record.schema.json"
DEFAULT_TARGET = REPO_ROOT / "data" / "schema" / "example_failure_record.json"
DATE_TIME_PATTERN = re.compile(
    r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|[+-]\d{2}:\d{2})$"
)


def _reject_nonstandard_number(value: str) -> None:
    raise ValueError(f"JSON 비표준 숫자는 허용되지 않습니다: {value}")


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as file:
        document = json.load(file, parse_constant=_reject_nonstandard_number)
    if not isinstance(document, dict):
        raise ValueError("JSON 최상위 값은 객체여야 합니다")
    return document


def collect_targets(target: Path) -> list[Path]:
    if target.is_dir():
        flat_records = {path for path in target.glob("*.json") if path != SCHEMA_PATH}
        bundled_records = set(target.rglob("failure.json"))
        return sorted(flat_records | bundled_records)
    return [target]


def _ordered_time_range(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 2
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)
        and value[0] <= value[1]
    )


def _valid_date_time(value: Any) -> bool:
    if not isinstance(value, str) or DATE_TIME_PATTERN.fullmatch(value) is None:
        return False
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00")).tzinfo is not None
    except ValueError:
        return False


def _semantic_errors(path: Path, document: dict) -> list[str]:
    errors: list[str] = []
    if not _valid_date_time(document.get("recorded_at")):
        errors.append(f"{path.name}: recorded_at — 올바른 ISO 8601 date-time이 아님")
    evidence = document.get("evidence", [])
    evidence_ids = [item.get("id") for item in evidence if isinstance(item, dict)]
    known_evidence_ids = set(evidence_ids)
    duplicate_evidence_ids = sorted(
        item_id for item_id in known_evidence_ids if evidence_ids.count(item_id) > 1
    )
    for item_id in duplicate_evidence_ids:
        errors.append(f"{path.name}: evidence — 중복 evidence ID: {item_id}")

    observation_range = document.get("observation", {}).get("time_range_s")
    if observation_range is not None and not _ordered_time_range(observation_range):
        errors.append(f"{path.name}: observation/time_range_s — 시작 시각이 종료 시각보다 늦음")
    for index, item in enumerate(evidence):
        time_range = item.get("time_range_s") if isinstance(item, dict) else None
        if time_range is not None and not _ordered_time_range(time_range):
            errors.append(
                f"{path.name}: evidence/{index}/time_range_s — 시작 시각이 종료 시각보다 늦음"
            )

    observation_stage = document.get("observation", {}).get("stage")
    context = document.get("context", {})
    stage_phase = {
        "cup_grasp": "CUP_PICK",
        "jug_grasp": "JUG_PICK",
        "pour": "POUR",
        "jug_return": "JUG_RETURN",
        "shelf_place": "SHELF_PLACE",
        "table_place": "TABLE_PICK_PLACE",
    }
    expected_phase = stage_phase.get(observation_stage)
    if expected_phase:
        for field in ("control_strategy", "phase_id", "backend"):
            if not context.get(field):
                errors.append(
                    f"{path.name}: context/{field} — 조작 phase 실패 재현에 필요"
                )
        if context.get("phase_id") and context.get("phase_id") != expected_phase:
            errors.append(
                f"{path.name}: context/phase_id — observation stage와 phase가 모순됨"
            )
    strategy = context.get("control_strategy")
    backend = context.get("backend")
    required_backend = {
        "TELEOP": "TELEOP",
        "PLANNED_ALL": "PLANNED",
        "ACT_ALL": "ACT",
    }.get(strategy)
    if required_backend and backend and backend != required_backend:
        errors.append(
            f"{path.name}: context/backend — control_strategy와 backend가 모순됨"
        )
    backend_artifacts = context.get("backend_artifacts", {})
    if backend == "ACT" and not (
        context.get("checkpoint_id") or backend_artifacts.get("checkpoint")
    ):
        errors.append(f"{path.name}: context/checkpoint_id — ACT 실패 재현에 필요")
    if backend == "PLANNED" and not backend_artifacts:
        errors.append(f"{path.name}: context/backend_artifacts — PLANNED 실패 재현에 필요")

    diagnosis = document.get("diagnosis", {})
    hypotheses = diagnosis.get("hypotheses", [])
    hypothesis_ids = [item.get("id") for item in hypotheses if isinstance(item, dict)]
    for item_id in sorted(set(hypothesis_ids)):
        if hypothesis_ids.count(item_id) > 1:
            errors.append(f"{path.name}: diagnosis/hypotheses — 중복 hypothesis ID: {item_id}")

    def check_evidence_refs(location: str, references: Any) -> None:
        if not isinstance(references, list):
            return
        for evidence_id in references:
            if evidence_id not in known_evidence_ids:
                errors.append(
                    f"{path.name}: {location} — 존재하지 않는 evidence ID 참조: {evidence_id}"
                )

    for index, hypothesis in enumerate(hypotheses):
        if isinstance(hypothesis, dict):
            check_evidence_refs(
                f"diagnosis/hypotheses/{index}/evidence_ids", hypothesis.get("evidence_ids")
            )

    cause_status = diagnosis.get("cause_status")
    root_cause = diagnosis.get("root_cause")
    if root_cause is not None and cause_status != "confirmed":
        errors.append(
            f"{path.name}: diagnosis/root_cause — cause_status=confirmed일 때만 기록 가능"
        )
    if document.get("status") == "unresolved" and cause_status == "confirmed":
        errors.append(
            f"{path.name}: status — unresolved 상태에 confirmed 원인을 기록할 수 없음"
        )
    if isinstance(root_cause, dict):
        check_evidence_refs("diagnosis/root_cause/confirmed_by", root_cause.get("confirmed_by"))

    validation = document.get("validation", {})
    check_evidence_refs(
        "validation/acceptance_evidence_ids", validation.get("acceptance_evidence_ids")
    )
    if document.get("status") == "resolved":
        resolved_requirements = (
            (validation.get("protocol_id"), "protocol_id가 없음"),
            (validation.get("result") == "pass", "동일 조건 재시험 result가 pass가 아님"),
            (validation.get("same_condition_run_ids"), "same_condition_run_ids가 비어 있음"),
            (validation.get("regression_result") == "pass", "regression_result가 pass가 아님"),
            (validation.get("regression_run_ids"), "regression_run_ids가 비어 있음"),
            (validation.get("acceptance_evidence_ids"), "acceptance_evidence_ids가 비어 있음"),
        )
        for satisfied, message in resolved_requirements:
            if not satisfied:
                errors.append(f"{path.name}: validation — resolved 조건 미충족: {message}")

    data_use = document.get("data_use", {})
    disposition = data_use.get("primary_disposition")
    include_in_training = data_use.get("include_in_training") is True
    if disposition == "counterexample" and include_in_training:
        errors.append(
            f"{path.name}: data_use — counterexample 원본은 BC 학습에 자동 포함할 수 없음"
        )
    if disposition == "hil_recovery" and include_in_training:
        if cause_status != "confirmed":
            errors.append(f"{path.name}: data_use — 원인 미확정 실패의 HIL 학습 포함은 보류")
        if not isinstance(root_cause, dict) or root_cause.get("layer") != "policy_data":
            errors.append(f"{path.name}: data_use — HIL 학습 보강은 확인된 policy_data 원인에 한함")
        if document.get("response", {}).get("retraining_decision") not in ("collect_hil_recovery", "retrain"):
            errors.append(f"{path.name}: data_use — HIL 수집 또는 재학습 결정 근거가 필요")
        if data_use.get("recovery_verified_success") is not True:
            errors.append(f"{path.name}: data_use — recovery_verified_success로 복구 성공 확인 필요")
        if data_use.get("recovery_control_source") not in ("human", "teleop"):
            errors.append(f"{path.name}: data_use — 사람 제어 복구 출처가 필요")
        required = (
            (data_use.get("source_dataset_id"), "source_dataset_id"),
            (data_use.get("derived_dataset_id"), "derived_dataset_id"),
            (data_use.get("recovery_evidence_ids"), "recovery_evidence_ids"),
        )
        for value, field in required:
            if not value:
                errors.append(f"{path.name}: data_use/{field} — HIL 학습 포함에 필요")
        recovery_range = data_use.get("recovery_time_range_s")
        if not _ordered_time_range(recovery_range) or recovery_range[0] == recovery_range[1]:
            errors.append(
                f"{path.name}: data_use/recovery_time_range_s — 정확한 양의 길이 복구 구간이 필요"
            )
        check_evidence_refs(
            "data_use/recovery_evidence_ids", data_use.get("recovery_evidence_ids")
        )
        if _ordered_time_range(recovery_range):
            supporting_ranges = [
                item.get("time_range_s") for item in evidence
                if item["id"] in data_use.get("recovery_evidence_ids", [])
            ]
            if not any(
                _ordered_time_range(window)
                and window[0] <= recovery_range[0] < recovery_range[1] <= window[1]
                for window in supporting_ranges
            ):
                errors.append(f"{path.name}: data_use — 복구 구간을 포함하는 근거 시간 범위가 없음")
        source_dataset_id = data_use.get("source_dataset_id")
        derived_dataset_id = data_use.get("derived_dataset_id")
        if source_dataset_id and source_dataset_id == derived_dataset_id:
            errors.append(
                f"{path.name}: data_use/source_dataset_id — 원본과 파생 데이터셋을 분리해야 함"
            )
        if derived_dataset_id and data_use.get("training_dataset_id") != derived_dataset_id:
            errors.append(
                f"{path.name}: data_use/training_dataset_id — derived_dataset_id와 일치해야 함"
            )
    return errors


def validate_full(schema: dict, documents: list[tuple[Path, dict]]) -> list[str]:
    from jsonschema import Draft202012Validator, FormatChecker

    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    errors = []
    for path, document in documents:
        schema_errors = sorted(validator.iter_errors(document), key=lambda item: list(item.path))
        for error in schema_errors:
            location = "/".join(str(part) for part in error.path) or "(root)"
            errors.append(f"{path.name}: {location} — {error.message}")
        if not schema_errors and isinstance(document, dict):
            errors.extend(_semantic_errors(path, document))
    return errors


def validate_minimal(schema: dict, documents: list[tuple[Path, dict]]) -> list[str]:
    required = schema.get("required", [])
    errors = []
    for path, document in documents:
        for key in required:
            if key not in document:
                errors.append(f"{path.name}: 필수 필드 누락 — {key}")

        diagnosis = document.get("diagnosis", {})
        if not isinstance(diagnosis, dict):
            errors.append(f"{path.name}: diagnosis는 객체여야 함")
            diagnosis = {}
        if diagnosis.get("cause_status") == "confirmed" and not diagnosis.get("root_cause"):
            errors.append(f"{path.name}: 원인 확정 상태인데 root_cause가 없음")

        validation = document.get("validation", {})
        if not isinstance(validation, dict):
            errors.append(f"{path.name}: validation은 객체여야 함")
            validation = {}
        if validation.get("result") not in (None, "not_run") and not validation.get(
            "same_condition_run_ids"
        ):
            errors.append(f"{path.name}: 재시험 결과가 있는데 same_condition_run_ids가 없음")

        data_use = document.get("data_use", {})
        if not isinstance(data_use, dict):
            errors.append(f"{path.name}: data_use는 객체여야 함")
            data_use = {}
        if data_use.get("include_in_training") is True and not data_use.get("training_dataset_id"):
            errors.append(f"{path.name}: 학습 포함인데 training_dataset_id가 없음")
    return errors


def main() -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("target", nargs="?", type=Path, default=DEFAULT_TARGET)
    args = parser.parse_args()
    target = args.target.resolve()
    if not target.exists():
        print(f"대상이 없습니다: {target}", file=sys.stderr)
        return 2

    try:
        schema = load_json(SCHEMA_PATH)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        print(f"스키마 JSON을 읽을 수 없습니다: {SCHEMA_PATH} — {error}", file=sys.stderr)
        return 2
    paths = collect_targets(target)
    if not paths:
        print(f"검증할 json 이 없습니다: {target}", file=sys.stderr)
        return 2

    documents = []
    for path in paths:
        try:
            documents.append((path, load_json(path)))
        except (OSError, json.JSONDecodeError, ValueError) as error:
            print(f"JSON을 읽을 수 없습니다: {path} — {error}", file=sys.stderr)
            return 2

    try:
        errors = validate_full(schema, documents)
        mode = "full"
    except ImportError:
        errors = validate_minimal(schema, documents)
        mode = "minimal"

    for line in errors:
        print(f"FAIL  {line}")

    if mode == "minimal":
        print("경고: jsonschema 미설치 — 핵심 필드만 확인했습니다 (pip install jsonschema)")

    if errors:
        print(f"\n{len(paths)}개 중 {len(errors)}건 실패")
        return 1

    print(f"OK  {len(paths)}개 검증 통과 ({mode} 모드)")
    return 0 if mode == "full" else 3


if __name__ == "__main__":
    sys.exit(main())
