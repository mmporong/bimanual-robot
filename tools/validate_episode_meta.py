#!/usr/bin/env python3
"""에피소드 수집 조건 메타데이터를 스키마로 검증한다.

사용법:
    python3 tools/validate_episode_meta.py <메타데이터.json 또는 디렉터리>

인자를 주지 않으면 data/schema/example_episode_meta.json 을 검증한다.
스키마: data/schema/episode_metadata.schema.json

jsonschema 패키지가 없으면 필수 필드 존재 여부만 확인하는 축소 모드로 돈다.
축소 모드는 통과해도 스키마 준수를 보장하지 않으므로 종료 코드와 함께 경고를 낸다.
"""

from __future__ import annotations

import json
import math
import re
import sys
from argparse import ArgumentParser
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SCHEMA_PATH = REPO_ROOT / "data" / "schema" / "episode_metadata.schema.json"
DEFAULT_TARGET = REPO_ROOT / "data" / "schema" / "example_episode_meta.json"
DATE_TIME_PATTERN = re.compile(
    r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|[+-]\d{2}:\d{2})$"
)


def _reject_nonstandard_number(value: str) -> None:
    raise ValueError(f"JSON 비표준 숫자는 허용되지 않습니다: {value}")


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        document = json.load(f, parse_constant=_reject_nonstandard_number)
    if not isinstance(document, dict):
        raise ValueError("JSON 최상위 값은 객체여야 합니다")
    return document


def collect_targets(target: Path) -> list[Path]:
    if target.is_dir():
        return sorted(p for p in target.rglob("*.json") if p != SCHEMA_PATH)
    return [target]


def _ordered_time_range(value: object) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 2
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)
        and value[0] < value[1]
    )


def _valid_date_time(value: object) -> bool:
    if not isinstance(value, str) or DATE_TIME_PATTERN.fullmatch(value) is None:
        return False
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00")).tzinfo is not None
    except ValueError:
        return False


def _semantic_errors(path: Path, doc: dict) -> list[str]:
    errors = []
    if not _valid_date_time(doc.get("recorded_at")):
        errors.append(f"{path.name}: recorded_at — 올바른 ISO 8601 date-time이 아님")
    outcome = doc.get("outcome", {})
    include = doc.get("include") is True
    if doc.get("include") is False and not doc.get("exclude_reason", "").strip():
        errors.append(f"{path.name}: exclude_reason — 제외 사유가 비어 있음")
    if outcome.get("success") is False and include:
        errors.append(f"{path.name}: include — 실패 원본(success=false)은 포함할 수 없음")
    if include:
        if outcome.get("verified_by") == "log_string":
            errors.append(f"{path.name}: include — E8 로그 문자열만으로 포함 승인할 수 없음")
        excluded_anomalies = {"servo_stall", "teleop_desync", "operator_error", "collision"}
        invalid = excluded_anomalies.intersection(outcome.get("anomalies", []))
        if invalid:
            errors.append(f"{path.name}: include — 원본 제외 대상 이상 현상: {', '.join(sorted(invalid))}")
    if outcome.get("success") is True and any(
        result is False for result in outcome.get("stage_results", {}).values()
    ):
        errors.append(f"{path.name}: outcome — 실패한 최종 단계 결과와 전체 성공이 모순됨")

    objects = doc.get("objects", [])
    object_ids = [item["id"] for item in objects]
    if len(object_ids) != len(set(object_ids)):
        errors.append(f"{path.name}: objects — 객체 ID가 중복됨")
    required_roles = {
        "water_kitchen_policy1": {"cup", "jug", "shelf"},
        "water_table_policy2": {"cup", "shelf", "table"},
    }.get(doc.get("task", {}).get("name"), set())
    missing_roles = required_roles - {item["role"] for item in objects}
    if missing_roles:
        errors.append(f"{path.name}: objects — 정책 필수 역할 누락: {', '.join(sorted(missing_roles))}")
    water_measurement = doc.get("water_measurement")
    if isinstance(water_measurement, dict) and sum(
        item["id"] == water_measurement["cup_id"] and item["role"] == "cup"
        for item in objects
    ) != 1:
        errors.append(f"{path.name}: water_measurement/cup_id — 측정 대상 cup 객체를 정확히 참조해야 함")
    if (doc.get("task", {}).get("name") == "water_kitchen_policy1"
            and outcome.get("success") is True
            and isinstance(water_measurement, dict)
            and water_measurement.get("fill_class") not in ("TARGET", "DRY_RUN")):
        errors.append(f"{path.name}: water_measurement — 주방 정책 성공에는 TARGET 또는 DRY_RUN 필요")
    if isinstance(water_measurement, dict) and water_measurement.get("fill_class") == "TARGET":
        for field in (
            "tolerance_ml",
            "estimated_ml",
            "ground_truth_ml",
            "ground_truth_method",
            "spill_detected",
        ):
            if field not in water_measurement:
                errors.append(
                    f"{path.name}: water_measurement/{field} — TARGET 판정 근거에 필요"
                )
        if water_measurement.get("ground_truth_method") == "manual_unknown":
            errors.append(
                f"{path.name}: water_measurement/ground_truth_method — "
                "TARGET 판정에는 실측 방법이 필요"
            )
        target = water_measurement.get("target_ml")
        tolerance = water_measurement.get("tolerance_ml")
        estimate = water_measurement.get("estimated_ml")
        measured = water_measurement.get("ground_truth_ml")
        if all(isinstance(value, (int, float)) for value in (target, tolerance, estimate)):
            if abs(estimate - target) > tolerance:
                errors.append(f"{path.name}: water_measurement — TARGET과 추정량 범위가 모순됨")
        if outcome.get("success") is True:
            if water_measurement.get("spill_detected") is True:
                errors.append(f"{path.name}: water_measurement — 흘림이 있는데 전체 성공으로 기록됨")
            if all(isinstance(value, (int, float)) for value in (target, tolerance, measured)):
                if abs(measured - target) > tolerance:
                    errors.append(f"{path.name}: water_measurement — 실측량이 목표 허용 범위를 벗어남")

    policy = doc.get("policy", {})
    phase_executions = policy.get("phase_executions", [])
    phase_ids = [item["phase_id"] for item in phase_executions]
    expected_phases = {
        "water_kitchen_policy1": [
            "CUP_PICK",
            "JUG_PICK",
            "MOVE_TO_PREPOUR",
            "POUR",
            "JUG_RETURN",
            "SHELF_PLACE",
        ],
        "water_table_policy2": ["TABLE_PICK_PLACE"],
    }.get(doc.get("task", {}).get("name"), [])
    water_measurement_required = (
        doc.get("task", {}).get("name") == "water_kitchen_policy1"
        and (
            policy.get("episode_scope") == "full_skill"
            or "POUR" in phase_ids
        )
    )
    if water_measurement_required and not isinstance(water_measurement, dict):
        errors.append(
            f"{path.name}: water_measurement — 주방 전체 실행 또는 POUR phase에 필요"
        )
    if policy.get("episode_scope") == "full_skill" and phase_ids != expected_phases:
        errors.append(
            f"{path.name}: policy/phase_executions — full_skill phase 순서가 태스크 계약과 다름"
        )
    if policy.get("episode_scope") == "phase_slice" and len(phase_executions) != 1:
        errors.append(
            f"{path.name}: policy/phase_executions — phase_slice는 phase 하나만 포함해야 함"
        )

    duration_s = outcome.get("duration_s")
    previous_end = 0.0
    for index, phase in enumerate(phase_executions):
        time_range = [phase.get("start_s"), phase.get("end_s")]
        if not _ordered_time_range(time_range):
            errors.append(
                f"{path.name}: policy/phase_executions/{index} — "
                "phase 시작·종료 시각이 올바르지 않음"
            )
            continue
        if time_range[0] < previous_end:
            errors.append(
                f"{path.name}: policy/phase_executions — phase 시간 순서 역전 또는 겹침"
            )
        previous_end = time_range[1]
        if isinstance(duration_s, (int, float)) and time_range[1] > duration_s:
            errors.append(
                f"{path.name}: policy/phase_executions/{index}/end_s — "
                "episode duration_s를 벗어남"
            )

    strategy = policy.get("control_strategy")
    backends = [item["backend"] for item in phase_executions]
    strategy_backend = {
        "TELEOP": "TELEOP",
        "PLANNED_ALL": "PLANNED",
        "ACT_ALL": "ACT",
    }
    required_backend = strategy_backend.get(strategy)
    if required_backend and any(backend != required_backend for backend in backends):
        errors.append(
            f"{path.name}: policy/control_strategy — {strategy}와 phase backend가 모순됨"
        )
    if strategy == "HYBRID" and set(backends) != {"PLANNED", "ACT"}:
        errors.append(
            f"{path.name}: policy/control_strategy — HYBRID에는 PLANNED와 ACT phase가 모두 필요"
        )
    for index, phase in enumerate(phase_executions):
        if phase["backend"] == "ACT" and not (
            phase.get("checkpoint_id") or policy.get("checkpoint_id")
        ):
            errors.append(
                f"{path.name}: policy/phase_executions/{index}/checkpoint_id — "
                "ACT phase 재현에 필요"
            )
        if phase["backend"] == "PLANNED" and not phase.get("artifact_ids"):
            errors.append(
                f"{path.name}: policy/phase_executions/{index}/artifact_ids — "
                "PLANNED phase 재현에 필요"
            )
    if policy.get("execution_mode") == "teleop_demo" and strategy != "TELEOP":
        errors.append(
            f"{path.name}: policy/execution_mode — teleop_demo는 TELEOP 전략이어야 함"
        )
    if policy.get("execution_mode") == "planned_baseline" and strategy != "PLANNED_ALL":
        errors.append(
            f"{path.name}: policy/execution_mode — planned_baseline은 PLANNED_ALL 전략이어야 함"
        )
    if policy.get("execution_mode") == "autonomous_rollout" and strategy == "TELEOP":
        errors.append(
            f"{path.name}: policy/execution_mode — autonomous_rollout은 TELEOP일 수 없음"
        )

    interventions = outcome.get("interventions", [])
    intervention_count = outcome.get("human_intervention_count", 0)
    if isinstance(intervention_count, int) and not isinstance(intervention_count, bool):
        if intervention_count != len(interventions):
            errors.append(
                f"{path.name}: outcome/interventions — human_intervention_count와 개수가 다름"
            )
    previous_end = 0.0
    intervention_duration = 0.0
    for index, intervention in enumerate(interventions):
        if not isinstance(intervention, dict):
            continue
        time_range = [intervention.get("start_s"), intervention.get("end_s")]
        if not _ordered_time_range(time_range):
            errors.append(
                f"{path.name}: outcome/interventions/{index} — 개입 시작·종료 시각이 올바르지 않음"
            )
        elif isinstance(duration_s, (int, float)) and time_range[1] > duration_s:
            errors.append(
                f"{path.name}: outcome/interventions/{index}/end_s — episode duration_s를 벗어남"
            )
        if _ordered_time_range(time_range):
            if time_range[0] < previous_end:
                errors.append(f"{path.name}: outcome/interventions — 개입 구간 순서 역전 또는 겹침")
            previous_end = time_range[1]
            intervention_duration += time_range[1] - time_range[0]
    declared_duration = outcome.get("human_intervention_duration_s")
    if isinstance(declared_duration, (int, float)) and not math.isclose(
        declared_duration, intervention_duration, rel_tol=1e-9, abs_tol=1e-6
    ):
        errors.append(f"{path.name}: outcome/human_intervention_duration_s — 개입 구간 합과 다름")

    provenance = doc.get("training_provenance", {})
    if provenance and provenance.get("source_split") != doc.get("split"):
        errors.append(f"{path.name}: training_provenance/source_split — 원본과 파생물 split이 다름")
    if (
        policy.get("execution_mode") == "human_in_the_loop"
        and doc.get("split") == "train"
        and include
    ):
        provenance = doc.get("training_provenance", {})
        required = (
            (provenance.get("source_dataset_id"), "source_dataset_id"),
            (provenance.get("derived_dataset_id"), "derived_dataset_id"),
            (provenance.get("source_episode_index") is not None, "source_episode_index"),
            (provenance.get("evidence_uri"), "evidence_uri"),
            (provenance.get("control_source"), "control_source"),
            (provenance.get("source_duration_s"), "source_duration_s"),
            (provenance.get("source_intervention_time_range_s"), "source_intervention_time_range_s"),
        )
        for value, field in required:
            if not value:
                errors.append(
                    f"{path.name}: training_provenance/{field} — HIL 학습 포함에 필요"
                )
        recovery_range = provenance.get("recovery_time_range_s")
        if not _ordered_time_range(recovery_range):
            errors.append(
                f"{path.name}: training_provenance/recovery_time_range_s — "
                "정확한 양의 길이 복구 구간이 필요"
            )
        source_range = provenance.get("source_intervention_time_range_s")
        source_duration = provenance.get("source_duration_s")
        if not _ordered_time_range(source_range):
            errors.append(f"{path.name}: training_provenance — 원본 개입 구간이 필요")
        elif isinstance(source_duration, (int, float)) and source_range[1] > source_duration:
            errors.append(f"{path.name}: training_provenance — 개입 구간이 원본 실행 길이를 벗어남")
        elif _ordered_time_range(recovery_range) and not (
            source_range[0] <= recovery_range[0] < recovery_range[1] <= source_range[1]
        ):
            errors.append(f"{path.name}: training_provenance — 복구 구간이 원본 개입 범위를 벗어남")
        if not interventions or not isinstance(duration_s, (int, float)) or duration_s <= 0:
            errors.append(f"{path.name}: outcome — HIL 파생 학습에는 개입 기록과 실행 길이가 필요")
        elif not any(item["start_s"] == 0 and item["end_s"] >= duration_s for item in interventions):
            errors.append(f"{path.name}: outcome — HIL 파생 구간 전체를 덮는 개입 기록이 필요")
        if provenance.get("derived_dataset_id") and (
            provenance.get("derived_dataset_id") != doc.get("dataset_id")
        ):
            errors.append(
                f"{path.name}: training_provenance/derived_dataset_id — dataset_id와 일치해야 함"
            )
        if provenance.get("source_dataset_id") == doc.get("dataset_id"):
            errors.append(
                f"{path.name}: training_provenance/source_dataset_id — "
                "HIL 복구 원본과 파생 데이터셋을 분리해야 함"
            )
    return errors


def validate_full(schema: dict, docs: list[tuple[Path, dict]]) -> list[str]:
    from jsonschema import Draft202012Validator, FormatChecker

    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    errors = []
    episode_paths: dict[tuple[str, int], Path] = {}
    for path, doc in docs:
        schema_errors = sorted(validator.iter_errors(doc), key=lambda e: list(e.path))
        for err in schema_errors:
            location = "/".join(str(p) for p in err.path) or "(root)"
            errors.append(f"{path.name}: {location} — {err.message}")
        if not schema_errors and isinstance(doc, dict):
            errors.extend(_semantic_errors(path, doc))
            key = (doc["dataset_id"], doc["episode_index"])
            if key in episode_paths:
                errors.append(f"{path.name}: 중복 에피소드 조인키 {key} — {episode_paths[key]}")
            else:
                episode_paths[key] = path
    return errors


def validate_failure_references(
    docs: list[tuple[Path, dict]], failure_record_dir: Path
) -> list[str]:
    try:
        from validate_failure_record import collect_targets as collect_failure_targets
        from validate_failure_record import load_json as load_failure_json
        from validate_failure_record import SCHEMA_PATH as failure_schema_path
        from validate_failure_record import validate_full as validate_failure_full
    except ImportError:
        from tools.validate_failure_record import collect_targets as collect_failure_targets
        from tools.validate_failure_record import load_json as load_failure_json
        from tools.validate_failure_record import SCHEMA_PATH as failure_schema_path
        from tools.validate_failure_record import validate_full as validate_failure_full

    failure_paths = collect_failure_targets(failure_record_dir)
    if not failure_paths:
        return [f"{failure_record_dir}: 검증할 실패 레코드 json 이 없음"]
    failure_schema = load_failure_json(failure_schema_path)
    failure_documents = []
    failure_ids: set[str] = set()
    duplicate_ids: set[str] = set()
    for path in failure_paths:
        try:
            failure_record = load_failure_json(path)
        except (OSError, json.JSONDecodeError, ValueError) as error:
            return [f"{path.name}: 실패 레코드 JSON을 읽을 수 없음 — {error}"]
        failure_documents.append((path, failure_record))
        failure_id = failure_record.get("failure_id")
        if isinstance(failure_id, str):
            if failure_id in failure_ids:
                duplicate_ids.add(failure_id)
            failure_ids.add(failure_id)

    errors = validate_failure_full(failure_schema, failure_documents)
    errors.extend(
        f"{failure_record_dir}: 중복 failure_id: {failure_id}"
        for failure_id in sorted(duplicate_ids)
    )
    for path, doc in docs:
        outcome = doc.get("outcome")
        if not isinstance(outcome, dict):
            continue  # episode 스키마 오류는 호출자의 기본 검증 결과에 남긴다.
        failure_record_id = outcome.get("failure_record_id")
        if failure_record_id and failure_record_id not in failure_ids:
            errors.append(
                f"{path.name}: outcome/failure_record_id — "
                f"대응하는 실패 레코드가 없음: {failure_record_id}"
            )
    return errors


def validate_minimal(schema: dict, docs: list[tuple[Path, dict]]) -> list[str]:
    required = schema.get("required", [])
    errors = []
    for path, doc in docs:
        for key in required:
            if key not in doc:
                errors.append(f"{path.name}: 필수 필드 누락 — {key}")
        if doc.get("include") is False and not doc.get("exclude_reason"):
            errors.append(f"{path.name}: include=false 인데 exclude_reason 이 없음")
    return errors


def main() -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("target", nargs="?", type=Path, default=DEFAULT_TARGET)
    parser.add_argument(
        "--failure-record-dir",
        type=Path,
        help="failure_record_id 참조를 대조할 실패 레코드 디렉터리",
    )
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

    docs = []
    for path in paths:
        try:
            docs.append((path, load_json(path)))
        except (OSError, json.JSONDecodeError, ValueError) as error:
            print(f"JSON을 읽을 수 없습니다: {path} — {error}", file=sys.stderr)
            return 2

    try:
        errors = validate_full(schema, docs)
        mode = "full"
    except ImportError:
        errors = validate_minimal(schema, docs)
        mode = "minimal"

    if args.failure_record_dir is not None:
        failure_record_dir = args.failure_record_dir.resolve()
        if not failure_record_dir.exists():
            print(f"실패 레코드 경로가 없습니다: {failure_record_dir}", file=sys.stderr)
            return 2
        if mode == "full":
            errors.extend(validate_failure_references(docs, failure_record_dir))
        else:
            print("경고: 축소 모드에서는 실패 레코드 스키마·참조 대조를 수행하지 못했습니다")

    for line in errors:
        print(f"FAIL  {line}")

    if mode == "minimal":
        print("경고: jsonschema 미설치 — 필수 필드만 확인했습니다 (pip install jsonschema)")

    if errors:
        print(f"\n{len(paths)}개 중 {len(errors)}건 실패")
        return 1

    print(f"OK  {len(paths)}개 검증 통과 ({mode} 모드)")
    return 0 if mode == "full" else 3


if __name__ == "__main__":
    sys.exit(main())
