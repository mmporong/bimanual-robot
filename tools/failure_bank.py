#!/usr/bin/env python3
"""실패 근거를 로컬 번들로 보존하고 검증된 분석 레코드를 집계한다.

실물 제어·네트워크 업로드·학습 데이터 변경은 수행하지 않는다.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

from validate_failure_record import SCHEMA_PATH, collect_targets, load_json, validate_full

REPO_ROOT = Path(__file__).resolve().parent.parent
CAPTURE_FIELDS = {"failure_id", "triage_owner", "source", "observation", "context", "immediate_action"}
EVIDENCE_KINDS = {
    ".mp4": "video", ".avi": "video", ".mcap": "rosbag", ".bag": "rosbag",
    ".png": "image", ".jpg": "image", ".jpeg": "image", ".json": "metric_json",
    ".parquet": "trajectory", ".log": "console_log",
}


def checked_documents(target: Path) -> list[tuple[Path, dict]]:
    if not target.exists():
        raise ValueError(f"대상이 없습니다: {target}")
    paths = collect_targets(target)
    if not paths:
        raise ValueError(f"실패 레코드가 없습니다: {target}")
    documents = [(path, load_json(path)) for path in paths]
    errors = validate_full(load_json(SCHEMA_PATH), documents)
    ids = [doc.get("failure_id") for _, doc in documents if isinstance(doc, dict)]
    duplicates = [key for key, count in Counter(ids).items() if count > 1]
    if duplicates:
        errors.append(f"중복 failure_id: {', '.join(duplicates)}")
    if errors:
        raise ValueError("\n".join(errors))
    return documents


def capture(input_path: Path, evidence_paths: list[Path], root: Path) -> Path:
    """입력 사실과 실제 파일만 보존한다. 원인·성공·재시험은 추정하지 않는다."""
    payload = load_json(input_path)
    if not isinstance(payload, dict) or set(payload) != CAPTURE_FIELDS:
        raise ValueError("capture 입력 필드: " + ", ".join(sorted(CAPTURE_FIELDS)))
    if not 1 <= len(evidence_paths) <= 99:
        raise ValueError("근거 파일을 1~99개 지정해야 합니다")
    files = [path.expanduser().resolve() for path in evidence_paths]
    if len(set(files)) != len(files):
        raise ValueError("같은 근거 파일을 중복 지정했습니다")
    for path in files:
        if not path.is_file():
            raise ValueError(f"읽을 근거 파일이 없습니다: {path}")
    root = root.expanduser().resolve()
    if root == REPO_ROOT or REPO_ROOT in root.parents:
        raise ValueError("실데이터 보관 경로는 Git 저장소 밖으로 지정하세요")
    document = {
        **{key: value for key, value in payload.items() if key != "immediate_action"},
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "status": "open",
        "evidence": [
            {
                "id": f"EV-{index:02d}",
                "kind": EVIDENCE_KINDS.get(path.suffix.lower(), "other"),
                "uri": f"evidence/EV-{index:02d}-{path.name}",
                "description": f"수집 시 제공된 근거 파일: {path.name}",
            }
            for index, path in enumerate(files, 1)
        ],
        "diagnosis": {"cause_status": "untriaged", "hypotheses": []},
        "response": {
            "immediate_action": payload["immediate_action"],
            "retraining_decision": "not_evaluated",
        },
        "validation": {
            "result": "not_run", "same_condition_run_ids": [],
            "regression_result": "not_run", "regression_run_ids": [],
        },
        "data_use": {
            "primary_disposition": "failure_bank", "include_in_training": False,
            "collection_priority": "none", "reason": "원인 분석 전 원본 보존",
        },
    }
    errors = validate_full(load_json(SCHEMA_PATH), [(input_path, document)])
    if errors:
        raise ValueError("\n".join(errors))
    # 스키마가 ID를 검증한 뒤에만 디렉터리를 만든다. 동일 ID는 덮어쓰지 않는다.
    bundle = root / document["failure_id"]
    bundle.mkdir(parents=True, exist_ok=False)
    (bundle / "evidence").mkdir()
    for path, evidence in zip(files, document["evidence"]):
        digest = hashlib.sha256()
        with path.open("rb") as source, (bundle / evidence["uri"]).open("xb") as dest:
            for block in iter(lambda: source.read(1024 * 1024), b""):
                dest.write(block)
                digest.update(block)
        evidence["sha256"] = digest.hexdigest()
    record = bundle / "failure.json"
    # 모든 파일 복사 뒤 마지막으로 기록한다. 중간 실패 번들은 조사용으로 보존한다.
    with record.open("x", encoding="utf-8") as output:
        json.dump(document, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write("\n")
    return record


def summarize(documents: list[tuple[Path, dict]], include_examples: bool = False) -> dict:
    examples = sum(doc["failure_id"].startswith("EXAMPLE-") for _, doc in documents)
    records = [doc for _, doc in documents
               if include_examples or not doc["failure_id"].startswith("EXAMPLE-")]

    def counts(values):
        return dict(sorted(Counter(values).items()))

    return {
        "count_unit": "failure_record",
        "record_count": len(records),
        "excluded_example_count": 0 if include_examples else examples,
        "includes_examples": include_examples,
        "success_rate": None,
        "limitations": [
            "원인 조사 묶음 수이며 실패 실행 횟수가 아님",
            "전체 성공·실패 실행 분모가 없어 성공률을 계산하지 않음",
            "근거의 실측 진위·재시험 실행 여부는 집계로 검증하지 않음",
        ],
        "by_environment": counts(doc["source"]["environment"] for doc in records),
        "by_stage": counts(doc["observation"]["stage"] for doc in records),
        "by_stage_and_code": {
            stage: counts(doc["observation"]["observed_code"] for doc in records
                          if doc["observation"]["stage"] == stage)
            for stage in sorted({doc["observation"]["stage"] for doc in records})
        },
        "by_observed_code": counts(doc["observation"]["observed_code"] for doc in records),
        "by_confirmed_cause": counts(
            doc["diagnosis"]["root_cause"]["layer"]
            if doc["diagnosis"]["cause_status"] == "confirmed" else "unconfirmed"
            for doc in records
        ),
        "by_status": counts(doc["status"] for doc in records),
        "by_retraining_decision": counts(doc["response"]["retraining_decision"] for doc in records),
        "by_disposition": counts(doc["data_use"]["primary_disposition"] for doc in records),
        "by_same_condition_result": counts(doc["validation"]["result"] for doc in records),
        "by_regression_result": counts(doc["validation"]["regression_result"] for doc in records),
        "needs_triage": sorted(doc["failure_id"] for doc in records
                               if doc["diagnosis"]["cause_status"] != "confirmed"),
        "needs_retest": sorted(doc["failure_id"] for doc in records
                               if doc["diagnosis"]["cause_status"] == "confirmed"
                               and doc["status"] != "resolved"),
        "training_review": [
            {"failure_id": doc["failure_id"], "data_use": doc["data_use"]}
            for doc in sorted(records, key=lambda item: item["failure_id"])
            if doc["data_use"]["include_in_training"]
        ],
    }


def verify_evidence(documents: list[tuple[Path, dict]]) -> int:
    """로컬 번들 근거를 재해시한다. 외부 URI를 확인된 파일로 취급하지 않는다."""
    verified = 0
    for record, document in documents:
        for evidence in document["evidence"]:
            uri = evidence["uri"]
            if "://" in uri or Path(uri).is_absolute():
                raise ValueError(f"{record}: 로컬 상대 경로 근거만 검사 가능: {evidence['id']}")
            base = record.parent.resolve()
            path = (base / uri).resolve()
            if base not in path.parents or not path.is_file():
                raise ValueError(f"{record}: 근거 누락 또는 번들 밖 경로: {evidence['id']}")
            if "sha256" not in evidence:
                raise ValueError(f"{record}: SHA-256 없음: {evidence['id']}")
            with path.open("rb") as source:
                actual = hashlib.file_digest(source, "sha256").hexdigest()
            if actual != evidence["sha256"].lower():
                raise ValueError(f"{record}: SHA-256 불일치: {evidence['id']}")
            verified += 1
    return verified


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture", help="근거 복사·해시와 미분석 레코드 생성")
    capture_parser.add_argument("--input", type=Path, required=True)
    capture_parser.add_argument("--evidence", nargs="+", type=Path, required=True)
    capture_parser.add_argument("--root", type=Path, required=True)
    summary_parser = commands.add_parser("summary", help="유효한 실패 레코드만 집계")
    summary_parser.add_argument("target", type=Path)
    summary_parser.add_argument("--include-examples", action="store_true")
    verify_parser = commands.add_parser("verify-evidence", help="로컬 근거 파일 SHA-256 대조")
    verify_parser.add_argument("target", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "capture":
            print(capture(args.input, args.evidence, args.root))
        else:
            documents = checked_documents(args.target.expanduser().resolve())
            if args.command == "summary":
                print(json.dumps(summarize(documents, args.include_examples),
                                 ensure_ascii=False, indent=2, allow_nan=False))
            else:
                print(f"OK  근거 {verify_evidence(documents)}개 SHA-256 일치")
    except ImportError:
        print("FAIL  jsonschema가 필요합니다. 전체 검증 없이 저장·집계하지 않습니다", file=sys.stderr)
        return 3
    except (ValueError, OSError) as error:
        print(f"FAIL  {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
