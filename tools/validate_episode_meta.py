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
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SCHEMA_PATH = REPO_ROOT / "data" / "schema" / "episode_metadata.schema.json"
DEFAULT_TARGET = REPO_ROOT / "data" / "schema" / "example_episode_meta.json"


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def collect_targets(target: Path) -> list[Path]:
    if target.is_dir():
        return sorted(p for p in target.glob("*.json") if p != SCHEMA_PATH)
    return [target]


def validate_full(schema: dict, docs: list[tuple[Path, dict]]) -> list[str]:
    from jsonschema import Draft202012Validator

    validator = Draft202012Validator(schema)
    errors = []
    for path, doc in docs:
        for err in sorted(validator.iter_errors(doc), key=lambda e: list(e.path)):
            location = "/".join(str(p) for p in err.path) or "(root)"
            errors.append(f"{path.name}: {location} — {err.message}")
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
    target = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_TARGET
    if not target.exists():
        print(f"대상이 없습니다: {target}", file=sys.stderr)
        return 2

    schema = load_json(SCHEMA_PATH)
    paths = collect_targets(target)
    if not paths:
        print(f"검증할 json 이 없습니다: {target}", file=sys.stderr)
        return 2

    docs = [(p, load_json(p)) for p in paths]

    try:
        errors = validate_full(schema, docs)
        mode = "full"
    except ImportError:
        errors = validate_minimal(schema, docs)
        mode = "minimal"

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
