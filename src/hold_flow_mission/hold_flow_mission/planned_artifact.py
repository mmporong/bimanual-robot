"""Validate a completed Isaac Sim water-service run for Action replay.

This module reads evidence only.  It does not start Isaac Sim or command a
robot.  A result is accepted only when its recorded tool/input hashes still
match the files that produced it and the requested phase has explicit evidence.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Iterable


PHASE_SAMPLE_EVIDENCE = {
    "ALIGN_KITCHEN": ("LEFT_RESET",),
    "GRASP_CUP": ("LEFT_LIFT_HOLD",),
    "GRASP_BOTTLE": ("RIGHT_LIFT_HOLD",),
    "POUR": ("POUR_HOLD",),
    "RETURN_BOTTLE": ("RIGHT_PLACE_HOLD",),
    "PLACE_DECK": ("TRAY_PLACE_HOLD",),
    "ALIGN_TABLE": ("SETTLE_BASE",),
    "REGRASP_CUP": ("TRAY_LIFT_HOLD",),
    "SERVE": ("LEFT_PLACE_HOLD",),
}
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class ArtifactValidationError(ValueError):
    """A stable failure code plus a human-readable validation message."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class PlannedArtifactReport:
    result_path: Path
    result_sha256: str
    simulated_s: float
    elapsed_wall_s: float
    verified_phases: tuple[str, ...]
    table_id: str = "table_1"
    drink: str = "COLD_WATER"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_true(result: dict, key: str) -> None:
    if result.get(key) is not True:
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID",
            f"result.{key} must be true",
        )


def _nonnegative_finite(result: dict, key: str) -> float:
    value = result.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0.0
    ):
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID",
            f"result.{key} must be a non-negative finite number",
        )
    return float(value)


def _validate_recorded_hashes(result: dict, repo_root: Path) -> None:
    tool_hashes = result.get("tool_sha256")
    input_hashes = result.get("input_sha256")
    if not isinstance(tool_hashes, dict) or not tool_hashes:
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID", "result.tool_sha256 is required"
        )
    if not isinstance(input_hashes, dict) or not input_hashes:
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID", "result.input_sha256 is required"
        )

    for name, expected in tool_hashes.items():
        if (
            not isinstance(name, str)
            or Path(name).name != name
            or not isinstance(expected, str)
            or not SHA256_PATTERN.fullmatch(expected)
        ):
            raise ArtifactValidationError(
                "ARTIFACT_CONTRACT_INVALID",
                "tool_sha256 entries must use bare filenames and lowercase SHA-256",
            )
        path = repo_root / "tools" / name
        if not path.is_file():
            raise ArtifactValidationError(
                "ARTIFACT_FILE_MISSING", f"recorded tool is missing: {path}"
            )
        if _sha256(path) != expected:
            raise ArtifactValidationError(
                "ARTIFACT_HASH_MISMATCH", f"recorded tool changed: {path}"
            )

    for raw_path, expected in input_hashes.items():
        if (
            not isinstance(raw_path, str)
            or not Path(raw_path).is_absolute()
            or not isinstance(expected, str)
            or not SHA256_PATTERN.fullmatch(expected)
        ):
            raise ArtifactValidationError(
                "ARTIFACT_CONTRACT_INVALID",
                "input_sha256 entries must use absolute paths and lowercase SHA-256",
            )
        path = Path(raw_path).expanduser()
        if not path.is_file():
            raise ArtifactValidationError(
                "ARTIFACT_FILE_MISSING", f"recorded input is missing: {path}"
            )
        if _sha256(path) != expected:
            raise ArtifactValidationError(
                "ARTIFACT_HASH_MISMATCH", f"recorded input changed: {path}"
            )


def load_planned_artifact(
    result_path: Path,
    *,
    repo_root: Path,
) -> PlannedArtifactReport:
    result_path = Path(result_path).expanduser().resolve()
    repo_root = Path(repo_root).expanduser().resolve()
    if not result_path.is_file():
        raise ArtifactValidationError(
            "ARTIFACT_NOT_FOUND", f"result artifact is missing: {result_path}"
        )
    try:
        result = json.loads(result_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ArtifactValidationError(
            "ARTIFACT_JSON_INVALID", f"cannot read result artifact: {exc}"
        ) from exc
    if not isinstance(result, dict):
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID", "result artifact must be a JSON object"
        )
    if result.get("mode") != "water-service":
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID", "result.mode must be water-service"
        )
    if result.get("hardware_accessed") is not False:
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID", "artifact must record hardware_accessed=false"
        )
    if result.get("object_attachment_used") is not False:
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID", "artifact must not use hidden object attachment"
        )
    if result.get("failure") != "water_served":
        raise ArtifactValidationError(
            "ARTIFACT_TASK_FAILED", f"artifact ended with {result.get('failure')!r}"
        )
    for key in (
        "task_pass",
        "sequence_pass",
        "ground_verified",
        "water_pour_completed",
        "deck_transport_completed",
        "regrasp_completed",
    ):
        _require_true(result, key)
    _validate_recorded_hashes(result, repo_root)
    simulated_s = _nonnegative_finite(result, "simulated_s")
    elapsed_wall_s = _nonnegative_finite(result, "elapsed_wall_s")

    samples = result.get("samples")
    if not isinstance(samples, list):
        raise ArtifactValidationError(
            "ARTIFACT_CONTRACT_INVALID", "result.samples must be a list"
        )
    sampled_phases = {
        sample.get("phase") for sample in samples if isinstance(sample, dict)
    }
    verified = tuple(
        phase
        for phase, evidence in PHASE_SAMPLE_EVIDENCE.items()
        if all(item in sampled_phases for item in evidence)
    )
    missing = sorted(set(PHASE_SAMPLE_EVIDENCE) - set(verified))
    if missing:
        raise ArtifactValidationError(
            "ARTIFACT_PHASE_EVIDENCE_MISSING",
            f"artifact lacks phase evidence: {', '.join(missing)}",
        )

    return PlannedArtifactReport(
        result_path=result_path,
        result_sha256=_sha256(result_path),
        simulated_s=simulated_s,
        elapsed_wall_s=elapsed_wall_s,
        verified_phases=verified,
    )


def validate_request_scope(
    report: PlannedArtifactReport,
    *,
    table_id: str,
    drink: str,
    phase_ids: Iterable[str],
) -> None:
    if table_id != report.table_id:
        raise ArtifactValidationError(
            "ARTIFACT_TABLE_UNSUPPORTED",
            f"artifact covers {report.table_id}, not {table_id}",
        )
    if drink != report.drink:
        raise ArtifactValidationError(
            "ARTIFACT_DRINK_UNSUPPORTED",
            f"artifact covers {report.drink}, not {drink}",
        )
    unsupported = sorted(set(phase_ids) - set(report.verified_phases))
    if unsupported:
        raise ArtifactValidationError(
            "ARTIFACT_PHASE_UNSUPPORTED",
            f"artifact does not verify phases: {', '.join(unsupported)}",
        )
