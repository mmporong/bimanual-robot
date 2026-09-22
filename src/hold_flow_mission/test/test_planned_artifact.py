import hashlib
import json
from pathlib import Path
import unittest

from hold_flow_mission.planned_artifact import (
    ArtifactValidationError,
    PHASE_SAMPLE_EVIDENCE,
    load_planned_artifact,
    validate_request_scope,
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class PlannedArtifactTest(unittest.TestCase):
    def setUp(self):
        import tempfile

        self._temp = tempfile.TemporaryDirectory()
        self.root = Path(self._temp.name)
        (self.root / "tools").mkdir()
        self.tool = self.root / "tools" / "runner.py"
        self.tool.write_text("print('runner')\n", encoding="utf-8")
        self.source = self.root / "source.json"
        self.source.write_text("{}\n", encoding="utf-8")
        self.result_path = self.root / "result.json"
        self.result = {
            "mode": "water-service",
            "hardware_accessed": False,
            "object_attachment_used": False,
            "failure": "water_served",
            "task_pass": True,
            "sequence_pass": True,
            "ground_verified": True,
            "water_pour_completed": True,
            "deck_transport_completed": True,
            "regrasp_completed": True,
            "simulated_s": 273.0,
            "elapsed_wall_s": 900.0,
            "tool_sha256": {"runner.py": digest(self.tool)},
            "input_sha256": {str(self.source): digest(self.source)},
            "samples": [
                {"phase": evidence}
                for values in PHASE_SAMPLE_EVIDENCE.values()
                for evidence in values
            ],
        }
        self._write_result()

    def tearDown(self):
        self._temp.cleanup()

    def _write_result(self):
        self.result_path.write_text(json.dumps(self.result), encoding="utf-8")

    def test_valid_result_verifies_every_manipulation_phase(self):
        report = load_planned_artifact(self.result_path, repo_root=self.root)
        self.assertEqual(set(report.verified_phases), set(PHASE_SAMPLE_EVIDENCE))
        validate_request_scope(
            report,
            table_id="table_1",
            drink="COLD_WATER",
            phase_ids=("POUR", "SERVE"),
        )

    def test_changed_tool_invalidates_artifact(self):
        self.tool.write_text("print('changed')\n", encoding="utf-8")
        with self.assertRaisesRegex(ArtifactValidationError, "recorded tool changed") as caught:
            load_planned_artifact(self.result_path, repo_root=self.root)
        self.assertEqual(caught.exception.code, "ARTIFACT_HASH_MISMATCH")

    def test_failed_result_is_rejected(self):
        self.result["task_pass"] = False
        self._write_result()
        with self.assertRaises(ArtifactValidationError) as caught:
            load_planned_artifact(self.result_path, repo_root=self.root)
        self.assertEqual(caught.exception.code, "ARTIFACT_CONTRACT_INVALID")

    def test_missing_phase_evidence_is_rejected(self):
        self.result["samples"] = []
        self._write_result()
        with self.assertRaises(ArtifactValidationError) as caught:
            load_planned_artifact(self.result_path, repo_root=self.root)
        self.assertEqual(caught.exception.code, "ARTIFACT_PHASE_EVIDENCE_MISSING")

    def test_malformed_tool_hash_entry_is_rejected_without_path_escape(self):
        self.result["tool_sha256"] = {"../outside.py": "0" * 64}
        self._write_result()
        with self.assertRaises(ArtifactValidationError) as caught:
            load_planned_artifact(self.result_path, repo_root=self.root)
        self.assertEqual(caught.exception.code, "ARTIFACT_CONTRACT_INVALID")

    def test_nonfinite_timing_is_rejected(self):
        self.result["simulated_s"] = float("nan")
        self._write_result()
        with self.assertRaises(ArtifactValidationError) as caught:
            load_planned_artifact(self.result_path, repo_root=self.root)
        self.assertEqual(caught.exception.code, "ARTIFACT_CONTRACT_INVALID")

    def test_scope_rejects_unverified_table_and_drink(self):
        report = load_planned_artifact(self.result_path, repo_root=self.root)
        with self.assertRaises(ArtifactValidationError) as table_error:
            validate_request_scope(
                report,
                table_id="table_2",
                drink="COLD_WATER",
                phase_ids=("POUR",),
            )
        self.assertEqual(table_error.exception.code, "ARTIFACT_TABLE_UNSUPPORTED")
        with self.assertRaises(ArtifactValidationError) as drink_error:
            validate_request_scope(
                report,
                table_id="table_1",
                drink="HOT_WATER",
                phase_ids=("POUR",),
            )
        self.assertEqual(drink_error.exception.code, "ARTIFACT_DRINK_UNSUPPORTED")


if __name__ == "__main__":
    unittest.main()
