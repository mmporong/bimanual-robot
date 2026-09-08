"""가상 근거 파일로 보존·무결성·집계 경계를 검증한다."""

import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))
import failure_bank


class FailureBankTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.input = self.root / "input.json"
        self.payload = json.loads((REPO_ROOT / "data/schema/example_failure_capture.json")
                                  .read_text(encoding="utf-8"))
        self.input.write_text(json.dumps(self.payload), encoding="utf-8")
        self.evidence = self.root / "observed.log"
        self.evidence.write_text("synthetic observation\n", encoding="utf-8")

    def capture(self):
        return failure_bank.capture(self.input, [self.evidence], self.root / "bank")

    def test_capture_preserves_bytes_and_does_not_infer_cause(self):
        record = self.capture()
        document = json.loads(record.read_text(encoding="utf-8"))
        evidence = document["evidence"][0]
        self.assertEqual((record.parent / evidence["uri"]).read_bytes(), self.evidence.read_bytes())
        self.assertEqual(evidence["sha256"], hashlib.sha256(self.evidence.read_bytes()).hexdigest())
        self.assertEqual(document["diagnosis"], {"cause_status": "untriaged", "hypotheses": []})
        self.assertFalse(document["data_use"]["include_in_training"])
        self.assertEqual(failure_bank.verify_evidence(failure_bank.checked_documents(record)), 1)

    def test_duplicate_capture_never_overwrites(self):
        record = self.capture()
        original = record.read_bytes()
        with self.assertRaises(FileExistsError):
            self.capture()
        self.assertEqual(record.read_bytes(), original)

    def test_bad_id_does_not_create_files(self):
        self.payload["failure_id"] = "../../escape"
        self.input.write_text(json.dumps(self.payload), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.capture()
        self.assertFalse((self.root / "bank").exists())

    def test_evidence_is_required_and_must_exist(self):
        for files in ([], [self.root / "missing.log"], [self.evidence, self.evidence]):
            with self.subTest(files=files), self.assertRaises(ValueError):
                failure_bank.capture(self.input, files, self.root / "bank")
        self.assertFalse((self.root / "bank").exists())

    def test_repository_cannot_be_artifact_root(self):
        with self.assertRaisesRegex(ValueError, "저장소 밖"):
            failure_bank.capture(self.input, [self.evidence], REPO_ROOT / "data/raw")

    def test_hash_detects_modified_evidence(self):
        record = self.capture()
        documents = failure_bank.checked_documents(record)
        path = record.parent / documents[0][1]["evidence"][0]["uri"]
        path.write_text("modified\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "불일치"):
            failure_bank.verify_evidence(documents)

    def test_missing_and_outside_evidence_is_not_verified(self):
        record = self.capture()
        original = failure_bank.checked_documents(record)
        for uri in ("evidence/missing.log", "../observed.log", "hf-private://unverified/log"):
            docs = copy.deepcopy(original)
            docs[0][1]["evidence"][0]["uri"] = uri
            with self.subTest(uri=uri), self.assertRaises(ValueError):
                failure_bank.verify_evidence(docs)

    def test_summary_excludes_examples_and_cannot_report_success_rate(self):
        docs = failure_bank.checked_documents(self.capture())
        result = failure_bank.summarize(docs)
        self.assertEqual(result["record_count"], 0)
        self.assertEqual(result["excluded_example_count"], 1)
        self.assertIsNone(result["success_rate"])
        included = failure_bank.summarize(docs, include_examples=True)
        self.assertEqual(included["record_count"], 1)
        self.assertEqual(included["by_confirmed_cause"], {"unconfirmed": 1})
        self.assertEqual(included["by_stage_and_code"], {"cup_grasp": {"CUP_GRASP_MISS": 1}})
        self.assertEqual(included["by_control_strategy"], {"ACT_ALL": 1})
        self.assertEqual(included["by_phase_id"], {"CUP_PICK": 1})
        self.assertEqual(included["by_backend"], {"ACT": 1})
        self.assertEqual(included["training_review"], [])

    def test_recursive_bundle_read_and_duplicate_id_rejection(self):
        record = self.capture()
        # 근거 폴더의 metric JSON은 실패 레코드로 오인하면 안 된다.
        (record.parent / "evidence" / "metric.json").write_text('{"value": 1}', encoding="utf-8")
        self.assertEqual(len(failure_bank.checked_documents(self.root / "bank")), 1)
        duplicate = self.root / "bank" / "duplicate"
        duplicate.mkdir()
        (duplicate / "failure.json").write_bytes(record.read_bytes())
        with self.assertRaisesRegex(ValueError, "중복"):
            failure_bank.checked_documents(self.root / "bank")

    def test_cli_roundtrip(self):
        cli = [sys.executable, str(REPO_ROOT / "tools/failure_bank.py")]
        captured = subprocess.run(cli + ["capture", "--input", str(self.input), "--evidence",
                                        str(self.evidence), "--root", str(self.root / "bank")],
                                  text=True, capture_output=True)
        self.assertEqual(captured.returncode, 0, captured.stderr)
        verified = subprocess.run(cli + ["verify-evidence", str(self.root / "bank")],
                                  text=True, capture_output=True)
        self.assertEqual(verified.returncode, 0, verified.stderr)
        summary = subprocess.run(cli + ["summary", str(self.root / "bank"), "--include-examples"],
                                 text=True, capture_output=True)
        self.assertEqual(summary.returncode, 0, summary.stderr)
        self.assertEqual(json.loads(summary.stdout)["record_count"], 1)

    def test_invalid_json_returns_nonzero_without_traceback(self):
        self.input.write_text("{", encoding="utf-8")
        result = subprocess.run([sys.executable, str(REPO_ROOT / "tools/failure_bank.py"),
                                 "capture", "--input", str(self.input), "--evidence",
                                 str(self.evidence), "--root", str(self.root / "bank")],
                                text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("Traceback", result.stderr)
        self.assertFalse((self.root / "bank").exists())


if __name__ == "__main__":
    unittest.main()
