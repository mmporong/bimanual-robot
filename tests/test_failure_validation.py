from __future__ import annotations

import copy
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from tools import validate_episode_meta, validate_failure_record


REPO_ROOT = Path(__file__).resolve().parents[1]
FAILURE_SCHEMA = validate_failure_record.load_json(
    REPO_ROOT / "data/schema/failure_record.schema.json"
)
EPISODE_SCHEMA = validate_episode_meta.load_json(
    REPO_ROOT / "data/schema/episode_metadata.schema.json"
)
FAILURE_EXAMPLE = validate_failure_record.load_json(
    REPO_ROOT / "data/schema/example_failure_record.json"
)
EPISODE_EXAMPLE = validate_episode_meta.load_json(
    REPO_ROOT / "data/schema/example_episode_meta.json"
)


class FailureRecordValidationTest(unittest.TestCase):
    def validate(self, document: dict) -> list[str]:
        return validate_failure_record.validate_full(
            FAILURE_SCHEMA, [(Path("failure.json"), document)]
        )

    def test_example_is_valid(self) -> None:
        self.assertEqual([], self.validate(copy.deepcopy(FAILURE_EXAMPLE)))

    def test_manipulation_failure_requires_reproducible_backend_context(self) -> None:
        for field in ("control_strategy", "phase_id", "backend"):
            document = copy.deepcopy(FAILURE_EXAMPLE)
            document["context"].pop(field)
            self.assertTrue(any(field in error for error in self.validate(document)))

        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["context"]["phase_id"] = "POUR"
        self.assertTrue(any("stage와 phase" in error for error in self.validate(document)))

        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["context"].update(control_strategy="PLANNED_ALL", backend="ACT")
        self.assertTrue(any("strategy와 backend" in error for error in self.validate(document)))

        document["context"].update(backend="PLANNED", backend_artifacts={})
        document["context"].pop("checkpoint_id")
        self.assertTrue(any("PLANNED 실패 재현" in error for error in self.validate(document)))

    def test_duplicate_and_missing_evidence_ids_are_rejected(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["evidence"][1]["id"] = "EV-01"
        document["diagnosis"]["hypotheses"][0]["evidence_ids"] = ["EV-99"]
        errors = self.validate(document)
        self.assertTrue(any("중복 evidence ID" in error for error in errors))
        self.assertTrue(any("존재하지 않는 evidence ID" in error for error in errors))

    def test_duplicate_hypothesis_id_is_rejected(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["diagnosis"]["hypotheses"][1]["id"] = "H-01"
        self.assertTrue(any("중복 hypothesis ID" in error for error in self.validate(document)))

    def test_reversed_time_ranges_are_rejected(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["observation"]["time_range_s"] = [4.0, 3.0]
        document["evidence"][0]["time_range_s"] = [9.0, 8.0]
        errors = self.validate(document)
        self.assertEqual(2, sum("시작 시각이 종료 시각보다 늦음" in error for error in errors))

    def test_unconfirmed_root_cause_and_unresolved_confirmed_are_rejected(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["status"] = "unresolved"
        errors = self.validate(document)
        self.assertTrue(any("unresolved 상태" in error for error in errors))

        document["diagnosis"]["cause_status"] = "hypothesis"
        errors = self.validate(document)
        self.assertTrue(any("confirmed일 때만" in error for error in errors))

    def test_confirmed_cause_may_have_validation_not_run(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["status"] = "confirmed"
        document["validation"] = {
            "result": "not_run",
            "same_condition_run_ids": [],
            "regression_result": "not_run",
            "regression_run_ids": [],
        }
        self.assertEqual([], self.validate(document))

    def test_resolved_requires_complete_acceptance_evidence(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["validation"]["protocol_id"] = ""
        document["validation"]["regression_result"] = "not_run"
        document["validation"]["regression_run_ids"] = []
        document["validation"]["acceptance_evidence_ids"] = []
        errors = self.validate(document)
        self.assertTrue(any("protocol_id" in error for error in errors))
        self.assertTrue(any("regression_result" in error for error in errors))
        self.assertTrue(any("acceptance_evidence_ids" in error for error in errors))

    def test_counterexample_cannot_be_automatically_added_to_bc(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["data_use"].update(
            primary_disposition="counterexample",
            include_in_training=True,
            training_dataset_id="TEAM/bc-v2",
        )
        self.assertNotEqual([], self.validate(document))

    def test_hil_training_requires_derived_recovery_provenance(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["data_use"].update(
            primary_disposition="hil_recovery",
            include_in_training=True,
            training_dataset_id="TEAM/recovery-v1",
        )
        errors = self.validate(document)
        self.assertTrue(any("source_dataset_id" in error for error in errors))
        self.assertTrue(any("recovery_time_range_s" in error for error in errors))

        document["data_use"].update(
            source_dataset_id="TEAM/raw-hil-v1",
            derived_dataset_id="TEAM/recovery-v1",
            recovery_time_range_s=[12.5, 14.0],
            recovery_evidence_ids=["EV-01"],
            recovery_verified_success=True,
            recovery_control_source="teleop",
        )
        document["response"]["retraining_decision"] = "collect_hil_recovery"
        for layer in ("navigation", "hardware", "control_runtime"):
            document["diagnosis"]["root_cause"]["layer"] = layer
            self.assertTrue(any("policy_data 원인" in error for error in self.validate(document)))
        document["diagnosis"]["root_cause"]["layer"] = "policy_data"
        self.assertEqual([], self.validate(document))

        document["data_use"]["recovery_time_range_s"] = [999, 1000]
        self.assertTrue(any("근거 시간 범위" in error for error in self.validate(document)))
        document["data_use"]["recovery_time_range_s"] = [12.5, 14.0]
        document["data_use"]["source_dataset_id"] = "TEAM/recovery-v1"
        self.assertTrue(any("원본과 파생" in error for error in self.validate(document)))

        document["data_use"]["source_dataset_id"] = "TEAM/raw-hil-v1"
        document["data_use"]["training_dataset_id"] = "TEAM/other-v1"
        self.assertTrue(any("derived_dataset_id" in error for error in self.validate(document)))

    def test_untriaged_or_unverified_recovery_cannot_enter_training(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["data_use"].update(
            primary_disposition="hil_recovery", include_in_training=True,
            training_dataset_id="TEAM/recovery-v1", source_dataset_id="TEAM/raw-v1",
            derived_dataset_id="TEAM/recovery-v1", recovery_time_range_s=[12.5, 14.0],
            recovery_evidence_ids=["EV-01"], recovery_verified_success=True,
            recovery_control_source="teleop",
        )
        document["response"]["retraining_decision"] = "collect_hil_recovery"
        document["diagnosis"]["root_cause"]["layer"] = "policy_data"
        self.assertEqual([], self.validate(document))
        document["data_use"]["recovery_verified_success"] = False
        self.assertTrue(any("복구 성공" in error for error in self.validate(document)))
        document["data_use"]["recovery_verified_success"] = True
        document["status"] = "open"
        document["diagnosis"] = {"cause_status": "untriaged", "hypotheses": []}
        self.assertTrue(any("원인 미확정" in error for error in self.validate(document)))

    def test_invalid_datetime_is_rejected(self) -> None:
        document = copy.deepcopy(FAILURE_EXAMPLE)
        document["recorded_at"] = "2026-99-99"
        self.assertTrue(any("date-time" in error for error in self.validate(document)))

    def test_schema_invalid_nested_values_do_not_crash_semantic_validation(self) -> None:
        for document in (
            [],
            {**copy.deepcopy(FAILURE_EXAMPLE), "observation": None},
            {**copy.deepcopy(FAILURE_EXAMPLE), "evidence": 3},
        ):
            self.assertNotEqual([], self.validate(document))


class EpisodeMetadataValidationTest(unittest.TestCase):
    def validate(self, document: dict) -> list[str]:
        return validate_episode_meta.validate_full(
            EPISODE_SCHEMA, [(Path("episode.json"), document)]
        )

    def test_example_is_valid(self) -> None:
        self.assertEqual([], self.validate(copy.deepcopy(EPISODE_EXAMPLE)))

    def test_planned_act_and_hybrid_strategy_contracts_are_checked(self) -> None:
        planned = copy.deepcopy(EPISODE_EXAMPLE)
        planned["policy"].update(
            execution_mode="planned_baseline",
            control_strategy="PLANNED_ALL",
        )
        for phase in planned["policy"]["phase_executions"]:
            phase["backend"] = "PLANNED"
            phase["artifact_ids"] = {
                "trajectory": f"example-{phase['phase_id'].lower()}-trajectory",
                "controller": "example-controller-v1",
            }
        self.assertEqual([], self.validate(planned))

        inconsistent = copy.deepcopy(planned)
        inconsistent["policy"]["phase_executions"][0].update(
            backend="ACT", checkpoint_id="example-act-checkpoint"
        )
        inconsistent["policy"]["phase_executions"][0].pop("artifact_ids")
        self.assertTrue(any("모순" in error for error in self.validate(inconsistent)))

        hybrid = copy.deepcopy(planned)
        hybrid["request_id"] = "example-request-hybrid-0001"
        hybrid["policy"].update(
            execution_mode="autonomous_rollout",
            control_strategy="HYBRID",
        )
        pour = hybrid["policy"]["phase_executions"][3]
        pour.update(backend="ACT", checkpoint_id="example-pour-checkpoint")
        pour.pop("artifact_ids")
        self.assertEqual([], self.validate(hybrid))

        for phase in hybrid["policy"]["phase_executions"]:
            phase["backend"] = "PLANNED"
            phase.pop("checkpoint_id", None)
            phase.setdefault("artifact_ids", {"trajectory": "example-trajectory"})
        self.assertTrue(any("모두 필요" in error for error in self.validate(hybrid)))

    def test_act_phase_requires_checkpoint_and_phase_times_are_checked(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["request_id"] = "example-request-act-0001"
        document["policy"].update(
            execution_mode="autonomous_rollout",
            control_strategy="ACT_ALL",
        )
        for phase in document["policy"]["phase_executions"]:
            phase["backend"] = "ACT"
        self.assertTrue(any("checkpoint_id" in error for error in self.validate(document)))

        document["policy"]["checkpoint_id"] = "example-act-all-checkpoint"
        self.assertEqual([], self.validate(document))
        document["policy"]["phase_executions"][1].update(start_s=2.0, end_s=19.0)
        errors = self.validate(document)
        self.assertTrue(any("겹침" in error for error in errors))
        self.assertTrue(any("duration_s" in error for error in errors))

    def test_phase_slice_requires_only_phase_relevant_measurements(self) -> None:
        cup_pick = copy.deepcopy(EPISODE_EXAMPLE)
        cup_pick["policy"].update(
            episode_scope="phase_slice",
            phase_executions=[copy.deepcopy(cup_pick["policy"]["phase_executions"][0])],
        )
        cup_pick.pop("water_measurement")
        self.assertEqual([], self.validate(cup_pick))

        pour = copy.deepcopy(cup_pick)
        pour["policy"]["phase_executions"] = [
            {
                "phase_id": "POUR",
                "backend": "TELEOP",
                "start_s": 0.0,
                "end_s": 3.0,
                "start_state_source": "teleop_demo",
            }
        ]
        self.assertTrue(any("POUR phase" in error for error in self.validate(pour)))

    def test_duplicate_episode_join_key_is_rejected(self) -> None:
        documents = [(Path(name), copy.deepcopy(EPISODE_EXAMPLE)) for name in ("a.json", "b.json")]
        errors = validate_episode_meta.validate_full(EPISODE_SCHEMA, documents)
        self.assertTrue(any("중복 에피소드" in error for error in errors))
        documents[1][1]["dataset_id"] = "TEAM/separate-dataset-v1"
        self.assertEqual([], validate_episode_meta.validate_full(EPISODE_SCHEMA, documents))

    def test_failed_source_episode_cannot_be_included(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["outcome"].update(
            success=False,
            failure_stage="cup_grasp",
            failure_code="CUP_GRASP_MISS",
            failure_record_id="FAIL-20260908-0001",
        )
        self.assertTrue(any("실패 원본" in error for error in self.validate(document)))

    def test_holdout_eval_success_may_be_included(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["split"] = "holdout_eval"
        self.assertEqual([], self.validate(document))

    def test_excluded_anomalies_and_log_only_verification_cannot_be_included(self) -> None:
        for anomaly in ("servo_stall", "teleop_desync", "operator_error", "collision"):
            document = copy.deepcopy(EPISODE_EXAMPLE)
            document["outcome"]["anomalies"] = [anomaly]
            self.assertTrue(any("원본 제외" in error for error in self.validate(document)))
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["outcome"]["verified_by"] = "log_string"
        self.assertTrue(any("E8" in error for error in self.validate(document)))
        document["outcome"].update(verified_by="human_observation", anomalies=["depth_hole"])
        self.assertEqual([], self.validate(document))

    def test_successful_recovery_trace_is_allowed_when_described(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["outcome"].update(
            human_intervention_count=1,
            human_intervention_duration_s=1.5,
            interventions=[
                {
                    "start_s": 10.0,
                    "end_s": 11.5,
                    "control_source": "bi_so_leader",
                    "reason": "컵 미끄러짐을 관측해 파지를 복구함",
                }
            ],
        )
        self.assertEqual([], self.validate(document))

    def test_intervention_count_order_and_episode_bounds_are_checked(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["outcome"].update(
            human_intervention_count=2,
            interventions=[
                {
                    "start_s": 19.0,
                    "end_s": 18.0,
                    "control_source": "leader",
                    "reason": "복구",
                }
            ],
        )
        errors = self.validate(document)
        self.assertTrue(any("개수가 다름" in error for error in errors))
        self.assertTrue(any("시작·종료" in error for error in errors))

        document["outcome"]["human_intervention_count"] = 1
        document["outcome"]["interventions"][0].update(start_s=17.0, end_s=19.0)
        self.assertTrue(any("duration_s" in error for error in self.validate(document)))

    def test_hil_training_requires_separate_derived_dataset_and_exact_range(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["policy"].update(
            execution_mode="human_in_the_loop",
            checkpoint_id="TEAM/checkpoint-v1",
            episode_scope="phase_slice",
            phase_executions=[
                {
                    "phase_id": "CUP_PICK",
                    "backend": "TELEOP",
                    "start_s": 0.0,
                    "end_s": 2.2,
                    "start_state_source": "teleop_demo",
                }
            ],
        )
        document["request_id"] = "request-0001"
        errors = self.validate(document)
        self.assertTrue(any("training_provenance" in error for error in errors))

        document["dataset_id"] = "TEAM/hil-recovery-derived-v1"
        document["training_provenance"] = {
            "source_dataset_id": "TEAM/hil-raw-v1",
            "source_episode_index": 7,
            "source_split": "train",
            "derived_dataset_id": "TEAM/hil-recovery-derived-v1",
            "recovery_time_range_s": [8.2, 10.4],
            "evidence_uri": "artifact-private://hil-raw-v1/episode-7#t=8.2,10.4",
            "control_source": "teleop",
            "source_duration_s": 18.4,
            "source_intervention_time_range_s": [8.0, 11.0],
        }
        document["outcome"].update(
            duration_s=2.2,
            human_intervention_count=1,
            interventions=[{"start_s": 0, "end_s": 2.2, "control_source": "teleop", "reason": "복구"}],
        )
        self.assertEqual([], self.validate(document))

        document["training_provenance"]["source_split"] = "holdout_eval"
        self.assertTrue(any("원본과 파생물 split" in error for error in self.validate(document)))
        document["training_provenance"]["source_split"] = "train"
        document["training_provenance"]["recovery_time_range_s"] = [999, 1000]
        self.assertTrue(any("원본 개입 범위" in error for error in self.validate(document)))
        document["training_provenance"]["recovery_time_range_s"] = [8.2, 10.4]
        document["outcome"].update(human_intervention_count=0, interventions=[])
        self.assertTrue(any("개입 기록" in error for error in self.validate(document)))

        document["training_provenance"]["control_source"] = "policy"
        self.assertNotEqual([], self.validate(document))

    def test_policy_one_contract_requires_matching_stage_policy_and_both_arms(self) -> None:
        for mutation in (
            lambda doc: doc["task"].update(mission_stage="TABLE_POLICY_2"),
            lambda doc: doc["policy"].update(id="policy_2"),
            lambda doc: doc["arms"].update(used=["left"]),
            lambda doc: doc["arms"].update(mode="single"),
        ):
            document = copy.deepcopy(EPISODE_EXAMPLE)
            mutation(document)
            self.assertNotEqual([], self.validate(document))

    def test_policy_two_contract_requires_matching_stage_and_policy(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["task"].update(
            name="water_table_policy2",
            mission_stage="TABLE_POLICY_2",
            instruction="왼팔로 선반의 컵을 손님 테이블에 놓으세요",
        )
        document["policy"]["id"] = "policy_2"
        document["policy"]["phase_executions"] = [
            {
                "phase_id": "TABLE_PICK_PLACE",
                "backend": "TELEOP",
                "start_s": 0.0,
                "end_s": 18.4,
                "start_state_source": "episode_start",
            }
        ]
        document["arms"].update(mode="single", used=["left"])
        document["objects"].append({"role": "table", "id": "table_example", "start_zone": "table"})
        document.pop("water_measurement")
        self.assertEqual([], self.validate(document))
        document["arms"].update(used=["right"])
        self.assertEqual([], self.validate(document))
        document["arms"].update(mode="simultaneous", used=["left", "right"])
        self.assertNotEqual([], self.validate(document))
        document["arms"].update(mode="single", used=["left"])
        document["policy"]["id"] = "policy_1"
        self.assertNotEqual([], self.validate(document))

    def test_invalid_datetime_is_rejected(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["recorded_at"] = "yesterday"
        self.assertTrue(any("date-time" in error for error in self.validate(document)))

    def test_object_identity_roles_and_measurement_target_are_checked(self) -> None:
        for mutation in (
            lambda doc: doc["water_measurement"].update(cup_id="not-in-objects"),
            lambda doc: doc["water_measurement"].update(cup_id="jug_example_v1"),
            lambda doc: doc["objects"].append(copy.deepcopy(doc["objects"][0])),
            lambda doc: doc.update(objects=[{"role": "other", "id": "other", "start_zone": "bench"}]),
        ):
            document = copy.deepcopy(EPISODE_EXAMPLE)
            mutation(document)
            self.assertNotEqual([], self.validate(document))

    def test_target_fill_requires_measurement_evidence(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["water_measurement"]["fill_class"] = "TARGET"
        errors = self.validate(document)
        for field in (
            "tolerance_ml",
            "estimated_ml",
            "ground_truth_ml",
            "ground_truth_method",
        ):
            self.assertTrue(any(field in error for error in errors))

        document["water_measurement"].update(
            target_ml=200,
            tolerance_ml=10,
            estimated_ml=198,
            ground_truth_ml=201,
            ground_truth_method="scale",
        )
        self.assertEqual([], self.validate(document))

    def test_success_agrees_with_stage_results_and_fill_measurements(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["outcome"]["stage_results"]["cup_grasp"] = False
        self.assertTrue(any("전체 성공" in error for error in self.validate(document)))
        document["outcome"]["stage_results"]["cup_grasp"] = True
        document["water_measurement"].update(
            fill_class="TARGET", target_ml=200, tolerance_ml=10, estimated_ml=200,
            ground_truth_ml=0, ground_truth_method="scale", spill_detected=True,
        )
        self.assertTrue(any("실측량" in error for error in self.validate(document)))
        self.assertTrue(any("흘림" in error for error in self.validate(document)))
        # 비전 false positive는 실패로 보존할 수 있어야 한다.
        document["outcome"].update(success=False, failure_stage="pour", failure_code="UNDER_FILL",
                                   failure_record_id="FAIL-20260908-0001")
        document.update(include=False, exclude_reason="E1")
        self.assertEqual([], self.validate(document))
        document["water_measurement"]["estimated_ml"] = 0
        self.assertTrue(any("TARGET과 추정량" in error for error in self.validate(document)))

    def test_non_target_wet_run_cannot_claim_kitchen_success(self) -> None:
        for fill_class in ("UNDER", "OVER", "SPILL", "UNKNOWN"):
            document = copy.deepcopy(EPISODE_EXAMPLE)
            document["water_measurement"]["fill_class"] = fill_class
            self.assertTrue(any("주방 정책 성공" in error for error in self.validate(document)))

    def test_empty_exclusion_reason_and_wrong_intervention_duration_are_rejected(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document.update(include=False, exclude_reason="   ")
        self.assertTrue(any("제외 사유" in error for error in self.validate(document)))
        document.update(include=True)
        document.pop("exclude_reason")
        document["outcome"].update(
            human_intervention_count=1, human_intervention_duration_s=999,
            interventions=[{"start_s": 1, "end_s": 2, "control_source": "teleop", "reason": "복구"}],
        )
        self.assertTrue(any("구간 합" in error for error in self.validate(document)))
        document["outcome"]["human_intervention_duration_s"] = 1
        self.assertEqual([], self.validate(document))
        document["outcome"]["interventions"].append(
            {"start_s": 1.5, "end_s": 3, "control_source": "teleop", "reason": "중복 기록"}
        )
        document["outcome"].update(human_intervention_count=2, human_intervention_duration_s=2.5)
        self.assertTrue(any("겹침" in error for error in self.validate(document)))

    def test_schema_invalid_nested_values_do_not_crash_semantic_validation(self) -> None:
        for document in (
            [],
            {**copy.deepcopy(EPISODE_EXAMPLE), "outcome": None},
        ):
            self.assertNotEqual([], self.validate(document))

    def test_failure_record_reference_is_checked_when_directory_is_given(self) -> None:
        document = copy.deepcopy(EPISODE_EXAMPLE)
        document["outcome"]["failure_record_id"] = "FAIL-20260908-0001"
        with tempfile.TemporaryDirectory() as directory:
            unrelated = copy.deepcopy(FAILURE_EXAMPLE)
            unrelated["failure_id"] = "FAIL-20260908-0002"
            Path(directory, "unrelated.json").write_text(
                json.dumps(unrelated), encoding="utf-8"
            )
            errors = validate_episode_meta.validate_failure_references(
                [(Path("episode.json"), document)], Path(directory)
            )
        self.assertTrue(any("대응하는 실패 레코드가 없음" in error for error in errors))


class ValidatorCliTest(unittest.TestCase):
    def test_episode_failure_reference_cli_handles_invalid_input_and_minimal_mode(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            failures = root / "failures"
            failures.mkdir()
            (failures / "failure.json").write_text(json.dumps(FAILURE_EXAMPLE), encoding="utf-8")
            episode = root / "episode.json"
            document = copy.deepcopy(EPISODE_EXAMPLE)
            document["outcome"] = None
            episode.write_text(json.dumps(document), encoding="utf-8")
            command = [str(REPO_ROOT / "tools/validate_episode_meta.py"), str(episode),
                       "--failure-record-dir", str(failures)]
            result = subprocess.run([sys.executable, *command], capture_output=True, text=True)
            self.assertEqual(1, result.returncode, result.stdout + result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            episode.write_text(json.dumps(EPISODE_EXAMPLE), encoding="utf-8")
            result = subprocess.run([sys.executable, "-S", *command], capture_output=True, text=True)
            self.assertEqual(3, result.returncode, result.stdout + result.stderr)
            self.assertIn("참조 대조를 수행하지 못했습니다", result.stdout)
            self.assertNotIn("Traceback", result.stderr)

    def test_missing_dependency_and_null_fields_fail_without_traceback(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text('{"diagnosis":null,"validation":null,"data_use":null}', encoding="utf-8")
            result = subprocess.run(
                [sys.executable, "-S", str(REPO_ROOT / "tools/validate_failure_record.py"), str(path)],
                capture_output=True, text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertNotIn("Traceback", result.stderr)

    def run_validator(self, script: str, target: Path) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(REPO_ROOT / "tools" / script), str(target)],
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )

    def test_failure_directory_finds_nested_failure_json_only(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / "FAIL-20260908-0001"
            (bundle / "evidence").mkdir(parents=True)
            (bundle / "failure.json").write_text(
                json.dumps(FAILURE_EXAMPLE), encoding="utf-8"
            )
            (bundle / "evidence" / "metric.json").write_text("[]", encoding="utf-8")
            result = self.run_validator("validate_failure_record.py", root)
        self.assertEqual(0, result.returncode, result.stderr + result.stdout)

    def test_failure_directory_validates_flat_and_bundled_records_together(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / "FAIL-20260908-0001"
            bundle.mkdir()
            (bundle / "failure.json").write_text(
                json.dumps(FAILURE_EXAMPLE), encoding="utf-8"
            )
            (root / "bad.json").write_text("{}", encoding="utf-8")
            result = self.run_validator("validate_failure_record.py", root)
        self.assertEqual(1, result.returncode)
        self.assertIn("bad.json", result.stdout)

    def test_malformed_nonobject_and_empty_inputs_fail_cleanly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            malformed = root / "malformed.json"
            malformed.write_text("{", encoding="utf-8")
            nonobject = root / "nonobject.json"
            nonobject.write_text("[]", encoding="utf-8")
            nonstandard_number = root / "nan.json"
            nonstandard_number.write_text('{"value": NaN}', encoding="utf-8")
            empty = root / "empty"
            empty.mkdir()
            for script in ("validate_failure_record.py", "validate_episode_meta.py"):
                for target in (malformed, nonobject, nonstandard_number, empty):
                    result = self.run_validator(script, target)
                    self.assertNotEqual(0, result.returncode)
                    self.assertTrue(result.stderr.strip())


if __name__ == "__main__":
    unittest.main()
