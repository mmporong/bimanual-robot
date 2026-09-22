from simulate_restaurant_mobile import record_failure
import simulate_restaurant_mobile as runner


def test_recording_exception_invalidates_previously_successful_task():
    result = {"task_pass": True, "negative_control_pass": True, "failure": "served"}
    record_failure(result, TimeoutError("viewport capture timed out"))
    assert result["task_pass"] is False
    assert result["negative_control_pass"] is False
    assert result["failure"] == "TimeoutError: viewport capture timed out"


def test_early_exception_preserves_diagnostics():
    result = {"task_pass": False, "samples": [{"time_s": 2.0}]}
    record_failure(result, ValueError("invalid observation"))
    assert result["samples"] == [{"time_s": 2.0}]
    assert not result["task_pass"]


def test_exit_status_uses_post_cleanup_outcome(monkeypatch):
    def execute(args, result):
        result.update(task_pass=True)
        try:
            return 0
        finally:
            result.update(task_pass=False, failure='EVIDENCE_WRITE_FAILED')
    monkeypatch.setattr(runner, '_run', execute)
    assert runner.run(None) == 1
