import json

import pytest

from workcell_recording import PreviewRecording


def read_result(session: PreviewRecording):
    return json.loads(session.result_path.read_text(encoding="utf-8"))


def test_start_immediately_preserves_incomplete_result(tmp_path):
    session = PreviewRecording(tmp_path)

    result = read_result(session)
    assert result["completed"] is False
    assert result["reason"] == "recording_in_progress"
    assert result["frames"] == []
    assert session.frame_path().name == "frame_0000.png"


def test_interrupted_run_is_preserved_and_restart_is_isolated(tmp_path):
    first = PreviewRecording(tmp_path)
    first.add_frame({"time_s": 0.0, "phase": "RESET"})
    first.finish(completed=False, reason="user_reset")

    second = PreviewRecording(tmp_path)
    assert first.run_dir != second.run_dir
    assert second.frame_path().name == "frame_0000.png"
    assert read_result(first)["reason"] == "user_reset"
    assert len(read_result(first)["frames"]) == 1
    assert read_result(second)["frames"] == []


def test_completed_run_records_reset_error(tmp_path):
    session = PreviewRecording(tmp_path, frame_rate=20)
    session.add_frame({"time_s": 0.0, "phase": "HORIZONTAL_READY"})
    session.finish(completed=True, reason="demo_completed", final_reset_error_rad=0.0)

    result = read_result(session)
    assert result["completed"] is True
    assert result["reason"] == "demo_completed"
    assert result["frame_rate"] == 20
    assert result["final_reset_error_rad"] == 0.0
    with pytest.raises(RuntimeError):
        session.frame_path()


def test_finish_is_idempotent_and_keeps_first_terminal_reason(tmp_path):
    session = PreviewRecording(tmp_path)
    session.finish(completed=False, reason="exception:RuntimeError")
    session.finish(completed=True, reason="demo_completed")

    result = read_result(session)
    assert result["completed"] is False
    assert result["reason"] == "exception:RuntimeError"


def test_terminal_reason_is_required(tmp_path):
    session = PreviewRecording(tmp_path)
    with pytest.raises(ValueError):
        session.finish(completed=False, reason="")
    assert read_result(session)["reason"] == "recording_in_progress"


@pytest.mark.parametrize("frame_rate", [0, -1])
def test_invalid_frame_rate_is_rejected(tmp_path, frame_rate):
    with pytest.raises(ValueError):
        PreviewRecording(tmp_path, frame_rate=frame_rate)
