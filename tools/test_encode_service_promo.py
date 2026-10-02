import json
from pathlib import Path
import shutil
import subprocess

import pytest

import encode_service_promo as promo
from encode_service_promo import FPS, edited_shots, encode, validate_inputs, validate_records


def records(count=24):
    output = []
    time_s = 0.0
    for frame in range(count):
        speed = 4 if frame < count // 2 else 2
        if frame:
            previous_speed = 4 if frame - 1 < count // 2 else 2
            # At a cut the simulation timestamp only has to remain increasing.
            time_s += (speed if speed == previous_speed else 1) / FPS
        output.append({
            "frame": frame,
            "time_s": time_s,
            "shot_id": "approach" if frame < count // 2 else "serve",
            "title": "손님에게 이동" if frame < count // 2 else "안정적으로 서빙",
            "subtitle": "서비스 위치로 접근합니다" if frame < count // 2 else "두 팔로 작업을 마칩니다",
            "speed": speed,
            "view": {
                "camera": "wide" if frame < count // 2 else "detail",
                "eye_m": [frame / 100, 0.0, 1.2],
                "focal_length_mm": 35 + frame / 10,
            },
        })
    return output


def test_timeline_rejects_nonfinite_noncontiguous_and_returning_shots():
    invalid = records()
    invalid[3]["time_s"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        validate_records(invalid)

    invalid = records()
    invalid[3]["speed"] = float("inf")
    with pytest.raises(ValueError, match="speed must be finite"):
        validate_records(invalid)

    invalid = records()
    invalid[3]["view"]["eye_m"] = [0.0, float("nan"), 1.0]
    with pytest.raises(ValueError, match="view metadata numbers must be finite"):
        validate_records(invalid)

    invalid = records()
    invalid[3]["frame"] = 8
    with pytest.raises(ValueError, match="contiguous"):
        validate_records(invalid)

    invalid = records()
    invalid[3]["time_s"] = invalid[2]["time_s"] + 1 / FPS
    with pytest.raises(ValueError, match="speed/fps"):
        validate_records(invalid)

    invalid = records()
    invalid[-1]["shot_id"] = "approach"
    invalid[-1]["title"] = invalid[0]["title"]
    invalid[-1]["subtitle"] = invalid[0]["subtitle"]
    invalid[-1]["speed"] = invalid[0]["speed"]
    invalid[-1]["view"] = invalid[0]["view"]
    with pytest.raises(ValueError, match="contiguous interval"):
        validate_records(invalid)


def test_trim_does_not_mislabel_discarded_camera_as_output_camera():
    source = records()
    shots, dropped = edited_shots(validate_records(source), 2)
    assert dropped['approach'] == [0, 1]
    assert 'view' not in shots[0]
    assert shots[0]['source_first_view'] == source[0]['view']
    assert shots[0]['source_first_view'] != source[2]['view']
    assert shots[0]['source_kept_start_frame'] == 2


def _make_run(tmp_path: Path, frame_count=24) -> Path:
    run = tmp_path / "run"
    run.mkdir()
    subprocess.run(
        [
            "ffmpeg", "-n", "-hide_banner", "-loglevel", "error", "-f", "lavfi",
            "-i", f"testsrc2=size=1920x1080:rate={FPS}", "-frames:v", str(frame_count),
            "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p",
            str(run / "cinematic_raw.mp4"),
        ],
        check=True,
    )
    (run / "cinematic_frames.json").write_text(
        json.dumps(records(frame_count), ensure_ascii=False), encoding="utf-8"
    )
    (run / "result.json").write_text('{"task_pass": true}\n', encoding="utf-8")
    return run


@pytest.mark.skipif(not shutil.which("ffmpeg") or not shutil.which("ffprobe"), reason="ffmpeg unavailable")
def test_encode_real_ffmpeg_preserves_frames_and_writes_audit_manifest(tmp_path):
    run = _make_run(tmp_path)
    output = tmp_path / "promo.mp4"
    manifest_path = encode(run, output, music=True)

    assert output.is_file() and output.stat().st_size > 0
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["source_count"] == 24
    assert manifest["output_count"] == 20
    assert manifest["output"]["frame_count"] == 20
    assert manifest["dropped_source_frames"] == {
        "approach": [0, 1],
        "serve": [12, 13],
    }
    assert len(manifest["source"]["sha256"]) == 64
    assert len(manifest["metadata"]["sha256"]) == 64
    assert manifest["encoding"]["original_generated_music"] is True
    assert manifest["encoding"]["real_audio_false"] is True
    assert manifest["encoding"]["frame_interpolation"] is False
    assert manifest["encoding"]["retiming"] is False
    assert "minterpolate" not in manifest["encoding"]["filter_complex"]
    assert "split=" not in manifest["encoding"]["filter_complex"]
    assert "concat=" not in manifest["encoding"]["filter_complex"]
    assert "4× SPEED" in manifest["encoding"]["filter_complex"] or manifest["shots"][0]["speed"] == 4

    with pytest.raises(ValueError, match="overwrite"):
        encode(run, output)

    untrimmed = tmp_path / "promo-untrimmed.mp4"
    untrimmed_manifest = json.loads(encode(
        run, untrimmed, music=False, trim_shot_head_frames=0
    ).read_text(encoding="utf-8"))
    assert untrimmed_manifest["source_count"] == 24
    assert untrimmed_manifest["output_count"] == 24
    assert untrimmed_manifest["dropped_source_frames"] == {"approach": [], "serve": []}


@pytest.mark.skipif(not shutil.which("ffmpeg"), reason="ffmpeg unavailable")
def test_caption_background_is_visible_on_bright_footage(tmp_path):
    shots, _ = edited_shots(validate_records(records(96)), 0)
    graph = promo.build_filter(shots, 96, tmp_path)
    rendered = subprocess.run([
        "ffmpeg", "-v", "error", "-f", "lavfi", "-i",
        "color=white:size=1920x1080:rate=24:duration=4",
        "-filter_complex", graph, "-map", "[outv]", "-ss", "1",
        "-frames:v", "1", "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1",
    ], check=True, capture_output=True).stdout

    def pixel(x, y):
        offset = (y * 1920 + x) * 3
        return rendered[offset:offset + 3]

    assert len(rendered) == 1920 * 1080 * 3
    assert max(pixel(820, 990)) < 100  # Dark caption panel, below the text.
    assert min(pixel(900, 990)) > 240  # White footage outside the panel.
    red, green, blue = pixel(200, 1058)
    assert green > red + 60 and blue > red + 50  # Visible teal progress bar.


def test_reviewed_head_cuts_keep_complete_source_frame_mapping():
    shots = validate_records(records(24))
    kept, dropped = edited_shots(shots, 2, {'approach': 12, 'serve': 5})
    assert len(kept) == 1
    assert kept[0]['source_kept_start_frame'] == 17
    assert kept[0]['start_frame'] == 0 and kept[0]['end_frame'] == 7
    assert dropped == {'approach': list(range(12)), 'serve': list(range(12, 17))}
    for invalid in ({'missing': 2}, {'serve': -1}, {'serve': True}, {'serve': 13},
                    {'approach': 12, 'serve': 12}, ['not-a-map']):
        with pytest.raises(ValueError):
            edited_shots(shots, 2, invalid)


def test_trim_rejects_negative_or_shots_without_a_remaining_frame():
    shots = validate_records(records(4))
    with pytest.raises(ValueError, match="non-negative"):
        edited_shots(shots, -1)
    with pytest.raises(ValueError, match="too short"):
        edited_shots(shots, 2)


@pytest.mark.skipif(not shutil.which("ffmpeg") or not shutil.which("ffprobe"), reason="ffmpeg unavailable")
def test_validation_rejects_frame_count_and_failed_result(tmp_path):
    run = _make_run(tmp_path, frame_count=10)
    bad_metadata = records(12)
    (run / "cinematic_frames.json").write_text(json.dumps(bad_metadata), encoding="utf-8")
    with pytest.raises(ValueError, match="frame count"):
        validate_inputs(run)

    (run / "result.json").write_text('{"task_pass": false}\n', encoding="utf-8")
    with pytest.raises(ValueError, match="task_pass"):
        validate_inputs(run)


@pytest.mark.skipif(not shutil.which("ffmpeg") or not shutil.which("ffprobe"), reason="ffmpeg unavailable")
def test_validation_rejects_wrong_fps_and_duration(tmp_path, monkeypatch):
    run = _make_run(tmp_path)
    actual = promo._probe(run / "cinematic_raw.mp4")

    wrong_fps = {**actual, "avg_frame_rate": "25/1"}
    monkeypatch.setattr(promo, "_probe", lambda _path: wrong_fps)
    with pytest.raises(ValueError, match="24 fps"):
        validate_inputs(run)

    wrong_duration = {**actual, "format_duration": "9.0"}
    monkeypatch.setattr(promo, "_probe", lambda _path: wrong_duration)
    with pytest.raises(ValueError, match="duration"):
        validate_inputs(run)
