"""Encode the recorded service simulation as a polished HOLD FLOW promo video.

The encoder never retimes kept footage or synthesizes/interpolates frames.  Shot
speed is already baked into ``cinematic_raw.mp4`` and is only disclosed as an overlay.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import wave

import numpy as np


FPS = 24
WIDTH = 1920
HEIGHT = 1080
FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _probe(path: Path) -> dict:
    result = subprocess.run(
        [
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-count_frames", "-show_entries",
            "stream=width,height,r_frame_rate,avg_frame_rate,nb_read_frames,duration",
            "-show_entries", "format=duration", "-of", "json", str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)
    if not payload.get("streams"):
        raise ValueError("video stream is missing")
    return {**payload["streams"][0], "format_duration": payload["format"].get("duration")}


def _rate(value: str) -> float:
    numerator, denominator = value.split("/", 1)
    return float(numerator) / float(denominator)


def _load_records(path: Path) -> list[dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    records = payload.get("frames") if isinstance(payload, dict) else payload
    if not isinstance(records, list) or not records:
        raise ValueError("cinematic_frames.json must contain a non-empty frame list")
    return records


def _all_numbers_finite(value: object) -> bool:
    if isinstance(value, bool):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_all_numbers_finite(item) for item in value.values())
    if isinstance(value, list):
        return all(_all_numbers_finite(item) for item in value)
    return True


def validate_records(records: list[dict], fps: int = FPS) -> list[dict]:
    """Validate the one-record-per-source-frame contract and return shot spans."""
    required = {"frame", "time_s", "shot_id", "title", "subtitle", "speed", "view"}
    seen_closed: set[str] = set()
    shots: list[dict] = []
    previous_shot: str | None = None
    previous_time: float | None = None
    for index, record in enumerate(records):
        if not isinstance(record, dict) or not required.issubset(record):
            raise ValueError(f"frame {index} is missing required metadata")
        if isinstance(record["frame"], bool) or record["frame"] != index:
            raise ValueError("frame indices must be contiguous from zero")
        time_s = record["time_s"]
        speed = record["speed"]
        if not isinstance(time_s, (int, float)) or not math.isfinite(time_s):
            raise ValueError("frame timestamps must be finite")
        if not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed <= 0:
            raise ValueError("shot speed must be finite and positive")
        if not isinstance(record["view"], dict):
            raise ValueError("view metadata must be a dictionary")
        if not _all_numbers_finite(record["view"]):
            raise ValueError("view metadata numbers must be finite")
        if not all(isinstance(record[key], str) and record[key].strip()
                   for key in ("shot_id", "title", "subtitle")):
            raise ValueError("shot id, title, and subtitle must be non-empty strings")
        shot_id = record["shot_id"]
        if previous_time is not None and float(time_s) <= previous_time:
            raise ValueError("simulation timestamps must be strictly increasing")
        if shot_id == previous_shot:
            expected_delta = float(speed) / fps
            if abs((float(time_s) - previous_time) - expected_delta) > 1 / 120 + 1e-9:
                raise ValueError("timestamps inside a shot must follow speed/fps")
        if shot_id != previous_shot:
            if shot_id in seen_closed:
                raise ValueError("each shot_id must occupy one contiguous interval")
            if previous_shot is not None:
                seen_closed.add(previous_shot)
                shots[-1]["end_frame"] = index
            shots.append({
                "shot_id": shot_id,
                "start_frame": index,
                "end_frame": len(records),
                "title": record["title"],
                "subtitle": record["subtitle"],
                "speed": float(speed),
                "source_first_view": record["view"],
            })
            previous_shot = shot_id
        else:
            current = shots[-1]
            for key in ("title", "subtitle"):
                if record[key] != current[key]:
                    raise ValueError(f"{key} must remain constant inside a shot")
            if float(speed) != current["speed"]:
                raise ValueError("speed must remain constant inside a shot")
        previous_time = float(time_s)
    return shots


def validate_inputs(directory: Path) -> tuple[Path, Path, Path, list[dict], list[dict], dict]:
    source = directory / "cinematic_raw.mp4"
    metadata = directory / "cinematic_frames.json"
    result = directory / "result.json"
    for path in (source, metadata, result):
        if not path.is_file():
            raise ValueError(f"required input is missing: {path.name}")
    result_data = json.loads(result.read_text(encoding="utf-8"))
    if not isinstance(result_data, dict) or result_data.get("task_pass") is not True:
        raise ValueError("result.json must contain task_pass=true")
    records = _load_records(metadata)
    shots = validate_records(records)
    probe = _probe(source)
    if (probe.get("width"), probe.get("height")) != (WIDTH, HEIGHT):
        raise ValueError(f"source video must be {WIDTH}x{HEIGHT}")
    if abs(_rate(probe["avg_frame_rate"]) - FPS) > 1e-6:
        raise ValueError(f"source video must be {FPS} fps")
    if int(probe.get("nb_read_frames", -1)) != len(records):
        raise ValueError("source frame count must match cinematic_frames.json")
    duration = float(probe.get("format_duration") or probe.get("duration"))
    if not math.isfinite(duration) or abs(duration - len(records) / FPS) > 1 / FPS + 1e-3:
        raise ValueError("source duration must match its contiguous metadata timeline")
    return source, metadata, result, records, shots, probe


def _font(path: Path) -> str:
    if not path.is_file():
        raise RuntimeError(f"Noto CJK font is unavailable: {path}")
    return str(path).replace("\\", "\\\\").replace(":", "\\:")


def _textfile(path: Path) -> str:
    return str(path).replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")


def _speed_label(speed: float) -> str:
    value = int(speed) if speed.is_integer() else speed
    return f"{value}× SPEED"


def edited_shots(shots: list[dict], trim_shot_head_frames: int,
                 head_cuts: dict[str, int] | None = None) -> tuple[list[dict], dict[str, list[int]]]:
    if (isinstance(trim_shot_head_frames, bool)
            or not isinstance(trim_shot_head_frames, int)
            or trim_shot_head_frames < 0):
        raise ValueError("trim-shot-head-frames must be a non-negative integer")
    output_start = 0
    edited = []
    dropped = {}
    head_cuts = {} if head_cuts is None else head_cuts
    if not isinstance(head_cuts, dict) or set(head_cuts) - {s['shot_id'] for s in shots}:
        raise ValueError('head cuts must map known shot IDs to frame counts')
    if any(isinstance(n, bool) or not isinstance(n, int) or n < 0 for n in head_cuts.values()):
        raise ValueError('head cuts require non-negative integer frame counts')
    for shot in shots:
        source_start = shot["start_frame"]
        source_end = shot["end_frame"]
        trim = head_cuts.get(shot['shot_id'], trim_shot_head_frames)
        if trim > source_end - source_start:
            raise ValueError('head cut exceeds shot length')
        if shot['shot_id'] in head_cuts and trim == source_end - source_start:
            dropped[shot['shot_id']] = list(range(source_start, source_end))
            continue
        if source_end - source_start <= trim:
            raise ValueError("shot is too short for trim-shot-head-frames")
        kept_count = source_end - source_start - trim
        item = {
            **shot,
            "source_start_frame": source_start,
            "source_kept_start_frame": source_start + trim,
            "source_end_frame": source_end,
            "start_frame": output_start,
            "end_frame": output_start + kept_count,
        }
        edited.append(item)
        dropped[shot["shot_id"]] = list(range(source_start, source_start + trim))
        output_start += kept_count
    if not edited:
        raise ValueError('head cuts removed every shot')
    return edited, dropped


def build_filter(shots: list[dict], frame_count: int, caption_dir: Path) -> str:
    """Build a filter graph with one input frame producing one output frame."""
    regular = _font(FONT_REGULAR)
    bold = _font(FONT_BOLD)
    duration = frame_count / FPS
    caption_paths: list[dict[str, Path]] = []
    global_text = caption_dir / "simulation.txt"
    global_text.write_text("SIMULATION  /  시뮬레이션", encoding="utf-8")
    brand_text = caption_dir / "brand.txt"
    brand_text.write_text("HOLD FLOW", encoding="utf-8")
    for index, shot in enumerate(shots):
        group = {}
        for key, value in (
            ("title", shot["title"]),
            ("subtitle", shot["subtitle"]),
            ("speed", _speed_label(shot["speed"])),
            ("step", f"STEP {index + 1:02d}  /  {len(shots):02d}"),
        ):
            path = caption_dir / f"shot_{index:02d}_{key}.txt"
            path.write_text(value, encoding="utf-8")
            group[key] = path
        caption_paths.append(group)

    # The recorded camera already contains its cuts, dolly, tracking, and optical
    # zoom.  Keep one streaming branch so a minute-scale 1080p input stays bounded
    # in memory and every source frame remains in original order.
    selection = "+".join(
        f"between(n,{shot['source_kept_start_frame']},{shot['source_end_frame'] - 1})"
        for shot in shots
    )
    parts = [
        f"[0:v]select='{selection}',setpts=N/({FPS}*TB),"
        "scale=1778:1000:flags=lanczos,"
        f"pad={WIDTH}:{HEIGHT}:71:40:color=0x05080D,setsar=1[cut];"
    ]
    fade = min(0.45, duration / 3)
    fade_out = max(0.0, duration - fade)
    parts.append(
        f"[cut]fade=t=in:st=0:d={fade:.6f},fade=t=out:st={fade_out:.6f}:d={fade:.6f},"
        "drawbox=x=0:y=0:w=iw:h=40:color=0x05080D@0.96:t=fill,"
        "drawbox=x=0:y=ih-40:w=iw:h=40:color=0x05080D@0.96:t=fill,"
        f"drawtext=fontfile='{bold}':textfile='{_textfile(brand_text)}':"
        "fontsize=22:fontcolor=0xF4F7FA:x=70:y=9,"
        f"drawtext=fontfile='{regular}':textfile='{_textfile(global_text)}':"
        "fontsize=19:fontcolor=0xAAB7C4:x=(w-text_w)/2:y=10[base];"
    )
    label = "base"
    for index, shot in enumerate(shots):
        start_frame = shot["start_frame"]
        end_frame = shot["end_frame"]
        title_end_frame = min(end_frame, start_frame + round(2.8 * FPS))
        enable = f"between(n,{start_frame},{end_frame - 1})"
        title_enable = f"between(n,{start_frame},{title_end_frame - 1})"
        captions = caption_paths[index]
        next_label = f"overlay{index}"
        progress_width = round(1640 * (index + 1) / len(shots))
        parts.append(
            f"[{label}]drawbox=x=70:y=ih-194:w=780:h=118:color=0x05080D@0.72:t=fill:enable='{title_enable}',"
            f"drawbox=x=140:y=ih-23:w=1640:h=3:color=0x51606E@0.65:t=fill:enable='{enable}',"
            f"drawbox=x=140:y=ih-23:w={progress_width}:h=3:color=0x3DE0C5@0.95:t=fill:enable='{enable}',"
            f"drawtext=fontfile='{bold}':textfile='{_textfile(captions['title'])}':"
            f"fontsize=40:fontcolor=white:x=96:y=h-180:enable='{title_enable}',"
            f"drawtext=fontfile='{regular}':textfile='{_textfile(captions['subtitle'])}':"
            f"fontsize=24:fontcolor=0xC8D2DC:x=98:y=h-126:enable='{title_enable}',"
            f"drawtext=fontfile='{bold}':textfile='{_textfile(captions['speed'])}':"
            f"fontsize=20:fontcolor=0x3DE0C5:x=w-text_w-70:y=9:enable='{enable}',"
            f"drawtext=fontfile='{regular}':textfile='{_textfile(captions['step'])}':"
            f"fontsize=18:fontcolor=0xC8D2DC:x=w-text_w-70:y=h-34:enable='{enable}'[{next_label}];"
        )
        label = next_label
    parts.append(f"[{label}]format=yuv420p[outv]")
    return "".join(parts)


def generate_music(path: Path, duration_s: float, sample_rate: int = 48_000) -> None:
    """Create a deterministic, original low-volume ambient instrumental bed."""
    sample_count = max(1, math.ceil(duration_s * sample_rate))
    with wave.open(str(path), "wb") as output:
        output.setnchannels(2)
        output.setsampwidth(2)
        output.setframerate(sample_rate)
        # Stream chunks so a minute-scale render does not require hundreds of MB
        # of temporary oscillator arrays.
        for start in range(0, sample_count, 65_536):
            stop = min(start + 65_536, sample_count)
            t = np.arange(start, stop, dtype=np.float64) / sample_rate
            envelope = np.minimum(1.0, t / 1.5) * np.minimum(1.0, (duration_s - t) / 1.8)
            envelope = np.clip(envelope, 0.0, 1.0)
            pulse = 0.74 + 0.26 * np.sin(2 * np.pi * 0.10 * t)
            left = np.zeros_like(t)
            right = np.zeros_like(t)
            for frequency, phase in (
                (110.0, 0.0), (164.81, 0.4), (220.0, 1.1), (329.63, 0.7)
            ):
                left += np.sin(2 * np.pi * frequency * t + phase)
            for frequency, phase in (
                (110.0, 0.2), (164.81, 0.8), (246.94, 0.1), (329.63, 1.3)
            ):
                right += np.sin(2 * np.pi * frequency * t + phase)
            stereo = np.column_stack((left, right)) * envelope[:, None] * pulse[:, None] * 0.035
            pcm = np.clip(stereo * 32767, -32768, 32767).astype("<i2")
            output.writeframesraw(pcm.tobytes())


def encode(
    directory: Path,
    output: Path,
    music: bool = True,
    trim_shot_head_frames: int = 2,
    head_cuts: dict[str, int] | None = None,
) -> Path:
    directory = directory.resolve()
    output = output.resolve()
    manifest = output.with_suffix(output.suffix + ".manifest.json")
    if output.exists() or manifest.exists():
        raise ValueError("refusing to overwrite existing output or manifest")
    if output.suffix.lower() != ".mp4":
        raise ValueError("output must use the .mp4 extension")
    if not output.parent.is_dir():
        raise ValueError("output parent directory must already exist")
    if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
        raise RuntimeError("ffmpeg and ffprobe are required")
    source, metadata, result, records, source_shots, source_probe = validate_inputs(directory)
    shots, dropped_source_frames = edited_shots(source_shots, trim_shot_head_frames, head_cuts)
    output_frame_count = sum(shot["end_frame"] - shot["start_frame"] for shot in shots)
    duration = output_frame_count / FPS

    with tempfile.TemporaryDirectory(prefix="hold-flow-promo-", dir=output.parent) as temp_name:
        temp_dir = Path(temp_name)
        temp_video = temp_dir / "encoded.mp4"
        filter_script = temp_dir / "filter.ffscript"
        filter_graph = build_filter(shots, output_frame_count, temp_dir)
        filter_script.write_text(filter_graph, encoding="utf-8")
        command = [
            "ffmpeg", "-n", "-hide_banner", "-loglevel", "error", "-i", str(source)
        ]
        music_path = temp_dir / "original_ambient.wav"
        if music:
            generate_music(music_path, duration)
            command += ["-i", str(music_path)]
        command += ["-filter_complex_script", str(filter_script), "-map", "[outv]"]
        if music:
            audio_fade = min(1.2, duration / 3)
            command += [
                "-map", "1:a:0", "-af",
                f"volume=0.52,afade=t=in:st=0:d={audio_fade:.6f},"
                f"afade=t=out:st={max(0.0, duration-audio_fade):.6f}:d={audio_fade:.6f}",
                "-c:a", "aac", "-b:a", "160k", "-shortest",
            ]
        else:
            command += ["-an"]
        command += [
            "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p",
            "-r", str(FPS), "-frames:v", str(output_frame_count),
            "-movflags", "+faststart", str(temp_video),
        ]
        subprocess.run(command, check=True)
        output_probe = _probe(temp_video)
        if int(output_probe.get("nb_read_frames", -1)) != output_frame_count:
            raise RuntimeError("encoded output frame count does not match the cut list")
        if (output_probe.get("width"), output_probe.get("height")) != (WIDTH, HEIGHT):
            raise RuntimeError("encoded output changed the required dimensions")
        if abs(_rate(output_probe["avg_frame_rate"]) - FPS) > 1e-6:
            raise RuntimeError("encoded output changed the required frame rate")
        encoded_duration = float(output_probe.get("format_duration") or output_probe.get("duration"))
        if not math.isfinite(encoded_duration) or abs(encoded_duration - duration) > 1 / FPS + 1e-3:
            raise RuntimeError("encoded output duration does not match the cut list")
        os.replace(temp_video, output)

    command_for_manifest = [
        "<filter-script>" if item == str(filter_script) else
        "<generated-original-ambient.wav>" if item == str(music_path) else
        str(output) if item == str(temp_video) else item
        for item in command
    ]
    manifest_payload = {
        "schema_version": 1,
        "encoder": {"path": str(Path(__file__).resolve()), "sha256": _sha256(Path(__file__))},
        "source": {"path": str(source), "sha256": _sha256(source), "probe": source_probe},
        "metadata": {"path": str(metadata), "sha256": _sha256(metadata)},
        "simulation_result": {"path": str(result), "sha256": _sha256(result), "task_pass": True},
        "output": {"path": str(output), "sha256": _sha256(output), "frame_count": output_frame_count},
        "source_count": len(records),
        "output_count": output_frame_count,
        "dropped_source_frames": dropped_source_frames,
        "audio": {"original_generated_music": music, "real_audio_false": True},
        "encoding": {
            "fps": FPS,
            "dimensions": [WIDTH, HEIGHT],
            "frame_interpolation": False,
            "retiming": False,
            "timing_retime": False,
            "cut_deletion": any(dropped_source_frames.values()),
            "trim_shot_head_frames": trim_shot_head_frames,
            "head_cuts": head_cuts or {},
            "original_generated_music": music,
            "real_audio": False,
            "real_audio_false": True,
            "command": command_for_manifest,
            "filter_complex": filter_graph,
        },
        "shots": shots,
    }
    temp_manifest = manifest.with_name(manifest.name + ".tmp")
    temp_manifest.write_text(
        json.dumps(manifest_payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    os.replace(temp_manifest, manifest)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="run directory containing cinematic inputs")
    parser.add_argument("output", type=Path, help="new MP4 output path")
    parser.add_argument(
        "--music", action=argparse.BooleanOptionalAction, default=True,
        help="include an original generated ambient instrumental (default: enabled)",
    )
    parser.add_argument(
        "--trim-shot-head-frames", type=int, default=2, metavar="N",
        help="drop N render-lag frames at every shot head (default: 2; use 0 to disable)",
    )
    parser.add_argument(
        '--head-cuts', type=Path,
        help='JSON mapping shot IDs to total head frames to remove; full length omits a shot',
    )
    arguments = parser.parse_args()
    encode(
        arguments.directory,
        arguments.output,
        arguments.music,
        arguments.trim_shot_head_frames,
        json.loads(arguments.head_cuts.read_text(encoding='utf-8')) if arguments.head_cuts else None,
    )


if __name__ == "__main__":
    main()
