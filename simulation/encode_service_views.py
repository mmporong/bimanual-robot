"""Encode synchronized simulator views; do not invent or interpolate liquid motion."""
import argparse
import json
from pathlib import Path
import subprocess


def detail_window(records, fps=2):
    if not records or [r["frame"] for r in records] != list(range(len(records))):
        raise ValueError("wide frame indices must be contiguous")
    if any(abs(r["time_s"] - r["frame"] / fps) > 1e-6 for r in records):
        raise ValueError("wide frame timestamps must match recording rate")
    detail = [r for r in records if "detail_frame" in r]
    if not detail or [r["detail_frame"] for r in detail] != list(range(len(detail))):
        raise ValueError("detail frame indices must be contiguous")
    if [r["frame"] for r in detail] != list(range(detail[0]["frame"], detail[-1]["frame"]+1)):
        raise ValueError("pour detail must be one continuous interval")
    return detail[0]["time_s"], detail[-1]["time_s"] + 1/fps, len(detail)


def encode(directory, output, speed):
    records = json.loads((directory/"camera_frames.json").read_text())
    start_s, end_s, count = detail_window(records)
    if output.exists():
        raise ValueError("refusing to overwrite existing video")
    for folder, total in (("frames", len(records)), ("detail_frames", count)):
        if any(not (directory/folder/f"{i:06d}.png").is_file() for i in range(total)):
            raise ValueError("recorded frame missing: "+folder)
    rate = 2*speed
    filters = (
        f"[1:v]scale=640:360,drawbox=x=0:y=0:w=iw:h=ih:color=white:t=3,"
        f"setpts=PTS+{start_s/speed}/TB[detail];"
        f"[0:v][detail]overlay=x=24:y=24:eof_action=pass:"
        f"enable='gte(t,{start_s/speed})*lt(t,{end_s/speed})'[out]"
    )
    subprocess.run(["ffmpeg", "-n", "-hide_banner", "-loglevel", "error",
                    "-framerate", str(rate), "-i", str(directory/"frames/%06d.png"),
                    "-framerate", str(rate), "-i", str(directory/"detail_frames/%06d.png"),
                    "-filter_complex", filters, "-map", "[out]", "-frames:v", str(len(records)),
                    "-c:v", "libx264", "-crf", "21", "-pix_fmt", "yuv420p", str(output)], check=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--speed", type=int, choices=(1, 4), default=1)
    args = parser.parse_args()
    encode(args.directory, args.output, args.speed)
