#!/usr/bin/env python3
"""서보 EEPROM 에 들어 있는 오프셋·한계를 읽어 lerobot 캘리브레이션 JSON 으로 쓴다.

lerobot 은 연결할 때 서보의 Homing_Offset / Min / Max 를 JSON 과 비교해 하나라도
다르면 대화형 재캘리브레이션으로 서보를 덮어쓴다. 그래서 JSON 은 "지금 서보에 있는
값" 그대로여야 한다. 이 도구는 그 값을 읽어서 쓰는 것뿐이다. 서보는 움직이지 않는다.

    cd ~/bimanual-robot/tools/servo
    python3 export_lerobot_calibration.py --port <팔로워 by-id> --out <follower.json> --gripper-drive-mode 1
    python3 export_lerobot_calibration.py --port <리더 by-id>   --out <leader.json>
"""
import argparse
import json
import shutil
import time
from pathlib import Path

from sts_bus import Bus, A_MIN_ANGLE, A_MAX_ANGLE, A_OFFSET, decode_offset, find_port

SO101 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--out", required=True, help="쓸 JSON 경로. 있으면 .bak-<시각> 으로 백업")
    ap.add_argument("--gripper-drive-mode", type=int, default=0, choices=(0, 1),
                    help="그리퍼 방향이 리더와 반대면 1 (lerobot 이 100-값 으로 뒤집는다)")
    ap.add_argument("--execute", action="store_true", help="없으면 내용만 보여주고 쓰지 않는다")
    args = ap.parse_args()

    bus = Bus(find_port(args.port))
    calib = {}
    for sid, name in enumerate(SO101, start=1):
        lo, hi = bus.read(sid, A_MIN_ANGLE, 2), bus.read(sid, A_MAX_ANGLE, 2)
        raw = bus.read(sid, A_OFFSET, 2)
        if None in (lo, hi, raw):
            print(f"ID {sid} ({name}) 무응답. 중단.")
            return 1
        calib[name] = {
            "id": sid,
            "drive_mode": args.gripper_drive_mode if name == "gripper" else 0,
            "homing_offset": decode_offset(raw),
            "range_min": lo,
            "range_max": hi,
        }
    bus.close()

    print(f"{'motor':14s} {'id':>2} {'drive':>5} {'offset':>7} {'min':>5} {'max':>5}")
    for name, c in calib.items():
        print(f"{name:14s} {c['id']:>2} {c['drive_mode']:>5} {c['homing_offset']:>7} {c['range_min']:>5} {c['range_max']:>5}")

    out = Path(args.out).expanduser()
    if not args.execute:
        print(f"\ndry-run. --execute 를 주면 {out} 에 쓴다.")
        return 0
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        bak = out.with_name(out.name + ".bak-" + time.strftime("%Y%m%d-%H%M%S"))
        shutil.copy2(out, bak)
        print(f"\n기존 파일 백업 -> {bak}")
    out.write_text(json.dumps(calib, indent=4) + "\n")
    print(f"저장 -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
