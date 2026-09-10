#!/usr/bin/env python3
"""lerobot 캘리브레이션 JSON 과 서보 EEPROM 이 일치하는지, 범위가 멀쩡한지 읽기 전용으로 본다.

lerobot 은 연결할 때 서보의 오프셋·하한·상한을 JSON 과 비교해 하나라도 다르면 대화형
재캘리브레이션에 들어간다. 텔레옵·녹화 전에 이걸로 먼저 확인한다.

    cd ~/bimanual-robot/tools/servo
    python3 servo_check_calibration.py --port <팔로워 by-id> --json ~/.cache/huggingface/lerobot/calibration/robots/so_follower/follower.json
"""
import argparse
import json
from pathlib import Path

from sts_bus import Bus, A_MIN_ANGLE, A_MAX_ANGLE, A_OFFSET, A_POS, RESOLUTION, decode_offset, find_port

FULL_TURN = {"wrist_roll"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--json", required=True)
    args = ap.parse_args()

    calib = json.loads(Path(args.json).expanduser().read_text())
    bus = Bus(find_port(args.port))
    ok = True
    print(f"{'motor':14s} {'drive':>5} | {'JSON off':>8} {'min':>5} {'max':>5} | {'서보 off':>8} {'min':>5} {'max':>5} | {'위치':>5}  판정")
    for name, c in calib.items():
        sid = c["id"]
        s_off = bus.read(sid, A_OFFSET, 2)
        s_min, s_max, pos = bus.read(sid, A_MIN_ANGLE, 2), bus.read(sid, A_MAX_ANGLE, 2), bus.read(sid, A_POS, 2)
        if None in (s_off, s_min, s_max, pos):
            print(f"{name:14s} 무응답"); ok = False; continue
        s_off = decode_offset(s_off)
        notes = []
        if (c["homing_offset"], c["range_min"], c["range_max"]) != (s_off, s_min, s_max):
            notes.append("JSON≠서보"); ok = False
        span = c["range_max"] - c["range_min"]
        if name not in FULL_TURN:
            if c["range_min"] < 5 or c["range_max"] > RESOLUTION - 5:
                notes.append("0~4095 오염"); ok = False
            elif span < 300:
                notes.append("폭 작음"); ok = False
            if not (c["range_min"] <= pos <= c["range_max"]):
                notes.append("지금 위치가 범위 밖")
        print(f"{name:14s} {c['drive_mode']:>5} | {c['homing_offset']:>8} {c['range_min']:>5} {c['range_max']:>5} | "
              f"{s_off:>8} {s_min:>5} {s_max:>5} | {pos:>5}  {'OK' if not notes else ', '.join(notes)}")
    bus.close()
    print("\n결과:", "일치 — 연결해도 재캘리브레이션에 안 들어간다" if ok else "문제 있음 — 위 판정 참고")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
