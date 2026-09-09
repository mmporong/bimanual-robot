#!/usr/bin/env python3
"""각도 한계를 바꾼다. EEPROM 쓰기라 서보는 움직이지 않는다.

한계는 하드 스톱에 밀어붙이는 것을 막는 마지막 방어선이다. 실측한 양 끝에서
안쪽으로 조금 물려서 잡는다.

    cd ~/bimanual-robot/tools/servo
    python3 servo_limits.py --min 760 --max 3600
    python3 servo_limits.py --min 760 --max 3600 --execute
"""
import argparse
import json
import time
from pathlib import Path

from sts_bus import (Bus, FakeBus, A_MIN_ANGLE, A_MAX_ANGLE, A_TORQUE, A_POS,
                     RESOLUTION, find_port)

BACKUP = Path.home() / ".cache" / "bimanual-robot" / "servo_backup.jsonl"


def save_backup(record):
    BACKUP.parent.mkdir(parents=True, exist_ok=True)
    record["saved"] = time.strftime("%F %T")
    with BACKUP.open("a") as fp:
        fp.write(json.dumps(record, ensure_ascii=False) + "\n")
    print(f"원래값 기록 -> {BACKUP}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--id", type=int, default=6)
    ap.add_argument("--min", type=int, required=True)
    ap.add_argument("--max", type=int, required=True)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()

    if not (0 <= args.min < args.max <= RESOLUTION - 1):
        print(f"한계값이 0 ~ {RESOLUTION - 1} 범위이고 min < max 여야 합니다. 중단.")
        return 1

    sid = args.id
    bus = FakeBus(sid=sid) if args.mock else Bus(find_port(args.port))

    pos = bus.read(sid, A_POS, 2)
    if pos is None:
        print(f"ID {sid} 무응답. 중단.")
        return 1
    lo, hi = bus.read(sid, A_MIN_ANGLE, 2), bus.read(sid, A_MAX_ANGLE, 2)
    print(f"ID {sid}  현재 위치 {pos}  구동 {bus.read(sid, A_TORQUE)}")
    print(f"현재 한계 {lo} ~ {hi}  ->  바꿀 값 {args.min} ~ {args.max}")
    if bus.read(sid, A_TORQUE):
        print("구동이 걸려 있습니다. servo_release.py 로 푼 뒤 다시 하세요. 중단.")
        return 1
    if not (args.min <= pos <= args.max):
        print(f"주의: 현재 위치 {pos} 가 새 한계 밖입니다. "
              f"구동을 켜면 한계 안으로 끌려 들어갑니다. 먼저 손으로 옮기세요.")
    if not args.execute:
        print("dry-run 으로 끝났습니다. 실물 적용은 --execute 입니다.")
        return 0

    save_backup({"id": sid, "kind": "limits", "min": lo, "max": hi, "pos": pos})

    bus.write_eeprom(sid, A_MIN_ANGLE, args.min)
    bus.write_eeprom(sid, A_MAX_ANGLE, args.max)

    new_lo, new_hi = bus.read(sid, A_MIN_ANGLE, 2), bus.read(sid, A_MAX_ANGLE, 2)
    new_pos = bus.read(sid, A_POS, 2)
    ok = (new_lo == args.min and new_hi == args.max)
    print(f"확인   한계 {new_lo} ~ {new_hi}   위치 {new_pos} (변화 {new_pos - pos:+d})")
    print("적용 완료." if ok else "적용 실패 — 값이 바뀌지 않았습니다. 잠금 해제를 확인하세요.")
    bus.close()
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
