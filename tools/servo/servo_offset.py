#!/usr/bin/env python3
"""위치 보정 오프셋으로 "지금 이 자세"가 --target 으로 읽히게 만든다.

서보는 움직이지 않는다. 엔코더 눈금만 통째로 옮긴다. 그리퍼를 다시 물릴 때마다
닫힘값이 달라지므로, 재조립 뒤에는 이 스크립트 한 줄로 눈금을 다시 맞춘다.

오프셋의 부호 규약은 자료마다 다르다. 그래서 추정하지 않고 시험값을 한 번 써서
위치가 어느 쪽으로 움직이는지 보고 판정한다. 예상과 다르면 원래대로 돌리고 멈춘다.

    cd ~/bimanual-robot/tools/servo
    python3 servo_offset.py --target 3600
    python3 servo_offset.py --target 3600 --execute
"""
import argparse
import json
import time
from pathlib import Path

from sts_bus import (Bus, FakeBus, A_OFFSET, A_TORQUE, A_POS,
                     decode_offset, encode_offset, find_port)

PROBE = 100                    # 부호 판정용 시험값
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
    ap.add_argument("--target", type=int, required=True, help="지금 자세가 이 값으로 읽히게 한다")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--mock-sign", type=int, default=1, choices=(1, -1))
    args = ap.parse_args()

    sid = args.id
    bus = (FakeBus(sid=sid, pos=1721, offset=0, sign=args.mock_sign)
           if args.mock else Bus(find_port(args.port)))

    pos = bus.read(sid, A_POS, 2)
    if pos is None:
        print(f"ID {sid} 무응답. 중단.")
        return 1
    if bus.read(sid, A_TORQUE):
        print("구동이 걸려 있습니다. servo_release.py 로 푼 뒤 다시 하세요. 중단.")
        return 1

    raw = bus.read(sid, A_OFFSET, 2)
    current = decode_offset(raw)
    need = args.target - pos
    print(f"ID {sid}  현재 읽히는 위치 {pos}  현재 오프셋 {current} (raw {raw})")
    print(f"목표 {args.target}  ->  옮겨야 할 양 {need:+d}")
    if abs(need) > 2047:
        print("오프셋 한계 ±2047 을 넘습니다. 물린 톱니를 바꿔야 합니다. 중단.")
        return 1
    if not args.execute:
        print("dry-run 으로 끝났습니다. 실물 적용은 --execute 입니다.")
        return 0

    save_backup({"id": sid, "kind": "offset", "offset": current,
                 "offset_raw": raw, "pos": pos})

    def put(value):
        bus.write_eeprom(sid, A_OFFSET, encode_offset(value))
        return bus.read(sid, A_POS, 2)

    probed = put(current + PROBE)
    moved = probed - pos
    print(f"부호 시험: 오프셋 {current} -> {current + PROBE},  위치 {pos} -> {probed} ({moved:+d})")
    if abs(abs(moved) - PROBE) > 5:
        put(current)
        print(f"예상한 {PROBE} 만큼 안 움직였습니다. 원래대로 돌리고 중단합니다.")
        return 2
    sign = 1 if moved > 0 else -1
    print(f"판정: 오프셋을 키우면 위치가 {'커진다' if sign > 0 else '작아진다'}")

    final = current + sign * need
    if abs(final) > 2047:
        put(current)
        print(f"필요한 오프셋 {final} 이 한계를 넘습니다. 원래대로 돌리고 중단합니다.")
        return 1
    now = put(final)
    print(f"적용: 오프셋 {final}  위치 {now}  목표대비 {now - args.target:+d}")
    ok = abs(now - args.target) <= 3
    print("완료." if ok else "값이 목표와 다릅니다. 확인이 필요합니다.")
    bus.close()
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
