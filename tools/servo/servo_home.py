#!/usr/bin/env python3
"""지금 자세가 2047(한 바퀴의 한가운데)로 읽히도록 오프셋을 잡는다. 서보는 움직이지 않는다.

lerobot 의 "middle of range" 단계와 같은 일이다. 이걸 하는 이유는 하나 — 관절이 양 끝까지
움직이는 동안 읽는 값이 0/4095 경계를 넘지 않게 범위를 한가운데로 밀어 두는 것.
그래서 자세는 "각 관절이 양 끝 사이 대충 중간"이면 되고, 정확할 필요도 두 팔이 같을
필요도 없다 (자세 0점은 lerobot 이 기록된 범위의 한가운데로 따로 잡는다).

한 팔씩, 손으로 들고 실행하면 1초 안에 끝난다. 구동이 켜져 있으면 중단한다.

    cd ~/bimanual-robot/tools/servo
    python3 servo_home.py --port <by-id>                      # 6개 모두
    python3 servo_home.py --port <by-id> --only wrist_flex    # 한 관절만
"""
import argparse
import json
import time
from pathlib import Path

from sts_bus import (Bus, A_OFFSET, A_POS, A_TORQUE, RESOLUTION,
                     decode_offset, encode_offset, find_port)

SO101 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CENTER = 2047
BACKUP = Path.home() / ".cache" / "bimanual-robot" / "servo_backup.jsonl"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--only", help="쉼표로 구분한 관절 이름. 없으면 6개 모두")
    ap.add_argument("--execute", action="store_true", help="없으면 계산만 보여준다")
    args = ap.parse_args()

    names = [n.strip() for n in args.only.split(",")] if args.only else SO101
    bad = [n for n in names if n not in SO101]
    if bad:
        print(f"모르는 관절: {bad}. 가능: {SO101}")
        return 1

    bus = Bus(find_port(args.port))
    print(f"{'motor':14s} {'지금':>5} {'오프셋':>7}   ->  {'오프셋':>7} {'읽힘':>5}")
    for name in names:
        sid = SO101.index(name) + 1
        if bus.read(sid, A_TORQUE) != 0:
            print(f"{name}: 구동이 켜져 있습니다. 풀고 다시. 중단.")
            return 1
        pos0, ofs0 = bus.read(sid, A_POS, 2), decode_offset(bus.read(sid, A_OFFSET, 2))
        if pos0 is None or ofs0 is None:
            print(f"{name}: 무응답. 중단.")
            return 1
        if not args.execute:
            print(f"{name:14s} {pos0:>5} {ofs0:>7}   ->  (dry-run)")
            continue
        BACKUP.parent.mkdir(parents=True, exist_ok=True)
        BACKUP.open("a").write(json.dumps({"id": sid, "kind": "offset", "offset": ofs0, "pos": pos0,
                                           "note": f"{name} 0점 잡기 전", "saved": time.strftime("%F %T")},
                                          ensure_ascii=False) + "\n")
        # 부호 규약은 개체마다 다를 수 있어 시험값으로 판정한다 (읽는 값만 바뀌고 서보는 안 움직인다)
        bus.write_eeprom(sid, A_OFFSET, encode_offset(ofs0 + 100))
        sign = 1 if bus.read(sid, A_POS, 2) - pos0 > 0 else -1
        final = ofs0 + sign * (CENTER - pos0)
        final = final - RESOLUTION if final > 2047 else final + RESOLUTION if final < -2047 else final
        bus.write_eeprom(sid, A_OFFSET, encode_offset(final))
        pos1 = bus.read(sid, A_POS, 2)
        mark = "" if abs(pos1 - CENTER) <= 60 else "  <-- 손이 움직였나? 다시"
        print(f"{name:14s} {pos0:>5} {ofs0:>7}   ->  {decode_offset(bus.read(sid, A_OFFSET, 2)):>7} {pos1:>5}{mark}")
    bus.close()
    if args.execute:
        print("\n오프셋을 바꿨으므로 이동 전에 12 V 를 껐다 켜야 한다.")
    else:
        print("\ndry-run. 적용은 --execute")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
