#!/usr/bin/env python3
"""STS3215 Phase 레지스터 하나를 읽어 기대값과 비교한다.

읽기 명령만 사용한다. EEPROM이나 토크 상태를 변경하지 않는다.
"""
from __future__ import annotations

import argparse

from sts_bus import A_PHASE, Bus, find_port


def check_phase(bus, servo_id: int, expected: int) -> bool:
    actual = bus.read(servo_id, A_PHASE)
    if actual is None:
        print(f"ID {servo_id}: Phase 무응답")
        return False
    ok = actual == expected
    print(
        f"ID {servo_id}: Phase={actual}, expected={expected} — "
        f"{'일치' if ok else '불일치'}"
    )
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(
        description="STS3215 Phase(주소 18)를 읽기 전용으로 확인한다"
    )
    parser.add_argument("--port", help="팔로워 보드의 /dev/serial/by-id 경로")
    parser.add_argument("--id", type=int, required=True, help="검사할 서보 ID")
    parser.add_argument("--expected", type=int, required=True, help="기대 Phase 값")
    args = parser.parse_args()

    bus = Bus(find_port(args.port))
    try:
        return 0 if check_phase(bus, args.id, args.expected) else 1
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
