#!/usr/bin/env python3
"""구동을 푼다. 위치 명령은 보내지 않는다.

푸는 순간 자세가 밀렸는지 보이도록 전후 위치를 함께 찍는다.
중력을 받는 자세에서 풀면 팔이 떨어진다. 그리퍼 단독 작업에서만 무심히 쓸 것.

    cd ~/bimanual-robot/tools/servo
    python3 servo_release.py --id 6
"""
import argparse
import time

from sts_bus import Bus, A_TORQUE, A_POS, find_port


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--id", type=int, default=6)
    args = ap.parse_args()

    sid = args.id
    bus = Bus(find_port(args.port))
    before = bus.read(sid, A_POS, 2)
    if before is None:
        print(f"ID {sid} 무응답. 중단.")
        return 1
    print(f"해제 전  위치 {before}  구동 {bus.read(sid, A_TORQUE)}")
    bus.write(sid, A_TORQUE, 0)
    time.sleep(0.05)
    after = bus.read(sid, A_POS, 2)
    print(f"해제 후  위치 {after}  구동 {bus.read(sid, A_TORQUE)}  변화 {after - before:+d}")
    bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
