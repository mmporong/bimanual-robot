#!/usr/bin/env python3
"""버스에 붙은 서보 상태를 읽는다. 읽기 전용이라 언제 실행해도 안전하다.

    cd ~/bimanual-robot/tools/servo
    python3 servo_read.py
    python3 servo_read.py --id 6
"""
import argparse

from sts_bus import (Bus, A_MIN_ANGLE, A_MAX_ANGLE, A_MAX_TEMP, A_P, A_D, A_I,
                     A_OFFSET, A_TORQUE, A_POS, A_PRESENT_SPEED, A_LOAD,
                     A_VOLT, A_TEMP, decode_offset, find_port, signed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="생략하면 /dev/serial/by-id 에서 자동으로 찾는다")
    ap.add_argument("--id", type=int, help="지정하면 그 서보만 읽는다")
    ap.add_argument("--range", type=int, nargs=2, default=[1, 20], metavar=("LO", "HI"))
    args = ap.parse_args()

    bus = Bus(find_port(args.port))
    ids = [args.id] if args.id else bus.scan(*args.range)
    if not ids:
        print("응답한 서보가 없습니다. 12V 전원과 데이지체인 첫 단을 확인하세요.")
        bus.close()
        return 1

    head = (f"{'ID':>3} {'위치':>6} {'하한':>6} {'상한':>6} {'폭':>6} "
            f"{'오프셋':>7} {'구동':>4} {'부하':>6} {'속도':>6} "
            f"{'전압':>6} {'온도':>5} {'P':>3}{'D':>4}{'I':>3}")
    print(head)
    print("-" * len(head))
    for sid in ids:
        def rd(addr, n=1):
            return bus.read(sid, addr, n)

        pos, lo, hi = rd(A_POS, 2), rd(A_MIN_ANGLE, 2), rd(A_MAX_ANGLE, 2)
        if pos is None:
            print(f"{sid:>3}  무응답")
            continue
        raw_offset = rd(A_OFFSET, 2)
        offset = decode_offset(raw_offset) if raw_offset is not None else None
        volt = rd(A_VOLT)
        span = None if (lo is None or hi is None) else hi - lo
        print(f"{sid:>3} {pos:>6} {str(lo):>6} {str(hi):>6} {str(span):>6} "
              f"{str(offset):>7} {str(rd(A_TORQUE)):>4} "
              f"{str(signed(rd(A_LOAD, 2))):>6} {str(signed(rd(A_PRESENT_SPEED, 2))):>6} "
              f"{'' if volt is None else f'{volt / 10:.1f}':>6} "
              f"{str(rd(A_TEMP)):>5} {str(rd(A_P)):>3}{str(rd(A_D)):>4}{str(rd(A_I)):>3}")

    limit = bus.read(ids[0], A_MAX_TEMP)
    print(f"\n온도 상한 {limit}C · 구동 1 이면 토크가 걸린 상태다")
    bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
