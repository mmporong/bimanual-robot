#!/usr/bin/env python3
"""서보 하나를 지정 위치로 보낸다. 기본은 dry-run 이고 실물 적용은 --execute 다.

안전장치 네 개를 넣었다. 전부 실물 사고에서 나온 것이다.
  1. 구동을 켜기 전에 목표를 현재 위치로 덮어쓴다 — SRAM 에 남은 옛 목표로 튀는 것을 막는다
  2. 델타 상한 — 예상보다 크게 움직이려 하면 시작하지 않는다
  3. 부하 상한 — 넘으면 즉시 정지 명령을 보내고 구동을 푼다
  4. 정지 감시 — 위치가 안 변하면 걸린 것으로 보고 멈춘다. 스톨을 유지하면 서보가 탄다

    cd ~/bimanual-robot/tools/servo
    python3 servo_goto.py --goal 3000
    python3 servo_goto.py --goal 3000 --execute
"""
import argparse
import time

from sts_bus import (Bus, FakeBus, A_MIN_ANGLE, A_MAX_ANGLE, A_TORQUE, A_ACCEL,
                     A_GOAL, A_SPEED, A_POS, A_LOAD, A_TEMP, RESOLUTION, find_port)

STALL_SEC = 0.5        # 이 시간 동안 안 움직이면 걸린 것으로 본다
STALL_TOL = 4          # 이 카운트 이하 변화는 안 움직인 것으로 친다
ARRIVE_TOL = 3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--id", type=int, default=6)
    ap.add_argument("--goal", type=int, required=True)
    ap.add_argument("--max-delta", type=int, default=1200, help="이보다 크게 움직이려 하면 중단")
    ap.add_argument("--speed", type=int, default=300)
    ap.add_argument("--load-limit", type=int, default=600,
                    help="정상 이동 부하는 350 안팎이다. 너무 낮게 잡으면 정상 이동에서 걸린다")
    ap.add_argument("--diverge-tol", type=int, default=15,
                    help="목표에서 이 카운트 이상 멀어지면 극성 반대로 보고 즉시 차단")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--mock-wall", type=int, help="모의 하드 스톱 위치")
    args = ap.parse_args()

    sid = args.id
    bus = FakeBus(sid=sid, wall=args.mock_wall) if args.mock else Bus(find_port(args.port))

    pos = bus.read(sid, A_POS, 2)
    if pos is None:
        print(f"ID {sid} 무응답. 중단.")
        return 1
    lo, hi = bus.read(sid, A_MIN_ANGLE, 2), bus.read(sid, A_MAX_ANGLE, 2)
    delta = args.goal - pos
    print(f"ID {sid}  현재 {pos}  목표 {args.goal}  델타 {delta:+d} "
          f"({delta * 360 / RESOLUTION:+.1f}도)  한계 {lo}~{hi}  온도 {bus.read(sid, A_TEMP)}C")

    if lo is not None and hi is not None and not (lo <= args.goal <= hi):
        print("목표가 각도 한계 밖입니다. 한계를 먼저 확인하세요. 중단.")
        return 1
    if abs(delta) > args.max_delta:
        print(f"델타가 상한 {args.max_delta} 을 넘습니다. 의도한 이동이면 --max-delta 를 올리세요. 중단.")
        return 1
    if not args.execute:
        print("dry-run 으로 끝났습니다. 실물 적용은 --execute 입니다.")
        return 0

    def stop_and_release(where, why):
        if where is not None:
            bus.write(sid, A_GOAL, where, 2)
        bus.write(sid, A_TORQUE, 0)
        time.sleep(0.05)
        print(f"  {why} -> 정지·구동해제. 위치 {bus.read(sid, A_POS, 2)}")

    bus.write(sid, A_GOAL, pos, 2)                 # 안전장치 1
    bus.write(sid, A_ACCEL, 10)
    bus.write(sid, A_SPEED, args.speed, 2)
    bus.write(sid, A_TORQUE, 1)
    time.sleep(0.05)
    bus.write(sid, A_GOAL, args.goal, 2)

    budget = max(8.0, abs(delta) / max(args.speed, 1) * 1.5 + 2.0)
    start = time.time()
    peak, last, moved_at = 0, pos, time.time()
    err0 = abs(args.goal - pos)                    # 시작 오차
    worst_err = err0
    while time.time() - start < budget:
        time.sleep(0.03)
        now = bus.read(sid, A_POS, 2)
        load = (bus.read(sid, A_LOAD, 2) or 0) & 0x3FF
        if now is None:
            continue
        peak = max(peak, load)
        err = abs(args.goal - now)
        if abs(now - last) > STALL_TOL:
            last, moved_at = now, time.time()
        # 안전장치 5: 발산 — 목표에서 시작 오차보다 멀어지면 극성 반대. 즉시 차단
        if err > err0 + args.diverge_tol:
            stop_and_release(now, f"발산: 목표에서 멀어짐 (오차 {err0}->{err}). 모터-엔코더 극성 반대 의심")
            return 4
        worst_err = max(worst_err, err)
        if load > args.load_limit:                 # 안전장치 3
            stop_and_release(now, f"부하 {load} > {args.load_limit}")
            return 2
        if abs(now - args.goal) <= ARRIVE_TOL:
            break
        if time.time() - moved_at > STALL_SEC:     # 안전장치 4
            stop_and_release(now, f"{STALL_SEC}s 동안 정지 (목표까지 {args.goal - now:+d} 남음)")
            return 2

    bus.write(sid, A_TORQUE, 0)
    time.sleep(0.05)
    final = bus.read(sid, A_POS, 2)
    print(f"최종 {final}  오차 {final - args.goal:+d}  최대부하 {peak}  "
          f"소요 {time.time() - start:.2f}s  구동 해제됨")
    bus.close()
    return 0 if abs(final - args.goal) <= 5 else 3


if __name__ == "__main__":
    raise SystemExit(main())
