#!/usr/bin/env python3
"""관절을 손으로 끝에서 끝까지 움직이는 동안 위치를 읽어 범위를 기록한다.

lerobot 의 범위 기록은 6개 모터를 60 Hz 로 한꺼번에 읽는데, 이 버스에서는 깨진
패킷이 섞여 MIN 0 / MAX 4095 같은 값이 박혔다(2026-09-09). 이 도구는
  - 모터를 하나씩 읽고 (체크섬 검증은 sts_bus 가 한다)
  - 직전 값에서 --jump 이상 튀는 읽기는 버리며
  - 정지 상태에서 Enter 를 누르면 그때까지의 MIN/MAX 를 확정한다.

--execute 를 주면 서보 EEPROM 의 Min/Max_Angle_Limit 을 쓰고 lerobot JSON 을 낸다.
wrist_roll 은 lerobot 관례대로 0~4095 로 둔다. 서보는 움직이지 않는다(구동 꺼짐 확인).

    cd ~/bimanual-robot/tools/servo
    python3 servo_record_ranges.py --port <by-id> --out <lerobot json> [--gripper-drive-mode 1] [--gripper-margin 40]
"""
import argparse
import json
import select
import shutil
import sys
import time
from pathlib import Path

from sts_bus import (Bus, A_MIN_ANGLE, A_MAX_ANGLE, A_OFFSET, A_POS, A_TORQUE,
                     RESOLUTION, decode_offset, find_port)

SO101 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
FULL_TURN = {"wrist_roll"}


def enter_pressed():
    return select.select([sys.stdin], [], [], 0)[0] and sys.stdin.readline() is not None


def record(bus, ids, jump):
    """글리치 = 혼자 튀는 한 샘플. 직전 값에서 크게 벗어난 읽기는 보류해 두고,
    다음 읽기가 그 근처면 실제 이동으로 받아들이고 아니면 버린다."""
    last, pending, lo, hi, dropped = {}, {}, {}, {}, {}
    print("모든 관절을 끝에서 끝까지 천천히 움직이세요. 다 됐으면 Enter.\n")
    t_print = 0

    def accept(sid, p):
        last[sid] = p
        lo[sid] = min(lo.get(sid, p), p)
        hi[sid] = max(hi.get(sid, p), p)

    while not enter_pressed():
        for sid in ids:
            p = bus.read(sid, A_POS, 2)
            if p is None:
                continue
            if sid not in last or abs(p - last[sid]) <= jump:
                if pending.pop(sid, None) is not None:
                    dropped[sid] = dropped.get(sid, 0) + 1   # 보류값이 확인 안 됨 → 글리치
                accept(sid, p)
            elif sid in pending and abs(p - pending[sid]) <= jump:
                accept(sid, pending.pop(sid))          # 두 번 연속 같은 곳 → 실제 이동
                accept(sid, p)
            else:
                if sid in pending:
                    dropped[sid] = dropped.get(sid, 0) + 1   # 보류값이 확인 안 됨 → 글리치
                pending[sid] = p
        if time.time() - t_print > 0.25:
            t_print = time.time()
            rows = [f"{SO101[s-1]:14s} MIN {lo.get(s,'-'):>5} POS {last.get(s,'-'):>5} MAX {hi.get(s,'-'):>5} 버림 {dropped.get(s,0)}" for s in ids]
            sys.stdout.write("\033[F" * (len(ids)) if t_print else "")
            print("\n".join(rows))
        time.sleep(0.01)
    return lo, hi, dropped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--out", required=True, help="lerobot 캘리브레이션 JSON 경로")
    ap.add_argument("--jump", type=int, default=250, help="직전 값에서 이 이상 튀면 버린다")
    ap.add_argument("--gripper-drive-mode", type=int, default=0, choices=(0, 1))
    ap.add_argument("--gripper-margin", type=int, default=0,
                    help="그리퍼 양 끝에서 이만큼 안쪽으로 한계를 잡는다 (레일 이탈 방지, 40 권장)")
    ap.add_argument("--execute", action="store_true")
    args = ap.parse_args()

    bus = Bus(find_port(args.port))
    ids = list(range(1, 7))
    for sid in ids:
        if bus.read(sid, A_TORQUE) != 0:
            print(f"ID {sid} 구동이 켜져 있습니다. 풀고 다시. 중단.")
            return 1

    lo, hi, dropped = record(bus, ids, args.jump)
    print("\n=== 기록 결과 ===")
    calib = {}
    for sid, name in zip(ids, SO101):
        if name in FULL_TURN:
            mn, mx = 0, RESOLUTION - 1
        else:
            mn, mx = lo[sid], hi[sid]
            if name == "gripper" and args.gripper_margin:
                mn, mx = mn + args.gripper_margin, mx - args.gripper_margin
        span = mx - mn
        flag = "  <-- 폭이 작음, 안 움직였나?" if name not in FULL_TURN and span < 300 else ""
        print(f"{name:14s} {mn:>5} ~ {mx:>5}  폭 {span:>5}  버림 {dropped.get(sid,0)}{flag}")
        calib[name] = {"id": sid, "drive_mode": args.gripper_drive_mode if name == "gripper" else 0,
                       "homing_offset": decode_offset(bus.read(sid, A_OFFSET, 2)),
                       "range_min": mn, "range_max": mx}

    if not args.execute:
        print("\ndry-run. --execute 를 주면 서보 한계와 JSON 을 쓴다.")
        bus.close()
        return 0

    for name, c in calib.items():
        bus.write_eeprom(c["id"], A_MIN_ANGLE, c["range_min"])
        bus.write_eeprom(c["id"], A_MAX_ANGLE, c["range_max"])
        got = (bus.read(c["id"], A_MIN_ANGLE, 2), bus.read(c["id"], A_MAX_ANGLE, 2))
        print(f"서보 ID{c['id']} 한계 -> {got[0]}~{got[1]}" + ("" if got == (c["range_min"], c["range_max"]) else "  <-- 쓰기 실패"))
    bus.close()

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        bak = out.with_name(out.name + ".bak-" + time.strftime("%Y%m%d-%H%M%S"))
        shutil.copy2(out, bak)
        print(f"기존 JSON 백업 -> {bak}")
    out.write_text(json.dumps(calib, indent=4) + "\n")
    print(f"저장 -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
