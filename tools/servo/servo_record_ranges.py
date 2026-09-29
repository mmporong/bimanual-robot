#!/usr/bin/env python3
"""관절을 손으로 끝에서 끝까지 움직이는 동안 위치를 읽어 범위를 기록한다.

lerobot 의 범위 기록은 6개 모터를 60 Hz 로 한꺼번에 읽는데, 이 버스에서는 깨진
패킷이 섞여 MIN 0 / MAX 4095 같은 값이 박혔다(2026-09-09). 이 도구는
  - 모터를 하나씩 읽고 (체크섬 검증은 sts_bus 가 한다)
  - 직전 값에서 --jump 이상 튀는 읽기는 버리며
  - 정지 상태에서 Enter 를 누르면 그때까지의 MIN/MAX 를 확정한다.

--execute 를 주면 서보 EEPROM 의 Min/Max_Angle_Limit 을 쓰고 lerobot JSON 을 낸다.
--capture-only 는 후보 JSON만 새 파일에 저장한다. EEPROM에 쓰지 않는다.
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dashboard_stop import open_stop_bus
from servo.sts_bus import (A_MIN_ANGLE, A_MAX_ANGLE, A_OFFSET, A_POS, A_TORQUE,
                     RESOLUTION, decode_offset, find_port)

SO101 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
FULL_TURN = {"wrist_roll"}


def enter_pressed():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return False
    if not sys.stdin.readline():
        raise RuntimeError("입력이 닫혔습니다. 범위를 확정하지 않습니다.")
    return True


def validate_capture(calib):
    """관측 부족·wrap 오염을 파일/EEPROM 기록 전에 거절한다."""
    for name in SO101:
        c = calib.get(name)
        if not isinstance(c, dict) or c.get("id") != SO101.index(name) + 1:
            raise ValueError(f"{name}: 기록 누락/ID 오류")
        if any(type(c.get(k)) is not int for k in ("drive_mode", "homing_offset", "range_min", "range_max")):
            raise ValueError(f"{name}: 보정 값은 정수여야 합니다")
        lo, hi = c["range_min"], c["range_max"]
        if not 0 <= lo < hi < RESOLUTION:
            raise ValueError(f"{name}: 범위 오류")
        if name not in FULL_TURN and (lo < 5 or hi > RESOLUTION - 5 or hi - lo < 300):
            raise ValueError(f"{name}: 범위 부족 또는 encoder wrap; 재기록 필요")
        if c["drive_mode"] not in (0, 1) or (name != "gripper" and c["drive_mode"] != 0):
            raise ValueError(f"{name}: drive_mode 오류")
        if not -2047 <= c["homing_offset"] <= 2047:
            raise ValueError(f"{name}: offset 오류")


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
            if p is None or not 0 < p < RESOLUTION - 1:
                continue
            if sid in last and abs(p - last[sid]) > RESOLUTION // 2:
                raise RuntimeError(f"ID {sid}: encoder wrap/큰 위치 불연속; 확정하지 않습니다")
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
    ap.add_argument("--only", help="이 관절만 다시 기록하고 나머지는 기존 JSON 값을 유지. 예: wrist_flex 또는 wrist_flex,gripper")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--capture-only", action="store_true", help="후보 JSON만 저장; EEPROM 변경 없음")
    args = ap.parse_args()
    if args.capture_only and args.execute:
        ap.error("--capture-only와 --execute는 함께 사용할 수 없습니다")
    if args.jump <= 0 or args.gripper_margin < 0:
        ap.error("jump는 양수, margin은 음이 아닌 값이어야 합니다")

    bus = open_stop_bus(find_port(args.port))
    ids = list(range(1, 7))
    only = None
    if args.only:
        only = [n.strip() for n in args.only.split(",")]
        bad = [n for n in only if n not in SO101]
        if bad:
            print(f"모르는 관절: {bad}. 가능: {SO101}"); return 1
        ids = [SO101.index(n) + 1 for n in only]
        existing = Path(args.out).expanduser()
        if not existing.is_file():
            print(f"--only 는 기존 JSON 이 있어야 합니다: {existing}"); return 1
        keep = json.loads(existing.read_text())
    for sid in ids:
        if bus.read(sid, A_TORQUE) != 0:
            print(f"ID {sid} 구동이 켜져 있습니다. 풀고 다시. 중단.")
            return 1

    lo, hi, dropped = record(bus, ids, args.jump)
    print("\n=== 기록 결과 ===")
    calib = {}
    for sid in ids:
        name = SO101[sid - 1]
        if name in FULL_TURN:
            mn, mx = 0, RESOLUTION - 1
        else:
            if sid not in lo or sid not in hi:
                bus.close()
                raise RuntimeError(f"{name}: 유효한 위치 기록 없음")
            mn, mx = lo[sid], hi[sid]
            if name == "gripper" and args.gripper_margin:
                mn, mx = mn + args.gripper_margin, mx - args.gripper_margin
        span = mx - mn
        flag = "  <-- 폭이 작음, 안 움직였나?" if name not in FULL_TURN and span < 300 else ""
        print(f"{name:14s} {mn:>5} ~ {mx:>5}  폭 {span:>5}  버림 {dropped.get(sid,0)}{flag}")
        offset_raw = bus.read(sid, A_OFFSET, 2)
        if offset_raw is None:
            bus.close()
            raise RuntimeError(f"{name}: offset 읽기 실패")
        calib[name] = {"id": sid, "drive_mode": args.gripper_drive_mode if name == "gripper" else 0,
                       "homing_offset": decode_offset(offset_raw),
                       "range_min": mn, "range_max": mx}

    if only:
        merged = dict(keep)
        merged.update(calib)
        calib = {n: merged[n] for n in SO101 if n in merged}
        print(f"(--only) 나머지 관절은 기존 JSON 값 유지: {[n for n in SO101 if n not in only]}")

    try:
        validate_capture(calib)
    except ValueError:
        bus.close()
        raise
    if args.capture_only:
        bus.close()
        out = Path(args.out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("x", encoding="utf-8") as handle:
            json.dump(calib, handle, ensure_ascii=False, indent=4)
            handle.write("\n")
        print(f"후보 저장: {out} (EEPROM 미변경; 적용/끝점 관측 확인 전에는 사용 금지)")
        return 0
    if not args.execute:
        print("\ndry-run. --execute 를 주면 서보 한계와 JSON 을 쓴다.")
        bus.close()
        return 0

    for name, c in calib.items():
        if only and name not in only:
            continue                                   # 기존 관절은 서보도 건드리지 않는다
        bus.write_eeprom(c["id"], A_MIN_ANGLE, c["range_min"])
        bus.write_eeprom(c["id"], A_MAX_ANGLE, c["range_max"])
        got = (bus.read(c["id"], A_MIN_ANGLE, 2), bus.read(c["id"], A_MAX_ANGLE, 2))
        print(f"서보 ID{c['id']} 한계 -> {got[0]}~{got[1]}" + ("" if got == (c["range_min"], c["range_max"]) else "  <-- 쓰기 실패"))
        if got != (c["range_min"], c["range_max"]):
            bus.close()
            raise RuntimeError("EEPROM 읽기 대조 실패. JSON을 저장하지 않습니다. 백업으로 복구가 필요합니다.")
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
