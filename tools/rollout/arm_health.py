"""양팔 모터 읽기 전용 점검: 응답·전압·토크 상태. 팔을 움직이거나 토크를 바꾸지 않는다.

실행기를 켜기 전에 배터리(전압)와 모터 응답을 확인한다. 실행기가 포트를 쓰고 있으면 열 수 없다.

사용법:
    python tools/rollout/arm_health.py                 # COM15(왼팔), COM16(오른팔)
    python tools/rollout/arm_health.py --left COM5 --right COM6
"""
import argparse

import scservo_sdk as scs

ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_VOLTAGE = 62  # 0.1 V 단위
LOW_VOLTAGE = 9.0  # 2026-10-30 왼팔이 9.2 -> 8.4 V로 떨어진 뒤 통신이 끊겼다


def check(side: str, port: str) -> bool:
    ph, pk = scs.PortHandler(port), scs.PacketHandler(0)
    try:
        opened = ph.openPort()
    except Exception as e:  # noqa: BLE001 - pyserial은 실패 시 False 대신 예외를 낸다
        opened = False
        print(f"{side}({port}): {e}")
    if not opened:
        print(f"{side}({port}): 포트를 열 수 없음(실행기가 쓰는 중이거나 USB 없음)")
        return False
    ph.setBaudRate(1000000)
    ok, volts, cells = True, [], []
    for motor_id in range(1, 7):
        _, comm, _ = pk.ping(ph, motor_id)
        if comm != 0:
            cells.append(f"{motor_id}:응답없음")
            ok = False
            continue
        volt, c1, _ = pk.read1ByteTxRx(ph, motor_id, ADDR_PRESENT_VOLTAGE)
        torque, c2, _ = pk.read1ByteTxRx(ph, motor_id, ADDR_TORQUE_ENABLE)
        v = volt / 10 if c1 == 0 else None
        if v is not None:
            volts.append(v)
        cells.append(f"{motor_id}:{v if v is not None else '?'}V 토크{'켬' if torque == 1 and c2 == 0 else '끔'}")
    ph.closePort()
    low = volts and min(volts) < LOW_VOLTAGE
    print(f"{side}({port}): {' | '.join(cells)}" + ("  ⚠ 전압 낮음 - 배터리 교체" if low else ""))
    return ok and not low


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", default="COM15")
    ap.add_argument("--right", default="COM16")
    a = ap.parse_args()
    results = [check("왼팔", a.left), check("오른팔", a.right)]
    print("정상" if all(results) else "확인 필요")


if __name__ == "__main__":
    main()
