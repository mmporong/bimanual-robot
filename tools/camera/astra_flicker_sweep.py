"""Astra 컬러 노출값을 바꿔가며 조명 깜빡임 줄무늬(플리커) 세기와 밝기를 잰다.

60Hz 전원 조명은 초당 120번 깜빡이고, 카메라는 화면을 위에서 아래로 한 줄씩 찍어서
노출 시간이 깜빡임 주기(1/120초)의 정수배가 아니면 가로 줄무늬가 생겨 흘러간다.
노출 단위가 장치마다 달라 가정하지 않고 넓게 훑어서 줄무늬가 가장 약하고 화면이
너무 어둡거나 하얗게 날아가지 않는 값을 고른다.

측정: 장면을 움직이지 않은 상태에서 프레임마다 행 평균 밝기를 구하고, 시간 중앙값과의
차이(움직이는 가로줄만 남는다)의 표준편차를 '줄무늬 세기'로 쓴다. 낮을수록 좋다.

사용법 (브리지를 끄고, 카메라 앞을 움직이지 말 것):
    python tools/camera/astra_flicker_sweep.py
    python tools/camera/astra_flicker_sweep.py --gain 0 --values 50 83 100 167
"""

import argparse
import time

import numpy as np
from primesense import openni2

from astra_rgb_virtualcam_bridge import SDK_BIN

DEFAULT_VALUES = [1, 2, 4, 8, 16, 17, 25, 33, 42, 50, 67, 83, 100, 125, 167, 200, 250, 333, 417, 500, 667, 833, 1000]


def grab(stream, n: int) -> np.ndarray:
    frames = []
    for _ in range(n):
        f = stream.read_frame()
        a = np.frombuffer(f.get_buffer_as_uint8(), dtype=np.uint8).reshape(f.height, f.width, 3)
        frames.append(a.mean(axis=2))
    return np.stack(frames).astype(np.float32)


def banding(gray: np.ndarray) -> float:
    rows = gray.mean(axis=2)
    dev = rows - np.median(rows, axis=0)
    return float(np.median(dev.std(axis=1)))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--values", type=int, nargs="*", default=DEFAULT_VALUES)
    ap.add_argument("--gain", type=int, default=None, help="고정할 gain. 생략하면 gain은 건드리지 않는다")
    ap.add_argument("--settle", type=int, default=15, help="값을 바꾼 뒤 버리는 프레임 수")
    ap.add_argument("--frames", type=int, default=45, help="측정에 쓰는 프레임 수")
    args = ap.parse_args()

    openni2.initialize(SDK_BIN)
    dev = openni2.Device.open_any()
    st = dev.create_color_stream()
    st.start()
    try:
        for _ in range(10):
            st.read_frame()
        cam = st.camera
        cam.auto_exposure = False
        cam.auto_white_balance = False
        if args.gain is not None:
            cam.gain = args.gain
        base = grab(st, args.frames)
        print(f"기준(자동 수렴 후 고정): 줄무늬 {banding(base):5.2f}  밝기 {base.mean():5.1f}  "
              f"읽은 gain={cam.gain} exposure={cam.exposure}")
        print(f"{'설정 노출':>8} {'읽은 노출':>8} {'줄무늬':>7} {'밝기':>6} {'하얗게 날아간 %':>14}")
        rows = []
        for v in args.values:
            try:
                cam.exposure = v
            except Exception as e:  # noqa: BLE001
                print(f"{v:>8}  설정 실패: {e}")
                continue
            for _ in range(args.settle):
                st.read_frame()
            g = grab(st, args.frames)
            b, m, sat = banding(g), float(g.mean()), float((g > 250).mean() * 100)
            rows.append((v, b, m, sat))
            print(f"{v:>8} {cam.exposure:>8} {b:7.2f} {m:6.1f} {sat:14.1f}")
            time.sleep(0.05)
        ok = [r for r in rows if 60 <= r[2] <= 190 and r[3] < 5]
        if ok:
            best = min(ok, key=lambda r: r[1])
            print(f"\n추천 노출 = {best[0]} (줄무늬 {best[1]:.2f}, 밝기 {best[2]:.1f}). "
                  "밝기 60~190, 날아간 화소 5% 미만 중 줄무늬가 가장 약한 값")
        else:
            print("\n밝기 조건(60~190, 날아간 화소 5% 미만)을 만족하는 값이 없다. --gain 을 바꿔 다시 시험할 것")
    finally:
        st.stop()
        dev.close()
        openni2.unload()


if __name__ == "__main__":
    main()
