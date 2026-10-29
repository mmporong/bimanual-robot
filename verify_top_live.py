"""롤아웃 돌리기 전 top(Astra 브리지) 카메라가 '진짜 라이브'인지 올바른 방법으로 검증한다.

중요: cv2.VideoCapture를 열었다 닫았다 하는 방식은 매번 새 프레임을 받아서
'라이브'처럼 보이지만, 실제 롤아웃은 카메라를 한 번 열어 '연속으로' 읽는다.
Astra 브리지가 freeze되면 연속 읽기에서만 얼어붙은 게 드러난다 — 그래서 이
스크립트는 롤아웃과 똑같이 OpenCVCamera 연속 읽기로 검증한다.

겸사겸사 index 0~5를 스캔해서 각 인덱스가 라이브인지/어떤 화면인지도 보여준다
(USB 재연결 후 인덱스가 바뀌었을 수 있으므로).

사용법: python verify_top_live.py
로봇/모터는 건드리지 않는다(카메라 읽기 전용).
"""
import time
import numpy as np
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

print("index 0~5를 OpenCVCamera 연속 읽기(3초씩)로 검증합니다 (롤아웃과 동일 경로)...\n")
print("RESULT_START")
print(f"{'index':>5} {'frames':>7} {'mean':>8} {'std':>8} {'uniq':>5}  판정")
for idx in range(6):
    try:
        cam = OpenCVCamera(OpenCVCameraConfig(index_or_path=idx, width=640, height=480, fps=30))
        cam.connect()
    except Exception:
        print(f"{idx:>5}   (열 수 없음)")
        continue
    vals = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 3:
        try:
            vals.append(float(cam.read().mean()))
        except Exception:
            pass
        time.sleep(0.03)
    try:
        cam.disconnect()
    except Exception:
        pass
    if not vals:
        print(f"{idx:>5}   (프레임 못 읽음)")
        continue
    v = np.array(vals)
    uniq = len(np.unique(np.round(v, 2)))
    verdict = "FROZEN(얼어붙음)" if v.std() < 0.05 else "live"
    print(f"{idx:>5} {len(v):>7} {v.mean():>8.2f} {v.std():>8.4f} {uniq:>5}  {verdict}")
print("RESULT_END")
print("\n- top(Astra 브리지)에 해당하는 인덱스가 'live'여야 롤아웃을 돌린다.")
print("- 'FROZEN'이면 Astra USB를 뽑았다 꽂고 브리지를 재시작한 뒤 다시 확인한다.")
print("- 각 인덱스가 어느 카메라(오른손목/왼손목/top)인지는 화면 내용으로 확인하려면")
print("  캡처가 필요하다 — 의심되면 알려주면 캡처해준다.")
