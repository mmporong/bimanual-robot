"""지정한 OpenCV 카메라 index를 실시간 창으로 미리보기한다 (확인용, 녹화 아님).

사용법:
    python preview_index.py <index>
    예: python preview_index.py 3

ESC 또는 창 닫기로 종료.
"""

import sys

import cv2


def main() -> None:
    index = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        sys.exit(f"index {index} 를 열 수 없습니다.")

    window = f"preview index {index} (ESC to quit)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        cv2.imshow(window, frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
