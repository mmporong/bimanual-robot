# water-fill-detection

비스듬히 내려다보는 탑뷰 카메라 한 대로 **투명 컵 속 액체가 얼마나 찼는지(높이 %, 부피 %)**를 실시간으로 재는 도구입니다.
로봇 팔이 음료를 따를 때 "목표만큼 찼는지"를 판단하는 용도로 만들었습니다.

- 컵 검출: [YOLOE](https://docs.ultralytics.com/models/yoloe/) open-vocabulary 세그멘테이션 (`"cup"`)
- 액체 검출: 직접 라벨링한 사진으로 파인튜닝한 YOLOE 세그멘테이션 모델 (`water` 클래스)
- 충전율: 컵 입구·수위선·바닥을 타원으로 보고 기하 계산 (학습 없음, 컵마다 보정 없음)
- 웹 GUI: 실시간 영상 + 컵별 충전율 + 목표 도달 판정, 목표값은 슬라이더로 조절

## 원리

카메라가 컵을 비스듬히(약 45°) 내려다보면 컵 입구, 액체 수위선, 컵 바닥이 모두 **납작한 타원**으로 보입니다.
타원의 끝(박스 윗변, 액체 맨 윗줄)끼리 비교하면 위가 넓은 컵에서는 충전율이 7~15%p 높게 나오므로, **타원을 가로지르는 가운데 선**끼리 비교합니다.

```
          ___________            ← 컵 입구 타원 (노랑)   rim_y    = 박스 윗변 + k·(입구 반지름)
         (_____•_____)
          \         /
           \~~~•~~~/             ← 수위선 타원 (흰색)     surface_y = 액체 맨 윗줄 + k·(그 높이 컵 반지름)
            \:::::/
             (_•_)               ← 바닥 타원 (빨강)       bottom_y = 박스 아랫변 − k·(바닥 반지름)

높이 충전율 = (bottom_y − surface_y) / (bottom_y − rim_y)
부피 충전율 = Σ(바닥~수위선 줄의 컵 너비²) / Σ(바닥~입구 줄의 컵 너비²)
```

- **납작함 k** (타원 세로/가로 비): 컵이 아니라 카메라가 내려다보는 각도로 정해집니다. 화면에 보이는 컵 입구 타원에서 매 프레임 재고(컵 윤곽이 가장 넓은 줄 = 입구 타원 중심), 최근 30번의 중간값을 씁니다. 그래서 컵을 바꿔도 따로 보정하지 않습니다.
- **수위선**: 액체 마스크의 맨 윗줄(입구로 보이는 수면의 뒤쪽 끝)을 기준으로, 그 높이의 컵 너비(벽~벽)와 k로 수위선 타원을 정합니다. 컵 벽 너머로 보이는 액체까지 포함한 실제 수위입니다.
- **부피**: 비스듬히 본 각 단면 타원은 자기 중심 줄에서 가장 넓으므로 "컵 윤곽의 줄 너비 = 그 높이 단면의 지름"입니다. 단면 넓이 ∝ 지름²을 쌓아서, 원뿔대·와인잔 등 **단면이 둥근 컵이면 모양에 상관없이** 계산합니다.
- **원근 보정**: 원근 때문에 컵이 기울어 보이므로, 각 타원의 가운데 x를 그 높이에서 컵 윤곽의 왼쪽~오른쪽 끝 중간으로 따로 잡습니다.

### 안정화와 가려짐 처리

로봇이 따르는 동안에는 병 입구·물줄기·그리퍼가 컵을 가립니다.

1. 새 컵이 약 1초(10프레임) 그대로 있으면 그동안의 컵 윤곽(다수결)과 그때의 모습(회색조 중간값)을 **저장**합니다. 이후 입구·바닥·높이별 너비·k는 저장값을 쓰고, 매 프레임 바뀌는 건 수위선뿐입니다.
2. 저장된 윤곽에서 빠진 부분, 또는 저장된 모습과 크게 달라진 덩어리가 **수위선보다 위쪽**에 있으면 틀린 숫자 대신 `occluded`(판정 불가)를 냅니다. 가는 물줄기와 조명 전체 변화는 무시합니다. 수위선 아래가 가려진 것은 저장된 모양으로 계속 잽니다.
3. 컵이 밀리면 저장값을 버리고 다시 저장합니다. 화면 끝에 걸친 컵은 판정하지 않습니다.
4. 목표 ± 허용오차로 `under / target / over`를 내고, **같은 판정이 5프레임 연속**일 때만 확정합니다.

## 요구 사항

- Python 3.10+, CUDA GPU 권장 (CPU도 동작하지만 느림)
- 카메라: 컵을 약 45° 위에서 비스듬히 내려다보는 카메라 1대
  - Orbbec Astra (기본) - OpenNI2 헬퍼 프로그램 필요 (아래 참고)
  - 또는 일반 USB 웹캠 (`--camera`)
- 투명 컵 + 색 있는 음료에서 가장 잘 동작 (한계 참고)

```bash
pip install -r requirements.txt
```

### 모델

| 파일 | 내용 | 얻는 방법 |
|---|---|---|
| `models/water_yoloe.pt` | 액체 세그멘테이션 (`yoloe-26s-seg`를 직접 라벨링한 탑뷰 사진 56장으로 파인튜닝) | 저장소 Release 또는 직접 학습 |
| `models/yoloe-26s-seg.pt` | 컵 검출용 YOLOE 기본 모델 | 없으면 Ultralytics가 자동 다운로드 |

처음 실행할 때 Ultralytics가 텍스트 프롬프트용 `mobileclip2_b.ts`(약 250MB)를 현재 폴더에 내려받습니다.

### Orbbec Astra 사용 시

Astra의 컬러 영상은 일반 웹캠(`/dev/video*`)으로 잡히지 않아서, OpenNI2로 프레임을 표준출력에 내보내는 헬퍼 실행 파일을 씁니다.
헬퍼는 `uint8 stream_id(0=color,1=depth) + uint32 width + uint32 height + uint32 num_bytes + 픽셀` 형식으로 프레임을 계속 출력해야 합니다 (color는 RGB888).

```bash
export ASTRA_HELPER=/path/to/astra_stream
```

Astra는 한 프로그램만 열 수 있으므로 다른 프로그램이 쓰는 중이면 실패합니다.

## 실행

```bash
# Orbbec Astra 탑뷰 (기본)
python -m water_detection.detect_gui

# 일반 USB 웹캠 / 영상 파일 / 사진 시퀀스(반복 재생)
python -m water_detection.detect_gui --camera 0
python -m water_detection.detect_gui --camera /dev/v4l/by-path/...-video-index0
python -m water_detection.detect_gui --camera 'photos/img_%04d.jpg'

# 옵션
python -m water_detection.detect_gui --port 8095 --target-fill 0.7 --tolerance 0.05
```

브라우저에서 <http://localhost:8095> 을 엽니다. 서버는 브라우저 접속과 상관없이 백그라운드에서 계속 계산합니다.

### 화면

| 표시 | 뜻 |
|---|---|
| 노랑 타원 | 컵 입구 |
| 흰 타원 | 감지된 수위선 |
| 빨강 타원 | 컵 바닥 |
| 초록 선 | 목표 수위 (부피 기준이면 부피 누적 곡선으로 계산한 높이) |
| 어두운 빨강 칠 | 저장된 컵 영역 중 지금 가려진 부분 |
| `#1: vol 38% / h 56% under` | 컵 번호, 부피 %, 높이 %, 판정 |
| `[target]` | 5프레임 연속 같은 판정 (확정) |
| `locking 3/10` | 컵 모양 저장 중 |
| `no_water` / `occluded` / `cup_at_edge` / `rim_unclear` | 액체 미검출(비었다는 뜻 아님) / 수위선 위쪽 가려짐 / 화면 끝에 잘림 / 입구를 못 잡음 |

오른쪽 패널에서 판정 기준(부피 % / 높이 %), 목표 충전율, 허용 오차를 바꿉니다.

### HTTP API (로봇 제어 코드와 연결할 때)

| 요청 | 내용 |
|---|---|
| `GET /status` | 최신 판정 JSON |
| `GET /settings`, `POST /settings` | `{"target_fill": 0.7, "tolerance": 0.05, "by_volume": true}` |
| `GET /stream` | 표시 영상 (MJPEG) |
| `GET /raw.jpg` | 표시를 그리기 전 원본 한 장 |

`/status` 예시:

```json
{
  "cups": [{"id": 1, "side": "image-left", "status": "ok", "lock_progress": 0.0,
            "ratio": 0.56, "volume_ratio": 0.38, "reason": null,
            "state": "under", "confirmed": "under"}],
  "target_fill": 0.7, "tolerance": 0.05, "by_volume": true,
  "fps": 9.5, "source": "Astra 탑뷰", "flatness": 0.72, "error": null, "updated": 1791249267.0
}
```

따르기를 멈추는 신호로는 `confirmed`(확정 판정)를 쓰고, `ratio`가 `null`이면(`reason` 참고) 더 따르지 말고 기다리는 것을 권장합니다.

### 사진으로 확인

```bash
python -m water_detection.fill_eval photo1.jpg photo2.jpg --out captures/fill_eval
```

사진마다 충전율을 출력하고 타원을 그린 이미지를 저장합니다.

## 테스트

```bash
python -m unittest discover -s tests -t .
```

합성한 컵(비스듬히 본 원뿔대)으로 높이·부피 충전율, 원근 기울기, 물줄기·다른 컵 무시, 가려짐·이동·중복 검출 처리를 검사합니다. 카메라·모델 없이 실행됩니다.

## 주요 파라미터

| 위치 | 이름 | 기본값 | 뜻 |
|---|---|---|---|
| `detect_gui.py` | `CONFIRM_FRAMES` | 5 | 판정 확정에 필요한 연속 프레임 |
| | `CONF_THRESHOLD` / `MEASURE_CONF` | 0.15 / 0.05 | 화면 표시 / 측정용 액체 신뢰도 기준 |
| `fill_level.py` | `MIN_WATER_BOX_FRACTION` | 0.01 | 컵 면적 대비 이보다 적은 액체는 미검출(물기) |
| | `FLATNESS_RANGE` | (0.2, 1.0) | 허용하는 납작함 k 범위 |
| | `OCCLUSION_MIN_PX` | 80 | 수위선 위쪽에서 이보다 많이 가려지면 판정 불가 |
| `cup_tracker.py` | `LOCK_FRAMES` | 10 | 컵 모양을 저장하기까지 프레임 |
| | `DIFF_THRESHOLD` | 40 | 저장된 모습과 밝기 차이 기준 (가려짐 판단) |
| | `LOST_FRAMES` | 30 | 컵이 이만큼 안 보이면 저장값 버림 |

## 한계

- **정확도 미검증**: 자로 잰 실제 수위·계량한 부피와 비교한 정량 평가는 아직 하지 않았습니다.
- **mL 아님**: 비율만 냅니다. mL가 필요하면 컵 용량을 곱하거나 컵별 보정이 필요합니다.
- **투명 컵 전용**: 불투명 컵은 벽 너머 액체가 보이지 않아 동작하지 않습니다.
- **맑은 물**: 학습 사진이 대부분 색 있는 음료라 맑은 물은 잘 못 잡습니다 (사진을 더 모아 재학습 필요).
- **뚜껑 덮인 컵**: 컵 박스 윗변이 뚜껑 꼭대기가 되어 k가 틀어집니다.
- **바닥 타원**: 입구에서 잰 k를 그대로 써서 실제 바닥보다 둥글게 그려질 수 있습니다.
- **단면이 둥글지 않은 컵**(사각 컵)은 부피 계산이 맞지 않습니다.
- **액체를 덜어내는 경우**: 수위가 저장 시점보다 내려가면 빈 부분을 가려짐으로 잘못 볼 수 있습니다.
- 깊이 카메라(Astra 적외선 구조광)는 액체·투명 컵에서 깊이를 거의 못 읽어(컵 영역 유효 픽셀 약 1.2%) 쓰지 않습니다.

## 폴더 구조

```
water_detection/
  detect_gui.py     실시간 웹 GUI (FastAPI) - 카메라 → 검출 → 충전율 → 판정
  fill_level.py     충전율(높이·부피) 계산, 목표 판정, 연속 확정, 납작함 추정
  cup_tracker.py    컵 모양 저장, 가려짐·이동·중복 검출 판정
  fill_eval.py      사진으로 계산 확인
perception/
  yoloe_detector.py YOLOE 컵 검출 (박스 + 마스크)
  astra_stream.py   Orbbec Astra 컬러 스트림 (OpenNI2 헬퍼)
tests/              단위 테스트 (합성 컵)
models/             모델 가중치 (git에 기본 포함 안 함)
```

## 라이선스

이 코드는 [Ultralytics](https://github.com/ultralytics/ultralytics)(AGPL-3.0)를 사용합니다. 공개 배포 시 라이선스 조건을 확인하세요.
