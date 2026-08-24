# R25 · Pointmap을 우리 스택에 붙이는 최소 경로

작성 2026-08-24 · 대상: arXiv:2607.11498 *See like a Robot: Robot-Centric Pointmaps for VLA*
저자: Byungkun Lee, Dongyoon Hwang, Dongjin Kim, Hojoon Lee, Minho Park, Jaegul Choo (KAIST DAVIAN) · 2026-07-13 · CC BY 4.0
프로젝트: davian-robotics.github.io/pointmap

## 왜 우리 논점인가

"depth를 정책 입력으로 못 쓴다"는 판단을 무효화한다. pointmap은 **각 픽셀에 그 점의 로봇 좌표계 XYZ를 담은 이미지**라 RGB와 같은 형식이고, 카메라 프레임과 로봇 프레임의 불일치를 표현 자체에서 없앤다.

## 구조

- pointmap을 **두 번째 이미지 인코더**로 인코딩. 이 인코더는 **RGB 인코더 가중치로 초기화**
- RGB 토큰과 **element-wise 덧셈**으로 결합
- 저자 요약: *"one extra encoder and one element-wise addition"*
- ablation: Plücker ray(카메라 기하) + per-pixel depth 조합과 비교 — pointmap이 우위

## 실험

| 항목 | 내용 |
|---|---|
| 기반 모델 | **pi0.5 · SmolVLA** (우리 스택 그대로) |
| 시뮬 | RoboCasa 24 태스크 |
| 실기 | Franka FR3, 4 태스크(pick-and-place · 블록 쌓기 · 서랍 열기 · 서랍 닫기) |
| 데모 | **180개**, 학습 시점 **3개** |
| 이득 | 학습에서 못 본 **카메라 배치**에서 우위 (시점 일반화) |

## 우리에게 걸리는 지점

1. **hand-eye calibration이 전제다.** 픽셀→로봇좌표 변환에 카메라 intrinsics + base↔camera extrinsics가 필요하다. `~/so101_tools`의 hand-eye·Astra 자산이 그대로 입력 파이프라인이 된다 — 담당(IK+Depth)이 학습 축과 처음으로 직결된다
2. **규모가 현실적이다.** 4태스크·180데모·시점 3개는 우리 수집 예산과 같은 자리수다
3. **시점 일반화 이득이 우리 상황과 맞는다.** 상단 Astra + 손목캠 2대 배치가 세션마다 미세하게 달라지는데, 그 변동이 pointmap에서는 표현이 흡수한다
4. **리스크 — 코드 링크가 비활성.** 재현은 직접 구현 전제. 구현 범위: pointmap 생성(캘리브 + depth 역투영) / 두 번째 인코더 추가 / 토큰 덧셈. SmolVLA 쪽 개조 지점은 `modeling_smolvla.py:553-617`의 `embed_prefix` 계열

## 채택안

| depth 용도 | 방식 |
|---|---|
| 정책 입력 | **pointmap 채널** (인코더 1 + 덧셈 1) — 채택하되 **베이스라인 RGB 학습 이후** |
| 성공 판정 | 실좌표 관측 (로그 문자열 금지) |
| IK 타깃 | 물체 위치 → 접근 자세 |

순서를 뒤집지 않는다. **RGB SmolVLA 베이스라인 → pointmap 추가 → 같은 태스크 성공률 before/after.** 이게 3단 판정에 그대로 맞는 층이고, 실패해도 베이스라인이 남는다.
