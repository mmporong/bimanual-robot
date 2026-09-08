# HOLD THE FLOW v0.3 CAD

이 디렉터리는 300×300 mm 상판과 720 mm 작업 높이 이동형 양팔 베이스의 파라메트릭 CAD 원본과 생성물을 담는다. 시스템 외곽·배치·판 두께·팔 어댑터·기둥·카메라 마스트·라이다 받침 치수의 단일 원본은 `../mechanical/hold_flow_mechanical_v0_3.yaml`이다. 장공·경량 포켓·구동 캐리어처럼 제작 방식에 속하는 세부 치수는 생성기 안에 있으며 실측 게이트에서 확정한다.

## 생성 범위

- 300 mm 하판·중판·8 mm 상판
- 상판 지지용 20×20×644 mm 중공 기둥 4개
- SO-101 교체형 어댑터 2개
- 후방 중앙 20×20×240 mm 카메라 마스트와 Astra S 거치대
- LDS-03을 z=165 mm에 두는 80 mm 받침대
- JD-AMR 휠·C018용 측면 캐리어
- 앞·뒤 볼캐스터 공용 어댑터와 1/2/3 mm shim

SO-101 팔과 그리퍼 원본은 이 생성기가 다시 그리지 않는다. ROS 모델은 `../../src/hold_flow_description`의 SO-101 메시, 왼쪽 기본 moving jaw, 오른쪽 ggao50 치수 프록시를 쓴다.

## 재생성

```bash
cd "$HOME/bimanual-robot"
"$HOME/.local/bin/uv" venv --python 3.11 "$HOME/.cache/bimanual-cad-venv"
"$HOME/.local/bin/uv" pip install \
  --python "$HOME/.cache/bimanual-cad-venv/bin/python" cadquery pyyaml
"$HOME/.cache/bimanual-cad-venv/bin/python" design/cad/generate_hold_flow_cad.py
```

결과는 `exports/step`, `exports/stl`, `exports/manifest.json`에 생긴다. URDF가 참조하는 CAD STL은 `../../src/hold_flow_description/meshes/cad`에도 같은 바이트로 복사된다. STEP 헤더 생성 시각을 정규화하므로 같은 입력의 재생성 diff가 흔들리지 않는다.

## 2026-09-08 생성 결과

- 모든 BREP 유효
- 상판: 약 300.017×300.017×8.017 mm STL 경계
- 상판 지지기둥: 20.002×20.002×644.002 mm, 4개
- 부품별 명목 재료 밀도를 적용한 구조 질량 합: 2,866.2 g
- 300 mm 판 3종과 644 mm 기둥은 K1 Max 수납 불가 또는 안전 여유 없음
- FDM 후보 중 어댑터·카메라 부품·라이다 받침대·구동 캐리어·캐스터 부품은 K1 Max 안전 경계 안

상판은 K1 Max 300 mm 명목 폭과 같고 CAD 허용오차까지 더해지므로 한 장 출력 부품으로 취급하지 않는다. 8 mm 절삭 판재가 기준이다. 하판·중판은 통합 벽 때문에 현행 형상을 한 장 출력할 수 없으므로 분할 설계 또는 절삭 판재+별도 브래킷으로 바꾸기 전에는 제작 준비 완료가 아니다.

`estimated_total_material_mass_g`는 중공 CAD 형상에 명목 밀도를 곱한 값이다. FDM infill, 실제 판재, 실제 2020 압출재의 질량을 대신하지 않는다.

## 컵 오버캡

왼쪽 기본 SO-101 죠용 오버캡은 별도 생성한다.

```bash
cd "$HOME/bimanual-robot"
"$HOME/.cache/bimanual-cad-venv/bin/python" \
  design/gripper/generate_so101_cup_overcaps.py \
  --cup-diameter 70 --radial-clearance 0.8
python3 "$HOME/.codex/skills/3d/scripts/analyze_stl.py" \
  design/gripper/exports/cup_overcaps/stl/so101_cup_overcap.stl
```

현재 70 mm 기본형은 BREP 유효, 약 24.5×44.0×30.0 mm, 고체 PETG 22.0 g/개다. 분석기는 스트랩 슬롯 때문에 서포트 필요로 판정했고 `Y-90`을 제안했지만 회전은 적용하지 않았다. 실물 죠와 컵을 재기 전에는 fit coupon 단계다.

## M1 공차 쿠폰과 기존 슬라이서

기존 M1 쿠폰과 250 mm 설계용 PETG 슬라이서 도구는 이력으로 남아 있다. v0.3 전체 구조를 그대로 슬라이싱하는 용도가 아니다.

```bash
cd "$HOME/bimanual-robot"
"$HOME/.cache/bimanual-cad-venv/bin/python" design/cad/generate_m1_coupons.py
```

실제 출력 전에는 프린터·노즐·재료를 확정하고 각 STL을 다시 분석한다. 한 번에 `--all`로 기존 구조를 슬라이싱하지 않는다.

## 실측 게이트

1. 상판·하판·중판 재료, 판 두께, 평탄도와 제작 방식
2. 2020 압출재 단면·선형 질량·브래킷·체결 위치
3. SO-101 베이스 체결공 중심거리와 M 규격
4. C018 혼, JD-AMR 휠 허브, 외부 베어링 축 공차
5. 앞·뒤 볼캐스터 플랜지, 접촉 높이, preload
6. Astra S 하단 체결과 실제 광학 중심
7. LDS-03 하부 체결공과 레이저 스캔면 높이
8. 컵 외경·테이퍼와 기본 죠·오버캡 스트랩 간섭

실측 후 YAML과 생성기 상수를 함께 고치고 STEP/STL·URDF·Isaac 검증을 다시 수행한다.
