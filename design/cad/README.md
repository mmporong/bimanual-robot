# HOLD THE FLOW v0.3 CAD

이 디렉터리는 상·하부 외곽 450×340 mm, 4분할 상판, 720 mm 작업 높이 이동형 양팔 베이스의 파라메트릭 CAD 원본과 생성물을 담는다. 시스템 외곽·배치·판 두께·팔 어댑터·기둥·카메라 마스트·라이다 장착판 치수의 단일 원본은 `../mechanical/hold_flow_mechanical_v0_3.yaml`이다. 장공·경량 포켓·구동 캐리어처럼 제작 방식에 속하는 세부 치수는 생성기 안에 있으며 실측 게이트에서 확정한다.

## 생성 범위

- 450×340×6 mm 완성 상판을 이루는 PETG 분할판 4개
- 상·하부 둘레와 상판 이음부를 받치는 2020 레일
- 상판 지지용 20×20×634 mm 중공 기둥 4개
- SO-101 교체형 어댑터 2개
- 후방 중앙 20×20×240 mm 카메라 마스트와 Astra S 거치대
- 하단 배터리 스트랩 장착판과 LDS-03 조절 장착판
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

결과는 `exports/step`, `exports/stl`, `exports/manifest.json`에 생긴다. 프레임·상판·장착판 조립 STEP은 `exports/step/hold_flow_frame_structure.step`이다. URDF가 참조하는 CAD STL은 `../../src/hold_flow_description/meshes/cad`에도 같은 바이트로 복사된다. STEP 헤더 생성 시각을 정규화하므로 같은 입력의 재생성 diff가 흔들리지 않는다.

## 2026-09-10 생성 결과

- 모든 BREP 유효
- 앞쪽 상판 각 약 219.7×224.7×6.0 mm, 뒤쪽 각 약 119.7×224.7×6.0 mm
- 상판 지지기둥: 약 20×20×634 mm, 4개
- 상판 4개·배터리판·라이다판은 원본 방향에서 45도 기준 서포트 불필요
- 모든 FDM 부품은 K1 Max 안전 경계 안
- 긴 알루미늄 기둥·레일은 출력 대상이 아니므로 프린터 수납 판정에서 제외

이전 300×300 mm 한 장 상판과 하·중판 형상은 폐기했다. 네 상판은 둘레 링과 두 분할선 아래 프로파일에 체결한다. 출력 설정, 인서트 위치, 이음부 처짐과 평탄도는 제작 전 검증 대상이다.

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

1. 4분할 상판 출력 조건, 실측 질량, 이음부 평탄도와 처짐
2. 2020 압출재 단면·선형 질량·브래킷·체결 위치
3. SO-101 베이스 체결공 중심거리와 M 규격
4. C018 혼, JD-AMR 휠 허브, 외부 베어링 축 공차
5. 앞·뒤 볼캐스터 플랜지, 접촉 높이, preload
6. Astra S 하단 체결과 실제 광학 중심
7. LDS-03 하부 체결공과 레이저 스캔면 높이
8. 컵 외경·테이퍼와 기본 죠·오버캡 스트랩 간섭

실측 후 YAML과 생성기 상수를 함께 고치고 STEP/STL·URDF·Isaac 검증을 다시 수행한다.
