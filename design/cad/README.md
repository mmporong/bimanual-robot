# HOLD THE FLOW 출력용 CAD

이 디렉터리는 250×250 mm 이동형 양팔 베이스의 파라메트릭 CAD 원본과 생성물을 담는다. 원본은 `generate_hold_flow_cad.py`, 치수 기준은 `../mechanical/hold_flow_mechanical_v0_2.yaml`이다.

## 생성 범위

- K1 Max 한 번에 출력 가능한 하판·중판·상판
- SO-101 두 대를 상판 안쪽에 고정하는 교체형 어댑터
- 중앙 후방 단일 카메라 마스트 3분할 구조와 내부 이음관
- Orbbec Astra S 거치대
- LDS-03을 차체 전방 중앙의 165 mm 스캔 높이에 두는 저상 받침대
- JD-AMR 휠과 Feetech C018을 연결하기 위한 측면 캐리어
- JD-AMR 볼 캐스터 어댑터와 1/2/3 mm 높이 보정판

SO-101 팔과 ggao50 평행 그리퍼 자체는 검증된 공개 형상을 다시 그리지 않는다. ROS 모델은 `../../src/hold_flow_description`에 벤더링한 공개 메시를 사용한다.

## 재생성

프로젝트 전용 환경을 만든 뒤 다음 명령을 실행한다.

```bash
cd "$HOME/bimanual-robot"
uv venv --python 3.11 "$HOME/.cache/bimanual-cad-venv"
uv pip install --python "$HOME/.cache/bimanual-cad-venv/bin/python" \
  cadquery==2.6.1 pyyaml==6.0.3 trimesh==4.12.2 scipy shapely networkx
"$HOME/.cache/bimanual-cad-venv/bin/python" design/cad/generate_hold_flow_cad.py
```

출력은 `exports/step`, `exports/stl`, `exports/manifest.json`에 생긴다. URDF가 참조하는 8종은 `../../src/hold_flow_description/meshes/cad` 복사본도 같이 갱신한다. `hold_flow_printed_structure.step`은 판과 어댑터의 조립 위치를 확인하는 간이 조립체다.

M1 공차 쿠폰은 같은 환경에서 따로 생성한다. 출력 순서 1번이라 차체보다 먼저 뽑는다.

```bash
"$HOME/.cache/bimanual-cad-venv/bin/python" design/cad/generate_m1_coupons.py
```

결과는 `exports/step`, `exports/stl`과 `exports/manifest_m1_coupons.json`에 생긴다.

## 슬라이싱

`slice_k1max_petg.py`가 인계 문서 5.2절 시작 프로파일을 OrcaSlicer 시스템 프리셋 위에 얹어 K1 Max용 G-code를 만든다. 이쪽은 전용 venv가 아니라 시스템 python3로 실행한다.

```bash
python3 design/cad/slice_k1max_petg.py --out "$HOME/gcode" --all
python3 design/cad/slice_k1max_petg.py --out "$HOME/gcode" --infill 20% design/cad/exports/stl/coupon_m1_*.stl
python3 design/cad/slice_k1max_petg.py --emit-presets
```

`--emit-presets`는 `slicer/`에 OrcaSlicer GUI로 가져갈 수 있는 프로세스·필라멘트 프리셋을 남긴다. OrcaSlicer CLI가 필라멘트 상속을 풀지 못해 PETG를 PLA `200 °C`로 내보내는 문제가 있어, 스크립트가 상속을 미리 해석해 값을 명시한다.

## P0 검증 결과

- 출력 부품 14종 모두 B-Rep 유효
- K1 Max `300×300×300 mm` 빌드 볼륨에 모두 수납 가능
- 45° 오버행 기준 원본 방향에서 14종 모두 서포트 불필요
- STEP 생성 시각을 정규화해 연속 재생성 해시 일치
- 완전 고체 PETG 체적 상한 `2,243.6 g`; 최종 질량은 K1 Max 슬라이서와 실측으로 교체
- 마스트 세그먼트 3개와 이음관 2개 합계 `504.3 g`
- Astra S 310 g과 거치대 85.3 g 합계 `395.3 g`, 장착부 400 g 게이트 통과

G-code는 아직 만들지 않는다. 아래 실측 게이트와 K1 Max의 실제 노즐·PETG 프로파일이 확정된 뒤 슬라이싱한다.

## 확정 전 실측 게이트

다음 다섯 항목은 공개 사진이나 추정 치수로 원형공을 고정하면 조립 실패 위험이 크므로 장공·교체 어댑터로 남겼다.

1. SO-101 베이스 체결공 중심거리와 M 규격
2. C018 혼, JD-AMR 휠 허브, 외부 베어링을 연결하는 축 길이와 공차
3. JD-AMR 볼 캐스터의 플랜지 홀 패턴과 바닥 접촉 높이
4. Astra S 하단 M6 위치와 실제 본체 외곽
5. LDS-03 하부 체결공과 실제 레이저 스캔면 높이

실측 후에는 생성기 상수만 고치고 STEP/STL과 URDF 보정값을 함께 재생성한다. 체결공은 FDM 수축을 고려해 첫 출력에서 공칭보다 0.2~0.4 mm 크게 두고, 베어링 자리와 축 중심은 테스트 쿠폰으로 보정한다.

## 후가공 기준

- 마스트 세그먼트: 양 끝에서 20 mm 지점을 조립 상태로 고정한 뒤 4.2 mm 관통 드릴링
- 마스트 이음관: `38.2 × 28.2 mm` 외형을 `39 × 29 mm` 내부에 면당 0.4 mm 유격으로 30 mm씩 삽입하고 체결공을 동시 천공
- 하판 주행 지지벽: `drive_side_carrier`를 드릴 지그로 대고 M4 체결공 4개를 천공
- 베어링 중심: 캐리어의 16.1 mm 기준공을 테스트 출력 후 사용하는 625 베어링 외경에 맞춰 리머 가공

이 구멍들을 수평 원형공으로 그대로 출력하면 상부가 처지고 내부 서포트가 남아 축 정렬 오차가 커진다. 따라서 상부 개방 슬롯과 출력 후 동시 천공 방식을 사용한다.
