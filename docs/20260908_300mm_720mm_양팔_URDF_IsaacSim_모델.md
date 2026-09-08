# 300×300 mm·720 mm 양팔 로봇 URDF와 Isaac Sim 모델

- 기준일: 2026-09-08
- 단일 기구 사양: `design/mechanical/hold_flow_mechanical_v0_3.yaml`
- 현행 로봇 모델: `src/hold_flow_description/urdf/hold_flow.urdf.xacro`
- 목표 시뮬레이터: NVIDIA Isaac Sim 6.0
- 상태: URDF·CAD·정적 Isaac 가져오기 계약 검증 완료, 실제 USD·접촉 동역학 검증은 Isaac 장비 필요

## 1. 이번 모델에서 확정한 배치

좌표는 `base_footprint`를 차체 중심의 바닥에 두고 `+X 전방, +Y 좌측, +Z 위`를 쓴다.

| 항목 | 좌표·치수 | 모델 처리 |
|---|---:|---|
| 상판 | 300×300×8 mm | 바닥 z=712, 윗면 z=720 mm |
| 왼팔 SO-101 베이스 | `(0, 75, 726) mm` | 기본 SO-101 회전식 죠 포함 |
| 오른팔 SO-101 베이스 | `(0, -75, 726) mm` | ggao50 평행 그리퍼 치수 프록시 포함 |
| RGB-D 광학 중심 | `(-110, 0, 970) mm` | 상판보다 250 mm 위, 35° 아래 방향; 마스트는 8 mm 보강판 위 z=728 mm에서 시작 |
| LDS-03 후보 중심 | `(100, 0, 165) mm` | 중판의 80 mm 받침대 위 |
| 왼쪽/오른쪽 구동륜 | `(0, ±160, 32.9) mm` | 연속 회전 관절, 바퀴 중심거리 320 mm |
| 앞/뒤 볼캐스터 | `(±120, 0, 12.5) mm` | 고정 구 접촉 프록시 |
| 상판 지지기둥 | `(±130, ±130) mm` | 20×20×644 mm 중공 단면 4개 |

720 mm는 실측 평균값으로 단정한 수치가 아니다. 미국 Access Board가 제시하는 접근 가능한 식사·작업면 710~865 mm 범위 안에서 일반 책상 높이를 대표하도록 고른 설계 기준이다. 실제 손님 테이블과 주방 작업대 높이를 측정하면 그 값이 우선한다.

- 근거: [US Access Board, Chapter 9](https://www.access-board.gov/ada/ada-ibc-comparison/chapter-9/)

SO-101 베이스 폭 72 mm를 기준으로 두 팔 중심 간격을 150 mm로 두었다. 상판 양옆 여유는 각각 39 mm, 두 베이스 사이 간격은 78 mm다. 이는 외곽 포락선의 평면 배치가 겹치지 않는다는 뜻이지, 움직이는 팔과 카메라·상판·다른 팔의 모든 자세가 안전하다는 뜻은 아니다.

## 2. URDF가 시뮬레이션용으로 갖춘 것

단순 외형 그림이 아니라 물리 엔진에 넣기 위한 항목을 포함했다.

1. 모든 물리 링크에 양수 질량과 positive-definite 관성을 넣었다.
2. SO-101 관절 원점·축·hard limit는 로컬 JD-AMR에서 사용한 보정 URDF를 유지했다.
3. 회전·직선 관절에는 effort, velocity, damping, friction을 넣었다.
4. 바퀴 2개는 `continuous`, 왼쪽 기본 죠는 `revolute`, 오른쪽 평행 죠는 `prismatic + mimic`으로 분리했다.
5. 상판·판·기둥·받침대의 시각 형상은 CAD STL을 쓰되 충돌은 단순 볼록 박스로 만들었다. 얇고 복잡한 삼각형 충돌보다 실시간 접촉이 안정적이고 계산량이 작다.
6. SO-101 팔은 Apache-2.0 원본 메시를 시각·충돌에 사용한다. Isaac에서 실시간 배율이 낮거나 접촉이 불안정하면 이 메시만 convex decomposition으로 교체한다.
7. 카메라 광학 프레임은 ROS optical convention인 `+Z forward, +X right, +Y down`을 따로 둔다.
8. 상판·팔·카메라·라이다·바퀴·캐스터·기둥 좌표를 YAML과 확장 URDF 사이에서 자동 대조한다.

Isaac Sim 6.0 공식 URDF importer는 URDF를 USD로 바꾸고 rigid body, joint, collision 후처리를 수행한다. 이 저장소의 가져오기 스크립트도 공식 `URDFImporterConfig → URDFImporter → import_urdf()` 경로만 사용한다.

- [Isaac Sim 6.0 URDF Importer API](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/index.html)
- [Isaac Sim 6.0 URDF 가져오기 튜토리얼](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/importer_exporter/import_urdf.html)

## 3. 좌우 그리퍼 처리

### 3.1 왼팔: 기본 SO-101 죠부터 사용

왼팔에는 기본 SO-101 회전식 moving jaw를 복원했다. moving jaw의 질량·관성·메시·관절 원점·범위는 로컬 JD-AMR 설명에서 가져왔고, 형상은 TheRobotStudio의 Apache-2.0 자산이다.

컵용 최종 그리퍼는 아직 확정하지 않았다. 지금 만든 것은 기본 죠에 각각 따로 묶는 동일한 오버캡 2개다.

- 기본 설계 컵 외경: 70 mm
- 파라메트릭 범위: 60~82 mm
- 접촉면: 컵 외주를 따르는 오목 면
- 고정: 각 오버캡의 4 mm 스트랩 슬롯 2개
- 하단: 젖은 컵이 미끄러질 때 낙하를 늦추는 3 mm 얕은 턱
- 금지: 두 죠를 하나의 출력물로 단단히 연결하는 구조

생성 명령:

```bash
cd "$HOME/bimanual-robot"
"$HOME/.cache/bimanual-cad-venv/bin/python" \
  design/gripper/generate_so101_cup_overcaps.py \
  --cup-diameter 70 --radial-clearance 0.8
```

산출물:

- `design/gripper/exports/cup_overcaps/step/so101_cup_overcap.step`
- `design/gripper/exports/cup_overcaps/stl/so101_cup_overcap.stl`
- `design/gripper/exports/cup_overcaps/manifest.json`

3D 분석기는 원래 자세도 스트랩 슬롯에 서포트가 필요하다고 판정했고 `Y-90`을 대안으로 제안했다. 회전은 적용하지 않았다. 실제 출력 전에는 죠 접촉면·스트랩 간섭을 재고, 필요하면 슬롯 형상을 바꾼 뒤 다시 분석한다. 현재 부품은 fit coupon과 시뮬레이션 검토용이며 물이 든 컵의 안전 파지를 승인한 부품이 아니다.

### 3.2 오른팔: ggao50 평행 그리퍼

오른팔은 사용자가 고른 [ggao50/SO101-Parallel-Gripper](https://github.com/ggao50/SO101-Parallel-Gripper)를 기준으로 한다. 공개 사양은 최대 개구 105 mm, 도달 72 mm, 약 165 g이다.

2026-09-08 확인 당시 상류 저장소에는 라이선스 파일이나 SPDX 표기가 없었다. 그래서 원본 STEP/STL은 이 저장소에 복제하지 않았다. URDF에는 공개 STL에서 측정한 레일·연결판·랙·죠·접촉 인서트 크기로 직접 만든 프록시를 넣었다. 프록시 명목 질량은 159 g이고 상류의 약 165 g 표기와 6 g 차이가 있으므로 실물 계량 뒤 교체한다. 관절 스트로크와 충돌 포락선 검토에는 쓸 수 있지만, 접촉면의 모서리·기어 물림·탄성까지 재현하는 제조 원본은 아니다.

정확한 메시가 필요한 Isaac 실험은 다음 중 하나가 해결된 뒤 진행한다.

1. 저작권자에게 재배포·사용 허가를 받아 라이선스와 함께 넣는다.
2. 사용자가 내려받은 원본을 Git 비추적 로컬 자산으로만 연결한다.
3. 같은 인터페이스의 자체 그리퍼 형상을 새로 설계한다.

## 4. CAD와 제작 해석

`design/cad/generate_hold_flow_cad.py`가 v0.3 YAML에서 다음을 재생성한다.

- 300 mm 하판·중판·상판
- 상판 지지용 20×20×644 mm 중공 기둥
- SO-101 어댑터 2개
- 카메라 보강판·240 mm 마스트·Astra 거치대
- LDS-03 받침대
- 구동측 캐리어와 볼캐스터 어댑터·높이 shim

300 mm 판은 K1 Max의 명목 XY 한계와 같다. CAD 커널 허용오차까지 포함한 STL 경계는 약 300.017 mm라서 한 장 FDM 출력 불가로 판정된다. 상판은 8 mm 절삭 판재로 잡았고, 하판·중판도 실제 제작에서는 분할 출력이나 절삭 판재 전환을 결정해야 한다. ‘모델이 생성됐다’와 ‘그대로 출력할 수 있다’를 분리한다.

재생성:

```bash
cd "$HOME/bimanual-robot"
"$HOME/.cache/bimanual-cad-venv/bin/python" design/cad/generate_hold_flow_cad.py
```

## 5. Isaac Sim 가져오기

일반 개발 장비에서는 먼저 정적 계약을 확인한다.

```bash
cd "$HOME/bimanual-robot"
python3 src/hold_flow_description/scripts/import_isaac_sim.py --check-only
```

Isaac Sim 6.0 설치 장비에서는 Isaac의 Python으로 실행한다.

```bash
cd "$HOME/bimanual-robot"
/path/to/isaac-sim/python.sh \
  src/hold_flow_description/scripts/import_isaac_sim.py \
  --usd-dir "$HOME/bimanual-robot/build/isaac/hold_flow"
```

스크립트는 먼저 정적 검증을 실행하고, `package://` 메시 URI를 빌드 디렉터리의 Isaac 전용 중간 URDF에서 절대 경로로 바꾼 뒤 USD를 만든다. 가져오기 설정은 `src/hold_flow_description/config/isaac_sim_6.yaml`에 있고, `collision_from_visuals`, `merge_mesh`, `merge_fixed_joints`, `allow_self_collision`, `fix_base`, `robot_type`을 Isaac 6.0 importer에 그대로 전달한다. 지원하지 않는 필드가 있으면 가져오기를 중단한다.

USD 생성 후 반드시 별도로 확인할 것은 다음이다.

1. ground plane에서 두 바퀴와 두 캐스터가 모두 닿는지 확인한다.
2. 바퀴에는 높은 마찰, 고정 구 캐스터 프록시에는 낮은 마찰 재질을 준다.
3. 팔 position drive와 바퀴 velocity drive gain을 실제 articulation에서 조정한다.
4. 120 Hz 후보에서 60초 동안 폭발·침하·자발 이동이 없는지 본다.
5. 직진·제자리 회전으로 유효 wheel separation을 보정한다.
6. zero·운반·컵 파지·물통 파지·붓기·복귀 자세의 자가충돌과 여유를 다시 잰다.
7. RGB-D 내부 파라미터와 base→optical 외부 파라미터를 실측 보정한다.

볼캐스터는 URDF의 고정 구로만 표현했다. 실제 볼의 3축 자유 회전을 URDF 조인트 하나로 정확히 표현할 수 없으므로, 저마찰 접촉 프록시로 주행을 먼저 검증한다. 작은 회전 저항과 진동까지 필요하면 USD 단계에서 전용 caster assembly로 교체한다.

## 6. 현재 검증 결과

2026-09-08 로컬 결과:

| 검사 | 결과 |
|---|---|
| Xacro 확장과 `check_urdf` | PASS |
| 링크/관절 수 | 46 / 45 |
| 질량·관성·관절한계·메시 경로 감사 | PASS, 문제 0건 |
| YAML↔URDF 핵심 좌표 16건 | PASS |
| URDF 0자세 명목 질량·COM·정적 지지 여유 | 6.126 kg / `(30.1, -1.5, 439.7) mm` / 71.0 mm, 각 링크 inertial의 FK 합산; 실측 전 가정 |
| 오른쪽 mimic 관절 | `right_finger2_joint` 1개, PASS |
| 프레임 기둥/볼캐스터 수 | 4 / 2, PASS |
| 커스텀 구조 primitive collision | PASS |
| CAD BREP | 전부 유효 |
| 컵 오버캡 BREP | 유효, 24.5×44.0×30.0 mm, 22.0 g 명목 |
| 실제 Isaac Sim USD 생성 | 미실행 — 이 호스트에 Isaac Sim 없음 |

전체 로컬 검증:

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hold_flow_description --symlink-install
python3 src/hold_flow_description/scripts/validate_description.py
python3 src/hold_flow_description/scripts/audit_urdf_quality.py --strict
python3 src/hold_flow_description/scripts/validate_isaac_contract.py
python3 -m unittest discover -s tests -v
```

## 7. 아직 확정이 아닌 것

아래 항목은 시뮬레이션 모델에 명목값이 들어갔지만 실물 확정값은 아니다.

- 8 mm 상판 재질·실제 질량·평탄도
- 2020 압출재 제품·단면·선형 질량·브래킷과 체결 위치
- SO-101 베이스 홀 중심거리와 어댑터 원형공
- 왼쪽 기본 죠와 컵 오버캡의 실제 끼움·스트랩 간섭·젖은 컵 마찰
- 오른쪽 ggao50의 손목 변환과 원본 충돌 메시
- 바퀴 유효 반경·유효 중심거리·캐스터 preload
- Astra S 개별 장치의 내부 파라미터와 장착 후 외부 파라미터
- 컵·물통·선반·주방·손님 테이블 좌표와 허용오차
- 모든 작업 자세의 동적 전도·팔 사이·팔과 카메라·팔과 상판 충돌

따라서 이 모델의 현재 용도는 Isaac Sim 자산 가져오기, TF·관절·센서 배치 확인, 제어 인터페이스 연결, 충돌 모델 1차 검증이다. 물을 든 실물 파지와 최종 제작 승인은 위 측정과 실험을 통과한 뒤에만 가능하다.
