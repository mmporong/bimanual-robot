# hold_flow_description

450×340 mm 상·하부 프레임, 4분할 상판, 720 mm 작업 높이, SO-101 두 대, 왼쪽 SO-101
순정 회전식 죠, 오른쪽 ggao50 평행 그리퍼 프록시, Astra S, LDS-03 센서 프록시, 4륜 구동을 하나의
TF 트리로 묶는 ROS 2 Jazzy 설명 패키지다. 2026-09-22 결정에 따라 왼손은 TPU FinRay를
사용하지 않으며, 이 패키지의 스톡 죠 형상이 현행 하드웨어 기준이다.

## 좌표 기준

- `base_footprint`: 차체 중심의 바닥
- `base_link`: `base_footprint`에서 z=32.9 mm인 구동륜 축 높이
- 상판: 450(W)×340(D)×6 mm 4분할, 윗면 z=720 mm
- 좌우 팔 베이스: `(20, ±170, 726) mm`, 450 mm 상판의 좌우 끝에 배치하고 전면에서 150 mm
- Astra S depth optical 중심: `(-110, 0, 1260) mm`, 상판보다 540 mm 위. 상판 z=720 mm와 상대 높이는 사용자 확인값
- LDS-03 후보 중심: `(100, 0, 165) mm`
- 전륜 중심: `(105, ±255, 32.9) mm`, 앞끝–축65 mm·좌우 중심거리510 mm의 기존 실측 반영
- 후륜 중심: `(-105, ±255, 32.9) mm`, 사용자 확인 대칭 장착·축간210 mm 환산값이며 별도 줄자 계측은 아님
- 구동륜4개, 볼캐스터0개. 전좌ID2·전우ID1·후좌ID3·후우ID4

모델 `base_footprint`는 차체 중심이고 실차 JD-AMR은 전륜축 바닥 투영을 원점으로 쓴다.
모델 좌표는 실차 좌표에 x=105 mm를 더한 값이며 이번 수정으로 실차 TF를 바꾸지 않는다.
4륜 스키드스티어의 유효 트레드는 기하510 mm와 구분해 적재 상태에서 보정한다.
상판 윗면의 바닥 기준720 mm와 카메라의 상판 기준 높이540 mm를 사용자가 확인했다.
카메라 x·y는 기존 설계값을 유지한다. 이 양팔 모델의35° 하향 자세와 JD-AMR 주행
모델의 별도 실차 장착각은 서로 다른 범위다. 방향·G4 모델은 실측 후 맞춘다.
최종4륜 선택이 모든 외부 보정 완료를 뜻하지 않는다.

## 형상과 라이선스

`third_party/so_arm_101`과 `meshes/so101`은 TheRobotStudio SO-ARM100/101의 Apache-2.0 형상이다. 관절 원점과 한계는 로컬 JD-AMR에서 사용한 SO-101 설명을 유지한다. 왼쪽 moving jaw도 같은 범위로 포함했다.

오른쪽은 [ggao50 SO101-Parallel-Gripper](https://github.com/ggao50/SO101-Parallel-Gripper) 기준이다. 2026-09-08 확인 당시 라이선스 표기가 없어 원본 메시를 벤더링하지 않고, 공개 STL에서 측정한 치수로 만든 프로젝트 프록시를 `urdf/ggao50_gripper.xacro`에 둔다. 현행 프록시는 **순정 평면 죠를 홈·교체형 인서트 없이 사용**하는 구성이다. 스트로크·작업공간·충돌 포락선 검토용이며 제조 원본과 같은 접촉면을 보장하지 않는다.

왼쪽 컵 오버캡과 TPU 손가락은 기본 URDF에 붙이지 않는다. 생성기와 과거 후보 자료는
의사결정 이력으로만 보존한다.

## 빌드와 검증

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hold_flow_description --symlink-install
python3 src/hold_flow_description/scripts/validate_description.py
python3 src/hold_flow_description/scripts/audit_urdf_quality.py --strict
python3 src/hold_flow_description/scripts/validate_isaac_contract.py
python3 src/hold_flow_description/scripts/import_isaac_sim.py --check-only
```

검사는 Xacro 확장, `check_urdf`, 커밋 URDF 일치, 메시 존재·CAD 복사본 해시, 양의 질량·관성, 관절축·hard/soft limit·dynamics, 핵심 좌표, 상판4개·구동륜4개·캐스터 부재·기둥 수, 오른쪽 mimic 관절과 커스텀 구조의 primitive collision을 확인한다.

현행4륜 모델은58링크·57관절이며 구동륜4개를 포함한 축 관절은17개다. 0자세 명목 질량은7.479 kg이다. 추가 서보·체결부의 실측 질량을 모두 분리한 값은 아니다. 이 결과는 정적 모델 일관성 증거이며 Isaac Sim 물리 안정성이나 실물 안전 증거가 아니다.

## Isaac Sim 6.0

일반 Python에서는 정적 검사만 실행한다. Isaac Sim 설치 장비에서는 Isaac의 `python.sh`로 USD를 만든다.

```bash
cd "$HOME/bimanual-robot"
/path/to/isaac-sim/python.sh \
  src/hold_flow_description/scripts/import_isaac_sim.py \
  --usd-dir "$HOME/bimanual-robot/build/isaac/hold_flow"
```

가져오기 옵션과 사후 접촉·gain 검증표는 `config/isaac_sim_6.yaml`에 있다. 커스텀 판·기둥·받침대는 CAD visual + 볼록 primitive collision을 쓰고, SO-101은 원본 메시 collision을 유지한다.

## 아직 측정해야 하는 값

- 4분할 상판 출력 조건·실제 질량·평탄도와 2020 압출재 제품
- SO-101 베이스 체결공
- C018 혼–휠–외부 베어링 축 조합
- 후륜축 위치·축간거리, 네 바퀴 접촉 높이·하중 분배·4륜 유효 트레드
- Astra S 장착점·intrinsic·extrinsic
- 왼쪽 순정 죠 ID 6 재캘리브레이션과 마른·젖은 컵 파지
- 오른쪽 ggao50 손목 변환과 정확한 충돌 메시
- 물 서빙 작업 자세별 팔↔팔·팔↔상판·팔↔카메라 충돌

전체 배치·무게·JDAMR 바퀴 서보 재사용·Pi 5 후순위안·가져오기 검증 경계는 `docs/20260910_450x340_4분할상판_URDF_IsaacSim_모델.md`에 있다.
