# hold_flow_description

450×340 mm 상·하부 프레임, 4분할 상판, 720 mm 작업 높이, SO-101 두 대, 왼쪽 기본 회전식 죠, 오른쪽 ggao50 평행 그리퍼 프록시, Astra S, LDS-03, 2륜+2볼캐스터를 하나의 TF 트리로 묶는 ROS 2 Jazzy 설명 패키지다.

## 좌표 기준

- `base_footprint`: 차체 중심의 바닥
- `base_link`: `base_footprint`에서 z=32.9 mm인 구동륜 축 높이
- 상판: 450(W)×340(D)×6 mm 4분할, 윗면 z=720 mm
- 좌우 팔 베이스: `(20, ±170, 726) mm`, 450 mm 상판의 좌우 끝에 배치하고 전면에서 150 mm
- Astra S depth optical 중심: `(-110, 0, 970) mm`, 상판보다 250 mm 위
- LDS-03 후보 중심: `(100, 0, 165) mm`
- 구동륜 중심: `(0, ±160, 32.9) mm`, 기하 중심거리 320 mm
- 볼캐스터 중심: `(±120, 0, 12.5) mm`

바퀴 320 mm는 CAD 기하 중심거리다. Nav2 오도메트리에는 완성 차체의 직진·제자리 회전으로 보정한 유효 `wheel_separation`을 넣는다.

## 형상과 라이선스

`third_party/so_arm_101`과 `meshes/so101`은 TheRobotStudio SO-ARM100/101의 Apache-2.0 형상이다. 관절 원점과 한계는 로컬 JD-AMR에서 사용한 SO-101 설명을 유지한다. 왼쪽 moving jaw도 같은 범위로 포함했다.

오른쪽은 [ggao50 SO101-Parallel-Gripper](https://github.com/ggao50/SO101-Parallel-Gripper) 기준이다. 2026-09-08 확인 당시 라이선스 표기가 없어 원본 메시를 벤더링하지 않고, 공개 STL에서 측정한 치수로 만든 프로젝트 프록시를 `urdf/ggao50_gripper.xacro`에 둔다. 스트로크·작업공간·충돌 포락선 검토용이며 제조 원본과 같은 접촉면을 보장하지 않는다.

왼쪽 컵 오버캡은 아직 기본 URDF에 붙이지 않는다. 실물 죠와 컵을 재기 전 fit coupon 단계이기 때문이다. 생성기는 `design/gripper/generate_so101_cup_overcaps.py`다.

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

검사는 Xacro 확장, `check_urdf`, 커밋 URDF 일치, 메시 존재·CAD 복사본 해시, 양의 질량·관성, 관절축·hard/soft limit·dynamics, 핵심 좌표, 상판 4개·바퀴·캐스터·기둥 수, 오른쪽 mimic 관절과 커스텀 구조의 primitive collision을 확인한다.

2026-09-10 결과는 58링크, 57관절, 축 관절 15개, 품질 문제 0건이다. 0자세 명목 질량은 7.379 kg이다. 이 결과는 정적 모델 일관성 증거이며 Isaac Sim 물리 안정성이나 실물 안전 증거가 아니다.

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
- 앞·뒤 볼캐스터 플랜지·preload·접촉 높이
- Astra S 장착점·intrinsic·extrinsic
- 왼쪽 컵 오버캡 맞춤과 젖은 컵 마찰
- 오른쪽 ggao50 손목 변환과 정확한 충돌 메시
- 물 서빙 작업 자세별 팔↔팔·팔↔상판·팔↔카메라 충돌

전체 배치·무게·JDAMR 바퀴 서보 재사용·Pi 5 후순위안·가져오기 검증 경계는 `docs/20260910_450x340_4분할상판_URDF_IsaacSim_모델.md`에 있다.
