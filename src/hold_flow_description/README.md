# hold_flow_description

250×250 mm 이동 베이스, SO-101 두 대, ggao50 수평 평행 그리퍼, Astra S와 LDS-03 후보를 하나의 TF 트리로 묶는 ROS 2 Jazzy 설명 패키지다.

## 모델의 좌표 기준

- `base_footprint`: 두 구동 바퀴축의 바닥 투영점
- `base_link`: `base_footprint`에서 z=32.9 mm
- 차체 중심: `base_footprint`보다 x 방향으로 90 mm 뒤
- 좌우 팔 베이스: 차체 중심 기준 `(25, ±70, 119) mm`
- 카메라 광학 중심 목표: 차체 중심 기준 `(-80, 0, 800) mm`
- LDS-03 중심: 차체 중심 기준 `(90, 0, 165) mm`, 전용 29 mm 저상 받침대

바퀴 간 270 mm는 기하 중심거리다. Nav2 오도메트리에 넣을 유효 `wheel_separation`은 완성 차체로 직진·제자리 회전 시험을 한 뒤 보정해야 하며, JD-AMR의 183.6 mm 값을 복사하지 않는다.

## 공개 형상 사용

`third_party/so_arm_101`과 `meshes/so101`은 상류 TheRobotStudio SO-ARM100/101(Apache-2.0) 형상이며, 관절 오리진과 한계는 로컬 JD-AMR 실기에서 검증된 값을 그대로 쓴다. `scripts/generate_so101_arm_xacro.py`가 생성한다.

그리퍼는 [ggao50 SO101-Parallel-Gripper](https://github.com/ggao50/SO101-Parallel-Gripper)를 쓴다. 그 저장소에 라이선스 표기가 없어 원본 메시를 벤더링하지 않고, 배포된 STL을 실측한 치수로 만든 단순 형상 프록시를 `urdf/ggao50_gripper.xacro`에 둔다. 실제 출력은 상류 STL을 그대로 쓴다. 관성은 제어용 최종값이 아니라 충돌·가동범위 검토용 P0 값이다.

## 빌드와 검증

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hold_flow_description --symlink-install
source install/setup.bash
python3 src/hold_flow_description/scripts/validate_description.py
python3 src/hold_flow_description/scripts/audit_urdf_quality.py --strict
python3 src/hold_flow_description/scripts/audit_task_pose.py
"$HOME/.cache/bimanual-cad-venv/bin/python" design/cad/audit_urdf_clearance.py
```

이 호스트처럼 사용자 Python이 ROS의 Python보다 먼저 잡히면 다음처럼 실행한다.

```bash
colcon build --packages-select hold_flow_description --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

검증은 Xacro 확장, `check_urdf`, 링크/조인트 존재, 모든 `package://` 메시의 실제 파일 존재, 바퀴 중심거리 270 mm, 좌우 평행 그리퍼 mimic joint를 확인한다. `audit_urdf_quality.py`는 양의 질량, positive-definite 관성, 관성 주축 삼각부등식, 정규화된 관절축, hard/soft limit, effort/velocity, visual/collision 짝과 메시 스케일을 별도로 검사한다.
또한 커밋된 `hold_flow.urdf`가 Xacro의 현재 확장 결과와 같은지, 카메라 광학 중심·LDS-03·좌우 팔 베이스가 설계 좌표에 놓이는지도 수치로 검사한다.

`validate_description.py`가 통과한다는 것은 구조와 fixed TF가 일관된다는 뜻이다. 물 따르기 task pose까지 가능한지는 `audit_task_pose.py`에서 별도로 확인한다. 2026-09-01 P0 재검증에서는 다음 이유로 `task_ready=false`다.

- 좌우 `tool0`, `bottle_tcp`, `cup_tcp` frame이 없다.
- 계산 문서의 task 좌표가 현재 URDF FK가 아니라 상수로 저장되어 있다.
- 2026-09-03 해소: 이전 그리퍼 후보의 팔 체인을 걷어내고 검증된 SO-101 체인으로 되돌렸다. 붓기·운반 관절값이 모두 hard limit 안에 든다.
- 2026-09-03 해소: `left_tool0`/`right_tool0`와 `bottle_tcp`/`cup_tcp` 프레임을 추가했다.
- zero joint pose는 좌우 팔 collision mesh가 교차하며 시작 자세로 사용할 수 없다.
- 운반 후보도 링크 간 최소 `18.78 mm`, 마스트 중심 반경 `31.36 mm`로 프로젝트 안전여유를 만족하지 않는다.

이 상태에서도 YAML의 `transport_joint_degrees`는 실기체에 전송하지 않는다. 2026-09-03 에 `solve_task_poses.py` 로 다시 풀어 간섭을 통과했다(교차 여유 27.35 mm, 충돌 0쌍). 남은 작업은 문서의 목표 좌표를 상수가 아니라 FK 산출값으로 대체하는 것이다.

zero 자세는 통과한다 — 교차 여유 75.60 mm, 그리퍼-반대팔 67.70 mm, 팔↔LiDAR 28.83 mm, 마스트 반경 91.66 mm, 충돌 0쌍.

## 아직 확정하지 않은 값

아래 값은 장공과 YAML의 `null`로 남겼다. 실물을 재기 전 숫자를 확정하면 출력물이 조립되지 않을 수 있다.

- SO-101 베이스 체결공 패턴
- C018 혼–JD-AMR 휠–외부 베어링 축 조합
- JD-AMR 볼 캐스터 플랜지와 접촉 높이
- Astra S 하단 M6 위치와 외곽 공차
