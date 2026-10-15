# 실행 소스 지도

팀 공용 소스는 이 저장소에서 수정한다. 기능별 폴더 안의 ROS 패키지명과 설치된 실행 파일 이름은 유지한다.

| 경로 | 구현 | 시작할 파일 |
|---|---|---|
| navigation/ | Cartographer·AMCL·Nav2·면 정렬·정밀 주차·4WD·LiDAR | [주행 소스](navigation/README.md), [사용법](../docs/guides/JDAMR_RUNTIME.md) |
| manipulation/ | DLS IK·파지·붓기·복귀·엔코더 실행 계획 | [픽앤플레이스](manipulation/ik_pick_place.py), [상대 IK](manipulation/bench_relative_ik.py) |
| perception/ | 카메라/작업면 보정·컵 추적·액체 충전율 | [컵·액체 인식](perception/water_detection/README.md) |
| mission/ | 주문 상태기계·SQLite·실행 backend·ROS Action | [상태기계](mission/service_mission_control.py), [Action 패키지](mission/hold_flow_mission/) |
| description/ | 양팔 URDF/Xacro·기계 설정·검증 | [모델 패키지](description/hold_flow_description/README.md) |

실차 SLAM은 `src/navigation/jdamr_cube_cartographer/`, 저장 지도 주행·주차는
`src/navigation/jdamr_cube_navigation/`에 있다. 운영 명령은 루트의 `tools/jdamr/`를 사용한다.
원본 JD-AMR 저장소는 과거 이력이며 이후 수정 기준은 양팔 저장소다.

## ROS 패키지

`description/hold_flow_description`, `mission/hold_flow_interfaces`,
`mission/hold_flow_mission`과 `navigation/`의 기존 JD-AMR 패키지를 colcon이 찾는다.
폴더 이름을 ROS 패키지 이름으로 바꿔 호출하지 않는다.

```bash
cd "$HOME/bimanual-robot"
source /opt/ros/jazzy/setup.bash
python3 tools/jdamr/runtime.py build-pc
source .runtime/jdamr/pc_ws/install/setup.bash
```

실차 전륜축 원점과 양팔 모델 차체 중심 원점은105 mm 차이가 있다.
같은 TF에 두 모델을 동시에 발행하거나 좌표·카메라 자세를 혼용하지 않는다.
카메라 높이는 바닥 기준1260 mm이며 실차와 설계의 x·y·장착각까지 같은 뜻은 아니다.

## Python 실행

공용 Python 코드는 저장소 루트에서 모듈로 실행한다. 옮긴 파일을 옛 tools에 복제하거나
여러 경로를 순회하는 import 우회 사본을 만들지 않는다.

```bash
cd "$HOME/bimanual-robot"
python3 -m src.manipulation.ik_pick_place --help
python3 -m src.perception.workspace_plane_calibration --help
python3 -m apps.service_order.service_order_server --help
```

웹 화면은 [apps](../apps/README.md), Isaac 실행은 [simulation](../simulation/README.md),
하드웨어·운영 도구는 [tools](../tools/README.md)에 있다.
검사는 루트 pytest.ini의 범위와 ROS 패키지의 test/를 구분해 실행한다.

## 통합 상태

미션은 ExecuteManipulationSkill Action과 Unix socket으로 시뮬 실행 결과·feedback·취소·timeout을 연결한다.
실차 도착과 실물 양팔 완료 신호의 주문 미션 연결, 물 양 정지·충전 시작·반복 주문 실물 검증은 남아 있다.
구현된 모듈과 예정 통합을 구분하며 시뮬·모의 실행을 실물 전체 미션 성공으로 기록하지 않는다.
