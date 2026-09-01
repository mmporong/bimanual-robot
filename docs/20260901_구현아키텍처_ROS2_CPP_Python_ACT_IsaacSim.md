# HOLD THE FLOW 구현 아키텍처

> 문서 성격: 팀 구현 기준서 · 2026-09-01 기준
>
> 범위: 이동형 양팔 붓기 1안만 다룬다. 실물 검증 전 항목은 계획이며, 달성 결과처럼 쓰지 않는다.

기구 치수, 좌표, 체결, 질량 예산과 URDF 시작값은 [기구설계·제작 명세 v0.2](20260901_기구설계_제작명세_v0.2.md)과 [`hold_flow_mechanical_v0_2.yaml`](../design/mechanical/hold_flow_mechanical_v0_2.yaml)을 단일 원본으로 사용한다.

![HOLD THE FLOW 시스템 아키텍처](20260901_HOLD_THE_FLOW_구현아키텍처.svg)

## 1. 한 문장으로 정한 구조

**ROS 2 Jazzy가 이동·인지·조작을 Action으로 연결하고, C++이 시간 제약이 있는 궤적 실행과 안전을 맡으며, Python이 RGB-D 인지·실험 자동화·LeRobot ACT 학습을 맡는다. 시뮬레이터는 Isaac Sim 6.0과 ROS 2 Bridge를 사용한다.**

Nav2와 ACT를 한 모델로 합치지 않는다. 로봇은 병과 빈 컵을 운반 자세로 들고 지정 위치까지 Nav2로 이동하고, 마지막 정렬은 RGB-D와 AprilTag로 보정한다. 양팔 조작은 먼저 IK와 상태기계로 통과시킨 뒤 같은 조작 구간에만 ACT를 적용한다. 모든 IK·ACT·텔레옵 명령은 하나의 명령 중재기와 안전 게이트를 지나야 한다.

## 2. 개발 기준선

### 현재 확인한 로컬 환경

| 항목 | 확인값 | 프로젝트 결정 |
|---|---|---|
| 운영체제 | Ubuntu 24.04 | ROS 2·학습 코드의 기준 OS |
| ROS 2 | Jazzy 설치 확인 | 프로젝트 배포판으로 고정 |
| C++ | GCC 13.3 | `ament_cmake`, C++17 기준 |
| Python | 3.12.3 | ROS 보조 노드와 LeRobot 환경을 분리 |
| LeRobot | 0.6.1, 로컬 커밋 `bad0260a` | ACT 수집·학습·롤아웃 기준 |
| GPU | RTX 5050 Laptop, VRAM 8GB | Isaac Sim 6.0 공식 최소 VRAM 미달 |

ROS 2 Jazzy는 Ubuntu 24.04를 공식 지원하고 2029년 5월까지 지원되는 장기 유지 배포판이다. Isaac Sim 최신 공식 문서는 Ubuntu 24.04와 ROS 2 Jazzy 조합을 권장한다. 따라서 ROS 배포판을 더 최신인 비-LTS 배포판으로 올리지 않는다.

### Isaac Sim 실행 조건

시뮬레이터는 **Isaac Sim 6.0**으로 고정한다. 다만 NVIDIA가 밝힌 6.0 최소 사양은 VRAM 16GB이고 현재 확인한 노트북은 8GB다. 이 노트북에서 정상 구동된다고 가정하지 않는다.

| 실행 위치 | 역할 | 조건 |
|---|---|---|
| 현재 개발 노트북 | ROS 2 패키지, URDF, 설정, 로그 분석 | Isaac Sim 실행 성공을 완료 조건으로 삼지 않음 |
| GPU 워크스테이션 또는 수업 장비 | Isaac Sim 6.0 GUI·센서·ROS 2 Bridge | Compatibility Checker 통과, VRAM 16GB 이상 |
| 원격 GPU 장비 | headless 반복 평가와 합성 데이터 | Isaac Sim 버전·드라이버·USD 해시를 실행 기록에 고정 |

`Isaac Sim을 쓴다`는 결정과 `현재 노트북에서 돌릴 수 있다`는 주장은 다르다. S0 환경 게이트가 통과하기 전에는 Isaac Sim 실행 완료로 표시하지 않는다.

### Python 환경을 한곳에 섞지 않는다

| 환경 | 실행 대상 | 규칙 |
|---|---|---|
| ROS 2 system Python | rclpy 노드, launch, Nav2·MoveIt 도구 | `/opt/ros/jazzy/setup.bash` 기준 |
| LeRobot 가상환경 | 수집, ACT 학습, rollout, policy server | 로컬 LeRobot 0.6.1 의존성으로 고정 |
| Isaac Sim Python | USD, sensor, Replicator, standalone script | Isaac Sim이 제공하는 `python.sh` 사용 |

LeRobot 패키지를 Isaac Sim Python에 설치하거나, Isaac Sim 패키지를 ROS system Python에 넣지 않는다. 세 환경은 ROS 2 Topic·Action·Service와 파일 산출물로 연결한다. 이 경계를 지키면 NumPy·OpenCV·PyTorch·ROS Python 패키지 충돌을 별도 문제로 격리할 수 있다.

## 3. 전체 제어 흐름

```text
mission_manager.py
  ├─ NavigateToPose --------------------------> Nav2
  ├─ AlignToWorkcell -------------------------> RGB-D / AprilTag / tf2
  ├─ ExecuteBimanualSkill(mode=IK | ACT) -----> command_mux
  └─ VerifyTransfer --------------------------> FSR / 병·컵 도킹 로드셀

command_mux
  ├─ IK_TRAJECTORY: MoveIt 2가 만든 양팔 궤적
  ├─ ACT_CHUNK: ACT가 예측한 짧은 관절 목표 묶음
  └─ TELEOP: 리더 암 입력
          ↓ 한 시점에 한 소유자만 허용
safety_guard.cpp
  ├─ 관절 범위·속도 제한
  ├─ 팔 사이 최소 거리
  ├─ 명령 타임아웃·하트비트
  ├─ FSR 과압·미끄럼 징후
  └─ 위반 시 HOLD 또는 SAFE_RETURN
          ↓
bimanual_trajectory_server.cpp
  └─ 좌우 관절을 하나의 타임라인으로 보간·동기 실행
          ↓
so101_hardware_bridge.py
  └─ LeRobot 0.6.1의 bi_so_follower로 좌우 시리얼 포트 단독 소유
```

가장 중요한 규칙은 **모터 포트 소유자가 하나**라는 점이다. LeRobot, MoveIt, 텔레옵 코드가 좌우 SO-101 포트를 각각 열면 명령이 충돌한다. `so101_hardware_bridge.py`만 포트를 열고 나머지는 ROS 2 인터페이스로 요청한다.

## 4. 무엇을 C++로 만들고 무엇을 Python으로 만드는가

### C++로 구현할 부분

| 패키지/노드 | 구현 내용 | C++인 이유 | 완료 증거 |
|---|---|---|---|
| `bimanual_command_mux` | IK·ACT·텔레옵 명령의 lease와 모드 전환 | 동시 명령 차단과 상태 전환이 결정적이어야 함 | 이중 명령 주입 테스트에서 한 경로만 통과 |
| `bimanual_safety_guard` | 관절 한계, 변화량, 타임아웃, 팔 간 거리, 정지 | 실행 직전의 공통 안전 경로 | 잘못된 명령 6종 차단 로그 |
| `bimanual_trajectory_server` | 양팔 `FollowJointTrajectory` 수신, 공통 시간축 보간, 피드백·취소 | 좌우 팔을 독립 호출하지 않고 동기 실행 | goal/feedback/result, 좌우 시작 편차와 취소 시험 |
| `bimanual_kinematics` | FK·IK 검증, 양팔 작업공간·충돌 검사 | Eigen·MoveIt·tf2와 정합, 반복 계산 | 목표 pose와 실관절 FK 오차표 |
| `workcell_tf_guard` | TF 경로·시각·중복 발행자 검사 | 좌표 오류를 상위 로직과 분리 | `map→workcell` 경로와 timestamp 검사 |
| `sensor_filter` | 로드셀 필터, 안정 구간, 센서 타임아웃 | 검증값 계산을 재현 가능하게 고정 | 원시값·필터값·지연 동시 로그 |

Nav2, MoveIt 2, SLAM Toolbox, ros2_control의 핵심은 이미 C++로 구현돼 있다. 우리는 이를 다시 작성하지 않고 설정과 Action 인터페이스를 구성한다. 팔 하드웨어 드라이버까지 처음부터 C++로 다시 쓰는 것은 선행 조건이 아니다.

### Python으로 구현할 부분

| 패키지/노드 | 구현 내용 | Python인 이유 | 완료 증거 |
|---|---|---|---|
| `mission_manager` | 이동→정렬→조작→검증 상태기계 | 상태·실험 시나리오를 빠르게 변경 | 상태 전이와 실패 위치 로그 |
| `workcell_perception` | RGB-D ROI, AprilTag pose, 컵·병 관측 | OpenCV·NumPy 실험 속도 | 반복 정렬 오차표와 재생 가능한 입력 |
| `so101_hardware_bridge` | LeRobot 양팔 API, 관절 상태·명령 변환 | 현재 검증 대상 API가 Python | 좌우 포트 단독 소유와 30Hz 주기 로그 |
| `act_policy_server` | PyTorch 모델 로드, observation 전처리, action chunk 예측 | LeRobot·PyTorch 기본 경로 | 추론 지연·deadline miss·출력 shape |
| `episode_logger` | ROS 메시지·LeRobot episode·사이드카 메타데이터 연결 | Parquet/JSON/영상 도구 활용 | 하나의 `episode_id`로 전 구간 조인 |
| `evaluation` | 성공률, 진행 점수, 조건별 실패 분석 | Pandas/시각화 도구 활용 | 자동 생성 평가표 |
| `recovery_supervisor` | 규칙 기반 복구, 선택 확장으로 로컬 LLM 호출 | 구조화 출력과 실험 변경이 쉬움 | 허용되지 않은 행동 0건 |

### MCU 또는 센서 보드에서 처리할 부분

- FSR 4채널과 로드셀 2채널 원시값 읽기
- 센서 timestamp와 sequence number 부여
- 통신이 끊겨도 안전한 기본 출력 유지
- 모터 비상 정지는 ROS 메시지에만 의존하지 않고 물리 정지 회로와 분리

MCU가 `성공/실패`를 판단하지 않는다. MCU는 원시 측정과 통신 상태를 제공하고, 판정 규칙은 버전 관리되는 상위 코드에 둔다.

## 5. ROS 2 패키지 구조

```text
src/
├── hold_flow_interfaces/          # msg, srv, action 정의
├── hold_flow_description/         # 모바일 베이스+SO-101×2 URDF/xacro, SRDF
├── hold_flow_bringup/             # launch, params, lifecycle 순서
├── hold_flow_navigation/          # slam_toolbox, AMCL, Nav2 설정
├── hold_flow_perception/          # RGB-D, AprilTag, workcell pose
├── hold_flow_motion/              # MoveIt 2, IK, 양팔 trajectory server
├── hold_flow_safety/              # command mux, safety guard
├── hold_flow_hardware/            # LeRobot bridge, FSR/load-cell bridge
├── hold_flow_learning/            # ACT train/export/inference adapter
├── hold_flow_mission/             # 전체 상태기계와 복구 감독
├── hold_flow_logging/             # episode/event/metric 기록
└── hold_flow_isaac/               # USD, ROS 2 Bridge, 센서·시나리오 설정
```

실제 패키지를 만들 때는 한 Issue에서 하나씩 추가한다. 지금 표는 구현 계약이며 빈 패키지 폴더를 미리 만들지는 않는다.

## 6. SLAM과 Nav2 구현

### SLAM이 맡는 것

SLAM은 로봇이 처음 보는 공간에서 2D 점유 지도를 만들면서 자신의 위치를 추정하는 과정이다. 이 프로젝트에서는 지정 붓기 위치까지 가는 **거친 전역 좌표**를 만든다. 팔이 요구하는 수 mm 정렬을 SLAM에 맡기지 않는다.

### 선택한 구성

```text
2D LiDAR ─┐
wheel odom ├─> slam_toolbox(mapping) ─> map.yaml + map.pgm + pose graph
IMU ───────┘

저장 지도 + 2D LiDAR ─> AMCL(localization) ─> map → odom
wheel odom ────────────────────────────────> odom → base_link
Nav2 ─> 지정 붓기 위치 staging pose
```

1. 지도 작성 때만 `slam_toolbox` mapping 모드를 실행한다.
2. 데모 반복 실행에서는 `map_server + AMCL + Nav2`를 사용한다.
3. SLAM과 AMCL을 동시에 띄워 `map→odom`을 두 노드가 발행하게 하지 않는다.
4. 목표는 토픽이 아니라 `NavigateToPose` Action으로 전달해 성공·취소·실패와 복구 횟수를 기록한다.
5. staging pose에 도착하면 base를 정지하고 낮은 지정 패드의 표식에 대한 로컬 정렬 단계로 넘긴다.

베이스 속도 명령은 `Nav2 controller → velocity_smoother → collision_monitor → C018 base driver` 한 경로로만 흐르게 한다. 1차 후보는 STS3215-C018 2대의 속도 폐루프 모터 모드다. C++ `ros2_control` hardware interface가 `/cmd_vel`에서 변환된 좌우 속도, 위치 래핑을 해제한 wheel odom, 전류·온도·부하를 관리한다. mission node가 개별 속도를 직접 만들지 않는다. 최대 선속도는 0.12 m/s, 가속 0.20 m/s², 감속 0.25 m/s²다. 조작 상태에 들어갈 때 Nav2 goal을 종료하고 zero velocity 확인 뒤 팔 명령 lease를 연다.

차체 판 footprint와 병·컵을 든 footprint는 다르다. `base_footprint`가 바퀴축에 있으므로 후방 차체 모서리까지의 정적 회전반경은 약 0.249 m다. 계산 운반 자세는 병·컵 외피와 padding을 포함해 최소 0.27 m다. Nav2 costmap은 적재 상태 footprint를 사용한다.

### SLAM/Nav2 초기 통과 기준

| 게이트 | 시험 | 초기 목표 | 기록 |
|---|---|---|---|
| N0 TF | 정지 상태 5분 | 중복 TF 발행자 0, 끊긴 링크 0 | `view_frames`, topic info |
| N1 지도 | 동일 경로 3회 | 벽 중첩과 루프 클로징 육안·좌표 대조 | map, pose graph, rosbag |
| N2 위치추정 | 초기 pose 10회 | AMCL 수렴 실패 조건을 분리 기록 | 수렴 시간, covariance |
| N3 접근 | staging pose 10회 | 8회 이상 Action 성공 | 도달 오차, 시간, recovery 수 |
| N4 인계 | 접근 후 로컬 정렬 10회 | workcell이 양팔 작업공간 안에 8회 이상 진입 | base→workcell pose |

`8/10`은 첫 POC 통과 기준이지 달성 결과가 아니다. 실물 반복 오차를 본 뒤 상향한다.

## 7. RGB-D와 AprilTag 로컬 정렬

Nav2 완료 뒤에는 다음 TF 사슬을 사용한다.

```text
map → odom → base_link → camera_link → tag → workcell
                                  ├── bottle_dock
                                  ├── receiver_dock
                                  └── pour_center
```

- AprilTag는 작업 셀의 기준 자세를 만든다.
- RGB-D는 병·컵과 도킹 패드의 국소 위치를 보정한다.
- 검출 timestamp 시점의 TF를 조회한다. 최신 TF를 무조건 대입하지 않는다.
- 태그 크기·카메라 내부 파라미터·카메라 외부 파라미터를 버전과 함께 기록한다.
- 투명 컵의 depth hole을 실패로 숨기지 않고 RGB 표식 또는 고정 도킹 지그로 우회한다.

Python 프로토타입이 반복 시간 목표를 못 맞기 전까지 C++로 옮기지 않는다. C++ 포팅 기준은 처리 지연, frame drop, CPU 사용률 가운데 하나가 실제 병목으로 확인됐을 때다.

## 8. MoveIt 2, IK와 양팔 상태기계

### MoveIt 2가 맡는 것

- URDF/SRDF 기반 좌우 팔 FK·IK
- `left_arm`, `right_arm`, `both_arms` planning group
- 테이블·도킹 패드·병·컵의 Planning Scene
- 자기 충돌과 팔 사이 충돌 검사
- 궤적 생성 후 좌우 관절이 함께 들어 있는 `FollowJointTrajectory` Action 전달

MoveIt 2는 병과 컵을 인식하지 않는다. perception이 만든 목표 pose를 받아 관절 궤적으로 바꾸는 실행 계층이다.

### 결정론적 기준선

```text
APPROACH
→ ALIGN
→ GRASP_BOTH
→ VERIFY_GRASP
→ PRETILT_SLOW
→ POUR_CALIBRATED
→ RETURN_UPRIGHT
→ WEIGH_BOTH_DOCKS
→ POUR_MORE(최대 1회) 또는 PLACE_BOTH
```

- 컵 팔은 `STABILIZE`, 병 팔은 `ACT` 역할을 갖는다.
- 재안정화 중에는 병 팔 기울임을 멈춘다.
- 각 상태는 timeout, 취소, 안전 복귀 상태를 가진다.
- 결과는 로그 문자열이 아니라 관절 상태, 컵 자세, 로드셀 결과로 판정한다.

### ros2_control 적용 범위

모바일 베이스와 검증된 컨트롤러에는 ros2_control을 사용한다. SO-101 팔은 다음 두 단계로 간다.

1. **MVP**: LeRobot Python bridge가 포트를 소유하고 C++ `FollowJointTrajectory` adapter가 안전한 목표값만 전달한다.
2. **고도화**: 시리얼 protocol·정지·read/write 주기가 실측된 뒤 C++ `hardware_interface::SystemInterface`로 교체한다.

MoveIt 2는 ros2_control이 아니어도 `FollowJointTrajectory` 서버와 연결할 수 있다. 취업용 C++ 경험을 만들기 위해 검증되지 않은 드라이버를 급히 새로 작성하지 않는다. 상위 Action 계약을 먼저 고정하면 하드웨어 계층만 나중에 교체할 수 있다.

양팔 경로는 좌우 Action을 Python에서 순서대로 호출하지 않는다. `both_arms` planning group의 결과를 하나의 `control_msgs/action/FollowJointTrajectory` goal로 보내고, C++ trajectory server가 같은 `time_from_start`를 기준으로 좌우 목표를 분리해 bridge에 전달한다. 피드백과 취소도 두 팔을 하나의 실행으로 집계한다.

## 9. ACT는 무엇을 어떻게 학습하는가

### ACT의 역할

ACT(Action Chunking with Transformers)는 현재 관측에서 다음 한 동작이 아니라 여러 스텝의 행동 묶음을 예측하는 모방학습 정책이다. 이 프로젝트에서는 **양팔 조작 구간만** 학습한다.

학습 대상:

```text
GRASP_BOTH → PRETILT_SLOW → POUR → RETURN_UPRIGHT → PLACE_BOTH
```

학습하지 않는 것:

- SLAM과 Nav2
- 지정 패드의 전역 탐색
- 하드웨어 비상 정지
- 실패의 최종 성공 판정
- LLM의 자유 형식 모터 명령

### 수집 입력과 출력

| 구분 | LeRobot key | 내용 |
|---|---|---|
| 공용 시야 | `observation.images.top` | 병·컵·양팔 관계 |
| 왼손목 | `observation.images.left_wrist` | 컵 파지와 자세 |
| 오른손목 | `observation.images.right_wrist` | 병 파지와 유출 방향 |
| 관절 상태 | `observation.state` 계열 | 좌우 관절 실제 위치 |
| 행동 | `action` 계열 | 좌우 관절 목표 위치 |
| 사이드카 | `episode_metadata.json` | 용기, 배치, 매체, 실패, 계측 결과 |

LeRobot의 `bi_so_follower`와 `bi_so_leader`는 좌우 키에 `left_`와 `right_` 접두어를 사용한다. 30fps를 첫 기준으로 두되 실제 루프가 못 맞추면 fps 숫자만 유지하지 말고 achieved fps와 deadline miss를 기록한다.

Depth는 G0에서 ACT 입력으로 넣지 않는다. 로컬 정렬과 로그에는 사용하지만, ACT 첫 기준선은 동일 크기의 RGB 3시점과 관절 상태로 고정한다. 입력을 늘리는 실험은 RGB 기준선과 분리한다.

### 데이터 수집 단계

| 단계 | 목적 | 데이터 | 종료 조건 |
|---|---|---|---|
| D0 | 파이프라인 확인 | 성공 5회 | 좌우 키·영상·action/state 시간축 확인 |
| D1 | 작은 과적합 시험 | 같은 조건 성공 20회 | 학습 코드가 손실 감소·롤아웃까지 연결 |
| D2 | 기준 학습 | 조건별 균형 수집 | 중앙/오프셋과 기준/변화 용기 셀의 누락 없음 |
| D3 | 실패 보강 | 미끄럼·가림·미달 실패 | 실패 원인별 추가 시연과 재평가 |

20회는 과적합과 파이프라인 확인용이지 일반화에 충분하다는 뜻이 아니다. 본수집 규모는 D1 실패 분포를 본 뒤 늘린다.

### 학습 방식

1. 리더 암 두 개로 좌우 팔을 동시에 텔레옵한다.
2. LeRobot v3 데이터셋과 프로젝트 사이드카 메타데이터를 같은 `episode_index`로 연결한다.
3. 프레임을 무작위로 섞어 분할하지 않고 에피소드 단위로 train/validation을 나눈다.
4. 용기 또는 배치 조건 하나는 수집 전에 holdout으로 지정한다.
5. ACT는 사전학습 VLA에 LoRA를 붙이는 방식이 아니다. 첫 기준선은 LeRobot ACT를 행동 복제로 학습한다.
6. 학습 손실만 보고 채택하지 않고 실물 반복 평가로 판정한다.

공식 CLI를 기준으로 한 첫 학습 명령은 다음 형태다.

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/hold-the-flow-bimanual-v0 \
  --policy.type=act \
  --output_dir=outputs/train/act_hold_the_flow_v0 \
  --job_name=act_hold_the_flow_v0 \
  --policy.device=cuda \
  --wandb.enable=true \
  --steps=20000
```

`20000` step은 LeRobot 공식 예제의 시작값이다. 우리 데이터에서 최적임을 뜻하지 않는다. chunk size, action steps, temporal ensemble은 첫 실물 롤아웃 지연과 진동을 본 뒤 한 번에 하나씩 바꾼다. 로컬 0.6.1 기본 ACT는 `chunk_size=100`, `n_action_steps=100`, ResNet-18과 CVAE를 사용하며 temporal ensemble은 기본으로 꺼져 있다.

### 추론과 안전 연결

```text
camera + joint state
  → act_policy_server.py
  → action chunk
  → command_mux(ACT lease)
  → safety_guard.cpp
  → trajectory server
  → SO-101 bridge
```

- 추론 timeout이면 새 명령을 유지하지 않고 hold 상태로 전환한다.
- action chunk 전체를 한 번에 무검사 실행하지 않는다.
- 각 step을 joint limit, 최대 변화량, 팔 간 거리로 검사한다.
- ACT와 IK를 같은 조건에서 비교한다: 전체 성공률, 진행 점수, 목표량 오차, 흘림, 실행 시간.

## 10. Isaac Sim에서 무엇을 구현하는가

Isaac Sim의 목적은 화려한 장면이 아니라 **실물과 같은 ROS 2 인터페이스를 먼저 검증하는 것**이다.

### 시뮬레이션 구성

```text
SO-101 URDF + 모바일 베이스 URDF
  → Isaac Sim URDF Importer
  → USD robot asset
  → 지정 위치 표식·병·컵·낮은 도킹 패드 USD scene
  → RTX camera / depth / 2D LiDAR / joint state
  → ROS 2 Bridge
  → 실제와 동일한 Nav2·MoveIt·mission_manager
```

### 단계별 구현

| 게이트 | 구현 | 확인할 것 | 금지할 주장 |
|---|---|---|---|
| S0 환경 | Isaac Sim 6.0 설치·Compatibility Checker | GPU·드라이버·ROS bridge | 현재 8GB 노트북에서 지원된다는 주장 |
| S1 자산 | SO-101·베이스 URDF→USD | joint 축·limit·단위·관성·collider | import 성공만으로 실물 일치 주장 |
| S2 센서 | RGB-D, LiDAR, joint state publish | topic, frame_id, timestamp, publish rate | 센서 노이즈가 실물과 같다는 주장 |
| S3 제어 | Nav2와 MoveIt 2 연결 | Action 결과, TF, 취소, 정지 | 실물 성공률로 전용 |
| S4 시나리오 | 병·컵 pose, 조명, 질감 randomization | seed와 설정 재현 | 합성 데이터를 실물 데이터로 합산 |
| S5 gap | 같은 궤적의 sim/real 비교 | 관절 오차, 지연, 접촉, 카메라 차이 | 보정 전 sim-to-real 완료 주장 |

### Isaac Sim과 ACT의 관계

초기 ACT 기준선은 **실물 시연만** 사용한다. Isaac Sim 데이터는 다음 조건을 모두 통과한 뒤 별도 실험으로만 섞는다.

1. URDF 관절 방향과 범위가 실물과 일치한다.
2. 명령 대비 실제 관절 응답 지연을 시뮬레이터에 반영했다.
3. 카메라 intrinsics와 설치 pose를 맞췄다.
4. 합성 데이터만 학습한 정책과 실물 데이터만 학습한 정책을 분리 평가한다.
5. 혼합 학습은 `real-only` 기준선을 이길 때만 채택한다.

Replicator는 조명, 배경, 물체 pose, 재질을 seed 기반으로 바꾸고 합성 이미지를 저장하는 데 사용한다. 액체 물리의 사실성을 MVP 핵심으로 두지 않는다. G2a까지는 비드·쌀과 보정 붓기 궤적을 중심으로 검증한다.

## 11. 센서와 판정 구조

| 센서 | 책임 | 구현 위치 | 판정하지 않는 것 |
|---|---|---|---|
| 병·컵 FSR | 접촉, 과압, 좌우 불균형 | MCU raw → C++ safety | 유입량 |
| 병 도킹 로드셀 | 붓기 전후 병 감소량 | MCU raw → C++ filter | 실시간 컵 유입량 |
| 컵 도킹 로드셀 | 붓기 전후 컵 증가량 | MCU raw → C++ filter | 팔이 든 상태의 질량 |
| RGB-D | workcell과 물체 pose | Python perception | 최종 질량 |
| LiDAR | SLAM, AMCL, costmap | Nav2 stack | 파지 정렬 |
| 관절 상태 | 실행 오차, timeout, overload 징후 | bridge → C++ safety | 정밀 접촉력 |

```text
transferred_mass_g = receiver_after_g - receiver_before_g
spill_estimate_g = (source_before_g - source_after_g) - transferred_mass_g
```

두 도킹 저울이 모두 안정값을 내지 못하면 질량 기반 흘림량을 주장하지 않고 `spill_detected`만 기록한다.

## 12. ROS 2 인터페이스 계약

| 이름 | 방식 | 생산자 | 소비자 | 결과/실패 |
|---|---|---|---|---|
| `/navigate_to_pose` | Action | mission | Nav2 | 성공·취소·중단, recovery 수 |
| `/workcell/align` | Custom Action | mission | perception | workcell pose, 오차, confidence |
| `/bimanual/execute_skill` | Custom Action | mission | motion | stage, progress, failure code |
| `/bimanual_controller/follow_joint_trajectory` | 표준 Action | MoveIt 2 | trajectory server | 좌우 관절 오차, tolerance 위반, 동기 취소 |
| `/policy/predict` | Service 또는 Action | mission | ACT server | chunk, inference latency |
| `/safety/state` | transient-local Topic | safety | 전체 노드 | mode, stop reason |
| `/episode/event` | Topic | 전체 노드 | logger | timestamp, stage, code |
| `/loadcell/raw` | Topic | sensor bridge | filter/logger | raw, sequence, timestamp |

이름과 message field는 `hold_flow_interfaces` 구현 때 확정한다. 장시간 동작과 취소가 필요한 기능은 Topic이 아니라 Action을 쓴다.

## 13. 실패 복구와 로컬 LLM 경계

규칙 기반 상태기계가 먼저 동작한다. 로컬 LLM은 나중에 다음 열거형 중 하나만 선택한다.

```text
REOBSERVE
REGRASP_BOTTLE
RESTABILIZE_CUP
POUR_MORE
ABORT_AND_HOME
```

- 좌표·각도·토크·PWM을 출력하지 않는다.
- `POUR_MORE`는 최대 1회다.
- 안전 상태가 아니면 선택 결과와 관계없이 `ABORT_AND_HOME`이다.
- 입력은 자연어 영상 설명이 아니라 구조화된 실패 코드와 센서 요약이다.
- 규칙 기반과 LLM 기반의 복구 성공률을 따로 기록한다.

## 14. 파트별 구현 순서와 역할

GitHub 권한 상태와 실제 역할은 다르다. 아래는 현재 회의에서 확인된 담당과 작업 lane이다.

| lane | 우선 담당 | 구현 범위 | 다른 lane에 넘기는 계약 |
|---|---|---|---|
| Navigation & Integration | 사용자 | SLAM Toolbox, AMCL, Nav2, TF, staging pose, 전체 mission 연결 | `NavigateToPose` result와 `base→workcell` pose |
| Hardware & Gripper | 팀원 협의 | 평행 그리퍼 인서트, FSR, 병·컵 도킹 저울, 센서 보드 | timestamp가 있는 raw sensor와 calibration |
| Manipulation | 공동 | URDF/SRDF, MoveIt 2, IK, 상태기계, 양팔 충돌 | `ExecuteBimanualSkill` Action |
| Learning | 공동 | 텔레옵, LeRobot dataset, ACT train/rollout | policy artifact와 입력 feature contract |
| Simulation | 공동, 실행 장비 담당 지정 필요 | Isaac Sim 6.0, USD, ROS 2 Bridge, sim/real gap | scene version, seed, topic·TF contract |
| Supervision | 강사/멘토 | 규칙 기반 복구 검토, 선택형 로컬 LLM | 허용 행동 enum만 반환 |

[@Minsuk-ji](https://github.com/Minsuk-ji)는 저장소 write 권한을 수락했다. [@jangjunseo05](https://github.com/jangjunseo05)는 2026-09-01 확인 시 초대 수락 대기 상태다. 계정별 세부 담당은 다음 회의에서 확정해 이 표를 갱신한다.

## 15. 구현 게이트

| 게이트 | 실제로 끝내는 것 | 다음 단계로 가는 조건 |
|---|---|---|
| S0 | Isaac Sim 실행 장비와 버전 고정 | Compatibility Checker와 ROS 2 Bridge smoke 통과 |
| G0 | 병·컵 파지, FSR, 포트 단독 소유 | 정적 파지 10회 중 7회 이상 |
| G1 | IK 기울임, cup stabilize, 안전 정지 | 무수 기울임과 직립 복귀 반복 |
| G2a | 건식 붓기와 병·컵 도킹 계측 | 목표량·흘림 자동 판정 |
| D0 | LeRobot 양팔 수집 5회 | 좌우 상태·action·영상 시간축 정상 |
| A0 | ACT 20회 과적합 기준선 | 학습→롤아웃→평가 자동 연결 |
| G3 | Nav2→정렬→양팔 조작 통합 | 단계별 결과가 한 episode로 연결 |
| S3 | Isaac Sim에서 동일 ROS 경로 실행 | sim과 real이 같은 Action·TF 계약 사용 |
| G4 | 실패 6종과 제한 복구 | 감지 실패와 안전 중단 실패를 분리 보고 |
| S5 | sim/real gap 표 | 관절·지연·센서 차이를 수치화 |

## 16. 바로 만들 첫 Issue

1. `[S0][sim] Isaac Sim 6.0 실행 장비와 ROS 2 Bridge smoke test`
2. `[N0][slam] map→odom→base_link TF 단일 발행자 검증`
3. `[G0][hardware] 양팔 포트 소유권과 stop path 검증`
4. `[G0][gripper] 병·컵 인서트와 FSR 정적 파지`
5. `[G1][motion] 양팔 URDF/SRDF와 무수 기울임`
6. `[G2a][sensor] 병·컵 도킹 저울 전후 질량 판정`
7. `[D0][data] bi_so_follower 5 episode smoke dataset`
8. `[G3][integration] NavigateToPose result에서 AlignToWorkcell 인계`

## 17. 공식 근거

- [ROS 2 Jazzy · Ubuntu 24.04 지원](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html)
- [ROS 2 배포판 지원 일정](https://docs.ros.org/en/humble/Releases.html)
- [Nav2 · Mapping and Localization](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
- [Nav2 · SLAM Toolbox와 함께 주행](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [MoveIt 2 · Planning Scene](https://moveit.picknik.ai/main/api/html/planning_scene_overview.html)
- [MoveIt 2 · Low Level Controllers](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)
- [ros2_control · Jazzy 구조](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [Joint Trajectory Controller · Jazzy](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [LeRobot · 실물 로봇 모방학습](https://huggingface.co/docs/lerobot/il_robots)
- [LeRobot · ACT 학습과 rollout CLI](https://huggingface.co/docs/lerobot/main/en/cheat-sheet)
- [ACT 원 논문](https://arxiv.org/abs/2304.13705)
- [Isaac Sim · ROS 2 Jazzy 설치](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Isaac Sim · 시스템 요구사항](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim · MoveIt 2 연동](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_moveit.html)
- [Isaac Sim · URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html)
- [Isaac Sim · Replicator 합성 데이터](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_scene_based_sdg.html)

## 18. 아직 검증하지 않은 것

- Isaac Sim 6.0을 실행할 16GB 이상 GPU 장비와 설치 경로
- SO-101 URDF를 Isaac Sim으로 가져왔을 때 관절 축·관성·collider 정합성
- C018 base driver의 20분 연속주행 온도·wheel odom 래핑·실제 topic 이름
- LeRobot bridge에서 유지 가능한 양팔 achieved fps
- C++ trajectory adapter와 Python hardware bridge 사이의 end-to-end 지연
- 실제 병·컵에 맞는 FSR 임계값과 로드셀 안정 시간
- ACT의 실제 데이터 요구량, chunk 설정, 추론 지연과 성공률
- sim-only, real-only, sim+real ACT의 성능 차이

이 값은 설치 예제나 논문 수치로 채우지 않고 S0·G0~G4·D0·A0 실측으로 갱신한다.
