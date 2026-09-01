# src 구현 계약

이 디렉터리는 HOLD THE FLOW의 실행 코드를 담는다. 현재는 구현 순서와 패키지 경계를 먼저 고정하며, 빈 ROS 2 패키지는 만들지 않는다. 전체 근거는 [구현 아키텍처](../docs/20260901_구현아키텍처_ROS2_CPP_Python_ACT_IsaacSim.md)를 따른다.

## 패키지별 언어와 책임

| 패키지 | 주 언어 | 사용 기술 | 책임 | 착수 게이트 |
|---|---|---|---|---|
| `hold_flow_interfaces` | ROS IDL | msg/srv/action | 패키지 사이 계약 | 첫 코드 패키지와 함께 |
| `hold_flow_description` | xacro/YAML | URDF, SRDF | 모바일 베이스·SO-101×2·센서 TF | S1/G1 |
| `hold_flow_bringup` | Python/YAML | ROS 2 launch, lifecycle | 실행 순서와 설정 조합 | N0 |
| `hold_flow_navigation` | 기존 C++ 노드 + YAML | SLAM Toolbox, AMCL, Nav2 | 지도 작성, 위치추정, 작업대 접근 | N0~N4 |
| `hold_flow_perception` | Python | RGB-D, AprilTag, OpenCV, tf2 | `base_link→workcell` 자세 | N4 |
| `hold_flow_motion` | C++ | MoveIt 2, Eigen, FollowJointTrajectory | IK, Planning Scene, 양팔 궤적 | G1 |
| `hold_flow_safety` | C++ | rclcpp, tf2, joint limits | command mux, timeout, 충돌·한계 차단 | G0 |
| `hold_flow_hardware` | Python→C++ 선택 | LeRobot, serial, sensor bridge | SO-101 포트 소유, FSR·로드셀 입출력 | G0/G2a |
| `hold_flow_learning` | Python | LeRobot 0.6.1, PyTorch, ACT | 수집, 학습, rollout, 정책 서버 | D0/A0 |
| `hold_flow_mission` | Python | rclpy, Action client | 이동→정렬→조작→검증 상태기계 | G3 |
| `hold_flow_logging` | Python | rosbag2, JSON, Parquet | episode와 평가 지표 연결 | G0부터 |
| `hold_flow_isaac` | Python/USD | Isaac Sim 6.0, ROS 2 Bridge, Replicator | 센서·Nav2·MoveIt 경로와 sim/real gap | S0~S5 |

## 명령 경로 규칙

```text
Nav2 / MoveIt 2 / ACT / Teleop
            ↓
       command_mux
            ↓
       safety_guard
            ↓
  bimanual_trajectory_server / base controller
            ↓
     hardware bridge 또는 Isaac Sim ROS 2 Bridge
```

- 좌우 SO-101 시리얼 포트는 한 프로세스만 연다.
- MoveIt 2와 ACT가 동시에 팔 명령을 보내지 않는다.
- Isaac Sim과 실물은 같은 ROS 2 Action·Topic·TF 이름을 사용하고 launch profile만 바꾼다.
- ROS 2 system Python, LeRobot 가상환경, Isaac Sim `python.sh`를 섞지 않는다.
- Action 결과 없이 성공으로 기록하지 않는다.
- 실물 데이터, Isaac Sim 데이터, 혼합 데이터는 dataset id와 split을 분리한다.

## 첫 구현 순서

1. `hold_flow_interfaces` — `AlignToWorkcell`, `ExecuteBimanualSkill`, `EpisodeEvent`
2. `hold_flow_description` — 실제 치수 기반 TF와 좌우 planning group
3. `hold_flow_safety` — 명령 lease, joint delta, timeout
4. `hold_flow_navigation` — Nav2 Action과 작업대 staging pose
5. `hold_flow_perception` — AprilTag/RGB-D 로컬 정렬
6. `hold_flow_motion` — MoveIt 2 무수 기울임
7. `hold_flow_hardware` — FSR·도킹 로드셀
8. `hold_flow_learning` — LeRobot 5 episode smoke와 ACT 기준선
9. `hold_flow_isaac` — 같은 인터페이스의 Isaac Sim 실행과 gap 측정

## 코드 리뷰에서 반드시 확인할 것

- 단위가 degree인지 radian인지 명시돼 있는가
- `frame_id`와 timestamp가 있는가
- timeout과 취소 경로가 있는가
- 하드웨어 포트 소유자가 하나인가
- 센서 원시값과 필터값을 함께 남기는가
- 논문 수치, 시뮬레이션 수치, 실물 수치를 구분했는가
