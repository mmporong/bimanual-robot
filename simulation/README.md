# 시뮬레이션과 영상

Isaac 장면·접촉·물붓기·식당 이동·왕복 executor와 영상 생성 도구를 모았다.
실차 Cartographer/Nav2는 src/navigation/, 공유 IK·파지·붓기 계산은 src/manipulation/,
주문 미션은 src/mission/에서 관리한다.

| 파일 | 역할 |
|---|---|
| simulate_urdf_workcell.py | URDF 작업셀 프리뷰·오프라인 입력 확인 |
| simulate_cup_contact.py, simulate_bottle_contact.py | 컵·병 접촉 물리 |
| simulate_bimanual_pour.py | 양팔 물붓기 |
| build_restaurant_scene.py, simulate_restaurant_mobile.py | 식당 장면·상판 운반·도크 왕복 |
| mobile_service_control.py, mobile_service_model.py | 시뮬 바퀴 제어·모델 |
| isaac_phase_executor.py, planned_executor_ipc_mock.py | phase IPC 실행·모의 실행 |
| encode_service_views.py, encode_service_promo.py | 결과 영상 인코딩 |

저장소 루트에서 모듈로 실행한다. 물리 실행은 Isaac Sim 5.1 Python을 사용한다.

```bash
cd "$HOME/bimanual-robot"
python3 -m simulation.simulate_urdf_workcell --help
python3 -m simulation.simulate_restaurant_mobile --help
# ISAAC_PYTHON에는 설치된 Isaac Sim의 python.sh 경로를 지정한다.
"$ISAAC_PYTHON" -m simulation.simulate_restaurant_mobile --help
```

ROS Action/phase IPC에는 [Action·IPC 문서](../docs/experiments/20260922_ExecuteManipulationSkill_ROS2_Action.md)의
hold_flow_mission 설치 또는 Python 경로를 준비한다. 산출물·동영상·로그는 저장소 밖에 둔다.
source_provenance.py는 분야가 갈라진 뒤에도 결과에 실제 소스 해시를 기록할 수 있도록 고정된 파일 위치를 제공한다.
실행 조건은 [식당 환경 기록](../docs/experiments/20260921_식당_시뮬레이션_환경.md)과
[물붓기 기록](../docs/experiments/20260921_양팔_PBD_물붓기_시뮬레이션.md)을 따른다.
