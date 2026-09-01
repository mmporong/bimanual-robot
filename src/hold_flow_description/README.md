# hold_flow_description

250×250 mm 이동 베이스, SO-101 두 대, Robonine 수평 평행 그리퍼, Astra S와 LDS-03 후보를 하나의 TF 트리로 묶는 ROS 2 Jazzy 설명 패키지다.

## 모델의 좌표 기준

- `base_footprint`: 두 구동 바퀴축의 바닥 투영점
- `base_link`: `base_footprint`에서 z=32.9 mm
- 차체 중심: `base_footprint`보다 x 방향으로 90 mm 뒤
- 좌우 팔 베이스: 차체 중심 기준 `(25, ±70, 119) mm`
- 카메라 광학 중심 목표: 차체 중심 기준 `(-80, 0, 800) mm`
- LDS-03 중심: 차체 중심 기준 `(90, 0, 165) mm`, 전용 29 mm 저상 받침대

바퀴 간 270 mm는 기하 중심거리다. Nav2 오도메트리에 넣을 유효 `wheel_separation`은 완성 차체로 직진·제자리 회전 시험을 한 뒤 보정해야 하며, JD-AMR의 183.6 mm 값을 복사하지 않는다.

## 공개 형상 사용

`third_party/robonine`과 `meshes/robonine`은 Robonine의 SO-ARM100/101 Parallel Gripper 저장소 `305ad0f6e8f19e4e739616160cbdc7cae1ab153f` 커밋에서 가져왔다. 형상과 관절축은 원본을 유지하고, 공개 URDF의 과도한 질량은 원래 SO-101 링크 질량과 170 g 평행 그리퍼 총질량에 맞춰 합계 0.703 kg/arm으로 스케일했다. 이 관성은 제어용 최종값이 아니라 충돌·가동범위 검토용 P0 값이다.

## 빌드와 검증

```bash
cd "$HOME/bimanual-robot"
colcon build --packages-select hold_flow_description --symlink-install
source install/setup.bash
python3 src/hold_flow_description/scripts/validate_description.py
```

이 호스트처럼 사용자 Python이 ROS의 Python보다 먼저 잡히면 다음처럼 실행한다.

```bash
colcon build --packages-select hold_flow_description --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

검증은 Xacro 확장, `check_urdf`, 링크/조인트 존재, 모든 `package://` 메시의 실제 파일 존재, 바퀴 중심거리 270 mm, 좌우 평행 그리퍼 mimic joint를 확인한다.
또한 커밋된 `hold_flow.urdf`가 Xacro의 현재 확장 결과와 같은지, 카메라 광학 중심·LDS-03·좌우 팔 베이스가 설계 좌표에 놓이는지도 수치로 검사한다.

## 아직 확정하지 않은 값

아래 값은 장공과 YAML의 `null`로 남겼다. 실물을 재기 전 숫자를 확정하면 출력물이 조립되지 않을 수 있다.

- SO-101 베이스 체결공 패턴
- C018 혼–JD-AMR 휠–외부 베어링 축 조합
- JD-AMR 볼 캐스터 플랜지와 접촉 높이
- Astra S 하단 M6 위치와 외곽 공차
