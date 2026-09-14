# JD-AMR SLAM 실기체 이식

작업 브랜치: `feat/jdamr-base-migration`. 이번 단계는 검증된 원본 고정과 새 차체 이식 준비다.
실기체 주행 완료를 뜻하지 않는다. 기존 JD-AMR의 사용자 미커밋 변경은 이식하지 않는다.

## 2026-09-14 후속 결정

다음 세션은 같은 모델의 다른 바퀴 모터를 새 차체에 이식하고 기본 주행을 먼저 검증한다.
[SLAM·RGB-D 검증](20260913_RGBD_SLAM_우선순위와_다음세션.md)은 기본 주행 게이트를 통과한 뒤 진행한다.
라이다와 카메라를 먼저 켤 수는 있지만, 기본 주행 검증 전의 센서 확인은 배선·토픽·고정 TF 점검으로 제한하고 지도 생성·위치 추정 성능으로 확대하지 않는다.

- 모터는 기존과 같은 모델의 다른 차체 개체다. 좌우 ID·회전 부호는 아직 확인하지 않았으므로 기존 ID를 그대로 적용하지 않는다.
- 기존 제어보드와 Raspberry Pi를 그대로 사용한다.
- 기존 YDLIDAR G4도 사용하며 장착 높이·위치가 변경될 수 있다. 방향을 포함한 센서 TF를 최종 실측한다.
- `config/navigation/jdamr_migration.json`의 schema v2는 과거의 모터·보드 재사용 결합 필드를 분리해 동일 모델 모터·기존 보드/Pi 사용 계획과 `planned_lidar=YDLIDAR G4`를 기록한다. 목표 모터 ID·실장 LiDAR·실측 TF·외곽은 `null`, `physical_motion_enabled=false`는 유지한다. 계획 확인은 장착 검증이나 이동 승인이 아니다. 내보내기 도구의 하드웨어 실행 기능은 추가하지 않는다.

## 기준선 재현

원본은 `mmporong/jdamr_cube_ros`의 `8204ffde34985fa7984fe4e6093801ff3897432c`다.
이 커밋에는 실차 SLAM·AMCL·Nav2, 구동 드라이버, Keepout, Gazebo 시나리오와 평가 코드가 있다.
MCAP·카메라 기록은 외부 아티팩트이므로 소스 내보내기와 별도로 보관한다.

```bash
cd "$HOME/bimanual-robot"
python3 tools/prepare_jdamr_migration.py \
  --source "$HOME/jdamr_cube_ws/src/jdamr_cube_ros" \
  --output "$HOME/jdamr_migration_ws/source_snapshot"
```

결과의 `baseline/`은 해당 커밋의 깨끗한 파일 집합이고 `manifest.json`은 파일별 SHA-256이다.
같은 출력 경로에 다시 실행하면 거부한다. 도구는 ROS·직렬 포트·모터를 실행하지 않는다.
빌드·설치 경로는 새 워크스페이스를 사용하고 기존 JD-AMR install에 덮어쓰지 않는다.

## 재사용과 교체 경계

| 항목 | 기존 JD-AMR | 새 차체에서 할 일 |
|---|---|---|
| 지도 작성 | Cartographer | 동일 백엔드 유지, 실제 센서 입력 확인 |
| 반복 주행 | 저장 지도+AMCL+Nav2 | 같은 공간이면 지도 재사용, 통과 가능 폭 재평가 |
| 모터·보드 | C018·General Driver 계열 | 동일 모델의 다른 모터 개체·기존 보드/Pi 사용, ID·방향 확인 |
| 반경 | 0.0329 m 보정값 | 같은 바퀴의 초기 후보, 하중 상태에서 재보정 |
| 유효 트레드 | 0.1836 m | 복사 금지, 새 설계 0.320 m부터 실측 보정 |
| LiDAR | 실행 코드상 YDLIDAR G4 | 사용자 G4 유지 확정, 설계상 LDS-03과 차이 반영·실장 TF 측정 |
| TF | JD-AMR URDF | 새 차체·센서 위치 반영, 고정 TF 중복 발행 방지 |
| footprint | JD-AMR 외곽 | 340(x)×450(y) mm 및 팔·선반 돌출 실측 반영 |
| Collision Monitor | JD-AMR 보호영역 | 새 외곽·정지거리로 재산정 |
| DDS | 온보드 LOCALHOST | 센서·제어 로컬 운용 유지 |

`src/hold_flow_description/config/mechanical_calibration.yaml`에는 이전 270 mm 설계값이
남아 있다. 현행 설계는 `design/mechanical/hold_flow_mechanical_v0_3.yaml`과 생성 URDF의
320 mm다. 둘 모두 유효 트레드 실측값은 아니다. 실측 전 기존 calibration 파일을 실기체
주행 설정으로 사용하지 않는다.

`real_bringup.launch.py`에 바퀴 간격만 전달해도 로봇 설명은 여전히 JD-AMR URDF이며
LiDAR 드라이버도 G4다. 따라서 그것만 실행해서 새 차체 이식이 완료됐다고 판정하지 않는다.
현재 브랜치의 `config/navigation/jdamr_migration.json`은 이 차이와 미확정 입력을 기록한다.

## 적용 순서와 판정

1. **M0 모터 이식:** 전원을 끈 상태에서 같은 모델의 다른 좌우 바퀴 모터를 장착하고 체결·배선·휠 간섭·캐스터 접촉을 확인한다. 모터 ID는 읽어서 기록하며 기존 ID를 추정해 쓰지 않는다.
2. **M1 비접지 구동 확인:** 동일 보드의 펌웨어·UART 프로토콜과 watchdog을 유지한다. 사용자 실기체 실행 요청 후 바퀴를 바닥에서 띄운 상태로 좌우 모터를 하나씩 짧게 명령해 ID·회전 부호·정지 명령·명령 두절 시 정지를 확인한다. 실제 watchdog 만료부터 바퀴 속도 0 판정까지의 시간을 측정한다.
3. **M1.5 접지 시험 계약 고정:** M1 로그와 시험 공간의 가용 정지 여유를 근거로 접지 시험의 최대 명령 지속시간, 최대 정지 반응시간·거리, 수동 중단 조건을 먼저 기록한다. 이 안전 상한은 접지 주행 결과를 본 뒤 넓히지 않는다. 한 항목이라도 미정이면 M2를 시작하지 않는다.
4. **M2 기본 주행:** 팔을 수납하고 주변을 통제한 상태에서 `0.08 m/s` 이하의 저속으로 짧은 직진·후진, 좌·우 제자리 회전, 명령 중 정지와 명령 두절 정지를 수행한다. 바퀴/차체 실제 방향과 odom 선속도·yaw 부호가 모두 일치해야 하며, M1.5의 정지 상한을 넘으면 즉시 실패로 기록한다.
5. **M3 기구·오도메트리 보정:** `wheel_radius=0.0329 m`, 기하 `wheel_separation=0.320 m`는 시작값으로만 사용한다. 측정 구간의 직진 거리와 제자리 회전각으로 유효값을 보정하고, 반복 편차·전류·전압·온도·캐스터 떨림·정지거리를 기록한다. 실측 뒤 거리·각도 반복 오차와 보정 허용치를 동결하고 같은 조건을 다시 시험한다.
6. **G-DRIVE 기본 주행 통과:** 좌우 ID와 부호가 고정되고, watchdog/수동 정지가 M1.5의 사전 상한 안에서 작동하며, 직진·후진·좌우 회전에서 명령·차체·odom 부호 불일치가 0건이어야 한다. 체결 풀림·배선 끌림·바퀴/차체 간섭·예상 밖 접촉도 0건이어야 한다. M3의 동결된 거리·각도 반복 오차와 보정 허용치 재시험도 통과해야 한다. 미확정 허용치가 있거나 시험이 실패하면 SLAM으로 넘어가지 않는다.
7. **S0 센서 실장:** G-DRIVE 통과 후 G4 LiDAR와 RGB-D의 실제 높이·위치·방향을 측정하고 새 차체 로봇 설명과 TF에 반영한다. 고정 TF 중복 발행과 차체 자체 가림을 점검한다.
8. **S1 SLAM:** 센서 시간축과 odom을 확인한 뒤 Cartographer/LiDAR 기준선을 재현하고 RGB-D 단독 Visual SLAM 비교를 수행한다. 기본 주행 보정 전의 궤적과 지도는 새 차체 성능 기준으로 사용하지 않는다.
9. **S2 위치 추정·주행:** 저장 지도+AMCL+Nav2에서 새 외곽의 경로·Keepout·정지·동일 goal 재개를 검증하고, 이후 LiDAR 위치 추정과 RGB-D 3D 장애물 관측을 역할 분담 방식으로 통합한다.

기구 문서의 초기 속도 후보는 0.08 m/s 이하다. 실측 속도·감속·정지거리와 팔 수납 상태를
확인하기 전에는 기존 20/20 결과를 새 차체 성능으로 전용하지 않는다. 동일 코드·입력 버전은
고정할 수 있지만 질량·트레드·센서 위치가 달라지면 동일 궤적을 보장할 수는 없다.

Gazebo 저마찰 복구의 주입 활성 신호는 시뮬레이터 전용이다. 이를 실차 미끄럼 검출기로
연결하지 않는다. 실차 마찰 이상 검출은 독립 구현·평가 대상이다.

## 보존한 기존 성과

- 실차 원본: `$HOME/jdamr_artifacts/real_combined_obstacle_retry_20260910T122006/`
- 기존 시뮬레이션 PASS: `$HOME/jdamr_artifacts/restaurant_actual_map_visible_vehicle_v3/summary.json`
- 지도 재현 계약: `$HOME/jdamr_artifacts/restaurant_actual_map_3d_v20/scenario_contract.json`
- 기존 영상: `$HOME/jdamr_artifacts/restaurant_actual_map_visible_vehicle_v3/gazebo_actual_map_4x.mp4`
- 기존 시뮬레이션 결과: 경유점 20/20, 장애물 장면 3/3, 합성 저마찰 복구 1회

위 아티팩트와 지도 YAML이 참조하는 PGM은 경로와 해시를 유지해 이식 작업공간에서도
접근 가능하게 둔다. 저장소 기본 브랜치는 이번 이식 브랜치 검증 뒤 별도로 통합한다.
