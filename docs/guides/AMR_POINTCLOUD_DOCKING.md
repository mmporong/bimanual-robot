# 뎁스 포인트클라우드 개발판

깊이 영상에서 복원한 XYZ 점군을 책상별 후보로 나누고, 로봇 이동을 보상해 같은 접근면을 추적하는 선택형 관측 경로다. 기존 주차 제어기·1 cm 목표·AMCL·NavFn·RPP는 유지한다. 기존 장애물용 PointCloud2·VoxelLayer 경로와 이번 정밀 관측 경로는 용도가 다르다.

현재는 구현·합성 회귀·영상 재생과 Pi 무이동 배포 시험 단계다. Pi에서 깊이 입력과 부분 부하는 측정했으나 카메라와 차체의 TF 연결을 수신하지 못해 추적 통합은 확인 전이다. 이동 중 실측, 상판 점유 통합, 반복 주차 성능도 남아 있다. 전체 개발 목표와 판정 기준은 [개발 계획](../plans/AMR_POINTCLOUD_DOCKING_PLAN.md)에 있다.

## 구현과 경계

모듈 경로의 기준은 `src/navigation/jdamr_cube_navigation/jdamr_cube_navigation/`다.

| 모듈 | 동작 |
|---|---|
| `pointcloud_components.py` | 깊이 영상 격자의 상하좌우 인접점 사이 XYZ 거리가 연결 기준 이내일 때 묶는다. 가림이나 빈 픽셀을 건너 연결하지 않는다. 전체 공간의 모든 이웃을 찾는 Euclidean clustering과는 구별한다. |
| `table_surface_model.py` | 높이 ROI의 연결 성분별로 기존 상판·앞 경계 검출기를 실행한다. 평가 후보 수에 상한을 둔다. 정밀 면 맞춤에는 원래 점을 사용하며 복셀 평균으로 바꾸지 않는다. |
| `table_surface_tracking.py` | 취득 시각의 odom←optical 변환으로 선분을 비교한다. 방향·직선 거리·높이·실제 선분 겹침으로 연계한다. 잘린 선분 중점이나 양끝 순서 변화만으로 대상을 교체하지 않는다. |
| `depth_box_parking.py` | 새 관측을 기존 상태 메시지로 변환한다. 예측은 `detected/stable` 관측으로 보내지 않는다. 속도 명령 발행은 없다. |
| `box_service.py` | 선택한 책상의 지도 영역과 장착 설정 해시를 기존 원자적 설정 요청에 넣는다. 최종 거리·각도 판정과 제어 경로는 유지한다. |
| `pointcloud_replay.py` | raw·보정 출처·처리 설정·구현 해시를 남기며 같은 프레임에서 기존/새 검출을 비교한다. ROS 서비스나 로봇을 사용하지 않는다. |

지도 영역은 첫 대상을 고르는 근방이지 정밀 거리 정답이 아니다. 영역 안의 후보가 여러 개면 임의로 고르지 않는다. 영역 중심은 첫 관측 시 odom으로 변환해 보존하므로 AMCL map 보정이 진행 중인 대상의 좌표를 옮기지 않는다. 대상 영역이나 장착 버전이 바뀌면 캐시와 추적을 지운다.

가림 때 이전 선분을 유지하는 기본 예산은 0.25 s·3 cm·5°다. 이는 실차에서 검증된 허용 오차가 아니라 개발 설정이다. 예산 이내도 진단용 `predicted`이고 새 제어 관측은 아니다. 예산을 넘으면 `expired`로 표시한다. 관측 시각과 현재 프레임 시각을 구분한다.

옆면 평면을 별도로 맞추는 기능, 의미 분류, ICP, PCL, 새 상판 점유 모델, 이동 중 관측을 이용한 정차시간 단축은 이번 구현에 포함하지 않는다. 기존 높이 장애물 입력은 변경하지 않았다. 이 기능만으로 기존 주행의 충돌 감시 공백이 해결됐다고 보지 않는다.

## 기본값과 선택 실행

기본 observer와 일반 `go`는 기존 검출기를 사용한다. 다음 두 단계가 모두 있어야 새 관측을 사용한다.

1. 배포할 observer의 capability를 켠다. systemd 기본 유닛과 `50-four-frames` 오버라이드는 배포 디렉터리의 선택형 `pointcloud_observer.env`를 읽는다. 내용은 다음과 같다. 파일을 두지 않으면 두 값 모두 `false`다.

   ```ini
   JDAMR_TABLE_COMPONENTS=true
   JDAMR_TABLE_TRACKING=true
   ```

2. 해당 실행에 기존 운영 진입점 `tools/jdamr/jdamr_depart.py go`의 `--pointcloud-perception`을 지정한다. 실행기가 등록된 책상 근방을 전달하면서 `table_request_enabled=true`를 원자적으로 적용한다. capability가 꺼져 있으면 설정 실패를 반환한다. 활성화하지 않은 기능을 사용한 것처럼 처리하지 않는다.

일반 `go`는 `table_request_enabled=false`와 빈 영역·장착 ID를 보내 기존 검출기로 복귀하고 이전 추적을 지운다. 이 전송도 기존 관측기 설정 요청 안에서 처리하며 새 출발 검사나 별도 승인을 추가하지 않는다. 새 실행기와 observer는 같은 소스 버전으로 설치해야 한다. 구버전 observer는 새 파라미터를 지원하지 않는다.

`--pointcloud-perception`은 `table_edge` 접근 태스크에만 사용할 수 있다. `front`, 도크 복귀만 수행하는 요청, 후면 박스 주차만 수행하는 요청과 조합하면 명령 해석 단계에서 거부한다.

관측기만 띄우는 launch에는 capability인 `table_components`, `table_tracking`이 있으며 기본값은 모두 `false`다. `table_tracking=true`에는 `table_components=true`가 필요하다. 실행별 요청·유효한 대상 영역·비어 있지 않은 보정 ID는 실행기가 동적으로 함께 설정한다. 관측 범위만 바꿀 때는 추적 안정성을 새로 채우지만, 이미 고정한 odom 대상 영역을 map에서 다시 변환하지 않는다.

2026-10-23 사용자 요청으로 Pi에 capability와 개발판을 설치하고 센서만 켜 시험했다. 실행별 요청은 시험 뒤 false로 복원했다. 실물 이동과 기존 운영 worktree 수정은 없으며 상세는 아래 정지 시험 기록에 있다.

## 오프라인 재생

새 ROS 패키지를 빌드한 설치 환경을 source한 뒤 실행한다. 재생기는 센서 노드를 띄우지 않는다.

```bash
cd "$HOME/bimanual-robot-pointcloud"
source "$HOME/.cache/codex/pointcloud-install/setup.bash"
ros2 run jdamr_cube_navigation pointcloud_replay \
  --manifest "$HOME/.local/state/bimanual-robot/pointcloud_development_20261022/current_gap_manifest.json" \
             "$HOME/.local/state/bimanual-robot/pointcloud_development_20261022/stationary_sequence_manifest.json" \
  --output "$HOME/.local/state/bimanual-robot/pointcloud_development_20261022/replay_final"
```

위 경로는 개발 기기에 보존한 원본과 manifest다. raw·실행 로그는 Git에 넣지 않는다. 다른 기기에서는 해당 원본과 보정 출처를 확보하고 manifest를 만든다.

| manifest 키 | 계약 |
|---|---|
| `schema_version` | `1` |
| `capture_group`, `split`, `frame_id` | 수집 단위, `calibration/development/evaluation`, 실제 광학 프레임 이름 |
| `input.path`, `input.sha256`, `input.encoding` | npz 위치·원본 SHA-256·`16UC1_mm` 단위 선언 |
| `calibration.id/status/source/source_sha256` | 보정 버전·`derived/measured`·출처 파일과 SHA-256 |
| `table_config` | 해당 기록에 적용할 `TableEdgeConfig` 필드. 현재 장착값을 과거 기록에 대입하지 않음 |
| `scene_config`, `track_config` | 선택형 `SceneConfig`, `TrackConfig` 필드. 생략해도 실제 기본값을 결과에 남김 |
| `target_region_odom` | 선택형 `center_xy_m: [x, y]`, `radius_m`. 취득 기록과 같은 odom 좌표계의 대상 근방 |

npz에는 uint16 mm `depth`와 `intrinsics=[fx,fy,cx,cy]` 또는 유효한 rectified `p`가 필요하다. `p`가 있으면 그것을 사용하며 NaN `K`를 유효한 보정값으로 대신 쓰지 않는다. 시간은 실제 `stamps[N]`만 사용한다. 추적 재생에는 같은 취득 시각의 `odom_from_optical[N,4,4]`도 필요하다. 파일 수정 시각이나 별도로 조회한 현재 TF를 영상 취득 시각으로 대체하지 않는다.

출력 `frames.csv`에는 프레임별 검출·후보 수, 원본 광학 좌표계의 경계 선분, 추적 ID·odom 선분·측정 시각·처리 시간이 있다. `summary.json`에는 입력·manifest·보정 출처·실효 설정·구현 6파일의 SHA-256과 입력 가용성을 남긴다. 같은 수집 그룹이나 같은 raw를 서로 다른 split에 넣으면 실행을 거부한다. 독립 정답 평가기는 아직 없으므로 `accuracy_evaluated=false`다.

## 2026-10-22 보존 영상 결과

| 기록 | 입력 | 기존 검출 | 성분별 검출 | 추적 평가 가능 여부 |
|---|---|---:|---:|---|
| `camera_gap_mismatch_20261019_152144/depth.npz` | 실제 깊이 1장 | 1/1 | 1/1 | 취득 시각·동기 odom 없음 |
| `table_edge_20261013/stationary_depth.npz` | 정지 연속 영상 20장 | 1/20 | 0/20 | 취득 시각은 있으나 동기 odom 없음 |

두 기록 모두 외부 정답이 없고 과거 메타데이터에서 유도한 장착값을 사용했다. 위 수치는 검출 반환 횟수이며 정확도·주차 성공률이 아니다. 새 방식의 우수성이 확인되지 않았으므로 기본 활성화하지 않는다. 과거 영상의 미검출은 분리·높이 ROI·보정·영상의 실제 유효성을 구분해 후속 평가한다. 영상의 결과를 맞추려고 장착값을 재조정하지 않았다.

최종 재생 근거는 위 명령의 `replay_final/000`과 `replay_final/001`이다. 기록에는 이 PC의 오프라인 처리 시간만 있으며 Pi CPU·RSS·DDS·제동 지연의 근거로 쓰지 않는다.

## 2026-10-23 Pi 배포와 정지 시험

### 배포 범위와 보존

- 소스 `5c29ac1`의 9개 패키지를 Pi에서 2개 병렬 작업으로 빌드했다(3분 1초). 모듈 설치 경로와 점군 4개 모듈의 PC/Pi SHA 일치를 확인하고 source/install/tools 슬롯을 승격했다.
- Pi stage는 `$HOME/jdamr_deploy/bimanual_stage_pointcloud_5c29ac1_20261023_073244`(82 MiB), 이전 슬롯은 `$HOME/jdamr_deploy/bimanual_backup_pointcloud_5c29ac1_20261023_073244`(88 MiB)에 보존했다. 지도·목적지·keepout·장착 설정은 교체하지 않았다. 최종 디스크 여유는 15 GiB였다. 백업은 복원용 파일이며 실행 중 서비스를 둔 채 슬롯만 바꾸는 복원은 검증하지 않았다.
- 기존 observer 유닛·`90-local-dds.conf`의 SHA와 stable 8프레임을 보존하고 `85-pointcloud-capability.conf`와 `$HOME/jdamr_deploy/pointcloud_observer.env`만 추가했다. capability는 true, 실행별 요청은 시험 전후 false다. `50-four-frames`는 설치하지 않았다.
- 기존 base PID99657·Nav2 PID99423은 재시작하지 않았다. idle executor만 교체해 최종 PID104674의 `box_service --serve`를 확인했다. 센서 두 서비스는 시험 뒤 inactive/MainPID0이며 임시 진단 프로세스와 UDP override를 제거했다. 요청/실행 spool은 모두 0이다. 속도 발행·action·위치초기화는 수행하지 않았다.

### 실제 입력과 자원

배치는 사용자가 확인한 도크→통로 방향이다. 의자가 책상면을 가리거나 바닥만 보일 수 있고 물체 정답 주석은 없다. 실제 `16UC1` 깊이·rectified P·광학 프레임 이름·취득 시각을 저장했다. 장착 모델 투영에서는 낮은 높이의 넓은 점군과 검색 높이 일부 점이 보이지만 책상임을 확인한 것은 아니다. 물체·거리 정확도나 미검출 원인의 근거로 쓰지 않는다.

같은 capability 활성화 binary·같은 PID·정지 장면에서 요청 OFF/ON을 각 20초 비교했다. CPU는 한 코어 100% 기준이며 전체 Pi CPU만 4코어 합계 100% 기준이다. RSS는 프로세스 트리 합산으로 공유 페이지가 중복될 수 있다.

| 항목 | 기존 요청 OFF | 점군 요청 ON |
|---|---:|---:|
| 깊이 수신 / 상태 메시지 | 525 / 94 | 526 / 94 |
| observer CPU / peak RSS | 55.63% / 100.96 MiB | 69.39% / 101.50 MiB |
| 카메라 CPU / peak RSS | 22.94% / 147.50 MiB | 21.60% / 147.50 MiB |
| Pi 전체 CPU | 39.78% | 42.08% |
| 최소 가용 RAM | 2,806.80 MiB | 2,802.80 MiB |
| 최고 온도 / 최소 배터리 | 63.78℃ / 12.392 V | 65.24℃ / 12.384 V |
| 검출 / stable | 0 / 0 | 0 / 0 |

새 경로는 성분 처리 뒤 `table_tracking_transform_unavailable`로 94회 반환됐다. 정상 추적의 처리 지연은 측정되지 않았으며 위 수치를 성공 경로·최악 부하·주행 성능으로 일반화하지 않는다. 기존 요청 OFF도 새 binary이므로 구버전 설치본 대비 메모리 절감 비교가 아니다. 속도 명령 수신은 전 구간 0건, odom의 한 구간 최대 XY 변화는 약 0.025 mm였으며 외부 이동 실측은 아니다.

저장한 새 깊이 90장을 ROS 노드 없이 오프라인 재생했다. 기존/성분별 모두 90장 중 0장 검출했고, 성분 처리 p50 109.30 ms·p95 112.44 ms다. 실행 호스트를 이 summary만으로 확정하지 않으며 위 Pi live 자원 표와 합쳐 성능을 해석하지 않는다. live DDS·추적·제어 전체 지연도 아니다. 동기 odom 변환이 없어 추적 평가와 독립 정확도 평가는 수행하지 못했다.

### 남은 통합 문제

1. 기존 LOCALHOST DDS 환경에서 새 노드가 `rclpy.Node` 생성 중 멈췄다. 정리 후에도 재현됐고 메모리 고갈 원인은 확인되지 않았다. 기존 base/Nav2는 유지했다. 저장소의 UDP loopback XML을 센서·진단에만 임시 적용했을 때 새 노드와 실제 입력 수신이 가능했다. 이 우회는 시험 뒤 제거했으며 영구 해결이 아니다.
2. 임시 UDP 경로에서 `odom→base_footprint`와 `camera_link→camera_color_optical_frame`은 각각 수신했으나 두 트리를 잇는 차체 정적 TF와 `map→odom`은 수신하지 못했다. 취득 시각 odom←optical 기록은 0개다. URDF 오류로 확정하지 않으며 기존 정적 TF의 late join/DDS 수신을 구분해야 한다. 임의 static TF나 pose를 만들지 않았다.
3. active Nav2 identity와 현재 registry 입력의 불일치는 배포 전부터 있었다. 관련 입력 4개 SHA가 전후 같음을 확인했고 이번 작업에서 identity 갱신·Nav2 재시작은 하지 않았다. 서비스 active만으로 주행 준비 완료라고 판정하지 않는다.

근거 원본은 Pi `$HOME/jdamr_deploy/pointcloud_check_20261023_073244/`, 회수본은 PC `$HOME/.local/state/bimanual-robot/pointcloud_pi_20261023_073244/pi/`다. 주요 파일은 `promotion_and_service_receipt.json`, `cleanup_receipt.json`, `measurements/summary.json`, `measurements/*_raw.npz`, `replay_manifest.json`, `replay/summary.json`과 `replay/frames.csv`다. PC 상위 폴더의 `motion_before_bounded.txt`, `join_after_cleanup.txt`, `stationary_probe_live.log`, `pi_replay_run.txt`에 통신·실행 기록을 보존했다. `depth_pointcloud_preview.png`는 대표 깊이와 장착 모델 투영이며 독립 보정·물체 정답 그림이 아니다. 임시 수집기 자체의 초기 실패 기록은 최종 비교와 혼용하지 않는다.

## 2026-10-23 바닥 후보 오프라인 로직 시험

사용자 요청으로 위 정지 깊이 90장만 PC에서 분석했다. ROS 노드·Pi 서비스·속도 발행·실물 이동은 없고 운영 장착값·TF는 변경하지 않았다. 원본·장착 SHA를 대조한 뒤 분석 프로토콜·스크립트 SHA와 프레임별 CSV를 보존했다.

| 검사 | 결과 | 해석 범위 |
|---|---|---|
| 독립 축 회전·ray renderer와 production 좌표 변환 | 행렬 절대오차 허용치 1e-14 이내 | 광학→level 축·부호 대조 |
| 무잡음 합성 바닥 3개 높이·자세 | 3조건 모두 높이 2 mm·각도 0.1° 진단 허용치 이내 복원 | 이상 기하 계산 검사. 실물 정확도 허용치가 아님 |
| 현 설정의 합성 바닥 / 책상 | 바닥 0검출, 카메라 기준 전방 0.30/0.45/0.60 m의 책상은 기존·성분별 모두 검출 | 4개 대조 검사 통과. 실제 가림·센서 노이즈 강건성은 미검증 |
| 실제 바닥 후보 ROI의 높이 필터 | 90장 모두 유효점 최소 6,383개에서 검색 높이 통과점 0개 | 해당 ROI의 점들이 현 50–105 cm 검색 범위 밖이라는 검사 |
| 실제 전체 영상의 기존/성분별 검출 | 두 방식 모두 90장 중 0장 | 책상 정답이 없는 한 정지 장면의 관측 사실 |
| 기존 관련 4스위트 | 85 PASS(1.17초) | 면 검출·성분·재생·추적의 소프트웨어 회귀 |

시각 선택한 ROI는 `(x:32–287, y:40–144)`의 **바닥 후보**이며 바닥 정답 주석은 아니다. ROI 밖을 0으로 만든 입력은 경계 아래 배경도 없애므로 그 입력의 0검출만으로 높이 필터 성공을 주장하지 않았다. 별도로 유효점 수와 높이 필터 통과점 0개를 확인했다.

90장 평면 추정의 중앙값은 카메라↔평면 수직거리 1.14778 m, roll -4.81309°, pitch 50.88301°다. 기존 설정 1.26 m·-2.23586°·52.54996°와 다르다. 기존 변환으로 환산한 ROI 높이 중앙값은 0.07684 m, 전방 거리에 대한 높이 기울기는 약 -0.03045 m/m다. 적합점의 평면 RMS 중앙값은 6.69 mm, ROI 전체 점의 프레임별 잔차 p95 중앙값은 15.60 mm였다. 첫 프레임에서 고정한 면에 대한 전체 ROI 잔차 p95 중앙값도 15.71 mm다. 이는 같은 capture 내부의 일관성 수치이며 독립 반복 정확도가 아니다.

**판정:** 계산·단위·축 변환·선택 영역의 높이 제외와 이상 기하 대조는 통과했다. 실제 거리 보정·장착 일치·1 cm 주차 정확도는 통과 판정하지 않는다. 후보가 실제 수평 바닥인지, 깊이/P의 거리 보정이 맞는지 외부 기준이 없으므로 위 추정치를 새 높이·각도로 자동 적용하지 않았다. 이 시험으로 DDS/TF 연결 문제도 해결되지 않는다.

PC 산출물은 `$HOME/.local/state/bimanual-robot/pointcloud_pi_20261023_073244/floor_logic_check/`의 `protocol.json`, `analyze.py`, `summary.json`, `frames.csv`, `execution.log`, `floor_logic_preview.png`다. 합성 입력의 진단 허용치와 ROI·RANSAC 선택 근거는 `protocol.json`에 있고 운영 보정값은 아니다. 오프라인 재현 명령은 다음과 같다.

```bash
cd "$HOME/bimanual-robot-pointcloud"
PYTHONPATH="$PWD/src/navigation/jdamr_cube_navigation" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 "$HOME/.local/state/bimanual-robot/pointcloud_pi_20261023_073244/floor_logic_check/analyze.py" \
  "$HOME/.local/state/bimanual-robot/pointcloud_pi_20261023_073244/floor_logic_check/protocol.json"
```

## 다음 실증에 필요한 입력

1. DDS 합류·차체 정적 TF 수신 문제를 분리한 뒤 현재 장착의 깊이·CameraInfo·취득 시각 TF/odom을 같은 기록에 보존한다. 정지/이동, 인접 책상, 경계 가림을 구분한다.
2. 보정용과 겹치지 않는 장면의 거리·각도 외부 기준을 확보한다. 같은 프레임에서 유도한 장착값을 그 프레임의 거리 정답으로 쓰지 않는다.
3. S1 정확도를 확인한 뒤 추적 인계와 상판 점유를 평가한다. Pi 성능·실차 반복 주차·전체 왕복은 각각 별도 지표로 남긴다.

새 물리 실행 요청 없이 가능한 구현·재생·회귀는 로컬에서 수행한다. 실제 1 cm 성능 개선, 정차시간 감소, 장애물 회피 개선은 해당 실측 전에는 성과로 기록하지 않는다.
