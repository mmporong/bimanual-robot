# 뎁스 포인트클라우드 개발판

깊이 영상에서 복원한 XYZ 점군을 책상별 후보로 나누고, 로봇 이동을 보상해 같은 접근면을 추적하는 선택형 관측 경로다. 기존 주차 제어기·1 cm 목표·AMCL·NavFn·RPP는 유지한다. 기존 장애물용 PointCloud2·VoxelLayer 경로와 이번 정밀 관측 경로는 용도가 다르다.

현재는 로컬 구현·합성 회귀·보존 영상 재생 단계다. Pi 배포, 이동 중 실측, 상판 점유 통합, 반복 주차 성능은 확인 전이다. 전체 개발 목표와 판정 기준은 [개발 계획](../plans/AMR_POINTCLOUD_DOCKING_PLAN.md)에 있다.

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

이번 개발에서는 env 파일 설치·서비스 재시작·파이 배포·실물 이동을 실행하지 않았다. 기존 운영 worktree도 변경하지 않았다.

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

## 다음 실증에 필요한 입력

1. 현재 장착의 깊이·CameraInfo·취득 시각 TF/odom을 같은 기록에 보존한다. 정지/이동, 인접 책상, 경계 가림을 구분한다.
2. 보정용과 겹치지 않는 장면의 거리·각도 외부 기준을 확보한다. 같은 프레임에서 유도한 장착값을 그 프레임의 거리 정답으로 쓰지 않는다.
3. S1 정확도를 확인한 뒤 추적 인계와 상판 점유를 평가한다. Pi 성능·실차 반복 주차·전체 왕복은 각각 별도 지표로 남긴다.

새 물리 실행 요청 없이 가능한 구현·재생·회귀는 로컬에서 수행한다. 실제 1 cm 성능 개선, 정차시간 감소, 장애물 회피 개선은 해당 실측 전에는 성과로 기록하지 않는다.
