# SLAM·실차 주행 사용법

SLAM·저장 지도 주행·정밀 주차는 양팔 저장소의 `src/navigation/`에서 관리한다.
원본 JD-AMR의 `2e25c423926d02badd8e3461a32c587912f5dd0a`를 가져왔으며
패키지명·Nav2 설정·실차 TF·4륜 펌웨어를 유지한다.
이전 JD-AMR 리포를 별도로 clone하거나 외부 작업 폴더의 Python 코드를 가져올 필요가 없다.
ROS 2 Jazzy와 Astra 같은 외부 의존성은 설치 도구로 준비한다.

## 설치와 PC 빌드

```bash
cd "$HOME/bimanual-robot"
bash src/navigation/deployment/install_dependencies.sh plan
bash src/navigation/deployment/install_dependencies.sh --apply
source /opt/ros/jazzy/setup.bash
python3 tools/jdamr/runtime.py bootstrap
python3 tools/jdamr/runtime.py build-pc
source .runtime/jdamr/pc_ws/install/setup.bash
```

기본 빌드는 실차 런타임 9개 패키지다. 과거 팔·MoveIt·다른 라이다 패키지까지
빌드하려면 `build-pc --all`을 사용한다. 빌드 산출물은 Git에서 제외한다.
Astra의 별도 빌드와 장치 설정은 [배포 자료](../../src/navigation/deployment/README.md)를 따른다.

`bootstrap`은 현재 지도 포인터에서 등록부·지도·keepout·station 프로파일을 만든다.
기존 도크, `state.json`, 마지막 초기 위치와 실행 기록을 초기화하지 않는다.
새 PC에는 현재 지도·등록·센서 참조를 함께 준비하며 과거의 `localized=true`를 복사하지 않는다.
이전 지도 번들은 `bootstrap --legacy-map`을 명시했을 때만 사용한다.
지도와 마스크는 실행 설정이며 rosbag·영상·원본 센서 데이터와 분리한다.

## Pi 배포

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/runtime.py plan
python3 tools/jdamr/runtime.py deploy-pi
```

기본 배포는 Pi의 타임스탬프 stage에 전체 소스를 복사해 런타임 9개 패키지를 빌드한다.
현재 서비스가 참조하는 소스를 교체하려면 명시적으로 `--promote`를 사용한다.

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/runtime.py deploy-pi --promote
```

검증한 stage의 소스·설치·도구 경로를 심볼릭 링크로 연결하고 기존 디렉터리·링크는 백업한다.
중간 실패 시 변경한 경로를 역순으로 복구한다. 설치가 참조하는 승격 stage는 삭제하지 않는다.
교체 파일은 Pi의 배포 백업에 보존한다. `--delete`를 사용하지 않으며 기존 `.git`은
전송에서 제외한다. 배포 자산은 현재 지도 포인터가 선택한 runtime에서 가져오고,
기존 지도·도크 등록·카메라 설정은 기본적으로 유지한다. 실행 상태·클릭·run 로그는 배포하지 않는다.
정적 자산을 교체할 때만 `--replace-assets`를 사용하며 먼저 백업한다.
이 명령은 서비스 재시작이나 이동을 실행하지 않는다. 코드 변경 후 실행기 재시작이
필요한 경우 수정·배포 단계에서 마치며 다음 출발 때 빌드를 반복하지 않는다.

새 Pi의 의존성·Astra 패치·systemd·udev와 대시보드 설치는
[런타임 재구성 절차](../../src/navigation/deployment/README.md)에 있다.
현재 Pi의 서비스 경로인 `$HOME/jdamr_ws`는 유지한다. 이 경로는 빌드·설치 위치이며
이후 수정할 소스의 원본은 양팔 리포다.

## 운영 명령

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/jdamr_depart.py status
python3 tools/jdamr/jdamr_depart.py session-start
python3 tools/jdamr/jdamr_depart.py init
python3 tools/jdamr/jdamr_depart.py go table_01
python3 tools/jdamr/jdamr_depart.py stop
```

`init`은 등록된 도크 자세에서 위치를 정합한다. 로봇을 옮겼을 때는 기존 실행기의
위치 초기화 옵션을 사용한다. 출발을 요청하면 준비된 실행기의 `go`로 이동·정렬·최종 접근을
이어가며 리뷰·전체 검사·문서 작업을 출발 단계에 끼워 넣지 않는다.
실물 이동은 사용자가 요청한 실행 범위에서만 수행한다.

기본 주차 간격은 책상·박스 모두 계약의 1 cm다. `--final-gap-m`으로 지정한 간격이
우선한다. `go`는 상주 실행기를 사용하고 종료 코드와 도크 도착을 함께 확인한다.
이동 전 지도 수신 실패는 현재 위치추정을 유지하며 실행기만 한 번 재시작한다.
출발 중 전체 서비스 복구나 화면·녹화 기동은 수행하지 않는다.
RViz는 `display-start`, PC 녹화까지 필요하면 `display-start --record-diagnostics`를 쓴다.

상세 하위 명령과 `JDAMR_HOST`, `JDAMR_PI_HOME`, `JDAMR_PI_WS`,
`JDAMR_DATA_ROOT`, `JDAMR_PC_WS` 등의 환경변수는
[운영 도구 안내](../../tools/jdamr/README.md)를 참고한다.
현재 지도는 `$HOME/jdamr_data/maps/current_manual_mapping.json`, 실행 자산은 해당
포인터가 선택한 디렉터리의 `runtime/`에 있다. 다른 포인터는 `--map-session` 또는
`JDAMR_MAP_SESSION`으로 선택한다. 과거 `tools/` 사본을 개발 원본으로 사용하지 않는다.

## 지도 작성과 위치추정

PC 또는 Pi에서 해당 설치 환경을 읽은 뒤 기존 ROS 패키지를 사용한다.
`real_bringup`은 베이스·라이다를, Cartographer는 지도 작성을 담당한다.
저장 지도 반복 이동에서는 Cartographer 대신 AMCL·Nav2를 사용한다.
RTAB-Map VSLAM도 소스에 포함하지만 같은 `map → odom`을 중복 발행하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/bimanual-robot/.runtime/jdamr/pc_ws/install/setup.bash"
ros2 launch jdamr_cube_cartographer cartographer_real.launch.py
```

현행 실차 모델의 `base_footprint` 원점은 전륜축의 바닥 투영이다.
양팔 시뮬 모델 `hold_flow_description`의 차체 중심 원점과 105 mm 차이가 있다.
주행의 검증된 TF를 임의로 양팔 시뮬 모델로 바꾸거나 두 모델을 같은 프레임으로 동시에 띄우지 않는다.
카메라 렌즈는 상판 720 mm + 상대 높이 540 mm = 바닥 기준 1260 mm다.
1 cm 주차 성공은 사용자가 확인했다. 이후 제어 수정본의 실차 결과와 반복 정밀도는 별도로 확인한다.

## 개발과 검증

```bash
cd "$HOME/bimanual-robot"
git switch main
git pull --ff-only origin main
git switch -c feat/navigation-task
```

AMR 작업은 `src/navigation/AGENTS.md`를 따른다. 기능 수정과 실제 반영을 우선하고,
변경한 기능의 필요한 구문·로드·비이동 확인을 수행한다. 별도 리뷰·pytest·전체 검사를
출발 조건으로 자동 추가하지 않는다. 기존 AMCL Axis B 기록과 실차 정확도의 미확인
항목은 보존한다. 수정 파일을 명시해 작업 브랜치에 커밋하며 push·PR·main 반영은
사용자가 요청한 범위에서 수행한다.

원본 파일별 SHA와 제외 범위는
[가져오기 명세](../../config/navigation/jdamr_import_manifest.json),
운영 보조 도구와 자산의 출처는 [도구 출처](../../tools/jdamr/provenance.json)에 있다.
원본 저장소의 미커밋 RViz 변경과 개인 에이전트 상태·영상·주행 기록은 가져오지 않았다.
