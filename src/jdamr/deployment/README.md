# JD-AMR 런타임 재구성

양팔 저장소 하나를 clone한 뒤 JD-AMR의 SLAM·Nav2·실차 주행 환경을 다시 만들기 위한 배포 자료다. ROS 패키지는 `src/jdamr/`, 실차 실행 스크립트와 지도 자산은 저장소의 `tools/jdamr/` 및 런타임 설정 경로에 있다. 이 디렉터리는 저장소 밖에 있던 Astra 드라이버, systemd, udev, 대시보드를 보충한다.

## 1. 의존성 계획 확인

아래 명령은 계획만 출력하며 파일이나 시스템 패키지를 바꾸지 않는다.

```bash
cd ~/bimanual-robot
bash src/jdamr/deployment/install_dependencies.sh plan
```

설치를 실행할 때만 `--apply`를 사용한다. Git·vcstool·rosdep·colcon과 Astra의 C++ 빌드 의존성(`libuvc`, glog, Eigen3, OpenCV)을 준비한다. Astra는 공식 Orbbec 저장소의 `f7e71d9ce806e788cb48d8580aac2c778fba4214`를 가져온 뒤 현재 파이에서 확인한 10파일 호환 패치를 적용한다. `rosdep`은 양팔 저장소의 `src/jdamr`와 clone한 Astra 소스를 함께 검사한다.

```bash
cd ~/bimanual-robot
bash src/jdamr/deployment/install_dependencies.sh --apply
```

설치 스크립트는 colcon 빌드, systemd 설치·기동, 주행을 실행하지 않는다.

## 2. 작업공간 빌드

의존성 설치가 끝난 뒤 ROS 환경을 먼저 불러오고 Astra를 빌드한다. Python은 ROS Jazzy가 설치된 시스템 Python으로 고정한다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/jdamr_runtime/astra_ws
colcon build --symlink-install \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source ~/jdamr_runtime/astra_ws/install/setup.bash
ros2 pkg prefix astra_camera
```

JD-AMR 본체는 저장소의 기본 빌드 도구를 사용한다. 기본값은 실차에 필요한 9개 패키지만 선택하고 `/usr/bin/python3`를 CMake에 전달하며, 결과는 `~/bimanual-robot/.runtime/jdamr/pc_ws`에 둔다.

```bash
cd ~/bimanual-robot
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 tools/jdamr/runtime.py build-pc
```

전체 import 패키지가 필요한 별도 검증에서만 `build-pc --all`을 사용한다. 파이 반영 전 명령은 `/usr/bin/python3 tools/jdamr/runtime.py plan`으로 확인한다.

## 3. 실차 파일 배치

대시보드 보존본을 운영 경로로 복사한다.

```bash
mkdir -p ~/ammr_dashboard
cp ~/bimanual-robot/src/jdamr/dashboard/ammr_dashboard.py ~/ammr_dashboard/
cp ~/bimanual-robot/src/jdamr/dashboard/dashboard.html ~/ammr_dashboard/
cp ~/bimanual-robot/src/jdamr/dashboard/dashboard.js ~/ammr_dashboard/
cp ~/bimanual-robot/src/jdamr/dashboard/host_metrics.py ~/ammr_dashboard/
```

실차 설정 디렉터리에는 최소한 다음 파일이 필요하다.

- `self_filter_columns_20261012.yaml`: `src/jdamr/jdamr_cube_bringup/config/`의 현재 열 필터 설정
- `camera_mount_current.yaml`: `src/jdamr/jdamr_cube_navigation/config/`의 렌즈 높이 1.26m 설정
- 지도 YAML·PGM과 keepout mask: 저장소의 런타임 지도 자산

## 4. systemd 템플릿 렌더링

렌더러는 지정한 출력 디렉터리에 파일만 만든다. `daemon-reload`, `enable`, `start`, `restart`는 실행하지 않는다.

```bash
cd ~/bimanual-robot
python3 src/jdamr/deployment/render_systemd_templates.py \
  --output ~/jdamr-rendered-systemd \
  --user "$USER" \
  --home "$HOME" \
  --jdamr-ws "$HOME/jdamr_ws" \
  --astra-ws "$HOME/jdamr_runtime/astra_ws" \
  --deploy-dir "$HOME/jdamr_deploy" \
  --dashboard-dir "$HOME/ammr_dashboard"
```

렌더링 결과를 검토한 뒤 다음 위치에 설치한다.

- `*.service` → `/etc/systemd/system/`
- `*.service.d/*.conf` → 같은 이름의 `/etc/systemd/system/*.service.d/`
- `udev/*.rules` → `/etc/udev/rules.d/`

Astra 부팅 복구 helper는 `src/jdamr/jdamr_cube_bringup/scripts/jdamr-astra-usb-recover`에 있다. systemd 템플릿은 `/usr/local/sbin/jdamr-astra-usb-recover`를 사용한다.

## 5. 실차 장치 계약

- 제어보드 UART: `/dev/ttyS0`, `dialout`, `0660`
- YDLIDAR G4: CP2102 `10c4:ea60`, serial `0001`, symlink `/dev/ydlidar_g4`
- RGB-D: Orbbec Astra S, 별도 Astra 작업공간
- 대시보드: 루프백 `127.0.0.1:8090`, 읽기 전용

파이에서 확인한 원본 해시와 실행 인자는 [provenance.json](provenance.json)에 있다. 서비스 설치 후 기동 여부는 운영자가 별도 단계에서 결정한다.
