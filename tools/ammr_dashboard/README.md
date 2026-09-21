# AMMR 읽기 전용 관제

## 관리 원본 이전

2026-09-21 승인 화면부터 대시보드 관리 원본은
[`mmporong/robot-dashboard`의 `projects/ammr-live/`](https://github.com/mmporong/robot-dashboard/tree/main/projects/ammr-live)다.
화면·수집기 수정과 배포는 그 저장소에서 진행한다. 이 디렉터리는 이관 기준 커밋
`43d4afc`의 보존본이며 새 기본 화면을 만드는 경로로 사용하지 않는다.
기존 설치에 관한 아래 설명은 이관 전 기록이다. 새 실행·업데이트 절차는 관리 원본의 README를 따른다.

파이 자원, RGB-D·LiDAR, ROS 그래프, 지도·위치·계획 경로, 주행 차단 근거를 보는
화이트톤 화면이다. 주행 명령 API, ROS 명령 publisher, 제어 service client는 없다.
대시보드 실행이 카메라나 주행 서비스를 자동으로 시작하지 않는다.

## 표시 내용

- 호스트: CPU 사용률·주파수·온도, 현재/과거 저전압·스로틀링, 메모리·swap·디스크,
  1/5/15분 load average, 가동 시간, CPU 상위 프로세스 6개, 주요 systemd 서비스.
- ROS: 발견한 노드 수·전체 이름·동일 이름 관측 수, 등록 토픽 목록, domain·discovery 설정.
  노드 수는 OS 프로세스 수가 아니다. 노드 발견만으로 SLAM 정상 동작을 판정하지 않는다.
- 센서: RGB/Depth, 수신률·깊이 유효 비율, LiDAR 전방위 점군·방향별 거리.
- 주행: 입력 → 감속기 출력 → 최종 명령 → odom 실측 속도. 정지한 Collision Monitor가
  0 명령을 반복 발행하지 않는 경우는 마지막 명령과 경과 시간으로 표시한다.
- 항법: `/map` 점유 지도, TF의 map 기준 위치, `/plan` 계획 경로, Nav2 목표 상태,
  박스 관측 안정성·거리·각도·처리 시간, 실행기 차단 이유와 충돌 감시 상태.

지도·위치·계획이 없으면 미수신으로 표시한다. 정적 지도의 갱신이 오래됐다고 지도를
지우지는 않지만, 그 지도만으로 SLAM이 실행 중이라고 표시하지 않는다. 지도 위치와
경로는 같은 frame일 때만 겹쳐 그린다. 현재 화면은 점유 지도 관제이며 3D SLAM 뷰어가 아니다.
센서 수신이나 서비스 active 상태도 실차 출발 승인과는 별개다.

## 운용 화면 구성

공간·센서 패널의 상하 경계를 맞추고, 주행·장치 상태와 ROS 진단을 분리한다.
구성 참고는 [Foxglove 패널 기반 레이아웃](https://docs.foxglove.dev/docs/visualization/layouts)이며,
현재 서비스에 맞춘 관측 전용 UI다. 특정 상용 관제 제품과 동일한 기능을 제공한다는 뜻은 아니다.

노드와 토픽 오른쪽의 역할 설명은 현재 이름·구성에 대응하는 정적 설명이다.
검색은 경로와 역할 모두에 적용한다. 등록되지 않은 이름은 추정하지 않고 미등록으로 표시한다.
같은 전체 경로의 노드는 한 행에 묶어 관측 개수를 표시한다. 같은 이름이 여럿 보이는 것만으로
중복 프로세스 실행이나 장애를 확정하지 않는다. 그래프에는 구독만 있는 토픽도 포함되므로
`/map` 이름이 보인다고 지도 발행·수신이 확인된 것은 아니다.

온도 70°C 이상은 주의, 80°C 이상은 빨간 경고이며 디스크 사용률은 80%/90% 기준이다.
이 값은 화면의 경고 기준이고 주행 정지 조건이나 제조사 보증 한계가 아니다.
`vcgencmd`의 현재 제한 상태와 과거 이력을 별도로 표시한다.

## 실행과 접속

현재 운용은 Pi와 노트북이 신뢰된 실습 LAN에서 domain 12 / SUBNET으로 통신한다.
DDS를 인터넷에 노출하지 않는다. HTTP 기본 바인딩은 `127.0.0.1`이며 SSH 터널로 접속한다.

Pi 설치 디렉터리에서 실행:

```bash
cd "$HOME/ammr_dashboard"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
python3 ammr_dashboard.py --bind 127.0.0.1 --port 8090 \
  --scan-yaw-deg 180 --scan-x-m -0.010 --scan-y-m 0 \
  --scan-display-frame base_footprint
```

노트북에서 현재 Pi 주소에 SSH 터널을 연결한 뒤 `http://127.0.0.1:8090`을 연다.
예: `ssh -N -L 8090:127.0.0.1:8090 lim@<현재-Pi-IP>`.
바인딩을 LAN으로 넓히면 별도 인증이 없는 읽기 전용 API와 영상이 노출되므로 주의한다.

현재 하부차체의 LiDAR는 yaw 180°, x -0.010 m, y 0 m다. 표시한 거리는 좌표 원점부터의
거리이며 차체 외곽에서 장애물까지의 안전 여유가 아니다. 다른 기체에서는 장착값을 갱신한다.

## 지속 실행과 화면 상태

- 영구 서비스: `deploy/systemd/jdamr-ammr-dashboard.service`.
- 설치 파일: Pi의 `$HOME/ammr_dashboard/` 아래 `ammr_dashboard.py`, `host_metrics.py`,
  `dashboard.html`, `dashboard.js`. 이전 파일은 그 아래 `backups/`에 보존한다.
- 현재 카메라 서비스는 `jdamr-box-rgbd.service`다. 예전 depth-only
  `jdamr-astra-camera.service`를 대시보드 종속성으로 다시 실행하지 않는다.
- 서비스 enable은 재부팅 후 대시보드 실행 설정이며 재부팅 시험 완료를 뜻하지 않는다.
- 전체/간결 보기와 접힌 영역만 브라우저 localStorage에 저장한다. 센서 상태는 저장하지
  않으며 연결이 끊기면 숫자와 정상 표시를 지운다. 재접속 시 새 데이터를 받는다.

## 부하와 freshness

HTTP는 캐시된 상태만 읽는다. 호스트 계측은 2초, ROS 그래프는 5초, 프로세스는 최소 5초,
서비스·스로틀링은 최소 10초 간격이다. command timeout은 0.5/1초로 제한한다.
CPU와 프로세스 CPU는 모든 코어를 합쳐 100%인 기준이며 MB/GB는 10진 단위다.

이미지와 지도는 최신 1개만 보관하고 별도 워커에서 변환한다. 영상 preview는 최대 3fps,
지도 변환은 최대 0.5Hz이고, 지도는 400만 셀·미리보기 한 변 512px로 제한한다.
기존 ROS 영상 발행률이나 주행 제어 주기는 변경하지 않는다. OpenCV/BLAS 스레드도 제한한다.
원본 수신 FPS와 웹 preview FPS는 다르다.

수신 시각과 원본 데이터의 age를 구분하며 오래된 데이터를 새 READY로 표시하지 않는다.
`/api/state`의 `age_s`는 관제 수신 후 시간, `source_age_s`는 발행자/영상 원본 경과 시간이다.
기본 freshness는 2초, 호스트는 6초, 그래프는 12초다. `/health`는 Depth/LiDAR 수신만
검사하므로 그 HTTP 200을 SLAM·주행·도킹 정상 판정으로 사용하지 않는다.

## 검증

```bash
cd "$HOME/bimanual-robot-ammr"
python3 -m unittest -v tools/ammr_dashboard/test_ammr_dashboard.py \
  tools/ammr_dashboard/test_host_metrics.py
node --check tools/ammr_dashboard/dashboard.js
node --test tools/ammr_dashboard/test_dashboard_ui.cjs
python3 -m py_compile tools/ammr_dashboard/ammr_dashboard.py \
  tools/ammr_dashboard/host_metrics.py
systemd-analyze verify deploy/systemd/jdamr-ammr-dashboard.service
```

ROS를 포함한 실행에는 Jazzy의 rclpy, sensor/nav/geometry/action/nav2 messages,
tf2_ros, Python OpenCV·NumPy가 필요하다. 비구동 UI 시험은 실물과 분리한
LOCALHOST/domain 197에서 한다. 실물 배포 때는 대시보드 서비스만 교체하고 기존
베이스·센서·주행 서비스를 재시작하지 않는다.
