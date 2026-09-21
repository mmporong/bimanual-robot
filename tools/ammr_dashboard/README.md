# AMMR 실기체 센서 관제

Pi에 연결된 Astra S 깊이 카메라, 2D LiDAR, 베이스 상태를 한 화면에서 확인하는
읽기 전용 대시보드다. ROS publisher와 로봇 명령 API를 만들지 않는다.

## 실행

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/jdamr_ws/install/setup.bash
source $HOME/astra_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
python3 $HOME/bimanual-robot/tools/ammr_dashboard/ammr_dashboard.py --port 8090
```

브라우저에서 `http://<Pi-IP>:8090`을 연다. 기본 구독 토픽은 다음과 같다.

- Astra S: `/camera/depth/image_raw`, `/camera/color/image_raw`
- LiDAR: `/scan`
- 베이스: `/odom`, `/cmd_vel`, `/cmd_vel_smoothed`
- 상태: `/battery_state`, `/imu/data_raw`, `/depth_navigation/status` (JSON String)

카메라 드라이버가 다른 네임스페이스를 쓰면 `--depth-topic`, `--color-topic`으로
바꾼다. `/health`는 Depth와 LiDAR가 모두 2초 이내에 들어오면 HTTP 200을 반환한다.

LiDAR 표시 기준은 기본 `laser_link`이며, 스캔을 다른 프레임의 원점과
방향으로 보려면 변환을 명시한다. 예를 들어 `laser_link -> base_footprint`
변환이 yaw 180°, x -0.010 m, y 0 m인 역방향 장착은 다음과 같다.

```bash
python3 $HOME/bimanual-robot/tools/ammr_dashboard/ammr_dashboard.py --port 8090 \
  --scan-yaw-deg 180 --scan-x-m -0.010 --scan-y-m 0 \
  --scan-display-frame base_footprint
```

이 변환은 각 스캔 점의 측정 원점과 표시 방향을 바꾸는 것이다. 라이다 거리에서
차체 외곽을 빼는 여유거리 계산은 하지 않는다. 변환 숫자는 유한한 값만
받으며 `NaN`, `inf`는 실행 시 거부한다.

`/depth_navigation/status`의 `age_s`는 상태 발행자가 계산한 원본 경과 시간이므로
API에서 `source_age_s`로 보존한다. API의 `age_s`는 대시보드가 해당 상태
메시지를 수신한 뒤 흐른 시간이며, 2초 이상이면 `live: false`다.

## 검증

```bash
python3 -m unittest -v $HOME/bimanual-robot/tools/ammr_dashboard/test_ammr_dashboard.py
```

## Pi 설치 기록

- 카메라: Orbbec Astra S `2bc5:0402`, serial `17120813010`
- 드라이버: `orbbec/ros2_astra_camera` commit
  `f7e71d9ce806e788cb48d8580aac2c778fba4214`
- Jazzy 호환 패치: `$HOME/bimanual-robot/config/navigation/astra_ros2_jazzy.patch`
- 운용 모드: Depth `320×240 @ 30Hz`, RGB·IR·point cloud 비활성
- 서비스 원본: `$HOME/bimanual-robot/deploy/systemd/jdamr-astra-camera.service`,
  `$HOME/bimanual-robot/deploy/systemd/jdamr-ammr-dashboard.service`

Depth는 실제 관제와 장애물 인식용 원본을 보존하면서 Pi 4B의 USB·CPU 부하를
제한한다. RGB가 필요한 작업 단계에서는 Astra 색상 스트림 지원 여부와 USB 처리량을
다시 실측한 뒤 `enable_color`을 켠다.

대시보드는 센서 수신률 30Hz를 그대로 계측하지만, 브라우저용 Depth JPEG는 약 5Hz로
제한한다. 화면을 열어 둔 동안에도 주행·SLAM 프로세스의 CPU 여유를 우선하기 위해서다.
