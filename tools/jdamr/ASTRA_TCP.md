# Windows Astra → Pi 센서 수신

Windows OpenNI2 브리지가 USB를 유지하고 Pi `192.168.0.159:5600`에 접속한다. 현재 허용 송신 IP는 `192.168.0.175`이며 변경 시 서비스의 `--peer`를 변경한다. 주문·팔 실행은 별도 `:8765` API가 담당한다. 이 수신기는 센서만 발행한다.

## 송수신 계약

- `AST1` + uint32 big-endian JSON 길이 + UTF-8 JSON + RGB PNG/JPEG + depth PNG16.
- 헤더 필드는 사용자 제시 규격의 seq/rgb/depth/send_ns. RGB 640×480 bgr8이며 compression은 png 또는 jpeg, depth 320×240 16UC1 mm이며 compression=png16, 양쪽 frame_id는 camera_color_optical_frame. JPEG 사용 시 RGB 헤더의 len만 실제 JPEG 바이트 수로 갱신하며 encoding=bgr8은 유지한다.
- depth는 color 좌표계에 registration되어야 한다. 각 CameraInfo는 해당 해상도의 장치 보정값이며 D/K/R/P와 distortion_model=plumb_bob을 포함한다. 기준 영상 보정값을 복사해 쓰지 않는다.
- Pi → Windows: `PING` + uint64 big-endian Pi UNIX ns.
- Windows → Pi: `PONG` + uint64 big-endian echoed Pi ns + uint64 big-endian Windows UNIX ns.
- PONG과 AST1은 동일 송신 잠금으로 직렬화한다. PONG을 헤더·PNG 중간에 삽입하지 않는다. 완성된 프레임 사이에 보낸다.
- 유효 PONG 1개부터 최소 RTT 표본으로 촬영 시각을 변환하고, 최근 최대 120초의 유효 표본을 유지한다. RTT 상한 기본 100ms(--clock-max-rtt-ms, 최대 200ms), 유지 시간 기본 120초(--clock-hold-s, 최대 120초), 영상 나이 상한 500ms. 큰 RTT의 PONG 때문에 유효 보정값을 바로 버리지 않는다. 수신 시각으로 덮어쓰거나 오래된 영상을 허용하는 변경은 아니다. RTT/2는 표본의 추정 불확실성이지 시계 드리프트까지 보장하는 값은 아니다. 120초 안에 유효 PONG을 갱신해야 한다.
- 수신 시각으로 촬영 시각을 덮어쓰지 않는다. 오래되거나 미래인 영상은 발행하지 않는다. 최신 디코딩 프레임만 보관하며, TCP 내부에 쌓인 오래된 데이터도 촬영 시각 기준으로 제외한다.
- RGB/depth는 별도 촬영 시각을 유지하며 시간 차이는 상태 토픽에 기록한다. 이 수신기에서 두 촬영을 동기화하는 것은 아니다.

## ROS 출력

| 토픽 | 형식 |
|---|---|
| /camera/color/image_raw | sensor_msgs/msg/Image, bgr8 |
| /camera/depth/image_raw | sensor_msgs/msg/Image, 16UC1 mm |
| /camera/color/camera_info | sensor_msgs/msg/CameraInfo |
| /camera/depth/camera_info | sensor_msgs/msg/CameraInfo |
| /camera/tcp_status | std_msgs/msg/String, JSON 상태 |

영상·보정값 QoS는 best effort / volatile / keep last 1. 같은 센서의 Image와 CameraInfo는 같은 촬영 시각을 사용한다. 발행할 영상이 없으면 과거 프레임을 재발행하지 않는다.

## 운영

Pi user systemd unit: `jdamr-astra-tcp.service`. 설치 원본은 tools/jdamr/systemd/jdamr-astra-tcp.service. Windows 송신기 연결을 기다리기 위해 유지하며 자동 부팅 enable은 하지 않는다.

```bash
systemctl --user start jdamr-astra-tcp.service
systemctl --user stop jdamr-astra-tcp.service
journalctl --user -u jdamr-astra-tcp.service -n 30
```

기존 USB Astra 발행기와 동시에 사용하지 않는다. 이번 배포는 독립 센서 수신 서비스이며 기존 실행기의 카메라 자동 기동 서비스를 교체하지 않았다. 주행 연결 전에는 카메라 기동 경로 전환, 기존 광학 TF 유지, Windows의 registration/CameraInfo/미러·크롭 일치, 실제 영상 지연을 확인해야 한다. 320×240 및 5fps 이상 입력 지원과 1cm 주차 달성은 별개다.

## Pi 비교 자료

2026-10-29 Pi 소스 확인: astra.launch.xml의 flip_color=false, flip_depth=false. ob_camera_node.cpp는 setMirroringEnabled(false)를 호출한다. 사용자 표현 color_mirror/depth_mirror 대신 실제 드라이버 설정명을 기록했다.

전달 묶음: `$HOME/Downloads/astra_windows_comparison_20261029.zip`. RGB PPM 원본과 2026-10-29T03:05:23.712091Z 교시 저장 CameraInfo를 포함한다. 새 ros2 topic echo 결과가 아니다. 저장 자료에 없던 R/distortion_model을 측정값으로 만들지 않았다.

확인 범위: 로컬 TCP에서 분할 전송, PONG 시각 변환, RGB와 uint16 깊이값 보존, 오래된 영상 제외 확인. 실물 이동 없이 수행했다. Windows 실제 PNG 수신과 RGB·깊이 ROS 발행도 확인했다. 고정 0.5초 socket recv timeout은 3초 레코드 읽기 한도 안에서 부분 수신을 이어받게 고쳤다. 연결 종료 사유와 읽기 단계를 journal에 남긴다. 실제 PNG 입력은 약 6~7fps·3.1~3.5MB/s지만 촬영→수신 지연이 0.6~1.3초 이상인 구간이 있어 연속 제어 준비 완료로 판단하지 않았다. RGB JPEG 품질 95 송신 전환 후 실제 지연 재확인이 필요하며 깊이는 PNG16을 유지한다. CameraInfo K/P/D와 크기는 교시 저장값과 일치했다. RGB·깊이 촬영 시각 차이는 중앙값 약 30~36ms, 상위 95% 약 46~47ms였다. 이 수치는 정지 수신 구간의 진단이며 주차 성능 증거가 아니다.
