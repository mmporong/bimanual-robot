# Windows Astra → Pi 센서 수신

Windows OpenNI2 브리지가 USB를 유지하고 Pi `192.168.0.159:5600`에 접속한다. 현재 허용 송신 IP는 `192.168.0.175`이며 변경 시 서비스의 `--peer`를 변경한다. 주문·팔 실행은 별도 `:8765` API가 담당한다. 이 수신기는 센서만 발행한다.

## 송수신 계약

- `AST1` + uint32 big-endian JSON 길이 + UTF-8 JSON + RGB PNG + depth PNG16.
- 헤더 필드는 사용자 제시 규격의 seq/rgb/depth/send_ns. RGB 640×480 bgr8, depth 320×240 16UC1 mm, 양쪽 frame_id는 camera_color_optical_frame.
- depth는 color 좌표계에 registration되어야 한다. 각 CameraInfo는 해당 해상도의 장치 보정값이며 D/K/R/P와 distortion_model=plumb_bob을 포함한다. 기준 영상 보정값을 복사해 쓰지 않는다.
- Pi → Windows: `PING` + uint64 big-endian Pi UNIX ns.
- Windows → Pi: `PONG` + uint64 big-endian echoed Pi ns + uint64 big-endian Windows UNIX ns.
- PONG과 AST1은 동일 송신 잠금으로 직렬화한다. PONG을 헤더·PNG 중간에 삽입하지 않는다. 완성된 프레임 사이에 보낸다.
- 최소 3개 유효 PONG 후 최근 10초 중 최소 RTT 표본으로 촬영 시각을 변환한다. RTT 상한 100ms, 영상 나이 상한 500ms. RTT/2는 추정 불확실성이지 동기 정확도 보장이 아니다.
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

기존 USB Astra 발행기와 동시에 사용하지 않는다. 이번 배포는 독립 센서 수신 서비스이며 기존 실행기의 카메라 자동 기동 서비스를 교체하지 않았다. 주행 연결 전에는 카메라 기동 경로 전환, 기존 광학 TF 유지, Windows의 registration/CameraInfo/미러·크롭 일치, 실제 10fps 지연을 확인해야 한다. 320×240·10fps 지원과 1cm 주차 달성은 별개다.

## Pi 비교 자료

2026-10-29 Pi 소스 확인: astra.launch.xml의 flip_color=false, flip_depth=false. ob_camera_node.cpp는 setMirroringEnabled(false)를 호출한다. 사용자 표현 color_mirror/depth_mirror 대신 실제 드라이버 설정명을 기록했다.

전달 묶음: `$HOME/Downloads/astra_windows_comparison_20261029.zip`. RGB PPM 원본과 2026-10-29T03:05:23.712091Z 교시 저장 CameraInfo를 포함한다. 새 ros2 topic echo 결과가 아니다. 저장 자료에 없던 R/distortion_model을 측정값으로 만들지 않았다.

확인 범위: 로컬 TCP에서 분할 전송, PONG 시각 변환, RGB와 uint16 깊이값 보존, 오래된 영상 제외 확인. 실물 이동 없이 수행했다. 실제 Windows 송신 데이터 수신 및 주차 성능은 별도 확인 대상이다.
