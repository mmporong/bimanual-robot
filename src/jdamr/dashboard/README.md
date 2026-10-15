# JD-AMR 실시간 대시보드

이 디렉터리는 2026-10-15 라즈베리 파이에서 실행 중이던 읽기 전용 대시보드 소스의 보존본이다. ROS 2 토픽을 구독하고 상태 JSON, 지도·RGB·깊이 미리보기를 `127.0.0.1:8090`에 제공한다. 로봇 명령 publisher나 service client는 만들지 않는다.

## 실행 의존성

- ROS 2 Jazzy의 `rclpy`, `action_msgs`, `geometry_msgs`, `nav2_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`
- Python 3의 OpenCV(`cv2`)와 NumPy
- 같은 디렉터리의 `dashboard.html`, `dashboard.js`, `host_metrics.py`

파이의 현재 실행 인자는 다음 systemd 템플릿에 보존했다.

`../deployment/systemd/jdamr-ammr-dashboard.service.in`

서버는 기본적으로 루프백에만 바인딩한다. 다른 PC에서는 SSH 포트 포워딩을 사용한다.

```bash
ssh -N -L 8090:127.0.0.1:8090 <robot-host>
```

소스 해시와 원본 위치는 `../deployment/provenance.json`에서 확인한다.
