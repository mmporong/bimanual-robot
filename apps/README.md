# 화면과 서버

| 경로 | 용도 | 실행 |
|---|---|---|
| service_order/ | 주문 API·주문/미션 화면 | `python3 -m apps.service_order.service_order_server --backend immediate --port 8765` |
| camera_dashboard/ | RGB 카메라 관제·요청에 따른 정지 | `python3 -m apps.camera_dashboard.camera_dashboard --help` |
| project_site/ | 프로젝트 소개 소스 | 해당 폴더에서 `npm ci`, `npm run dev` |
| ../src/navigation/dashboard/ | Pi 주행 telemetry 대시보드 | [배포 안내](../src/navigation/deployment/README.md) |

Python 명령은 저장소 루트에서 실행한다.

```bash
cd "$HOME/bimanual-robot"
python3 -m apps.service_order.service_order_server --help
python3 -m apps.camera_dashboard.camera_dashboard --help
```

카메라·서보 옵션은 각 도움말과 [실물 실행 문서](../docs/guides/20260929_IK_픽앤플레이스_실행전_준비.md)를 따른다.
CPU immediate 주문 모드는 하드웨어를 움직이지 않는다. 주문 상태·저장·backend는 src/mission/에 두며 복제하지 않는다.
프로젝트 소개 사이트는 로봇 주행 대시보드와 별개의 화면이다.
