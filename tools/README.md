# 공용 운영 도구

제품 로직은 src/, 화면은 apps/, 시뮬 실행은 simulation/, 공용 회귀는 tests/에 둔다.
이 폴더는 설치·배포·하드웨어 연결·검증 진입점을 관리한다.

| 경로 | 역할 |
|---|---|
| [jdamr](jdamr/README.md) | AMR 상태·출발·정지·PC 빌드·Pi stage 배포 |
| [servo](servo/README.md) | 서보 버스·판독·보정·복귀 |
| camera/ | 카메라 연결 진단·Astra RGB 중계 |
| recording/, rollout/ | LeRobot 수집·정책 실행 호환 패치 |
| eval/ | 실물 롤아웃 평가 기록 시트·집계(표·그래프) |
| data/ | 메타데이터·실패 레코드 검증·failure bank |
| design/ | 설계 산출물 생성 |
| ci/ | Markdown 링크·README 요약·PR 정책 |

일반 Python 도구는 저장소 루트에서 모듈로 실행한다. AMR의 기존 파일 진입점은 유지한다.

```bash
cd "$HOME/bimanual-robot"
python3 tools/jdamr/jdamr_depart.py status
python3 -m tools.servo.servo_read --help
python3 -m tools.data.failure_bank --help
python3 tools/ci/check_local_markdown_links.py
```

실물 동작은 사용자가 요청한 범위에서 실행한다. 도움말·오프라인 검사와 실제 실행을 구분한다.
