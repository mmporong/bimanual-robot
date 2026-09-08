# HOLD THE FLOW · 이동형 양팔 로봇 프로젝트

동국대 DAPIER 부트캠프 최종 팀 프로젝트의 **팀 공용 저장소**다. 팀원이 설계·코드·실험 데이터 규격·회의 결정·검증 증거를 이곳에서 함께 관리한다. 현행 태스크는 웹 요청을 받은 이동형 양팔 로봇이 주방에서 물을 준비해 손님 테이블에 서빙하고 충전소로 복귀하는 단일 시나리오다.

- 시작: 2026-08-19 (킥오프 회의)
- 기간: 약 3개월
- 개발 기준: Ubuntu 24.04 · ROS 2 Jazzy · C++17 · Python 3.12 · LeRobot 0.6.1
- 시뮬레이터: Isaac Sim 6.0 · ROS 2 Bridge
- 운영 방식: Issue → 작업 브랜치 → Pull Request → 로컬 검증 → `main`
- 태스크·정책·구현 상세 단일 원본: [2026-09-07 물 서빙 로봇 회의 결정](docs/20260907_물서빙로봇_회의결정과_실행범위.md)
- 기구 수치 단일 원본: [`hold_flow_mechanical_v0_2.yaml`](design/mechanical/hold_flow_mechanical_v0_2.yaml)

## 프로젝트 한 줄 정의

### 충전소에서 출발해 한 잔을 준비하고 서빙한 뒤 돌아온다

**로봇은 충전소에서 웹 요청을 받고 주방으로 자율주행한다. 왼팔로 컵을, 오른팔로 물통을 조작해 물을 따른 뒤 컵을 로봇 선반에 싣는다. 손님 테이블로 이동해 컵을 내려놓고 충전소로 복귀한다.**

현재 개발과 실물 검증은 이 물 서빙 시나리오 하나에 집중한다. 주행 중에는 팔이 컵을 들지 않고 선반에 적재하며, 팔 조작 중에는 베이스를 정지한다. ACT 정책은 조작 구간만 담당하고 SLAM/Nav2는 별도 이동 계층으로 유지한다. 물 양은 YOLO 계열 비전으로 판정하되, 검출 형식·컵 조건·높이-부피 보정은 본수집 전에 확정한다.

- 현행 기준 문서: [docs/20260907_물서빙로봇_회의결정과_실행범위.md](docs/20260907_물서빙로봇_회의결정과_실행범위.md)
- 세션 인계: [docs/20260904_양팔로봇_프로젝트_인계.md](docs/20260904_양팔로봇_프로젝트_인계.md)
- 과거 붓기안 이력: [docs/20260828_1안_확정_이동형_양팔_붓기.md](docs/20260828_1안_확정_이동형_양팔_붓기.md)
- 구현 아키텍처 참고: [docs/20260901_구현아키텍처_ROS2_CPP_Python_ACT_IsaacSim.md](docs/20260901_구현아키텍처_ROS2_CPP_Python_ACT_IsaacSim.md) — 태스크·policy 경계는 2026-09-07 회의 문서가 우선
- 250 mm 차체 계산: [docs/20260901_250mm_차체_계산검증_v0.2.md](docs/20260901_250mm_차체_계산검증_v0.2.md)
- 기구설계 명세: [docs/20260901_기구설계_제작명세_v0.2.md](docs/20260901_기구설계_제작명세_v0.2.md)
- 기구 검증표: [docs/20260901_기구설계_검증체크리스트_v0.2.md](docs/20260901_기구설계_검증체크리스트_v0.2.md)
- JD-AMR 구동계·K1 Max·평행그리퍼 반영: [docs/20260901_JDAMR_K1Max_평행그리퍼_설계반영.md](docs/20260901_JDAMR_K1Max_평행그리퍼_설계반영.md)
- CAD·URDF 파라미터: [design/mechanical/hold_flow_mechanical_v0_2.yaml](design/mechanical/hold_flow_mechanical_v0_2.yaml)
- 실패 데이터 저장·진단·활용 규약: [data/failures/README.md](data/failures/README.md)
- 출력용 CAD 원본·STEP·STL: [design/cad/README.md](design/cad/README.md)
- 양팔 ROS 2 모델: [src/hold_flow_description/README.md](src/hold_flow_description/README.md)
- 확장된 단일 URDF: [src/hold_flow_description/urdf/hold_flow.urdf](src/hold_flow_description/urdf/hold_flow.urdf)
- 과거 붓기안 논문 적용 이력: [research/R31_1안_이동형_양팔_붓기_논문적용.md](research/R31_1안_이동형_양팔_붓기_논문적용.md)
- 팀 작업 방식: [docs/TEAM_WORKFLOW.md](docs/TEAM_WORKFLOW.md)
- 과거 후보 회의 기록: [docs/20260828_회의결과_주제후보_역할분담.md](docs/20260828_회의결과_주제후보_역할분담.md)
- 과거 붓기안 발표자료: [docs/20260828_HANDOVER_양팔로봇_5페이지_발표자료.pptx](docs/20260828_HANDOVER_양팔로봇_5페이지_발표자료.pptx)
- 과거 붓기안 발표 디자인 리포트: [docs/20260828_HOLD_THE_FLOW_발표자료_디자인_리포트.docx](docs/20260828_HOLD_THE_FLOW_발표자료_디자인_리포트.docx)
- 과거 붓기안 프로젝트 사이트: [HOLD THE FLOW · Team Project](https://hold-the-flow-bimanual.mmporong.chatgpt.site) — 현행 물 서빙 흐름 미반영

## CAD와 URDF

차체는 문서상의 치수표에 머물러 있지 않다. CadQuery 원본에서 K1 Max용 STEP·STL 14종을 다시 만들 수 있고, 하판·중판·상판과 팔 보강판, 카메라 마스트, Astra 거치대, LDS-03 받침대, 주행·캐스터 어댑터가 포함돼 있다. 상·중·하판과 상판 어댑터의 기준 위치는 [`hold_flow_printed_structure.step`](design/cad/exports/step/hold_flow_printed_structure.step)에서 확인한다.

ROS 2 모델은 250 mm 이동 베이스와 SO-101 두 대, ggao50 평행 그리퍼 두 대, Astra S, LDS-03을 하나의 TF 트리로 묶었다. 좌우 팔은 같은 원본에서 `left_`와 `right_` prefix를 붙여 생성하며, 평행 죠는 prismatic·mimic joint로 움직인다. Xacro 확장본과 커밋된 URDF가 같은지, 메시 참조 51개가 실제로 존재하는지, 바퀴 간격과 센서·팔 좌표가 설계값과 일치하는지는 검증 스크립트가 확인한다.

다만 장공을 원형공으로 바꾸는 일은 아직 남았다. SO-101 베이스, C018 혼과 JD-AMR 휠 허브, 볼 캐스터, Astra와 LDS-03 체결부를 실측한 뒤 어댑터를 확정해야 한다. 현재 출력물은 조립 검토용 P0이며 실측을 건너뛴 양산판이 아니다.

## 기술 선택과 구현 상태

아래 표의 `채택`은 현행 기준 기술, `권장`은 첫 구현 후보, `미확정`은 실험이나 팀 결정이
필요한 항목이다. 기술 이름이 적혀 있어도 구현 완료를 뜻하지 않는다.

| 파트 | 기술 | 결정 상태 | 구현 상태 | 직접 구현·검증할 부분 |
|---|---|---|---|---|
| 시뮬레이션 | Isaac Sim 6.0, USD, ROS 2 Bridge | 채택 | 계획 | 최종 URDF 반입, 센서·TF·ROS 2 Bridge smoke |
| 요청 입력 | 웹 UI, FastAPI, ROS 2 Action client | 권장 | 계획 | 요청 검증, request ID, 테이블 목표 전달 |
| 지도·이동 | SLAM Toolbox, AMCL, Nav2 DWB, Collision Monitor, Docking Server | 권장 | 계획 | station, 장애물 회피, 반복 접근·도킹 평가 |
| 로컬 정렬 | RGB-D, OpenCV, tf2 | 권장 | 계획 | 주방·선반·테이블 상대 자세와 시간축 검증 |
| 양팔 조작 | IK, 사전 검증 궤적, LeRobot ACT | 일부 채택 | 기존 IK 수정 필요·정책 계획 | 주방 policy 1, 테이블 policy 2, 미정 policy 번호 경계 확정 |
| 실행·안전 | C++17, FollowJointTrajectory, command mux | 권장 | 계획 | 관절 한계, 타임아웃, 팔 간 거리, 안전 정지 |
| 실물 연결 | LeRobot `bi_so_follower` Python bridge | 권장 | 양팔 ROS mux 어댑터 smoke 미검증 | 좌우 SO-101 포트 단독 소유, 상태·명령 변환 |
| 모방학습 | LeRobot ACT, PyTorch | 채택 | 현행 태스크 데이터·모델 없음 | 시연 수집, 학습·배포, 실패 분포 수집·수정·재학습 |
| 물 양 인지 | YOLO 계열 비전 | 채택 | 형식 미확정·구현 전 | 컵·액체·흘림 관측, 컵별 높이-부피 보정, 결과 판정 |
| 충전 | Nav2 도킹 목표, 충전 인터페이스 | 일부 권장·미확정 | 계획 | 복귀 자세, 접점, 충전 시작 판정 |

Isaac Sim 6.0의 [공식 최소 VRAM은 16GB](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)이며 현재 확인한 개발 노트북은 8GB다. ROS 2 코드와 자산은 이 장비에서 준비하되, Isaac Sim 실행은 Compatibility Checker를 통과한 GPU 워크스테이션이나 원격 장비에서 검증한다.

## 팀 저장소 운영

- 이 저장소는 특정 팀원의 개인 작업 기록이 아니라 **팀 공용 단일 원본**이다.
- 담당 작업은 GitHub Issue에 목표·범위·통과 조건·증거를 적고 기능별 브랜치에서 진행한다.
- `main`에는 직접 푸시하지 않는다. 작업자가 Pull Request에 변경 내용과 로컬 검증 결과를 남기면 팀원 승인 없이 직접 병합할 수 있다.
- 팀원 리뷰는 필수 승인이 아니라, 공용 인터페이스·안전·실기체 변경처럼 교차 확인이 필요한 작업에서 선택적으로 요청한다.
- 개인 실험도 재현 가능한 코드·설정·결과 요약을 남겨 팀원이 이어서 실행할 수 있게 한다.
- 데이터셋·모델·rosbag 같은 대용량 파일은 저장소에 직접 올리지 않고 `data/README.md`의 인덱스로 위치와 버전을 기록한다.
- 세부 절차는 [팀 협업과 작업 관리](docs/TEAM_WORKFLOW.md)를 따른다.

## 현재 역할

- 강사/멘토: 교육·리뷰
- [@mmporong](https://github.com/mmporong): **SLAM/Nav2·장애물 회피**, 옆 세션의 이동 계층 작업, IL·IK 세부 연결, 최종 모델 URDF 수신 후 Isaac Sim 시뮬레이션
- 협업자 [@Minsuk-ji](https://github.com/Minsuk-ji): write 권한 수락, 세부 담당 확정 대기
- 팀원 [@jangjunseo05](https://github.com/jangjunseo05): write 초대 수락 대기, 세부 담당 확정 대기
- 팀 1순위: 회의에서 지목한 policy 학습·배포, rollout 실패 케이스 수집, 수정·보강·재학습
- 공통 선결: `3번 policy` 의미, policy 2 사용 팔, YOLO 물 양 규격 확정

파트별 언어와 ROS 인터페이스는 [구현 아키텍처](docs/20260901_구현아키텍처_ROS2_CPP_Python_ACT_IsaacSim.md)를 참고하되, 태스크·policy 범위와 구현 우선순위는 [2026-09-07 회의 결정](docs/20260907_물서빙로봇_회의결정과_실행범위.md)을 따른다.

- 단계별 기술·입출력·성공·실패 처리: 현행 회의 문서 3~5절
- ACT 수집·학습·배포·실패 개선: 현행 회의 문서 6절과 [data 규격](data/README.md)
- Isaac Sim 범위와 안전 조건: 현행 회의 문서 8~9절
- 실패의 관측·원인·조치·동일 조건 재시험: 현행 회의 문서 11절과 [실패 데이터 규약](data/failures/README.md)

## 폴더 구조

```
bimanual-robot/
├── design/        기구 파라미터, CadQuery 원본, STEP·STL 출력물
├── docs/          회의록·결정사항·담당 파트·설계 문서 (파일명 YYYYMMDD_ 접두)
├── research/      리서치 산출물 (양팔 텔레옵 선례, 적용 사례, GPU 비용 등)
├── data/          데이터셋 규격·인덱스·실패 데이터 계약 (실데이터는 커밋하지 않음)
│   ├── failures/  실패 저장·진단·재시험·활용 규약
│   └── schema/    에피소드·실패 레코드 스키마, dataset card 템플릿
├── src/           ROS 2·IK·ACT·Isaac Sim 구현 코드
├── site/          과거 붓기안 HTML 사이트 소스·배포 설정
├── tools/         수집·검증·변환 스크립트
├── PROGRESS.md    세션별 진행 로그 (최상단 append)
└── CLAUDE.md      팀 저장소 작업 규칙
```

## 현재 상태

- [x] 최종 태스크를 충전소→주방→선반 운반→테이블 서빙→충전소 복귀의 물 서빙 로봇으로 고정
- [x] 실제 키오스크 대신 웹 버튼 입력으로 고정
- [x] 커넥터 체결·범용 3태스크·음성/STT/VLA를 현행 범위에서 제외
- [x] 사용자 담당을 SLAM·장애물 회피·IL/IK 세부·URDF 이후 Isaac Sim으로 정리
- [x] 웹·Nav2·정렬·ACT·YOLO·선반·도킹의 입력·성공 조건·실패 처리 상세화
- [x] 물 서빙용 episode sidecar에 policy·다중 물체·물 양·실패·사람 개입 추적 추가
- [x] 실패의 관측·가설·확정 원인·조치·동일 조건 재시험을 분리한 레코드 규격 추가
- [x] 실패 근거 복사·SHA-256 검증·집계 CLI와 데이터 계약 회귀 테스트 구현
- [ ] `3번 policy`의 정확한 시작·행동·종료 조건 확정
- [ ] policy 2의 사용 팔 확정: 회의 본문은 왼팔, 괄호 주석은 오른팔
- [ ] YOLO 태스크 형식·컵 재질·목표 물 양·허용 오차·높이-부피 보정 확정
- [ ] 로봇 선반과 충전 도킹의 기구·좌표·인터페이스 확정
- [x] 킥오프 회의 기록·결정사항 정리
- [x] 데이터 저장 규격 초안 (스키마·dataset card·포함/제외 기준)
- [x] 과거 3인 회의의 두 후보 시나리오·잠정 역할 문서화
- [x] 구형 붓기안의 접촉 센서·실패 복구·모바일 양팔 논문 재검증
- [x] 구형 붓기안의 논문 근거를 제어 상태기계·센서 책임·평가표·실패 데이터 규격으로 변환
- [x] 물 따르기 중심의 구형 Plan A를 만들었으며, 2026-09-07 물 서빙 전체 흐름으로 대체
- [x] 물 따르기 전용 5쪽 발표자료와 디자인 리포트 제작
- [x] 구형 붓기안의 ROS 2·C++·Python·Nav2·ACT·Isaac Sim 구현 경계 문서화(현행 태스크·policy는 2026-09-07 회의 문서가 우선)
- [x] 250 mm 정사각 차체·SO-101×2·단일 Astra S 기둥의 기구 명세와 검증표 작성
- [x] 5.092 kg 계산 질량·3점 접지·C018 12 V 주행·출력 구조 계산
- [x] JD-AMR 65 mm 바퀴·볼 캐스터 재사용, K1 Max 한 장 판, ggao50 평행그리퍼 P0 기준 확정
- [x] K1 Max용 파라메트릭 CAD와 STEP·STL 14종 생성, B-Rep·빌드 볼륨·서포트 검사 통과
- [x] 모바일 베이스·SO-101×2·평행그리퍼×2·Astra·LDS-03 Xacro/URDF 생성과 TF 검사 통과
- [x] 팀 공용 구현 아키텍처 HTML 사이트 제작·ChatGPT Sites 배포
- [ ] M0 부품 실측: SO-101 체결홀·Astra S·평행그리퍼·JD-AMR 바퀴/캐스터·구동부 외피와 질량
- [ ] M1 K1 Max 출력 공차·6 mm 로드 시험편과 250 mm 한 장 판 평탄도 검증
- [ ] M2~M4 차체 건식조립·정적하중·3점 지지 안정성 검증
- [ ] Isaac Sim 6.0 실행 장비 Compatibility Checker와 ROS 2 Bridge smoke test
- [ ] 주방의 컵·물통 파지와 건식 붓기 POC
- [ ] SLAM/Nav2 충전소·주방·테이블 반복 접근 기준선
- [ ] Depth 기반 주방·선반·테이블 로컬 정렬 기준선
- [ ] 웹 요청 → 이동 → 주방 policy → 선반 운반 → 테이블 policy → 복귀 상태기계 통합
- [ ] 지정 policy LeRobot smoke dataset과 ACT 과적합 기준선
- [ ] 정책 배포 rollout, 실패 케이스 수집, 수정·재학습 비교
