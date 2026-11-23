# 2026-11-03 급수대 접근 중 깊이 입력 지연 (발표 시연 회차)

## 결론과 조사 상태

- **현상:** 발표 시연 회차(주문 `261103002`, 15:38)의 급수대 최종 접근에서 깊이 시각 대기 5회, 테이프 기준 대기 4회가 났다. 대기마다 입력을 회복해 접근을 이어 갔지만, 테이프가 시야에서 빠진 뒤 측면 재정렬을 한 번 했고 두 번째 접근 끝에 `table lateral alignment requires more clearance`로 종료했다.
- **판단:** 현장 무선 혼잡을 가장 유력한 원인으로 본다. 근거는 세 갈래의 정황이다.
  1. 같은 코드·같은 장소에서 관람 인원이 모이기 전 회차(13:37 미션)는 깊이 시각 대기 0회였다. 그 사이 바뀐 것은 라이다 차체 반사 필터 경계(YAML)뿐이다.
  2. Wi-Fi를 거치지 않는 양팔 추론(같은 노트북의 USB 카메라·USB 팔)은 직후 팔 전용 시연에서 정상이었다. Wi-Fi로 깊이 영상을 받는 주행 쪽에서만 대기가 났다.
  3. 당시 20~30명이 같은 공간에서 같은 공용 Wi-Fi를 썼고, 다른 팀들도 무선 관련 문제를 겪었다(사용자 증언, 2026-11-23).
- **확정하지 않은 것:** 해당 회차의 촬영→수신 지연·RTT·채널 점유율 수치는 회수하지 못했다. 같은 회차에 파이 제어 경로 검증 지연(50 ms 이상)도 13:37 미션 1건에서 29건으로 늘었다. 그래서 측정으로 확정한 원인이 아니라 정황으로 가장 유력한 원인이며, 처리 지연과 겹친 복합 요인일 수 있다.
- **구조적 배경:** 수치가 남은 회차에서 입력이 파이 관측기에 들어올 때까지의 나이는 0.097~0.125초였다. 반면 파이의 면 검출 처리는 0.312~0.660초였다. 처리만으로도 0.5초 유효 기준의 상당 부분을 쓰고(일부 회차는 초과) 있어, 전송 지연이 조금만 늘어도 기준을 넘기 쉬운 구조였다. 관람 인원이 없던 10-30~11-02 회차에도 깊이 시각 대기가 회차당 1~4회 있었던 것과 맞는 설명이다.
- **종료 사유와의 관계:** 이벤트 순서상 입력 대기·테이프 시야 이탈 → 측면 재정렬 → 재접근 중 대기 → 정렬 공간 부족 종료로 이어진다. 입력이 늦어 좌우 보정이 늦어졌다는 흐름과 맞지만, 이 인과는 추론이며 확정하지 않는다.

이 문서는 기록과 현행 소스의 대조 결과다. 깊이 지연 해결 완료나 전체 시나리오 성공을 뜻하지 않는다. 2026-11-23 갱신은 2026-11-04 주행 PC 백업 사본의 로그로 재집계한 것이며, 실행 코드·센서 유효 기준을 바꾸거나 로봇을 움직이지 않았다.

## 대상 회차와 근거

| 항목 | 값 |
| --- | --- |
| 주문 | `261103002` (발표 시연 회차, 사용자 확인 2026-11-23) |
| PC 미션 | `20261103_153829_d723e714` |
| 급수대 실행 | `water_station_20261103_153855_e23ce186` |
| 당시 로컬 코드 HEAD | `f5d21a1` |
| PC 실행 로그 | `$HOME/.local/state/bimanual-robot/water-service/runs/20261103_153829_d723e714/controller.log` |
| Pi 상세 이벤트 원본 | `$HOME/jdamr_data/maps/20261026_manual_160135/runtime/runs/water_station_20261103_153855_e23ce186/cycle_events.jsonl` |
| PC 로그 SHA256 | `db2ef13eb484af90e60cfb9ab14068a3266833ae946f19c83371847e9a8b4df6` |
| 무선망 | 노트북은 공용 캠퍼스 Wi-Fi `aicampus_286_6G`([로봇 구동 런북](../../../../docs/guides/20261031_로봇구동_런북.md) 109행), Pi와 노트북은 같은 Wi-Fi로 연결([주행–조작 연동 구현 명세](../../../../docs/guides/20261029_주행_조작_연동_구현명세.md) 361행) |

집계는 PC `controller.log`에서 추출한 JSON 이벤트 385건을 대상으로 했다. 원본 로그·영상은 저장소에 넣지 않는다. 이 회차의 Pi 상세 이벤트는 백업 사본에 없고(실행 폴더에 `command.txt`만 보존), PC 로그에는 이벤트 이름만 있고 지연 수치가 없다. 조사 시점(11-03)의 Pi SSH `No route to host`는 조사 시점의 상태이며 주행 중 Wi-Fi 장애의 증거로 쓰지 않는다.

## 확인한 현상

| PC 로그 이벤트 | 건수 | 해석 범위 |
| --- | ---: | --- |
| `table_approach_waiting_for_input` / `table approach depth stale or future-dated` | 5 | 깊이 기반 관측의 시각 유효성 때문에 대기에 들어간 기록 |
| `table_approach_waiting_for_input` / `table approach tape reference pending` | 4 | 테이프 목표 확보·확정 대기. 깊이 지연과 구분 |
| `table_approach_latency_breakdown` | 24 | 관측 시각 판정의 진단 기록. 서로 다른 정지 24회를 뜻하지 않음 |
| `table_control_validation_slow` | 29 | 제어 경로 검증이 50ms 이상 걸린 기록 |
| `table_approach_late_frame_skipped` | 9 | 늦게 끝난 관측 결과를 건너뛴 기록. 네트워크 패킷 손실 건수가 아님 |
| `table_approach_input_recovered` | 7 | 입력 대기에서 복구한 기록. 특정 원인과 일대일 대응시키지 않음 |
| `table_lateral_restage` | 1 | 목표 재기준 뒤 측면 재정렬 |
| `failed` | 1 | `final_approach`: `table lateral alignment requires more clearance` |

PC 로그에는 15:40:49에 위 좌우 정렬 사유로 실행 종료가 보고됐다. 같은 로컬 로그 배치에 찍힌 출력 시각을 개별 이벤트 발생 시각으로 간주해 대기 시간을 계산하지 않는다.

## 이벤트 순서

PC 로그의 순서만 옮긴다. 배치 출력 시각이라 간격은 계산하지 않는다.

1. 15:39:38 테이블 시야 접근 시작 → 최종 접근 진입. 테이프 기준 대기·늦은 결과 건너뛰기·입력 회복이 이어짐.
2. 15:40:05 깊이 시각 대기·회복 뒤 `table_unconfirmed_tape_correction_discarded`(`tape_pair_left_view`). 테이프 쌍이 시야에서 빠져 확정되지 않은 보정을 버림.
3. 15:40:19 `rebased_station_goal_requires_restage`로 측면 재정렬, 15:40:33 재정렬 확인 뒤 재접근.
4. 재접근 중 깊이 시각 대기 3회·테이프 기준 대기 2회와 회복이 반복됨.
5. 15:40:46 `final_approach_short` 뒤 `table lateral alignment requires more clearance`로 종료.

## 같은 코드·같은 장소 회차 비교

13:37 미션과 15:38 미션 사이 변경은 라이다 차체 반사 필터의 기둥 경계 두 건(`self_filter_columns_20261012.yaml`, 13:54·14:04 배포)뿐이다([PROGRESS](../../../../PROGRESS.md) 2026-11-03 항목). 깊이 수신·면 검출·주차 제어 코드는 같다. 장소는 같은 발표 공간이며, 13:37은 관람 인원이 모이기 전, 15:38은 관람 중이었다(사용자 확인).

| 회차 | 근거 | 깊이 시각 대기 | 테이프 기준 대기 | 늦은 결과 건너뛰기 | 50ms 이상 제어 경로 검증 | 결과 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 13:37 미션 `20261103_133754_4a03ff0c` (급수대 13:41) | PC `controller.log` | 0 | 1 | 1 | 1 | 급수대 주차·물따르기·컵배치 성공 |
| 13:57 급수대 `water_station_20261103_135711_dbd6588c` | Pi 이벤트 사본 | 1 | 2 | 4 | — | 측면 재정렬 1회 뒤 주차·도크 복귀 |
| 15:38 미션 `20261103_153829_d723e714` (발표 시연) | PC `controller.log` | 5 | 4 | 9 | 29 | 정렬 공간 부족 종료 |

13:57 회차는 PC 미션이 아닌 단독 실행이라 Pi 이벤트 사본(`$HOME/jdamr_data/analysis_20261103/full_serving_133754/`)에서 같은 이벤트 이름을 셌다. 제어 경로 검증 지연 이벤트는 PC 로그에만 남아 비교하지 않는다.

## 깊이 입력과 팔 추론의 경로 차이

| 기능 | 입력·출력 연결 | Wi-Fi 경유 | 발표 시연 시점 결과 |
| --- | --- | --- | --- |
| 양팔 추론(SmolVLA) | 차체 위 Windows 노트북에 팔 2개(USB 직렬), 손목 카메라 2개(USB), Astra 상단 카메라(USB). 노트북 안에서 추론 | 시작·종료 HTTP 요청(`/run/pour` 등)만 경유 | 직후 팔 전용 시연 주문 `261103003` 물따르기 68.83초·컵배치 26.02초, 두 정책 `auto_done`, 주문 SUCCEEDED |
| 주행 정밀주차 깊이 입력 | 같은 노트북의 Astra RGB-D를 AST1로 Pi에 연속 송신(`--send-pi 192.168.0.159:5600`) | 연속 영상 스트림 전체가 경유 | 깊이 시각 대기 5회·테이프 기준 대기 4회 |

근거: [로봇 노트북 IL 추론 환경](../../../../docs/guides/20261029_로봇노트북_IL_추론환경_구축.md) 39~41·86행, [로봇 구동 런북](../../../../docs/guides/20261031_로봇구동_런북.md) 109~115·183행, [PROGRESS](../../../../PROGRESS.md) 2026-11-03 깊이 입력 지연 항목. 같은 Astra 카메라가 노트북 안에서는 팔 추론에 정상 입력으로 쓰였으므로, 촬영 장치 자체보다 송신 이후 경로(전송·Pi 처리)를 먼저 의심할 근거가 된다. 팔 전용 시연은 베이스가 움직이지 않아 주차 깊이 경로를 쓰지 않았다.

## 이전 회차의 깊이 시각 대기

10-29~11-03 PC `controller.log` 41개를 같은 방식으로 셌다. 깊이 시각 대기가 1회 이상인 회차와 비교 기준 13:37 미션만 적는다. 나머지 회차는 0회다.

| PC 미션 | 깊이 시각 대기 | 테이프 기준 대기 | 늦은 결과 건너뛰기 | 50ms 이상 제어 경로 검증 |
| --- | ---: | ---: | ---: | ---: |
| `20261030_150258_6c31b57d` | 3 | 0 | 7 | 0 |
| `20261030_155306_4fd4dd95` | 4 | 0 | 11 | 0 |
| `20261101_180623_da335e69` | 2 | 0 | 1 | 2 |
| `20261101_184225_665161ac` | 1 | 0 | 2 | 14 |
| `20261102_182308_48631f33` | 4 | 0 | 10 | 14 |
| `20261102_191252_65d8accb` | 4 | 1 | 6 | 25 |
| `20261102_192956_dac49049` | 3 | 0 | 7 | 20 |
| `20261102_194713_193dc6a6` | 4 | 1 | 9 | 36 |
| `20261102_213031_7ec652f5` | 1 | 0 | 4 | 3 |
| `20261102_215003_e374564a` | 2 | 0 | 6 | 8 |
| `20261102_220922_7f64b066` | 1 | 0 | 2 | 30 |
| `20261103_133754_4a03ff0c` | 0 | 1 | 1 | 1 |
| `20261103_153829_d723e714` | 5 | 4 | 9 | 29 |

관람 인원이 없던 회차에도 대기가 있었으므로 무선 혼잡만으로 모든 대기를 설명하지 않는다. 회차마다 접근 횟수·경로 길이가 달라 건수를 그대로 비율로 비교하지 않는다.

## 구간별 지연 수치가 남은 회차

`table_approach_latency_breakdown`은 관측 나이가 0.5초를 넘은 시점에 구간별 값을 남긴다([`box_service.py`](../jdamr_cube_navigation/box_service.py) 4136~4146행). `observer_input_age_s`는 촬영 시각부터 관측기가 깊이 영상을 복호화한 시점까지의 나이로, Windows 송신·Wi-Fi 전송·Pi 수신과 ROS 전달이 모두 들어간다. `observer_processing_s`는 관측기 콜백 시작부터 결과 발행까지의 면 검출 처리 시간이다([`depth_box_parking.py`](../jdamr_cube_navigation/depth_box_parking.py) 427~440·638~642행).

| 회차(Pi 이벤트 사본) | 진단 건수 | 입력 나이 | 면 검출 처리 | 관측 나이 |
| --- | ---: | --- | --- | --- |
| `water_station_20261102_170812_fabd2f83` | 5 | 0.121초 | 0.312초 | 0.523~0.777초 |
| 미션 `20261102_180934_a6f19a29` 급수대 | 6 | 0.097~0.125초 | 0.507~0.660초 | 0.510~1.574초 |
| `water_station_20261103_135711_dbd6588c` (발표 당일 오후 앞 회차) | 6 | 0.118초 | 0.391초 | 0.537~0.821초 |

수치가 남은 범위에서는 처리 시간이 입력 나이보다 컸고, 처리만으로도 0.5초 기준의 상당 부분을 썼다. 15:38 회차의 같은 값은 없어 발표 시연 중 입력 나이가 얼마나 늘었는지는 알 수 없다. 입력 나이에는 AST1 시계 보정 오차도 들어갈 수 있다. 사본 위치는 `$HOME/jdamr_data/analysis_20261102/`와 `$HOME/jdamr_data/analysis_20261103/full_serving_133754/`다.

## ‘깊이 지연’이 의미하는 것

입력 경로는 다음과 같다.

```text
Windows Astra 촬영
  → RGB·깊이 압축 및 AST1 송신
  → Wi-Fi 전송
  → Pi 수신·시계 보정·복호화·ROS 발행
  → 책상 면 검출
  → 주차 제어기의 관측 사용·경로 검증
```

[`box_service.py`](../jdamr_cube_navigation/box_service.py)의 `_table_fine_approach`는 관측의 촬영 시각과 제어 시각을 비교한다. 기본 유효 범위는 `0 ≤ 관측 나이 ≤ 0.5초`다. 최신 깊이 입력과 유지 중인 면이 유효하면 이전 승인 관측을 이어 쓰거나 늦은 결과를 건너뛰는 예외가 있으므로, 0.5초 초과가 곧바로 모든 이동 종료를 뜻하지는 않는다.

`depth stale or future-dated`는 오래된 시각뿐 아니라 미래·무효 시각도 함께 나타내는 메시지다. PC 요약 로그에는 실제 `observation_age_s` 값이 없어, 5건 모두가 양의 0.5초 초과였다고 단정할 수 없다.

또한 이 판정에는 촬영·전송·검출·제어 단계의 누적 지연이 들어간다. 카메라 고장, Wi-Fi 지연, Pi 계산 지연 중 어느 하나를 가리키는 전용 오류가 아니다.

## 원인 후보와 확인 수준

| 후보 | 확보한 근거 | 확인 수준 | 아직 모르는 것 |
| --- | --- | --- | --- |
| 현장 무선망 혼잡·재전송 | 같은 코드·같은 장소의 관람 전 회차 대기 0회 대비 발표 시연 5회, Wi-Fi를 거치지 않는 팔 추론은 정상, 20~30명 공용 Wi-Fi 사용과 다른 팀 무선 문제(사용자 증언) | 가장 유력(정황). 측정 확정 아님 | 해당 회차의 RTT·재전송·채널 점유율·촬영→수신 지연 |
| Pi 면 검출·제어 처리 지연 | 수치가 남은 회차의 면 검출 처리 0.31~0.66초, 발표 시연 회차의 50ms 이상 제어 경로 검증 29건 | 상시 여유 부족은 기록으로 확인. 발표 시연 중 증가 폭은 미확인 | 발표 시연 중 검출·콜백·제어 계산 각각의 비중과 CPU 부하 |
| 시계 보정·타임스탬프 문제 | 오류 문구에 미래 시각도 포함됨 | 증거 없음 | 해당 시점의 관측 나이 부호, PONG RTT·시계 오프셋 변화 |
| 좌우 목표·접근 경로의 불일치 | 테이프 시야 이탈, 재정렬 1회, 최종 공간 부족 오류 | 입력 대기와 순서상 이어짐(추론) | 실제 테이프 좌우 오차와 목표 변화 원인 |

Wi-Fi는 같은 채널의 송수신이 많아지면 전송 대기와 지연이 늘 수 있다. 신호 세기가 좋아도 혼잡이 생길 수 있다는 일반적인 설명은 [Cisco의 무선 성능 진단 문서](https://www.cisco.com/c/en/us/support/docs/wireless/catalyst-9800-series-wireless-controllers/221766-validate-wi-fi-throughput-testing-and.html)와 [Ubiquiti의 airtime 혼잡 설명](https://help.ui.com/hc/en-us/articles/30266615758743-Resolving-Wi-Fi-Performance-Issues-Due-to-High-Airtime-Utilization)을 참고한다. 이 자료는 해당 회차의 원인 증명이 아니다.

## 다음 조치

1. **차체 내부 통신을 공용망에서 분리한다.** 노트북과 Pi는 같은 차체에 실려 있으므로 깊이 스트림은 유선 이더넷이나 로봇 전용 무선망으로 보낸다. 공용 Wi-Fi에는 주문·관제 같은 짧은 요청만 남긴다.
2. **회차 기록에 수신기 구간 지연을 함께 남긴다.** 아래 기존 필드를 실행 이벤트에 주기적으로 기록해, 다음 장애 때 전송과 처리를 같은 회차 안에서 나눈다.
3. **면 검출 처리 시간을 줄여 0.5초 기준의 여유를 만든다.** 처리만으로 0.31~0.66초를 쓰는 현재 구조에서는 작은 전송 지연에도 기준을 넘는다.

## 다음에 원인을 분리할 때 볼 값

이미 구현된 진단 필드를 사용한다. 새 출발 차단 절차나 긴 검사를 추가하는 계획은 아니다.

| 관측 지점 | 기존 필드 | 구분할 내용 |
| --- | --- | --- |
| AST1 수신기 상태 | `receive_fps`, `accepted_fps`, `depth_capture_to_send_ms`, `depth_capture_to_receive_ms`, `decode_ms` | Windows 촬영·송신 대기, 전송·수신, 복호화 지연 |
| AST1 시계 보정 | `last_clock_rtt_ns`, `clock_rtt_ns`, `clock_offset_ns`, `clock_uncertainty_ns`, `clock_rejected` | RTT·보정 오차 또는 유효 동기화 표본 부족 |
| 주차 관측 진단 | `observation_age_s`, `observer_input_age_s`, `observer_processing_s`, `observer_calculation_s` | 촬영 시각부터의 나이와 관측 처리 구간 |
| 제어기 진단 | `status_callback_age_s`, `previous_control_validation_s`, `fresh_depth_available` | 콜백 전달 지연, 제어 계산 지연, 새 깊이의 도착 여부 |

수신기 필드는 [`astra_tcp_receiver.py`](../jdamr_cube_navigation/astra_tcp_receiver.py)의 `Receiver.current_status()`에, 주차 진단 필드는 [`box_service.py`](../jdamr_cube_navigation/box_service.py)에 있다. 수신기 통계는 최근 30초 창이므로 다음 회차의 현재 값으로 이번 과거 회차를 소급 설명하지 않는다.

같은 시각의 촬영→수신 지연이 증가하면 전송 경로를, 수신은 정상인데 관측·계산 시간이 증가하면 Pi 처리 경로를 먼저 조사한다. 촬영→수신 시간에도 Windows 송신 전 대기와 시계 보정 오차가 들어갈 수 있으므로 그 값 하나만으로 Wi-Fi를 확정하지 않는다.

## 집계 재현

주행이나 서비스 조작 없이 기존 PC 로그와 Pi 이벤트 사본만 읽는다.

단일 회차 집계:

```bash
python3 - <<'PY'
import collections
import json
from pathlib import Path

path = Path.home() / (
    '.local/state/bimanual-robot/water-service/runs/'
    '20261103_153829_d723e714/controller.log'
)
counts = collections.Counter()
for line in path.read_text().splitlines():
    start = line.find('{')
    if start < 0:
        continue
    try:
        event = json.loads(line[start:])
    except json.JSONDecodeError:
        continue
    if isinstance(event, dict) and 'event' in event:
        counts[event['event'], event.get('reason')] += 1
for (event, reason), count in sorted(counts.items(), key=lambda item: str(item[0])):
    if any(word in event for word in ('latency', 'waiting', 'recovered', 'slow', 'late_frame', 'restage', 'failed')):
        print(event, reason, count)
PY
```

회차별 비교와 구간별 지연:

```bash
python3 - <<'PY'
import collections
import json
from pathlib import Path

runs = Path.home() / '.local/state/bimanual-robot/water-service/runs'
columns = {
    'depth_stale': ('table_approach_waiting_for_input', 'table approach depth stale or future-dated'),
    'tape_pending': ('table_approach_waiting_for_input', 'table approach tape reference pending'),
    'late_skip': ('table_approach_late_frame_skipped', None),
    'slow_validation': ('table_control_validation_slow', None),
}
for log in sorted(runs.glob('*/controller.log')):
    counts = collections.Counter()
    for line in log.read_text(errors='replace').splitlines():
        start = line.find('{')
        if start < 0:
            continue
        try:
            event = json.loads(line[start:])
        except json.JSONDecodeError:
            continue
        if not isinstance(event, dict):
            continue
        for name, (kind, reason) in columns.items():
            if event.get('event') == kind and (reason is None or event.get('reason') == reason):
                counts[name] += 1
    print(log.parent.name, *(counts[name] for name in columns))

fields = ('observation_age_s', 'observer_input_age_s', 'observer_processing_s')
for path in sorted((Path.home() / 'jdamr_data').glob('analysis_2026110[23]/**/*.jsonl')):
    values = collections.defaultdict(list)
    for line in path.read_text(errors='replace').splitlines():
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        for field in fields:
            if isinstance(event.get(field), (int, float)):
                values[field].append(event[field])
    if values:
        print(path, {field: (min(v), max(v), len(v)) for field, v in values.items()})
PY
```

## 완료한 일과 남은 문제

- 완료: PC 로그 재집계, 현행 소스의 판정과 대조, 같은 코드·같은 장소 회차 비교, 깊이 입력과 팔 추론의 경로 비교, 이전 회차 대기 집계, 구간별 지연이 남은 회차의 수치 정리.
- 판단: 현장 무선 혼잡이 가장 유력한 원인(정황 근거). 면 검출 처리가 0.5초 기준의 여유를 줄여 온 구조적 요인이 함께 있다.
- 미해결: 발표 시연 회차의 전송·검출·제어 지연 비중, 마지막 좌우 정렬 실패의 근본 원인, 차체 내부 통신 분리 뒤 정지 없는 재주행의 실차 확인.
- 이 기록을 측정으로 확정한 Wi-Fi 원인, Pi CPU 부족 확정, 깊이 지연 수정 완료로 해석하지 않는다.
