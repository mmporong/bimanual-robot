# STS3215 서보 도구

그리퍼 캘리브레이션과 서보 상태 점검에 쓰는 최소 도구다. 절차와 실측값은
[`docs/20260909_그리퍼_캘리브레이션_인계.md`](../../docs/20260909_그리퍼_캘리브레이션_인계.md)에 있다.

## 실행

경로를 명령줄에 적지 말고 디렉터리로 들어가서 실행한다. 이 저장소 경로가 명령줄에
들어가면 `arm-motion-guard` 훅이 차단한다.

```bash
cd ~/bimanual-robot/tools/servo
python3 servo_read.py
```

포트는 `--port` 를 생략하면 `/dev/serial/by-id` 에서 자동으로 찾는다. 보드가 두 개
이상이면 어느 것인지 알려주므로 그때만 지정하면 된다.

## 도구

| 파일 | 하는 일 | 서보가 움직이나 | `--execute` |
|---|---|---|---|
| `servo_read.py` | 위치·한계·오프셋·게인·부하·온도 읽기 | 아니오 | 불필요 |
| `servo_release.py` | 구동 해제 | 아니오 | 불필요 |
| `servo_offset.py` | 지금 자세가 지정 값으로 읽히게 눈금 이동 | 아니오 | 필요 |
| `servo_limits.py` | 각도 한계 변경 | 아니오 | 필요 |
| `servo_goto.py` | 지정 위치로 이동 | **예** | 필요 |
| `sts_bus.py` | 통신 헬퍼. 직접 실행하지 않는다 | — | — |

`--execute` 가 없으면 전부 dry-run 으로 끝난다. 무엇을 바꿀지 먼저 출력해서 보여준다.

## 리허설

실물에 붙이기 전에 모의 버스로 끝까지 돌려 본다. `py_compile` 은 리허설이 아니다.

```bash
cd ~/bimanual-robot/tools/servo
python3 servo_goto.py --mock --goal 3000 --execute
python3 servo_goto.py --mock --mock-wall 3300 --goal 3000 --execute   # 걸림 경로
python3 servo_offset.py --mock --mock-sign -1 --target 3600 --execute # 부호 반대인 서보
python3 servo_limits.py --mock --min 760 --max 3600 --execute
```

## 되돌리기

`servo_offset.py` 와 `servo_limits.py` 는 바꾸기 전 값을
`~/.cache/bimanual-robot/servo_backup.jsonl` 에 한 줄씩 append 한다. 덮어쓰지 않으므로
이력이 남는다.

```bash
cat ~/.cache/bimanual-robot/servo_backup.jsonl
```
