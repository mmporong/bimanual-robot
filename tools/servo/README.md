# STS3215 서보 도구

그리퍼 캘리브레이션과 서보 상태 점검에 쓰는 최소 도구다. 절차와 실측값은
[`docs/20260909_그리퍼_캘리브레이션_인계.md`](../../docs/20260909_그리퍼_캘리브레이션_인계.md)에 있다.

## 처음 받는 사람 — 환경 준비

**1) 저장소를 받는다.**

```bash
git clone git@github.com:mmporong/bimanual-robot.git
cd bimanual-robot
```

**2) pyserial 을 확인한다.** 외부 의존성은 이것 하나뿐이다.

```bash
python3 -c "import serial; print(serial.__version__)"
```

없으면 설치한다. Ubuntu 는 apt 쪽이 낫다.

```bash
sudo apt install python3-serial
```

**3) 시리얼 포트 권한을 확인한다.**

```bash
id -nG | tr ' ' '\n' | grep dialout
```

안 나오면 그룹에 넣고 **로그아웃 후 다시 로그인**한다. 재로그인 전에는 적용되지 않는다.

```bash
sudo usermod -aG dialout $USER
```

**4) 보드를 연결한다.** 12V 어댑터를 먼저 꽂고 USB 를 연결한다.

```bash
ls -l /dev/serial/by-id/
```

`usb-1a86_USB_Single_Serial_...-if00` 이 보이면 준비가 끝났다.

**5) 읽어본다.** 여기까지 오면 설치가 맞은 것이다.

```bash
cd ~/bimanual-robot/tools/servo
python3 servo_read.py
```

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

## 그리퍼 캘리브레이션 한 판 (실제 순서)

죠를 다시 물린 뒤 이 다섯 줄이면 끝난다. 절차의 근거는
[인계 문서 3절](../../docs/20260909_그리퍼_캘리브레이션_인계.md)에 있다.

```bash
cd ~/bimanual-robot/tools/servo

# 1. 죠를 손으로 완전히 맞닿게 한 뒤
python3 servo_read.py --id 6                          # 구동이 0 인지 확인
python3 servo_offset.py --target 3600 --execute       # 그 자세를 3600 으로

# 2. 죠를 손으로 완전히 벌린 뒤
python3 servo_read.py --id 6                          # 열림 끝 값을 읽는다 (예: 732)
python3 servo_limits.py --min 760 --max 3600 --execute

# 3. 검증 — 20 mm 를 명령하고 자로 잰다
python3 servo_goto.py --goal 3000 --execute
```

`--execute` 를 빼고 먼저 돌리면 무엇을 바꿀지 보여주고 끝난다.

## 문제 해결

| 증상 | 먼저 볼 것 |
|---|---|
| `시리얼 포트가 없습니다` | USB 케이블이 충전 전용은 아닌지. `lsusb` 에 `1a86` 이 뜨는지 |
| 포트는 있는데 `응답한 서보가 없습니다` | **12V 어댑터**. USB 만으로는 서보가 안 산다. 그다음 보드→첫 서보 3핀 선 |
| 특정 ID 하나만 무응답 | 데이지체인에서 그 단의 커넥터. **전원을 끄고** 다시 꽂는다 |
| `Permission denied` | `dialout` 그룹. 추가했으면 재로그인했는지 |
| `0.5s 동안 정지 (목표까지 N 남음)` | 하드 스톱이거나 걸림. 구동을 풀고 손으로 그 방향으로 되는지 본다 |
| `부하 N > 600` | 진짜 걸림. 피니언 허브와 레일의 서포트·브림 잔재를 본다 |
| `적용 실패 — 값이 바뀌지 않았습니다` | EEPROM 잠금. 구동이 걸려 있으면 안 써진다. `servo_release.py` 먼저 |
| `예상한 100 만큼 안 움직였습니다` | 오프셋 레지스터가 다른 계열일 수 있다. 자동으로 원복됐으니 그대로 두고 문의 |
| `목표가 각도 한계 밖입니다` | `servo_read.py` 로 한계를 보고, 의도한 것이면 `servo_limits.py` 로 먼저 넓힌다 |

## 되돌리기

`servo_offset.py` 와 `servo_limits.py` 는 바꾸기 전 값을
`~/.cache/bimanual-robot/servo_backup.jsonl` 에 한 줄씩 append 한다. 덮어쓰지 않으므로
이력이 남는다.

```bash
cat ~/.cache/bimanual-robot/servo_backup.jsonl
```
