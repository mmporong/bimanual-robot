"""리더 팔을 현재 자세 그대로 토크로 고정한다 (물따르기 녹화 종료 직후 실행).

리더는 평소 토크 없이(수동, backdrivable) 사람이 손으로 움직이는 장치라,
녹화 사이 업로드 대기 시간 동안 사람이 계속 손으로 붙잡고 있어야 했다.
이 스크립트는 리더가 "지금 있는 그 자리"에서 토크를 걸어 스스로 자세를
유지하게 만든다 - 실행 후 손을 놔도 된다.

중요: 물따르기 녹화 프로세스가 막 끝난 직후, 팔로워를 마지막으로 구동하던
바로 그 리더 위치에서 바로 실행해야 한다 - 그래야 리더=팔로워 정렬이
그대로 유지된다. 리더를 먼저 다른 데로 옮기고 나서 실행하면 그 어긋난
위치로 고정돼버린다.

다음 컵배치 녹화(lerobot-record)를 시작하면, 그 프로세스의 정상적인
connect() 과정(SOLeader.configure() -> bus.disable_torque())에서 리더
토크가 자동으로 풀리고 평소처럼 텔레옵 가능한 상태로 돌아간다 - 이
스크립트가 따로 "풀기" 동작을 할 필요는 없다.

안전: 이동 명령을 전혀 보내지 않는다 - 현재 위치에서 토크만 건다.
      이 스크립트 프로세스는 disconnect()를 호출하지 않고 그냥 종료한다
      (SOLeader.disconnect()는 기본적으로 토크를 꺼버리므로 일부러 안 부름).
      터미널을 닫아도 무방하다 - 토크는 서보 자체가 유지한다.

사용법: python tools/recording/hold_leader_pose.py
"""
from pathlib import Path

from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig
from lerobot.teleoperators.so_leader import SOLeaderConfig

CALIB_DIR = Path(r"C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_leader")

cfg = BiSOLeaderConfig(
    id="arms",
    calibration_dir=CALIB_DIR,
    left_arm_config=SOLeaderConfig(port="COM5"),
    right_arm_config=SOLeaderConfig(port="COM14"),
)
leader = BiSOLeader(cfg)
leader.connect(calibrate=False)
print("connected. 현재 자세에서 토크를 겁니다 - 움직이지 않습니다.")

leader.enable_torque()

print("\n*** 리더 토크 ON. 지금 자세로 고정됨 - 손을 놔도 됩니다. ***")
print("*** 다음 컵배치 lerobot-record를 시작하면 자동으로 다시 풀립니다. ***")
