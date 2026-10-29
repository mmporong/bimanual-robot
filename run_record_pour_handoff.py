"""물따르기(handoff) 1개 에피소드 녹화 - 카메라 인덱스를 실행 직전에 자동으로 다시 찾아서 채운다.

2라운드부터는 --resume=true 를 추가해서 실행.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "tools" / "camera"))
from launch_with_resolved_cameras import run_lerobot

EXE = r"C:\Users\USER\miniconda3\envs\so101_env\Scripts\lerobot-record.exe"

ARGS = [
    "--robot.type=bi_so_follower",
    "--robot.id=arms",
    r"--robot.calibration_dir=C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower",
    "--robot.left_arm_config.port=COM15",
    "--robot.left_arm_config.gripper_protection=true",
    "--robot.left_arm_config.disable_torque_on_disconnect=false",
    "--robot.left_arm_config.cameras={{ wrist: {{type: opencv, index_or_path: {left}, width: 640, height: 480, fps: 30}} }}",
    "--robot.right_arm_config.port=COM16",
    "--robot.right_arm_config.disable_torque_on_disconnect=false",
    "--robot.right_arm_config.cameras={{ wrist: {{type: opencv, index_or_path: {right}, width: 640, height: 480, fps: 30}} }}",
    "--robot.cameras={{ top: {{type: opencv, index_or_path: {depth}, width: 640, height: 480, fps: 30, mirror: true}} }}",
    "--teleop.type=bi_so_leader",
    "--teleop.id=arms",
    r"--teleop.calibration_dir=C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_leader",
    "--teleop.left_arm_config.port=COM5",
    "--teleop.right_arm_config.port=COM14",
    "--dataset.repo_id=jun-seo/water-pour-position-jitter-gravel-tabletop-handoff-v1",
    r"--dataset.root=C:\Users\USER\lerobot-data\water-pour-position-jitter-gravel-tabletop-handoff-v1",
    "--dataset.no_stamp=true",
    "--dataset.single_task=Pick up the cup with the left arm, pick up the water bottle with the right arm, "
    "pour water from the bottle into the cup, set the bottle down to the right of where it was picked up, "
    "move the cup onto the robot chassis with the left arm, set it down, and hold it gently but securely.",
    "--dataset.num_episodes=1",
    "--dataset.episode_time_s=100",
    "--dataset.reset_time_s=5",
    "--dataset.fps=30",
    "--dataset.streaming_encoding=false",
    "--dataset.push_to_hub=true",
    "--dataset.private=true",
    "--display_data=true",
] + sys.argv[1:]

run_lerobot(EXE, ARGS)
