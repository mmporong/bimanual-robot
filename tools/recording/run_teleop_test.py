"""양팔 텔레옵 - 카메라 인덱스를 실행 직전에 자동으로 다시 찾아서 채운다."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera"))
from launch_with_resolved_cameras import run_lerobot

EXE = r"C:\Users\USER\miniconda3\envs\so101_env\Scripts\lerobot-teleoperate.exe"

ARGS = [
    "--robot.type=bi_so_follower",
    "--robot.id=arms",
    r"--robot.calibration_dir=C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower",
    "--robot.left_arm_config.port=COM15",
    "--robot.left_arm_config.gripper_protection=true",
    "--robot.left_arm_config.cameras={{ wrist: {{type: opencv, index_or_path: {left}, width: 640, height: 480, fps: 30}} }}",
    "--robot.right_arm_config.port=COM16",
    "--robot.right_arm_config.cameras={{ wrist: {{type: opencv, index_or_path: {right}, width: 640, height: 480, fps: 30}} }}",
    "--robot.cameras={{ top: {{type: opencv, index_or_path: {depth}, width: 640, height: 480, fps: 30, mirror: true}} }}",
    "--teleop.type=bi_so_leader",
    "--teleop.id=arms",
    r"--teleop.calibration_dir=C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_leader",
    "--teleop.left_arm_config.port=COM5",
    "--teleop.right_arm_config.port=COM14",
    "--display_data=true",
] + sys.argv[1:]

run_lerobot(EXE, ARGS)
