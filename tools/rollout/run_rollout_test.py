"""물따르기 롤아웃 - 카메라 인덱스를 실행 직전에 자동으로 다시 찾아서 채운다."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera"))
from launch_with_resolved_cameras import run_lerobot

EXE = r"C:\Users\USER\miniconda3\envs\so101_env\Scripts\lerobot-rollout.exe"

ARGS = [
    "--strategy.type=base",
    r"--policy.path=C:\Users\USER\lerobot-training\outputs\water_pour_gravel_tabletop_smolvla_v1\checkpoints\008000\pretrained_model",
    "--policy.device=cuda",
    "--robot.type=bi_so_follower",
    "--robot.id=arms",
    r"--robot.calibration_dir=C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower",
    "--robot.left_arm_config.port=COM15",
    "--robot.left_arm_config.gripper_protection=true",
    "--robot.left_arm_config.disable_torque_on_disconnect=false",
    "--robot.left_arm_config.max_relative_target=15",
    "--robot.left_arm_config.cameras={{ wrist: {{type: opencv, index_or_path: {left}, width: 640, height: 480, fps: 30}} }}",
    "--robot.right_arm_config.port=COM16",
    "--robot.right_arm_config.disable_torque_on_disconnect=false",
    "--robot.right_arm_config.max_relative_target=15",
    "--robot.right_arm_config.cameras={{ wrist: {{type: opencv, index_or_path: {right}, width: 640, height: 480, fps: 30}} }}",
    "--robot.cameras={{ top: {{type: opencv, index_or_path: {depth}, width: 640, height: 480, fps: 30, mirror: true}} }}",
    "--task=Pick up the cup with the left arm, pick up the water bottle with the right arm, pour water from "
    "the bottle into the cup, return the bottle to its place, and load the cup onto the robot chassis.",
    "--duration=100",
    "--display_data=true",
] + sys.argv[1:]

run_lerobot(EXE, ARGS)
