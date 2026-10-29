@echo off
chcp 65001 >nul
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8

"C:\Users\USER\miniconda3\envs\so101_env\Scripts\lerobot-rollout.exe" ^
  --strategy.type=base ^
  --policy.path="C:\Users\USER\lerobot-training\outputs\cup_place_gravel_tabletop_smolvla_v1\checkpoints\009500\pretrained_model" ^
  --policy.device=cuda ^
  --robot.type=bi_so_follower ^
  --robot.id=arms ^
  --robot.calibration_dir="C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower" ^
  --robot.left_arm_config.port=COM15 ^
  --robot.left_arm_config.gripper_protection=true ^
  --robot.left_arm_config.disable_torque_on_disconnect=false ^
  --robot.left_arm_config.max_relative_target=15 ^
  --robot.left_arm_config.cameras="{ wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30} }" ^
  --robot.right_arm_config.port=COM16 ^
  --robot.right_arm_config.disable_torque_on_disconnect=false ^
  --robot.right_arm_config.max_relative_target=15 ^
  --robot.right_arm_config.cameras="{ wrist: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30} }" ^
  --robot.cameras="{ top: {type: opencv, index_or_path: 3, width: 640, height: 480, fps: 30, mirror: true} }" ^
  --task="Pick up the cup loaded on the robot chassis with the left arm and place it on the table." ^
  --duration=40 ^
  --display_data=true
