@echo off
REM Scenario start (run once when the robot leaves the dock):
REM   1) manipulation service: loads pour + place policies, connects arms and cameras ONCE, warms up, waits
REM   2) order server with the rollout-service backend (guest /order, operator /control)
REM   3) opens the operator control page
REM Arm work starts only after the "alignment done" button on /control (or a navigation signal).
REM Driving pose: --stow.pose=place_end. When an order starts (vehicle still at the dock) both arms fold into it;
REM after the pour the right arm folds while the left keeps the cup; the place run ends in it.
REM Policies and per-task settings match the single-run rollout_handoff.py commands (max_relative_target=15,
REM pour: start pose pour / end final / auto pour / 100 s, place: end place_end / auto place / 50 s).
REM Guide: docs\guides\20261026_상주형_조작실행기_사용법.md
REM Edit the paths/ports below for this laptop.

set PY=C:\Users\USER\miniconda3\envs\so101_env\python.exe
set POUR=C:\Users\USER\lerobot-training\outputs\water_pour_chassis_smolvla_80\checkpoints\016000\pretrained_model
set PLACE=C:\Users\USER\lerobot-training\outputs\cup_place_chassis_smolvla_80\checkpoints\007000\pretrained_model
set LEFT_PORT=COM15
set RIGHT_PORT=COM16

cd /d "%~dp0..\.."

start "manip-service" "%PY%" tools\rollout\rollout_service.py --pour.policy="%POUR%" --place.policy="%PLACE%" --stow.pose=place_end --policy.device=cuda --robot.type=bi_so_follower --robot.id=arms --robot.calibration_dir=calibration\bi_follower --robot.left_arm_config.port=%LEFT_PORT% --robot.left_arm_config.gripper_protection=true --robot.left_arm_config.disable_torque_on_disconnect=false --robot.left_arm_config.max_relative_target=15 --robot.left_arm_config.cameras="{ wrist: {type: opencv, index_or_path: {left}, width: 640, height: 480, fps: 30} }" --robot.right_arm_config.port=%RIGHT_PORT% --robot.right_arm_config.disable_torque_on_disconnect=false --robot.right_arm_config.max_relative_target=15 --robot.right_arm_config.cameras="{ wrist: {type: opencv, index_or_path: {right}, width: 640, height: 480, fps: 30} }" --robot.cameras="{ top: {type: opencv, index_or_path: {depth}, width: 640, height: 480, fps: 30, mirror: true} }"

REM Live robot position on the /control map (needs SSH key login to the Pi and its navigation dashboard running):
REM   add  --robot-pose-url http://127.0.0.1:8090/api/state --robot-ssh lim@192.168.0.159
start "order-server" "%PY%" -m apps.service_order.service_order_server --host 0.0.0.0 --port 8765 --backend rollout-service --auto-step-s 0.5

timeout /t 3 /nobreak >nul
start "" http://127.0.0.1:8765/control
echo Started. Wait until the control page shows the manipulation service as ready before taking orders.
