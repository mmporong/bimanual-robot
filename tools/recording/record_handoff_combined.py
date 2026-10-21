"""물따르기 -> 컵배치 한 라운드를 연결을 끊지 않고 이어 녹화한다.

기존 방식(두 개의 lerobot-record를 따로 실행)은 물따르기 프로세스가 끝나고
컵배치 프로세스가 새로 연결을 맺는 구간에서 계속 문제가 났다(포트 재연결
실패, 그 사이 리더를 손으로 붙잡고 있어야 하는 번거로움, 업로드 대기).
이 스크립트는 명령어 한 번 실행 = 한 라운드다:

  1. 물따르기 에피소드 녹화 (-> 또는 n으로 종료) -> water-pour 데이터셋에 저장
  2. 끊김 없이 바로 이어서 컵배치 에피소드 녹화 -> cup-place 데이터셋에 저장
  3. 두 데이터셋 모두 로컬에 저장 + 각자의 HF 레포에 업로드

다음 라운드는 병·컵을 재배치한 뒤 이 스크립트를 다시 실행하면 된다 - 로컬
root가 이미 있으면 자동으로 resume되어 이어서 쌓인다. 재녹화(r)는 각
단계(물따르기/컵배치) 안에서 기존과 동일하게 동작한다.

카메라 인덱스는 매 실행 시 resolve_camera_indices.py로 새로 찾는다.

사용법:
    python tools/recording/record_handoff_combined.py
    (2회차 이후 재실행 시에도 로컬 root가 이미 있으면 자동으로 resume)

키 안내 (각 단계 공통):
    -> 또는 n : 이 단계 종료하고 저장, 다음 단계로
    r         : 이 단계 버리고 다시 녹화
    ESC 또는 q: 지금 단계를 중단 (저장 안 됨). 물따르기 중 중단 시 컵배치는
                건너뛴다. 어느 경우든 그때까지 저장된 에피소드는 로컬+HF로
                업로드된다.
"""

import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera"))
from resolve_camera_indices import resolve  # noqa: E402

from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: E402
from lerobot.common.control_utils import sanity_check_dataset_robot_compatibility  # noqa: E402
from lerobot.datasets import LeRobotDataset, VideoEncodingManager  # noqa: E402
from lerobot.processor import make_default_processors  # noqa: E402
from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig  # noqa: E402
from lerobot.robots.so_follower import SOFollowerConfig  # noqa: E402
from lerobot.scripts.lerobot_record import record_loop  # noqa: E402
from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig  # noqa: E402
from lerobot.teleoperators.so_leader import SOLeaderConfig  # noqa: E402
from lerobot.utils.keyboard_input import init_keyboard_listener  # noqa: E402
from lerobot.utils.utils import init_logging, log_say  # noqa: E402
from lerobot.utils.visualization_utils import init_visualization, shutdown_visualization  # noqa: E402

DISPLAY_MODE = "rerun"

REPO_ROOT = Path(__file__).resolve().parents[2]
CALIB_FOLLOWER = REPO_ROOT / "calibration" / "bi_follower"
CALIB_LEADER = REPO_ROOT / "calibration" / "bi_leader"

POUR_REPO_ID = "jun-seo/water-pour-position-jitter-gravel-tabletop-handoff-v1"
POUR_ROOT = Path(r"C:\Users\USER\lerobot-data\water-pour-position-jitter-gravel-tabletop-handoff-v1")
POUR_TASK = (
    "Pick up the cup with the left arm, pick up the water bottle with the right arm, "
    "pour water from the bottle into the cup, set the bottle down to the right of where "
    "it was picked up, move the cup onto the robot chassis with the left arm, and keep "
    "holding the cup."
)
POUR_EPISODE_TIME_S = 100

PLACE_REPO_ID = "jun-seo/cup-place-position-jitter-gravel-tabletop-handoff-v1"
PLACE_ROOT = Path(r"C:\Users\USER\lerobot-data\cup-place-position-jitter-gravel-tabletop-handoff-v1")
PLACE_TASK = (
    "While holding the cup already loaded on the robot chassis with the left arm, "
    "move it to the table and place it down."
)
PLACE_EPISODE_TIME_S = 50

FPS = 30


# lerobot-record CLI 기본값(RecordConfig.num_image_writer_threads_per_camera)과 동일.
# 이 값을 안 주면 AsyncImageWriter가 아예 안 켜지고(start_image_writer 미호출),
# 매 프레임 이미지 저장이 메인 제어 루프에서 동기로 실행돼 FPS가 떨어진다.
NUM_IMAGE_WRITER_THREADS_PER_CAMERA = 4


def make_dataset(repo_id: str, root: Path, single_task_text: str, robot, dataset_features):
    """로컬 root가 이미 있으면 resume, 없으면 새로 생성."""
    num_cameras = len(robot.cameras) if hasattr(robot, "cameras") else 0
    image_writer_threads = NUM_IMAGE_WRITER_THREADS_PER_CAMERA * num_cameras
    if (root / "meta" / "info.json").is_file():
        logging.info(f"[{repo_id}] 기존 로컬 데이터셋 발견 - resume")
        ds = LeRobotDataset.resume(
            repo_id,
            root=str(root),
            image_writer_processes=0,
            image_writer_threads=image_writer_threads,
        )
        # 원본 lerobot-record가 resume 시 하는 것과 동일한 안전장치: 기존 데이터셋의
        # robot_type/fps/features가 지금 로봇·녹화 설정과 어긋나면(카메라 키 누락,
        # 관절 수·shape 변경 등) 한 프레임도 찍기 전에 큰 소리로 막는다.
        sanity_check_dataset_robot_compatibility(ds, robot, FPS, dataset_features)
    else:
        logging.info(f"[{repo_id}] 로컬 데이터셋 없음 - 새로 생성")
        ds = LeRobotDataset.create(
            repo_id,
            FPS,
            root=str(root),
            robot_type=robot.name,
            features=dataset_features,
            use_videos=True,
            image_writer_processes=0,
            image_writer_threads=image_writer_threads,
        )
    return ds


def run_phase(robot, teleop, events, episode_time_s, single_task, dataset,
              teleop_action_processor, robot_action_processor, robot_observation_processor,
              display_data, label):
    """한 단계(물따르기 또는 컵배치)를 **녹화만** 한다. 저장(save_episode)은 하지 않는다.

    핸드오프 한 라운드는 '물따르기 + 컵배치'가 하나의 단위라, 어느 쪽이든 중간에
    끊기면(->키 안 누름) 아무것도 저장되면 안 된다. 그래서 이 함수는 버퍼를 채우기만
    하고, 저장/폐기 결정은 호출부가 두 단계 결과를 보고 일괄로 내린다.

    r(재녹화): 이 단계 버퍼를 비우고 다시 녹화(단계 내부에서만 처리).
    반환: -> 또는 n으로 정상 완료하면 True(버퍼 채워진 채, 저장 안 함),
          ESC/q로 중간 종료하면 False(버퍼는 호출부가 비운다).
    """
    while True:
        log_say(f"Recording episode {dataset.num_episodes} ({label})", True)
        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            teleop=teleop,
            dataset=dataset,
            control_time_s=episode_time_s,
            single_task=single_task,
            display_data=display_data,
            display_mode=DISPLAY_MODE,
        )
        if events["stop_recording"]:
            return False
        if events["rerecord_episode"]:
            log_say("Re-record", True)
            events["rerecord_episode"] = False
            events["exit_early"] = False
            dataset.clear_episode_buffer()
            continue
        return True


def main():
    init_logging()

    cams = resolve()
    logging.info(f"[camera resolve] left={cams['left']} right={cams['right']} depth={cams['depth']}")

    robot_cfg = BiSOFollowerConfig(
        id="arms",
        calibration_dir=CALIB_FOLLOWER,
        left_arm_config=SOFollowerConfig(
            port="COM15",
            gripper_protection=True,
            disable_torque_on_disconnect=False,
            cameras={"wrist": OpenCVCameraConfig(index_or_path=cams["left"], width=640, height=480, fps=30)},
        ),
        right_arm_config=SOFollowerConfig(
            port="COM16",
            disable_torque_on_disconnect=False,
            cameras={"wrist": OpenCVCameraConfig(index_or_path=cams["right"], width=640, height=480, fps=30)},
        ),
        cameras={"top": OpenCVCameraConfig(index_or_path=cams["depth"], width=640, height=480, fps=30, mirror=True)},
    )
    teleop_cfg = BiSOLeaderConfig(
        id="arms",
        calibration_dir=CALIB_LEADER,
        left_arm_config=SOLeaderConfig(port="COM5"),
        right_arm_config=SOLeaderConfig(port="COM14"),
    )

    robot = BiSOFollower(robot_cfg)
    teleop = BiSOLeader(teleop_cfg)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    from lerobot.datasets import aggregate_pipeline_dataset_features, create_initial_features  # noqa: E402
    from lerobot.utils.feature_utils import combine_feature_dicts  # noqa: E402

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )

    dataset_pour = make_dataset(POUR_REPO_ID, POUR_ROOT, POUR_TASK, robot, dataset_features)
    dataset_place = make_dataset(PLACE_REPO_ID, PLACE_ROOT, PLACE_TASK, robot, dataset_features)

    listener = None
    round_saved = False
    try:
        teleop.connect()
        robot.connect()

        listener, events = init_keyboard_listener()
        init_visualization(DISPLAY_MODE, session_name="recording")

        with VideoEncodingManager(dataset_pour), VideoEncodingManager(dataset_place):
            log_say("Round: pour", True)
            pour_ok = run_phase(
                robot, teleop, events, POUR_EPISODE_TIME_S, POUR_TASK, dataset_pour,
                teleop_action_processor, robot_action_processor, robot_observation_processor,
                display_data=True, label="pour",
            )

            place_ok = False
            if pour_ok:
                log_say("Round: cup place", True)
                place_ok = run_phase(
                    robot, teleop, events, PLACE_EPISODE_TIME_S, PLACE_TASK, dataset_place,
                    teleop_action_processor, robot_action_processor, robot_observation_processor,
                    display_data=True, label="cup place",
                )

            # 원자적 저장: 물따르기·컵배치 둘 다 ->로 완료됐을 때만 둘 다 저장한다.
            # 어느 하나라도 중간에 끊기면(ESC/q) 둘 다 버린다 - 짝 없는 에피소드나
            # 반쪽짜리 라운드가 절대 저장되지 않는다.
            if pour_ok and place_ok:
                dataset_pour.save_episode()
                dataset_place.save_episode()
                round_saved = True
                log_say("Round saved", True)
            else:
                dataset_pour.clear_episode_buffer()
                dataset_place.clear_episode_buffer()
                log_say("Round discarded - nothing saved", True)
    finally:
        log_say("Stop recording", True, blocking=True)

        dataset_pour.finalize()
        dataset_place.finalize()

        if robot.is_connected:
            robot.disconnect()
        if teleop.is_connected:
            teleop.disconnect()
        if listener is not None:
            listener.stop()

        shutdown_visualization(DISPLAY_MODE)

        # 이번 라운드가 실제로 저장됐을 때만 업로드한다. 중간에 끊긴 라운드는
        # 로컬에도 저장이 안 됐으니 HF도 건드리지 않는다(아무 일도 안 일어남).
        if round_saved:
            logging.info(f"물따르기 총 {dataset_pour.num_episodes}개, 컵배치 총 {dataset_place.num_episodes}개 - HF 업로드 시작")
            dataset_pour.push_to_hub(private=True)
            logging.info("물따르기 업로드 완료")
            dataset_place.push_to_hub(private=True)
            logging.info("컵배치 업로드 완료")
        else:
            logging.info("이번 라운드는 저장 안 됨 - 로컬·HF 모두 변경 없음, 업로드 생략")

        log_say("Exiting", True)


if __name__ == "__main__":
    main()
