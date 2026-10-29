"""물따르기 -> 컵배치 라운드를 연결을 끊지 않고 Ctrl+C까지 연속 녹화한다.

기존 방식(두 개의 lerobot-record를 따로 실행)은 물따르기 프로세스가 끝나고
컵배치 프로세스가 새로 연결을 맺는 구간에서 계속 문제가 났다(포트 재연결
실패, 그 사이 리더를 손으로 붙잡고 있어야 하는 번거로움, 업로드 대기).
로봇·리더 연결은 실행 동안 한 번만 맺고, 라운드마다:

  1. 물따르기 에피소드 녹화 (-> 또는 n으로 종료)
  2. 끊김 없이 바로 이어서 컵배치 에피소드 녹화 (-> 또는 n으로 종료)
  3. 둘 다 끝났으면 둘 다 저장 -> 로컬 확정 + 각자의 HF 레포에 업로드
     (업로드 중엔 리더를 움직이지 말 것)
  4. 리셋 구간(녹화 없음, 텔레옵만): 병·컵 재배치 + 팔 시작 자세 -> -> 로 다음 라운드

Ctrl+C를 누르면 세션이 끝난다. 진행 중이던 라운드는 저장하지 않고, 이미
완료된 라운드는 각 라운드 끝에서 업로드돼 있다. 로컬 root가 이미 있으면
자동으로 resume되어 이어서 쌓인다.

카메라 인덱스는 매 실행 시 resolve_camera_indices.py로 새로 찾는다.

사용법:
    python tools/recording/record_handoff_combined.py

키 안내:
    -> 또는 n : 이 단계 종료(물따르기 -> 컵배치 -> 저장/업로드), 리셋 구간 종료
    r         : 이 단계 버리고 다시 녹화
    ESC 또는 q: 이번 라운드 폐기(저장 안 됨) 후 리셋 구간으로. 세션은 계속됨
    Ctrl+C    : 세션 종료
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

# 2026-10-23: 팔을 실제 로봇 차체에 결합한 뒤 수집분은 별도 레포로 쌓는다(이전 v2 = 테이블 거치 상태).
DATA_DIR = Path(r"C:\Users\USER\lerobot-data")
# 2026-10-25: part2(23개)를 -v1 뒤에 병합(87개)한 뒤 7개 삭제·20개를 -part2로 옮겨 -v1 60개, -part2 20개.
# 이후 수집은 -v1에 이어 쌓는다(다음 에피소드 60번).
# HF 시각화 사이트는 에피소드 목록 파일을 64개까지만 읽는다(라운드마다 1개씩 생김).
# 넘으면 수집을 마친 뒤 lerobot-edit-dataset으로 다시 묶어 올리면 된다.
DATASET_SUFFIX = "position-jitter-gravel-chassis-mounted-handoff-v1"

POUR_REPO_ID = f"jun-seo/water-pour-{DATASET_SUFFIX}"
POUR_ROOT = DATA_DIR / f"water-pour-{DATASET_SUFFIX}"
POUR_TASK = (
    "Pick up the cup with the left arm, pick up the water bottle with the right arm, "
    "pour water from the bottle into the cup, set the bottle down to the right of where "
    "it was picked up, move the cup onto the robot chassis with the left arm, and keep "
    "holding the cup."
)
POUR_EPISODE_TIME_S = 100

PLACE_REPO_ID = f"jun-seo/cup-place-{DATASET_SUFFIX}"
PLACE_ROOT = DATA_DIR / f"cup-place-{DATASET_SUFFIX}"
PLACE_TASK = (
    "While holding the cup already loaded on the robot chassis with the left arm, "
    "move it to the table and place it down."
)
PLACE_EPISODE_TIME_S = 50

FPS = 30
RESET_TIME_S = 120  # 라운드 사이 리셋 최대 시간. 준비되면 -> 로 바로 넘어간다


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

    procs = (teleop_action_processor, robot_action_processor, robot_observation_processor)

    listener = None
    round_idx = 0
    try:
        teleop.connect()
        robot.connect()

        listener, events = init_keyboard_listener()
        init_visualization(DISPLAY_MODE, session_name="recording")

        # Ctrl+C를 누를 때까지 라운드를 반복한다.
        while True:
            round_idx += 1
            # 라운드마다 데이터셋을 새로 연다(2라운드부터는 자동 resume). 라운드가
            # 끝나면 VideoEncodingManager가 finalize()해 파일을 완전히 닫으므로,
            # 쓰는 중인 데이터셋을 업로드하는 일이 없다.
            dataset_pour = make_dataset(POUR_REPO_ID, POUR_ROOT, POUR_TASK, robot, dataset_features)
            dataset_place = make_dataset(PLACE_REPO_ID, PLACE_ROOT, PLACE_TASK, robot, dataset_features)

            with VideoEncodingManager(dataset_pour), VideoEncodingManager(dataset_place):
                round_saved = run_round(robot, teleop, events, dataset_pour, dataset_place, procs, round_idx)

            if round_saved:
                upload_round(dataset_pour, dataset_place)

            # 다음 라운드 전 리셋: 녹화 없이 텔레옵만. 병·컵을 재배치하고 팔을 시작
            # 자세로 돌려놓은 뒤 -> 를 누르면 바로 다음 라운드가 시작된다.
            reset_events(events)
            log_say("Reset the environment, then press right arrow", True)
            logging.info(f"[리셋] 병·컵 재배치 + 팔을 시작 자세로. 준비되면 -> (최대 {RESET_TIME_S}s). 종료는 Ctrl+C")
            record_loop(
                robot=robot, events=events, fps=FPS,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                teleop=teleop, control_time_s=RESET_TIME_S,
                display_data=True, display_mode=DISPLAY_MODE,
            )
            reset_events(events)
    except KeyboardInterrupt:
        logging.info("Ctrl+C - 세션 종료. 진행 중이던 라운드는 저장하지 않는다(완료된 라운드는 이미 업로드됨).")
    finally:
        log_say("Stop recording", True, blocking=True)
        if robot.is_connected:
            robot.disconnect()
        if teleop.is_connected:
            teleop.disconnect()
        if listener is not None:
            listener.stop()
        shutdown_visualization(DISPLAY_MODE)
        log_say("Exiting", True)


def reset_events(events):
    events["exit_early"] = False
    events["rerecord_episode"] = False
    events["stop_recording"] = False


def run_round(robot, teleop, events, dataset_pour, dataset_place, procs, round_idx):
    """물따르기 -> 컵배치 한 라운드. 둘 다 -> 로 끝났을 때만 둘 다 저장(원자적)."""
    reset_events(events)
    log_say(f"Round {round_idx}: pour", True)
    pour_ok = run_phase(robot, teleop, events, POUR_EPISODE_TIME_S, POUR_TASK, dataset_pour,
                        *procs, display_data=True, label="pour")
    place_ok = False
    if pour_ok:
        log_say(f"Round {round_idx}: cup place", True)
        place_ok = run_phase(robot, teleop, events, PLACE_EPISODE_TIME_S, PLACE_TASK, dataset_place,
                             *procs, display_data=True, label="cup place")

    if pour_ok and place_ok:
        dataset_pour.save_episode()
        dataset_place.save_episode()
        log_say("Round saved", True)
        return True
    dataset_pour.clear_episode_buffer()
    dataset_place.clear_episode_buffer()
    log_say("Round discarded - nothing saved", True)
    return False


def upload_round(dataset_pour, dataset_place):
    """완전히 닫힌 두 데이터셋을 HF에 올린다. 실패해도 로컬은 안전하고 다음 라운드
    업로드 때 함께 올라가므로, 세션을 끊지 않고 경고만 남긴다."""
    log_say("Uploading - keep the leader still", True)
    logging.info(f"물따르기 총 {dataset_pour.num_episodes}개, 컵배치 총 {dataset_place.num_episodes}개 - HF 업로드 (리더 움직이지 말 것)")
    for name, ds in (("물따르기", dataset_pour), ("컵배치", dataset_place)):
        try:
            ds.push_to_hub(private=True)
            logging.info(f"{name} 업로드 완료")
        except Exception as e:  # noqa: BLE001
            logging.warning(f"{name} 업로드 실패({e}) - 로컬엔 저장됨, 다음 라운드에 다시 올라감")


if __name__ == "__main__":
    main()
