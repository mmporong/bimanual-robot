"""속 빈 용기와 유체 입자 계수. 실측 물성이나 실물 성공 판정이 아니다."""
from __future__ import annotations

import numpy as np
import xml.etree.ElementTree as ET
import json
from pathlib import Path


def load_experiment(path):
    config = json.loads(Path(path).read_text())
    if config.get("schema") != "bimanual_pour_experiment_v1":
        raise ValueError("물붓기 설정 스키마 오류")
    positive = ("wall_m","bottle_body_height_m","bottle_shoulder_height_m","bottle_neck_height_m",
                "bottle_neck_outer_radius_m","particle_spacing_m","initial_fill_height_m","fluid_density_kg_m3","clear_bottle_mouth_z_m","pour_joint_speed_rad_s","pour_hold_s",
                "bottle_approach_right_offset_m", "cup_pregrasp_backoff_m")
    for key in positive:
        value = config[key]
        if type(value) not in (float,int) or not np.isfinite(value) or value <= 0:
            raise ValueError("유한한 양수 필요: "+key)
    for key in ("cup_work_center_m","bottle_mouth_target_m"):
        vector = np.asarray(config[key],dtype=float)
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ValueError("유한한 3벡터 필요: "+key)
    for key in ("additional_outward_mount_m", "pour_tilt_deg", "pour_azimuth_deg"):
        if type(config[key]) not in (float, int) or not np.isfinite(config[key]):
            raise ValueError("유한한 수 필요: "+key)
    if not 60 <= config["pour_azimuth_deg"] <= 120:
        raise ValueError("붓기 방향은 로봇 오른쪽에서 왼쪽(+Y)이어야 함")
    seed = np.asarray(config["right_pick_ik_seed_joint_deg"], dtype=float)
    if seed.shape != (5,) or not np.isfinite(seed).all():
        raise ValueError("오른손 파지 IK 초기값 오류")
    if not 0 <= config["additional_outward_mount_m"] <= .02 or not 90 <= config["pour_tilt_deg"] <= 110:
        raise ValueError("장착 가설 또는 붓기 각도 범위 오류")
    if config["pour_tilt_deg"] % 5 != 0:
        raise ValueError("붓기 각도는 5도 경로 격자에 맞아야 함")
    schedule = np.asarray(config["mouth_height_schedule"], dtype=float)
    if (schedule.ndim != 2 or schedule.shape[1] != 2 or len(schedule) < 2
            or not np.isfinite(schedule).all() or np.any(np.diff(schedule[:,0]) <= 0)
            or schedule[0,0] != 0 or schedule[-1,0] != config["pour_tilt_deg"]
            or not np.isclose(schedule[-1,1], config["bottle_mouth_target_m"][2])
            or np.any(schedule[:,1] <= 0)):
        raise ValueError("병 입구 높이 경로 오류")
    if config["bottle_neck_outer_radius_m"] <= config["wall_m"]+2*config["particle_spacing_m"]:
        raise ValueError("병목 내부 폭에 비해 입자가 큼")
    if config["initial_fill_height_m"] >= config["bottle_body_height_m"]-config["wall_m"]:
        raise ValueError("초기 유체가 병 몸통 밖에 있음")
    profile = np.asarray(config["cup_radius_profile_m"], dtype=float)
    if (profile.ndim != 2 or profile.shape[1] != 2 or len(profile) < 2
            or not np.isfinite(profile).all() or np.any(np.diff(profile[:,0]) <= 0)
            or np.any(profile[:,1] <= config["wall_m"])):
        raise ValueError("컵 높이별 반경 오류")
    return config


def minimum_jaw_gap(model):
    """현재 박스 블레이드·직선 관절 한계가 만드는 최소 간격(m)."""
    root = ET.parse(model).getroot()
    edges = []
    for index,sign in ((1,-1),(2,1)):
        joint = root.find(f"./joint[@name='right_finger{index}_joint']")
        if joint is None or joint.get("type") != "prismatic" or joint.find("limit") is None:
            raise ValueError("직선 죠 관절 한계 누락")
        collisions = root.find(f"./link[@name='right_finger{index}_link']").findall("collision")
        candidates = [c for c in collisions if c.find("geometry/box") is not None
                      and np.allclose(np.fromstring(c.find("geometry/box").get("size"),sep=" "),[.012,.060,.050])
                      and np.allclose(np.fromstring(c.find("origin").get("xyz"),sep=" "),[sign*.006,-.030,0])]
        if len(candidates) != 1:
            raise ValueError("순정 평면 블레이드 박스 계약 불일치")
        collision = candidates[0]
        if not np.allclose(np.fromstring(collision.find("origin").get("rpy","0 0 0"),sep=" "),0):
            raise ValueError("회전하지 않은 블레이드 박스 필요")
        axis = np.fromstring(joint.find("axis").get("xyz"),sep=" ")
        origin = joint.find("origin")
        if not np.allclose(axis,[sign,0,0]) or not np.allclose(np.fromstring(origin.get("rpy","0 0 0"),sep=" "),0):
            raise ValueError("직선 평행 죠 계약 불일치")
        lower = float(joint.find("limit").get("lower"))
        upper = float(joint.find("limit").get("upper"))
        if not np.isfinite([lower, upper]).all() or lower > upper:
            raise ValueError("죠 관절 한계 오류")
        center = np.fromstring(collision.find("origin").get("xyz"),sep=" ")[0]
        size = np.fromstring(collision.find("geometry/box").get("size"),sep=" ")[0]
        base = np.fromstring(origin.get("xyz"),sep=" ")[0]
        if not np.isfinite(base):
            raise ValueError("죠 관절 원점 오류")
        edges.append(base+sign*lower+center-sign*size/2)
    gap = edges[1]-edges[0]
    if not np.isfinite(gap) or gap <= .003:
        raise ValueError("죠 블레이드 자체 충돌 가능: 랙 쌍 필터 거부")
    return float(gap)


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError("유한한 wxyz 쿼터니언 필요")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def mount_clearance_evidence(model):
    """손목 원본 메쉬와 연결판 사이 Y 분리 평면을 관절 한계 내 1도 이하 간격으로 검사."""
    import hashlib
    from audit_bottle_target_geometry import binary_stl_vertices
    from cup_contact_model import Chain
    from solve_task_poses import rpy_matrix
    root = ET.parse(model).getroot()
    chain = Chain(model)
    limit = root.find("./joint[@name='right_wrist_roll']/limit")
    lower, upper = float(limit.get("lower")), float(limit.get("upper"))
    angles = np.linspace(lower, upper, int(np.ceil(np.degrees(upper-lower)))+1)
    boxes = root.findall("./link[@name='right_gripper_base_link']/collision")
    rear_y = max(float(c.find("origin").get("xyz").split()[1])
                 + float(c.find("geometry/box").get("size").split()[1])/2 for c in boxes)
    rows = []
    for collision in root.findall("./link[@name='right_wrist_link']/collision"):
        mesh = collision.find("geometry/mesh")
        path = Path(mesh.get("filename"))
        vertices = binary_stl_vertices(path)*np.fromstring(mesh.get("scale","1 1 1"),sep=" ")
        origin = collision.find("origin")
        rotation = rpy_matrix(*np.fromstring(origin.get("rpy","0 0 0"),sep=" "))
        vertices = vertices @ rotation.T + np.fromstring(origin.get("xyz","0 0 0"),sep=" ")
        minima = []
        for angle in angles:
            transforms = chain.transforms({"right_wrist_roll":angle})
            relative = np.linalg.inv(transforms["right_gripper_base_link"]) @ transforms["right_wrist_link"]
            minima.append(float((vertices @ relative[:3,:3].T + relative[:3,3])[:,1].min()))
        rows.append({"mesh":path.name,"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
                     "minimum_y_m":min(minima),"separating_plane_gap_m":min(minima)-rear_y})
    return {"model_sha256":hashlib.sha256(Path(model).read_bytes()).hexdigest(),
            "wrist_roll_range_rad":[lower,upper],"sample_count":len(angles),
            "connector_rear_y_m":rear_y,"meshes":rows,
            "minimum_sampled_gap_m":min(r["separating_plane_gap_m"] for r in rows),
            "scope":"sampled_wrist_mesh_to_base_box_plane_only",
            "physical_calibration":False,"full_robot_collision_certificate":False}


def contained_mask(points, position, orientation, radius, height, wall=.003, bottle_profile=False, cup_profile=None):
    local = (np.asarray(points)-position) @ quaternion_matrix(orientation)
    if bottle_profile:
        wall = bottle_profile["wall_m"]
        neck_inner = bottle_profile["bottle_neck_outer_radius_m"]-wall
        inner_radius = np.interp(local[:,2],[-height/2,-height/2+bottle_profile["bottle_body_height_m"],
            height/2-bottle_profile["bottle_neck_height_m"],height/2],[radius-wall,radius-wall,neck_inner,neck_inner])
    elif cup_profile is not None:
        profile = np.asarray(cup_profile, dtype=float)
        inner_radius = np.interp(local[:,2], profile[:,0], profile[:,1])-wall
    else:
        inner_radius = radius-wall
    return ((np.linalg.norm(local[:, :2], axis=1) < inner_radius)
            & (local[:, 2] > -height/2+wall) & (local[:, 2] < height/2))


def initial_liquid(center, radius, height, spacing=.004, fill_height=.04, wall=.003):
    if not (0 < spacing < radius-wall and 0 < fill_height < height-wall):
        raise ValueError("유체 격자와 용기 크기 오류")
    extent = radius-wall-spacing
    xy = np.arange(-extent, extent+1e-9, spacing)
    z = np.arange(-height/2+wall+spacing, -height/2+wall+fill_height, spacing)
    points = np.array([[x, y, zz] for x in xy for y in xy for zz in z
                       if x*x+y*y < extent*extent])
    if not len(points):
        raise ValueError("빈 유체 격자")
    return points + center


def liquid_counts(points, cup_pose, bottle_pose, cup_config, bottle_config):
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("유체 좌표 오류")
    def inside(pose, config):
        return contained_mask(points, *pose, config["cup_radius_m"], config["cup_height_m"],
                              wall=config.get("container_wall_m",.003),
                              bottle_profile=config.get("container_profile",False),
                              cup_profile=config.get("cup_radius_profile_m"))
    cup = inside(cup_pose, cup_config)
    bottle = inside(bottle_pose, bottle_config)
    return {"cup": int(cup.sum()), "bottle": int(bottle.sum()),
            "outside": int((~(cup | bottle)).sum()), "overlap": int((cup & bottle).sum()), "total": len(points)}


def maximum_contact_loss_s(samples, kind):
    """물체를 유지해야 하는 동안 양측 접촉이 끊긴 가장 긴 시간."""
    start, longest, previous = None, 0., None
    for sample in samples:
        timestamp = sample["time_s"]
        if not np.isfinite(timestamp) or (previous is not None and timestamp <= previous):
            return float("inf")
        if sample[kind+"_holding_expected"] and min(sample[kind+"_hand_n"]) < .02:
            if start is None:
                start = previous if previous is not None else timestamp
            longest = max(longest, timestamp-start)
        else:
            start = None
        previous = timestamp
    return longest


def inward_pour_direction(sample):
    """실제 병 축이 왼쪽을 향하고 몸통이 컵 오른쪽에 있는지 검사한다.

    로봇 기준 X 전방, Y 왼쪽, Z 위. 화면의 시계 방향과 무관하다.
    """
    try:
        axis = quaternion_matrix(sample["bottle_orientation_wxyz"])[:, 2]
        cup_position = np.asarray(sample["cup_position_m"], dtype=float)
        bottle_position = np.asarray(sample["bottle_position_m"], dtype=float)
        if cup_position.shape != (3,) or bottle_position.shape != (3,):
            return False
        if not np.isfinite([*cup_position, *bottle_position]).all():
            return False
        # +Y 중심 ±30도 방위각. 몸통 중심은 컵 중심보다 오른쪽에 남긴다.
        return bool(axis[1] > 0 and abs(axis[0]) <= axis[1]*np.tan(np.radians(30))
                    and bottle_position[1] < cup_position[1])
    except (KeyError, ValueError, TypeError):
        return False


def preclose_violation(sample, initial):
    """닫기 전 손가락 접촉과 용기 XY 이동을 검출한다. 단위 N, m."""
    stages={"RESET","REORIENT_ABOVE","PREGRASP_ABOVE","ALIGN_MIDDLE","APPROACH"}
    for side,kind in (("LEFT","cup"),("RIGHT","bottle")):
        if sample["phase"] not in {side+"_"+stage for stage in stages}:
            continue
        force=np.asarray(sample[kind+"_hand_n"],dtype=float)
        displacement=np.asarray(sample[kind+"_position_m"])-initial[kind+"_position_m"]
        if not np.isfinite(force).all() or not np.isfinite(displacement).all():
            return "nonfinite_preclose_observation:"+kind
        if np.max(force) >= .02:
            return "premature_hand_contact:"+kind
        if np.linalg.norm(displacement[:2]) > .003:
            return "container_displaced_before_close:"+kind
    return None


def evaluate_pour(samples, particle_count, completed):
    """컵 내부 잔류와 양팔 복귀·분리를 함께 판정한다. 누락은 실패다."""
    terminal = [s for s in samples if s["phase"] == "FINAL_HOLD"][-120:]
    if particle_count <= 0 or len(terminal) < 120:
        return {"task_pass": False, "reason": "final_window_missing"}
    received = min(s["liquid"]["cup"] for s in terminal)/particle_count
    outside = max(s["liquid"]["outside"] for s in terminal)/particle_count
    supported = all(s["cup_support_n"] > .04 and s["bottle_support_n"] > .12 for s in terminal)
    released = all(max(s["cup_hand_n"]+s["bottle_hand_n"]) < .02 for s in terminal)
    returned = all(abs(s[kind+"_rise_m"]) < .005 and s[kind+"_tilt_deg"] < 5
                   for s in terminal for kind in ("cup", "bottle"))
    placement_xy_error = {kind:max(float(np.linalg.norm(
        np.asarray(s[kind+"_position_m"])[:2]-np.asarray(samples[0][kind+"_position_m"])[:2]))
        for s in terminal) for kind in ("cup","bottle")}
    returned &= all(error <= .02 for error in placement_xy_error.values())
    opened = all(abs(s["left_gripper_actual_rad"]-1.) < .05
                 and max(abs(x-.0433) for x in s["right_gripper_actual_m"]) < .002 for s in terminal)
    settled = all(np.linalg.norm(np.ptp([s[kind+"_position_m"] for s in terminal],axis=0)) < .001
                  for kind in ("cup","bottle"))
    count_valid = all(s["liquid"]["total"] == particle_count and s["liquid"].get("overlap",1) == 0
                      and sum(s["liquid"][k] for k in ("cup","bottle","outside")) == particle_count for s in samples)
    lift_seen = True
    for kind, side in (("cup", "LEFT"), ("bottle", "RIGHT")):
        hold = [s for s in samples if s["phase"] == side+"_LIFT_HOLD"][-120:]
        lift_seen &= len(hold) == 120 and all(s[kind+"_rise_m"] > .04 and min(s[kind+"_hand_n"]) >= .02 for s in hold)
    initially_in_bottle = samples[0]["liquid"]["bottle"] >= .95*particle_count and samples[0]["liquid"]["cup"] == 0
    contact_loss = {kind:maximum_contact_loss_s(samples,kind) for kind in ("cup","bottle")}
    grasp_continuity = all(duration < .1 for duration in contact_loss.values())
    before_pour = [s for s in samples if s["phase"] == "POUR_MOVE"][-120:]
    pouring = [s for s in samples if s["phase"] == "POUR_HOLD"][-120:]
    def valid_transfer_pose(s):
        return ((s["phase"].startswith("POUR_TILT") or s["phase"] == "POUR_HOLD")
            and 90 <= s["bottle_tilt_deg"] <= 110 and s["cup_tilt_deg"] < 5
            and inward_pour_direction(s)
            and np.linalg.norm(s["mouth_relative_to_cup_rim_m"][:2]) < .027
            and .015 < s["mouth_relative_to_cup_rim_m"][2] < .08)
    valid_indices = [i for i,s in enumerate(samples) if valid_transfer_pose(s)]
    gained, drained = 0, 0
    for previous, current in zip(samples,samples[1:]):
        if valid_transfer_pose(previous) and valid_transfer_pose(current):
            gained += current["liquid"]["cup"]-previous["liquid"]["cup"]
            drained += previous["liquid"]["bottle"]-current["liquid"]["bottle"]
    pose_coupled_transfer = (bool(valid_indices) and gained >= .5*particle_count and drained >= .5*particle_count
        and all(s["liquid"]["bottle"] >= .95*particle_count and s["liquid"]["cup"] <= .05*particle_count
                for s in samples[:valid_indices[0]+1]))
    transfer_during_pour = (len(before_pour) == 120 and len(pouring) == 120
        and all(s["liquid"]["bottle"] >= .95*particle_count and s["liquid"]["cup"] <= .05*particle_count
                for s in before_pour)
        and min(s["liquid"]["cup"] for s in pouring)-max(s["liquid"]["cup"] for s in before_pour) >= .5*particle_count
        and min(s["liquid"]["bottle"] for s in before_pour)-max(s["liquid"]["bottle"] for s in pouring) >= .5*particle_count)
    actual_pour = len(pouring) == 120 and all(
        90 <= s["bottle_tilt_deg"] <= 110 and s["cup_tilt_deg"] < 5
        and np.linalg.norm(s["mouth_relative_to_cup_rim_m"][:2]) < .027
        and .015 < s["mouth_relative_to_cup_rim_m"][2] < .08
        and all(min(s[kind+"_hand_n"]) >= .02 and s[kind+"_grasp_slip_m"] < .02
                and s[kind+"_grasp_rotation_error_deg"] < 15 for kind in ("cup","bottle"))
        for s in pouring)
    inward_direction = len(pouring) == 120 and all(inward_pour_direction(s) for s in pouring)
    preclose_clear=not any(preclose_violation(s,samples[0]) for s in samples)
    passed = completed and received >= .5 and outside <= .05 and supported and released and returned and opened and settled and count_valid and lift_seen and initially_in_bottle and actual_pour and transfer_during_pour and grasp_continuity and pose_coupled_transfer and inward_direction and preclose_clear
    return {"task_pass": bool(passed), "received_fraction_min": received,
            "outside_fraction_max": outside, "supported": supported, "released": released,
            "returned_upright": returned,
            "final_xy_error_from_initial_m":placement_xy_error,
            "grippers_open":opened,"settled":settled,"actual_pour_pose":actual_pour,
            "inward_pour_direction":inward_direction,
            "preclose_clear":preclose_clear,
            "particle_count_conserved": count_valid, "both_lifted": lift_seen,
            "transfer_during_pour": bool(transfer_during_pour),
            "pose_coupled_transfer":bool(pose_coupled_transfer),
            "valid_pose_net_particles":{"cup_gain":gained,"bottle_drain":drained},
            "grasp_continuity":grasp_continuity,"maximum_contact_loss_s":contact_loss,
            "initially_in_bottle": initially_in_bottle,
            "reason": "criteria_pass" if passed else "criteria_failed",
            "physical_pour_verified": False, "model_calibrated": False}
