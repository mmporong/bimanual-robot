#!/usr/bin/env python3
"""HOLD THE FLOW 250 mm 차체의 재현 가능한 1차 기구 계산.

외부 패키지 없이 실행한다. 좌우 SO-101 링크 COM은 로컬 기준 URDF와
선정된 운반/붓기 관절 자세를 FK로 계산한 값을 사용한다. 실제 CAD 질량과
관성으로 교체하기 전까지는 설계 게이트용 보수 계산이다.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass


G = 9.81
# 2026-09-03 개정: 로드 조달 불가로 선정 그리퍼를 ggao50(완전 출력형)로 바꾸고,
# 붓기 물체를 물 50 mL 기준 60 g으로 낮춘다. 물체를 낮추면 컵 유지 토크가 내려가
# 165 g 그리퍼가 정격 안에 들어온다.
SELECTED_GRIPPER_MASS_KG = 0.165      # ggao50 SO101-Parallel-Gripper
ROBONINE_GRIPPER_MASS_KG = 0.170      # 비교용 대안 (Ø6 로드 2 + 베어링 2 필요)
LIGHTWEIGHT_TARGET_MASS_KG = 0.130    # 경량 파생형 목표. 현재 필수 조건 아님
OBJECT_MASS_KG = 0.060                # 병: 빈 500 mL 페트 25 g + 물 35 mL 상당
GATED_OBJECT_MASS_KG = 0.120          # 전류·온도 통과 뒤 허용
WHEEL_RADIUS_M = 0.0329
TOOL_HORIZONTAL_REACH_GATE_M = 0.280

# 컵 유지 중력토크 기준점: 그리퍼 130 g + 물체 140 g 조합에서 0.893 N·m.
# 여기서 벗어난 조합은 공구측 질량 증감을 280 mm 수평 레버암으로 환산해 더한다.
CUP_HOLD_TORQUE_BASE_NM = 0.893
CUP_HOLD_BASE_TOOL_MASS_KG = LIGHTWEIGHT_TARGET_MASS_KG + 0.140


@dataclass(frozen=True)
class Component:
    name: str
    mass_kg: float
    xyz_m: tuple[float, float, float]


COMMON_COMPONENTS = (
    Component("plates_backings_spacers", 1.150, (0.000, 0.000, 0.070)),
    Component("camera_mast", 0.5043, (-0.080, 0.000, 0.450)),
    Component("astra_and_bracket", 0.400, (-0.080, 0.000, 0.800)),
    Component("drive_wheels_mounts_caster", 0.450, (0.060, 0.000, 0.045)),
    Component("battery", 0.550, (-0.045, 0.000, 0.040)),
    Component("electronics", 0.350, (0.000, 0.000, 0.080)),
    Component("lidar", 0.042, (0.090, 0.000, 0.165)),
    Component("wiring_fasteners", 0.250, (0.000, 0.000, 0.080)),
)


POUR_COMPONENTS = (
    Component("left_arm_without_gripper", 0.533006, (0.087497, 0.077050, 0.296208)),
    Component("right_arm_without_gripper", 0.533006, (0.093235, -0.050408, 0.286602)),
    Component("left_parallel_gripper", SELECTED_GRIPPER_MASS_KG, (0.271683, 0.114913, 0.539505)),
    Component("right_parallel_gripper", SELECTED_GRIPPER_MASS_KG, (0.301724, 0.047315, 0.444633)),
    Component("bottle_total", OBJECT_MASS_KG, (0.271683, 0.114913, 0.539505)),
    Component("cup_total", OBJECT_MASS_KG, (0.301724, 0.047315, 0.444633)),
)


TRANSPORT_COMPONENTS = (
    Component("left_arm_without_gripper", 0.533006, (0.042240, 0.061869, 0.267619)),
    Component("right_arm_without_gripper", 0.533006, (0.042413, -0.056518, 0.267544)),
    Component("left_parallel_gripper", SELECTED_GRIPPER_MASS_KG, (0.157236, 0.133610, 0.415848)),
    Component("right_parallel_gripper", SELECTED_GRIPPER_MASS_KG, (0.144204, -0.134074, 0.424740)),
    Component("bottle_total", OBJECT_MASS_KG, (0.157236, 0.133610, 0.415848)),
    Component("cup_total", OBJECT_MASS_KG, (0.144204, -0.134074, 0.424740)),
)


def cup_hold_gravity_torque_nm(gripper_mass_kg: float, object_mass_kg: float) -> float:
    """컵을 든 팔의 보수적 중력토크. 기준 조합에서 공구측 질량 증감만 반영한다."""
    delta = (gripper_mass_kg + object_mass_kg) - CUP_HOLD_BASE_TOOL_MASS_KG
    return CUP_HOLD_TORQUE_BASE_NM + delta * G * TOOL_HORIZONTAL_REACH_GATE_M


def mass_and_com(components: tuple[Component, ...]) -> tuple[float, tuple[float, float, float]]:
    total = sum(component.mass_kg for component in components)
    com = tuple(
        sum(component.mass_kg * component.xyz_m[axis] for component in components) / total
        for axis in range(3)
    )
    return total, com


def replace_gripper_mass(
    components: tuple[Component, ...], mass_kg: float
) -> tuple[Component, ...]:
    """같은 자세에서 그리퍼 질량만 바꿔 공개 기준기의 영향을 비교한다."""
    return tuple(
        Component(component.name, mass_kg, component.xyz_m)
        if component.name.endswith("parallel_gripper")
        else component
        for component in components
    )


def support_reactions_kg(
    total_mass_kg: float,
    com_xy_m: tuple[float, float],
    wheel_axis_x_m: float = 0.090,
    wheel_contact_y_m: float = 0.135,
    caster_x_m: float = -0.105,
) -> tuple[float, float, float]:
    """왼바퀴, 오른바퀴, 후방 캐스터의 등가 질량 반력을 반환한다."""
    com_x, com_y = com_xy_m
    caster = total_mass_kg * (wheel_axis_x_m - com_x) / (wheel_axis_x_m - caster_x_m)
    wheel_sum = total_mass_kg - caster
    left_minus_right = total_mass_kg * com_y / wheel_contact_y_m
    left = (wheel_sum + left_minus_right) / 2.0
    right = (wheel_sum - left_minus_right) / 2.0
    return left, right, caster


def dynamic_front_margin_m(
    com_xyz_m: tuple[float, float, float],
    wheel_axis_x_m: float = 0.090,
    slope_deg: float = 5.0,
    deceleration_m_s2: float = 0.3,
) -> float:
    com_x, _, com_z = com_xyz_m
    effective_x = com_x + com_z * (math.tan(math.radians(slope_deg)) + deceleration_m_s2 / G)
    return wheel_axis_x_m - effective_x


def drive_requirement(
    total_mass_kg: float,
    wheel_radius_m: float = WHEEL_RADIUS_M,
    slope_deg: float = 5.0,
    acceleration_m_s2: float = 0.3,
    rolling_resistance: float = 0.02,
    safety_factor: float = 2.0,
) -> tuple[float, float]:
    theta = math.radians(slope_deg)
    traction_n = total_mass_kg * (
        acceleration_m_s2 + G * math.sin(theta) + rolling_resistance * G * math.cos(theta)
    )
    torque_each_nm = traction_n * wheel_radius_m / 2.0
    return traction_n, torque_each_nm * safety_factor


def wheel_speed_m_s(wheel_radius_m: float = WHEEL_RADIUS_M, rpm: float = 45.0) -> float:
    return 2.0 * math.pi * wheel_radius_m * rpm / 60.0


def rectangular_tube_deflection_mm(
    length_m: float = 0.681,
    outer_width_m: float = 0.045,
    outer_height_m: float = 0.035,
    wall_m: float = 0.003,
    elastic_modulus_pa: float = 2.0e9,
    lateral_force_n: float = 1.0,
) -> float:
    inner_width = outer_width_m - 2.0 * wall_m
    inner_height = outer_height_m - 2.0 * wall_m
    inertia = (
        outer_width_m * outer_height_m**3 - inner_width * inner_height**3
    ) / 12.0
    return lateral_force_n * length_m**3 / (3.0 * elastic_modulus_pa * inertia) * 1000.0


def plate_deflection_mm(
    thickness_m: float,
    moment_nm: float = 1.6,
    strip_width_m: float = 0.090,
    effective_length_m: float = 0.065,
    elastic_modulus_pa: float = 2.0e9,
) -> float:
    inertia = strip_width_m * thickness_m**3 / 12.0
    return moment_nm * effective_length_m**2 / (2.0 * elastic_modulus_pa * inertia) * 1000.0


def gripper_force_each_jaw_n(
    object_mass_kg: float = 0.140,
    friction_coefficient: float = 0.5,
    safety_factor: float = 2.0,
) -> float:
    return safety_factor * object_mass_kg * G / (2.0 * friction_coefficient)


def main() -> None:
    pour_mass, pour_com = mass_and_com(COMMON_COMPONENTS + POUR_COMPONENTS)
    transport_mass, transport_com = mass_and_com(COMMON_COMPONENTS + TRANSPORT_COMPONENTS)
    robonine_pour_mass, robonine_pour_com = mass_and_com(
        COMMON_COMPONENTS + replace_gripper_mass(POUR_COMPONENTS, ROBONINE_GRIPPER_MASS_KG)
    )
    robonine_transport_mass, robonine_transport_com = mass_and_com(
        COMMON_COMPONENTS + replace_gripper_mass(TRANSPORT_COMPONENTS, ROBONINE_GRIPPER_MASS_KG)
    )
    pour_reactions = support_reactions_kg(pour_mass, pour_com[:2])
    transport_reactions = support_reactions_kg(transport_mass, transport_com[:2])
    robonine_pour_reactions = support_reactions_kg(robonine_pour_mass, robonine_pour_com[:2])
    robonine_transport_reactions = support_reactions_kg(
        robonine_transport_mass, robonine_transport_com[:2]
    )
    traction_n, drive_torque_nm = drive_requirement(transport_mass)
    robonine_traction_n, robonine_drive_torque_nm = drive_requirement(robonine_transport_mass)

    cup_hold = {
        "selected_ggao50_165g_with_60g_object": cup_hold_gravity_torque_nm(
            SELECTED_GRIPPER_MASS_KG, OBJECT_MASS_KG
        ),
        "selected_ggao50_165g_with_120g_object": cup_hold_gravity_torque_nm(
            SELECTED_GRIPPER_MASS_KG, GATED_OBJECT_MASS_KG
        ),
        "robonine_170g_with_60g_object": cup_hold_gravity_torque_nm(
            ROBONINE_GRIPPER_MASS_KG, OBJECT_MASS_KG
        ),
        "robonine_170g_with_120g_object": cup_hold_gravity_torque_nm(
            ROBONINE_GRIPPER_MASS_KG, GATED_OBJECT_MASS_KG
        ),
        "lightweight_130g_with_120g_object": cup_hold_gravity_torque_nm(
            LIGHTWEIGHT_TARGET_MASS_KG, GATED_OBJECT_MASS_KG
        ),
    }

    usable_battery_wh = 11.1 * 5.0 * 0.8
    report = {
        "mass": {
            "robot_without_objects_kg": round(pour_mass - 2.0 * OBJECT_MASS_KG, 3),
            "pour_with_objects_kg": round(pour_mass, 3),
            "transport_with_objects_kg": round(transport_mass, 3),
        },
        "pour": {
            "com_mm": [round(value * 1000.0, 1) for value in pour_com],
            "support_reactions_kg": [round(value, 3) for value in pour_reactions],
            "support_reactions_percent": [round(value / pour_mass * 100.0, 1) for value in pour_reactions],
            "static_front_margin_mm": round((0.090 - pour_com[0]) * 1000.0, 1),
            "dynamic_front_margin_5deg_0_3m_s2_mm": round(dynamic_front_margin_m(pour_com) * 1000.0, 1),
        },
        "transport": {
            "com_mm": [round(value * 1000.0, 1) for value in transport_com],
            "support_reactions_kg": [round(value, 3) for value in transport_reactions],
            "support_reactions_percent": [round(value / transport_mass * 100.0, 1) for value in transport_reactions],
            "static_front_margin_mm": round((0.090 - transport_com[0]) * 1000.0, 1),
            "dynamic_front_margin_5deg_0_3m_s2_mm": round(dynamic_front_margin_m(transport_com) * 1000.0, 1),
        },
        "robonine_reference_gripper_170g_with_120g_objects": {
            "robot_without_objects_kg": round(robonine_pour_mass - 2.0 * OBJECT_MASS_KG, 3),
            "pour_with_objects_kg": round(robonine_pour_mass, 3),
            "transport_with_objects_kg": round(robonine_transport_mass, 3),
            "pour_com_mm": [round(value * 1000.0, 1) for value in robonine_pour_com],
            "transport_com_mm": [round(value * 1000.0, 1) for value in robonine_transport_com],
            "pour_support_reactions_percent": [
                round(value / robonine_pour_mass * 100.0, 1)
                for value in robonine_pour_reactions
            ],
            "transport_support_reactions_percent": [
                round(value / robonine_transport_mass * 100.0, 1)
                for value in robonine_transport_reactions
            ],
            "pour_dynamic_front_margin_mm": round(
                dynamic_front_margin_m(robonine_pour_com) * 1000.0, 1
            ),
            "transport_dynamic_front_margin_mm": round(
                dynamic_front_margin_m(robonine_transport_com) * 1000.0, 1
            ),
            "drive_traction_required_n": round(robonine_traction_n, 3),
            "drive_torque_each_with_sf2_nm": round(robonine_drive_torque_nm, 3),
            "cup_hold_gravity_torque_60g_object_nm": round(
                cup_hold["robonine_170g_with_60g_object"], 3
            ),
            "cup_hold_gravity_torque_120g_object_nm": round(
                cup_hold["robonine_170g_with_120g_object"], 3
            ),
        },
        "drive": {
            "traction_required_n": round(traction_n, 3),
            "torque_each_with_sf2_nm": round(drive_torque_nm, 3),
            "sts3215_c018_rated_torque_nm": 0.981,
            "rated_torque_usage_percent": round(drive_torque_nm / 0.981 * 100.0, 1),
            "speed_at_45rpm_m_s": round(wheel_speed_m_s(), 3),
        },
        "power": {
            "battery_nominal_wh": 55.5,
            "battery_usable_wh": round(usable_battery_wh, 1),
            "runtime_at_42_5w_min": round(usable_battery_wh / 42.5 * 60.0, 1),
            "runtime_at_60w_min": round(usable_battery_wh / 60.0 * 60.0, 1),
        },
        "structure": {
            "top_plate_8mm_deflection_at_1_6Nm_mm": round(plate_deflection_mm(0.008), 3),
            "top_plus_backing_14mm_ideal_deflection_mm": round(plate_deflection_mm(0.014), 3),
            "mast_45x35x3_1N_ideal_deflection_mm": round(rectangular_tube_deflection_mm(), 3),
            "gripper_force_each_jaw_for_140g_n": round(gripper_force_each_jaw_n(), 3),
        },
        "cup_hold_gravity_torque_nm": {
            key: round(value, 3) for key, value in cup_hold.items()
        },
        "cup_hold_rated_usage_percent": {
            key: round(value / 0.981 * 100.0, 1) for key, value in cup_hold.items()
        },
        "payload_policy_kg": {
            "initial_each": OBJECT_MASS_KG,
            "gated_next_each": GATED_OBJECT_MASS_KG,
            "selected_gripper_each": SELECTED_GRIPPER_MASS_KG,
        },
    }

    assert pour_mass <= 5.30
    assert min(pour_reactions) / pour_mass >= 0.05
    assert min(transport_reactions) / transport_mass >= 0.05
    assert dynamic_front_margin_m(pour_com) >= 0.020
    assert dynamic_front_margin_m(transport_com) >= 0.020
    assert drive_torque_nm <= 0.300
    assert wheel_speed_m_s() >= 0.150
    assert cup_hold["selected_ggao50_165g_with_60g_object"] < 0.981
    assert dynamic_front_margin_m(robonine_pour_com) >= 0.020
    assert robonine_drive_torque_nm <= 0.300
    assert cup_hold["selected_ggao50_165g_with_120g_object"] < 0.981
    assert cup_hold["robonine_170g_with_120g_object"] < 0.981

    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
