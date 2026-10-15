"""Source locations used by simulation result provenance after domain separation."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL_SOURCES = {
    "isaac_phase_executor.py": "simulation/isaac_phase_executor.py",
    "restaurant_roundtrip.py": "simulation/restaurant_roundtrip.py",
    "service_cinematic.py": "simulation/service_cinematic.py",
    "tray_pose_probe.py": "simulation/tray_pose_probe.py",
    "simulate_bimanual_pour.py": "simulation/simulate_bimanual_pour.py",
    "audit_bottle_target_geometry.py": "src/manipulation/audit_bottle_target_geometry.py",
    "simulate_urdf_workcell.py": "simulation/simulate_urdf_workcell.py",
    "workcell_envelopes.py": "simulation/workcell_envelopes.py",
    "workcell_recording.py": "simulation/workcell_recording.py",
    "bimanual_pour_plan.py": "src/manipulation/bimanual_pour_plan.py",
    "bottle_contact_model.py": "src/manipulation/bottle_contact_model.py",
    "build_restaurant_scene.py": "simulation/build_restaurant_scene.py",
    "cup_contact_model.py": "src/manipulation/cup_contact_model.py",
    "cup_contact_place.py": "src/manipulation/cup_contact_place.py",
    "cup_contact_recovery.py": "src/manipulation/cup_contact_recovery.py",
    "mobile_service_control.py": "simulation/mobile_service_control.py",
    "mobile_service_model.py": "simulation/mobile_service_model.py",
    "plan_body_side_grasp.py": "src/manipulation/plan_body_side_grasp.py",
    "pour_geometry.py": "src/manipulation/pour_geometry.py",
    "raised_tray_transfer.py": "src/manipulation/raised_tray_transfer.py",
    "released_cup_probe.py": "simulation/released_cup_probe.py",
    "restaurant_layout.py": "simulation/restaurant_layout.py",
    "search_tray_mounts.py": "src/manipulation/search_tray_mounts.py",
    "service_camera_views.py": "simulation/service_camera_views.py",
    "simulate_bottle_contact.py": "simulation/simulate_bottle_contact.py",
    "simulate_cup_contact.py": "simulation/simulate_cup_contact.py",
    "simulate_restaurant_mobile.py": "simulation/simulate_restaurant_mobile.py",
    "tray_transfer_plan.py": "src/manipulation/tray_transfer_plan.py",
    "water_service_mission.py": "simulation/water_service_mission.py",
    "workcell_preview_inputs.py": "simulation/workcell_preview_inputs.py",
    "workcell_preview_motion.py": "src/manipulation/workcell_preview_motion.py"
}


def tool_path(filename: str) -> Path:
    return ROOT / TOOL_SOURCES[filename]


def source_paths(filenames) -> dict[str, str]:
    return {filename: TOOL_SOURCES[filename] for filename in filenames}
