"""Presentation cameras only; no sensor calibration or simulation-state writes."""
import numpy as np


def camera_views(config, state, phase, cup_position_m):
    wide = config["wide"]
    result = {"wide": {"eye_m": list(wide["eye_m"]), "target_m": list(wide["target_m"]),
                       "focal_length_mm": wide["focal_length_mm"]}}
    if state == "POUR" and phase.startswith("POUR_"):
        detail = config["pour_detail"]
        cup = np.asarray(cup_position_m, dtype=float)
        result["pour_detail"] = {
            "eye_m": (cup + detail["eye_offset_from_cup_m"]).tolist(),
            "target_m": (cup + detail["target_offset_from_cup_m"]).tolist(),
            "focal_length_mm": detail["focal_length_mm"],
        }
    for view in result.values():
        values = [*view["eye_m"], *view["target_m"], view["focal_length_mm"]]
        if not np.isfinite(values).all() or view["focal_length_mm"] <= 0:
            raise ValueError("invalid presentation camera")
        if np.linalg.norm(np.subtract(view["eye_m"], view["target_m"])) < .01:
            raise ValueError("camera eye and target coincide")
    return result
