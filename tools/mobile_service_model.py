"""Fixed URDF assemblies do not collide internally; moving joints remain separate."""
from itertools import combinations
import math
import xml.etree.ElementTree as ET

GROUND_LINKS = {"left_wheel_link", "right_wheel_link", "front_caster_link", "rear_caster_link"}


def ground_collision(forces, threshold=.05):
    return any(not math.isfinite(force) or (name not in GROUND_LINKS and force > threshold)
               for name, force in forces.items())


def ground_supported(forces):
    return (not ground_collision(forces)
            and forces.get("left_wheel_link", 0.) > .1
            and forces.get("right_wheel_link", 0.) > .1
            and max(forces.get("front_caster_link", 0.), forces.get("rear_caster_link", 0.)) > .1)


def ground_window_verified(samples, window_count=5):
    """10 Hz records: reject sustained 0.5 s support loss, not a single solver impulse."""
    if len(samples) < window_count:
        return False
    if any(ground_collision(s["ground_force_n"]) or not math.isfinite(s["position_m"][2])
           or abs(s["position_m"][2]) > .01 or not math.isfinite(s["tilt_deg"])
           or s["tilt_deg"] > 1. for s in samples):
        return False
    for end in range(window_count, len(samples)+1):
        window = samples[end-window_count:end]
        forces = {name: sum(s["ground_force_n"].get(name, 0.) for s in window)/window_count
                  for name in GROUND_LINKS}
        if not ground_supported(forces):
            return False
    return True


def fixed_component_pairs(model, available_names):
    root = ET.parse(model).getroot()
    parent = {link.attrib["name"]: link.attrib["name"] for link in root.findall("link")}

    def find(name):
        while parent[name] != name:
            parent[name] = parent[parent[name]]
            name = parent[name]
        return name

    for joint in root.findall("joint"):
        if joint.attrib["type"] == "fixed":
            a = joint.find("parent").attrib["link"]
            b = joint.find("child").attrib["link"]
            parent[find(a)] = find(b)
    groups = {}
    for name in sorted(available_names):
        groups.setdefault(find(name), []).append(name)
    return [pair for group in groups.values() for pair in combinations(group, 2)]
