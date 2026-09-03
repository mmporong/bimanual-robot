#!/usr/bin/env python3
"""커밋된 URDF와 STL에서 설계팀 인계용 상태 이미지를 생성한다.

AI 콘셉트 이미지를 사용하지 않고 현재 저장소의 실제 메시와 관절 좌표를 렌더한다.
길이 계산과 VTK 렌더 좌표는 mm를 사용한다.
"""

from __future__ import annotations

import argparse
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import vtk
import yaml
from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = REPO_ROOT / "src/hold_flow_description/urdf/hold_flow.urdf"
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_2.yaml"
MANIFEST_PATH = REPO_ROOT / "design/cad/exports/manifest.json"
DEFAULT_OUTPUT = REPO_ROOT / "docs/assets/design_handoff"

FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_MEDIUM = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")

INK = "#111827"
MUTED = "#667085"
LINE = "#D9E2F0"
BLUE = "#0B5FFF"
BLUE_LIGHT = "#EAF1FF"
GREEN = "#087A55"
AMBER = "#B54708"
RED = "#B42318"
WHITE = "#FFFFFF"


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    mimic: tuple[str, float, float] | None


def font(size: int, weight: str = "regular") -> ImageFont.FreeTypeFont:
    path = {"regular": FONT_REGULAR, "medium": FONT_MEDIUM, "bold": FONT_BOLD}[weight]
    return ImageFont.truetype(str(path), size=size, index=0)


def parse_vector(value: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not value:
        return np.asarray(default, dtype=float)
    return np.asarray([float(item) for item in value.split()], dtype=float)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def transform(xyz_m: np.ndarray | None = None, rpy: np.ndarray | None = None) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_matrix(np.zeros(3) if rpy is None else rpy)
    if xyz_m is not None:
        matrix[:3, 3] = xyz_m * 1000.0
    return matrix


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = np.linalg.norm(axis)
    if norm == 0:
        return np.eye(4)
    x, y, z = axis / norm
    c, s, one_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ]
    )
    return matrix


def vtk_matrix(matrix: np.ndarray) -> vtk.vtkMatrix4x4:
    output = vtk.vtkMatrix4x4()
    for row in range(4):
        for column in range(4):
            output.SetElement(row, column, float(matrix[row, column]))
    return output


def color_tuple(hex_value: str) -> tuple[float, float, float]:
    value = hex_value.lstrip("#")
    return tuple(int(value[index : index + 2], 16) / 255.0 for index in (0, 2, 4))


def resolve_package_mesh(filename: str) -> Path:
    prefix = "package://hold_flow_description/"
    if not filename.startswith(prefix):
        raise ValueError(f"지원하지 않는 메시 URI: {filename}")
    return REPO_ROOT / "src/hold_flow_description" / filename.removeprefix(prefix)


def actor_from_mesh(path: Path, matrix: np.ndarray, rgb: tuple[float, float, float]) -> vtk.vtkActor:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetUserMatrix(vtk_matrix(matrix))
    actor.GetProperty().SetColor(*rgb)
    actor.GetProperty().SetInterpolationToPhong()
    actor.GetProperty().SetSpecular(0.12)
    actor.GetProperty().SetSpecularPower(18)
    return actor


def primitive_actor(kind: str, values: tuple[float, ...], matrix: np.ndarray, rgb: tuple[float, float, float], opacity: float = 1.0) -> vtk.vtkActor:
    local = np.eye(4)
    if kind == "box":
        source = vtk.vtkCubeSource()
        source.SetXLength(values[0] * 1000.0)
        source.SetYLength(values[1] * 1000.0)
        source.SetZLength(values[2] * 1000.0)
    elif kind == "cylinder":
        source = vtk.vtkCylinderSource()
        source.SetRadius(values[0] * 1000.0)
        source.SetHeight(values[1] * 1000.0)
        source.SetResolution(72)
        local = transform(rpy=np.array([math.pi / 2.0, 0.0, 0.0]))
    elif kind == "sphere":
        source = vtk.vtkSphereSource()
        source.SetRadius(values[0] * 1000.0)
        source.SetThetaResolution(48)
        source.SetPhiResolution(32)
    else:
        raise ValueError(kind)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetUserMatrix(vtk_matrix(matrix @ local))
    actor.GetProperty().SetColor(*rgb)
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetInterpolationToPhong()
    actor.GetProperty().SetSpecular(0.12)
    actor.GetProperty().SetSpecularPower(18)
    return actor


class UrdfScene:
    def __init__(self, path: Path):
        self.root = ET.parse(path).getroot()
        self.materials = self._materials()
        self.links = {element.attrib["name"]: element for element in self.root.findall("link")}
        self.joints = self._joints()
        self.children = {joint.child for joint in self.joints}
        self.root_link = next(name for name in self.links if name not in self.children)

    def _materials(self) -> dict[str, tuple[float, float, float]]:
        materials: dict[str, tuple[float, float, float]] = {}
        for item in self.root.findall("material"):
            color = item.find("color")
            if color is not None:
                rgba = [float(value) for value in color.attrib["rgba"].split()]
                materials[item.attrib["name"]] = tuple(rgba[:3])
        return materials

    def _joints(self) -> list[Joint]:
        output: list[Joint] = []
        for element in self.root.findall("joint"):
            origin = element.find("origin")
            xyz = parse_vector(None if origin is None else origin.attrib.get("xyz"), (0, 0, 0))
            rpy = parse_vector(None if origin is None else origin.attrib.get("rpy"), (0, 0, 0))
            axis_element = element.find("axis")
            axis = parse_vector(None if axis_element is None else axis_element.attrib.get("xyz"), (1, 0, 0))
            mimic_element = element.find("mimic")
            mimic = None
            if mimic_element is not None:
                mimic = (
                    mimic_element.attrib["joint"],
                    float(mimic_element.attrib.get("multiplier", "1")),
                    float(mimic_element.attrib.get("offset", "0")),
                )
            output.append(
                Joint(
                    name=element.attrib["name"],
                    joint_type=element.attrib["type"],
                    parent=element.find("parent").attrib["link"],
                    child=element.find("child").attrib["link"],
                    origin=transform(xyz, rpy),
                    axis=axis,
                    mimic=mimic,
                )
            )
        return output

    def link_transforms(self, positions: dict[str, float]) -> dict[str, np.ndarray]:
        transforms = {self.root_link: np.eye(4)}
        pending = list(self.joints)
        while pending:
            progressed = False
            for joint in pending[:]:
                if joint.parent not in transforms:
                    continue
                value = positions.get(joint.name, 0.0)
                if joint.mimic is not None:
                    source, multiplier, offset = joint.mimic
                    value = positions.get(source, 0.0) * multiplier + offset
                motion = np.eye(4)
                if joint.joint_type in {"revolute", "continuous"}:
                    motion = axis_angle(joint.axis, value)
                elif joint.joint_type == "prismatic":
                    motion[:3, 3] = joint.axis * value * 1000.0
                transforms[joint.child] = transforms[joint.parent] @ joint.origin @ motion
                pending.remove(joint)
                progressed = True
            if not progressed:
                names = ", ".join(joint.name for joint in pending)
                raise ValueError(f"URDF 트리 연결 실패: {names}")
        return transforms

    def actors(self, positions: dict[str, float]) -> list[vtk.vtkActor]:
        transforms = self.link_transforms(positions)
        output: list[vtk.vtkActor] = []
        for name, element in self.links.items():
            link_matrix = transforms[name]
            for visual in element.findall("visual"):
                origin = visual.find("origin")
                xyz = parse_vector(None if origin is None else origin.attrib.get("xyz"), (0, 0, 0))
                rpy = parse_vector(None if origin is None else origin.attrib.get("rpy"), (0, 0, 0))
                visual_matrix = link_matrix @ transform(xyz, rpy)
                material = visual.find("material")
                material_name = "silver" if material is None else material.attrib.get("name", "silver")
                rgb = self.materials.get(material_name, (0.65, 0.68, 0.72))
                geometry = visual.find("geometry")
                mesh = geometry.find("mesh")
                if mesh is not None:
                    scale = parse_vector(mesh.attrib.get("scale"), (1, 1, 1)) * 1000.0
                    scale_matrix = np.eye(4)
                    scale_matrix[0, 0], scale_matrix[1, 1], scale_matrix[2, 2] = scale
                    output.append(actor_from_mesh(resolve_package_mesh(mesh.attrib["filename"]), visual_matrix @ scale_matrix, rgb))
                    continue
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                sphere = geometry.find("sphere")
                if box is not None:
                    values = tuple(float(value) for value in box.attrib["size"].split())
                    output.append(primitive_actor("box", values, visual_matrix, rgb))
                elif cylinder is not None:
                    values = (float(cylinder.attrib["radius"]), float(cylinder.attrib["length"]))
                    output.append(primitive_actor("cylinder", values, visual_matrix, rgb))
                elif sphere is not None:
                    output.append(primitive_actor("sphere", (float(sphere.attrib["radius"]),), visual_matrix, rgb))
        return output


def pose_positions(prefix: str, values_deg: list[float]) -> dict[str, float]:
    joints = [
        f"{prefix}_shoulder_pan",
        f"{prefix}_shoulder_lift",
        f"{prefix}_elbow_flex",
        f"{prefix}_wrist_flex",
        f"{prefix}_wrist_roll",
    ]
    output = {name: math.radians(value) for name, value in zip(joints, values_deg)}
    # 죠는 65 mm 병을 문 상태로 둔다. 인서트 포함 유효 개폐의 중간값이다.
    output[f"{prefix}_finger1_joint"] = 0.0325
    output[f"{prefix}_finger2_joint"] = 0.0325
    return output


def add_floor(renderer: vtk.vtkRenderer) -> None:
    plane = vtk.vtkPlaneSource()
    plane.SetOrigin(-420, -360, -1)
    plane.SetPoint1(500, -360, -1)
    plane.SetPoint2(-420, 360, -1)
    plane.SetXResolution(18)
    plane.SetYResolution(14)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(plane.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*color_tuple("#F8FAFC"))
    actor.GetProperty().SetEdgeColor(*color_tuple("#E6ECF4"))
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetLineWidth(1.0)
    renderer.AddActor(actor)


def object_actor(kind: str, center_chassis_mm: list[float], tilt_deg: float = 0.0) -> list[vtk.vtkActor]:
    center = np.asarray(center_chassis_mm, dtype=float)
    center[0] -= 90.0
    matrix = np.eye(4)
    matrix[:3, 3] = center
    if tilt_deg:
        matrix[:3, :3] = rpy_matrix(np.array([0.0, math.radians(tilt_deg), 0.0]))
    if kind == "bottle":
        body = primitive_actor("cylinder", (0.028, 0.155), matrix, color_tuple("#69A6FF"), 0.78)
        cap_matrix = matrix.copy()
        cap_matrix[:3, 3] += matrix[:3, :3] @ np.array([0.0, 0.0, 88.0])
        cap = primitive_actor("cylinder", (0.012, 0.020), cap_matrix, color_tuple(BLUE), 0.95)
        return [body, cap]
    return [primitive_actor("cylinder", (0.032, 0.080), matrix, color_tuple("#DDE6F3"), 0.9)]


def render_urdf(path: Path, positions: dict[str, float], *, view: str, task_objects: dict[str, tuple[list[float], float]] | None = None) -> None:
    scene = UrdfScene(URDF_PATH)
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(*color_tuple(WHITE))
    for actor in scene.actors(positions):
        renderer.AddActor(actor)
    add_floor(renderer)
    if task_objects:
        for kind, (center, tilt) in task_objects.items():
            for actor in object_actor(kind, center, tilt):
                renderer.AddActor(actor)

    light = vtk.vtkLight()
    light.SetLightTypeToSceneLight()
    light.SetPosition(650, -800, 1150)
    light.SetFocalPoint(-50, 0, 300)
    light.SetIntensity(0.95)
    renderer.AddLight(light)
    fill = vtk.vtkLight()
    fill.SetLightTypeToSceneLight()
    fill.SetPosition(-700, 500, 650)
    fill.SetFocalPoint(-50, 0, 300)
    fill.SetIntensity(0.55)
    renderer.AddLight(fill)

    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    if view == "iso":
        camera.SetPosition(1050, -1180, 880)
        camera.SetFocalPoint(-40, 0, 360)
        camera.SetViewUp(0, 0, 1)
        camera.SetParallelScale(520)
    elif view == "top":
        camera.SetPosition(-40, 0, 1450)
        camera.SetFocalPoint(-40, 0, 100)
        camera.SetViewUp(1, 0, 0)
        camera.SetParallelScale(390)
    elif view == "side":
        camera.SetPosition(-40, -1450, 380)
        camera.SetFocalPoint(-40, 0, 380)
        camera.SetViewUp(0, 0, 1)
        camera.SetParallelScale(500)
    else:
        raise ValueError(view)

    window = vtk.vtkRenderWindow()
    # VTK의 XOpenGL 빌드는 진짜 off-screen 버퍼에서 빈 프레임을 반환할 수 있다.
    # xvfb-run의 가상 디스플레이에서 일반 back buffer를 사용해 재현성 있게 캡처한다.
    window.SetOffScreenRendering(0)
    window.SetSize(920, 820)
    window.SetMultiSamples(8)
    window.AddRenderer(renderer)
    renderer.ResetCameraClippingRange()
    window.Render()

    image_filter = vtk.vtkWindowToImageFilter()
    image_filter.SetInput(window)
    image_filter.SetInputBufferTypeToRGB()
    image_filter.ReadFrontBufferOff()
    image_filter.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(path))
    writer.SetInputConnection(image_filter.GetOutputPort())
    writer.Write()
    window.Finalize()


def wrapped(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str, max_width: int, *, face: ImageFont.FreeTypeFont, fill: str, line_gap: int = 8) -> int:
    words = text.split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = word if not current else f"{current} {word}"
        if draw.textbbox((0, 0), candidate, font=face)[2] <= max_width:
            current = candidate
        else:
            lines.append(current)
            current = word
    if current:
        lines.append(current)
    x, y = xy
    line_height = face.size + line_gap
    for line in lines:
        draw.text((x, y), line, font=face, fill=fill)
        y += line_height
    return y


def card(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int], *, fill: str = WHITE, outline: str = LINE, radius: int = 20) -> None:
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=2)


def compose_model_status(output: Path, temporary: Path, spec: dict) -> None:
    transport = {}
    transport.update(pose_positions("left", spec["workspace"]["transport_joint_degrees"]["left"]))
    transport.update(pose_positions("right", spec["workspace"]["transport_joint_degrees"]["right"]))
    pour = {}
    pour.update(pose_positions("left", spec["workspace"]["pour_joint_degrees"]["left_bottle"]))
    pour.update(pose_positions("right", spec["workspace"]["pour_joint_degrees"]["right_cup"]))

    transport_png = temporary / "transport.png"
    pour_png = temporary / "pour.png"
    render_urdf(transport_png, transport, view="iso")
    render_urdf(pour_png, pour, view="iso")

    canvas = Image.new("RGB", (2000, 1280), WHITE)
    draw = ImageDraw.Draw(canvas)
    draw.text((80, 58), "HOLD THE FLOW · P0 DIGITAL MODEL", font=font(24, "bold"), fill=BLUE)
    draw.text((80, 112), "URDF P0에 적용한 운반·붓기 후보 자세", font=font(54, "bold"), fill=INK)
    draw.text((80, 190), "35 links · 34 joints · SO-101 ×2 · ggao50 평행그리퍼 · Astra S · LDS-03", font=font(25), fill=MUTED)

    panels = [(70, 270, 970, 1110), (1030, 270, 1930, 1110)]
    for box, image_path, label, detail in (
        (panels[0], transport_png, "01  운반 후보 · CLEARANCE FAIL", "관절 한계는 통과 · 링크/마스트 여유 미달"),
        (panels[1], pour_png, "02  붓기 후보 · LIMIT FAIL", "양쪽 elbow joint가 URDF hard limit 밖"),
    ):
        card(draw, box)
        source = Image.open(image_path).convert("RGB")
        source.thumbnail((860, 690), Image.Resampling.LANCZOS)
        canvas.paste(source, (box[0] + (900 - source.width) // 2, box[1] + 28))
        draw.text((box[0] + 38, box[3] - 118), label, font=font(28, "bold"), fill=INK)
        draw.text((box[0] + 38, box[3] - 70), detail, font=font(21), fill=MUTED)

    draw.rounded_rectangle((70, 1155, 1930, 1225), radius=18, fill="#FFF5EB")
    draw.text((100, 1175), "OPEN GATE  P0 형상·TF는 생성 완료 · TCP 프레임과 목표 자세 FK는 설계 동결 전 추가 검증", font=font(23, "medium"), fill=AMBER)
    canvas.save(output, quality=95)


def render_single_stl(path: Path, output: Path, rgb: tuple[float, float, float]) -> None:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    bounds = reader.GetOutput().GetBounds()
    center = np.array([(bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2, (bounds[4] + bounds[5]) / 2])
    extent = max(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4])

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*rgb)
    actor.GetProperty().SetInterpolationToPhong()
    actor.GetProperty().SetSpecular(0.12)
    actor.GetProperty().SetSpecularPower(18)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(1, 1, 1)
    renderer.AddActor(actor)
    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    camera.SetFocalPoint(*center)
    camera.SetPosition(*(center + np.array([1.4, -1.7, 1.15]) * max(extent, 20)))
    camera.SetViewUp(0, 0, 1)
    camera.SetParallelScale(max(extent * 0.72, 10))

    light = vtk.vtkLight()
    light.SetPosition(*(center + np.array([1.2, -1.5, 2.0]) * max(extent, 20)))
    light.SetFocalPoint(*center)
    light.SetIntensity(1.0)
    renderer.AddLight(light)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(0)
    window.SetSize(380, 270)
    window.SetMultiSamples(8)
    window.AddRenderer(renderer)
    renderer.ResetCameraClippingRange()
    window.Render()
    image_filter = vtk.vtkWindowToImageFilter()
    image_filter.SetInput(window)
    image_filter.SetInputBufferTypeToRGB()
    image_filter.ReadFrontBufferOff()
    image_filter.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(output))
    writer.SetInputConnection(image_filter.GetOutputPort())
    writer.Write()
    window.Finalize()


def compose_part_catalog(output: Path, temporary: Path, manifest: dict) -> None:
    name_ko = {
        "chassis_bottom": "하판",
        "chassis_middle": "중판",
        "chassis_top": "상판",
        "arm_adapter": "팔 어댑터",
        "camera_backing": "마스트 보강판",
        "camera_mast_segment": "마스트 세그먼트",
        "camera_mast_coupler": "마스트 이음관",
        "astra_cradle": "Astra 거치대",
        "lidar_riser": "LiDAR 받침대",
        "drive_side_carrier": "휠 측면 캐리어",
        "rear_caster_adapter": "후방 캐스터 어댑터",
        "rear_caster_shim_1mm": "캐스터 시임 1 mm",
        "rear_caster_shim_2mm": "캐스터 시임 2 mm",
        "rear_caster_shim_3mm": "캐스터 시임 3 mm",
    }
    canvas = Image.new("RGB", (2000, 2090), WHITE)
    draw = ImageDraw.Draw(canvas)
    draw.text((80, 58), "PRINT PACKAGE · K1 MAX", font=font(24, "bold"), fill=BLUE)
    draw.text((80, 112), "차체 CAD 14종, 원본 방향 그대로 출력", font=font(54, "bold"), fill=INK)
    draw.text((80, 190), "모든 형상은 300 × 300 × 300 mm 안에 수납 · 45° 기준 서포트 없음 · 회전 없음", font=font(25), fill=MUTED)

    columns, card_width, card_height = 4, 445, 405
    left, top, gap = 70, 275, 26
    colors = [color_tuple("#DCE8FF"), color_tuple("#D7DEE8"), color_tuple("#B9CDF8"), color_tuple("#EEF2F7")]
    for index, part in enumerate(manifest["parts"]):
        row, column = divmod(index, columns)
        x = left + column * (card_width + gap)
        y = top + row * (card_height + gap)
        card(draw, (x, y, x + card_width, y + card_height))
        thumbnail = temporary / f"part_{index:02d}.png"
        render_single_stl(REPO_ROOT / part["stl"], thumbnail, colors[index % len(colors)])
        image = Image.open(thumbnail).convert("RGB")
        canvas.paste(image, (x + 32, y + 24))
        draw.text((x + 28, y + 298), f"{index + 1:02d}  {name_ko[part['name']]}", font=font(22, "bold"), fill=INK)
        bounds = " × ".join(f"{value:.1f}" for value in part["bounds_mm"])
        draw.text((x + 28, y + 338), f"{bounds} mm  ·  {part['quantity']}개", font=font(17), fill=MUTED)
        draw.rounded_rectangle((x + 28, y + 370, x + 154, y + 396), radius=13, fill="#E7F6F0")
        draw.text((x + 42, y + 373), "SUPPORT OFF", font=font(14, "bold"), fill=GREEN)

    draw.rounded_rectangle((70, 1980, 1930, 2045), radius=18, fill="#FFF5EB")
    draw.text((100, 1997), "주의  ggao50 평행그리퍼 STL은 이 14종에 포함되지 않으며 별도 tree support 대상", font=font(20, "medium"), fill=AMBER)
    canvas.save(output, quality=95)


def compose_layout_reference(output: Path, temporary: Path, spec: dict) -> None:
    positions = {}
    positions.update(pose_positions("left", spec["workspace"]["transport_joint_degrees"]["left"]))
    positions.update(pose_positions("right", spec["workspace"]["transport_joint_degrees"]["right"]))
    top_png, side_png = temporary / "top.png", temporary / "side.png"
    render_urdf(top_png, positions, view="top")
    render_urdf(side_png, positions, view="side")

    canvas = Image.new("RGB", (2000, 1320), WHITE)
    draw = ImageDraw.Draw(canvas)
    draw.text((80, 58), "LAYOUT CONTROL · CHASSIS FRAME", font=font(24, "bold"), fill=BLUE)
    draw.text((80, 112), "치수와 좌표를 먼저 맞추고, 구멍은 실측 뒤 확정", font=font(54, "bold"), fill=INK)
    draw.text((80, 190), "+X 전방 · +Y 좌측 · +Z 위 · 원점은 250 mm 차체 중심의 바닥", font=font(25), fill=MUTED)

    views = [(70, 275, 1030, 1105, top_png, "TOP · 250 × 250 mm"), (1065, 275, 1930, 1105, side_png, "SIDE · CAMERA Z=800 mm")]
    for x1, y1, x2, y2, image_path, label in views:
        card(draw, (x1, y1, x2, y2))
        source = Image.open(image_path).convert("RGB")
        source.thumbnail((x2 - x1 - 50, y2 - y1 - 100), Image.Resampling.LANCZOS)
        canvas.paste(source, (x1 + (x2 - x1 - source.width) // 2, y1 + 35))
        draw.text((x1 + 32, y2 - 58), label, font=font(22, "bold"), fill=INK)

    facts = [
        ("ARM", "(25, ±70, 119) mm", BLUE),
        ("LIDAR", "(90, 0, 165) mm", BLUE),
        ("CAMERA", "(-80, 0, 800) mm", BLUE),
        ("WHEEL", "X=+90 · Y=±135 mm", INK),
        ("CASTER", "(-105, 0, 0) mm", INK),
    ]
    x = 70
    for label, value, accent in facts:
        width = 350 if label != "WHEEL" else 400
        draw.rounded_rectangle((x, 1150, x + width, 1260), radius=18, fill="#F8FAFC", outline=LINE, width=2)
        draw.text((x + 22, 1167), label, font=font(15, "bold"), fill=accent)
        draw.text((x + 22, 1205), value, font=font(18, "medium"), fill=INK)
        x += width + 18
    canvas.save(output, quality=95)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    with tempfile.TemporaryDirectory(prefix="hold-flow-render-") as directory:
        temporary = Path(directory)
        compose_model_status(output / "01_p0_model_status.png", temporary, spec)
        compose_part_catalog(output / "02_print_part_catalog.png", temporary, manifest)
        compose_layout_reference(output / "03_layout_reference.png", temporary, spec)
    print(f"generated: {output}")


if __name__ == "__main__":
    main()
