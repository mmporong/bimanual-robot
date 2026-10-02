#!/usr/bin/env python3
"""현행 URDF의 왼팔 영점 자세를 실물 정렬 참고용으로 렌더한다."""
from pathlib import Path
import argparse
import sys

import numpy as np
import vtk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "design/cad"))
from render_design_handoff import UrdfScene, URDF_PATH, FONT_REGULAR


def render(output: Path, comparisons=None) -> None:
    if output.exists():
        raise FileExistsError(output)
    scene = UrdfScene(URDF_PATH)
    transforms = scene.link_transforms({})
    base = transforms["left_base_link"][:3, 3]
    scene.links = {name: link for name, link in scene.links.items() if name.startswith("left_")
                   and name not in {"left_wheel_link", "left_arm_backing_link"}}
    window = vtk.vtkRenderWindow()
    # XOpenGL 빌드에서는 xvfb-run의 가상 디스플레이 back buffer를 사용한다.
    window.SetOffScreenRendering(0)
    window.SetSize(1400, 650)
    window.SetMultiSamples(4)
    views = comparisons or [
        ("옆면 · 5개 팔 관절 q=0°", {}, np.array([180, -1200, 150])),
        ("입체 · 같은 자세", {}, np.array([850, -950, 600])),
    ]
    if len(views) != 2:
        raise ValueError("두 비교 화면이 필요합니다")
    for index, (title, positions, camera_offset) in enumerate(views):
        renderer = vtk.vtkRenderer()
        renderer.SetViewport(index / 2, 0, (index + 1) / 2, 1)
        renderer.SetBackground(0.96, 0.97, 0.98)
        for actor in scene.actors({"left_gripper": 0.4, **positions}):
            renderer.AddActor(actor)
        label = vtk.vtkTextActor()
        label.SetInput(title + "\n자동 이동 명령 아님 · 그리퍼 개구는 기준에서 제외")
        label.SetPosition(20, 575)
        prop = label.GetTextProperty()
        prop.SetFontFamily(vtk.VTK_FONT_FILE)
        prop.SetFontFile(str(FONT_REGULAR))
        prop.SetFontSize(19)
        prop.SetColor(0.08, 0.12, 0.18)
        renderer.AddActor2D(label)
        camera = renderer.GetActiveCamera()
        camera.SetPosition(*(base + camera_offset))
        camera.SetFocalPoint(*(base + [195, 0, 125]))
        camera.SetViewUp(0, 0, 1)
        camera.ParallelProjectionOn()
        camera.SetParallelScale(220)
        window.AddRenderer(renderer)
        renderer.ResetCameraClippingRange()
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        window.Render()
        capture = vtk.vtkWindowToImageFilter()
        capture.SetInput(window)
        capture.SetInputBufferTypeToRGB()
        capture.ReadFrontBufferOff()
        capture.Update()
        writer = vtk.vtkPNGWriter()
        writer.SetFileName(str(output))
        writer.SetInputConnection(capture.GetOutputPort())
        writer.Write()
    finally:
        window.Finalize()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    render(parser.parse_args().output)
