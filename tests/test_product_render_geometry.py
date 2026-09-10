from __future__ import annotations

import json
import struct
import sys
import unittest
from pathlib import Path

from matplotlib import image as mpl_image
import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
RAISED_FRAME_SPEC = REPO_ROOT / "design/mechanical/raised_frame_concept_20260909.yaml"
GEOMETRY_CONTRACT = REPO_ROOT / "docs/assets/product_candidates_20260909/geometry_contract.json"
A_RENDER = REPO_ROOT / "docs/assets/product_candidates_20260909/Real_scale/A_real_scale.png"
sys.path.insert(0, str(REPO_ROOT / "design/mechanical"))

import render_product_candidates as render  # noqa: E402


class ProductRenderGeometryTest(unittest.TestCase):
    def test_a_brace_dimensions_match_design_source_and_contract(self) -> None:
        source = yaml.safe_load(RAISED_FRAME_SPEC.read_text(encoding="utf-8"))
        expected_mm = [
            source["materials"]["strap_width"] * 1000,
            source["materials"]["strap_thickness"] * 1000,
        ]
        contract = json.loads(GEOMETRY_CONTRACT.read_text(encoding="utf-8"))

        self.assertEqual([render.A_BRACE_WIDTH, render.A_BRACE_THICKNESS], expected_mm)
        self.assertEqual(contract["a_brace_flat_bar"], expected_mm)

    def test_a_brace_flat_bar_preserves_documented_cross_section(self) -> None:
        start = np.array((-130.0, 140.0, 85.0))
        end = np.array((130.0, 140.0, 693.0))
        faces = render.flat_strap_faces(start, end, plane_normal=(0, 1, 0))
        vertices = np.concatenate(faces)

        direction = (end - start) / np.linalg.norm(end - start)
        normal = np.array((0.0, 1.0, 0.0))
        across = np.cross(normal, direction)

        self.assertAlmostEqual(np.ptp(vertices @ direction), np.linalg.norm(end - start), places=9)
        self.assertAlmostEqual(np.ptp(vertices @ across), render.A_BRACE_WIDTH, places=9)
        self.assertAlmostEqual(np.ptp(vertices @ normal), render.A_BRACE_THICKNESS, places=9)

    def test_a_braces_touch_profile_outer_faces_without_intersection(self) -> None:
        cases = (
            ((-130, 140, 85), (130, 140, 693), (0, 1, 0), 1, 140.0),
            ((-130, -140, 85), (130, -140, 693), (0, -1, 0), 1, -140.0),
            ((-140, -130, 85), (-140, 130, 693), (-1, 0, 0), 0, -140.0),
        )
        for start, end, outward, axis, surface in cases:
            with self.subTest(outward=outward):
                vertices = np.concatenate(
                    render.mounted_flat_strap_faces(start, end, outward)
                )
                if outward[axis] > 0:
                    self.assertAlmostEqual(vertices[:, axis].min(), surface, places=9)
                else:
                    self.assertAlmostEqual(vertices[:, axis].max(), surface, places=9)

    def test_a_brace_rejects_invalid_geometry(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be positive"):
            render.flat_strap_faces((0, 0, 0), (1, 0, 0), width=0)
        with self.assertRaisesRegex(ValueError, "endpoints must differ"):
            render.flat_strap_faces((0, 0, 0), (0, 0, 0))
        with self.assertRaisesRegex(ValueError, "must lie in its mounting plane"):
            render.flat_strap_faces((0, 0, 0), (0, 1, 0), plane_normal=(0, 1, 0))

    def test_a_overview_has_documented_canvas(self) -> None:
        header = A_RENDER.read_bytes()[:24]
        self.assertEqual(header[:8], b"\x89PNG\r\n\x1a\n")
        self.assertEqual(struct.unpack(">II", header[16:24]), (1900, 1500))

    def test_a_overview_contains_all_three_visual_panels(self) -> None:
        pixels = mpl_image.imread(A_RENDER)[..., :3]
        height, width, _ = pixels.shape
        panels = {
            "assembly": pixels[int(height * 0.10):int(height * 0.90), int(width * 0.12):int(width * 0.60)],
            "front": pixels[int(height * 0.04):int(height * 0.48), int(width * 0.67):int(width * 0.98)],
            "side": pixels[int(height * 0.54):int(height * 0.98), int(width * 0.67):int(width * 0.98)],
        }
        for name, panel in panels.items():
            with self.subTest(panel=name):
                non_background = np.any(panel < 0.90, axis=2)
                self.assertGreater(np.mean(non_background), 0.02)


if __name__ == "__main__":
    unittest.main()
