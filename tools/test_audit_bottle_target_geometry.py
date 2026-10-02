import struct

import numpy as np
import pytest

from audit_bottle_target_geometry import binary_stl_vertices, cylinder_intrusion


def test_cylinder_is_not_an_aabb_and_boundary_contact_is_not_intrusion():
    points = [[.02, .02, 0], [.029, .029, 0], [.03, 0, 0], [0, 0, .101], [0, 0, 0]]
    assert cylinder_intrusion(points, [0, 0, 0], .03, .2).tolist() == [True, False, False, False, True]


@pytest.mark.parametrize("radius,height,margin", [(0, .2, .001), (.03, -.1, 0), (.03, .2, .04), (.03, .2, -1)])
def test_bad_dimensions_rejected(radius, height, margin):
    with pytest.raises(ValueError):
        cylinder_intrusion([[0, 0, 0]], [0, 0, 0], radius, height, margin)


def test_binary_stl_and_truncation(tmp_path):
    path = tmp_path / "one.stl"
    data = bytes(80)+struct.pack("<I", 1)+struct.pack("<12fH", 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0)
    path.write_bytes(data)
    assert binary_stl_vertices(path).shape == (3, 3)
    path.write_bytes(data[:-1])
    with pytest.raises(ValueError):
        binary_stl_vertices(path)


def test_nonfinite_points_rejected():
    with pytest.raises(ValueError):
        cylinder_intrusion([[np.nan, 0, 0]], [0, 0, 0], .03, .2)
