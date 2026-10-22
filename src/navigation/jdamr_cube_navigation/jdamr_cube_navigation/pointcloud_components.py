"""Split organized XYZ samples without averaging precision measurements."""

import math

import numpy as np


def organized_components(points, rows, columns, image_shape, pixel_step,
                         *, distance_m=0.05, minimum_points=120):
    """Return original indices connected by short 4-neighbor metric edges.

    Connectivity follows the sampled image, not all Euclidean neighbors. Missing
    pixels therefore do not bridge an occlusion. No centroid or voxel position
    replaces the original XYZ points used for a precision surface fit.
    """
    points = np.asarray(points, dtype=np.float64)
    rows, columns = np.asarray(rows), np.asarray(columns)
    if (points.ndim != 2 or points.shape[1] != 3
            or not np.all(np.isfinite(points))):
        raise ValueError('points must be finite Nx3 XYZ')
    if (rows.shape != (len(points),) or columns.shape != rows.shape
            or rows.dtype.kind not in 'iu' or columns.dtype.kind not in 'iu'):
        raise ValueError('pixel indices must be integer vectors matching XYZ')
    if (isinstance(pixel_step, bool) or not isinstance(pixel_step, int)
            or pixel_step <= 0 or isinstance(minimum_points, bool)
            or not isinstance(minimum_points, int) or minimum_points <= 0):
        raise ValueError('step and minimum_points must be positive integers')
    if (len(image_shape) != 2 or any(
            isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v <= 0
            for v in image_shape)):
        raise ValueError('image_shape must contain positive height and width')
    if (isinstance(distance_m, bool) or not math.isfinite(distance_m)
            or distance_m <= 0.0):
        raise ValueError('component distance must be finite and positive')
    height, width = image_shape
    if (np.any(rows >= height) or np.any(columns >= width)
            or np.any(rows < 0) or np.any(columns < 0)
            or np.any(rows % pixel_step) or np.any(columns % pixel_step)):
        raise ValueError('pixel indices must lie on the sampled image grid')
    grid = np.full(((height - 1) // pixel_step + 1,
                    (width - 1) // pixel_step + 1), -1, dtype=np.int64)
    rr, cc = rows // pixel_step, columns // pixel_step
    if len(np.unique(rr * grid.shape[1] + cc)) != len(points):
        raise ValueError('duplicate pixel coordinates')
    grid[rr, cc] = np.arange(len(points))
    parent = np.arange(len(points))
    sizes = np.ones(len(points), dtype=np.int64)

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for first, second in ((grid[:, :-1], grid[:, 1:]),
                          (grid[:-1, :], grid[1:, :])):
        valid = (first >= 0) & (second >= 0)
        left, right = first[valid], second[valid]
        difference = points[left] - points[right]
        close = np.einsum('ij,ij->i', difference, difference) <= distance_m ** 2
        for a, b in zip(left[close], right[close]):
            a, b = root(a), root(b)
            if a != b:
                if sizes[a] < sizes[b]:
                    a, b = b, a
                parent[b] = a
                sizes[a] += sizes[b]
    groups = {}
    for index in range(len(points)):
        groups.setdefault(root(index), []).append(index)
    components = [np.asarray(indices, dtype=np.int64)
                  for indices in groups.values() if len(indices) >= minimum_points]
    return tuple(sorted(components, key=lambda group: (-len(group), int(group[0]))))
