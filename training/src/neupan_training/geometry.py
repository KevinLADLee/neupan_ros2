"""Body-frame convex footprints, with the same edge order as neupan_core."""

import numpy as np


def polygon_gh(vertices):
    """Convert ordered [x, y] pairs to Gx <= h; accept CW or CCW input."""
    v = np.asarray(vertices, dtype=np.float64)
    if v.ndim != 2 or v.shape[1] != 2 or len(v) < 3:
        raise ValueError("robot.vertices must contain >= 3 [x, y] pairs")
    if not np.isfinite(v).all():
        raise ValueError("robot.vertices must be finite")
    scale = np.ptp(v, axis=0).max()
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("degenerate polygon")
    local = (v - v[0]) / scale
    cross = lambda a, b: a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]
    area = cross(local, np.roll(local, -1, axis=0)).sum()
    if abs(area) <= 1e-12:
        raise ValueError("polygon has zero area")
    for i, point in enumerate(local):
        if np.any(np.linalg.norm(local[i + 1:] - point, axis=1) <= 1e-12):
            raise ValueError("repeated polygon vertex")
        edge = local[(i + 1) % len(v)] - point
        if np.any(np.sign(area) * cross(edge, local - point) < -1e-12):
            raise ValueError("polygon is not convex and ordered")
    if area < 0:
        v = np.concatenate((v[:1], v[:0:-1]))
    edges = np.roll(v, -1, axis=0) - v
    g = np.column_stack((edges[:, 1], -edges[:, 0]))
    h = np.sum(g * v, axis=1, keepdims=True)
    return g, h


def rectangle_gh(length: float, width: float, wheelbase: float = 0.0):
    """G, h matching Robot::diffRectangle, including the axle-frame offset."""
    if not np.isfinite([length, width, wheelbase]).all() or length <= 0 or width <= 0:
        raise ValueError("Robot dimensions must be finite, with positive length and width")
    sx, sy = -(length - wheelbase) / 2.0, -width / 2.0
    return polygon_gh([[sx, sy], [sx + length, sy],
                       [sx + length, sy + width], [sx, sy + width]])


def normalize_robot(robot):
    """Keep only the active geometry; vertices take precedence as in NeuPAN."""
    if robot.get("vertices") is not None:
        polygon_gh(robot["vertices"])
        return {"vertices": np.asarray(robot["vertices"], dtype=float).tolist()}
    if "length" not in robot or "width" not in robot:
        raise ValueError("Robot vertices or length and width are required")
    result = {key: float(robot.get(key, 0.0)) for key in ("length", "width", "wheelbase")}
    rectangle_gh(**result)
    return result


def robot_gh(robot):
    geometry = normalize_robot(robot)
    if "vertices" in geometry:
        return polygon_gh(geometry["vertices"])
    return rectangle_gh(**geometry)
