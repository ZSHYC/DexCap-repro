"""Compound rigid approximations of the objects in the author's Packaging videos."""

from typing import NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation


class Part(NamedTuple):
    kind: str
    half_size: tuple
    center: tuple
    color: tuple
    orientation: tuple = (0, 0, 0, 1)


def box(size, center, color):
    return Part("box", tuple(length / 2 for length in size), center, color)


def cylinder(radius, height, center, color, orientation=(0, 0, 0, 1)):
    return Part("cylinder", (radius, radius, height / 2), center, color, orientation)


def tube(start, end, radius, color):
    direction = np.subtract(end, start)
    height = np.linalg.norm(direction)
    rotation, _ = Rotation.align_vectors(direction / height, [0, 0, 1])
    return cylinder(radius, height, tuple(np.add(start, end) / 2), color, tuple(rotation.as_quat()))


def ring(outer, inner, height, center, color):
    sides = 10
    radius = (outer + inner) / 2
    tangent = radius * np.tan(np.pi / sides)
    half_size = ((outer - inner) / 2, tangent, height / 2)
    return tuple(
        Part("box", half_size, tuple(np.add(center, [radius * np.cos(yaw), radius * np.sin(yaw), 0])),
             color, tuple(Rotation.from_euler("z", yaw).as_quat()))
        for yaw in np.arange(sides) * 2 * np.pi / sides
    )


VIDEO_1 = "normal_packaging_unseenobj.mp4 @ 0.6 s"
VIDEO_2 = "normal_packaging_unseenobj2.mp4 @ 0.6 s"
BLUE = (0.03, 0.09, 0.67, 1)
RED = (0.80, 0.07, 0.035, 1)
APPLE_RED = (0.68, 0.035, 0.035, 1)
YELLOW = (0.98, 0.70, 0.04, 1)
TAN = (0.70, 0.41, 0.17, 1)
GIRAFFE = (0.78, 0.49, 0.25, 1)
DARK = (0.22, 0.12, 0.06, 1)
BLACK = (0.04, 0.04, 0.04, 1)
ORANGE = (0.90, 0.25, 0.05, 1)

DEMO_OBJECT_XY = np.array([[0.482, 0.115], [0.517, -0.223]])
PACKAGING_OBJECTS = {
    "red-box": ("box", (0.061, 0.017, 0.047), (0.60, 0.59, 0.55, 1), "demo_0"),
    "blue-block": ("box", (0.055, 0.0275, 0.018), (0.04, 0.45, 0.72, 1), "demo_50"),
    # The pear rests with its shortest principal radius upright.
    "green-pear": ("ellipsoid", (0.029, 0.027, 0.025), (0.31, 0.47, 0.09, 1), "demo_100"),
    "green-can": ("cylinder", (0.0405, 0.0405, 0.034), (0.17, 0.45, 0.12, 1), "demo_150"),
    "tape-measure": ("cylinder", (0.038, 0.038, 0.017), (0.085, 0.09, 0.072, 1), "demo_175"),
    "yellow-cloth": ("box", (0.075, 0.065, 0.020), (0.91, 0.75, 0.09, 1), "demo_250"),
}
UNSEEN_PRIMITIVES = {
    "earplug-jar": ("cylinder", (0.035, 0.035, 0.059), (0.72, 0.70, 0.64, 1), VIDEO_1),
    "golden-curry": ("box", (0.09, 0.015, 0.0575), (0.67, 0.39, 0.10, 1), VIDEO_1),
}


UNSEEN_COMPOUNDS = {
    "red-apple": (VIDEO_1, (
        # Recess the curved underside above a small flat footprint so the apple stands upright.
        Part("ellipsoid", (0.0395, 0.0395, 0.0427), (0, 0, 0.0008), APPLE_RED),
        cylinder(0.0106, 0.0036, (0, 0, -0.0417), APPLE_RED),
    )),
    "tape-dispenser": (VIDEO_1, (
        *ring(0.0525, 0.031, 0.048, (-0.013, 0, 0), (0.79, 0.76, 0.66, 1)),
        box((0.042, 0.080, 0.060), (0.057, 0, 0), RED),
        box((0.042, 0.024, 0.008), (0.034, -0.033, 0.024), RED),
        box((0.042, 0.024, 0.008), (0.034, 0.033, -0.024), RED),
    )),
    "stacked-bricks": (VIDEO_2, (
        cylinder(0.012, 0.015, (-0.081, -0.015, 0), YELLOW,
                 tuple(Rotation.from_euler("y", np.pi / 2).as_quat())),
        box((0.046, 0.061, 0.047), (-0.053, 0, 0), YELLOW),
        box((0.040, 0.061, 0.047), (-0.010, 0, 0), RED),
        box((0.046, 0.061, 0.047), (0.033, 0, 0), YELLOW),
        box((0.033, 0.065, 0.055), (0.0725, 0, -0.004), YELLOW),
    )),
    "blue-mug": (VIDEO_2, (
        cylinder(0.0475, 0.006, (0, 0, -0.052), BLUE),
        *ring(0.0475, 0.0425, 0.104, (0, 0, 0.003), BLUE),
        tube((0.044, 0, 0.031), (0.071, 0, 0.025), 0.005, BLUE),
        tube((0.071, 0, 0.025), (0.081, 0, 0.006), 0.005, BLUE),
        tube((0.081, 0, 0.006), (0.075, 0, -0.018), 0.005, BLUE),
        tube((0.075, 0, -0.018), (0.045, 0, -0.030), 0.005, BLUE),
    )),
    "giraffe": (VIDEO_2, (
        box((0.060, 0.033, 0.042), (-0.023, 0, 0.005), GIRAFFE),
        tube((-0.005, 0, 0.013), (0.044, 0, 0.078), 0.013, GIRAFFE),
        box((0.027, 0.024, 0.020), (0.051, 0, 0.081), GIRAFFE),
        box((0.005, 0.005, 0.011), (0.044, -0.006, 0.094), DARK),
        box((0.005, 0.005, 0.011), (0.044, 0.006, 0.094), DARK),
        *(
            tube((hip, side, -0.008), (foot, side, -0.060), 0.0055, GIRAFFE)
            for hip, foot in ((-0.043, -0.049), (-0.003, 0.009))
            for side in (-0.012, 0.012)
        ),
        *(
            box((0.013, 0.011, 0.008), (foot, side, -0.062), DARK)
            for foot in (-0.049, 0.009) for side in (-0.012, 0.012)
        ),
        tube((-0.049, 0, 0.010), (-0.058, 0, -0.027), 0.004, DARK),
    )),
    "lion": (VIDEO_2, (
        box((0.076, 0.034, 0.041), (-0.010, 0, 0.003), TAN),
        cylinder(0.023, 0.044, (0.031, 0, 0.008), DARK),
        box((0.022, 0.027, 0.024), (0.047, 0, 0.008), TAN),
        *(
            tube((hip, side, -0.004), (foot, side, -0.035), 0.007, TAN)
            for hip, foot in ((-0.039, -0.042), (0.019, 0.025))
            for side in (-0.013, 0.013)
        ),
        tube((-0.045, 0, 0.006), (-0.063, 0, -0.004), 0.005, TAN),
        tube((-0.063, 0, -0.004), (-0.073, -0.005, -0.009), 0.004, TAN),
        box((0.013, 0.009, 0.010), (-0.075, -0.005, -0.009), DARK),
    )),
    "spring-clamp": ("DexCap Fig. 10, comparable 6.5-inch spring clamp", (
        cylinder(0.013, 0.024, (-0.018, 0, 0), BLACK),
        *(
            part
            for side in (-1, 1)
            for part in (
                tube((-0.072, side * 0.033, 0), (-0.023, side * 0.006, 0), 0.009, BLACK),
                tube((-0.023, side * 0.006, 0), (0.038, side * 0.017, 0), 0.010, BLACK),
                tube((0.038, side * 0.017, 0), (0.070, side * 0.006, 0), 0.008, BLACK),
                box((0.026, 0.010, 0.022), (0.070, side * 0.005, 0), ORANGE),
            )
        ),
    )),
}
PACKAGING_UNSEEN_OBJECTS = (*UNSEEN_PRIMITIVES, *UNSEEN_COMPOUNDS)


def sample_object_xy(rng):
    """Sample the published 10 x 10 cm box and 40 x 30 cm object areas."""
    half_area = np.array([[0.05, 0.05], [0.20, 0.15]])
    return DEMO_OBJECT_XY + rng.uniform(-half_area, half_area)


def cylinder_extent(rotation, half_size):
    axis = np.abs(rotation[:, 2])
    return half_size[0] * np.sqrt(np.maximum(0, 1 - axis**2)) + half_size[2] * axis


def compound_bounds(parts, rotation):
    lower, upper = [], []
    for part in parts:
        axes = rotation @ Rotation.from_quat(part.orientation).as_matrix()
        center = rotation @ part.center
        if part.kind == "cylinder":
            extent = cylinder_extent(axes, part.half_size)
        else:
            extent = np.abs(axes) @ part.half_size
        lower.append(center - extent)
        upper.append(center + extent)
    return np.min(lower, axis=0), np.max(upper, axis=0)
