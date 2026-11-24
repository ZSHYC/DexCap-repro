"""Physical MuJoCo scene and metrics for DexCap Ball Collecting.

The paper gives the two 40 x 30 cm placement regions but no CAD or inertial
measurements. The ball uses the midpoint of the ITF Type 1/2 ranges
(65.4--68.6 mm, 56.0--59.4 g). The open 230 x 150 x 130 mm basket is inferred
from Fig. 5(b), using the visible ball as scale; exact dimensions and material
are therefore assumptions.
"""

from __future__ import annotations

import itertools

import mujoco
import numpy as np


TABLE_HEIGHT = -0.030
DEMO_OBJECT_XY = np.array([[0.54, 0.13], [0.54, -0.15]])
PLACEMENT_HALF_SIZE = np.array([0.20, 0.15])
BALL_RADIUS = 0.0335
BALL_MASS = 0.0577
BALL_LIFT_HEIGHT = 0.03
BASKET_HALF_X = 0.115
BASKET_HALF_Y = 0.075
BASKET_HEIGHT = 0.130
BASKET_WIRE_RADIUS = 0.003
BASKET_MASS = 0.25

SUMMARY_KEYS = (
    "right_ball_contacts", "ball_height_m", "ball_lifted", "ball_released",
    "ball_in_basket", "basket_displacement_m", "sim_success",
)


def sample_object_xy(rng):
    """Sample basket then ball in the paper's separate 40 x 30 cm regions."""
    while True:
        xy = DEMO_OBJECT_XY + rng.uniform(
            -PLACEMENT_HALF_SIZE, PLACEMENT_HALF_SIZE, size=(2, 2)
        )
        delta = np.abs(xy[1] - xy[0])
        clearance = np.array([BASKET_HALF_X, BASKET_HALF_Y]) + BALL_RADIUS
        if np.any(delta >= clearance):
            return xy


def _wire_quat(start, end):
    """Return a wxyz quaternion aligning a cylinder's +z axis to a segment."""
    direction = np.asarray(end, dtype=float) - np.asarray(start, dtype=float)
    direction /= np.linalg.norm(direction)
    quat = np.array([1.0 + direction[2], -direction[1], direction[0], 0.0])
    return quat / np.linalg.norm(quat)


def _basket_segments():
    hx, hy, height = BASKET_HALF_X, BASKET_HALF_Y, BASKET_HEIGHT
    segments = []
    # A crossed 30--33 mm bottom grid supports a 67 mm ball without a solid tray.
    for index, y in enumerate(np.linspace(-hy, hy, 6)):
        segments.append((f"bottom_x_{index}", (-hx, y, 0), (hx, y, 0)))
    for index, x in enumerate(np.linspace(-hx, hx, 8)):
        segments.append((f"bottom_y_{index}", (x, -hy, 0), (x, hy, 0)))
    for level, z in enumerate(np.linspace(0, height, 5)):
        segments.extend((
            (f"wall_x_neg_{level}", (-hx, -hy, z), (hx, -hy, z)),
            (f"wall_x_pos_{level}", (-hx, hy, z), (hx, hy, z)),
            (f"wall_y_neg_{level}", (-hx, -hy, z), (-hx, hy, z)),
            (f"wall_y_pos_{level}", (hx, -hy, z), (hx, hy, z)),
        ))
    for index, x in enumerate(np.linspace(-hx, hx, 5)):
        segments.extend((
            (f"post_y_neg_{index}", (x, -hy, 0), (x, -hy, height)),
            (f"post_y_pos_{index}", (x, hy, 0), (x, hy, height)),
        ))
    for index, y in enumerate(np.linspace(-hy, hy, 5)[1:-1], start=1):
        segments.extend((
            (f"post_x_neg_{index}", (-hx, y, 0), (-hx, y, height)),
            (f"post_x_pos_{index}", (hx, y, 0), (hx, y, height)),
        ))
    return segments


def add_ball_collecting_scene(spec: mujoco.MjSpec, object_xy=DEMO_OBJECT_XY):
    """Add a freely moving wire basket and tennis-like free ball to ``spec``."""
    positions = np.asarray(object_xy, dtype=float)
    if positions.shape != (2, 2) or not np.isfinite(positions).all():
        raise ValueError("object_xy must contain finite basket and ball XY positions")
    basket_xy, ball_xy = positions
    basket = spec.worldbody.add_body(
        name="ball_basket", pos=[*basket_xy, TABLE_HEIGHT + BASKET_WIRE_RADIUS]
    )
    basket.add_freejoint(name="ball_basket_free")
    segments = _basket_segments()
    total_volume = sum(
        np.pi * BASKET_WIRE_RADIUS**2 * np.linalg.norm(np.subtract(end, start))
        for _, start, end in segments
    )
    density = BASKET_MASS / total_volume
    for name, start, end in segments:
        start, end = np.asarray(start, float), np.asarray(end, float)
        basket.add_geom(
            name=f"ball_basket_{name}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            pos=(start + end) / 2, quat=_wire_quat(start, end),
            size=[BASKET_WIRE_RADIUS, np.linalg.norm(end - start) / 2],
            rgba=[0.035, 0.035, 0.040, 1], density=density,
            friction=[0.75, 0.01, 0.001], condim=4,
        )
    ball = spec.worldbody.add_body(
        name="collecting_ball", pos=[*ball_xy, TABLE_HEIGHT + BALL_RADIUS]
    )
    ball.add_freejoint(name="collecting_ball_free")
    ball.add_geom(
        name="collecting_ball_geom", type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[BALL_RADIUS, 0, 0], mass=BALL_MASS,
        rgba=[0.73, 0.92, 0.08, 1], friction=[0.80, 0.01, 0.001],
        solref=[0.006, 0.8], solimp=[0.90, 0.98, 0.002, 0.5, 2], condim=4,
    )


class BallCollectingMetrics:
    """Measure physical lift-and-release completion without changing simulation."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.ball_body = model.body("collecting_ball").id
        self.ball_joint = model.joint("collecting_ball_free").id
        self.ball_geom = model.geom("collecting_ball_geom").id
        self.basket_body = model.body("ball_basket").id
        self.basket_joint = model.joint("ball_basket_free").id
        self.table_geom = model.geom("table").id
        self.basket_geoms = {
            g for g in range(model.ngeom) if model.geom(g).name.startswith("ball_basket_")
        }
        self.bottom_geoms = {
            g for g in self.basket_geoms
            if model.geom(g).name.startswith("ball_basket_bottom_")
        }
        self.right_hand_geoms = {
            g for g in range(model.ngeom) if model.geom(g).name.startswith("right_hand_")
        }
        ball_qpos = model.jnt_qposadr[self.ball_joint]
        basket_qpos = model.jnt_qposadr[self.basket_joint]
        self.initial_ball_z = float(model.qpos0[ball_qpos + 2])
        self.initial_basket_xy = np.asarray(model.qpos0[basket_qpos:basket_qpos + 2]).copy()
        self.right_ball_contacts = 0
        self.ball_lifted = False
        self.ball_collected = False

    def _contacts(self, data, first, second):
        contacts = []
        for index in range(data.ncon):
            contact = data.contact[index]
            if contact.geom1 in first and contact.geom2 in second:
                contacts.append((contact, np.asarray(contact.frame[:3])))
            elif contact.geom2 in first and contact.geom1 in second:
                contacts.append((contact, -np.asarray(contact.frame[:3])))
        return contacts

    @staticmethod
    def _opposed_grip(contacts):
        normals = [normal for _, normal in contacts]
        return len(normals) >= 2 and any(
            np.dot(a, b) < -0.25 for a, b in itertools.combinations(normals, 2)
        )

    def _ball_in_basket(self, data):
        rotation = data.xmat[self.basket_body].reshape(3, 3)
        local = rotation.T @ (data.xpos[self.ball_body] - data.xpos[self.basket_body])
        inside_xy = (
            abs(local[0]) <= BASKET_HALF_X - BASKET_WIRE_RADIUS - BALL_RADIUS
            and abs(local[1]) <= BASKET_HALF_Y - BASKET_WIRE_RADIUS - BALL_RADIUS
        )
        inside_z = (
            BALL_RADIUS - BASKET_WIRE_RADIUS <= local[2]
            <= BASKET_HEIGHT - BALL_RADIUS + BASKET_WIRE_RADIUS
        )
        supported = bool(self._contacts(data, {self.ball_geom}, self.bottom_geoms))
        dof = self.model.jnt_dofadr[self.ball_joint]
        settled = np.linalg.norm(data.qvel[dof:dof + 6]) < 0.08
        return bool(inside_xy and inside_z and supported and settled)

    def update(self, data: mujoco.MjData) -> dict:
        right_contacts = self._contacts(data, self.right_hand_geoms, {self.ball_geom})
        self.right_ball_contacts += len(right_contacts)
        table_supported = bool(self._contacts(data, {self.ball_geom}, {self.table_geom}))
        gripping = self._opposed_grip(right_contacts)
        lifted_now = (
            data.xpos[self.ball_body, 2] >= self.initial_ball_z + BALL_LIFT_HEIGHT
            and gripping and not table_supported
        )
        self.ball_lifted |= lifted_now
        ball_released = self.ball_lifted and not right_contacts
        ball_in_basket = self._ball_in_basket(data)
        self.ball_collected |= self.ball_lifted and ball_released and ball_in_basket
        basket_xy = data.xpos[self.basket_body, :2]
        return {
            "right_ball_contacts": self.right_ball_contacts,
            "ball_height_m": float(data.xpos[self.ball_body, 2] - TABLE_HEIGHT),
            "ball_lifted": bool(self.ball_lifted),
            "ball_released": bool(ball_released),
            "ball_in_basket": ball_in_basket,
            "basket_displacement_m": float(np.linalg.norm(basket_xy - self.initial_basket_xy)),
            "sim_success": bool(self.ball_collected),
        }
