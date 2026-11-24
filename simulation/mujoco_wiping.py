"""MuJoCo objects and physical metrics for DexCap plate wiping."""

from itertools import combinations
from pathlib import Path

import mujoco
import numpy as np

from simulation.mujoco_sponge import SPONGE_HALF_XY, sponge_flexcomp, texture_sponge


DEMO_OBJECT_XY = np.array([[0.5509, -0.1559], [0.542, 0.112]])
TABLE_HEIGHT = -0.03
PLATE_RADIUS = 0.134
PLATE_INNER_RADIUS = 0.09
PLATE_BASE_THICKNESS = 0.002
# Top profile from RGB-D frames 2220 and 2295, relative to the base center.
PLATE_PROFILE = (
    (PLATE_INNER_RADIUS, PLATE_BASE_THICKNESS / 2),
    (0.100, 0.0105),
    (0.105, 0.0190),
    (0.115, 0.0222),
    (0.125, 0.0223),
    (PLATE_RADIUS, 0.0181),
)
PLATE_CONTACT_TOLERANCE = 0.001
MIN_WIPE_STROKE_M = 0.06

SUMMARY_KEYS = (
    "sponge_plate_contacts",
    "contact_force",
    "sponge_plate_contact_steps",
    "wipe_surface_contact_steps",
    "peak_contact_force",
    "wipe_distance",
    "wipe_passes",
    "plate_grasped",
    "sponge_grasped",
    "sim_success",
)


def sample_object_xy(rng):
    """Sample the paper's 40 x 30 cm areas around both demo placements."""
    half_area = np.array([0.20, 0.15])
    clearance = PLATE_RADIUS + np.linalg.norm(SPONGE_HALF_XY)
    while True:
        xy = DEMO_OBJECT_XY + rng.uniform(-half_area, half_area, size=(2, 2))
        if np.linalg.norm(xy[0] - xy[1]) >= clearance:
            return xy


def _plate_top_rings():
    radii, heights = np.asarray(PLATE_PROFILE).T
    theta = np.arange(64) * (2 * np.pi / 64)
    return np.stack((
        radii[:, None] * np.cos(theta),
        radii[:, None] * np.sin(theta),
        np.broadcast_to(heights[:, None], (len(radii), len(theta))),
    ), axis=-1)


def _add_plate_collision_rim(spec, top_rings):
    plate = spec.body("wiping_plate")
    faces = np.array([
        (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
    ]).ravel()
    sectors = top_rings.shape[1]
    for band, (inner, outer) in enumerate(zip(top_rings, top_rings[1:])):
        for sector in range(sectors):
            next_sector = (sector + 1) % sectors
            top = np.array([inner[sector], outer[sector], outer[next_sector], inner[next_sector]])
            bottom = top - [0, 0, PLATE_BASE_THICKNESS]
            name = f"wiping_plate_rim_{band}_{sector}"
            mesh = spec.add_mesh(
                name=name, uservert=np.concatenate((top, bottom)).ravel(),
                userface=faces,
            )
            plate.add_geom(
                name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh.name,
                group=3, rgba=(0.88, 0.90, 0.96, 1), mass=0, condim=4,
                friction=(0.8, 0.01, 0.001),
            )


def _add_plate_rim_visual(spec, top_rings):
    sectors = top_rings.shape[1]
    vertices = np.concatenate((top_rings, top_rings[::-1] - [0, 0, PLATE_BASE_THICKNESS]))
    profile_len = len(vertices)
    vertices = vertices.reshape(-1, 3)
    rings = np.arange(len(vertices)).reshape(profile_len, sectors)
    inner, outer = rings, np.roll(rings, -1, axis=0)
    next_inner, next_outer = np.roll(inner, -1, axis=1), np.roll(outer, -1, axis=1)
    faces = np.stack((inner, outer, next_outer, inner, next_outer, next_inner), axis=-1)
    mesh = spec.add_mesh(
        name="wiping_rim_mesh", uservert=vertices.ravel(), userface=faces.ravel(),
        usertexcoord=(0.5 + vertices[:, :2] / (2 * PLATE_RADIUS)).ravel(),
    )
    texture = spec.add_texture(
        name="wiping_rim_texture", type=mujoco.mjtTexture.mjTEXTURE_2D, vflip=True,
        file=str(Path(__file__).with_name("assets") / "wiping_plate_rim.png"),
    )
    material = spec.add_material(name="wiping_rim_material", specular=0)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = texture.name
    spec.body("wiping_plate").add_geom(
        name="wiping_plate_rim_visual", type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname=mesh.name, material=material.name, contype=0, conaffinity=0, mass=0,
    )


def add_wiping_scene(spec, object_xy=DEMO_OBJECT_XY):
    """Add the measured plate and a volumetric elastic sponge to ``spec``."""
    object_xy = np.asarray(object_xy, dtype=float)
    if object_xy.shape != (2, 2) or not np.isfinite(object_xy).all():
        raise ValueError("object_xy must be a finite 2 x 2 array")
    plate_xy, sponge_xy = object_xy
    child = mujoco.MjSpec.from_string(
        f"""
        <mujoco model="dexcap_wiping">
          <worldbody>
            <body name="wiping_plate" pos="{plate_xy[0]} {plate_xy[1]} {TABLE_HEIGHT + PLATE_BASE_THICKNESS / 2}">
              <freejoint name="wiping_plate_free"/>
              <inertial pos="0 0 0" mass="0.25" diaginertia="0.001 0.001 0.002"/>
              <geom name="wiping_plate_base" type="cylinder"
                    size="{PLATE_INNER_RADIUS} {PLATE_BASE_THICKNESS / 2}"
                    rgba="0.88 0.90 0.96 1" friction="0.8 0.01 0.001" condim="4"/>
            </body>
            {sponge_flexcomp(sponge_xy)}
          </worldbody>
        </mujoco>
        """
    )
    top_rings = _plate_top_rings()
    _add_plate_collision_rim(child, top_rings)
    _add_plate_rim_visual(child, top_rings)
    texture_sponge(child)
    mount = spec.worldbody.add_frame(name="wiping_scene")
    spec.attach(child, frame=mount, prefix="")


class WipingMetrics:
    """Accumulate legacy Wiping summary metrics from MuJoCo contacts."""

    def __init__(self, model):
        self.model = model
        self.plate_body = model.body("wiping_plate").id
        self.table_geom = model.geom("table").id
        self.plate_geoms = {
            geom for geom in range(model.ngeom)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or "").startswith("wiping_plate_")
        }
        self.right_hand_geoms = self._named_geoms("right_hand_")
        self.left_hand_geoms = self._named_geoms("left_hand_")
        self.sponge_flex = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "wiping_sponge")
        self.flex_start = model.flex_vertadr[self.sponge_flex]
        self.flex_count = model.flex_vertnum[self.sponge_flex]
        joint = model.body_jntadr[self.plate_body]
        self.initial_plate_z = model.qpos0[model.jnt_qposadr[joint] + 2]
        # Trilinear flex_vert0 stores normalized coordinates, not meters.
        self.initial_sponge_z = float(np.mean(model.flex_vert[self.flex_start:self.flex_start + self.flex_count, 2]))
        self.contact_steps = 0
        self.surface_contact_steps = 0
        self.peak_contact_force = 0.0
        self.wipe_distance = 0.0
        self.wipe_passes = 0
        self.plate_grasped = False
        self.sponge_grasped = False
        self.last_contact_position = None
        self.stroke_axis = None
        self.stroke_extents = None
        self.stroke_direction = 0

    def _named_geoms(self, prefix):
        return {
            geom for geom in range(self.model.ngeom)
            if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom) or "").startswith(prefix)
        }

    def _is_sponge_plate(self, contact):
        return (
            contact.flex[0] == self.sponge_flex and contact.geom[1] in self.plate_geoms
        ) or (
            contact.flex[1] == self.sponge_flex and contact.geom[0] in self.plate_geoms
        )

    def _opposed_hand_grip(self, contacts, hand_geoms, target):
        hits = []
        for contact in contacts:
            first, second = contact.geom
            if target == "plate":
                touching = first in self.plate_geoms or second in self.plate_geoms
            else:
                touching = self.sponge_flex in contact.flex
            if not touching:
                continue
            if first in hand_geoms:
                hits.append((self.model.geom_bodyid[first], contact.frame[:3]))
            elif second in hand_geoms:
                hits.append((self.model.geom_bodyid[second], -contact.frame[:3]))
        return any(
            first != second and np.dot(a, b) < -0.25
            for (first, a), (second, b) in combinations(hits, 2)
        )

    def update(self, data):
        contacts = [data.contact[index] for index in range(data.ncon)]
        plate_contacts = [contact for contact in contacts if self._is_sponge_plate(contact)]
        contact_force = 0.0
        for index, contact in enumerate(contacts):
            if not self._is_sponge_plate(contact):
                continue
            wrench = np.zeros(6)
            mujoco.mj_contactForce(self.model, data, index, wrench)
            contact_force += max(0.0, float(wrench[0]))
        self.contact_steps += int(bool(plate_contacts))
        self.peak_contact_force = max(self.peak_contact_force, contact_force)

        plate_pos = data.xpos[self.plate_body]
        plate_rotation = data.xmat[self.plate_body].reshape(3, 3)
        interior = [
            contact for contact in plate_contacts
            if (local := plate_rotation.T @ (contact.pos - plate_pos))[2] >= -PLATE_CONTACT_TOLERANCE
            and np.linalg.norm(local[:2]) <= PLATE_INNER_RADIUS
        ]
        self.surface_contact_steps += int(bool(interior))

        table_contacts = [contact for contact in contacts if self.table_geom in contact.geom]
        plate_held = (
            plate_pos[2] > self.initial_plate_z + 0.01
            and not any(self.plate_geoms.intersection(contact.geom) for contact in table_contacts)
            and self._opposed_hand_grip(contacts, self.right_hand_geoms, "plate")
        )
        sponge_held = (
            self._sponge_center(data)[2] > self.initial_sponge_z + 0.01
            and not any(self.sponge_flex in contact.flex for contact in table_contacts)
            and self._opposed_hand_grip(contacts, self.left_hand_geoms, "sponge")
        )
        self.plate_grasped |= bool(plate_held)
        self.sponge_grasped |= bool(sponge_held)
        self._update_wipe(interior if plate_held and sponge_held else (), data, plate_pos, plate_rotation)

        return {
            "sponge_plate_contacts": len(plate_contacts),
            "contact_force": contact_force,
            "sponge_plate_contact_steps": self.contact_steps,
            "wipe_surface_contact_steps": self.surface_contact_steps,
            "peak_contact_force": self.peak_contact_force,
            "wipe_distance": self.wipe_distance,
            "wipe_passes": self.wipe_passes,
            "plate_grasped": self.plate_grasped,
            "sponge_grasped": self.sponge_grasped,
            "sim_success": self.wipe_passes >= 2 and self.plate_grasped and self.sponge_grasped,
        }

    def _sponge_center(self, data):
        return data.flexvert_xpos[self.flex_start:self.flex_start + self.flex_count].mean(axis=0)

    def _update_wipe(self, interior, data, plate_pos, plate_rotation):
        if not interior:
            self.last_contact_position = None
            self.stroke_axis = None
            return
        local = plate_rotation.T @ (self._sponge_center(data) - plate_pos)
        tangent = local[:2]
        if self.last_contact_position is not None:
            self.wipe_distance += float(np.linalg.norm(tangent - self.last_contact_position))
        self.last_contact_position = tangent.copy()
        self._update_vertical_strokes(tangent, plate_rotation[2, :2])

    def _update_vertical_strokes(self, tangent, projected_up):
        if self.stroke_axis is None:
            tilt = np.linalg.norm(projected_up)
            if tilt <= 1e-3:
                return
            self.stroke_axis = projected_up / tilt
            self.stroke_extents = np.stack((tangent, tangent))
            self.stroke_direction = 0
            return
        deltas = tangent - self.stroke_extents
        vertical = deltas @ self.stroke_axis
        for index, direction in enumerate((1, -1)):
            if direction == self.stroke_direction or direction * vertical[index] < MIN_WIPE_STROKE_M:
                continue
            across = deltas[index] - vertical[index] * self.stroke_axis
            if np.linalg.norm(across) <= abs(vertical[index]):
                self.wipe_passes += 1
                self.stroke_direction = direction
                self.stroke_extents[:] = tangent
                return
        if vertical[0] < 0:
            self.stroke_extents[0] = tangent
        if vertical[1] > 0:
            self.stroke_extents[1] = tangent
