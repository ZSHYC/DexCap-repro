"""MuJoCo objects and physical metrics for DexCap plate wiping."""

import math

import mujoco
import numpy as np


DEMO_OBJECT_XY = np.array([[0.54, -0.15], [0.542, 0.112]])
TABLE_HEIGHT = -0.03
PLATE_RADIUS = 0.125
PLATE_INNER_RADIUS = 0.09
PLATE_BASE_THICKNESS = 0.004
PLATE_CONTACT_TOLERANCE = 0.001
SPONGE_HALF_XY = (0.04, 0.065)
SPONGE_HALF_HEIGHT = 0.017
SPONGE_YAW = 0.33
MIN_WIPE_STROKE_M = 0.06

# The released depth measurement fixes the size, not the foam material parameters.
SPONGE_YOUNG_MODULUS_PA = 50_000
SPONGE_POISSON_RATIO = 0.25
SPONGE_DAMPING_S = 0.02

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


def _quat_for_ring(yaw, slope):
    """Quaternion for a radial box whose x axis follows the measured slope."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(-slope / 2), math.sin(-slope / 2)
    return cy * cp, -sy * sp, cy * sp, sy * cp


def _plate_geoms():
    profile = ((0.09, 0.002), (0.10, 0.007), (0.115, 0.0195), (0.125, 0.0185))
    color = "0.88 0.90 0.96 1"
    geoms = [
        '<geom name="wiping_plate_base" type="cylinder" size="0.09 0.002" '
        f'rgba="{color}" friction="0.8 0.01 0.001" condim="4"/>'
    ]
    segments = 32
    shell_half_thickness = PLATE_BASE_THICKNESS / 2
    for band, ((r0, z0), (r1, z1)) in enumerate(zip(profile, profile[1:])):
        dr, dz = r1 - r0, z1 - z0
        slope = math.atan2(dz, dr)
        radial_half = math.hypot(dr, dz) / 2
        normal_r, normal_z = -math.sin(slope), math.cos(slope)
        radius = (r0 + r1) / 2 - shell_half_thickness * normal_r
        z = (z0 + z1) / 2 - shell_half_thickness * normal_z
        tangent_half = radius * math.tan(math.pi / segments)
        for sector in range(segments):
            yaw = sector * 2 * math.pi / segments
            quat = _quat_for_ring(yaw, slope)
            geoms.append(
                f'<geom name="wiping_plate_rim_{band}_{sector}" type="box" '
                f'pos="{radius * math.cos(yaw)} {radius * math.sin(yaw)} {z}" '
                f'quat="{quat[0]} {quat[1]} {quat[2]} {quat[3]}" '
                f'size="{radial_half} {tangent_half} {shell_half_thickness}" '
                f'rgba="{color}" friction="0.8 0.01 0.001" condim="4"/>'
            )
    return "\n".join(geoms)


def _texture_sponge(spec):
    # Color boundaries come from the five depth and RGB samples in frame_2220.
    height = 128
    bands = np.array([[19, 21, 12], [151, 129, 42], [48, 47, 24], [163, 147, 62], [236, 212, 106]])
    edges = [-0.0116, -0.0015, 0.0086, 0.0148]
    levels = np.linspace(-SPONGE_HALF_HEIGHT, SPONGE_HALF_HEIGHT, height)
    texture = spec.add_texture(
        name="wiping_sponge_bands", type=mujoco.mjtTexture.mjTEXTURE_2D, width=1, height=height, nchannel=3,
    )
    texture.data = bands[np.searchsorted(edges, levels)].astype(np.uint8).tobytes()
    material = spec.add_material(name="wiping_sponge_material", specular=0)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = texture.name
    flex = spec.flex("wiping_sponge")
    z = np.asarray(flex.vert).reshape(-1, 3)[:, 2]
    vertical = (0.5 + (height - 1) * (z - z.min()) / np.ptp(z)) / height
    flex.texcoord = np.column_stack((np.full(len(z), 0.5), vertical)).ravel()
    flex.material = material.name


def add_wiping_scene(spec, object_xy=DEMO_OBJECT_XY):
    """Add the measured plate and a volumetric elastic sponge to ``spec``."""
    object_xy = np.asarray(object_xy, dtype=float)
    if object_xy.shape != (2, 2) or not np.isfinite(object_xy).all():
        raise ValueError("object_xy must be a finite 2 x 2 array")
    plate_xy, sponge_xy = object_xy
    yaw_half = SPONGE_YAW / 2
    child = mujoco.MjSpec.from_string(
        f"""
        <mujoco model="dexcap_wiping">
          <worldbody>
            <body name="wiping_plate" pos="{plate_xy[0]} {plate_xy[1]} {TABLE_HEIGHT + 0.002}">
              <freejoint name="wiping_plate_free"/>
              <inertial pos="0 0 0" mass="0.25" diaginertia="0.001 0.001 0.002"/>
              {_plate_geoms()}
            </body>
            <flexcomp name="wiping_sponge" type="grid" count="5 6 3"
                      spacing="0.02 0.026 {SPONGE_HALF_HEIGHT}" dim="3" radius="0.002"
                      pos="{sponge_xy[0]} {sponge_xy[1]} {TABLE_HEIGHT + SPONGE_HALF_HEIGHT + 0.001}"
                      quat="{math.cos(yaw_half)} 0 0 {math.sin(yaw_half)}"
                      mass="0.06" dof="trilinear" rgba="1 1 1 1">
              <elasticity young="{SPONGE_YOUNG_MODULUS_PA}" poisson="{SPONGE_POISSON_RATIO}"
                          damping="{SPONGE_DAMPING_S}"/>
              <contact selfcollide="none" friction="1.2 0.01 0.001"
                       solref="0.005 1" solimp="0.9 0.99 0.001"/>
            </flexcomp>
          </worldbody>
        </mujoco>
        """
    )
    _texture_sponge(child)
    mount = spec.worldbody.add_frame(name="wiping_scene")
    spec.attach(child, frame=mount, prefix="")


class WipingMetrics:
    """Accumulate legacy Wiping summary metrics from MuJoCo contacts."""

    def __init__(self, model):
        self.model = model
        self.plate_body = model.body("wiping_plate").id
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
        self.initial_sponge_z = float(np.mean(model.flex_vert0[self.flex_start:self.flex_start + self.flex_count, 2]))
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

    def _hand_contacts(self, contacts, hand_geoms, target):
        hits = set()
        for contact in contacts:
            if target == "plate":
                pair = set(contact.geom)
                if pair & self.plate_geoms:
                    hits.update(pair & hand_geoms)
            elif contact.flex[0] == self.sponge_flex or contact.flex[1] == self.sponge_flex:
                hits.update(set(contact.geom) & hand_geoms)
        return hits

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
        self._update_wipe(interior, data, plate_pos, plate_rotation)

        plate_hands = self._hand_contacts(contacts, self.right_hand_geoms, "plate")
        sponge_hands = self._hand_contacts(contacts, self.left_hand_geoms, "sponge")
        sponge_center = self._sponge_center(data)
        self.plate_grasped |= bool(len(plate_hands) >= 2 and plate_pos[2] > self.initial_plate_z + 0.01)
        self.sponge_grasped |= bool(len(sponge_hands) >= 2 and sponge_center[2] > self.initial_sponge_z + 0.01)

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
        self.surface_contact_steps += 1
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
