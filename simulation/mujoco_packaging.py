"""Physical MuJoCo scene and task measurements for DexCap Packaging."""

from __future__ import annotations

import itertools

import mujoco
import numpy as np

from simulation.packaging_objects import (
    DEMO_OBJECT_XY,
    PACKAGING_OBJECTS,
    PACKAGING_UNSEEN_OBJECTS,
    UNSEEN_COMPOUNDS,
    UNSEEN_PRIMITIVES,
    compound_bounds,
    sample_object_xy,
)


TABLE_HEIGHT = -0.030
BOX_HALF_XY = (0.149, 0.095)
BOX_PANEL_HALF_THICKNESS = 0.0015
BOX_TOP = 0.078
BOX_YAW = np.deg2rad(-62.8)
LID_OPEN_ANGLE = np.deg2rad(3.35)
LID_TAB_ANGLE = np.deg2rad(26)
LID_TAB_HALF_XY = (0.145, 0.035)
LID_CLOSED_TOLERANCE = np.deg2rad(6)
DEFAULT_OBJECT = "red-box"
RED_BOX_YAW = np.deg2rad(64)
RED_BOX_HALF_SIZE = (0.070, 0.028, 0.044)
CARDBOARD = (0.62, 0.42, 0.22, 1.0)
RED_BOX_TOP_COLOR = (0.93, 0.20, 0.06, 1.0)
WORKSPACE_CENTER = np.array([0.4, 0.0])
CLOTH_OBJECT = "yellow-cloth"
CLOTH_SIZE = (0.15, 0.13, 0.04)
CLOTH_RADIUS = 0.003
# The release measures shape but does not provide folded-textile material calibration.
CLOTH_YOUNG_MODULUS_PA = 10_000
CLOTH_POISSON_RATIO = 0.2
CLOTH_DAMPING_S = 0.05


def object_reference(name):
    """Return the demo/video evidence associated with an object geometry."""
    if name in UNSEEN_COMPOUNDS:
        return UNSEEN_COMPOUNDS[name][0]
    registry = UNSEEN_PRIMITIVES if name in UNSEEN_PRIMITIVES else PACKAGING_OBJECTS
    try:
        return registry[name][3]
    except KeyError as error:
        raise ValueError(f"Unknown Packaging object: {name}") from error


def _wxyz(xyzw):
    return [xyzw[3], *xyzw[:3]]


def _yaw(angle):
    return [np.cos(angle / 2), 0, 0, np.sin(angle / 2)]


def _roll(angle):
    return [np.cos(angle / 2), np.sin(angle / 2), 0, 0]


def _geom_volume(kind, size):
    if kind == "box":
        return 8 * np.prod(size)
    if kind == "cylinder":
        return 2 * np.pi * size[0] ** 2 * size[2]
    return 4 * np.pi * np.prod(size) / 3


def _add_geom(body, *, name, kind, size, pos=(0, 0, 0), quat=(1, 0, 0, 0),
              rgba=(0.7, 0.7, 0.7, 1), density=500):
    geom_types = {
        "box": mujoco.mjtGeom.mjGEOM_BOX,
        "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
        "ellipsoid": mujoco.mjtGeom.mjGEOM_ELLIPSOID,
    }
    return body.add_geom(
        name=name,
        type=geom_types[kind],
        size=size,
        pos=pos,
        quat=quat,
        rgba=rgba,
        density=density,
        friction=[1.0, 0.01, 0.001],
    )


def _add_box(spec, xy):
    half_x, half_y = BOX_HALF_XY
    panel = BOX_PANEL_HALF_THICKNESS
    wall_half_height = BOX_TOP / 2 - panel
    wall_center = panel + wall_half_height
    components = (
        ((half_x, half_y, panel), (0, 0, 0)),
        ((panel, half_y, wall_half_height), (panel - half_x, 0, wall_center)),
        ((panel, half_y, wall_half_height), (half_x - panel, 0, wall_center)),
        ((half_x, panel, wall_half_height), (0, panel - half_y, wall_center)),
        ((half_x, panel, wall_half_height), (0, half_y - panel, wall_center)),
    )
    density = 0.6 / sum(_geom_volume("box", size) for size, _ in components)
    box = spec.worldbody.add_body(
        name="packaging_box",
        pos=[*xy, TABLE_HEIGHT + panel],
        quat=_yaw(BOX_YAW),
    )
    box.add_freejoint(name="packaging_box_free")
    for index, (size, pos) in enumerate(components):
        _add_geom(
            box,
            name="packaging_box_floor" if index == 0 else f"packaging_box_wall_{index}",
            kind="box",
            size=size,
            pos=pos,
            rgba=CARDBOARD,
            density=density,
        )

    lid = box.add_body(
        name="packaging_lid", pos=[0, half_y, BOX_TOP], quat=_roll(LID_OPEN_ANGLE),
    )
    lid.add_joint(
        name="packaging_lid_hinge",
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[1, 0, 0],
        limited=True,
        range=[0, np.pi],
        ref=LID_OPEN_ANGLE,
        damping=0.003,
    )
    lid_density = 0.02 / _geom_volume("box", (half_x, half_y, panel))
    _add_geom(
        lid,
        name="packaging_lid_panel",
        kind="box",
        size=(half_x, half_y, panel),
        pos=[0, half_y, 0],
        rgba=CARDBOARD,
        density=lid_density,
    )


    # The frame_2710 depth shows a 70 mm flap beyond the lid's second fold line.
    tab = lid.add_body(
        name="packaging_lid_tab", pos=[0, 2 * half_y, 0], quat=_roll(LID_TAB_ANGLE),
    )
    tab.add_joint(
        name="packaging_tab_hinge",
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[1, 0, 0],
        limited=True,
        range=[0, np.deg2rad(110)],
        ref=LID_TAB_ANGLE,
        springref=LID_TAB_ANGLE,
        stiffness=0.04,
        damping=0.002,
    )
    tab_size = (*LID_TAB_HALF_XY, panel)
    _add_geom(
        tab,
        name="packaging_tab_panel",
        kind="box",
        size=tab_size,
        pos=[0, LID_TAB_HALF_XY[1], 0],
        rgba=CARDBOARD,
        density=0.008 / _geom_volume("box", tab_size),
    )

def _object_definition(object_name):
    if object_name in UNSEEN_COMPOUNDS:
        parts = UNSEEN_COMPOUNDS[object_name][1]
        lower, _ = compound_bounds(parts, np.eye(3))
        return parts, -lower[2]
    registry = UNSEEN_PRIMITIVES if object_name in UNSEEN_PRIMITIVES else PACKAGING_OBJECTS
    kind, size, color, _ = registry[object_name]
    if object_name == DEFAULT_OBJECT:
        size = RED_BOX_HALF_SIZE
    return ((kind, size, (0, 0, 0), color, (0, 0, 0, 1)),), size[2]


def _add_cloth(spec, xy):
    color = PACKAGING_OBJECTS[CLOTH_OBJECT][2]
    child = mujoco.MjSpec.from_string(
        f"""
        <mujoco model="dexcap_packaging_cloth">
          <worldbody>
            <flexcomp name="packaging_object" type="grid" count="6 6 3"
                      spacing="0.03 0.026 0.02" dim="3" radius="{CLOTH_RADIUS}"
                      pos="{xy[0]} {xy[1]} {TABLE_HEIGHT + CLOTH_SIZE[2] / 2 + CLOTH_RADIUS}"
                      mass="0.12" dof="trilinear" rgba="{' '.join(map(str, color))}">
              <elasticity young="{CLOTH_YOUNG_MODULUS_PA}" poisson="{CLOTH_POISSON_RATIO}"
                          damping="{CLOTH_DAMPING_S}"/>
              <contact selfcollide="none" friction="1.0 0.01 0.001"
                       solref="0.005 1" solimp="0.9 0.99 0.001"/>
            </flexcomp>
          </worldbody>
        </mujoco>
        """
    )
    spec.attach(child, frame=spec.worldbody.add_frame(name="packaging_cloth_scene"), prefix="")


def _add_object(spec, xy, object_name):
    if object_name == CLOTH_OBJECT:
        _add_cloth(spec, xy)
        return
    definitions, support_height = _object_definition(object_name)
    body = spec.worldbody.add_body(
        name="packaging_object",
        pos=[*xy, TABLE_HEIGHT + support_height],
        quat=_yaw(RED_BOX_YAW) if object_name == DEFAULT_OBJECT else [1, 0, 0, 0],
    )
    body.add_freejoint(name="packaging_object_free")

    if object_name in UNSEEN_COMPOUNDS:
        parts = definitions
        volumes = [_geom_volume(part.kind, part.half_size) for part in parts]
        density = 0.12 / sum(volumes)
        for index, part in enumerate(parts):
            _add_geom(
                body,
                name=f"packaging_object_part_{index}",
                kind=part.kind,
                size=part.half_size,
                pos=part.center,
                quat=_wxyz(part.orientation),
                rgba=part.color,
                density=density,
            )
    else:
        kind, size, pos, color, quat = definitions[0]
        density = 0.12 / _geom_volume(kind, size)
        _add_geom(
            body,
            name="packaging_object_geom",
            kind=kind,
            size=size,
            pos=pos,
            quat=_wxyz(quat),
            rgba=color,
            density=density,
        )
        if object_name == DEFAULT_OBJECT:
            body.add_geom(
                name="packaging_object_top_visual",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[size[0], size[1], 0.0003],
                pos=[0, 0, size[2]],
                rgba=RED_BOX_TOP_COLOR,
                contype=0,
                conaffinity=0,
                mass=0,
            )


def add_packaging_scene(spec: mujoco.MjSpec, object_xy, object_name=DEFAULT_OBJECT):
    """Add the freely moving box, hinged lid, and selected free object to ``spec``."""
    if object_name not in PACKAGING_OBJECTS and object_name not in PACKAGING_UNSEEN_OBJECTS:
        raise ValueError(f"Unknown Packaging object: {object_name}")
    positions = np.asarray(object_xy, dtype=float)
    if positions.shape != (2, 2):
        raise ValueError("object_xy must contain box and object XY positions with shape (2, 2)")
    spec.compiler.degree = False
    _add_box(spec, positions[0])
    _add_object(spec, positions[1], object_name)


class PackagingMetrics:
    """Accumulate legacy report fields from contacts and generalized coordinates."""

    def __init__(self, model: mujoco.MjModel, object_name: str):
        if object_name not in PACKAGING_OBJECTS and object_name not in PACKAGING_UNSEEN_OBJECTS:
            raise ValueError(f"Unknown Packaging object: {object_name}")
        self.model = model
        self.box_body = model.body("packaging_box").id
        self.box_joint = model.joint("packaging_box_free").id
        self.lid_body = model.body("packaging_lid").id
        self.lid_joint = model.joint("packaging_lid_hinge").id
        self.floor_geom = model.geom("packaging_box_floor").id
        self.table_geom = model.geom("table").id
        self.object_flex = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "packaging_object")
        if self.object_flex >= 0:
            self.object_body = -1
            self.object_geoms = set()
            self.flex_start = model.flex_vertadr[self.object_flex]
            self.flex_count = model.flex_vertnum[self.object_flex]
            self.flex_radius = model.flex_radius[self.object_flex]
            reference = mujoco.MjData(model)
            mujoco.mj_forward(model, reference)
            self.initial_object_z = float(np.mean(self._flex_vertices(reference)[:, 2]))
            self.object_dofs = np.concatenate([
                np.arange(model.body_dofadr[body], model.body_dofadr[body] + model.body_dofnum[body])
                for body in range(model.nbody)
                if model.body(body).name.startswith("packaging_object_")
            ])
        else:
            self.object_body = model.body("packaging_object").id
            object_joint = model.body_jntadr[self.object_body]
            self.initial_object_z = model.qpos0[model.jnt_qposadr[object_joint] + 2]
            self.object_geoms = self._body_geoms(self.object_body, collidable_only=True)
            self.object_dofs = None
        self.box_geoms = self._body_geoms(self.box_body, collidable_only=True)
        self.lid_geoms = self._body_geoms(self.lid_body, collidable_only=True)
        self.lid_geoms.update(self._body_geoms(model.body("packaging_lid_tab").id, collidable_only=True))
        self.right_geoms = self._named_geoms("right")
        self.left_geoms = self._named_geoms("left")
        self.contact_counts = [0, 0, 0]
        box_qpos = model.jnt_qposadr[self.box_joint]
        self.initial_box_xy = np.asarray(model.qpos0[box_qpos:box_qpos + 2]).copy()
        self.initial_center_distance = np.linalg.norm(self.initial_box_xy - WORKSPACE_CENTER)
        self.minimum_center_distance = self.initial_center_distance
        self.ever_grasped = False

    def _body_geoms(self, body_id, collidable_only=False):
        start = self.model.body_geomadr[body_id]
        geoms = set(range(start, start + self.model.body_geomnum[body_id]))
        if collidable_only:
            geoms = {geom for geom in geoms if self.model.geom_contype[geom]}
        return geoms

    def _named_geoms(self, side):
        return {
            geom for geom in range(self.model.ngeom)
            if self.model.geom(geom).name.startswith(f"{side}_hand_")
        }

    def _contacts(self, data, first, second):
        result = []
        for index in range(data.ncon):
            contact = data.contact[index]
            if contact.geom1 in first and contact.geom2 in second:
                result.append((contact, np.asarray(contact.frame[:3])))
            elif contact.geom2 in first and contact.geom1 in second:
                result.append((contact, -np.asarray(contact.frame[:3])))
        return result

    def _flex_vertices(self, data):
        return data.flexvert_xpos[self.flex_start:self.flex_start + self.flex_count]

    def _flex_contacts(self, data, geoms):
        result = []
        for index in range(data.ncon):
            contact = data.contact[index]
            if contact.geom[0] in geoms and contact.flex[1] == self.object_flex:
                result.append((contact, np.asarray(contact.frame[:3])))
            elif contact.geom[1] in geoms and contact.flex[0] == self.object_flex:
                result.append((contact, -np.asarray(contact.frame[:3])))
        return result

    def _object_contacts(self, data, geoms):
        if self.object_flex >= 0:
            return self._flex_contacts(data, geoms)
        return self._contacts(data, geoms, self.object_geoms)

    def _object_center(self, data):
        if self.object_flex >= 0:
            return np.mean(self._flex_vertices(data), axis=0)
        return data.xpos[self.object_body]

    def _object_grasped(self, contacts):
        if len(contacts) < 2:
            return False
        normals = [normal for _, normal in contacts]
        return any(np.dot(a, b) < -0.25 for a, b in itertools.combinations(normals, 2))

    def _geom_extent_in_box(self, data, geom):
        box_rotation = data.xmat[self.box_body].reshape(3, 3)
        geom_rotation = data.geom_xmat[geom].reshape(3, 3)
        rotation = box_rotation.T @ geom_rotation
        size = self.model.geom_size[geom]
        geom_type = self.model.geom_type[geom]
        if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            axis = np.abs(rotation[:, 2])
            extent = size[0] * np.sqrt(np.maximum(0, 1 - axis**2)) + size[1] * axis
        elif geom_type in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
            extent = np.sqrt((rotation**2) @ (size**2))
        else:
            extent = np.abs(rotation) @ size
        center = box_rotation.T @ (data.geom_xpos[geom] - data.xpos[self.box_body])
        return center - extent, center + extent

    def _object_in_box(self, data):
        if self.object_flex >= 0:
            box_position = data.xpos[self.box_body]
            box_rotation = data.xmat[self.box_body].reshape(3, 3)
            local = (self._flex_vertices(data) - box_position) @ box_rotation
            lower = np.min(local, axis=0) - self.flex_radius
            upper = np.max(local, axis=0) + self.flex_radius
            inner_half = np.asarray(BOX_HALF_XY) - 2 * BOX_PANEL_HALF_THICKNESS
            inside = np.all(lower[:2] >= -inner_half) and np.all(upper[:2] <= inner_half)
            supported = bool(self._flex_contacts(data, {self.floor_geom}))
            settled = np.max(np.abs(data.qvel[self.object_dofs])) < 0.05
            return bool(inside and supported and settled)

        bounds = [self._geom_extent_in_box(data, geom) for geom in self.object_geoms]
        lower = np.min([bound[0] for bound in bounds], axis=0)
        upper = np.max([bound[1] for bound in bounds], axis=0)
        inner_half = np.asarray(BOX_HALF_XY) - 2 * BOX_PANEL_HALF_THICKNESS
        inside = np.all(lower[:2] >= -inner_half) and np.all(upper[:2] <= inner_half)
        supported = bool(self._contacts(data, self.object_geoms, {self.floor_geom}))
        joint = self.model.body_jntadr[self.object_body]
        dof = self.model.jnt_dofadr[joint]
        settled = np.linalg.norm(data.qvel[dof:dof + 6]) < 0.05
        return bool(inside and supported and settled)

    def update(self, data: mujoco.MjData) -> dict:
        right_object = self._object_contacts(data, self.right_geoms)
        left_lid = self._contacts(data, self.left_geoms, self.lid_geoms)
        left_box = self._contacts(data, self.left_geoms, self.box_geoms)
        for index, contacts in enumerate((right_object, left_lid, left_box)):
            self.contact_counts[index] += len(contacts)

        box_xy = np.asarray(data.xpos[self.box_body, :2]).copy()
        center_distance = np.linalg.norm(box_xy - WORKSPACE_CENTER)
        self.minimum_center_distance = min(self.minimum_center_distance, center_distance)
        lifted = self._object_center(data)[2] > self.initial_object_z + 0.01
        object_grasped = lifted and self._object_grasped(right_object) and not self._object_contacts(
            data, {self.table_geom}
        )
        object_released = self.ever_grasped and not object_grasped
        self.ever_grasped |= object_grasped
        object_in_box = self._object_in_box(data)
        lid_angle = data.qpos[self.model.jnt_qposadr[self.lid_joint]]
        lid_closed = abs(lid_angle - np.pi) <= LID_CLOSED_TOLERANCE
        return {
            "right_object_contacts": self.contact_counts[0],
            "left_lid_contacts": self.contact_counts[1],
            "left_box_contacts": self.contact_counts[2],
            "box_displacement_m": float(np.linalg.norm(box_xy - self.initial_box_xy)),
            "box_center_progress_m": float(self.initial_center_distance - self.minimum_center_distance),
            "object_grasped": bool(object_grasped),
            "object_released": bool(object_released),
            "object_in_box": object_in_box,
            "lid_closed": bool(lid_closed),
            "sim_success": bool(object_in_box and lid_closed),
        }
