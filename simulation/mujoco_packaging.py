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
BOX_INNER_HALF_XY = np.asarray(BOX_HALF_XY) - 2 * BOX_PANEL_HALF_THICKNESS
BOX_CONTACT_TOLERANCE_M = 0.0001
BOX_TOP = 0.078
BOX_YAW = np.deg2rad(-62.8)
LID_OPEN_ANGLE = np.deg2rad(3.35)
LID_TAB_ANGLE = np.deg2rad(26)
LID_TAB_HALF_XY = (0.145, 0.035)
LID_CLOSED_TOLERANCE = np.deg2rad(6)
LID_CLEARANCE_TOLERANCE_M = 0.001
LID_CONTACT_FORCE_N = .05
DEFAULT_OBJECT = "red-box"
RED_BOX_YAW = np.deg2rad(61.5)
# Calibrated against the box floor and open lid in the source RGB-D frame.
CARDBOARD = (0.62, 0.54, 0.43, 1.0)
RED_BOX_TOP_COLOR = (0.93, 0.20, 0.06, 1.0)
WORKSPACE_CENTER = np.array([0.4, 0.0])
CLOTH_OBJECT = "yellow-cloth"
CLOTH_HALF_XY = (0.06950121451927635, 0.05741672959751279)
CLOTH_RADIUS = 0.00265
CLOTH_LAYER_GAP = 0.0055
CLOTH_LAYER_COUNT = 10
CLOTH_GRID_SIZE = 4
CLOTH_LAYER_XY_FRACTIONS = (
    (1.00, 1.00),
    (1.00, 1.00),
    (1.00, 1.00),
    (0.95, 0.84),
    (0.90, 0.72),
    (0.82, 0.68),
    (0.75, 0.60),
    (0.68, 0.49),
    (0.40, 0.50),
    (0.20, 0.20),
)
CLOTH_LAYER_CENTER_OFFSETS = (
    (0.000, 0.000),
    (0.000, 0.000),
    (0.000, 0.000),
    (-0.003, -0.003),
    (-0.006, -0.005),
    (-0.010, -0.007),
    (-0.017, -0.009),
    (-0.022, -0.011),
    (-0.040, -0.012),
    (-0.040, -0.012),
)
CLOTH_YOUNG_MODULUS_PA = 1_000
CLOTH_POISSON_RATIO = 0.2
CLOTH_DAMPING_S = 0.2
CLOTH_THICKNESS = 0.0008
# The comparable 6-inch clamp has a 65 mm jaw opening. Spring force is estimated.
SPRING_CLAMP_MAX_ANGLE = np.deg2rad(44)
SPRING_CLAMP_PRELOAD_ANGLE = np.deg2rad(8)
SPRING_CLAMP_STIFFNESS_NM_PER_RAD = 0.30
SPRING_CLAMP_DAMPING_NMS_PER_RAD = 0.01


# Centers of the visible object components in the six released reference frames.
REFERENCE_OBJECT_XY = {
    "red-box": (0.51791, -0.22465),
    "blue-block": (0.58056, -0.25922),
    "green-pear": (0.47753, -0.18266),
    "green-can": (0.53101, -0.23900),
    "tape-measure": (0.53260, -0.22439),
    "yellow-cloth": (0.48688, -0.24359),
}
REFERENCE_OBJECT_YAW = {"red-box": np.deg2rad(61.19), "blue-block": np.deg2rad(61.47)}


def replay_placement(name):
    xy = DEMO_OBJECT_XY.copy()
    xy[1] = REFERENCE_OBJECT_XY[name]
    return xy, REFERENCE_OBJECT_YAW.get(name)


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
    if kind == "cylinder":
        # Convert the authored half-extents to MuJoCo radius and half-length.
        size = (size[0], size[2], 0.0)
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
    lid_panel = _add_geom(
        lid,
        name="packaging_lid_panel",
        kind="box",
        size=(half_x, half_y, panel),
        pos=[0, half_y, 0],
        rgba=CARDBOARD,
        density=lid_density,
    )
    # A thin box can miss deep overlaps at zero margin. Equal gap preserves the force boundary.
    lid_panel.margin = panel
    lid_panel.gap = panel

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
    return ((kind, size, (0, 0, 0), color, (0, 0, 0, 1)),), size[2]


def _add_cloth(spec, xy):
    # The published RGB-D footprint and height describe a folded textile, not
    # a solid cuboid. Ten connected accordion plies preserve that measured
    # spatial envelope while leaving both boundaries physically free; camera
    # visibility depends on the view and is validated separately.
    spec.option.timestep = 0.001

    half_x, half_y = CLOTH_HALF_XY
    vertices = []
    previous_end_y = None
    previous_xs = None
    row_count = 0
    for layer in range(CLOTH_LAYER_COUNT):
        x_fraction, y_fraction = CLOTH_LAYER_XY_FRACTIONS[layer]
        x_offset, y_offset = CLOTH_LAYER_CENTER_OFFSETS[layer]
        layer_xs = np.linspace(
            xy[0] + x_offset - half_x * x_fraction,
            xy[0] + x_offset + half_x * x_fraction,
            CLOTH_GRID_SIZE,
        )
        panel_ys = np.linspace(
            xy[1] + y_offset - half_y * y_fraction,
            xy[1] + y_offset + half_y * y_fraction,
            CLOTH_GRID_SIZE,
        )
        if layer % 2:
            panel_ys = panel_ys[::-1]
        z = TABLE_HEIGHT + CLOTH_RADIUS + 0.0007 + layer * CLOTH_LAYER_GAP
        if layer:
            vertices.extend((x, previous_end_y, z) for x in previous_xs)
            row_count += 1
            panel_ys = panel_ys[1:]
        for y in panel_ys:
            vertices.extend((x, y, z) for x in layer_xs)
            row_count += 1
        previous_end_y = panel_ys[-1]
        previous_xs = layer_xs

    elements = []
    for row in range(row_count - 1):
        for column in range(CLOTH_GRID_SIZE - 1):
            a = row * CLOTH_GRID_SIZE + column
            b = a + 1
            c = a + CLOTH_GRID_SIZE
            d = c + 1
            elements.extend(((a, b, d), (a, d, c)))
    points_xml = " ".join(f"{value:.9g}" for vertex in vertices for value in vertex)
    elements_xml = " ".join(str(value) for face in elements for value in face)
    color = PACKAGING_OBJECTS[CLOTH_OBJECT][2]
    child = mujoco.MjSpec.from_string(
        f"""
        <mujoco model="dexcap_packaging_cloth">
          <worldbody>
            <flexcomp name="packaging_object" type="direct" dim="2"
                      point="{points_xml}" element="{elements_xml}"
                      radius="{CLOTH_RADIUS}" mass="0.12" dof="full"
                      rgba="{' '.join(map(str, color))}">
              <elasticity young="{CLOTH_YOUNG_MODULUS_PA}" poisson="{CLOTH_POISSON_RATIO}"
                          damping="{CLOTH_DAMPING_S}" thickness="{CLOTH_THICKNESS}"
                          elastic2d="both"/>
              <contact selfcollide="auto" vertcollide="true" activelayers="0"
                       contype="1" conaffinity="1" friction="1.0 0.01 0.001"
                       solref="0.018 1" solimp="0.95 0.99 0.001"/>
            </flexcomp>
          </worldbody>
        </mujoco>
        """
    )
    spec.attach(child, frame=spec.worldbody.add_frame(name="packaging_cloth_scene"), prefix="")


def _add_red_box_hanger(body, rgba):
    # Outside and slot bounds in the carton frame from frame_2710.
    x0, x1, z0, z1 = 0.061, 0.099, -0.046, 0.037
    slot_x0, slot_x1, slot_z0, slot_z1 = 0.072, 0.084, -0.020, 0.008
    thickness = 0.0014
    solid_area = (x1 - x0) * (z1 - z0) - (slot_x1 - slot_x0) * (slot_z1 - slot_z0)
    density = 0.002 / (solid_area * thickness)
    panels = (
        (x0, slot_x0, z0, z1),
        (slot_x1, x1, z0, z1),
        (slot_x0, slot_x1, z0, slot_z0),
        (slot_x0, slot_x1, slot_z1, z1),
    )
    for i, (a, b, c, d) in enumerate(panels):
        _add_geom(
            body, name=f"packaging_object_hanger_{i}", kind="box",
            size=[(b - a) / 2, thickness / 2, (d - c) / 2],
            pos=[(a + b) / 2, -0.019, (c + d) / 2], rgba=rgba, density=density,
        )


def _add_spring_clamp(body, parts):
    density = 0.12 / sum(_geom_volume(part.kind, part.half_size) for part in parts)
    pivot = np.asarray(parts[0].center)
    moving = body.add_body(name="packaging_object_clamp_jaw", pos=pivot)
    moving.add_joint(
        name="packaging_object_clamp_hinge",
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[0, 0, 1],
        limited=True,
        range=[0, SPRING_CLAMP_MAX_ANGLE],
        springref=-SPRING_CLAMP_PRELOAD_ANGLE,
        stiffness=SPRING_CLAMP_STIFFNESS_NM_PER_RAD,
        damping=SPRING_CLAMP_DAMPING_NMS_PER_RAD,
        solref_limit=[0.004, 1],
        solimp_limit=[0.95, 0.995, 0.001, 0.5, 2],
    )
    for index, part in enumerate(parts):
        # The hub and the first four side pieces move together as one jaw.
        mount = body if index < 5 else moving
        pos = part.center if index < 5 else np.asarray(part.center) - pivot
        _add_geom(
            mount,
            name=f"packaging_object_part_{index}",
            kind=part.kind,
            size=part.half_size,
            pos=pos,
            quat=_wxyz(part.orientation),
            rgba=part.color,
            density=density,
        )


def _add_object(spec, xy, object_name, object_yaw):
    if object_name == CLOTH_OBJECT:
        _add_cloth(spec, xy)
        return
    definitions, support_height = _object_definition(object_name)
    if any(part[0] == "cylinder" for part in definitions):
        # Flat cylinder ends need several contact points to avoid rocking on a single point.
        spec.option.enableflags |= int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
    if object_yaw is None:
        object_yaw = RED_BOX_YAW if object_name == DEFAULT_OBJECT else 0
    body = spec.worldbody.add_body(
        name="packaging_object",
        pos=[*xy, TABLE_HEIGHT + support_height],
        quat=_yaw(object_yaw),
    )
    body.add_freejoint(name="packaging_object_free")

    if object_name in UNSEEN_COMPOUNDS:
        parts = definitions
        if object_name == "spring-clamp":
            _add_spring_clamp(body, parts)
        else:
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
        mass = 0.118 if object_name == DEFAULT_OBJECT else 0.12
        density = mass / _geom_volume(kind, size)
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
            _add_red_box_hanger(body, color)
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
        elif object_name == "tape-measure":
            # RGB-D shows a dark case with a yellow band and an off-center blue label.
            for name, layer_size, layer_pos, rgba in (
                ("band", [size[0] + .0001, .0035, 0], [0, 0, size[2] - .007], [.72, .59, .14, 1]),
                ("label", [.017, .00015, 0], [.012, -.012, size[2] + .0001], [.095, .31, .54, 1]),
            ):
                body.add_geom(
                    name=f"packaging_object_tape_{name}",
                    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=layer_size,
                    pos=layer_pos,
                    rgba=rgba,
                    contype=0,
                    conaffinity=0,
                    mass=0,
                )

    if object_name in ("green-pear", "red-apple"):
        skin_name = "packaging_object_part_0" if object_name == "red-apple" else "packaging_object_geom"
        sliding, torsional, rolling = spec.geom(skin_name).friction
        spec.add_pair(
            name="packaging_fruit_floor",
            geomname1=skin_name,
            geomname2="packaging_box_floor",
            condim=6,
            friction=[sliding, sliding, torsional, rolling, rolling],
        )


def add_packaging_scene(spec: mujoco.MjSpec, object_xy, object_name=DEFAULT_OBJECT, *, object_yaw=None):
    """Add the freely moving box, hinged lid, and selected free object to ``spec``."""
    if object_name not in PACKAGING_OBJECTS and object_name not in PACKAGING_UNSEEN_OBJECTS:
        raise ValueError(f"Unknown Packaging object: {object_name}")
    positions = np.asarray(object_xy, dtype=float)
    if positions.shape != (2, 2):
        raise ValueError("object_xy must contain box and object XY positions with shape (2, 2)")
    spec.compiler.degree = False
    _add_box(spec, positions[0])
    _add_object(spec, positions[1], object_name, object_yaw)


class PackagingMetrics:
    """Score centering, placement, and coordinated lid closure."""

    def __init__(self, model: mujoco.MjModel, object_name: str):
        if object_name not in PACKAGING_OBJECTS and object_name not in PACKAGING_UNSEEN_OBJECTS:
            raise ValueError(f"Unknown Packaging object: {object_name}")
        self.model = model
        self.box_body = model.body("packaging_box").id
        self.box_joint = model.joint("packaging_box_free").id
        self.lid_body = model.body("packaging_lid").id
        self.lid_panel = model.geom("packaging_lid_panel").id
        self.lid_joint = model.joint("packaging_lid_hinge").id
        self.lid_qpos = model.jnt_qposadr[self.lid_joint]
        self.lid_dof = model.jnt_dofadr[self.lid_joint]
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
            for body in range(model.nbody):
                if model.body(body).name.startswith("packaging_object_"):
                    self.object_geoms.update(self._body_geoms(body, collidable_only=True))
        else:
            self.object_body = model.body("packaging_object").id
            object_joint = model.body_jntadr[self.object_body]
            self.initial_object_z = model.qpos0[model.jnt_qposadr[object_joint] + 2]
            object_bodies = {self.object_body}
            for body in range(self.object_body + 1, model.nbody):
                if model.body_parentid[body] in object_bodies:
                    object_bodies.add(body)
            self.object_geoms = set().union(*(
                self._body_geoms(body, collidable_only=True) for body in object_bodies
            ))
            self.object_dofs = np.concatenate([
                np.arange(model.body_dofadr[body], model.body_dofadr[body] + model.body_dofnum[body])
                for body in object_bodies
            ])
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
        self.released_in_box = False
        self.placed_in_centered_box = False
        self.bimanual_lid_closure = False

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

    def _object_contacts(self, data, geoms):
        if self.object_flex >= 0:
            result = []
            for index in range(data.ncon):
                contact = data.contact[index]
                first_is_object = (
                    contact.flex[0] == self.object_flex or contact.geom[0] in self.object_geoms
                )
                second_is_object = (
                    contact.flex[1] == self.object_flex or contact.geom[1] in self.object_geoms
                )
                if contact.geom[0] in geoms and second_is_object:
                    result.append((contact, np.asarray(contact.frame[:3])))
                elif contact.geom[1] in geoms and first_is_object:
                    result.append((contact, -np.asarray(contact.frame[:3])))
            return result
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

    def _geom_extent_in_frame(self, data, geom, origin, frame_rotation):
        geom_rotation = data.geom_xmat[geom].reshape(3, 3)
        rotation = frame_rotation.T @ geom_rotation
        size = self.model.geom_size[geom]
        geom_type = self.model.geom_type[geom]
        if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            axis = np.abs(rotation[:, 2])
            extent = size[0] * np.sqrt(np.maximum(0, 1 - axis**2)) + size[1] * axis
        elif geom_type in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
            extent = np.sqrt((rotation**2) @ (size**2))
        else:
            extent = np.abs(rotation) @ size
        center = frame_rotation.T @ (data.geom_xpos[geom] - origin)
        return center - extent, center + extent

    def _object_in_box(self, data):
        # Settled soft contacts can overlap the inner wall by micrometers.
        inner_half = BOX_INNER_HALF_XY + BOX_CONTACT_TOLERANCE_M
        box_position = data.xpos[self.box_body]
        box_rotation = data.xmat[self.box_body].reshape(3, 3)
        if self.object_flex >= 0:
            local = (self._flex_vertices(data) - box_position) @ box_rotation
            lower = np.min(local, axis=0) - self.flex_radius
            upper = np.max(local, axis=0) + self.flex_radius
            inside = np.all(lower[:2] >= -inner_half) and np.all(upper[:2] <= inner_half)
            supported = bool(self._object_contacts(data, {self.floor_geom}))
            settled = np.max(np.abs(data.qvel[self.object_dofs])) < 0.05
            return bool(inside and supported and settled)

        bounds = [
            self._geom_extent_in_frame(data, geom, box_position, box_rotation)
            for geom in self.object_geoms
        ]
        lower = np.min([bound[0] for bound in bounds], axis=0)
        upper = np.max([bound[1] for bound in bounds], axis=0)
        inside = np.all(lower[:2] >= -inner_half) and np.all(upper[:2] <= inner_half)
        supported = bool(self._contacts(data, self.object_geoms, {self.floor_geom}))
        settled = np.linalg.norm(data.qvel[self.object_dofs]) < 0.05
        return bool(inside and supported and settled)

    def _object_clears_closed_lid(self, data):
        # The panel's local +Z faces into the box once the lid is closed.
        rotation = data.geom_xmat[self.lid_panel].reshape(3, 3)
        inward = rotation[:, 2]
        panel_inside = data.geom_xpos[self.lid_panel] + inward * self.model.geom_size[self.lid_panel, 2]
        if self.object_flex >= 0:
            clearance = ((self._flex_vertices(data) - panel_inside) @ inward).min() - self.flex_radius
        else:
            clearance = min(
                self._geom_extent_in_frame(data, geom, panel_inside, rotation)[0][2]
                for geom in self.object_geoms
            )
        return clearance >= -LID_CLEARANCE_TOLERANCE_M

    def physics_step(self, data: mujoco.MjData) -> None:
        if (self.bimanual_lid_closure or not self.released_in_box
                or data.qvel[self.lid_dof] <= 0
                or data.qpos[self.lid_qpos] >= np.pi - LID_CLOSED_TOLERANCE):
            return
        supporting_hands = set()
        lid_hands = set()
        force = np.empty(6)
        for index, contact in enumerate(data.contact[:data.ncon]):
            first, second = contact.geom1, contact.geom2
            if second in self.box_geoms or second in self.lid_geoms:
                hand, surface = first, second
            elif first in self.box_geoms or first in self.lid_geoms:
                hand, surface = second, first
            else:
                continue
            if hand in self.right_geoms:
                side = 1
            elif hand in self.left_geoms:
                side = -1
            else:
                continue
            mujoco.mj_contactForce(self.model, data, index, force)
            if force[0] <= LID_CONTACT_FORCE_N:
                continue
            if surface in self.box_geoms:
                supporting_hands.add(side)
            else:
                lid_hands.add(side)
        self.bimanual_lid_closure = any(-side in lid_hands for side in supporting_hands)

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
        if object_grasped:
            self.released_in_box = False
            self.placed_in_centered_box = False
            self.bimanual_lid_closure = False
        elif not self.released_in_box and object_in_box and object_released and not right_object:
            self.released_in_box = True
            box_axes_xy = data.xmat[self.box_body].reshape(3, 3)[:2, :2]
            local_center = (WORKSPACE_CENTER - box_xy) @ box_axes_xy
            self.placed_in_centered_box = bool(np.all(np.abs(local_center) <= BOX_INNER_HALF_XY))
        lid_angle = data.qpos[self.lid_qpos]
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
            "placed_in_centered_box": self.placed_in_centered_box,
            "lid_closed": bool(lid_closed),
            "bimanual_lid_closure": self.bimanual_lid_closure,
            "sim_success": bool(self.placed_in_centered_box and object_in_box
                                and lid_closed and self.bimanual_lid_closure
                                and self._object_clears_closed_lid(data)),
        }
