"""Physical MuJoCo approximation of DexCap Scissor Cutting.

Appendix A-F establishes the divided container, separate-handle scissor grasp,
pink paper tape, two-hand lift, and a successful cut longer than 3 mm. Figure
5(e) further shows the scissor rings beyond a short tray end, with the pivot on
the wall and the blades inside. The first frame of the official task video
shows both rings above the end rim. The tape leans against the other short
wall. The release provides no CAD, exact dimensions, masses, material law, or
trajectory. The dimensions and resting pose are visual and physical-fit
estimates. MuJoCo has no native
arbitrary tearing, so the tape has an explicit cohesive seam whose local bonds
release only under opposing blade contact during physical hinge closure.
"""

from __future__ import annotations

import mujoco
import numpy as np


TABLE_HEIGHT = -0.030
# Free-body pose measured after settling on the rim and divider.
SCISSOR_DEFAULT_POSITION = np.array([0.4121830696, 0.1716311354, 0.0318467208])
SCISSOR_DEFAULT_QUATERNION = np.array([
    -0.6580609833, -0.1966494774, -0.1727617294, 0.7060014945,
])
TRAY_CENTER = np.array([0.50, 0.0])
TRAY_HALF_XY = (0.18, 0.20)
TRAY_WALL_HEIGHT = 0.075
TRAY_THICKNESS = 0.003
SCISSOR_OPEN_ANGLE = 0.3316140037
# Oval opening dimensions and lever lengths are estimated from the video.
THUMB_OPENING_XY = np.array([0.040, 0.037])
TWO_FINGER_OPENING_XY = np.array([0.080, 0.040])
HANDLE_BAR_HALF_WIDTH = 0.004
HANDLE_HALF_HEIGHT = 0.001
FIXED_HANDLE_CENTER = np.array([-0.082, 0.032])
MOVING_HANDLE_CENTER = np.array([-0.075, -0.032])
PAPER_LENGTH = 0.120
PAPER_COLUMNS = 10
PAPER_ROWS = 5
PAPER_CELL_LENGTH = PAPER_LENGTH / PAPER_COLUMNS
PAPER_CELL_WIDTH = 0.004
PAPER_HALF_THICKNESS = 0.0004
PAPER_LEAN = np.deg2rad(8)
PAPER_STIFFNESS = 1.0e5
PAPER_SOLREF = [-PAPER_STIFFNESS, -2 * np.sqrt(PAPER_STIFFNESS)]
PAPER_SOLIMP = [0.998, 0.9999, 0.0001, 0.5, 2]
# The tape's lower edge meets the tray floor and its face meets the inner rim.
PAPER_CENTER_Y = (
    TRAY_CENTER[1] - TRAY_HALF_XY[1] + TRAY_THICKNESS
    + TRAY_WALL_HEIGHT * np.tan(PAPER_LEAN)
    + PAPER_HALF_THICKNESS / np.cos(PAPER_LEAN)
    - PAPER_LENGTH / 2 * np.sin(PAPER_LEAN)
)
DEMO_OBJECT_XY = np.array([SCISSOR_DEFAULT_POSITION[:2], [0.41, PAPER_CENTER_Y]])
BOND_BREAK_FORCE_N = 0.05
GRASP_CONTACT_FORCE_N = 0.05
CUT_CONTACT_RADIUS_M = 0.006
SUCCESS_CUT_M = 0.003


def sample_object_xy(rng):
    positions = DEMO_OBJECT_XY.copy()
    # Shift along the supports so the scissors and upright tape keep their seating.
    positions[:, 0] += rng.uniform([-.008, -.035], [.008, .035])
    return positions


def _box(body, name, size, pos, rgba, *, density=500, quat=(1, 0, 0, 0)):
    return body.add_geom(
        name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=pos,
        quat=quat, rgba=rgba, density=density, friction=[0.8, 0.01, 0.001],
        solref=[0.004, 1], solimp=[0.9, 0.98, 0.001, 0.5, 2],
    )


def _add_tray(spec):
    x, y = TRAY_CENTER
    hx, hy = TRAY_HALF_XY
    t, h = TRAY_THICKNESS, TRAY_WALL_HEIGHT
    tray = spec.worldbody.add_body(name="cutting_tray", pos=[x, y, TABLE_HEIGHT])
    color = [0.36, 0.25, 0.16, 1]
    _box(tray, "cutting_tray_floor", [hx, hy, t], [0, 0, -t], color)
    for name, size, pos in (
        ("x_neg", [t, hy, h / 2], [-hx, 0, h / 2]),
        ("x_pos", [t, hy, h / 2], [hx, 0, h / 2]),
        ("y_neg", [hx, t, h / 2], [0, -hy, h / 2]),
        ("y_pos", [hx, t, h / 2], [0, hy, h / 2]),
        ("divider", [t, hy, h / 2], [0, 0, h / 2]),
        ("front_divider", [hx / 2, t, h / 2], [-hx / 2, 0.065, h / 2]),
    ):
        _box(tray, f"cutting_tray_{name}", size, pos, color)


def _add_handle(spec, body, prefix, z, center, opening, segments):
    angles = (np.arange(segments) - 0.5) * (2 * np.pi / segments)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    radii = opening / 2
    # Tangent chords preserve the specified clear ellipse inside the segmented ring.
    inner = center + directions * (radii / np.cos(np.pi / segments))
    outer = center + directions * (radii + 2 * HANDLE_BAR_HALF_WIDTH)
    sides = {0: "right", segments // 4: "top",
             segments // 2: "left", 3 * segments // 4: "bottom"}
    faces = np.array([
        (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
    ]).ravel()
    for index in range(segments):
        next_index = (index + 1) % segments
        xy = np.array([inner[index], outer[index], outer[next_index], inner[next_index]])
        top = np.column_stack((xy, np.full(4, z + 0.002)))
        bottom = top - [0.0, 0.0, 0.004]
        name = f"{prefix}_{sides.get(index, f'segment_{index:02d}')}"
        mesh = spec.add_mesh(
            name=name, uservert=np.concatenate((top, bottom)).ravel(), userface=faces,
        )
        body.add_geom(
            name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh.name,
            rgba=(0.02, 0.60, 0.62, 1), density=720, friction=(0.8, 0.01, 0.001),
            solref=(0.004, 1), solimp=(0.9, 0.98, 0.001, 0.5, 2),
        )


def _add_shank(body, name, z, center, opening):
    start = np.array([-0.004, center[1] * 0.12])
    end = np.array([center[0] + opening[0] / 2 + HANDLE_BAR_HALF_WIDTH, center[1]])
    delta = end - start
    angle = np.arctan2(delta[1], delta[0])
    _box(
        body, name,
        [np.linalg.norm(delta) / 2 + HANDLE_BAR_HALF_WIDTH,
         HANDLE_BAR_HALF_WIDTH, HANDLE_HALF_HEIGHT],
        [*((start + end) / 2), z], [0.02, 0.60, 0.62, 1], density=720,
        quat=[np.cos(angle / 2), 0, 0, np.sin(angle / 2)],
    )


def _add_scissors(spec, xy):
    base = spec.worldbody.add_body(
        name="scissor_base",
        pos=[*xy, SCISSOR_DEFAULT_POSITION[2]],
        quat=SCISSOR_DEFAULT_QUATERNION,
    )
    base.add_freejoint(name="scissor_free")
    teal = [0.02, 0.60, 0.62, 1]
    _box(base, "scissor_fixed_blade", [0.060, 0.0025, 0.0008],
         [0.060, 0, -0.0006], teal, density=7800)
    _add_handle(spec, base, "scissor_fixed_handle", -0.0021,
                FIXED_HANDLE_CENTER, TWO_FINGER_OPENING_XY, 24)
    _add_shank(base, "scissor_fixed_shank", -0.0015,
               FIXED_HANDLE_CENTER, TWO_FINGER_OPENING_XY)
    base.add_geom(
        name="scissor_pivot", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[0.006, 0.002], rgba=[0.65, 0.65, 0.68, 1], density=7800,
    )

    moving = base.add_body(
        name="scissor_moving", pos=[0, 0, 0],
        quat=[np.cos(SCISSOR_OPEN_ANGLE / 2), 0, 0,
              np.sin(SCISSOR_OPEN_ANGLE / 2)],
    )
    moving.add_joint(
        name="scissor_hinge", type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[0, 0, 1], limited=True, range=[0.02, 0.55],
        ref=SCISSOR_OPEN_ANGLE, damping=0.002, frictionloss=0.002, armature=0.0001,
    )
    _box(moving, "scissor_moving_blade", [0.060, 0.0025, 0.0008],
         [0.060, 0, 0.0006], teal, density=7800)
    _add_handle(spec, moving, "scissor_moving_handle", 0.0021,
                MOVING_HANDLE_CENTER, THUMB_OPENING_XY, 20)
    _add_shank(moving, "scissor_moving_shank", 0.0015,
               MOVING_HANDLE_CENTER, THUMB_OPENING_XY)


def _add_paper(spec, xy):
    along = np.array([0.0, -np.sin(PAPER_LEAN), np.cos(PAPER_LEAN)])
    across = np.array([1.0, 0.0, 0.0])
    rotation = np.column_stack([along, across, np.cross(along, across)])
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    foot_z = TABLE_HEIGHT + PAPER_HALF_THICKNESS * np.sin(PAPER_LEAN) + 0.00005
    center = np.array([*xy, foot_z + PAPER_LENGTH / 2 * along[2]])
    cell_size = np.array([PAPER_CELL_LENGTH, PAPER_CELL_WIDTH])

    def name(column, row):
        return f"cutting_paper_{column}_{row}"

    for column in range(PAPER_COLUMNS):
        for row in range(PAPER_ROWS):
            offset = np.array([
                (column - (PAPER_COLUMNS - 1) / 2) * PAPER_CELL_LENGTH,
                (row - (PAPER_ROWS - 1) / 2) * PAPER_CELL_WIDTH,
                0,
            ])
            body = spec.worldbody.add_body(
                name=name(column, row), pos=center + rotation @ offset, quat=quat,
            )
            body.add_freejoint(name=f"{body.name}_free")
            paper = _box(
                body, f"{body.name}_geom",
                [*(cell_size / 2 - 0.00005), PAPER_HALF_THICKNESS],
                [0, 0, 0], [0.98, 0.25, 0.55, 1], density=180,
            )
            # Firm paper contacts avoid slow slip while the tape rests on the rim.
            paper.solmix = 12
            paper.solimp = [0.985, 0.998, 0.00005, 0.5, 2]

    for column in range(PAPER_COLUMNS):
        for row in range(PAPER_ROWS):
            neighbors = [(column + 1, row), (column, row + 1)]
            for axis, (next_column, next_row) in enumerate(neighbors):
                if next_column == PAPER_COLUMNS or next_row == PAPER_ROWS:
                    continue
                anchor = np.zeros(3)
                anchor[axis] = cell_size[axis] / 2
                spec.add_equality(
                    name=f"paper_bond_{column}_{row}_{axis}", type=mujoco.mjtEq.mjEQ_WELD,
                    objtype=mujoco.mjtObj.mjOBJ_BODY,
                    name1=name(column, row), name2=name(next_column, next_row),
                    # A smaller bending compliance keeps the finer tape upright on the rim.
                    data=[*(-anchor), *anchor, 1, 0, 0, 0, 0.12],
                    solref=PAPER_SOLREF, solimp=PAPER_SOLIMP,
                )


def add_scissor_cutting_scene(spec: mujoco.MjSpec, object_xy=DEMO_OBJECT_XY):
    """Add fixed tray, passive free scissors, and free bonded paper tape."""
    positions = np.asarray(object_xy, dtype=float)
    if positions.shape != (2, 2) or not np.isfinite(positions).all():
        raise ValueError("object_xy must contain finite scissors and paper XY positions")
    spec.compiler.degree = False
    _add_tray(spec)
    _add_scissors(spec, positions[0])
    _add_paper(spec, positions[1])


def _mesh_section_fits_opening(vertices, faces, half_opening):
    height = vertices[:, 2]
    if not height.min() < 0 < height.max():
        return False

    triangles = vertices[faces]
    start = triangles.reshape(-1, 3)
    end = triangles[:, [1, 2, 0], :].reshape(-1, 3)
    crossing = (start[:, 2] * end[:, 2]) < 0
    first, second = start[crossing], end[crossing]
    fraction = first[:, 2] / (first[:, 2] - second[:, 2])
    section = first[:, :2] + fraction[:, None] * (second[:, :2] - first[:, :2])
    section = np.concatenate((section, vertices[height == 0, :2]))
    return bool(len(section) and np.all(np.sum((section / half_opening) ** 2, axis=1) <= 1))


class ScissorCuttingMetrics:
    """Release paper bonds from local opposing-blade contact and report progress."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.fixed_blade = model.geom("scissor_fixed_blade").id
        self.moving_blade = model.geom("scissor_moving_blade").id
        hinge = model.joint("scissor_hinge")
        self.hinge_dof = hinge.dofadr[0]
        self.scissor_body = model.body("scissor_base").id
        paper_geom_by_body = {
            model.geom_bodyid[geom]: geom for geom in self._geoms("cutting_paper_")
        }
        self.paper_bodies = list(paper_geom_by_body)
        self.paper_geoms = set(paper_geom_by_body.values())
        self.geom_bonds = {geom: [] for geom in self.paper_geoms}
        self.bond_lengths = {}
        for equality in range(model.neq):
            if not model.equality(equality).name.startswith("paper_bond_"):
                continue
            first, second = model.eq_obj1id[equality], model.eq_obj2id[equality]
            second_anchor, first_anchor = model.eq_data[equality, :6].reshape(2, 3)
            tangent = 1 - int(np.argmax(np.abs(first_anchor)))
            length = (PAPER_CELL_LENGTH, PAPER_CELL_WIDTH)[tangent]
            self.bond_lengths[equality] = length
            self.geom_bonds[paper_geom_by_body[first]].append(
                (equality, first_anchor, tangent, length),
            )
            self.geom_bonds[paper_geom_by_body[second]].append(
                (equality, second_anchor, tangent, length),
            )
        self.tray_geoms = self._geoms("cutting_tray_")
        self.fixed_handle = self._geoms("scissor_fixed_handle_")
        self.moving_handle = self._geoms("scissor_moving_handle_")
        self.movable_geoms = self._geoms("scissor_") | self.paper_geoms
        self.support_geoms = self.tray_geoms | {
            geom for geom in range(model.ngeom)
            if model.geom(geom).name in ("table", "room_floor_surface")
        }
        left_links = {
            model.geom(geom).name.removeprefix("left_hand_collision_"): geom
            for geom in self._geoms("left_hand_collision_")
        }
        # A whole-link mesh measures passage even when its collision mesh is split.
        left_surface = {int(link): geom for link, geom in left_links.items() if link.isdecimal()}
        self.left_index = [left_surface[i] for i in range(1, 5) if i in left_surface]
        self.left_middle = [left_surface[i] for i in range(5, 9) if i in left_surface]
        self.left_thumb = {left_surface[i] for i in range(13, 17) if i in left_surface}
        self.left_fingers = {
            geom for link, geom in left_links.items() if 1 <= int(link.partition("_")[0]) <= 8
        }
        self.fixed_opening = (
            model.geom("scissor_fixed_handle_top").id,
            model.geom("scissor_fixed_handle_bottom").id,
            TWO_FINGER_OPENING_XY / 2,
        )
        self.moving_opening = (
            model.geom("scissor_moving_handle_top").id,
            model.geom("scissor_moving_handle_bottom").id,
            THUMB_OPENING_XY / 2,
        )
        self.right_pads = {
            geom: int(model.geom(geom).name.removeprefix("right_hand_collision_").partition("_")[0]) >= 13
            for geom in self._geoms("right_hand_collision_")
        }
        self.right_pads.update(dict.fromkeys(self._geoms("right_hand_palm_collision_"), False))
        self.held_cut_length = 0.0
        self.initial_scissor_z = float(model.body_pos[self.scissor_body, 2])
        self.initial_paper_z = float(np.mean(model.body_pos[self.paper_bodies, 2]))
        self.max_scissor_lift = 0.0
        self.max_paper_lift = 0.0
        self.max_blade_force = 0.0

    def _geoms(self, prefix):
        return {geom for geom in range(self.model.ngeom) if self.model.geom(geom).name.startswith(prefix)}

    def _finger_in_opening(self, data, links, opening):
        top, bottom, half_opening = opening
        center = (data.geom_xpos[top] + data.geom_xpos[bottom]) / 2
        handle_body = self.model.geom_bodyid[top]
        handle_rotation = data.xmat[handle_body].reshape(3, 3)
        for geom in links:
            mesh = self.model.geom_dataid[geom]
            vertex_start = self.model.mesh_vertadr[mesh]
            face_start = self.model.mesh_faceadr[mesh]
            vertices = self.model.mesh_vert[
                vertex_start:vertex_start + self.model.mesh_vertnum[mesh]
            ]
            faces = self.model.mesh_face[
                face_start:face_start + self.model.mesh_facenum[mesh]
            ]
            link_rotation = data.geom_xmat[geom].reshape(3, 3)
            relative = data.geom_xpos[geom] - center
            local = (relative + vertices @ link_rotation.T) @ handle_rotation
            if _mesh_section_fits_opening(local, faces, half_opening):
                return True
        return False

    def _held_for_cut(self, data):
        thumb_engaged = fixed_engaged = False
        paper_normals = ([], [])
        force = np.zeros(6)
        for index, contact in enumerate(data.contact[:data.ncon]):
            pair = {contact.geom1, contact.geom2}
            if pair & self.support_geoms and pair & self.movable_geoms:
                return False
            if pair & self.moving_handle and pair & self.left_thumb:
                mujoco.mj_contactForce(self.model, data, index, force)
                thumb_engaged |= force[0] > GRASP_CONTACT_FORCE_N
            if pair & self.fixed_handle and pair & self.left_fingers:
                mujoco.mj_contactForce(self.model, data, index, force)
                fixed_engaged |= force[0] > GRASP_CONTACT_FORCE_N
            if pair & self.paper_geoms and (pads := pair & self.right_pads.keys()):
                pad = next(iter(pads))
                mujoco.mj_contactForce(self.model, data, index, force)
                if force[0] > GRASP_CONTACT_FORCE_N:
                    normal = contact.frame[:3] if contact.geom1 == pad else -contact.frame[:3]
                    paper_normals[self.right_pads[pad]].append(normal.copy())
        return (
            thumb_engaged and fixed_engaged
            and any(a @ b < -.25 for a in paper_normals[0] for b in paper_normals[1])
            and self._finger_in_opening(data, self.left_thumb, self.moving_opening)
            and self._finger_in_opening(data, self.left_index, self.fixed_opening)
            and self._finger_in_opening(data, self.left_middle, self.fixed_opening)
        )

    def _blade_crossing(self, data):
        fixed, moving = self.fixed_blade, self.moving_blade
        fixed_axes = data.geom_xmat[fixed].reshape(3, 3)[:, :2]
        moving_axes = data.geom_xmat[moving].reshape(3, 3)[:, :2]
        fixed_edge = data.geom_xpos[fixed] + fixed_axes[:, 1] * self.model.geom_size[fixed, 1]
        moving_edge = data.geom_xpos[moving] - moving_axes[:, 1] * self.model.geom_size[moving, 1]
        offset = fixed_axes.T @ (moving_edge - fixed_edge)
        direction = fixed_axes.T @ moving_axes[:, 0]
        if direction[1] <= 0:
            return None
        along_moving = -offset[1] / direction[1]
        along_fixed = offset[0] + along_moving * direction[0]
        if (abs(along_fixed) > self.model.geom_size[fixed, 0]
                or abs(along_moving) > self.model.geom_size[moving, 0]):
            return None
        return (fixed_edge + along_fixed * fixed_axes[:, 0]
                + moving_edge + along_moving * moving_axes[:, 0]) / 2

    def physics_step(self, data: mujoco.MjData) -> None:
        """Apply cohesive failure from contacts produced by the latest substep."""
        closing = data.qvel[self.hinge_dof] < -0.01
        crossing = self._blade_crossing(data) if closing else None
        contacts = {}
        force = np.zeros(6)
        blades = {self.fixed_blade, self.moving_blade}
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            first, second = contact.geom1, contact.geom2
            blade, paper = (first, second) if first in blades else (second, first)
            if blade not in blades or paper not in self.paper_geoms:
                continue
            mujoco.mj_contactForce(self.model, data, contact_index, force)
            normal_force = abs(float(force[0]))
            self.max_blade_force = max(self.max_blade_force, normal_force)
            if crossing is None or np.linalg.norm(contact.pos - crossing) > CUT_CONTACT_RADIUS_M:
                continue
            body = self.model.geom_bodyid[paper]
            local = data.xmat[body].reshape(3, 3).T @ (contact.pos - data.xpos[body])
            closest_bond, closest_distance = None, CUT_CONTACT_RADIUS_M
            for equality, anchor, tangent, length in self.geom_bonds[paper]:
                delta = local - anchor
                delta[tangent] -= np.clip(delta[tangent], -length / 2, length / 2)
                distance = np.linalg.norm(delta)
                if distance < closest_distance:
                    closest_bond, closest_distance = equality, distance
            if closest_bond is None:
                continue
            by_blade = contacts.setdefault(closest_bond, {})
            by_blade[blade] = by_blade.get(blade, 0.0) + normal_force

        held_for_cut = None
        for equality, blade_forces in contacts.items():
            opposed_force = min(
                blade_forces.get(self.fixed_blade, 0.0),
                blade_forces.get(self.moving_blade, 0.0),
            )
            length = self.bond_lengths[equality]
            break_force = BOND_BREAK_FORCE_N * length / PAPER_CELL_WIDTH
            if data.eq_active[equality] and opposed_force >= break_force:
                if held_for_cut is None:
                    held_for_cut = self._held_for_cut(data)
                if held_for_cut:
                    self.held_cut_length += length
                data.eq_active[equality] = 0

    def update(self, data: mujoco.MjData) -> dict:
        scissor_lift = float(data.xpos[self.scissor_body, 2] - self.initial_scissor_z)
        paper_lift = float(np.mean(data.xpos[self.paper_bodies, 2]) - self.initial_paper_z)
        self.max_scissor_lift = max(self.max_scissor_lift, scissor_lift)
        self.max_paper_lift = max(self.max_paper_lift, paper_lift)
        broken = [length for equality, length in self.bond_lengths.items()
                  if not data.eq_active[equality]]
        return {
            "blade_contact_force_n": float(self.max_blade_force),
            "broken_bonds": len(broken),
            "cut_length_m": sum(broken, 0.0),
            "held_cut_length_m": self.held_cut_length,
            "scissors_lift_m": float(self.max_scissor_lift),
            "paper_lift_m": float(self.max_paper_lift),
            "scissors_lifted": self.max_scissor_lift > 0.01,
            "paper_lifted": self.max_paper_lift > 0.01,
            "sim_success": self.held_cut_length > SUCCESS_CUT_M,
        }
