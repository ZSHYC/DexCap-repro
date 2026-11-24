"""Physical MuJoCo approximation of DexCap Scissor Cutting.

Appendix A-F establishes the divided container, separate-handle scissor grasp,
pink paper tape, two-hand lift, and a successful cut longer than 3 mm. Figure
5(e) further shows the scissor rings beyond a short tray end, with the pivot on
the wall and the blades inside. The tape leans against the other short wall.
The release provides no CAD, exact dimensions,
masses, material law, or trajectory. The dimensions and calibrated resting pose
below are therefore visual and physical-fit estimates. MuJoCo has no native
arbitrary tearing, so the tape has an explicit cohesive seam whose local bonds
release only under opposing blade contact during physical hinge closure.
"""

from __future__ import annotations

import mujoco
import numpy as np


TABLE_HEIGHT = -0.030
SCISSOR_DEFAULT_POSITION = np.array([
    0.4099535585178729, 0.18993592402601608, 0.005504892349298587,
])
SCISSOR_DEFAULT_QUATERNION = np.array([
    0.6993540402338588, 0.10171915241320749,
    0.10170667927428328, -0.7001520490807083,
])
TRAY_CENTER = np.array([0.50, 0.0])
TRAY_HALF_XY = (0.18, 0.20)
TRAY_WALL_HEIGHT = 0.035
TRAY_THICKNESS = 0.003
SCISSOR_OPEN_ANGLE = 0.45
# LEAP mesh AABBs measured from the released collision assets are
# 34.7 x 36.3 mm for the thumb and 21.6 x 34.6 mm per fingertip.
THUMB_OPENING_XY = np.array([0.040, 0.042])
TWO_FINGER_OPENING_XY = np.array([0.050, 0.040])
HANDLE_BAR_HALF_WIDTH = 0.004
HANDLE_HALF_HEIGHT = 0.001
FIXED_HANDLE_CENTER = np.array([-0.105, 0.035])
MOVING_HANDLE_CENTER = np.array([-0.100, -0.035])
PAPER_HALF_LENGTH = 0.030
PAPER_HALF_THICKNESS = 0.0004
PAPER_LEAN = np.deg2rad(8)
PAPER_STIFFNESS = 1.0e5
PAPER_SOLREF = [-PAPER_STIFFNESS, -2 * np.sqrt(PAPER_STIFFNESS)]
PAPER_SOLIMP = [0.998, 0.9999, 0.0001, 0.5, 2]
# The tape's lower edge meets the tray floor and its face meets the inner rim.
PAPER_SEAM_Y = (
    TRAY_CENTER[1] - TRAY_HALF_XY[1] + TRAY_THICKNESS
    + TRAY_WALL_HEIGHT * np.tan(PAPER_LEAN)
    + PAPER_HALF_THICKNESS / np.cos(PAPER_LEAN)
    - 2 * PAPER_HALF_LENGTH * np.sin(PAPER_LEAN)
)
DEMO_OBJECT_XY = np.array([SCISSOR_DEFAULT_POSITION[:2], [0.41, PAPER_SEAM_Y]])
BOND_COUNT = 5
BOND_PITCH = 0.004
BOND_BREAK_FORCE_N = 0.05
GRASP_CONTACT_FORCE_N = 0.05
CUT_CONTACT_RADIUS_M = 0.006
SUCCESS_CUT_M = 0.003


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


def _add_handle(body, prefix, z, center, opening):
    color = [0.02, 0.60, 0.62, 1]
    x, y = center
    half_x, half_y = opening / 2
    bar = HANDLE_BAR_HALF_WIDTH
    for suffix, size, pos in (
        ("top", [half_x + bar, bar, HANDLE_HALF_HEIGHT],
         [x, y + half_y + bar, z]),
        ("bottom", [half_x + bar, bar, HANDLE_HALF_HEIGHT],
         [x, y - half_y - bar, z]),
        ("left", [bar, half_y + bar, HANDLE_HALF_HEIGHT],
         [x - half_x - bar, y, z]),
        ("right", [bar, half_y + bar, HANDLE_HALF_HEIGHT],
         [x + half_x + bar, y, z]),
    ):
        _box(body, f"{prefix}_{suffix}", size, pos, color, density=720)


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
    _add_handle(base, "scissor_fixed_handle", -0.0015,
                FIXED_HANDLE_CENTER, TWO_FINGER_OPENING_XY)
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
        ref=SCISSOR_OPEN_ANGLE, damping=0.002, armature=0.0001,
    )
    _box(moving, "scissor_moving_blade", [0.060, 0.0025, 0.0008],
         [0.060, 0, 0.0006], teal, density=7800)
    _add_handle(moving, "scissor_moving_handle", 0.0015,
                MOVING_HANDLE_CENTER, THUMB_OPENING_XY)
    _add_shank(moving, "scissor_moving_shank", 0.0015,
               MOVING_HANDLE_CENTER, THUMB_OPENING_XY)


def _add_paper(spec, xy):
    pink = [0.98, 0.25, 0.55, 1]
    segment_half_width = BOND_PITCH * 0.45
    along = np.array([0.0, -np.sin(PAPER_LEAN), np.cos(PAPER_LEAN)])
    across = np.array([1.0, 0.0, 0.0])
    rotation = np.column_stack([along, across, np.cross(along, across)])
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    foot_z = TABLE_HEIGHT + PAPER_HALF_THICKNESS * np.sin(PAPER_LEAN) + 0.00005
    seam_z = foot_z + 2 * PAPER_HALF_LENGTH * along[2]
    bodies = {"left": [], "right": []}
    for index in range(BOND_COUNT):
        seam = np.array([*xy, seam_z]) + across * (index - (BOND_COUNT - 1) / 2) * BOND_PITCH
        names = []
        for side, direction in (("left", -1), ("right", 1)):
            name = f"cutting_paper_{side}_{index}"
            body = spec.worldbody.add_body(
                name=name,
                pos=seam + direction * PAPER_HALF_LENGTH * along, quat=quat,
            )
            body.add_freejoint(name=f"{name}_free")
            paper = _box(
                body, f"{name}_geom",
                [PAPER_HALF_LENGTH, segment_half_width, PAPER_HALF_THICKNESS],
                [0, 0, 0], pink, density=180,
            )
            # Firm paper contacts avoid slow slip while the tape rests on the rim.
            paper.solmix = 12
            paper.solimp = [0.985, 0.998, 0.00005, 0.5, 2]
            names.append(name)
            bodies[side].append(name)
        spec.add_equality(
            name=f"paper_bond_{index}", type=mujoco.mjtEq.mjEQ_WELD,
            objtype=mujoco.mjtObj.mjOBJ_BODY, name1=names[0], name2=names[1],
            data=[-PAPER_HALF_LENGTH, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0.04],
            solref=PAPER_SOLREF, solimp=PAPER_SOLIMP,
        )
    for side, names in bodies.items():
        for index, (first, second) in enumerate(zip(names, names[1:])):
            spec.add_equality(
                name=f"paper_{side}_link_{index}",
                type=mujoco.mjtEq.mjEQ_WELD,
                objtype=mujoco.mjtObj.mjOBJ_BODY,
                name1=first, name2=second,
                data=[0, -BOND_PITCH / 2, 0, 0, 1, 0, 0, 0, 0, 0, 0.024],
                solref=PAPER_SOLREF, solimp=PAPER_SOLIMP,
            )


def add_scissor_cutting_scene(spec: mujoco.MjSpec, object_xy=DEMO_OBJECT_XY):
    """Add fixed tray, passive free scissors, and seamed free paper tape."""
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
    return bool(len(section) and np.all(np.abs(section) <= half_opening))


class ScissorCuttingMetrics:
    """Release seam bonds from local opposing-blade contact and report progress."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.fixed_blade = model.geom("scissor_fixed_blade").id
        self.moving_blade = model.geom("scissor_moving_blade").id
        hinge = model.joint("scissor_hinge")
        self.hinge_dof = hinge.dofadr[0]
        self.scissor_body = model.body("scissor_base").id
        self.paper_bodies = []
        self.paper_geoms = set()
        self.geom_bond = {}
        self.geom_body = {}
        self.geom_anchor = {}
        self.bond_equalities = []
        for index in range(BOND_COUNT):
            self.bond_equalities.append(model.equality(f"paper_bond_{index}").id)
            for side in ("left", "right"):
                body = model.body(f"cutting_paper_{side}_{index}").id
                geom = model.geom(f"cutting_paper_{side}_{index}_geom").id
                self.paper_bodies.append(body)
                self.paper_geoms.add(geom)
                self.geom_bond[geom] = index
                self.geom_body[geom] = body
                self.geom_anchor[geom] = np.array([
                    PAPER_HALF_LENGTH if side == "left" else -PAPER_HALF_LENGTH,
                    0, 0,
                ])
        self.tray_geoms = self._geoms("cutting_tray_")
        self.fixed_handle = self._geoms("scissor_fixed_handle_")
        self.moving_handle = self._geoms("scissor_moving_handle_")
        self.movable_geoms = self._geoms("scissor_") | self.paper_geoms
        self.support_geoms = self.tray_geoms | {
            geom for geom in range(model.ngeom)
            if model.geom(geom).name in ("table", "room_floor_surface")
        }
        left_links = {
            int(model.geom(geom).name.rsplit("_", 1)[1]): geom
            for geom in self._geoms("left_hand_collision_")
        }
        self.left_index = [left_links[i] for i in range(1, 5) if i in left_links]
        self.left_middle = [left_links[i] for i in range(5, 9) if i in left_links]
        self.left_thumb = {left_links[i] for i in range(13, 17) if i in left_links}
        self.left_fingers = set(self.left_index + self.left_middle)
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
            geom: int(model.geom(geom).name.rsplit("_", 1)[1]) >= 13
            for geom in self._geoms("right_hand_collision_")
        }
        self.right_pads.update(dict.fromkeys(self._geoms("right_hand_palm_collision_"), False))
        self.held_bonds = 0
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
        handle_rotation = data.geom_xmat[top].reshape(3, 3)
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

    def physics_step(self, data: mujoco.MjData) -> None:
        """Apply cohesive failure from contacts produced by the latest substep."""
        contacts = {index: {} for index in range(BOND_COUNT)}
        force = np.zeros(6)
        blades = {self.fixed_blade, self.moving_blade}
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            pair = {contact.geom1, contact.geom2}
            blade = next((geom for geom in blades if geom in pair), None)
            paper = next((geom for geom in self.paper_geoms if geom in pair), None)
            if blade is None or paper is None:
                continue
            body = self.geom_body[paper]
            seam = (
                data.xpos[body]
                + data.xmat[body].reshape(3, 3) @ self.geom_anchor[paper]
            )
            if np.linalg.norm(np.asarray(contact.pos) - seam) > CUT_CONTACT_RADIUS_M:
                continue
            mujoco.mj_contactForce(self.model, data, contact_index, force)
            normal_force = abs(float(force[0]))
            bond = self.geom_bond[paper]
            contacts[bond][blade] = contacts[bond].get(blade, 0.0) + normal_force
            self.max_blade_force = max(self.max_blade_force, normal_force)

        closing_speed = max(0.0, -float(data.qvel[self.hinge_dof]))
        held_for_cut = None
        for bond, blade_forces in contacts.items():
            equality = self.bond_equalities[bond]
            opposed_force = min(
                blade_forces.get(self.fixed_blade, 0.0),
                blade_forces.get(self.moving_blade, 0.0),
            )
            if (data.eq_active[equality] and closing_speed > 0.01
                    and opposed_force >= BOND_BREAK_FORCE_N):
                if held_for_cut is None:
                    held_for_cut = self._held_for_cut(data)
                self.held_bonds += int(held_for_cut)
                data.eq_active[equality] = 0

    def update(self, data: mujoco.MjData) -> dict:
        scissor_lift = float(data.xpos[self.scissor_body, 2] - self.initial_scissor_z)
        paper_lift = float(np.mean(data.xpos[self.paper_bodies, 2]) - self.initial_paper_z)
        self.max_scissor_lift = max(self.max_scissor_lift, scissor_lift)
        self.max_paper_lift = max(self.max_paper_lift, paper_lift)
        broken = sum(not data.eq_active[equality] for equality in self.bond_equalities)
        cut_length = broken * BOND_PITCH
        return {
            "blade_contact_force_n": float(self.max_blade_force),
            "broken_bonds": int(broken),
            "cut_length_m": float(cut_length),
            "held_cut_length_m": float(self.held_bonds * BOND_PITCH),
            "scissors_lift_m": float(self.max_scissor_lift),
            "paper_lift_m": float(self.max_paper_lift),
            "scissors_lifted": self.max_scissor_lift > 0.01,
            "paper_lifted": self.max_paper_lift > 0.01,
            "sim_success": self.held_bonds * BOND_PITCH > SUCCESS_CUT_M,
        }
