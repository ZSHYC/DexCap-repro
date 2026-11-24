"""Physical MuJoCo approximation of DexCap Scissor Cutting.

Appendix A-F establishes the divided container, separate-handle scissor grasp,
pink paper tape, two-hand lift, and a successful cut longer than 3 mm.  The
release provides no CAD, dimensions, masses, material law, or trajectory.  The
dimensions below are therefore visual assumptions.  MuJoCo has no native
arbitrary tearing, so the tape has an explicit cohesive seam whose local bonds
release only under opposing blade contact during physical hinge closure.
"""

from __future__ import annotations

import mujoco
import numpy as np


TABLE_HEIGHT = -0.030
DEMO_OBJECT_XY = np.array([[0.50, 0.12], [0.50, -0.12]])
TRAY_CENTER = np.array([0.50, 0.0])
TRAY_HALF_XY = (0.20, 0.18)
TRAY_WALL_HEIGHT = 0.035
TRAY_THICKNESS = 0.003
SCISSOR_OPEN_ANGLE = 0.45
PAPER_HALF_LENGTH = 0.030
PAPER_HALF_THICKNESS = 0.0004
BOND_COUNT = 5
BOND_PITCH = 0.004
BOND_BREAK_FORCE_N = 0.05
CUT_CONTACT_RADIUS_M = 0.006
SUCCESS_CUT_M = 0.003


def _box(body, name, size, pos, rgba, *, density=500):
    return body.add_geom(
        name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=pos,
        rgba=rgba, density=density, friction=[0.8, 0.01, 0.001],
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
        ("divider", [hx, t, h / 2], [0, 0, h / 2]),
    ):
        _box(tray, f"cutting_tray_{name}", size, pos, color)


def _add_handle(body, prefix, z):
    color = [0.02, 0.60, 0.62, 1]
    for suffix, size, pos in (
        ("top", [0.025, 0.003, 0.002], [-0.048, 0.014, z]),
        ("bottom", [0.025, 0.003, 0.002], [-0.048, -0.014, z]),
        ("left", [0.003, 0.011, 0.002], [-0.073, 0, z]),
        ("right", [0.003, 0.011, 0.002], [-0.023, 0, z]),
    ):
        _box(body, f"{prefix}_{suffix}", size, pos, color, density=720)


def _add_scissors(spec, xy):
    base = spec.worldbody.add_body(
        name="scissor_base", pos=[*xy, TABLE_HEIGHT + 0.001]
    )
    base.add_freejoint(name="scissor_free")
    teal = [0.02, 0.60, 0.62, 1]
    _box(base, "scissor_fixed_blade", [0.060, 0.0025, 0.0008],
         [0.060, 0, -0.0006], teal, density=7800)
    _add_handle(base, "scissor_fixed_handle", -0.0006)
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
    _add_handle(moving, "scissor_moving_handle", 0.0006)


def _add_paper(spec, xy):
    pink = [0.98, 0.25, 0.55, 1]
    segment_half_width = BOND_PITCH * 0.45
    z = TABLE_HEIGHT + PAPER_HALF_THICKNESS
    bodies = {"left": [], "right": []}
    for index in range(BOND_COUNT):
        y = xy[1] + (index - (BOND_COUNT - 1) / 2) * BOND_PITCH
        names = []
        for side, direction in (("left", -1), ("right", 1)):
            name = f"cutting_paper_{side}_{index}"
            body = spec.worldbody.add_body(
                name=name,
                pos=[xy[0] + direction * PAPER_HALF_LENGTH, y, z],
            )
            body.add_freejoint(name=f"{name}_free")
            _box(
                body, f"{name}_geom",
                [PAPER_HALF_LENGTH, segment_half_width, PAPER_HALF_THICKNESS],
                [0, 0, 0], pink, density=180,
            )
            names.append(name)
            bodies[side].append(name)
        spec.add_equality(
            name=f"paper_bond_{index}", type=mujoco.mjtEq.mjEQ_CONNECT,
            objtype=mujoco.mjtObj.mjOBJ_BODY, name1=names[0], name2=names[1],
            data=[PAPER_HALF_LENGTH, 0, 0, -PAPER_HALF_LENGTH, 0, 0,
                  0, 0, 0, 0, 0], solref=[0.004, 1],
        )
    for side, names in bodies.items():
        for index, (first, second) in enumerate(zip(names, names[1:])):
            spec.add_equality(
                name=f"paper_{side}_link_{index}",
                type=mujoco.mjtEq.mjEQ_CONNECT,
                objtype=mujoco.mjtObj.mjOBJ_BODY,
                name1=first, name2=second,
                data=[0, BOND_PITCH / 2, 0, 0, -BOND_PITCH / 2, 0,
                      0, 0, 0, 0, 0],
                solref=[0.015, 1],
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
        self.tray_geoms = {
            geom for geom in range(model.ngeom)
            if model.geom(geom).name.startswith("cutting_tray_")
        }
        self.initial_scissor_z = float(model.body_pos[self.scissor_body, 2])
        self.initial_paper_z = float(np.mean(model.body_pos[self.paper_bodies, 2]))
        self.max_scissor_lift = 0.0
        self.max_paper_lift = 0.0
        self.max_blade_force = 0.0

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
        for bond, blade_forces in contacts.items():
            equality = self.bond_equalities[bond]
            opposed_force = min(
                blade_forces.get(self.fixed_blade, 0.0),
                blade_forces.get(self.moving_blade, 0.0),
            )
            if (data.eq_active[equality] and closing_speed > 0.01
                    and opposed_force >= BOND_BREAK_FORCE_N):
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
            "scissors_lift_m": float(self.max_scissor_lift),
            "paper_lift_m": float(self.max_paper_lift),
            "scissors_lifted": self.max_scissor_lift > 0.01,
            "paper_lifted": self.max_paper_lift > 0.01,
            "sim_success": cut_length > SUCCESS_CUT_M,
        }
