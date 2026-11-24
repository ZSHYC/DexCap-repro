"""Physical Tea Preparing scene for DexCap.

The paper gives the action order but no Tea CAD, dimensions, masses, or material
calibration. SI dimensions here are Fig. 5(f) estimates: 60 mm bottle, 90 mm
teapot, 110 mm tweezers and 360 x 280 mm tray. The opening shot of the
official rollout gives the tray, tool rest and vessel arrangement. The cap
uses MuJoCo's hinge/slide equality as a 2 mm-pitch thread.
"""

from __future__ import annotations

import math
from itertools import combinations
from pathlib import Path

import mujoco
import numpy as np


TABLE_HEIGHT = -0.030
TRAY_YAW = np.deg2rad(-8)
TRAY_HALF_XY = (0.180, 0.140)
TRAY_ROTATION = np.array([
    [np.cos(TRAY_YAW), -np.sin(TRAY_YAW)],
    [np.sin(TRAY_YAW), np.cos(TRAY_YAW)],
])
TRAY_FLOOR_HALF = 0.012
TRAY_TOP = TABLE_HEIGHT + 2 * TRAY_FLOOR_HALF
BOTTLE_RADIUS, BOTTLE_INNER_RADIUS, BOTTLE_HEIGHT = 0.029, 0.024, 0.105
BOTTLE_WALL_THICKNESS = 0.002
BOTTLE_DENSITY = 1250
BOTTLE_NECK_RADIUS, BOTTLE_NECK_HEIGHT = 0.027, 0.014
CAP_RADIUS, CAP_HALF_HEIGHT = 0.035, 0.014
THREAD_PITCH, THREAD_TURNS = 0.002, 2.0
THREAD_LIMIT = 2 * np.pi * THREAD_TURNS
POT_INNER_RADIUS, POT_HEIGHT = 0.037, 0.060
TEA_PIECE_SIZE = (.0077, .0048, .0034)
TEA_DENSITY = 650
TEA_CHARGE_POSES = np.loadtxt(Path(__file__).with_name("assets") / "tea_charge_poses.txt")
TEA_PARTICLES = len(TEA_CHARGE_POSES)

# Tray-plane estimates from the first view, with wall clearance for the bottle.
TRAY_CENTER = np.array([0.52, 0.00])
TOOL_YAW = TRAY_YAW + math.atan2(.108, -.047)
DEMO_OBJECT_XY = TRAY_CENTER + np.array([
    [0, 0], [-.142, .105], [-.101, -.030], [.137, -.166],
]) @ TRAY_ROTATION.T
SUMMARY_KEYS = (
    "cap_turns", "cap_unthreaded", "cap_on_right", "tea_deposited_count",
    "tea_deposited", "bottle_used", "bottle_returned", "tweezers_used",
    "tweezers_returned", "left_hand_released", "sim_success",
)


def _yaw(angle):
    return [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]


def _ring(body, prefix, radius, height, thickness, color, density, *, z_center=None):
    z_center = height / 2 if z_center is None else z_center
    for index in range(12):
        angle = 2 * np.pi * index / 12
        body.add_geom(
            name=f"{prefix}_{index}", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[radius * np.cos(angle), radius * np.sin(angle), z_center],
            quat=_yaw(angle),
            size=[thickness / 2, radius * np.tan(np.pi / 12), height / 2],
            rgba=color, density=density, friction=[0.8, 0.01, 0.001], condim=4,
        )


def _add_tray(spec, xy):
    tray = spec.worldbody.add_body(
        name="tea_tray", pos=[*xy, TABLE_HEIGHT], quat=_yaw(TRAY_YAW)
    )
    hx, hy = TRAY_HALF_XY
    tray.add_geom(
        name="tea_tray_floor", type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0, 0, TRAY_FLOOR_HALF], size=[hx, hy, TRAY_FLOOR_HALF],
        rgba=[0.035, 0.040, 0.045, 1],
        friction=[1, 0.01, 0.001],
    )
    for name, pos, size in (
        ("x_neg", [-hx, 0, .016], [.004, hy, .012]),
        ("x_pos", [hx, 0, .016], [.004, hy, .012]),
        ("y_neg", [0, -hy, .016], [hx, .004, .012]),
        ("y_pos", [0, hy, .016], [hx, .004, .012]),
    ):
        tray.add_geom(
            name=f"tea_tray_rim_{name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=pos, size=size, rgba=[.025, .030, .035, 1],
            friction=[1, .01, .001],
        )


def _add_bottle_cap_and_tea(spec, xy):
    bottle_z = TRAY_TOP + .003
    bottle = spec.worldbody.add_body(name="tea_bottle", pos=[*xy, bottle_z])
    bottle.add_freejoint(name="tea_bottle_free")
    wall_inner_radius = BOTTLE_RADIUS - BOTTLE_WALL_THICKNESS
    bottle.add_geom(
        name="tea_bottle_floor", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0, 0, -.001], size=[wall_inner_radius, .002, 0],
        rgba=[.10, .15, .14, 1], density=BOTTLE_DENSITY, friction=[.8, .01, .001],
    )
    bottle_color = [.10, .22, .19, 1]
    shoulder_height = BOTTLE_HEIGHT - BOTTLE_NECK_HEIGHT
    shoulder_thickness = .003
    _ring(
        bottle, "tea_bottle_wall", (BOTTLE_RADIUS + wall_inner_radius) / 2,
        shoulder_height - shoulder_thickness, BOTTLE_WALL_THICKNESS,
        bottle_color, BOTTLE_DENSITY,
    )
    _ring(
        bottle, "tea_bottle_neck", (BOTTLE_NECK_RADIUS + BOTTLE_INNER_RADIUS) / 2,
        BOTTLE_NECK_HEIGHT, BOTTLE_NECK_RADIUS - BOTTLE_INNER_RADIUS,
        bottle_color, BOTTLE_DENSITY, z_center=shoulder_height + BOTTLE_NECK_HEIGHT / 2,
    )
    _ring(
        bottle, "tea_bottle_shoulder", (BOTTLE_RADIUS + BOTTLE_INNER_RADIUS) / 2,
        shoulder_thickness, BOTTLE_RADIUS - BOTTLE_INNER_RADIUS,
        bottle_color, BOTTLE_DENSITY, z_center=shoulder_height - shoulder_thickness / 2,
    )
    for index, height in enumerate((.024, .047, .070)):
        _ring(
            bottle, f"tea_bottle_rib_{index}", BOTTLE_RADIUS + .00035,
            .0025, .0015, bottle_color, BOTTLE_DENSITY, z_center=height,
        )

    carriage = bottle.add_body(
        name="tea_thread_carriage", pos=[0, 0, BOTTLE_HEIGHT + .010]
    )
    carriage.add_joint(
        name="tea_thread_slide", type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=[0, 0, 1], limited=True, range=[0, THREAD_PITCH * THREAD_TURNS],
        damping=.002, solref_limit=[.004, 1],
        solimp_limit=[.95, .99, .001, .5, 2],
    )
    # Dry drag holds against gravity. Added inertia reduces slip in the cap weld.
    carriage.add_joint(
        name="tea_thread_hinge", type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[0, 0, 1], limited=True, range=[0, THREAD_LIMIT],
        damping=.003, frictionloss=.00016, armature=.00001,
    )
    carriage.add_geom(
        name="tea_thread_carriage_inertial", type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[.001, 0, 0], mass=.002, rgba=[0, 0, 0, 0], contype=0, conaffinity=0,
    )

    cap_z = bottle_z + BOTTLE_HEIGHT + .010
    cap = spec.worldbody.add_body(name="tea_cap", pos=[*xy, cap_z])
    cap.add_freejoint(name="tea_cap_free")
    cap.add_geom(
        name="tea_cap_top", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0, 0, CAP_HALF_HEIGHT - .002], size=[CAP_RADIUS, .002, 0],
        rgba=[.35, .78, .20, 1], density=650, friction=[1, .01, .001],
    )
    _ring(
        cap, "tea_cap_wall", CAP_RADIUS - .002, 2 * CAP_HALF_HEIGHT, .004,
        [.35, .78, .20, 1], 650, z_center=0,
    )
    coupling = [0, 2 * np.pi / THREAD_PITCH, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    spec.add_equality(
        name="tea_thread_coupling", type=mujoco.mjtEq.mjEQ_JOINT,
        objtype=mujoco.mjtObj.mjOBJ_JOINT, name1="tea_thread_hinge",
        name2="tea_thread_slide", data=coupling, solref=[.004, 1],
        solimp=[.95, .99, .001, .5, 2],
    )
    spec.add_equality(
        name="tea_cap_threaded_weld", type=mujoco.mjtEq.mjEQ_WELD,
        objtype=mujoco.mjtObj.mjOBJ_BODY, name1="tea_thread_carriage",
        name2="tea_cap", data=[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
        solref=[.004, 1],
        solimp=[.95, .99, .001, .5, 2],
    )

    bottle_pos = np.array([*xy, bottle_z])
    for index, pose in enumerate(TEA_CHARGE_POSES):
        tea = spec.worldbody.add_body(
            name=f"tea_particle_{index}", pos=bottle_pos + pose[:3], quat=pose[3:],
        )
        tea.add_freejoint(name=f"tea_particle_free_{index}")
        tea.add_geom(
            name=f"tea_particle_geom_{index}", type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
            size=TEA_PIECE_SIZE, density=TEA_DENSITY,
            solref=[.004, 1], solimp=[.95, .99, .001, .5, 2],
            priority=1, rgba=[.19, .095, .023, 1], friction=[.7, .01, .001],
        )


def _add_teapot(spec, xy):
    pot = spec.worldbody.add_body(name="tea_teapot", pos=[*xy, TRAY_TOP + .003])
    pot.add_freejoint(name="tea_teapot_free")
    pot.add_geom(
        name="tea_teapot_floor", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0, 0, .003], size=[POT_INNER_RADIUS, .006, 0], rgba=[.20, .62, .58, 1],
        solref=[.004, 1], solimp=[.95, .99, .001, .5, 2],
        density=1100, friction=[.85, .01, .001],
    )
    _ring(
        pot, "tea_teapot_wall", POT_INNER_RADIUS + .003, POT_HEIGHT, .006,
        [.20, .62, .58, 1], 1100,
    )
    for name, fromto, radius in (
        ("handle", [-.043, 0, .018, -.070, 0, .050], .005),
        ("spout", [.040, 0, .035, .075, 0, .065], .009),
    ):
        pot.add_geom(
            name=f"tea_teapot_{name}", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=fromto, size=[radius, 0, 0], rgba=[.20, .62, .58, 1], density=800,
        )


def _add_tweezers(spec, xy):
    # Start the bridge above the cradle lip so gravity seats the free tool.
    root = spec.worldbody.add_body(
        name="tea_tweezers", pos=[*xy, TRAY_TOP + .025], quat=_yaw(TOOL_YAW)
    )
    root.add_freejoint(name="tea_tweezers_free")
    root.add_geom(
        name="tea_tweezers_bridge", type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[.009, .012, .003], rgba=[.20, .12, .07, 1], density=1500,
        friction=[.8, .01, .001],
    )
    for side, y, angle in (("upper", .009, -.075), ("lower", -.009, .075)):
        arm = root.add_body(
            name=f"tea_tweezers_{side}_arm", pos=[.008, y, 0], quat=_yaw(angle)
        )
        arm.add_joint(
            name=f"tea_tweezers_{side}_spring", type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=[0, 0, 1], limited=True, range=[-.22, .22],
            springref=0, stiffness=.08, damping=.003,
        )
        arm.add_geom(
            name=f"tea_tweezers_{side}_geom", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[.052, 0, 0], size=[.052, .002, .002],
            rgba=[.66, .38, .18, 1], density=1600,
            friction=[.9, .01, .001], condim=4,
        )


def _add_tool_cradle(spec, tray_xy):
    xy = tray_xy + TRAY_ROTATION @ [.087, -.094]
    cradle = spec.worldbody.add_body(
        name="tea_tool_cradle", pos=[*xy, TRAY_TOP + .004], quat=_yaw(TOOL_YAW),
    )
    cradle.add_freejoint(name="tea_tool_cradle_free")
    for name, pos, size in (
        ("base", [0, 0, 0], [.080, .025, .003]),
        ("side_pos", [0, .018, .008], [.080, .006, .008]),
        ("side_neg", [0, -.018, .008], [.080, .006, .008]),
        ("outboard_end", [-.076, 0, .008], [.004, .025, .008]),
    ):
        cradle.add_geom(
            name=f"tea_tool_cradle_{name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=pos, size=size, density=650, friction=[.9, .01, .001],
            rgba=[.72, .40, .16, 1],
        )


def add_tea_preparing_scene(spec: mujoco.MjSpec, object_xy=DEMO_OBJECT_XY):
    """Add fixed tray and freely manipulable Tea Preparing objects."""
    positions = np.asarray(object_xy, dtype=float)
    if positions.shape != (4, 2) or not np.isfinite(positions).all():
        raise ValueError("object_xy must contain finite tray, bottle, teapot, tweezers XY")
    spec.compiler.degree = False
    _add_tray(spec, positions[0])
    _add_bottle_cap_and_tea(spec, positions[1])
    _add_teapot(spec, positions[2])
    _add_tweezers(spec, positions[3])
    _add_tool_cradle(spec, positions[0])


class TeaPreparingMetrics:
    """Report physical task state; call physics_step after each integration step."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        for key, name in (
            ("bottle_body", "tea_bottle"), ("cap_body", "tea_cap"),
            ("pot_body", "tea_teapot"), ("tweezers_body", "tea_tweezers"),
        ):
            setattr(self, key, model.body(name).id)
        self.thread_slide = model.joint("tea_thread_slide").id
        for key, name in (
            ("bottle_joint", "tea_bottle_free"), ("cap_joint", "tea_cap_free"),
            ("tweezers_joint", "tea_tweezers_free"), ("thread_hinge", "tea_thread_hinge"),
        ):
            setattr(self, key, model.joint(name).id)
        self.thread_weld = model.equality("tea_cap_threaded_weld").id
        self.tray_body = model.body("tea_tray").id
        geoms = range(model.ngeom)
        self.tray_geoms = {g for g in geoms if model.geom(g).name.startswith("tea_tray_")}
        self.bottle_geoms = {g for g in geoms if model.geom(g).name.startswith("tea_bottle_")}
        self.cap_geoms = {g for g in geoms if model.geom(g).name.startswith("tea_cap_")}
        self.pot_geoms = {g for g in geoms if model.geom(g).name.startswith("tea_teapot_")}
        self.tweezers_geoms = {g for g in geoms if model.geom(g).name.startswith("tea_tweezers_")}
        self.left_hand_geoms = {g for g in geoms if model.geom(g).name.startswith("left_hand_")}
        self.right_hand_geoms = {g for g in geoms if model.geom(g).name.startswith("right_hand_")}
        self.hand_digits = {}
        for geom in self.left_hand_geoms | self.right_hand_geoms:
            link = model.geom(geom).name.partition("_hand_collision_")[2]
            # LEAP numbers four collision links per finger after the palm.
            self.hand_digits[geom] = (int(link) - 1) // 4 if link else -1
        self.cap_support_geoms = self.tray_geoms | {
            g for g in geoms if model.geom(g).name == "table"
        }
        self.tweezers_support_geoms = self.tray_geoms | {
            g for g in geoms if model.geom(g).name.startswith("tea_tool_cradle_")
        }
        self.support_geoms = self.cap_support_geoms | self.tweezers_support_geoms | {
            g for g in geoms if model.geom(g).name == "room_floor_surface"
        }
        self.tea_geoms = [g for g in geoms if model.geom(g).name.startswith("tea_particle_geom_")]
        self.tea_bodies = [model.geom_bodyid[geom] for geom in self.tea_geoms]
        self.tea_joints = [model.body_jntadr[body] for body in self.tea_bodies]
        pot_floor = model.geom("tea_teapot_floor")
        self.pot_floor_height = pot_floor.pos[2] + pot_floor.size[1]
        self.initial_bottle_pos = model.qpos0[model.jnt_qposadr[self.bottle_joint]:][:3].copy()
        self.initial_tweezers_pos = model.qpos0[model.jnt_qposadr[self.tweezers_joint]:][:3].copy()
        self.bottle_used = False
        self.tweezers_used = False

    def _contact(self, data, first, second):
        for index in range(data.ncon):
            pair = {data.contact[index].geom1, data.contact[index].geom2}
            if pair & first and pair & second:
                return True
        return False

    def _held(self, data, object_geoms, hand_geoms):
        contacts = []
        wrench = np.zeros(6)
        for index, contact in enumerate(data.contact[:data.ncon]):
            first, second = contact.geom
            if first in object_geoms:
                other, normal = second, contact.frame[:3]
            elif second in object_geoms:
                other, normal = first, -contact.frame[:3]
            else:
                continue
            if other in self.support_geoms:
                return False
            if other in hand_geoms:
                mujoco.mj_contactForce(self.model, data, index, wrench)
                if wrench[0] > 0:
                    contacts.append((self.hand_digits[other], normal))
        return any(
            first != second and np.dot(a, b) < -0.25
            for (first, a), (second, b) in combinations(contacts, 2)
        )

    def _settled(self, data, joint):
        dof = self.model.jnt_dofadr[joint]
        return np.linalg.norm(data.qvel[dof:dof + 6]) < .12

    def _turns(self, data):
        return max(0.0, float(data.qpos[self.model.jnt_qposadr[self.thread_hinge]] / (2 * np.pi)))

    def physics_step(self, data: mujoco.MjData):
        """Release only the weld; the cap keeps its integrated free pose and velocity."""
        slide = data.qpos[self.model.jnt_qposadr[self.thread_slide]]
        if (
            data.eq_active[self.thread_weld]
            and self._turns(data) >= THREAD_TURNS - .005
            and slide >= .95 * THREAD_PITCH * THREAD_TURNS
        ):
            data.eq_active[self.thread_weld] = 0

    def _tea_deposited_count(self, data):
        rotation = data.xmat[self.pot_body].reshape(3, 3)
        settled_inside = set()
        for geom, body, joint in zip(self.tea_geoms, self.tea_bodies, self.tea_joints):
            local = rotation.T @ (data.xpos[body] - data.xpos[self.pot_body])
            inside = (
                np.linalg.norm(local[:2]) <= POT_INNER_RADIUS
                and self.pot_floor_height <= local[2] <= POT_HEIGHT
            )
            dof = self.model.jnt_dofadr[joint]
            if inside and np.linalg.norm(data.qvel[dof:dof + 3]) < .12:
                settled_inside.add(geom)

        if not settled_inside:
            return 0

        supported = set()
        grain_contacts = []
        for contact in data.contact[:data.ncon]:
            pair = {contact.geom1, contact.geom2}
            if pair & self.pot_geoms:
                supported.update(pair & settled_inside)
            elif pair <= settled_inside:
                grain_contacts.append(pair)

        # Upper grains in a pile are supported through lower grains.
        while supported:
            reached = supported.union(*(pair for pair in grain_contacts if pair & supported))
            if reached == supported:
                break
            supported = reached
        return len(supported)

    def update(self, data: mujoco.MjData) -> dict:
        cap_turns = self._turns(data)
        cap_unthreaded = not bool(data.eq_active[self.thread_weld])
        bottle_tilted = data.xmat[self.bottle_body].reshape(3, 3)[2, 2] < np.cos(np.deg2rad(35))
        bottle_moved = np.linalg.norm(data.xpos[self.bottle_body] - self.initial_bottle_pos) > .035
        if not self.bottle_used and (bottle_tilted or bottle_moved):
            self.bottle_used = self._held(data, self.bottle_geoms, self.left_hand_geoms)
        tweezers_moved = np.linalg.norm(data.xpos[self.tweezers_body] - self.initial_tweezers_pos) > .035
        if cap_unthreaded and not self.tweezers_used and tweezers_moved:
            self.tweezers_used = self._held(data, self.tweezers_geoms, self.right_hand_geoms)
        bottle_returned = bool(
            self.bottle_used
            and np.linalg.norm(data.xpos[self.bottle_body, :2] - self.initial_bottle_pos[:2]) < .035
            and self._contact(data, self.bottle_geoms, self.tray_geoms)
            and self._settled(data, self.bottle_joint)
        )
        tweezers_returned = bool(
            self.tweezers_used
            and np.linalg.norm(data.xpos[self.tweezers_body, :2] - self.initial_tweezers_pos[:2]) < .035
            and self._contact(data, self.tweezers_geoms, self.tweezers_support_geoms)
            and not self._contact(data, self.tweezers_geoms, self.right_hand_geoms)
            and self._settled(data, self.tweezers_joint)
        )
        left_released = bool(
            self.bottle_used and not self._contact(data, self.left_hand_geoms, self.bottle_geoms)
        )
        tray_rotation = data.xmat[self.tray_body].reshape(3, 3)
        cap_from_tray = tray_rotation.T @ (
            data.xpos[self.cap_body] - data.xpos[self.tray_body]
        )
        # The official rollout places the free cap on the table beside the right rim.
        cap_on_right = bool(
            cap_unthreaded and self._contact(data, self.cap_geoms, self.cap_support_geoms)
            and not self._contact(
                data, self.cap_geoms, self.left_hand_geoms | self.right_hand_geoms,
            )
            and self._settled(data, self.cap_joint)
            and abs(cap_from_tray[0]) < TRAY_HALF_XY[0]
            and -TRAY_HALF_XY[1] - .12 < cap_from_tray[1] < -.02
        )
        deposited = self._tea_deposited_count(data)
        tea_deposited = deposited >= 1
        return {
            "cap_turns": cap_turns,
            "cap_unthreaded": cap_unthreaded,
            "cap_on_right": cap_on_right,
            "tea_deposited_count": deposited,
            "tea_deposited": bool(tea_deposited),
            "bottle_used": bool(self.bottle_used),
            "bottle_returned": bottle_returned,
            "tweezers_used": bool(self.tweezers_used),
            "tweezers_returned": tweezers_returned,
            "left_hand_released": left_released,
            "sim_success": bool(
                cap_unthreaded and cap_on_right and tea_deposited
                and bottle_returned and tweezers_returned and left_released
            ),
        }
