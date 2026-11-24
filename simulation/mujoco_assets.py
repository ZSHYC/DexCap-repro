"""Articulated Panda + DexCap LEAP assets for MuJoCo task scenes."""

import json
from pathlib import Path
from xml.etree import ElementTree

import mujoco
import numpy as np


SIDES = ("right", "left")
ARM_JOINTS = {side: tuple(f"{side}_arm_joint{i}" for i in range(1, 8)) for side in SIDES}
ARM_ACTUATORS = {side: tuple(f"{side}_arm_{i}" for i in range(1, 8)) for side in SIDES}
HAND_JOINTS = {side: tuple(f"{side}_hand_{i}" for i in range(16)) for side in SIDES}
HAND_ACTUATORS = {side: tuple(f"{side}_finger_{i}" for i in range(16)) for side in SIDES}
WRIST_SITES = {side: f"{side}_wrist" for side in SIDES}

_ARM_FORCE_LIMITS = (87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)
_LEAP_FORCE_LIMIT = 0.95
_LEAP_REFLECTED_INERTIA = 0.0005
_HAND_MESHES = Path(__file__).resolve().parents[1] / "DexCap/STEP2_build_dataset/leap_hand_mesh"
_PALM_PARTS = Path(__file__).with_name("assets") / "leap_v1_palm_parts.json"
# Provisional geometry registration: the released LEAP v1 Franka palm-upper
# mates opposite flange normals; these palm mesh centers retain a 6 mm standoff.
_PALM_MOUNT_QUATERNION = (0.0, 2**-0.5, 2**-0.5, 0.0)
_PALM_MOUNT_POSITIONS = {
    "right": (0.037, 0.05, 0.0173),
    "left": (-0.037, 0.05, 0.0173),
}


def _name_arm_geometry(panda):
    visual = collision = 0
    for geom in panda.geoms:
        if geom.contype == 0 and geom.conaffinity == 0:
            geom.name = f"arm_visual_{visual}"
            geom.group = 2
            visual += 1
        else:
            geom.name = f"arm_collision_{collision}"
            geom.group = 3
            geom.rgba = (0.0, 0.0, 0.0, 0.0)
            collision += 1


def _split_hand_geometry(hand):
    # MuJoCo's URDF importer merges identical visual/collision meshes, so restore
    # a non-colliding rendered copy before hiding each collision geom.
    for index, collision in enumerate(list(hand.geoms)):
        rgba = collision.rgba.copy()
        collision.name = f"hand_collision_{index}"
        collision.group = 3
        collision.rgba = (0.0, 0.0, 0.0, 0.0)
        if collision.type != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        collision.parent.add_geom(
            name=f"hand_visual_{index}",
            type=collision.type,
            meshname=collision.meshname,
            size=collision.size.copy(),
            pos=collision.pos.copy(),
            quat=collision.quat.copy(),
            rgba=rgba,
            group=2,
            contype=0,
            conaffinity=0,
        )


def _add_palm_collision_parts(hand):
    palm = hand.geom("hand_collision_0")
    scale = hand.mesh(palm.meshname).scale.copy()
    # Reuse the URDF pose and contact parameters, including the left hand's mirror.
    contact_fields = (
        "type", "pos", "quat", "group", "rgba", "contype", "conaffinity",
        "condim", "friction", "solref", "solimp", "solmix", "margin", "gap", "priority",
    )
    geometry = {name: getattr(palm, name) for name in contact_fields}
    for index, part in enumerate(json.loads(_PALM_PARTS.read_text())):
        mesh = hand.add_mesh(
            name=f"palm_part_{index}", scale=scale,
            uservert=part["vertices"], userface=part["faces"],
        )
        if index == 0:
            palm.meshname = mesh.name
        else:
            palm.parent.add_geom(
                name=f"hand_palm_collision_{index}", meshname=mesh.name, **geometry,
            )


def _physical_hand_spec(source, side):
    robot = ElementTree.parse(source).getroot()
    # DexCap adds fixed fingertip targets for IK. LEAP V1 has only the physical mesh links.
    markers = [link for link in robot.findall("link") if link.get("name").startswith("realtip")]
    marker_names = {link.get("name") for link in markers}
    for link in markers:
        robot.remove(link)
    for joint in robot.findall("joint"):
        if joint.find("child").get("link") in marker_names:
            robot.remove(joint)

    if side == "left":
        for origin in robot.iter("origin"):
            if "xyz" in origin.attrib:
                x, y, z = map(float, origin.attrib["xyz"].split())
                origin.set("xyz", f"{x} {-y} {z}")
            if "rpy" in origin.attrib:
                roll, pitch, yaw = map(float, origin.attrib["rpy"].split())
                origin.set("rpy", f"{-roll} {pitch} {-yaw}")
        for axis in robot.iter("axis"):
            x, y, z = map(float, axis.attrib["xyz"].split())
            axis.set("xyz", f"{-x} {y} {-z}")
        for mesh in robot.iter("mesh"):
            x, y, z = map(float, mesh.attrib.get("scale", "1 1 1").split())
            mesh.set("scale", f"{x} {-y} {z}")
        for inertia in robot.iter("inertia"):
            for term in ("ixy", "iyz"):
                inertia.set(term, str(-float(inertia.attrib[term])))

    hand = mujoco.MjSpec.from_string(ElementTree.tostring(robot, encoding="unicode"))
    hand.modelfiledir = f"{source.parent}/"
    return hand


def _load_hand(side):
    hand = _physical_hand_spec(_HAND_MESHES / "robot_pybullet.urdf", side)
    physical_joints = list(hand.joints)
    if len(physical_joints) != 16:
        raise ValueError(f"DexCap {side} LEAP URDF has {len(physical_joints)} actuated joints")

    for index, joint in enumerate(physical_joints):
        joint.name = f"hand_{index}"
        # The URDF gives link inertia and angle ranges but omits the geared rotor inertia.
        joint.armature = _LEAP_REFLECTED_INERTIA
        joint.solref_limit = (0.004, 1.0)
        joint.solimp_limit = (0.95, 0.995, 0.001, 0.5, 2.0)
        actuator = hand.add_actuator(
            name=f"finger_{index}",
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            target=joint.name,
            ctrllimited=True,
            ctrlrange=joint.range.copy(),
            forcelimited=True,
            forcerange=(-_LEAP_FORCE_LIMIT, _LEAP_FORCE_LIMIT),
        )
        actuator.set_to_position(kp=20.0, kv=1.0)

    _split_hand_geometry(hand)
    _add_palm_collision_parts(hand)
    hand.body("palm_lower").add_site(name="wrist", size=(0.006,))
    return hand


def _load_robot(asset_root, side):
    panda_path = Path(asset_root) / "franka_emika_panda/panda_nohand.xml"
    if not panda_path.exists():
        raise FileNotFoundError(
            f"missing {panda_path}; run scripts/fetch_mujoco_assets.sh"
        )
    panda = mujoco.MjSpec.from_file(str(panda_path))
    for key in list(panda.keys):
        panda.delete(key)

    for index, (joint, actuator, force) in enumerate(
        zip(panda.joints, panda.actuators, _ARM_FORCE_LIMITS), start=1
    ):
        joint.name = f"arm_joint{index}"
        actuator.name = f"arm_{index}"
        actuator.target = joint.name
        actuator.set_to_motor()
        actuator.ctrllimited = True
        actuator.ctrlrange = (-force, force)
        actuator.forcelimited = True
        actuator.forcerange = (-force, force)

    _name_arm_geometry(panda)
    hand = _load_hand(side)
    hand_frame = hand.worldbody.add_frame(
        name="leap_mount",
        pos=_PALM_MOUNT_POSITIONS[side],
        quat=_PALM_MOUNT_QUATERNION,
    )
    hand.body("palm_lower").set_frame(hand_frame)
    panda.body("attachment").attach_frame(hand_frame)
    return panda


def add_robots(spec, asset_root, base_positions, base_quaternions):
    """Attach right and left fixed-base Panda/LEAP robots to ``spec``.

    Base quaternions use MuJoCo and DexCap's ``wxyz`` ordering. The returned
    wrist sites are at each LEAP palm root, matching the old action target.
    """
    positions = np.asarray(base_positions, dtype=float)
    quaternions = np.asarray(base_quaternions, dtype=float)
    if positions.shape != (2, 3) or quaternions.shape != (2, 4):
        raise ValueError("robot bases require two xyz positions and two wxyz quaternions")
    if not np.isfinite(positions).all() or not np.isfinite(quaternions).all():
        raise ValueError("robot base poses must be finite")
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms < 1e-8):
        raise ValueError("robot base quaternions must be nonzero")

    for side, position, quaternion, norm in zip(SIDES, positions, quaternions, norms):
        base = spec.worldbody.add_frame(
            name=f"{side}_base",
            pos=position,
            quat=quaternion / norm,
        )
        spec.attach(_load_robot(asset_root, side), frame=base, prefix=f"{side}_")
    return spec
