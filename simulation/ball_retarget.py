"""Evaluate camera-guided Ball Collecting from the measured training references."""

import argparse
import json
from pathlib import Path

import h5py
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from simulation import mujoco_ball_collecting as task
from simulation.frames import AXES, TABLE_SHIFT
from simulation.mujoco_scene import MujocoScene
from simulation.rollout import ASSET_ROOT, load_demo


LAYOUT_SCALE = np.array([0.20, 0.15, 0.20, 0.15])


def camera_layout(pointcloud):
    world = pointcloud[:, :3] @ AXES.T + TABLE_SHIFT
    red, green, blue = pointcloud[:, 3:].T
    sphere = world[(green > 0.20) & (green > 2.2 * blue) & (red > 0.15)]
    rim_z = task.TABLE_HEIGHT + task.BASKET_WIRE_RADIUS + task.BASKET_HEIGHT
    brightness = pointcloud[:, 3:].max(axis=1)
    rim = world[(brightness > 0) & (brightness < 0.12) & (abs(world[:, 2] - rim_z) < 0.004)]
    if len(sphere) < 20 or len(rim) < 20:
        raise ValueError("Initial ball or basket rim is outside the calibrated RGB-D view")

    origin = sphere.mean(axis=0)
    centered = sphere - origin
    squared = np.einsum("ij,ij->i", centered, centered)
    offset = np.linalg.lstsq(2 * centered, squared - squared.mean(), rcond=None)[0]
    radii = np.linalg.norm(centered - offset, axis=1)
    if np.median(abs(radii - task.BALL_RADIUS)) > 0.004:
        raise ValueError("Visible ball points do not fit the measured sphere")
    ball = origin + offset
    lower, upper = np.quantile(rim[:, :2], [0.008, 0.992], axis=0)
    # The right rim remains visible when the camera crops the opposite edge.
    basket_x = upper[0] - task.BASKET_HALF_X - task.BASKET_WIRE_RADIUS
    return np.array([basket_x, 0.5 * (lower[1] + upper[1]), *ball[:2]], dtype=np.float32)


def training_references(directory):
    exports = directory / "at-20hz"
    dataset = exports / "ball-20hz-seeds-01-16.hdf5"
    rows = json.loads((exports / "ball-layout-primitive-training-phases.json").read_text())
    phases = {row["seed"]: row for row in rows}
    references = {}
    with h5py.File(dataset, "r") as source:
        for name in source["mask/train"][:]:
            demo = source["data"][name.decode()]
            seed = int(demo.attrs["source_seed"])
            layout = camera_layout(demo["obs/pointcloud"][0])
            native = directory / f"neutral-ball-seed-{seed:02d}.hdf5"
            _, _, _, actions = load_demo(native)
            with h5py.File(native, "r") as recording:
                cycles = np.asarray(recording["data/demo_0"].attrs["control_cycles_per_action"], dtype=int)
            phase_cycles = [int(cycles[:phases[seed][name]].sum())
                            for name in ("first_contact", "first_lift", "first_release")]
            references[seed] = (layout, np.repeat(actions, cycles, axis=0), phase_cycles)
    return references


def retarget(references, layout):
    seed = min(sorted(references), key=lambda k: np.linalg.norm((layout - references[k][0]) / LAYOUT_SCALE))
    reference_layout, actions, (contact, lift, release) = references[seed]
    ball_delta = layout[2:] - reference_layout[2:]
    basket_delta = layout[:2] - reference_layout[:2]
    approach_end = max(20, contact - 18)
    transfer_start = lift + 12
    transfer_end = max(transfer_start + 20, release - 65)

    targets = actions.copy()
    for step, action in enumerate(targets):
        if step < approach_end:
            shift = (step / approach_end) * ball_delta
        elif step < transfer_start:
            shift = ball_delta
        elif step < transfer_end:
            time = (step - transfer_start) / (transfer_end - transfer_start)
            time = time * time * (3 - 2 * time)
            shift = (1 - time) * ball_delta + time * basket_delta
        else:
            shift = basket_delta
        action[:2] += shift
    return seed, targets, (transfer_start, transfer_end, release)


def wrist_ik(model, arm, work, position, rotation, previous, weight=0.001, max_evaluations=240):
    def residual(q):
        work.qpos[arm.pos] = q
        mujoco.mj_kinematics(model, work)
        error = arm.error(work, position, rotation)
        return np.r_[error[:3], 0.22 * error[3:], weight * (q - previous)]

    fit = least_squares(residual, np.clip(previous, arm.lower, arm.upper),
                        bounds=(arm.lower, arm.upper), max_nfev=max_evaluations)
    return fit.x, float(np.linalg.norm(fit.fun[:3]))


def transport_risk(scene, targets, start, release):
    # Only joint proprioception and the camera-retargeted targets enter the preview.
    arm = scene.control.arms[0]
    work = mujoco.MjData(scene.model)
    current = scene.data.qpos[arm.pos].copy()
    rotations = Rotation.from_quat(targets[start:release + 1, 6:10], scalar_first=True).as_matrix()
    direct_q, direct_error = wrist_ik(scene.model, arm, work, targets[release, :3], rotations[-1], current)
    direct_margin = float(arm.upper[5] - direct_q[5])
    timed_margin = float("inf")
    for target, rotation in zip(targets[start:release + 1], rotations):
        current, timed_error = wrist_ik(scene.model, arm, work, target[:3], rotation, current, 0.010, 60)
        timed_margin = min(timed_margin, float(arm.upper[5] - current[5]))
    reroute = (direct_margin < 0.050 and direct_error > 0.020) or (timed_margin < 0.050 and timed_error > 0.050)
    return dict(rerouted=reroute, direct_q6_margin_rad=direct_margin, direct_wrist_error_m=direct_error,
                timed_q6_margin_rad=timed_margin, timed_wrist_error_m=timed_error)


def interpolate_wrist_targets(start, goal, steps):
    rotations = (slice(6, 10), slice(10, 14))
    paths = [Slerp([0.0, 1.0], Rotation.from_quat(np.stack((start[part], goal[part])), scalar_first=True))
             for part in rotations]
    for u in np.linspace(1.0 / steps, 1.0, steps):
        t = u * u * (3.0 - 2.0 * u)
        target = (1.0 - t) * start + t * goal
        for part, path in zip(rotations, paths):
            target[part] = path([t]).as_quat(scalar_first=True)[0]
        yield target


def elbow_route(scene, targets, branch_target, release):
    right, left = scene.control.arms
    pose = branch_target.copy()
    for arm, position, quaternion in ((right, slice(0, 3), slice(6, 10)), (left, slice(3, 6), slice(10, 14))):
        pose[position] = scene.data.site_xpos[arm.site]
        pose[quaternion] = Rotation.from_matrix(
            scene.data.site_xmat[arm.site].reshape(3, 3),
        ).as_quat(scalar_first=True)
    for axis, distance, steps in ((5, 0.060, 50), (4, 0.120, 70)):
        goal = pose.copy()
        goal[axis] += distance
        for pose in interpolate_wrist_targets(pose, goal, steps):
            yield pose

    work = mujoco.MjData(scene.model)
    initial = scene.data.qpos[right.pos].copy()
    rotation = Rotation.from_rotvec([np.deg2rad(40.0), 0.0, 0.0]) * Rotation.from_quat(
        targets[release, 6:10], scalar_first=True,
    )
    final, _ = wrist_ik(scene.model, right, work, targets[release, :3], rotation.as_matrix(), initial)
    for u in np.linspace(1.0 / 180.0, 1.0, 180):
        t = u * u * (3.0 - 2.0 * u)
        work.qpos[right.pos] = (1.0 - t) * initial + t * final
        mujoco.mj_kinematics(scene.model, work)
        pose[:3] = work.site_xpos[right.site]
        pose[6:10] = Rotation.from_matrix(work.site_xmat[right.site].reshape(3, 3)).as_quat(scalar_first=True)
        yield pose
    for _ in range(25):
        yield pose
    for target in targets[max(0, release - 70):]:
        pose[14:30] = target[14:30]
        yield pose
    for _ in range(100):
        yield pose


def transport_commands(scene, targets, phase, preview):
    start, transfer_end, release = phase
    yield from targets[:start]
    stop = len(targets)
    if preview is not None:
        preview.update(transport_risk(scene, targets, start, release))
        if preview["rerouted"]:
            stop = start + round(0.65 * (transfer_end - start)) + 1
    arm = scene.control.arms[0]
    for target in targets[start:stop]:
        command = target.copy()
        command[:3] += np.clip(0.7 * (target[:3] - scene.data.site_xpos[arm.site]), -0.060, 0.060)
        yield command
    if preview is not None and preview["rerouted"]:
        yield from elbow_route(scene, targets, command, release)


def evaluate(directory, first_seed, trials, *, guarded_transport=False):
    references = training_references(directory)
    frames, start, camera, _ = load_demo(directory / "neutral-ball-seed-01.hdf5")
    results = []
    for seed in range(first_seed, first_seed + trials):
        rng = np.random.default_rng(seed)
        placement = task.sample_object_xy(rng)
        scene = MujocoScene(ASSET_ROOT, frames, camera, "ball-collecting", placement, None)
        try:
            scene.reset(start)
            try:
                layout = camera_layout(scene.camera.capture(scene.data, rng))
            except ValueError as error:
                results.append(dict(seed=seed, sim_success=False, executed_steps=0, localization_error=str(error)))
                continue
            # The frozen preview uses the camera coordinates in double precision.
            donor, targets, phase = retarget(references, layout.astype(float) if guarded_transport else layout)
            preview = {} if guarded_transport else None
            arm = scene.control.arms[0]
            peak_load = 0.0
            first_release_error = None
            force = np.zeros(6)
            min_q6_margin = float("inf")
            for step, command in enumerate(transport_commands(scene, targets, phase, preview)):
                score, wrist_error, _ = scene.step(command, control_cycles=1)
                min_q6_margin = min(min_q6_margin, float(arm.upper[5] - scene.data.qpos[arm.pos][5]))
                load = 0.0
                for contact_id, contact in enumerate(scene.data.contact[:scene.data.ncon]):
                    pair = set(map(int, contact.geom))
                    if scene.metrics.ball_geom in pair and pair & scene.metrics.right_hand_geoms:
                        mujoco.mj_contactForce(scene.model, scene.data, contact_id, force)
                        load += float(force[0])
                peak_load = max(peak_load, load)
                if first_release_error is None and score["ball_lifted"] and score["ball_released"]:
                    first_release_error = float(wrist_error[0])
                if scene.invalid_hand_motion or scene.data.warning.number.any():
                    score["sim_success"] = False
                    break
            result = dict(
                score, seed=seed, source_seed=donor, camera_layout_m=layout.tolist(),
                executed_steps=step + 1, peak_ball_load_N=peak_load,
                first_release_wrist_error_m=first_release_error, arm_contact_steps=scene.arm_contact_steps,
                hand_collision_steps=scene.hand_collision_steps,
                same_hand_max_penetration_m=scene.same_hand_max_penetration_m,
                invalid_hand_motion=scene.invalid_hand_motion, physics_warnings=scene.data.warning.number.tolist(),
            )
            if preview is not None:
                result["transport_preview"] = preview
                result["min_q6_upper_margin_rad"] = min_q6_margin
            results.append(result)
            print(json.dumps(result), flush=True)
        finally:
            scene.close()
    return {"task": "ball-collecting", "trial_count": len(results),
            "success_count": sum(result["sim_success"] for result in results), "trials": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_directory", type=Path, help="Directory with neutral-ball HDF5 and at-20hz exports")
    parser.add_argument("--first-seed", type=int, required=True)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--guarded-transport", action="store_true",
                        help="Use the joint-limit preview and continuous elbow route")
    args = parser.parse_args()
    if args.first_seed < 1 or args.trials < 1:
        parser.error("First seed and trial count must be positive")
    result = evaluate(args.reference_directory, args.first_seed, args.trials,
                      guarded_transport=args.guarded_transport)
    args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
