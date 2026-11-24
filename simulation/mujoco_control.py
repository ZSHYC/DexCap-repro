"""Force-limited Cartesian control of the articulated Franka–LEAP pairs."""

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from simulation.frames import CALIBRATION
from simulation.hand_joints import decode_leap_joints, encode_leap_joints


ARM_HOME = np.array([0, -0.55, 0, -2.35, 0, 2.0, 0.785])
CONTROL_INTERVAL = 0.05
WRIST_RAMP_FRACTION = 0.3


def wrist_error(position, rotation, target_position, target_rotation):
    angular = Rotation.from_matrix(target_rotation @ rotation.T).as_rotvec()
    return np.concatenate((target_position - position, angular))


def wrist_ramp(position, rotation, target_position, target_rotation, fractions):
    positions = position + fractions[:, None] * (target_position - position)
    delta = Rotation.from_matrix(target_rotation @ rotation.T).as_rotvec()
    rotations = Rotation.from_rotvec(fractions[:, None] * delta) * Rotation.from_matrix(rotation)
    return list(zip(positions, rotations.as_matrix()))


class Arm:
    def __init__(self, model, side):
        self.site = model.site(f"{side}_wrist").id
        self.arm_actuators = np.array([model.actuator(f"{side}_arm_{i}").id for i in range(1, 8)])
        self.hand_actuators = np.array([model.actuator(f"{side}_finger_{i}").id for i in range(16)])
        joint = model.actuator_trnid[self.arm_actuators, 0]
        self.pos = model.jnt_qposadr[joint]
        self.dofs = model.jnt_dofadr[joint]
        self.lower, self.upper = model.jnt_range[joint].T
        self.hand_pos = model.jnt_qposadr[model.actuator_trnid[self.hand_actuators, 0]]
        self.hand_limits = model.actuator_ctrlrange[self.hand_actuators].T
        self.torque_limits = model.actuator_ctrlrange[self.arm_actuators].T

    def error(self, data, position, rotation):
        return wrist_error(data.site_xpos[self.site], data.site_xmat[self.site].reshape(3, 3),
                           position, rotation)


class DualArmController:
    def __init__(self, model, data):
        self.model, self.data = model, data
        self.arms = [Arm(model, side) for side in ("right", "left")]
        self.mass = np.empty((model.nv, model.nv))
        self.jacobian = np.empty((6, model.nv))
        gains = yaml.safe_load((CALIBRATION / "osc-controller.yml").read_text())["Kp"]
        self.kp = np.repeat([gains["translation"], gains["rotation"]], 3)
        self.kd = 2 * np.sqrt(self.kp)
        self.damping = np.eye(6) * 1e-3
        self.substeps = round(CONTROL_INTERVAL / model.opt.timestep)

    def _targets(self, action):
        action = np.asarray(action, dtype=float)
        if action.shape != (46,) or not np.isfinite(action).all():
            raise ValueError("action must contain 46 finite values")
        rotations = Rotation.from_quat(action[6:14].reshape(2, 4), scalar_first=True).as_matrix()
        for i, arm in enumerate(self.arms):
            fingers = decode_leap_joints(action[14 + i * 16:30 + i * 16], left=bool(i))
            yield action[3 * i:3 * i + 3], rotations[i], np.clip(fingers, *arm.hand_limits)

    def initialize(self, action):
        """Find the rest configuration at reset, before advancing simulated time."""
        for arm, (position, rotation, fingers) in zip(self.arms, self._targets(action)):
            self.data.qpos[arm.hand_pos] = fingers
            self.data.ctrl[arm.hand_actuators] = fingers

            def residual(q):
                self.data.qpos[arm.pos] = q
                mujoco.mj_kinematics(self.model, self.data)
                pose = arm.error(self.data, position, rotation)
                return np.concatenate((pose[:3], 0.25 * pose[3:], 0.002 * (q - ARM_HOME)))

            result = least_squares(residual, ARM_HOME, bounds=(arm.lower, arm.upper), max_nfev=100)
            self.data.qpos[arm.pos] = result.x
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def track(self, action, cycles, *, physics_step=None):
        """Interpolate wrists during the first cycle, then hold the absolute targets."""
        targets = tuple(self._targets(action))
        fractions = np.minimum(1, np.arange(1, self.substeps + 1) / (WRIST_RAMP_FRACTION * self.substeps))
        ramps = []
        for arm, (position, rotation, fingers) in zip(self.arms, targets):
            self.data.ctrl[arm.hand_actuators] = fingers
            ramps.append(wrist_ramp(
                self.data.site_xpos[arm.site], self.data.site_xmat[arm.site].reshape(3, 3),
                position, rotation, fractions,
            ))
        commands = list(zip(*ramps))
        hold = [(position, rotation) for position, rotation, _ in targets]

        for substep in range(cycles * self.substeps):
            mujoco.mj_step1(self.model, self.data)
            mujoco.mj_fullM(self.model, self.mass, self.data.qM)
            poses = commands[substep] if substep < self.substeps else hold
            for arm, (position, rotation) in zip(self.arms, poses):
                mujoco.mj_jacSite(self.model, self.data, self.jacobian[:3], self.jacobian[3:], arm.site)
                jacobian = self.jacobian[:, arm.dofs]
                mass = self.mass[np.ix_(arm.dofs, arm.dofs)]
                inverse_mass_jt = np.linalg.solve(mass, jacobian.T)
                task_inertia = jacobian @ inverse_mass_jt + self.damping
                velocity = self.data.qvel[arm.dofs]
                acceleration = self.kp * arm.error(self.data, position, rotation) - self.kd * (jacobian @ velocity)
                task_torque = jacobian.T @ np.linalg.solve(task_inertia, acceleration)
                posture = mass @ (10 * (ARM_HOME - self.data.qpos[arm.pos]) - 6.3 * velocity)
                null_torque = posture - jacobian.T @ np.linalg.solve(task_inertia, inverse_mass_jt.T @ posture)
                torque = task_torque + null_torque + self.data.qfrc_bias[arm.dofs]
                self.data.ctrl[arm.arm_actuators] = np.clip(torque, *arm.torque_limits)
            mujoco.mj_step2(self.model, self.data)
            if physics_step is not None:
                physics_step(self.data)
            if (substep + 1) % self.substeps == 0:
                mujoco.mj_forward(self.model, self.data)
                yield self.tracking_error(action)

    def tracking_error(self, action):
        errors = np.array([arm.error(self.data, position, rotation)
                           for arm, (position, rotation, _) in zip(self.arms, self._targets(action))])
        return np.linalg.norm(errors[:, :3], axis=1), np.linalg.norm(errors[:, 3:], axis=1)

    def observed_action(self):
        site_ids = [arm.site for arm in self.arms]
        wrist = self.data.site_xpos[site_ids].ravel()
        palm_matrix = self.data.site_xmat[site_ids].reshape(2, 3, 3)
        orientations = Rotation.from_matrix(palm_matrix).as_quat(canonical=True, scalar_first=True)
        fingers = [encode_leap_joints(self.data.qpos[arm.hand_pos], left=bool(i))
                   for i, arm in enumerate(self.arms)]
        return np.concatenate((wrist, orientations.ravel(), *fingers))
