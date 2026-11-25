"""Assemble the DexCap robot, calibrated camera, and task in one MuJoCo model."""

import mujoco
import numpy as np

from simulation.frames import TABLE_SHIFT
from simulation.mujoco_control import DualArmController
from simulation.mujoco_sponge import SPONGE_FRICTION_IMPEDANCE


TABLE_HEIGHT = -0.030
PHYSICS_STEP = 0.002
MAX_CONTROL_CYCLES = 10
HAND_STOP_TOLERANCE_RAD = 0.05
WRIST_POSITION_TOLERANCE_M = 0.005
WRIST_ANGLE_TOLERANCE_RAD = np.deg2rad(5)


def rig_base_positions(frames):
    # The released retargeter expresses each wrist in parallel axes with a different origin.
    return np.array([TABLE_SHIFT - absolute for absolute, _, _ in frames.offsets])


class MujocoScene:
    def __init__(self, asset_root, frames, camera, task, object_xy, object_name, *, object_yaw=None):
        from simulation.mujoco_assets import add_robots
        from simulation.mujoco_camera import DexCapCamera, add_camera

        spec = mujoco.MjSpec()
        spec.modelname = f"dexcap_{task}"
        spec.option.timestep = PHYSICS_STEP
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        spec.option.iterations = 80
        if task in ("wiping", "sponge-picking"):
            # Harder friction constraints keep soft sponge contacts from drifting under load.
            spec.option.impratio = SPONGE_FRICTION_IMPEDANCE
        elif task == "packaging":
            # Keep the measured cardboard geometry and suppress solver drift during carry.
            spec.option.impratio = 12
            spec.option.tolerance = 1e-10
        color = [0.12, 0.12, 0.12, 1] if task != "packaging" else [0.64, 0.59, 0.49, 1]
        spec.worldbody.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX,
                                pos=[0.40, 0, TABLE_HEIGHT - 0.025], size=[0.75, 0.60, 0.025],
                                rgba=color, friction=[0.8, 0.008, 0.0005])
        spec.worldbody.add_light(name="overhead", pos=[0.5, -0.1, 1.25], dir=[0, 0, -1],
                                 diffuse=[0.65, 0.65, 0.65])
        self.base_positions = rig_base_positions(frames)
        add_robots(spec, asset_root, self.base_positions, [[1, 0, 0, 0]] * 2)
        add_camera(spec, camera)

        metrics_args = ()
        if task == "wiping":
            from simulation.mujoco_wiping import WipingMetrics, add_wiping_scene

            add_wiping_scene(spec, object_xy)
            metrics_type = WipingMetrics
        elif task == "sponge-picking":
            from simulation.mujoco_sponge_picking import SpongePickingMetrics, add_sponge_picking_scene

            add_sponge_picking_scene(spec, object_xy)
            metrics_type = SpongePickingMetrics
        elif task == "ball-collecting":
            from simulation.mujoco_ball_collecting import BallCollectingMetrics, add_ball_collecting_scene

            add_ball_collecting_scene(spec, object_xy)
            metrics_type = BallCollectingMetrics
        elif task == "scissor-cutting":
            from simulation.mujoco_scissor_cutting import ScissorCuttingMetrics, add_scissor_cutting_scene

            add_scissor_cutting_scene(spec, object_xy)
            metrics_type = ScissorCuttingMetrics
        elif task == "tea-preparing":
            from simulation.mujoco_tea_preparing import TeaPreparingMetrics, add_tea_preparing_scene

            add_tea_preparing_scene(spec, object_xy)
            metrics_type = TeaPreparingMetrics
        else:
            from simulation.mujoco_packaging import PackagingMetrics, add_packaging_scene

            add_packaging_scene(spec, object_xy, object_name, object_yaw=object_yaw)
            metrics_type = PackagingMetrics
            metrics_args = (object_name,)

        # The table is finite, so a dropped object also needs a physical room floor.
        floor = spec.worldbody.add_body(name="room_floor", pos=[0, 0, TABLE_HEIGHT - 0.75])
        floor.add_geom(name="room_floor_surface", type=mujoco.mjtGeom.mjGEOM_PLANE,
                       size=[0, 0, 0.025], group=3, friction=[0.8, 0.008, 0.0005])
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.control = DualArmController(self.model, self.data)
        self.metrics = metrics_type(self.model, *metrics_args)
        self.task_physics_step = getattr(self.metrics, "physics_step", None)
        hand_actuators = np.concatenate([arm.hand_actuators for arm in self.control.arms])
        hand_joints = self.model.actuator_trnid[hand_actuators, 0]
        self.hand_qpos = self.model.jnt_qposadr[hand_joints]
        self.hand_lower, self.hand_upper = self.model.jnt_range[hand_joints].T
        self.max_hand_limit_excess_rad = 0.0
        self.camera = DexCapCamera(self.model, camera)
        names = [self.model.geom(index).name or "" for index in range(self.model.ngeom)]
        self.arm_geoms = np.array([name.startswith(("right_arm_", "left_arm_")) for name in names])
        self.hand_sides = np.array([
            int(name.startswith("right_hand_")) - int(name.startswith("left_hand_"))
            for name in names
        ])
        self.arm_contact_steps = 0
        self.hand_collision_steps = 0
        self.hand_max_penetration_m = 0.0
        self.same_hand_max_penetration_m = 0.0
        self.same_hand_max_penetration_pair = None

    def reset(self, initial):
        self.control.initialize(initial)
        self.max_hand_limit_excess_rad = 0.0

    @property
    def invalid_hand_motion(self):
        return self.max_hand_limit_excess_rad > HAND_STOP_TOLERANCE_RAD

    def physics_step(self, data):
        angles = data.qpos[self.hand_qpos]
        excess = np.maximum(self.hand_lower - angles, angles - self.hand_upper)
        self.max_hand_limit_excess_rad = max(self.max_hand_limit_excess_rad, float(excess.max()))
        if self.task_physics_step is not None and not self.invalid_hand_motion:
            self.task_physics_step(data)

    def _record_robot_contacts(self, data):
        if not data.ncon:
            return False, False
        geometries = data.contact.geom[:data.ncon]
        valid = geometries >= 0
        arm_contact = bool(self.arm_geoms[geometries[valid]].any())
        valid_pairs = np.all(valid, axis=1)
        rigid_pairs = geometries[valid_pairs]
        sides = self.hand_sides[rigid_pairs]
        side_pairs = sides[:, 0] * sides[:, 1]
        penetration = -data.contact.dist[:data.ncon][valid_pairs]
        interhand_peak = float(np.max(penetration[side_pairs < 0], initial=0))
        self.hand_max_penetration_m = max(self.hand_max_penetration_m, interhand_peak)
        same_hand = np.where(side_pairs > 0, penetration, 0.0)
        peak = float(np.max(same_hand, initial=0))
        if peak > self.same_hand_max_penetration_m:
            self.same_hand_max_penetration_m = peak
            pair = rigid_pairs[same_hand.argmax()]
            self.same_hand_max_penetration_pair = tuple(self.model.geom(g).name for g in pair)
        return arm_contact, interhand_peak > 0

    def step(self, targets, *, control_cycles=None, on_cycle=None):
        count = MAX_CONTROL_CYCLES if control_cycles is None else control_cycles
        arm_contact = hand_collision = False

        def sample_physics(data):
            nonlocal arm_contact, hand_collision
            self.physics_step(data)
            arm_touch, hands_touch = self._record_robot_contacts(data)
            arm_contact |= arm_touch
            hand_collision |= hands_touch

        cycles = self.control.track(targets, count, physics_step=sample_physics)
        for wrist_position_error, wrist_angle_error in cycles:
            arm_touch, hands_touch = self._record_robot_contacts(self.data)
            self.arm_contact_steps += int(arm_contact or arm_touch)
            self.hand_collision_steps += int(hand_collision or hands_touch)
            arm_contact = hand_collision = False
            metrics = self.metrics.update(self.data)
            if on_cycle is not None:
                on_cycle(wrist_position_error, wrist_angle_error)
            if self.invalid_hand_motion:
                metrics["sim_success"] = False
                break
            if self.data.warning.number.any() or (
                control_cycles is None
                and np.max(wrist_position_error) <= WRIST_POSITION_TOLERANCE_M
                and np.max(wrist_angle_error) <= WRIST_ANGLE_TOLERANCE_RAD
            ):
                break
        return metrics, wrist_position_error, wrist_angle_error

    def close(self):
        self.camera.close()
