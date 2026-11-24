"""Assemble the DexCap robot, calibrated camera, and task in one MuJoCo model."""

import mujoco
import numpy as np

from simulation.frames import TABLE_SHIFT
from simulation.mujoco_control import DualArmController


TABLE_HEIGHT = -0.030
PHYSICS_STEP = 0.002
MAX_CONTROL_CYCLES = 10
WRIST_POSITION_TOLERANCE_M = 0.005
WRIST_ANGLE_TOLERANCE_RAD = np.deg2rad(5)


def rig_base_positions(frames):
    # The released retargeter expresses each wrist in parallel axes with a different origin.
    return np.array([TABLE_SHIFT - absolute for absolute, _, _ in frames.offsets])


class MujocoScene:
    def __init__(self, asset_root, frames, camera, task, object_xy, object_name):
        from simulation.mujoco_assets import add_robots
        from simulation.mujoco_camera import DexCapCamera, add_camera

        spec = mujoco.MjSpec()
        spec.modelname = f"dexcap_{task}"
        spec.option.timestep = PHYSICS_STEP
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        spec.option.iterations = 80
        color = [0.12, 0.12, 0.12, 1] if task == "wiping" else [0.64, 0.59, 0.49, 1]
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
        else:
            from simulation.mujoco_packaging import PackagingMetrics, add_packaging_scene

            add_packaging_scene(spec, object_xy, object_name)
            metrics_type = PackagingMetrics
            metrics_args = (object_name,)

        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.control = DualArmController(self.model, self.data)
        self.metrics = metrics_type(self.model, *metrics_args)
        self.camera = DexCapCamera(self.model, camera)
        self.arm_geoms = np.array([
            (self.model.geom(index).name or "").startswith(("right_arm_", "left_arm_"))
            for index in range(self.model.ngeom)
        ])
        self.arm_contact_steps = 0

    def reset(self, initial):
        self.control.initialize(initial)

    def step(self, targets):
        for wrist_position_error, wrist_angle_error in self.control.track(targets, MAX_CONTROL_CYCLES):
            geometries = self.data.contact.geom[:self.data.ncon]
            rigid_contacts = geometries[geometries >= 0]
            self.arm_contact_steps += int(self.arm_geoms[rigid_contacts].any())
            metrics = self.metrics.update(self.data)
            if (np.max(wrist_position_error) <= WRIST_POSITION_TOLERANCE_M
                    and np.max(wrist_angle_error) <= WRIST_ANGLE_TOLERANCE_RAD):
                break
        return metrics, wrist_position_error, wrist_angle_error

    def close(self):
        self.camera.close()
