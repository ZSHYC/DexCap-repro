from pathlib import Path
import unittest

import numpy as np


try:
    import mujoco
except ImportError:  # The project simulation extra is optional for unrelated tests.
    mujoco = None


ASSETS = Path(__file__).resolve().parents[1] / "data/mujoco_menagerie"


@unittest.skipIf(mujoco is None, "mujoco is not installed")
class MujocoRobotAssetsTest(unittest.TestCase):
    def test_finger_collision_pieces_clear_concavities_and_retain_contact(self):
        from simulation.mujoco_assets import _load_hand

        positions = (
            ([-.1530701, -.0733279, -.0837274, 0, 0, 0, 0, 0], None),
            ([-.3144989, -1.0230371, -.4721639, -.366, 0, 0, 0, 0], None),
            ([-.1409182, -.0970604, -.1112902, .2381753,
              -.308781, -.3465988, -.0846278, -.2483467], (7, 12)),
        )
        for side in ("right", "left"):
            model = _load_hand(side).compile()
            data = mujoco.MjData(model)
            qpos = [model.joint(f"hand_{i}").qposadr[0] for i in range(4, 12)]
            body = {
                i: model.jnt_bodyid[model.joint(f"hand_{i - 1}").id] for i in (5, 7, 12)
            }
            for pose, (angles, expected) in enumerate(positions):
                with self.subTest(side=side, pose=pose):
                    data.qpos[qpos] = angles
                    mujoco.mj_forward(model, data)
                    body_pairs = [
                        set(model.geom_bodyid[np.asarray(c.geom)])
                        for c in data.contact[:data.ncon] if c.dist < 0
                    ]
                    self.assertNotIn({body[5], body[7]}, body_pairs)
                    if expected is not None:
                        self.assertIn({body[i] for i in expected}, body_pairs)

    def test_palm_shell_clears_measured_thumb_shank_poses(self):
        from simulation.mujoco_assets import _load_hand

        positions = (
            (
                .66044707, -.19274138, .59105834, .57361082,
                -.15164648, -.07933945, -.29952167, 0.0,
                -.29212783, .00052131, -.24949975, 0.0,
                .95121824, .56628131, .35120388, 1.28769834,
            ),
            (
                .68470671, -.14276708, .58989570, .55965313,
                -.15162836, -.07933377, -.29951979, 0.0,
                -.29211119, .00052725, -.24949797, 0.0,
                .95104111, .60381306, .35007304, 1.29001912,
            ),
        )
        for side in ("right", "left"):
            model = _load_hand(side).compile()
            data = mujoco.MjData(model)
            joints = [model.joint(f"hand_{i}").qposadr[0] for i in range(16)]
            palm_body = model.geom("hand_collision_0").bodyid[0]
            thumb_geom = model.geom("hand_collision_14").id
            for pose, angles in enumerate(positions):
                with self.subTest(side=side, pose=pose):
                    data.qpos[joints] = angles
                    mujoco.mj_forward(model, data)
                    self.assertFalse(any(
                        thumb_geom in contact.geom and contact.dist < 0
                        and palm_body in model.geom_bodyid[contact.geom]
                        for contact in data.contact
                    ))

    def test_distal_joint_stop_resists_a_servo_strength_external_load(self):
        from simulation.mujoco_assets import _load_hand

        spec = _load_hand("right")
        spec.option.timestep = 0.002
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        model = spec.compile()
        joint = model.joint("hand_3")
        actuator = model.actuator("finger_3")
        for sign, limit in zip((-1, 1), joint.range):
            with self.subTest(sign=sign):
                data = mujoco.MjData(model)
                data.qpos[joint.qposadr[0]] = limit - sign * .01
                data.ctrl[actuator.id] = limit
                data.qfrc_applied[joint.dofadr[0]] = sign * model.actuator_forcerange[actuator.id, 1]
                peak_excess = 0.0
                for _ in range(125):
                    mujoco.mj_step(model, data)
                    peak_excess = max(peak_excess, sign * (data.qpos[joint.qposadr[0]] - limit))

                self.assertLess(peak_excess, .005)
                self.assertFalse(np.any(data.warning.number))

    def test_adds_two_articulated_pandas_and_leap_hands(self):
        from simulation.hand_joints import decode_leap_joints
        from simulation.mujoco_assets import (
            ARM_ACTUATORS,
            ARM_JOINTS,
            HAND_ACTUATORS,
            HAND_JOINTS,
            WRIST_SITES,
            add_robots,
        )

        if not (ASSETS / "franka_emika_panda/panda_nohand.xml").exists():
            self.skipTest("run scripts/fetch_mujoco_assets.sh first")

        spec = mujoco.MjSpec()
        add_robots(
            spec,
            ASSETS,
            [[0.0, -0.4, 0.0], [0.0, 0.4, 0.0]],
            [[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        )
        model = spec.compile()

        self.assertEqual(model.nq, 46)
        self.assertEqual(model.nu, 46)
        for side in ("right", "left"):
            for name in ARM_JOINTS[side] + HAND_JOINTS[side]:
                self.assertGreaterEqual(model.joint(name).id, 0)
            for name in ARM_ACTUATORS[side] + HAND_ACTUATORS[side]:
                actuator = model.actuator(name)
                self.assertGreaterEqual(actuator.id, 0)
                self.assertTrue(model.actuator_forcelimited[actuator.id])
                self.assertTrue(np.isfinite(model.actuator_forcerange[actuator.id]).all())
            # LEAP V1's physical fingertip and thumb masses exclude IK markers.
            self.assertAlmostEqual(model.body(f"{side}_fingertip").mass[0], 0.016)
            self.assertAlmostEqual(model.body(f"{side}_thumb_fingertip").mass[0], 0.049)
            self.assertEqual(sum(
                model.geom(index).name.removeprefix(f"{side}_hand_collision_").isdecimal()
                for index in range(model.ngeom)
            ), 17)
            wrist = model.site(WRIST_SITES[side])
            palm = model.body(f"{side}_palm_lower")
            self.assertEqual(model.site_bodyid[wrist.id], palm.id)
            np.testing.assert_array_equal(wrist.pos, np.zeros(3))

        self.assertTrue(any(model.geom_contype))
        right_palm = model.geom("right_hand_visual_0")
        left_palm = model.geom("left_hand_visual_0")
        np.testing.assert_allclose(right_palm.pos[[0, 2]], left_palm.pos[[0, 2]], atol=1e-9)
        self.assertAlmostEqual(right_palm.pos[1], -left_palm.pos[1])
        self.assertEqual(right_palm.type, mujoco.mjtGeom.mjGEOM_MESH)
        np.testing.assert_array_equal(model.geom("right_hand_collision_0").rgba, np.zeros(4))

        right_axes = np.array([model.jnt_axis[model.joint(name).id] for name in HAND_JOINTS["right"]])
        left_axes = np.array([model.jnt_axis[model.joint(name).id] for name in HAND_JOINTS["left"]])
        np.testing.assert_allclose(left_axes, right_axes * [-1, 1, -1])

        torque_limits = (87, 87, 87, 87, 12, 12, 12)
        for side in ("right", "left"):
            for name, force in zip(ARM_ACTUATORS[side], torque_limits):
                actuator = model.actuator(name).id
                np.testing.assert_array_equal(model.actuator_ctrlrange[actuator], [-force, force])
                self.assertEqual(model.actuator_biastype[actuator], mujoco.mjtBias.mjBIAS_NONE)
            for name in HAND_ACTUATORS[side]:
                actuator = model.actuator(name).id
                self.assertEqual(model.actuator_biastype[actuator], mujoco.mjtBias.mjBIAS_AFFINE)
                np.testing.assert_array_equal(model.actuator_forcerange[actuator], [-0.95, 0.95])

        data = mujoco.MjData(model)
        for side in ("right", "left"):
            neutral = decode_leap_joints(np.zeros(16), left=side == "left")
            for name, angle in zip(HAND_JOINTS[side], neutral):
                joint = model.joint(name)
                data.qpos[joint.qposadr[0]] = np.clip(angle, *joint.range)
        mujoco.mj_forward(model, data)
        contacts = [set(contact.geom) for contact in data.contact[:data.ncon]]
        for side in ("right", "left"):
            palm = model.body(f"{side}_palm_lower")
            palm_collision = {
                geom for geom in np.flatnonzero(model.geom_bodyid == palm.id)
                if model.geom_contype[geom]
            }
            thumb_shank = {model.geom(f"{side}_hand_collision_{index}").id for index in (14, 15)}
            self.assertFalse(any(pair & palm_collision and pair & thumb_shank for pair in contacts))
            self.assertGreater(len(palm_collision), 1)
            self.assertAlmostEqual(palm.mass[0], 0.237)

        palm_centers = {"right": [-0.05, -0.037, 0.0113], "left": [-0.05, 0.037, 0.0113]}
        for side, center in palm_centers.items():
            palm = model.body(f"{side}_palm_lower")
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, palm.quat)
            rotation = rotation.reshape(3, 3)
            np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
            np.testing.assert_allclose(rotation @ [0, 0, 1], [0, 0, -1], atol=1e-12)
            registered_center = palm.pos + rotation @ center
            np.testing.assert_allclose(registered_center, [0, 0, 0.006], atol=1e-12)
            self.assertLess(registered_center[2], palm.pos[2])


if __name__ == "__main__":
    unittest.main()
