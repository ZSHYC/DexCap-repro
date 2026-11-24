"""Single-hand sponge pickup and lift from DexCap appendix A-F."""

from itertools import combinations

import mujoco
import numpy as np

from simulation.mujoco_sponge import sponge_flexcomp, texture_sponge


DEMO_OBJECT_XY = np.array([[0.54, 0.0]])
MIN_LIFT_M = 0.30


def sample_object_xy(rng):
    return DEMO_OBJECT_XY + rng.uniform([-0.20, -0.35], [0.20, 0.35], size=(1, 2))


def add_sponge_picking_scene(spec, object_xy):
    child = mujoco.MjSpec.from_string(
        f"""<mujoco model="dexcap_sponge_picking">
          <worldbody>{sponge_flexcomp(object_xy[0])}</worldbody>
        </mujoco>"""
    )
    texture_sponge(child)
    spec.attach(child, frame=spec.worldbody.add_frame(name="sponge_picking_scene"), prefix="")


class SpongePickingMetrics:
    def __init__(self, model):
        self.flex = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, "wiping_sponge")
        start = model.flex_vertadr[self.flex]
        self.vertices = slice(start, start + model.flex_vertnum[self.flex])
        self.initial_height = model.flex_vert[self.vertices, 2].mean()
        self.table = model.geom("table").id
        self.hands = [
            {geom for geom in range(model.ngeom) if model.geom(geom).name.startswith(f"{side}_hand_")}
            for side in ("right", "left")
        ]
        self.max_lift_m = 0.0

    def update(self, data):
        contacts = [c for c in data.contact[:data.ncon] if self.flex in c.flex]
        if not any(self.table in contact.geom for contact in contacts):
            grips = []
            for hand in self.hands:
                normals = []
                for contact in contacts:
                    first, second = contact.geom
                    if first in hand:
                        normals.append((first, contact.frame[:3]))
                    elif second in hand:
                        normals.append((second, -contact.frame[:3]))
                if normals:
                    grips.append(normals)
            if len(grips) == 1 and any(
                first != second and np.dot(a, b) < -0.25
                for (first, a), (second, b) in combinations(grips[0], 2)
            ):
                lift = float(data.flexvert_xpos[self.vertices, 2].mean() - self.initial_height)
                self.max_lift_m = max(self.max_lift_m, lift)
        return {
            "sponge_grasped": self.max_lift_m > 0.01,
            "sponge_lift_m": float(self.max_lift_m),
            "sim_success": self.max_lift_m > MIN_LIFT_M,
        }
