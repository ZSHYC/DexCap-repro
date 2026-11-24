"""Single-hand sponge pickup and lift from DexCap appendix A-F."""

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
        touch_geoms = {geom for c in contacts for geom in c.geom if geom >= 0}
        counts = [len(touch_geoms & hand) for hand in self.hands]
        if self.table not in touch_geoms and min(counts) == 0 and max(counts) >= 2:
            lift = float(data.flexvert_xpos[self.vertices, 2].mean() - self.initial_height)
            self.max_lift_m = max(self.max_lift_m, lift)
        return {
            "sponge_grasped": self.max_lift_m > 0.01,
            "sponge_lift_m": float(self.max_lift_m),
            "sim_success": self.max_lift_m > MIN_LIFT_M,
        }
