"""Measured volumetric sponge shared by the DexCap physical tasks."""

import math

import mujoco
import numpy as np


TABLE_HEIGHT = -0.03
SPONGE_HALF_XY = (0.04, 0.065)
SPONGE_HALF_HEIGHT = 0.017
SPONGE_YAW = 0.33

# The released depth measurement fixes the size, not the foam material parameters.
SPONGE_YOUNG_MODULUS_PA = 50_000
SPONGE_POISSON_RATIO = 0.25
SPONGE_DAMPING_S = 0.02


def sponge_flexcomp(sponge_xy):
    yaw_half = SPONGE_YAW / 2
    return f"""
            <flexcomp name="wiping_sponge" type="grid" count="5 6 3"
                      spacing="0.02 0.026 {SPONGE_HALF_HEIGHT}" dim="3" radius="0.002"
                      pos="{sponge_xy[0]} {sponge_xy[1]} {TABLE_HEIGHT + SPONGE_HALF_HEIGHT + 0.001}"
                      quat="{math.cos(yaw_half)} 0 0 {math.sin(yaw_half)}"
                      mass="0.06" dof="trilinear" rgba="1 1 1 1">
              <elasticity young="{SPONGE_YOUNG_MODULUS_PA}" poisson="{SPONGE_POISSON_RATIO}"
                          damping="{SPONGE_DAMPING_S}"/>
              <contact selfcollide="none" friction="1.2 0.01 0.001"
                       solref="0.005 1" solimp="0.9 0.99 0.001"/>
            </flexcomp>
    """


def texture_sponge(spec):
    # Color boundaries come from the five depth and RGB samples in frame_2220.
    height = 128
    bands = np.array([[19, 21, 12], [151, 129, 42], [48, 47, 24], [163, 147, 62], [236, 212, 106]])
    edges = [-0.0116, -0.0015, 0.0086, 0.0148]
    levels = np.linspace(-SPONGE_HALF_HEIGHT, SPONGE_HALF_HEIGHT, height)
    texture = spec.add_texture(
        name="wiping_sponge_bands", type=mujoco.mjtTexture.mjTEXTURE_2D, width=1, height=height, nchannel=3,
    )
    texture.data = bands[np.searchsorted(edges, levels)].astype(np.uint8).tobytes()
    material = spec.add_material(name="wiping_sponge_material", specular=0)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = texture.name
    flex = spec.flex("wiping_sponge")
    z = np.asarray(flex.vert).reshape(-1, 3)[:, 2]
    vertical = (0.5 + (height - 1) * (z - z.min()) / np.ptp(z)) / height
    flex.texcoord = np.column_stack((np.full(len(z), 0.5), vertical)).ravel()
    flex.material = material.name
