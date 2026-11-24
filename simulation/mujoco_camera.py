import mujoco
import numpy as np
import trimesh

from DexCap.STEP2_build_dataset.pointcloud_crop import table_point_mask
from simulation.frames import BULLET_FROM_MOCAP, HAND_MESH_OFFSETS, LEAP_TO_HAND


IMAGE_SIZE = np.array([1280, 720])
FOCAL = np.array([898.201, 897.867])
PRINCIPAL = np.array([657.498, 364.310])
CAMERA_NAME = "dexcap_camera"
POINTS = 10_000
HAND_POINTS = 80_000


def add_camera(spec: mujoco.MjSpec, mocap_from_camera, *, world_from_mocap=BULLET_FROM_MOCAP):
    """Add the source-calibrated L515 camera to a scene in mocap coordinates."""
    mocap_from_camera = np.asarray(mocap_from_camera, dtype=float)
    world_from_camera = np.asarray(world_from_mocap, dtype=float) @ mocap_from_camera
    # RealSense uses +Y down/+Z forward; MuJoCo cameras use +Y up/-Z forward.
    rotation = world_from_camera[:3, :3] @ np.diag([1.0, -1.0, -1.0])
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, int(IMAGE_SIZE[0]))
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, int(IMAGE_SIZE[1]))
    return spec.worldbody.add_camera(
        name=CAMERA_NAME,
        pos=world_from_camera[:3, 3],
        quat=quat,
        resolution=IMAGE_SIZE,
        focal_pixel=FOCAL,
        # MuJoCo specifies principal point as an offset from the image center.
        principal_pixel=IMAGE_SIZE / 2 - PRINCIPAL,
        sensor_size=[1.0, 1.0],
    )


def _surface_mesh(model, geom_id):
    geom_type = model.geom_type[geom_id]
    size = model.geom_size[geom_id]
    if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_id = model.geom_dataid[geom_id]
        vert = model.mesh_vertadr[mesh_id]
        face = model.mesh_faceadr[mesh_id]
        return trimesh.Trimesh(
            vertices=model.mesh_vert[vert:vert + model.mesh_vertnum[mesh_id]],
            faces=model.mesh_face[face:face + model.mesh_facenum[mesh_id]],
            process=False,
        )
    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        return trimesh.creation.box(extents=2 * size)
    if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
        return trimesh.creation.icosphere(radius=size[0])
    if geom_type == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        mesh = trimesh.creation.icosphere()
        mesh.vertices *= size
        return mesh
    if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return trimesh.creation.cylinder(radius=size[0], height=2 * size[1])
    if geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return trimesh.creation.capsule(radius=size[0], height=2 * size[1])
    raise ValueError(f"Unsupported LEAP visual geom type {geom_type}")


def _hand_atlas(model, prefix):
    geoms = [
        geom_id for geom_id in range(model.ngeom)
        if (model.geom(geom_id).name or "").startswith(prefix)
        and "_visual_" in (model.geom(geom_id).name or "")
        and model.geom_group[geom_id] < 3
        and model.geom_rgba[geom_id, 3] > 0
    ]
    meshes = [(geom_id, _surface_mesh(model, geom_id)) for geom_id in geoms]
    area = sum(mesh.area for _, mesh in meshes)
    if not area:
        return []
    samples = []
    remaining = HAND_POINTS
    for index, (geom_id, mesh) in enumerate(meshes):
        count = remaining if index == len(meshes) - 1 else round(HAND_POINTS * mesh.area / area)
        points, _ = trimesh.sample.sample_surface(mesh, count, seed=geom_id)
        samples.append((geom_id, points))
        remaining -= count
    return samples


class DexCapCamera:
    """Render the policy's 10,000-point XYZRGB observation in the mocap frame."""

    def __init__(self, model, mocap_from_camera, *, world_from_mocap=BULLET_FROM_MOCAP):
        self.model = model
        self.mocap_from_camera = np.asarray(mocap_from_camera, dtype=float)
        self.camera_from_mocap = np.linalg.inv(self.mocap_from_camera)
        self.mocap_from_world = np.linalg.inv(np.asarray(world_from_mocap, dtype=float))
        self.renderer = mujoco.Renderer(
            model, height=int(IMAGE_SIZE[1]), width=int(IMAGE_SIZE[0])
        )
        self.options = mujoco.MjvOption()
        self.options.geomgroup[:] = 0
        self.options.geomgroup[:3] = 1
        self.robot_geoms = {
            geom_id for geom_id in range(model.ngeom)
            if (model.geom(geom_id).name or "").startswith(
                ("right_arm_", "left_arm_", "right_hand_", "left_hand_")
            )
        }
        self.hands = [
            (
                side_index,
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_wrist"),
                _hand_atlas(model, f"{side}_hand_"),
            )
            for side_index, side in enumerate(("right", "left"))
        ]

    def _render(self, data):
        self.renderer.update_scene(data, camera=CAMERA_NAME, scene_option=self.options)
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(data, camera=CAMERA_NAME, scene_option=self.options)
        depth = self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        self.renderer.enable_segmentation_rendering()
        self.renderer.update_scene(data, camera=CAMERA_NAME, scene_option=self.options)
        segmentation = self.renderer.render().copy()
        self.renderer.disable_segmentation_rendering()
        return rgb, depth, segmentation

    def _visible_hands(self, data):
        clouds = []
        for side, site_id, samples in self.hands:
            if site_id < 0 or not samples:
                continue
            offset = LEAP_TO_HAND.T @ ([0.0, 0.0, -0.1] - HAND_MESH_OFFSETS[side])
            center = offset @ data.site_xmat[site_id].reshape(3, 3).T + data.site_xpos[site_id]
            center = center @ self.mocap_from_world[:3, :3].T + self.mocap_from_world[:3, 3]
            center = center @ self.camera_from_mocap[:3, :3].T
            center += self.camera_from_mocap[:3, 3]
            if center[2] <= 0:
                continue
            pixel = (center[:2] * FOCAL / center[2] + PRINCIPAL).astype(int)
            resized = (pixel / IMAGE_SIZE * 84).astype(int)
            if (resized < 0).any() or (resized >= 84).any():
                continue
            clouds.extend(
                points @ data.geom_xmat[geom_id].reshape(3, 3).T + data.geom_xpos[geom_id]
                for geom_id, points in samples
            )
        if not clouds:
            return np.empty((0, 3))
        world = np.concatenate(clouds)
        return world @ self.mocap_from_world[:3, :3].T + self.mocap_from_world[:3, 3]

    def capture(self, data, rng):
        """Capture the current scene as 10,000 XYZRGB points in mocap coordinates."""
        rgb_image, depth, segmentation = self._render(data)
        geom_ids = segmentation[:, :, 0]
        object_types = segmentation[:, :, 1]
        rows, cols = np.nonzero(
            (object_types == mujoco.mjtObj.mjOBJ_FLEX)
            | (
                (object_types == mujoco.mjtObj.mjOBJ_GEOM)
                & ~np.isin(geom_ids, tuple(self.robot_geoms))
            )
        )
        z = depth[rows, cols]
        xyz = np.column_stack((
            (cols + 0.5 - PRINCIPAL[0]) * z / FOCAL[0],
            (rows + 0.5 - PRINCIPAL[1]) * z / FOCAL[1],
            z,
        ))
        xyz = xyz @ self.mocap_from_camera[:3, :3].T + self.mocap_from_camera[:3, 3]
        colors = rgb_image[rows, cols] / 255.0
        hands = self._visible_hands(data)
        if len(hands):
            xyz = np.concatenate((xyz, hands))
            colors = np.concatenate((colors, np.zeros_like(hands)))
        if not len(xyz):
            raise RuntimeError("Camera has no surfaces in the DexCap observation")
        visible = np.flatnonzero(table_point_mask(xyz, POINTS))
        indices = rng.choice(visible, POINTS, replace=len(visible) < POINTS)
        return np.concatenate((xyz[indices], colors[indices]), axis=1).astype(np.float32)

    def close(self):
        self.renderer.close()
