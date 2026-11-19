import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
import trimesh

from DexCap.STEP2_build_dataset.pointcloud_crop import table_point_mask
from simulation.frames import HAND_MESH_OFFSETS, LEAP_TO_HAND

IMAGE_SIZE = np.array([1280, 720])
FOCAL = np.array([898.201, 897.867])
PRINCIPAL = np.array([657.498, 364.310])
RENDER_SCALE = 4


def sample_hand_surfaces(client, hand_bodies):
    """Sample the LEAP visual geometry in each link's local frame."""
    meshes = {}
    hands = []
    for body in hand_bodies:
        visuals = p.getVisualShapeData(body, physicsClientId=client)
        parts = []
        for _, link, geometry, dimensions, filename, pos, quat, _ in visuals:
            key = (geometry, dimensions, filename)
            if key not in meshes:
                if geometry == p.GEOM_SPHERE:
                    meshes[key] = trimesh.creation.icosphere(radius=dimensions[0])
                else:
                    mesh = trimesh.load_mesh(filename.decode(), process=False)
                    mesh.vertices *= dimensions
                    meshes[key] = mesh
            parts.append((link, meshes[key], pos, quat))
        area = sum(mesh.area for _, mesh, _, _ in parts)
        # DexCap samples 80,000 points per hand at the full RGB-D resolution.
        count = 80_000 // RENDER_SCALE**2
        samples = []
        for index, (link, mesh, pos, quat) in enumerate(parts):
            points, _ = trimesh.sample.sample_surface(mesh, round(count * mesh.area / area), seed=index)
            samples.append((link, Rotation.from_quat(quat).apply(points) + pos))
        hands.append(samples)
    return hands


def _visible_hand_points(client, body, samples, side, bullet_from_camera):
    pos, quat = p.getBasePositionAndOrientation(body, physicsClientId=client)
    rotation = Rotation.from_quat(quat)
    offset = LEAP_TO_HAND.T @ ([0.0, 0.0, -0.1] - HAND_MESH_OFFSETS[side])
    center = rotation.apply(offset) + pos
    x, y, z = (center - bullet_from_camera[:3, 3]) @ bullet_from_camera[:3, :3]
    if z <= 0:
        return np.empty((0, 3))
    # Match the dataset's integer projection at the sensor and 84×84 image sizes.
    sensor_pixel = (np.array([x, y]) * FOCAL / z + PRINCIPAL).astype(int)
    image_pixel = (sensor_pixel / IMAGE_SIZE * 84).astype(int)
    if (image_pixel < 0).any() or (image_pixel >= 84).any():
        return np.empty((0, 3))

    links = [link for link, _ in samples[1:]]
    states = p.getLinkStates(body, links, computeForwardKinematics=True, physicsClientId=client)
    poses = [(pos, quat), *(state[4:6] for state in states)]
    return np.concatenate([
        Rotation.from_quat(orientation).apply(points) + position
        for (position, orientation), (_, points) in zip(poses, samples)
    ])


def capture_pointcloud(
    client, mocap_from_camera, bullet_from_mocap, rng, *, hand_bodies, hand_samples, table_height=None,
):
    """Render the current scene as 10,000 XYZRGB points in the DexCap mocap frame."""
    width, height = IMAGE_SIZE // RENDER_SCALE
    fx, fy = FOCAL / RENDER_SCALE
    cx, cy = PRINCIPAL / RENDER_SCALE
    near, far = 0.05, 1.0
    bullet_from_camera = bullet_from_mocap @ mocap_from_camera
    eye = bullet_from_camera[:3, 3]
    view = p.computeViewMatrix(eye, eye + bullet_from_camera[:3, 2], -bullet_from_camera[:3, 1])
    projection = p.computeProjectionMatrix(
        -cx * near / fx, (width - cx) * near / fx,
        -(height - cy) * near / fy, cy * near / fy,
        near, far,
    )
    _, _, rgba, depth, segmentation = p.getCameraImage(
        width, height, view, projection, renderer=p.ER_TINY_RENDERER, physicsClientId=client
    )
    depth = np.asarray(depth).reshape(height, width)
    segmentation = np.asarray(segmentation).reshape(height, width)
    rows, cols = np.nonzero((depth < 1 - 1e-6) & ~np.isin(segmentation, hand_bodies))
    z = near * far / (far - (far - near) * depth[rows, cols])
    xyz = np.column_stack(((cols + 0.5 - cx) * z / fx, (rows + 0.5 - cy) * z / fy, z))
    xyz = xyz @ mocap_from_camera[:3, :3].T + mocap_from_camera[:3, 3]
    rgb = np.asarray(rgba).reshape(height, width, 4)[rows, cols, :3] / 255.0
    hands = np.concatenate([
        _visible_hand_points(client, body, samples, side, bullet_from_camera)
        for side, (body, samples) in enumerate(zip(hand_bodies, hand_samples))
    ])
    hand_xyz = (hands - bullet_from_mocap[:3, 3]) @ bullet_from_mocap[:3, :3]
    xyz = np.concatenate((xyz, hand_xyz))
    if not len(xyz):
        raise RuntimeError("Camera has no surfaces in the DexCap observation")
    rgb = np.concatenate((rgb, np.zeros_like(hand_xyz)))
    points = 10_000
    keep = table_point_mask(xyz, points // RENDER_SCALE**2)
    if table_height is not None:
        # The recorded crop can sit above low objects on a flat simulated table.
        height = xyz @ bullet_from_mocap[2, :3] + bullet_from_mocap[2, 3]
        keep |= height >= table_height + 0.020
    visible = np.flatnonzero(keep)
    indices = rng.choice(visible, points, replace=len(visible) < points)
    return np.concatenate((xyz[indices], rgb[indices]), axis=-1).astype(np.float32)
