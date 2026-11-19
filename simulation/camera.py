import numpy as np
import pybullet as p

from DexCap.STEP2_build_dataset.pointcloud_crop import table_point_mask


def capture_pointcloud(client, mocap_from_camera, bullet_from_mocap, rng, *, hand_bodies):
    """Render the current scene as 10,000 XYZRGB points in the DexCap mocap frame."""
    scale = 4
    width, height = 1280 // scale, 720 // scale
    fx, fy, cx, cy = np.array([898.201, 897.867, 657.498, 364.310]) / scale
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
    rows, cols = np.nonzero(depth < 1 - 1e-6)
    if not len(rows):
        raise RuntimeError("Camera rendered no surfaces within the DexCap depth range")
    z = near * far / (far - (far - near) * depth[rows, cols])
    xyz = np.column_stack(((cols + 0.5 - cx) * z / fx, (rows + 0.5 - cy) * z / fy, z))
    xyz = xyz @ mocap_from_camera[:3, :3].T + mocap_from_camera[:3, 3]
    points = 10_000
    # The reduced render has one pixel for each scale-by-scale patch of sensor pixels.
    visible = np.flatnonzero(table_point_mask(xyz, points // scale**2))
    indices = rng.choice(visible, points, replace=len(visible) < points)
    rows, cols = rows[indices], cols[indices]
    rgb = np.asarray(rgba).reshape(height, width, 4)[rows, cols, :3] / 255.0
    body_ids = np.asarray(segmentation).reshape(height, width)[rows, cols]
    # The dataset builder appends retargeted LEAP surface points with zero RGB.
    rgb[np.isin(body_ids, hand_bodies)] = 0.0
    return np.concatenate((xyz[indices], rgb), axis=-1).astype(np.float32)
