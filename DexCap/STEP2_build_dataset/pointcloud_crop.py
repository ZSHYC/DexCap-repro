import numpy as np
from scipy.linalg import svd


_TABLE_CORNERS = np.array([
    [-0.262721, -0.25, -0.077183],
    [0.228490, -0.23, -0.079729],
    [0.390924, -0.215, -0.783124],
    [-0.372577, -0.235, -0.781769],
])
_TABLE_GAPS = (0.020, 0.021, 0.022, 0.023, 0.024, 0.025)
_CENTROID = _TABLE_CORNERS.mean(axis=0)
_NORMAL = svd(_TABLE_CORNERS - _CENTROID)[2][-1]
_OFFSET = -np.dot(_NORMAL, _CENTROID)


def table_point_mask(xyz, minimum_points):
    """Select the first published table-plane crop with enough visible points."""
    heights = np.dot(xyz, _NORMAL) + _OFFSET
    for gap in _TABLE_GAPS:
        keep = heights + gap >= 0
        if np.count_nonzero(keep) > minimum_points:
            return keep
    return np.ones(len(xyz), dtype=bool)
