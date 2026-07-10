"""Frame normalization shared by the Isaac navigation bridge tests.

The current Isaac Lab integration reports the link-attached IMU vectors in the
navigation-aligned articulation frame. The Mid-360 visual/RTX prim still keeps
its physical 20 degree mount pose, but applying that pose again here rotates a
level gravity vector and tilts the SLAM map.
"""


def to_navigation_imu(acceleration, angular_velocity):
    """Return Isaac IMU vectors without applying the visual mount transform."""
    return tuple(acceleration), tuple(angular_velocity)
