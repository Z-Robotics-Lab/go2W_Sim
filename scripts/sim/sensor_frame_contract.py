"""Raw Mid-360 frame contract shared by the Isaac navigation bridge tests.

The LiDAR and link-attached IMU publish measurements in the same physical,
20-degree-pitched sensor frame. ARISE estimates their common gravity alignment
and exposes the resulting level navigation frame as ``sensor``. Applying the
mount pitch to either raw stream again would tilt the SLAM map.
"""


def to_navigation_imu(acceleration, angular_velocity):
    """Preserve raw IMU vectors for ARISE's shared gravity alignment."""
    return tuple(acceleration), tuple(angular_velocity)
