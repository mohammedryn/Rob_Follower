import numpy as np
from rpms.config import STOP_DIST_MM, FOLLOW_DIST_MM


def init_tof():
    from vl53l5cx_ctypes import VL53L5CX
    tof = VL53L5CX()
    tof.set_resolution(8 * 8)
    tof.start_ranging()
    return tof


def get_depth_info(tof) -> tuple[int, bool]:
    """Return (min_center_dist_mm, obstacle_detected).

    Fail-safe: if no fresh data is available, returns (0, True) so the
    caller stops the robot rather than continuing on a stale reading.
    """
    if not tof.data_ready():
        return 0, True

    data = tof.get_data()
    grid = np.array(data.distance_mm, dtype=np.int32).reshape(8, 8)

    # Center 4x4 block: zones [2:6, 2:6] face directly forward
    center_flat = grid[2:6, 2:6].flatten()
    center_valid = center_flat[center_flat > 0]
    min_center = int(np.min(center_valid)) if len(center_valid) > 0 else FOLLOW_DIST_MM

    # Obstacle check uses center zones only — outer zones see robot's own chassis
    obstacle = bool(np.any(center_valid < STOP_DIST_MM)) if len(center_valid) > 0 else False

    return min_center, obstacle
