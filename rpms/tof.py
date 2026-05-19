import time
import numpy as np
from rpms.config import STOP_DIST_MM, FOLLOW_DIST_MM

_cache_dist     = FOLLOW_DIST_MM
_cache_obstacle = False
_cache_time     = 0.0
_CACHE_TTL      = 1.0  # seconds before stale data triggers fail-safe


def init_tof():
    from vl53l5cx_ctypes import VL53L5CX
    tof = VL53L5CX()
    tof.set_resolution(8 * 8)
    tof.set_ranging_frequency_hz(10)
    tof.start_ranging()
    return tof


def get_depth_info(tof) -> tuple[int, bool]:
    """Return (min_center_dist_mm, obstacle_detected).

    Returns cached reading when fresh data isn't ready yet.
    Only triggers fail-safe if cache is older than _CACHE_TTL.
    """
    global _cache_dist, _cache_obstacle, _cache_time

    if not tof.data_ready():
        if time.monotonic() - _cache_time > _CACHE_TTL:
            return 0, True  # sensor silent too long — stop
        return _cache_dist, _cache_obstacle

    data = tof.get_data()
    grid = np.array(data.distance_mm, dtype=np.int32).reshape(8, 8)

    center_flat  = grid[2:6, 2:6].flatten()
    center_valid = center_flat[center_flat > 0]
    min_center   = int(np.min(center_valid)) if len(center_valid) > 0 else FOLLOW_DIST_MM
    obstacle     = bool(np.any(center_valid < STOP_DIST_MM)) if len(center_valid) > 0 else False

    _cache_dist     = min_center
    _cache_obstacle = obstacle
    _cache_time     = time.monotonic()
    return min_center, obstacle
