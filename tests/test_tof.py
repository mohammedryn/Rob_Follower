import numpy as np
import pytest
from unittest.mock import MagicMock


def _make_tof_mock(ready: bool, grid: list[int] | None = None):
    """Build a mock VL53L5CX sensor object."""
    tof = MagicMock()
    tof.data_ready.return_value = ready
    if grid is not None:
        data = MagicMock()
        data.distance_mm = grid
        tof.get_data.return_value = data
    return tof


def _flat_grid(value: int) -> list[int]:
    return [value] * 64


class TestGetDepthInfo:
    def test_no_data_returns_failsafe(self):
        """When sensor has no fresh data, return fail-safe stop values."""
        from rpms.tof import get_depth_info
        tof = _make_tof_mock(ready=False)
        dist, obstacle = get_depth_info(tof)
        assert dist == 0
        assert obstacle is True

    def test_clear_path_at_follow_distance(self):
        from rpms.tof import get_depth_info
        tof = _make_tof_mock(ready=True, grid=_flat_grid(800))
        dist, obstacle = get_depth_info(tof)
        assert dist == 800
        assert obstacle is False

    def test_obstacle_in_peripheral_zone(self):
        """Any zone < STOP_DIST_MM triggers obstacle flag."""
        from rpms.tof import get_depth_info
        grid = _flat_grid(800)
        grid[0] = 300  # corner zone, outside center 4x4
        tof = _make_tof_mock(ready=True, grid=grid)
        dist, obstacle = get_depth_info(tof)
        assert obstacle is True

    def test_obstacle_in_center_zone(self):
        from rpms.tof import get_depth_info
        grid = _flat_grid(800)
        grid[2 * 8 + 2] = 350  # zone at row=2, col=2 (inside center 4x4)
        tof = _make_tof_mock(ready=True, grid=grid)
        dist, obstacle = get_depth_info(tof)
        assert obstacle is True

    def test_center_dist_is_min_of_center_zones(self):
        """dist return value reflects the minimum in the center 4x4 block."""
        from rpms.tof import get_depth_info
        grid = _flat_grid(1000)
        grid[2 * 8 + 3] = 750  # row=2, col=3: inside center 4x4, closer
        tof = _make_tof_mock(ready=True, grid=grid)
        dist, obstacle = get_depth_info(tof)
        assert dist == 750
        assert obstacle is False

    def test_zero_values_ignored(self):
        """VL53L5CX returns 0 for invalid readings; these must not trigger stops."""
        from rpms.tof import get_depth_info
        grid = _flat_grid(800)
        grid[5] = 0  # invalid reading — should be ignored
        tof = _make_tof_mock(ready=True, grid=grid)
        dist, obstacle = get_depth_info(tof)
        assert obstacle is False

    def test_all_zero_grid_returns_safe_default(self):
        """If all readings are invalid (0), return FOLLOW_DIST_MM, no obstacle."""
        from rpms.tof import get_depth_info
        from rpms.config import FOLLOW_DIST_MM
        tof = _make_tof_mock(ready=True, grid=_flat_grid(0))
        dist, obstacle = get_depth_info(tof)
        assert dist == FOLLOW_DIST_MM
        assert obstacle is False
