import numpy as np
import pytest


@pytest.fixture
def blank_frame():
    return np.zeros((480, 640, 3), dtype=np.uint8)


@pytest.fixture
def mock_tof_data_factory():
    """Returns a factory that builds mock VL53L5CX data objects."""
    from unittest.mock import MagicMock

    def _factory(distances: list[int]):
        """distances: list of 64 ints (8x8 grid, row-major)."""
        data = MagicMock()
        data.distance_mm = distances
        return data

    return _factory
