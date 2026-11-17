import numpy as np


def test_display_world_axes_are_standard_right_handed():
    axes = np.eye(3)
    assert np.linalg.det(axes) == 1.0
    np.testing.assert_allclose(np.cross(axes[:, 0], axes[:, 1]), axes[:, 2])
