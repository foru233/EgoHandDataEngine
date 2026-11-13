import numpy as np

from egohand.vio.pose_continuity import PoseContinuityCorrector


def pose(x, y, z, angle_deg=0.0):
    c, s = np.cos(np.radians(angle_deg)), np.sin(np.radians(angle_deg))
    m = np.eye(4)
    m[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    m[:3, 3] = [x, y, z]
    return m


def feed(corrector, matrix, stamp_ns):
    return corrector.process({'pose_valid': True, 'matrix': matrix.tolist(), 'timestamp_ns': stamp_ns})


def test_continuous_motion_is_not_modified():
    corrector = PoseContinuityCorrector()
    first = feed(corrector, pose(0, 0, 0), 0)
    second = feed(corrector, pose(0.05, 0, 0), 33_000_000)
    assert first['continuity_applied'] is False
    assert second['continuity_applied'] is False
    assert np.allclose(np.array(second['matrix']), pose(0.05, 0, 0))


def test_relocalization_jump_is_reanchored_and_relative_motion_kept():
    corrector = PoseContinuityCorrector(min_jump_m=0.25)
    feed(corrector, pose(0, 0, 0), 0)
    feed(corrector, pose(0.02, 0, 0), 33_000_000)
    jumped = feed(corrector, pose(5.0, 3.0, -2.0), 66_000_000)
    assert jumped['continuity_applied'] is True
    assert np.allclose(np.array(jumped['matrix'])[:3, 3], [0.02, 0, 0], atol=1e-9)
    follow = feed(corrector, pose(5.03, 3.0, -2.0), 99_000_000)
    assert follow['continuity_applied'] is False
    assert np.allclose(np.array(follow['matrix'])[:3, 3], [0.05, 0, 0], atol=1e-9)


def test_large_rotation_jump_is_reanchored():
    corrector = PoseContinuityCorrector(min_jump_m=10.0)
    feed(corrector, pose(0, 0, 0, 0), 0)
    turned = feed(corrector, pose(0, 0, 0, 120.0), 33_000_000)
    assert turned['continuity_applied'] is True


def test_disabled_corrector_leaves_jump():
    corrector = PoseContinuityCorrector(enabled=False)
    feed(corrector, pose(0, 0, 0), 0)
    jumped = feed(corrector, pose(9, 9, 9), 33_000_000)
    assert jumped['continuity_applied'] is False
    assert np.allclose(np.array(jumped['matrix'])[:3, 3], [9, 9, 9])


def test_invalid_pose_is_ignored_and_last_pose_kept():
    corrector = PoseContinuityCorrector()
    feed(corrector, pose(0, 0, 0), 0)
    skipped = corrector.process({'pose_valid': False, 'matrix': None, 'timestamp_ns': 33_000_000})
    assert skipped['continuity_applied'] is False
    resumed = feed(corrector, pose(0.02, 0, 0), 66_000_000)
    assert resumed['continuity_applied'] is False
