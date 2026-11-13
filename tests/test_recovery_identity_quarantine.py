"""Recovery isolation must not be cleared by a malformed frame identity."""
import numpy as np
import pytest

from egohand.vio.recovery_coordinator import RecoveryCoordinator
from egohand.vio.recovery_monitor import RecoveryMonitor


def frame(epoch, stamp, *, consistent=True):
    return {'timestamp_ns': stamp, 'frame_id': stamp, 'map_epoch': epoch,
            'world_frame_id': epoch, 'map_id': epoch >> 40,
            'reset_id': (epoch >> 20) & ((1 << 20) - 1),
            'map_revision': epoch & ((1 << 20) - 1),
            'snapshot_available': True, 'snapshot_consistent': consistent,
            'tracking_state': 'OK', 'pose_valid': True,
            'imu_initialized': True, 'inertial_scale_finalized': True,
            'matrix': np.eye(4).tolist()}


@pytest.mark.parametrize('kind', ['inconsistent', 'missing', 'mismatch', 'nonnumeric', 'bool', 'float'])
def test_transient_untrusted_identity_cannot_clear_existing_isolation(kind):
    """The next valid frame returns to the original map, not to a new segment."""
    original = 1 << 40
    first = frame(original, 1_000_000_000)
    # Mirrors native early publication: default map identity on an inconsistent frame.
    bad = frame(0, 1_010_000_000, consistent=False)
    if kind == 'missing': bad.pop('snapshot_available')
    if kind == 'mismatch':
        bad['snapshot_consistent'] = True
        bad['world_frame_id'] = original
    if kind in ('nonnumeric', 'bool', 'float'):
        bad['snapshot_consistent'] = True
        bad['map_epoch'] = {'nonnumeric':'0', 'bool':False, 'float':0.0}[kind]
    c = RecoveryCoordinator(stable_seconds=0)
    c.finish_frame(first, None)
    c._quarantine('pose_jump', 'unavailable')
    m = RecoveryMonitor()
    m.observe(first, [])
    m.await_visual_requalification = True
    c.finish_frame(bad, None)
    m.observe(bad, [])
    cstate = c.finish_frame(frame(original, 1_020_000_000), None)
    m.observe(frame(original, 1_020_000_000), [])
    assert cstate['await_visual_requalification']
    assert m.await_visual_requalification


def test_consistent_new_map_can_still_open_a_new_segment():
    c = RecoveryCoordinator(stable_seconds=0)
    first = frame(1 << 40, 1_000_000_000)
    c.finish_frame(first, None)
    c._quarantine('pose_jump', 'unavailable')
    m = RecoveryMonitor()
    m.observe(first, [])
    m.await_visual_requalification = True
    fresh = frame(2 << 40, 1_010_000_000)
    assert not c.finish_frame(fresh, None)['await_visual_requalification']
    m.observe(fresh, [])
    assert not m.await_visual_requalification
