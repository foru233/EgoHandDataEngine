import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('recovery_timeline',
    Path(__file__).resolve().parents[1] / 'scripts/audit_vio_recovery_timeline.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def row(index, state='OK', native=True, admitted=True, reset=0):
    return audit.normalize(dict(timestamp_ns=1_000_000_000+index*100_000_000,
        frame_id=index, tracking_state=state, raw_pose_valid=native,
        pose_valid=admitted, reset_id=reset, map_id=0), index)


def test_native_recovery_host_wait_and_new_map_are_distinct():
    rows = [row(0), row(1, 'RECENTLY_LOST', False, False),
        row(2, admitted=False), row(3, admitted=False), row(4, reset=1)]
    episode = audit.summarize(rows)['loss_episodes'][0]
    assert episode['native_recovery']['index'] == 2
    assert episode['application_readmission']['index'] == 4
    assert episode['native_to_application_s'] == pytest.approx(.2)
    assert episode['same_reference_readmission'] is False
    assert episode['first_qualified_geometry'] is None


def test_missing_diagnostics_are_unknown_and_next_loss_does_not_count_as_recovery():
    rows = [row(0), row(1, 'LOST', False, False), row(2, admitted=False),
        row(3, 'LOST', False, False), row(4)]
    summary = audit.summarize(rows)
    assert not summary['prediction_diagnostics_available']
    assert summary['loss_episodes'][0]['application_readmission'] is None
    assert summary['loss_episodes'][1]['application_readmission']['index'] == 4


def test_trace_binding_and_monotonic_time_are_required():
    with pytest.raises(ValueError, match='bound'):
        audit.normalize(dict(timestamp_ns=10, frame_id=1), 0,
            dict(timestamp_ns=11, frame_id=1))
    with pytest.raises(ValueError, match='strictly increasing'):
        audit.summarize([row(1), row(1)])
