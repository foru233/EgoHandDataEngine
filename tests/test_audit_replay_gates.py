import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    'audit_replay_gates', Path(__file__).resolve().parents[1] / 'scripts/audit_replay_gates.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_summary_distinguishes_gate_counts_quarantine_and_host_timings():
    rows = [
        {'timestamp_ns': 1_000_000_000, 'tracking_state': 'OK', 'pose_valid': False,
         'quality_gate': {'reason_code': 'stable_window_pending'},
         'recovery_preaccept': {'await_visual_requalification': True, 'quarantine_cause': 'pose_jump'}},
        {'timestamp_ns': 2_000_000_000, 'tracking_state': 'OK', 'pose_valid': True,
         'quality_gate': {'reason_code': 'accepted'}},
    ]
    report = audit.summarize(rows, {'bridge_calls': [
        {'ipc_track_ms': 10}, {'ipc_track_ms': 20}, {'ipc_track_ms': float('nan')}]})
    assert report['frames'] == 2
    assert report['admitted_frames'] == 1
    assert report['first_admission_recorded_seconds'] == 1
    assert report['quarantined_frame_causes'] == {'pose_jump': 1}
    assert report['quality_reason_counts'] == {'stable_window_pending': 1, 'accepted': 1}
    assert report['ipc_track_ms']['count'] == 2
    assert report['ipc_track_ms']['p50'] == 15


def test_missing_measurements_remain_unknown():
    report = audit.summarize([{}], {})
    assert report['quality_reason_counts'] == {'missing': 1}
    assert report['first_admission_recorded_seconds'] is None
    assert report['ipc_track_ms']['p95'] is None


def test_loss_events_keep_identity_and_distinguish_tracking_from_admission():
    rows = [
        {'timestamp_ns': 1, 'tracking_state': 'OK', 'pose_valid': True, 'map_epoch': 1},
        {'timestamp_ns': 2, 'tracking_state': 'RECENTLY_LOST', 'pose_valid': False, 'map_epoch': 1},
        {'timestamp_ns': 3, 'tracking_state': 'OK', 'pose_valid': False, 'map_epoch': 1,
         'quality_gate': {'reason_code': 'stable_window_pending'}},
        {'timestamp_ns': 4, 'tracking_state': 'RECENTLY_LOST', 'pose_valid': False, 'map_epoch': 1},
        {'timestamp_ns': 5, 'tracking_state': 'OK', 'pose_valid': True, 'map_epoch': 2},
        {'timestamp_ns': 6, 'tracking_state': 'LOST', 'pose_valid': False, 'map_epoch': 2},
    ]
    episodes = audit.summarize(rows, {})['loss_episodes_after_first_admission']
    assert len(episodes) == 2
    assert episodes[0]['first_tracking_ok']['timestamp_ns'] == 3
    assert episodes[0]['first_admitted']['timestamp_ns'] == 5
    assert episodes[0]['loss']['map_epoch'] != episodes[0]['first_admitted']['map_epoch']
    assert episodes[0]['quality_reasons_before_admission']['stable_window_pending'] == 1
    assert episodes[0]['lost_frames_after_first_tracking_ok'] == 1
    assert episodes[0]['first_qualified_visual_release'] is None
    assert episodes[1]['censored'] is True and episodes[1]['first_admitted'] is None
