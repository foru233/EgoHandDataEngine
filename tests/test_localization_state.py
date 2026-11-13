import numpy as np
import pytest

from egohand.vio.localization_state import localization_evidence, recovery_evidence


def frame(**updates):
    value = dict(timestamp_ns=1_000_000_000, tracking_state='OK', mode='stereo_inertial',
                 pose_valid=True, raw_pose_valid=True, matrix=np.eye(4).tolist(),
                 snapshot_available=True,snapshot_consistent=True, map_epoch=3, world_frame_revision=0,
                 frame_id=7,world_frame_id=3,
                 map_id=0,reset_id=0,map_revision=3,
                 imu_initialized=True, ba2_complete=True, inertial_scale_finalized=True,
                 body_state_candidate={'snapshot_consistent': True, 'ba2_complete':True,'frame_id': 7,
                     'timestamp_ns': 1_000_000_000, 'world_frame_id': 3,
                     'T_world_from_body': np.eye(4).tolist(), 'velocity_world_mps': [0, 0, 0],
                     'accel_bias_mps2': [0, 0, 0], 'gyro_bias_rps': [0, 0, 0]})
    value.update(updates)
    return value


def test_map_pose_is_timestamped_but_odom_is_not_invented():
    pose = np.eye(4); pose[:3, 3] = [1, 2, 3]
    state = localization_evidence(frame(matrix=pose.tolist()))
    assert state['state'] == 'VIO_TRACKING'
    assert state['timestamp_ns'] == 1_000_000_000
    np.testing.assert_allclose(state['map']['T_map_from_camera'], pose)
    assert state['odom']['status'] == 'unavailable'
    assert state['odom']['T_odom_from_camera'] is None and state['T_map_from_odom'] is None
    assert state['coordinate_continuity_verified'] is False


@pytest.mark.parametrize('changes', [
    {'snapshot_consistent': None},
    {'snapshot_consistent': False},
    {'pose_valid': False},
    {'tracking_state': 'LOST'},
    {'display_map_coordinates_preserved': False},
    {'matrix': [[float('nan')]*4]*4},
    {'matrix': np.diag([-1, 1, 1, 1]).tolist()},
    {'recovery_preaccept': {'await_visual_requalification': True}},
    {'recovery_monitor_awaiting_visual_requalification': True},
])
def test_map_output_fails_closed_for_missing_or_incompatible_evidence(changes):
    state = localization_evidence(frame(**changes))
    assert state['map']['status'] == 'unavailable'
    assert state['map']['T_map_from_camera'] is None
    assert state['odom']['status'] == 'unavailable'


def test_imu_prediction_is_not_visual_or_odom_pose():
    state = localization_evidence(frame(tracking_state='RECENTLY_LOST', pose_valid=False,
        recovery_preaccept={'loss_prediction': {'status': 'predicted','mode':'imu_only_extrapolation_not_visual_pose'}}))
    assert state['state'] == 'RELOCALIZING'
    assert state['prediction'] == {'status': 'predicted', 'source': 'imu_only_not_visual_pose'}
    assert state['map']['T_map_from_camera'] is None
    assert state['odom']['T_odom_from_camera'] is None


def test_refinement_and_visual_modes_are_distinct():
    assert localization_evidence(frame(mode='stereo'))['state'] == 'VISUAL_TRACKING'
    assert localization_evidence(frame(imu_initialized=False))['state'] == 'INERTIAL_INITIALIZING'
    assert localization_evidence(frame(ba2_complete=False))['state'] == 'INERTIAL_REFINING'
    assert localization_evidence(frame(inertial_scale_finalized=None))['state'] == 'INERTIAL_REFINING'
    assert localization_evidence(frame(body_state_candidate=None))['state'] == 'DEGRADED'
    assert localization_evidence(frame(frame_id=8))['state'] == 'DEGRADED'
    assert localization_evidence(frame(mode='hybrid',pose_source='visual_fallback'))['state'] == 'VISUAL_TRACKING'
    assert localization_evidence(frame(mode='hybrid',pose_source='vio',inertial_display_ready=False))['state'] == 'DEGRADED'
    assert localization_evidence(frame(mode='unknown'))['state'] == 'DEGRADED'


def test_display_gate_waiting_preserves_inertial_phase_without_fabricating_pose():
    """A healthy native tracker can be waiting for the inertial output gate."""
    waiting = dict(pose_valid=False, raw_pose_valid=True,
                   raw_matrix=np.eye(4).tolist(), matrix=None)
    init = localization_evidence(frame(**waiting, imu_initialized=False,
                                       ba2_complete=False, inertial_scale_finalized=False))
    assert init['state'] == 'INERTIAL_INITIALIZING'
    assert init['raw_tracking_pose_available'] is True
    assert init['pose_output_available'] is False
    assert init['map']['T_map_from_camera'] is None
    assert init['odom']['T_odom_from_camera'] is None
    refining = localization_evidence(frame(**waiting, imu_initialized=True,
                                           ba2_complete=False, inertial_scale_finalized=False))
    assert refining['state'] == 'INERTIAL_REFINING'
    assert refining['map']['status'] == 'unavailable'


def test_waiting_phase_does_not_hide_real_jump_or_bad_snapshot():
    waiting = dict(pose_valid=False, raw_pose_valid=True,
                   raw_matrix=np.eye(4).tolist(), matrix=None, imu_initialized=False)
    assert localization_evidence(frame(**waiting, jump_detected=True))['state'] == 'DEGRADED'
    assert localization_evidence(frame(**waiting, snapshot_consistent=False))['state'] == 'DEGRADED'


def test_contradictory_accepted_jump_is_not_exposed_as_map_pose():
    # A downstream consumer must not receive a pose when upstream evidence
    # simultaneously reports a rejected jump, even if pose_valid is stale.
    state = localization_evidence(frame(jump_detected=True))
    assert state['state'] == 'DEGRADED'
    assert state['pose_output_available'] is False
    assert state['map']['status'] == 'unavailable'
    assert state['map']['T_map_from_camera'] is None


@pytest.mark.parametrize('coordinator,monitor,status,disagree', [
    (True, False, 'quarantined', True),
    (False, True, 'quarantined', True),
    (True, True, 'quarantined', False),
    (False, False, 'not_quarantined_reported', False),
    (None, False, 'unknown', None),
    (False, None, 'unknown', None),
    (None, True, 'quarantined', None),
    (None, None, 'unknown', None),
])
def test_recovery_evidence_preserves_sources_and_unknowns(coordinator, monitor, status, disagree):
    value = {'recovery_preaccept': {'await_visual_requalification': coordinator},
             'recovery_monitor_awaiting_visual_requalification': monitor}
    evidence = recovery_evidence(value)
    assert evidence['status'] == status
    assert evidence['source_disagreement'] is disagree
    assert evidence['coordinator']['await_visual_requalification'] is coordinator
    assert evidence['monitor']['await_visual_requalification'] is monitor
    assert evidence['advisory_only'] is True
    assert evidence['independent_visual_verification'] == 'not_assessed'


def test_recovery_evidence_malformed_inputs_do_not_claim_clear():
    evidence = recovery_evidence({'recovery_preaccept': {'await_visual_requalification': 'false',
        'quarantine_elapsed_s': float('nan'), 'quarantine_timed_out': 0},
        'recovery_monitor_awaiting_visual_requalification': False})
    assert evidence['status'] == 'unknown'
    assert evidence['coordinator']['elapsed_s'] is None
    assert evidence['coordinator']['timed_out'] is None
    assert recovery_evidence({'recovery_preaccept': 'bad'})['status'] == 'unknown'


def test_recovery_evidence_carries_cause_and_timeout_without_admission():
    recovery = {'await_visual_requalification': True, 'quarantine_cause': 'pose_jump',
        'evidence_status': 'unavailable', 'quarantine_elapsed_s': 105.8,
        'quarantine_timed_out': True, 'timeout_action': 'report_only',
        'anchor_available': False, 'request_new_map': False}
    state = localization_evidence(frame(recovery_preaccept=recovery,
        recovery_monitor_awaiting_visual_requalification=False))
    assert state['recovery_evidence']['coordinator'] == {
        'await_visual_requalification': True, 'cause': 'pose_jump',
        'evidence_status': 'unavailable', 'elapsed_s': 105.8, 'timed_out': True,
        'timeout_action': 'report_only', 'anchor_available': False, 'request_new_map': False}
    assert state['recovery_evidence']['status'] == 'quarantined'
    assert state['state'] == 'RELOCALIZING'
    assert state['map']['T_map_from_camera'] is None
