"""Application admission histories, including stale and quarantined evidence."""
import copy

import numpy as np
import pytest

from egohand.vio.hybrid_quality import make_quality_gate
from egohand.vio.localization_state import LocalizationPipeline, localization_evidence, consumer_map_pose
from egohand.vio.map_identity import decode_map_epoch, snapshot_body_matches_frame


def frame(stamp, *, epoch=0, state='OK', x=0.0, **changes):
    matrix = np.eye(4); matrix[0, 3] = x
    value = dict(timestamp_ns=stamp, frame_id=stamp, map_epoch=epoch, world_frame_id=epoch,
        world_frame_revision=0, **decode_map_epoch(epoch), snapshot_available=True,
        snapshot_consistent=True, pose_valid=state == 'OK', tracking_state=state,
        matrix=matrix.tolist() if state == 'OK' else None,
        imu_initialized=True, ba2_complete=True, inertial_scale_finalized=True,
        body_state_candidate=dict(timestamp_ns=stamp, frame_id=stamp, world_frame_id=epoch,
            snapshot_consistent=True, ba2_complete=True, T_world_from_body=matrix.tolist(),
            velocity_world_mps=[0, 0, 0], accel_bias_mps2=[0, 0, 0], gyro_bias_rps=[0, 0, 0]))
    value.update(changes)
    return value


def pipeline(mode='stereo_inertial'):
    return LocalizationPipeline(mode, make_quality_gate(mode, stable_seconds=0))


def samples(stamp):
    return [(stamp, 0, 0, 9.81, 0, 0, 0)]


def test_consumers_accept_pipeline_contract_and_keep_old_payload_adapter():
    row = pipeline().process(frame(1_000_000_000), [])
    original = copy.deepcopy(row)
    transform = consumer_map_pose(row, world_hands=True, expected_timestamp_ns=row['timestamp_ns'])
    np.testing.assert_array_equal(transform, row['matrix'])
    transform[0, 3] = 99
    assert row == original
    assert consumer_map_pose(row, world_hands=True, expected_timestamp_ns=2_000_000_000) is None
    legacy = {'pose_valid':True, 'matrix':np.eye(4).tolist()}
    np.testing.assert_array_equal(consumer_map_pose(legacy), np.eye(4))


@pytest.mark.parametrize('mutation', [
    lambda row: row['localization']['admission'].update(map_pose_available=False),
    lambda row: row['localization']['admission'].update(map_pose_available=1),
    lambda row: row['localization'].update(quarantined=True),
    lambda row: row['localization'].update(timestamp_ns=5),
    lambda row: row['localization'].update(map_epoch=2),
    lambda row: row['localization'].update(world_frame_revision=True),
    lambda row: row['localization'].update(contract_version=99),
    lambda row: row['localization'].update(contract_version=2.0),
    lambda row: row['localization'].update(native_snapshot_consistent=False),
    lambda row: row['localization']['lifecycle'].update(timestamp_monotonic=False),
    lambda row: row['localization']['map'].update(camera_frame='body'),
    lambda row: row['localization']['map'].update(T_map_from_camera=np.diag([2., 1., 1., 1.]).tolist()),
    lambda row: row['matrix'][0].__setitem__(3, 10),
    lambda row: row.update(localization=None),
    lambda row: row.update(recovery_preaccept={'await_visual_requalification':True}),
])
def test_consumer_contract_contradiction_cannot_fall_back_to_legacy_valid(mutation):
    row = pipeline().process(frame(1_000_000_000), [])
    assert row['pose_valid']
    mutation(row)
    assert consumer_map_pose(row) is None


def test_world_hand_admission_is_distinct_from_trajectory_admission():
    row = pipeline().process(frame(1_000_000_000), [])
    row['localization']['admission']['world_hands_permitted'] = False
    assert consumer_map_pose(row) is not None
    assert consumer_map_pose(row, world_hands=True) is None


def test_native_readiness_transitions_keep_entry_time_and_explicit_admission():
    p = pipeline()
    first = p.process(frame(1, imu_initialized=False), [])
    assert first['localization']['state'] == 'INERTIAL_INITIALIZING'
    assert first['localization']['lifecycle']['transition']['from'] == 'WAITING_FOR_SENSORS'
    second = p.process(frame(2, ba2_complete=False), [])
    assert second['localization']['state'] == 'INERTIAL_REFINING'
    third = p.process(frame(3), samples(3))
    fourth = p.process(frame(4), samples(4))
    assert third['localization']['state'] == fourth['localization']['state'] == 'VIO_TRACKING'
    assert fourth['localization']['lifecycle']['entered_timestamp_ns'] == 3
    assert fourth['localization']['lifecycle']['transition'] is None
    assert fourth['localization']['admission']['map_pose_available'] is True
    assert fourth['localization']['odom']['status'] == 'unavailable'
    assert fourth['localization']['admission']['cross_frame_merge_permitted'] is False


def test_coordinator_quarantine_cannot_anchor_or_disappear_on_imu_reset():
    p = pipeline()
    row = frame(1_000_000_000, recovery_preaccept={'await_visual_requalification': True})
    original = copy.deepcopy(row)
    out = p.process(row, samples(row['timestamp_ns']))
    assert row == original
    assert out['raw_pose_valid'] and not out['pose_valid']
    assert out['recovery_gate']['gate_id'] == 'VIO.RECOVERY_QUARANTINE'
    assert p.recovery_monitor.predictor.anchor is None
    p.recovery_monitor.await_visual_requalification = True
    p.reset_imu_alignment()
    assert p.recovery_monitor.await_visual_requalification
    out = p.process(frame(1_033_333_333), samples(1_033_333_333))
    assert not out['pose_valid'] and out['localization']['state'] == 'RELOCALIZING'


def test_loss_history_survives_imu_reset_and_requires_new_map_or_verification():
    p = pipeline()
    p.process(frame(1_000_000_000), samples(1_000_000_000))
    assert p.recovery_monitor.predictor.anchor is not None
    p.process(frame(1_010_000_000, state='LOST'), samples(1_010_000_000))
    p.reset_imu_alignment()
    blocked = p.process(frame(1_020_000_000), samples(1_020_000_000))
    assert not blocked['pose_valid']
    assert p.recovery_monitor.predictor.anchor is None
    new_map = p.process(frame(1_030_000_000, epoch=1 << 40), samples(1_030_000_000))
    assert new_map['pose_valid']
    assert new_map['map_revision_decision']['event'] == 'map_or_reset_changed'
    assert new_map['localization']['coordinate_continuity_verified'] is False


def test_imu_reset_retains_reference_identity_for_first_new_map_and_missing_world():
    p = pipeline()
    p.process(frame(1_000_000_000, epoch=1 << 40), samples(1_000_000_000))
    p.recovery_monitor.await_visual_requalification = True
    p.reset_imu_alignment()
    # A sensor-buffer reset must not lose the reference that a subsequent
    # explicitly authenticated new map is compared with.
    fresh = p.process(frame(1_010_000_000, epoch=2 << 40), samples(1_010_000_000))
    assert fresh['pose_valid']
    assert not p.recovery_monitor.await_visual_requalification
    p.reset_imu_alignment()
    missing = p.process(frame(1_020_000_000, epoch=2 << 40, world_frame_revision=None),
                        samples(1_020_000_000))
    assert not missing['pose_valid']
    assert missing['recovery_check']['reason'] == 'world_revision_unavailable'


@pytest.mark.parametrize('stamp', [0, 999_999_999, 1_000_000_000, True, '1000000001', None])
def test_stale_or_malformed_frame_cannot_poison_anchors_or_quality_history(stamp):
    p = pipeline()
    p.process(frame(1_000_000_000), samples(1_000_000_000))
    anchor = p.recovery_monitor.predictor.anchor
    previous_stamp = p.quality.previous[0]
    bad = frame(1_010_000_000, epoch=1 << 40, x=100)
    bad['timestamp_ns'] = stamp
    rejected = p.process(bad, samples(1_010_000_000))
    assert not rejected['pose_valid']
    assert rejected['localization']['state'] == 'DEGRADED'
    assert rejected['quality_gate']['gate_id'] == 'VIO.TIMESTAMP'
    assert p.recovery_monitor.predictor.anchor is anchor
    assert p.quality.previous[0] == previous_stamp
    assert p.process(frame(1_020_000_000), samples(1_020_000_000))['pose_valid']


@pytest.mark.parametrize('changes', [dict(map_id=99), dict(world_frame_revision=True),
                                      dict(snapshot_consistent=False)])
def test_contract_does_not_publish_contradictory_map_identity(changes):
    row = frame(1, **changes)
    assert localization_evidence(row)['map']['status'] == 'unavailable'
    assert pipeline().process(row, [])['localization']['map']['status'] == 'unavailable'


@pytest.mark.parametrize('key,value', [('frame_id', True), ('timestamp_ns', '1'),
                                     ('world_frame_id', 0.0)])
def test_shared_body_identity_refuses_coercible_wrong_types(key, value):
    row = frame(1)
    assert snapshot_body_matches_frame(row, row['body_state_candidate'])
    row['body_state_candidate'][key] = value
    assert not snapshot_body_matches_frame(row, row['body_state_candidate'])


def test_gate_reports_jump_without_changing_raw_matrix_or_calling_it_odom():
    p = pipeline('stereo')
    p.process(frame(1_000_000_000), [])
    rejected = p.process(frame(1_033_333_333, x=5), [])
    assert rejected['quality_gate']['reason_code'] == 'pose_jump'
    assert rejected['raw_matrix'][0][3] == 5 and rejected['matrix'] is None
    assert rejected['localization']['T_map_from_odom'] is None
