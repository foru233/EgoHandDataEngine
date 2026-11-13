import numpy as np
import pytest
from dataclasses import replace

from egohand.vio.trajectory import OfflinePoseProvider, load_euroc_trajectory, transform_hand_points
from egohand.vio.hand_world import world_hands_for_result
from egohand.core.types import Hand3D, StereoHandResult


def test_body_extrinsic_and_integer_timestamp(tmp_path):
    trajectory = tmp_path / 'trajectory.txt'
    trajectory.write_text('1403636579963555584.000000 1 2 3 0 0 0 1\n')
    extrinsic = np.eye(4)
    extrinsic[:3, 3] = [0.1, 0.2, 0.3]
    pose = load_euroc_trajectory(trajectory, output_frame='body', calibration_id='test', T_body_from_camera=extrinsic)[0]
    assert pose.timestamp_ns == 1403636579963555584
    np.testing.assert_allclose(transform_hand_points([[0, 0, 1]], pose.T_world_from_camera_left), [[1.1, 2.2, 4.3]])
    assert 'unknown' in pose.tracking_status


def test_trajectory_requires_explicit_valid_calibration_and_timestamps(tmp_path):
    trajectory = tmp_path / 'trajectory.txt'
    trajectory.write_text('10 0 0 0 0 0 0 1\n10 0 0 0 0 0 0 1\n')
    with pytest.raises(ValueError):
        load_euroc_trajectory(trajectory, output_frame='body', calibration_id='test')
    with pytest.raises(ValueError):
        load_euroc_trajectory(trajectory, output_frame='camera', calibration_id='test', T_exported_camera_from_camera_left=np.eye(4))


def test_camera_frame_requires_explicit_rectification(tmp_path):
    trajectory = tmp_path / 'trajectory.txt'
    trajectory.write_text('10 0 0 0 0 0 0 1\n')
    with pytest.raises(ValueError, match='校正相机'):
        load_euroc_trajectory(trajectory, output_frame='camera', calibration_id='test')


def test_provider_does_not_reuse_pose_for_missing_frame(tmp_path):
    trajectory = tmp_path / 'trajectory.txt'
    trajectory.write_text('1000000000 0 0 0 0 0 0 1\n1100000000 0 0 0 0 0 0 1\n')
    poses = load_euroc_trajectory(trajectory, output_frame='camera', calibration_id='test', T_exported_camera_from_camera_left=np.eye(4))
    provider = OfflinePoseProvider(poses, max_time_error_ns=1000)
    assert provider.pose_at(1000000256) is poses[0]
    assert provider.pose_at(1050000000) is None
    with pytest.raises(LookupError):
        provider.camera_to_world(1050000000)


def test_hand_world_is_separate_and_checks_calibration_and_clock(tmp_path):
    trajectory = tmp_path / 'trajectory.txt'
    trajectory.write_text('1000000000 1 2 3 0 0 0 1\n')
    provider = OfflinePoseProvider(load_euroc_trajectory(trajectory, output_frame='camera', calibration_id='cal', T_exported_camera_from_camera_left=np.eye(4)))
    hand = Hand3D('Left', 1.0, 7)
    hand.joints[0].valid = True
    hand.joints[0].xyz_m = np.array([0., 0., 1.])
    result = StereoHandResult(2, 1000000000, 'cal', 'test', hands=[hand], session_id='session')
    world = world_hands_for_result(result, provider, left_image_timestamp_ns=1000000000, result_clock_domain='sensor', pose_clock_domain='sensor', pose_session_id='session')
    np.testing.assert_allclose(world['hands'][0]['xyz_world_m'][0], [1, 2, 4])
    np.testing.assert_allclose(hand.joints[0].xyz_m, [0, 0, 1])
    assert world['hands'][0]['track_id'] == 7
    assert world['world_reference'] == {'schema_version': 1, 'map_epoch': None,
        'world_frame_revision': None, 'identity_status': 'unverified',
        'coordinate_continuity': 'unknown', 'cross_frame_merge_permitted': False}
    assert world_hands_for_result(result, provider, left_image_timestamp_ns=1000000000, result_clock_domain='a', pose_clock_domain='b', pose_session_id='session')['status'] == 'unavailable'
    assert world_hands_for_result(result, provider, left_image_timestamp_ns=999, result_clock_domain='sensor', pose_clock_domain='sensor', pose_session_id='session')['reason'] == 'left_image_timestamp_mismatch'
    assert world_hands_for_result(result, provider, left_image_timestamp_ns=1000000000, result_clock_domain='sensor', pose_clock_domain='sensor', pose_session_id='other')['reason'] == 'session_mismatch'
    result.calibration_id = 'different'
    assert world_hands_for_result(result, provider, left_image_timestamp_ns=1000000000, result_clock_domain='sensor', pose_clock_domain='sensor', pose_session_id='session')['reason'] == 'calibration_mismatch'


def test_engine_binding_is_session_scoped_and_exports_world(tmp_path, source, engine):
    from egohand.storage.exporter import export_all
    frame = source.read()
    frame = replace(frame, left_timestamp_ns=frame.timestamp_ns,
                    metadata={**frame.metadata, 'clock_domain': 'sensor'})
    trajectory = tmp_path / 'trajectory.txt'
    trajectory.write_text(f'{frame.timestamp_ns} 1 2 3 0 0 0 1\n')
    poses = load_euroc_trajectory(trajectory, output_frame='camera',
        calibration_id=engine.calibration.calibration_id, T_exported_camera_from_camera_left=np.eye(4))
    engine.bind_offline_pose_provider(OfflinePoseProvider(poses), clock_domain='sensor', pose_session_id=engine.session_id)
    result = engine.process(frame)
    assert result.world_hands['status'] == 'available'
    assert result.world_hands['hands'][0]['track_id'] == result.hands[0].track_id
    joint = next(j for j in result.hands[0].joints if j.valid)
    np.testing.assert_allclose(result.world_hands['hands'][0]['xyz_world_m'][joint.joint_id], joint.xyz_m + [1, 2, 3])
    destination = tmp_path / 'export'
    export_all([result], destination)
    assert (destination / 'world_hands.json').is_file()
    assert (destination / 'world_reference_policy.json').is_file()
    engine.reset_tracking()
    assert engine.vio_binding is None


def test_export_marks_legacy_and_distinct_world_references_unmergeable(tmp_path):
    import json
    from egohand.storage.exporter import export_all, load_json

    first = StereoHandResult(1, 100, 'cal', 'test', session_id='s')
    second = StereoHandResult(2, 200, 'cal', 'test', session_id='s')
    third = StereoHandResult(3, 300, 'cal', 'test', session_id='s')
    # Legacy data carries no identity; a caller's bare boolean cannot certify
    # that two map references or optimizer revisions are continuous.
    first.world_hands = {'status': 'available', 'hands': []}
    second.world_hands = {'status': 'available', 'hands': [], 'world_reference': {
        'map_epoch': 1 << 40, 'world_frame_revision': 2,
        'identity_status': 'verified', 'coordinate_continuity': 'continuous',
        'cross_frame_merge_permitted': True}}
    third.world_hands = {'status': 'available', 'hands': [], 'world_reference': {
        'map_epoch': 2 << 40, 'world_frame_revision': 2}}
    destination = tmp_path / 'export'; export_all([first, second, third], destination)
    world = json.loads((destination/'world_hands.json').read_text(encoding='utf8'))
    restored = load_json(destination/'results.json')
    for exported in (world, [row.world_hands for row in restored]):
        assert exported[0]['world_reference']['map_epoch'] is None
        assert exported[1]['world_reference']['map_epoch'] == 1 << 40
        assert exported[2]['world_reference']['map_epoch'] == 2 << 40
        assert all(row['world_reference']['cross_frame_merge_permitted'] is False for row in exported)
        assert all(row['world_reference']['coordinate_continuity'] == 'unknown' for row in exported)
    assert first.world_hands == {'status': 'available', 'hands': []}  # no source mutation
    assert second.world_hands['world_reference']['cross_frame_merge_permitted'] is True
    policy = json.loads((destination/'world_reference_policy.json').read_text(encoding='utf8'))
    assert policy['cross_frame_merge_permitted'] is False
