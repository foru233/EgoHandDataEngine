import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from evaluate_public_recovery_trajectory import reference_intervals, score_interval
from audit_recovery_prediction_gt import relative_motion_error


def test_returned_identity_does_not_join_separate_intervals():
    def row(index, map_id):
        return {'index': index, 'raw': {'map_id': map_id, 'reset_id': 0, 'map_revision': 1,
            'world_frame_revision': 0, 'reference_generation': 1, 'reference_frame': {'atlas_generation': 2}}}
    groups = reference_intervals([row(0, 0), row(1, 0), row(2, 1), row(3, 0)])
    assert [[r['index'] for r in group] for _, group in groups] == [[0, 1], [2], [3]]


def test_body_extrinsic_direction_and_rejected_frame_coverage():
    stamps = np.arange(16, dtype=np.int64) * 10_000_000 + 1_000_000_000
    poses = np.repeat(np.eye(4)[None], 16, axis=0)
    for i, pose in enumerate(poses):
        pose[:3, :3] = Rotation.from_euler('xyz', [i * .1, i * .03, i * .2]).as_matrix()
        pose[:3, 3] = [i * .01, (i % 3) * .02, i * i * .001]
    camera_from_body = np.eye(4)
    camera_from_body[:3, :3] = Rotation.from_euler('xyz', [.2, -.1, .3]).as_matrix()
    camera_from_body[:3, 3] = [.04, -.03, .02]
    rows = []
    for i, (stamp, body) in enumerate(zip(stamps, poses)):
        camera = body @ np.linalg.inv(camera_from_body)
        raw = {'timestamp_ns': int(stamp), 'pose_valid': True, 'tracking_state': 'OK', 'matrix': camera.tolist()}
        result = {**raw, 'pose_valid': i != 3}
        if i == 3:
            result['matrix'] = None
        rows.append({'index': i, 'raw': raw, 'result': result})
    truth = (stamps, poses[:, :3, 3], Rotation.from_matrix(poses[:, :3, :3]))
    native = score_interval(rows, truth, camera_from_body, 'native')
    application = score_interval(rows, truth, camera_from_body, 'application')
    assert native['ate_translation_m']['max'] < 1e-8
    assert application['coverage']['expected_frames'] == 16
    assert application['coverage']['estimate_available'] == 15
    assert application['rpe_by_horizon_s']['0.1']['translation_m']['count'] < native['rpe_by_horizon_s']['0.1']['translation_m']['count']
    wrong = score_interval(rows, truth, np.linalg.inv(camera_from_body), 'native')
    assert wrong['ate_translation_m']['max'] > .001


def test_prediction_motion_error_uses_preloss_anchor_and_preserves_scale():
    registration = np.eye(4)
    registration[:3, :3] = Rotation.from_euler('xyz', [.2, -.4, .7]).as_matrix()
    registration[:3, 3] = [100, -20, 3]
    anchor = np.eye(4)
    anchor[:3, 3] = [2, 3, 4]
    movement = np.eye(4)
    movement[0, 3] = 1
    actual = anchor @ movement
    predicted = registration @ actual
    position, angle = relative_motion_error(registration @ anchor, predicted, anchor, actual)
    assert position < 1e-12 and angle < 1e-10
    wrong = movement.copy()
    wrong[0, 3] = 2
    wrong[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    position, angle = relative_motion_error(registration @ anchor, registration @ anchor @ wrong, anchor, actual)
    assert abs(position - 1) < 1e-12 and abs(angle - 90) < 1e-10
