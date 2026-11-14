import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scripts.compare_native_vio_runs import body, metrics


def test_common_anchor_se3_and_exclude_missing_or_gt_gap_rpe():
    change = np.eye(4)
    change[:3, :3] = Rotation.from_euler('z', 30, degrees=True).as_matrix()
    change[:3, 3] = [5, -3, 1]
    key = ((1, 0, 2, 3, 0), (2, 0, 2, 3, 0))
    items = []
    for index in (1, 2, 4, 5):
        truth = np.eye(4)
        truth[0, 3] = index
        candidate = change @ truth
        items.append((index, key, index * 100, truth, truth.copy(), candidate, index != 5))
    result = metrics(items)
    assert result['paired_frames'] == 4
    assert result['candidate']['ape_translation_m']['max'] < 1e-12
    assert result['candidate']['rpe_translation_m']['count'] == 1
    assert result['candidate']['rpe_translation_m']['rmse'] == pytest.approx(0, abs=1e-12)


def test_rejects_non_rigid_body_and_ignores_unavailable():
    row = {'tracking_state': 'LOST', 'raw_pose_valid': False, 'snapshot': {}}
    assert body(row) is None
    matrix = np.eye(4); matrix[0, 0] = 2
    row = {'tracking_state': 'OK', 'raw_pose_valid': True, 'world_frame_revision': 0,
           'snapshot': {'snapshot_consistent': True, 'frame_id': 1, 'timestamp_ns': 2, 'world_frame_id': 3,
                        'map_id': 0, 'reset_id': 0, 'reference_generation': 0,
                        'reference_frame': {'frame_bound': True, 'generation': 0, 'atlas_generation': 0},
                        'body_state': {'frame_id': 1, 'timestamp_ns': 2, 'world_frame_id': 3,
                                       'T_world_from_body': matrix.tolist()}}}
    with pytest.raises(ValueError, match='invalid body matrix'):
        body(row)
