import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from audit_public_gate_gt import associate_truth, labels


def test_GT_gap_never_interpolated_and_exact_endpoints_preserved():
    times = np.array([0, 10_000_000, 40_000_000])
    poses = np.repeat(np.eye(4)[None], 3, axis=0)
    poses[1, 0, 3] = 1
    assert associate_truth(5_000_000, times, poses)[0, 3] == .5
    assert associate_truth(25_000_000, times, poses) is None
    np.testing.assert_array_equal(associate_truth(40_000_000, times, poses), poses[2])


def test_fixed_anchor_labels_rejection_error_and_reference_without_scale_fit():
    rows = []
    for i in range(5):
        pose = np.eye(4)
        pose[0, 3] = i * .01
        value = {'timestamp_ns': i * 50_000_000, 'tracking_state': 'OK', 'pose_valid': True,
                 'map_id': 0, 'reset_id': 0, 'map_revision': 1, 'reference_generation': 1,
                 'world_frame_revision': 0, 'reference_frame': {'atlas_generation': 2}, 'matrix': pose.tolist()}
        rows.append({'index': i, 'raw': value, 'result': dict(value)})
    rows[1]['result']['pose_valid'] = False
    rows[2]['raw']['matrix'][0][3] += 1
    rows[3]['raw']['map_id'] = 1
    rows[4]['raw']['tracking_state'] = 'LOST'
    def truth(t):
        pose = np.eye(4)
        pose[0, 3] = t / 50_000_000 * .01
        return pose
    samples = labels(rows, [1, 2], truth, np.eye(4))
    assert [s['classification'] for s in samples] == ['potential_false_rejection', 'outside_envelope_admitted', 'unassessable', 'unassessable']
    assert samples[2]['reason'] == 'different_reference_no_stitching'
    assert samples[3]['reason'] == 'native_visual_pose_unavailable'
