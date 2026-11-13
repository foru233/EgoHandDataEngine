"""Analytic controls for the diagnostic camera/body rotation convention."""
import importlib.util
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('rotation_audit', Path(__file__).parents[1] / 'scripts/audit_vio_rotation_phase.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_constant_gyro_and_nontrivial_camera_extrinsic_match_analytic_rotation():
    camera_from_body = Rotation.from_euler('xyz', [20, 35, 70], degrees=True).as_matrix()
    omega = np.array([.5, -.25, .75])
    delta = Rotation.from_rotvec(omega * .1).as_matrix()
    samples = [(20_000_000, 120_000_000, camera_from_body.T, delta @ camera_from_body.T)]
    times = np.arange(0, 200_000_001, 1_000_000)
    gyro = np.tile(omega, (len(times), 1))
    result, _ = audit.evaluate(samples, times, gyro, camera_from_body)
    assert result['max_deg'] < 1e-9
    bias = np.array([.02, -.01, .03])
    corrected, _ = audit.evaluate(samples, times, gyro + bias, camera_from_body, bias)
    assert corrected['max_deg'] < 1e-9
    wrong_axes, _ = audit.evaluate(samples, times, gyro, np.eye(3))
    assert wrong_axes['max_deg'] > 1


def test_raw_pair_selection_keeps_gated_invalid_and_excludes_map_boundary():
    def row(stamp, epoch):
        return {'timestamp_ns': stamp, 'map_epoch': epoch, 'raw_pose_valid': True,
                'pose_valid': False, 'raw_matrix': np.eye(4).tolist(), 'tracking_state': 'OK',
                'snapshot_consistent': True, 'tracked_inliers': 50}
    rows = [row(0, 1), row(33_333_333, 1), row(66_666_666, 2)]
    selected = audit.pairs(rows, 1)
    assert len(selected) == 1
    assert selected[0][:2] == (0, 33_333_333)
