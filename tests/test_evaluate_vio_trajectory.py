import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scripts.evaluate_vio_trajectory import evaluate, load_pose, load_timeline


def sequence(scale=1.0, missing=()):
    times = np.arange(8, dtype=np.int64) * 100_000_000
    points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                       [0, 1, 1], [1, 1, 1], [1, 0, 1], [0, 0, 1]], dtype=float)
    rotation = Rotation.from_euler('z', 35, degrees=True)
    estimates = rotation.apply(points * scale) + [2, 4, 3]
    good = np.array([i not in missing for i in range(len(times))])
    return times, (times[good], estimates[good], Rotation.from_quat(np.tile(rotation.as_quat(), (good.sum(), 1)))), (
        times, points, Rotation.identity(len(times)))


def test_rigid_alignment_keeps_missing_frames_and_recovers_transform():
    timeline, est, gt = sequence(missing=(3,))
    result = evaluate(timeline, est, gt, 1_000_000, 150_000_000, [0.1])
    assert result['coverage']['estimate_available'] == 7
    assert result['coverage']['paired_fraction'] == pytest.approx(7 / 8)
    assert result['ate_translation_m']['max'] < 1e-12
    assert result['rpe_by_horizon_s']['0.1']['translation_m']['count'] == 5
    assert result['alignment']['scale_applied'] == 1


def test_no_scale_fit_hides_metric_scale_error():
    timeline, est, gt = sequence(scale=2)
    result = evaluate(timeline, est, gt, 1_000_000, 150_000_000, [0.1])
    assert result['ate_translation_m']['rmse'] > 0.4
    assert result['rpe_by_horizon_s']['0.1']['sum_relative_distance_ratio_minus_one'] == pytest.approx(1)


def test_gt_gap_not_interpolated_or_bridged():
    timeline, est, gt = sequence()
    keep = np.array([0, 1, 2, 5, 6, 7])
    gt = (gt[0][keep], gt[1][keep], gt[2][keep])
    result = evaluate(timeline, est, gt, 1_000_000, 150_000_000, [0.1])
    assert result['coverage']['gt_available'] == 6
    assert result['rpe_by_horizon_s']['0.1']['translation_m']['count'] == 4


def test_rpe_handles_jitter_and_checks_gap_between_exact_endpoints():
    timeline, est, gt = sequence()
    timeline = timeline + np.array([0, 100_000, -100_000, 200_000, 0, -200_000, 100_000, 0])
    est = (timeline, est[1], est[2])
    gt = (timeline, gt[1], gt[2])
    # GT samples exist at each image, but their intervals exceed the GT limit.
    result = evaluate(timeline, est, gt, 1_000_000, 50_000_000, [0.1], 150_000_000)
    assert result['coverage']['paired'] == 8
    assert result['rpe_by_horizon_s']['0.1']['translation_m'] is None
    result = evaluate(timeline, est, gt, 1_000_000, 150_000_000, [0.1], 150_000_000)
    assert result['rpe_by_horizon_s']['0.1']['translation_m']['count'] == 7


def test_rejects_ambiguous_estimate_matching_and_bad_timestamps(tmp_path):
    timeline, est, gt = sequence()
    with pytest.raises(ValueError, match='multiple camera frames'):
        evaluate(timeline, (est[0][::2], est[1][::2], est[2][::2]),
                 gt, 110_000_000, 150_000_000, [0.1])
    f = tmp_path / 'timestamps.txt'
    f.write_text('1\n1\n')
    with pytest.raises(ValueError, match='strictly increasing'):
        load_timeline(f, 'ns')


def test_pose_rejects_invalid_quaternion(tmp_path):
    f = tmp_path / 'pose.txt'
    f.write_text('1 0 0 0 0 0 0 0\n2 0 0 0 0 0 0 1\n')
    with pytest.raises(ValueError, match='quaternion'):
        load_pose(f, 'ns')
