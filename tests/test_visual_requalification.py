"""Geometry experiments only: a consistent result never releases quarantine."""
import numpy as np
import pytest

from egohand.vio.visual_requalification import ExperimentalLimits, evaluate_stereo_map_geometry


@pytest.fixture
def sample():
    rng = np.random.default_rng(9823)
    points = np.column_stack((rng.uniform(-1.8, 1.8, 80), rng.uniform(-1.2, 1.2, 80),
                              rng.uniform(2.5, 5.0, 80)))
    K = np.array([[400., 0., 400.], [0., 400., 300.], [0., 0., 1.]])
    left = np.column_stack((400*points[:, 0]/points[:, 2]+400, 400*points[:, 1]/points[:, 2]+300))
    right = left.copy(); right[:, 0] -= 400*.065/points[:, 2]
    identity = {'map_id': 1, 'reset_id': 2, 'map_revision': 3, 'world_frame_revision': 4}
    return dict(map_identity=identity.copy(), frame_identity=identity.copy(), snapshot_consistent=True,
                timestamp_ns=100, point_ids=list(range(len(points))), map_points=points,
                left_pixels=left, right_pixels=right, camera_matrix=K, baseline_m=.065,
                image_size=(800, 600), candidate_T_map_from_camera=np.eye(4),
                limits=ExperimentalLimits(20, 15, .7, 2., 1., .05, 2., 4, .05))


def check(sample, **overrides):
    return evaluate_stereo_map_geometry(**{**sample, **overrides})


def test_consistent_geometry_is_advisory_not_release(sample):
    result = check(sample)
    assert result['status'] == 'geometry_consistent'
    assert result['production_release_authorized'] is False
    assert result['independence_verified'] is False
    assert result['snapshot_atomicity_verified'] is False
    assert result['coordinate_continuity_verified'] is False
    assert result['metrics']['timestamp_ns'] == 100
    assert result['metrics']['map_identity'] == sample['map_identity']


@pytest.mark.parametrize('field', ['map_id', 'reset_id', 'map_revision', 'world_frame_revision'])
def test_changed_map_generation_or_revision_cannot_pass(sample, field):
    identity = {**sample['frame_identity'], field: sample['frame_identity'][field] + 1}
    assert check(sample, frame_identity=identity)['reason'] == 'identity_mismatch'


def test_unverified_snapshot_and_missing_identity_never_pass(sample):
    assert check(sample, snapshot_consistent=False)['status'] == 'unavailable'
    assert check(sample, map_identity=None)['status'] == 'unavailable'


def test_wrong_candidate_pose_rejected_even_when_visual_matches_fit(sample):
    candidate = np.eye(4); candidate[0, 3] = .2
    assert check(sample, candidate_T_map_from_camera=candidate)['reason'] == 'candidate_pose_inconsistent'


def test_wrong_candidate_rotation_rejected(sample):
    angle = np.deg2rad(10)
    candidate = np.eye(4)
    candidate[:3, :3] = [[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
    assert check(sample, candidate_T_map_from_camera=candidate)['reason'] == 'candidate_pose_inconsistent'


def test_stereo_mismatch_rejected_despite_valid_left_pnp(sample):
    right = sample['right_pixels'].copy(); right[:, 0] += 5
    assert check(sample, right_pixels=right)['reason'] == 'stereo_reprojection_inconsistent'


def test_duplicate_map_points_nonfinite_and_out_of_image_rejected(sample):
    ids = sample['point_ids'].copy(); ids[-1] = ids[0]
    assert check(sample, point_ids=ids)['status'] == 'unavailable'
    points = sample['map_points'].copy(); points[0, 0] = np.nan
    assert check(sample, map_points=points)['status'] == 'unavailable'
    pixels = sample['left_pixels'].copy(); pixels[0, 0] = 800
    assert check(sample, left_pixels=pixels)['status'] == 'unavailable'


def test_planar_or_low_spread_geometry_does_not_authorize(sample):
    points = sample['map_points'].copy(); points[:, 2] = 3.0
    left = np.column_stack((400*points[:, 0]/3+400, 400*points[:, 1]/3+300))
    right = left.copy(); right[:, 0] -= 400*.065/3
    assert check(sample, map_points=points, left_pixels=left, right_pixels=right)['reason'] == 'degenerate_map_geometry'


def test_outliers_below_required_ratio_rejected(sample):
    left = sample['left_pixels'].copy()
    left[:40] = left[:40][::-1]
    assert check(sample, left_pixels=left)['status'] == 'rejected'


def test_experimental_limits_cannot_be_silent_or_invalid():
    with pytest.raises(ValueError):
        ExperimentalLimits(20, 15, .7, 2., 1., .05, 2., 4, 0.)


def test_research_geometry_cannot_clear_production_quarantine(sample):
    from egohand.vio.recovery_coordinator import RecoveryCoordinator
    from egohand.vio.recovery_monitor import RecoveryMonitor
    from egohand.vio.pose_quality import apply_recovery_quarantine

    coordinator = RecoveryCoordinator(stable_seconds=0)
    initial = {'timestamp_ns': 1_000_000_000, 'tracking_state': 'OK', 'pose_valid': True,
               'imu_initialized': True, 'inertial_scale_finalized': True,
               'map_epoch': 1, 'matrix': np.eye(4).tolist()}
    coordinator.begin_frame([])
    coordinator.finish_frame(initial, None)
    jump = {**initial, 'timestamp_ns': 1_010_000_000, 'matrix': np.eye(4).tolist()}
    jump['matrix'][0][3] = 4
    coordinator.begin_frame([])
    status = coordinator.finish_frame(jump, None)
    assert status['await_visual_requalification']
    assert check(sample)['status'] == 'geometry_consistent'
    assert coordinator.quarantined
    assert apply_recovery_quarantine({**jump, 'recovery_preaccept': status})['pose_valid'] is False
    monitor = RecoveryMonitor()
    monitor.await_visual_requalification = True
    assert monitor.requalify_after_independent_visual_check() is False
    assert monitor.await_visual_requalification


@pytest.mark.parametrize('frozen', [False,True])
@pytest.mark.parametrize('field', ['reference_generation','atlas_generation'])
def test_geometry_known_reference_and_atlas_are_required_on_both_sides(sample, frozen, field):
    expected = {**sample['map_identity'],field:1}
    actual = {**expected, 'map_revision':4 if frozen else 3}
    assert check(sample,map_identity=expected,frame_identity=actual,
                 allow_frozen_content=frozen)['status'] == 'geometry_consistent'
    assert check(sample,map_identity=expected,frame_identity={**actual,field:2},
                 allow_frozen_content=frozen)['reason'] == 'identity_mismatch'
    missing = {k:v for k,v in actual.items() if k != field}
    assert check(sample,map_identity=expected,frame_identity=missing,
                 allow_frozen_content=frozen)['reason'] == 'identity_mismatch'
    assert check(sample,map_identity=missing,frame_identity=expected,
                 allow_frozen_content=frozen)['reason'] == 'identity_mismatch'
    for invalid in (True,None,-1,2**64):
        assert check(sample,map_identity=expected,frame_identity={**actual,field:invalid},
                     allow_frozen_content=frozen)['reason'] == 'identity_missing_or_invalid'
