"""Recovery must regain availability through geometry or a distinct segment."""
import copy
import numpy as np
import pytest
from egohand.vio.recovery_coordinator import RecoveryCoordinator
from egohand.vio.recovery_contract import application_feedback
from egohand.vio.localization_state import LocalizationPipeline,consumer_map_pose
from egohand.vio.hybrid_quality import make_quality_gate
from egohand.vio.native_bridge import ImuMeasurement
from egohand.vio.short_recovery import geometry_matches_frame,visual_reference


def frame(t,*,state='OK',x=0,reset=0,geometry=False):
    T=np.eye(4);T[0,3]=x
    epoch=reset<<20
    r=dict(timestamp_ns=t,frame_id=t,map_id=0,reset_id=reset,map_revision=0,
        map_epoch=epoch,world_frame_id=epoch,world_frame_revision=0,map_change_index=0,
        reference_generation=1,snapshot_available=True,snapshot_consistent=True,
        tracking_state=state,pose_valid=state=='OK',matrix=T.tolist() if state=='OK' else None,
        imu_initialized=True,ba1_complete=True,ba2_complete=False,inertial_scale_finalized=False)
    ref={k:r[k] for k in ('timestamp_ns','frame_id','map_id','reset_id','map_revision','map_change_index')}
    ref.update(schema_version=1,generation=1,operation_sequence=1,atlas_generation=2+reset,
        geometry_sequence=2,frame_bound=True,snapshot_consistent=True,
        basis='successful_initializer_commit',coordinate_continuity_verified=False)
    r['reference_frame']=ref
    if state=='OK':
        r['body_state_candidate']=dict(timestamp_ns=t,frame_id=t,world_frame_id=epoch,
            snapshot_consistent=True,ba2_complete=False,T_world_from_body=T.tolist(),
            velocity_world_mps=[0,0,0],accel_bias_mps2=[0,0,0],gyro_bias_rps=[0,0,0])
    if geometry:
        r['recovery_observation']=dict(schema_version=1,source='native_stereo_map_geometry_v1',
            enabled=True,available=True,qualified=True,pose_binding_verified=True,
            frame_id=t,timestamp_ns=t,map_id=0,reset_id=reset,map_revision=0,
            atlas_generation=2+reset,reference_generation=1,geometry_sequence=2,
            geometric_inliers=80,stereo_inliers=50,occupied_cells=8,associated=100,
            stereo_p95_normalized=1.2,phase='short_imu_search',T_world_from_body=T.reshape(-1).tolist())
    return r


def system(*,stable=0,short=True,timeout=.03,anchor=True,bootstrap=False):
    c=RecoveryCoordinator(application_admission=True,short_recovery=short,
        allow_initialized_anchor=anchor,timeout_action='independent_new_map',
        quarantine_timeout_s=timeout)
    p=LocalizationPipeline('stereo_inertial',make_quality_gate('stereo_inertial',stable_seconds=stable,
        allow_verified_stereo_bootstrap=bootstrap),
        recovery_commit=lambda out:c.finish_application_frame(application_feedback(out)))
    def run(r):
        c.begin_frame([ImuMeasurement(r['timestamp_ns'],(0,0,9.81),(0,0,0))])
        row=copy.deepcopy(r);row['recovery_preaccept']=c.finish_frame(row,row.get('body_state_candidate'))
        return p.process(row,[])
    return c,p,run


def test_initialized_pre_ba2_anchor_is_prediction_only_and_not_a_release():
    c,p,run=system()
    assert run(frame(1_000_000_000))['pose_valid']
    assert c.predictor.anchor is not None
    lost=run(frame(1_010_000_000,state='RECENTLY_LOST'))
    assert not lost['pose_valid'] and lost['matrix'] is None
    assert lost['recovery_preaccept']['loss_prediction']['status']=='predicted'
    recovered=run(frame(1_020_000_000))
    assert c.quarantined and not recovered['pose_valid']


def test_real_geometry_contract_releases_loss_without_waiting_for_ba2():
    c,p,run=system()
    run(frame(1_000_000_000));run(frame(1_010_000_000,state='RECENTLY_LOST'))
    recovered=run(frame(1_020_000_000,geometry=True))
    assert recovered['pose_valid'] and not c.quarantined
    assert recovered['recovery_preaccept']['visual_release']['kind']=='same_reference_stereo_map_geometry'
    assert consumer_map_pose(recovered) is not None
    assert recovered['ba2_complete'] is False


@pytest.mark.parametrize('key,value',[
    ('frame_id',1),('timestamp_ns',1),('reset_id',1),('map_revision',1),
    ('atlas_generation',9),('reference_generation',2),('geometry_sequence',9),
    ('geometric_inliers',49),('stereo_inliers',29),('occupied_cells',5),
    ('stereo_p95_normalized',3.1),('stereo_p95_normalized',float('nan')),
    ('pose_binding_verified',False),('qualified',False),('source','other'),('associated',200)])
def test_stale_wrong_reference_or_insufficient_geometry_never_releases(key,value):
    c,p,run=system();run(frame(1_000_000_000));run(frame(1_010_000_000,state='LOST'))
    r=frame(1_020_000_000,geometry=True);r['recovery_observation'][key]=value
    assert not run(r)['pose_valid'] and c.quarantined


def test_geometric_release_still_obeys_original_quality_stability_window():
    c,p,run=system(stable=.02)
    run(frame(1_000_000_000));assert run(frame(1_020_000_000))['pose_valid']
    run(frame(1_030_000_000,state='LOST'))
    first=run(frame(1_040_000_000,geometry=True))
    assert not c.quarantined and not first['pose_valid']
    assert first['quality_gate']['reason_code']=='stable_window_pending'
    assert run(frame(1_060_000_000,geometry=True))['pose_valid']


def test_geometry_release_after_stable_quarantine_reports_imu_unavailable():
    c,p,run=system(stable=.1,anchor=False,timeout=5)
    assert not run(frame(1_000_000_000))['pose_valid']
    assert run(frame(1_100_000_000))['pose_valid']
    run(frame(1_110_000_000,state='LOST'))
    pending=run(frame(1_120_000_000))
    assert c.quarantined and not pending['pose_valid']
    stable=run(frame(1_220_000_000))
    assert stable['quality_gate']['passed'] is True
    assert c.quarantined and not stable['pose_valid']
    assert consumer_map_pose(stable,world_hands=True) is None
    released=run(frame(1_230_000_000,geometry=True))
    assert released['recovery_preaccept']['visual_release']['motion_check']['status']=='unavailable'
    assert released['quality_gate']['passed'] is True
    assert not c.quarantined and released['pose_valid']
    assert consumer_map_pose(released,world_hands=True) is not None


def test_timeout_requests_new_map_once_and_never_releases_old_coordinates():
    c,p,run=system()
    old=run(frame(1_000_000_000));old_segment=old['local_motion']['segment_id']
    jumped=run(frame(1_010_000_000,x=4,geometry=True))
    assert jumped['jump_detected'] and c.quarantined
    run(frame(1_020_000_000,x=4,geometry=True))
    timed=run(frame(1_050_000_000,x=4,geometry=True))
    assert timed['recovery_preaccept']['request_new_map'] and not timed['pose_valid']
    assert not run(frame(1_060_000_000,x=4,geometry=True))['recovery_preaccept']['request_new_map']
    fresh=run(frame(1_070_000_000,x=20,reset=1))
    assert fresh['pose_valid'] and not c.quarantined
    assert fresh['local_motion']['segment_id']!=old_segment
    assert fresh['local_motion']['T_session_from_segment'] is None
    assert fresh['local_motion']['cross_segment_merge_permitted'] is False


def test_identity_failure_timeout_requests_new_map_and_reset_clears_old_quarantine():
    c,p,run=system(timeout=.03)
    run(frame(1_000_000_000));run(frame(1_010_000_000,state='LOST'))
    first=frame(1_020_000_000,state='LOST')
    first['reference_generation']=2;first['reference_frame']['generation']=2
    assert not run(first)['recovery_preaccept']['request_new_map']
    late=frame(1_050_000_000,state='LOST')
    late['reference_generation']=2;late['reference_frame']['generation']=2
    timed=run(late)
    assert timed['recovery_preaccept']['quarantine_timed_out']
    assert timed['recovery_preaccept']['request_new_map']
    c.reset_for_new_map()
    assert not c.quarantined and c.predictor.anchor is None and c.map_epoch is None


def test_geometry_cannot_release_changed_world_reference_or_disabled_policy():
    c,p,run=system(short=False)
    run(frame(1_000_000_000));run(frame(1_010_000_000,state='LOST'))
    assert not run(frame(1_020_000_000,geometry=True))['pose_valid']
    r=frame(1_000_000_000,geometry=True);reference=visual_reference(r)
    r['world_frame_revision']=True
    assert not geometry_matches_frame(r,r['recovery_observation'],reference)


def test_inconsistent_imu_residual_blocks_even_qualified_geometry():
    c,p,run=system()
    run(frame(1_000_000_000));run(frame(1_010_000_000,state='LOST'))
    r=frame(1_020_000_000,x=10,geometry=True)
    c.begin_frame([ImuMeasurement(r['timestamp_ns'],(0,0,9.81),(0,0,0))])
    candidate=c.check_candidate(np.array(r['body_state_candidate']['T_world_from_body']),
        r['timestamp_ns'],r['map_epoch'],r['recovery_observation'])
    assert candidate['status']=='inconsistent'
    r['recovery_preaccept']=c.finish_frame(r,r['body_state_candidate'])
    assert not p.process(r,[])['pose_valid']


def visual_frame(t,**options):
    r=frame(t,geometry=True,**options)
    r['imu_initialized']=False;r['body_state_candidate']=None
    return r


def test_verified_bootstrap_is_visual_and_still_waits_for_stability():
    c,p,run=system(bootstrap=True,stable=.02)
    first=run(visual_frame(1_000_000_000))
    assert not first['pose_valid'] and first['quality_gate']['reason_code']=='stable_window_pending'
    ready=run(visual_frame(1_020_000_000))
    assert ready['pose_valid'] and ready['pose_source']=='stereo_bootstrap'
    assert ready['inertial_display_ready'] is False
    assert ready['localization']['state']=='VISUAL_TRACKING'
    assert consumer_map_pose(ready) is not None and c.predictor.anchor is None


@pytest.mark.parametrize('fault',['default_off','no_geometry','stale_geometry','unbound_pose','quarantine'])
def test_bootstrap_requires_geometry_and_cannot_clear_quarantine(fault):
    c,p,run=system(bootstrap=fault!='default_off')
    if fault=='quarantine':
        run(frame(990_000_000));run(frame(1_000_000_000,x=4))
        assert c.quarantined
    r=visual_frame(1_010_000_000,x=4 if fault=='quarantine' else 0)
    if fault=='no_geometry':r.pop('recovery_observation')
    if fault=='stale_geometry':r['recovery_observation']['frame_id']=1
    if fault=='unbound_pose':r['recovery_observation']['pose_binding_verified']=False
    out=run(r)
    assert not out['pose_valid'] and consumer_map_pose(out) is None
    assert c.predictor.anchor is None or fault=='quarantine'
    if fault=='quarantine':assert c.quarantined


def test_bootstrap_new_map_is_independent_and_initialization_starts_another_segment():
    c,p,run=system(bootstrap=True)
    old=run(frame(1_000_000_000));run(frame(1_010_000_000,state='LOST'))
    fresh=run(visual_frame(1_020_000_000,reset=1,x=20))
    assert fresh['pose_valid'] and not c.quarantined
    assert fresh['local_motion']['segment_id']!=old['local_motion']['segment_id']
    assert fresh['local_motion']['T_session_from_segment'] is None
    initialized=frame(1_030_000_000,reset=1,x=20)
    initialized['reference_generation']=2;initialized['reference_frame']['generation']=2
    ready=run(initialized)
    assert ready['pose_valid'] and ready['pose_source']=='stereo_inertial'
    assert ready['local_motion']['segment_id']!=fresh['local_motion']['segment_id']
    assert ready['local_motion']['cross_segment_merge_permitted'] is False


def test_bootstrap_jump_remains_rejected_with_full_geometry():
    c,p,run=system(bootstrap=True)
    run(visual_frame(1_000_000_000))
    jumped=run(visual_frame(1_010_000_000,x=4))
    assert jumped['jump_detected'] and not jumped['pose_valid'] and c.quarantined


def test_quarantined_prediction_is_separate_bounded_and_cannot_locate_hands():
    c,p,run=system(timeout=5)
    run(frame(1_000_000_000));run(frame(1_010_000_000,x=4))
    lost=run(frame(1_020_000_000,state='LOST'))
    prediction=lost['recovery_preaccept']['loss_prediction']
    assert prediction['status']=='predicted' and prediction['reference_quarantined']
    assert prediction['mode']=='imu_only_extrapolation_not_visual_pose'
    assert not lost['pose_valid'] and consumer_map_pose(lost,world_hands=True) is None
    late=run(frame(3_020_000_000,state='LOST'))
    assert late['recovery_preaccept']['loss_prediction']['status']=='unavailable'
    assert not late['pose_valid'] and c.quarantined


def test_jump_during_existing_loss_quarantine_cannot_be_forgotten_by_geometry():
    c,p,run=system(timeout=5)
    run(frame(1_000_000_000));run(frame(1_010_000_000,state='LOST'))
    run(frame(1_020_000_000))
    assert c.quarantined and c.quarantine_cause!='pose_jump'
    jumped=run(frame(1_030_000_000,x=4))
    assert jumped['jump_detected'] and c.quarantine_cause=='pose_jump'
    recovered=run(frame(1_040_000_000,geometry=True))
    assert c.quarantined and not recovered['pose_valid']
    assert recovered['recovery_preaccept']['visual_release'] is None
    assert consumer_map_pose(recovered,world_hands=True) is None


def test_unadmitted_startup_jump_restarts_stability_without_abandoning_new_map():
    c,p,run=system(bootstrap=True,stable=.02)
    assert not run(visual_frame(1_000_000_000))['pose_valid']
    rejected=run(visual_frame(1_010_000_000,x=4))
    assert rejected['jump_detected'] and not rejected['pose_valid']
    assert consumer_map_pose(rejected) is None
    assert not c.quarantined and c.trusted_visual_identity is None and c.predictor.anchor is None
    pending=run(visual_frame(1_020_000_000,x=4))
    assert not pending['pose_valid'] and pending['quality_gate']['reason_code']=='stable_window_pending'
    ready=run(visual_frame(1_040_000_000,x=4))
    assert ready['pose_valid'] and ready['pose_source']=='stereo_bootstrap'
    assert not ready['recovery_preaccept']['request_new_map']
    # Once admitted, the very same step requires the full quarantine path.
    jumped=run(visual_frame(1_050_000_000,x=8))
    assert jumped['jump_detected'] and not jumped['pose_valid'] and c.quarantined
    assert c.quarantine_cause=='pose_jump'


def test_startup_step_exception_does_not_clear_existing_loss_quarantine():
    c,p,run=system(stable=.1)
    run(frame(1_000_000_000));run(frame(1_010_000_000,state='LOST'))
    run(frame(1_020_000_000))
    assert c.quarantined and c.trusted_visual_identity is None
    jumped=run(frame(1_030_000_000,x=4))
    assert jumped['jump_detected'] and not jumped['pose_valid']
    assert c.quarantined and c.quarantine_cause=='pose_jump'
    assert not run(frame(1_040_000_000,x=4,geometry=True))['pose_valid']


def test_startup_step_legacy_policy_is_unchanged():
    c,p,run=system(short=False,stable=.1)
    run(frame(1_000_000_000))
    jumped=run(frame(1_010_000_000,x=4))
    assert jumped['jump_detected'] and not jumped['pose_valid'] and c.quarantined


def test_prediction_expiry_retires_anchor_but_valid_geometry_can_recover():
    c,p,run=system(timeout=5)
    assert run(frame(1_000_000_000))['pose_valid']
    for t in range(1_010_000_000,3_040_000_000,10_000_000):
        lost=run(frame(t,state='RECENTLY_LOST'))
        assert not lost['pose_valid'] and consumer_map_pose(lost,world_hands=True) is None
    assert c.predictor.anchor is None and c.invalid_reason is None
    assert lost['recovery_preaccept']['loss_prediction']['status']=='unavailable'
    assert not run(frame(3_040_000_000))['pose_valid']
    recovered=run(frame(3_050_000_000,geometry=True))
    release=recovered['recovery_preaccept']['visual_release']
    assert release['motion_check']['status']=='unavailable'
    assert recovered['pose_valid'] and not c.quarantined
    assert c.predictor.anchor.timestamp_ns==3_050_000_000


def test_prediction_expiry_legacy_policy_preserves_fault_and_anchor():
    c,p,run=system(short=False,timeout=5)
    run(frame(1_000_000_000))
    for t in range(1_010_000_000,3_040_000_000,10_000_000):
        run(frame(t,state='LOST'))
    assert c.predictor.anchor is not None
    assert c.invalid_reason=='预测区间超过上限'
    recovered=run(frame(3_050_000_000,geometry=True))
    assert not recovered['pose_valid'] and c.quarantined
    assert consumer_map_pose(recovered,world_hands=True) is None


@pytest.mark.parametrize('fault',['gap','non_monotonic','identity','pose_jump','stale_geometry'])
def test_expiry_fix_never_bypasses_data_identity_or_jump_failures(fault):
    c,p,run=system(timeout=5)
    run(frame(1_000_000_000))
    for t in range(1_010_000_000,3_040_000_000,10_000_000):run(frame(t,state='LOST'))
    if fault=='gap':
        # A new inertial anchor must first be established before its gap can
        # be evaluated; expiry itself must never synthesize an IMU interval.
        run(frame(3_040_000_000,geometry=True))
        run(frame(3_080_000_000,state='LOST'))
    elif fault=='non_monotonic':
        c.begin_frame([ImuMeasurement(3_020_000_000,(0,0,9.81),(0,0,0))])
    elif fault=='pose_jump':
        run(frame(3_040_000_000,x=4))
    r=frame(3_100_000_000,geometry=True)
    if fault=='identity':
        r['reference_frame']['frame_id']=0
    elif fault=='stale_geometry':
        r['recovery_observation']['frame_id']=0
    result=run(r)
    assert not result['pose_valid'] and c.quarantined
    assert consumer_map_pose(result,world_hands=True) is None


@pytest.mark.parametrize('fault', ['gap', 'non_monotonic'])
def test_prediction_expiry_does_not_erase_preexisting_sensor_failure(fault):
    c,p,run=system(timeout=5)
    run(frame(1_000_000_000))
    run(frame(1_010_000_000,state='LOST'))
    if fault=='non_monotonic':
        c.begin_frame([ImuMeasurement(1_005_000_000,(0,0,9.81),(0,0,0))])
    # The 40 ms hole is invalid under the existing 20 ms rule.
    run(frame(1_050_000_000,state='LOST'))
    assert c.invalid_reason is not None
    for t in range(1_060_000_000,3_050_000_000,10_000_000):
        run(frame(t,state='LOST'))
    assert c.predictor.anchor is None and c.invalid_reason is not None
    recovered=run(frame(3_050_000_000,geometry=True))
    assert not recovered['pose_valid'] and c.quarantined
    assert recovered['recovery_preaccept']['visual_release'] is None


def test_expired_prediction_geometry_still_requires_full_application_stability():
    c,p,run=system(stable=.1,timeout=5)
    for t in range(1_000_000_000,1_120_000_000,10_000_000):
        ready=run(frame(t))
    assert ready['pose_valid']
    for t in range(1_120_000_000,3_150_000_000,10_000_000):
        run(frame(t,state='LOST'))
    recovered=run(frame(3_150_000_000,geometry=True))
    assert not c.quarantined and not recovered['pose_valid']
    assert recovered['quality_gate']['reason_code']=='stable_window_pending'
    for t in range(3_160_000_000,3_270_000_000,10_000_000):
        ready=run(frame(t,geometry=True))
    assert ready['pose_valid']
