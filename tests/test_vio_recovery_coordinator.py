import numpy as np
import pytest
from egohand.vio.native_bridge import ImuMeasurement
from egohand.vio.recovery_coordinator import RecoveryCoordinator

def sample(t):return ImuMeasurement(t,(0,0,9.81),(0,0,0))
def result(t,state='OK',epoch=1):
    return {'timestamp_ns':t,'tracking_state':state,'pose_valid':state=='OK',
            'imu_initialized':True,'inertial_scale_finalized':True,'map_epoch':epoch,'matrix':np.eye(4).tolist()}
def body():return {'T_world_from_body':np.eye(4).tolist(),'velocity_world_mps':[0,0,0],
                   'accel_bias_mps2':[0,0,0],'gyro_bias_rps':[0,0,0],'ba2_complete':True}


def finish(coordinator, row, body_state):
    # Simulate the same-frame metadata actually produced by the snapshot bridge.
    row=dict(row)
    if type(row.get('map_epoch')) is int:
        from egohand.vio.map_identity import decode_map_epoch
        row.update(snapshot_available=True,snapshot_consistent=True,frame_id=row['timestamp_ns'],
                   world_frame_id=row['map_epoch'], **decode_map_epoch(row['map_epoch']))
    if body_state is not None:
        body_state=dict(body_state)
        row.update(snapshot_available=True,snapshot_consistent=True,frame_id=row['timestamp_ns'],
                   world_frame_id=row['map_epoch'])
        body_state.update(snapshot_consistent=True,frame_id=row['frame_id'],
                          timestamp_ns=row['timestamp_ns'],world_frame_id=row['map_epoch'])
    return coordinator.finish_frame(row,body_state)


@pytest.mark.parametrize('corruption', ['inconsistent', 'missing', 'mismatch', 'float'])
def test_untrusted_map_transition_cannot_clear_quarantine(corruption):
    c=RecoveryCoordinator(stable_seconds=0)
    initial=1<<40
    finish(c,result(1_000_000_000,epoch=initial),None)
    jump=result(1_010_000_000,epoch=initial);jump['matrix'][0][3]=4
    assert finish(c,jump,None)['await_visual_requalification']
    bogus=result(1_020_000_000,epoch=0)
    bogus.update(snapshot_available=True,snapshot_consistent=True,frame_id=3,
                 world_frame_id=0,map_id=0,reset_id=0,map_revision=0)
    if corruption=='inconsistent':bogus['snapshot_consistent']=False
    elif corruption=='missing':bogus.pop('snapshot_available')
    elif corruption=='mismatch':bogus['map_id']=9
    else:bogus['map_epoch']=0.0
    assert c.finish_frame(bogus,None)['await_visual_requalification']
    assert c.map_epoch==initial
    assert finish(c,result(1_030_000_000,epoch=initial),None)['await_visual_requalification']
    assert not finish(c,result(1_040_000_000,epoch=2<<40),None)['await_visual_requalification']


@pytest.mark.parametrize('prior_anchor', [False, True])
def test_contradictory_identity_cannot_create_or_replace_anchor(prior_anchor):
    c=RecoveryCoordinator(stable_seconds=0)
    if prior_anchor:
        c.begin_frame([sample(1_000_000_000)])
        assert finish(c,result(1_000_000_000,epoch=0),body())['anchor_available']
    row=result(1_010_000_000,epoch=0)
    row.update(snapshot_available=True,snapshot_consistent=True,frame_id=1,
               world_frame_id=0,map_id=9,reset_id=0,map_revision=0)
    b=body();b.update(snapshot_consistent=True,frame_id=1,
                     timestamp_ns=row['timestamp_ns'],world_frame_id=0)
    c.begin_frame([sample(row['timestamp_ns'])])
    status=c.finish_frame(row,b)
    assert not status['body_snapshot_consistent']
    assert (c.predictor.anchor.timestamp_ns if c.predictor.anchor else None)==(
        1_000_000_000 if prior_anchor else None)


def test_known_world_revision_becoming_unknown_discards_anchor():
    c=RecoveryCoordinator(stable_seconds=0)
    first=result(1_000_000_000,epoch=0);first['world_frame_revision']=0
    c.begin_frame([sample(first['timestamp_ns'])]);assert finish(c,first,body())['anchor_available']
    changed=result(1_010_000_000,epoch=0);changed['world_frame_revision']=None
    c.begin_frame([sample(changed['timestamp_ns'])]);state=finish(c,changed,body())
    assert state['await_visual_requalification'] and not state['anchor_available']
    assert state['quarantine_cause']=='world_revision_unavailable'


@pytest.mark.parametrize('revision', [-1, True, 0.0, '0', 1 << 64])
def test_invalid_world_revision_cannot_replace_anchor(revision):
    c=RecoveryCoordinator(stable_seconds=0)
    first=result(1_000_000_000,epoch=0);first['world_frame_revision']=0
    c.begin_frame([sample(first['timestamp_ns'])]);assert finish(c,first,body())['anchor_available']
    changed=result(1_010_000_000,epoch=0);changed['world_frame_revision']=revision
    c.begin_frame([sample(changed['timestamp_ns'])]);state=finish(c,changed,body())
    assert state['await_visual_requalification'] and not state['anchor_available']


def test_legacy_no_anchor_jump_cannot_be_requalified_by_time_alone():
    c=RecoveryCoordinator(stable_seconds=0,quarantine_timeout_s=.02)
    c.begin_frame([sample(1_000_000_000)])
    assert not c.finish_frame(result(1_000_000_000),None)['anchor_available']
    jump=result(1_010_000_000);jump['matrix'][0][3]=4
    c.begin_frame([sample(1_010_000_000)])
    assert c.finish_frame(jump,None)['await_visual_requalification']
    same=result(1_050_000_000);same['matrix'][0][3]=4
    c.begin_frame([sample(1_050_000_000)])
    state=c.finish_frame(same,None)
    assert state['quarantine_timed_out'] and state['await_visual_requalification']
    assert not state['request_new_map']

def test_candidate_is_checked_against_full_imu_interval_before_acceptance():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)])
    assert finish(c,result(1_000_000_000),body())['anchor_available']
    c.begin_frame([sample(1_010_000_000)])
    loss=finish(c,result(1_010_000_000,'LOST'),None)
    assert loss['loss_prediction']['status']=='predicted'
    assert loss['loss_prediction']['mode']=='imu_only_extrapolation_not_visual_pose'
    assert np.allclose(loss['loss_prediction']['T_world_from_body'],np.eye(4))
    c.begin_frame([sample(1_020_000_000)])
    good=c.check_candidate(np.eye(4),1_020_000_000,1)
    bad=np.eye(4);bad[0,3]=4
    rejected=c.check_candidate(bad,1_020_000_000,1)
    assert good['status']=='consistent' and rejected['status']=='inconsistent'
    assert rejected['duration_s']==.02

def test_missing_gap_and_map_change_are_unavailable():
    c=RecoveryCoordinator(stable_seconds=0)
    assert c.check_candidate(np.eye(4),10,1)['reason']=='no_trusted_anchor'
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_050_000_000)])
    assert c.check_candidate(np.eye(4),1_050_000_000,1)['status']=='unavailable'
    finish(c,result(1_050_000_000),body())
    assert c.quarantined
    c.begin_frame([sample(1_060_000_000)]);finish(c,result(1_060_000_000,epoch=1<<20),body())
    assert not c.quarantined

def test_loss_prediction_does_not_bridge_imu_gaps():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_050_000_000)])
    prediction=finish(c,result(1_050_000_000,'LOST'),None)['loss_prediction']
    assert prediction['status']=='unavailable'

def test_initial_tracking_recovery_without_body_does_not_block_first_anchor():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(990_000_000)])
    finish(c,result(990_000_000,'NOT_INITIALIZED'),None)
    c.begin_frame([sample(1_000_000_000)])
    early=finish(c,result(1_000_000_000),None)
    assert not early['await_visual_requalification']
    c.begin_frame([sample(1_010_000_000)])
    status=finish(c,result(1_010_000_000),body())
    assert status['anchor_available'] and not status['await_visual_requalification']

def test_long_loss_visual_candidate_is_not_mislabeled_as_imu_verified():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_000_000_000+i*10_000_000) for i in range(1,211)])
    candidate=c.check_candidate(np.eye(4),3_100_000_000,1)
    assert candidate['status']=='unavailable'
    assert finish(c,result(3_100_000_000),body())['await_visual_requalification']

def test_quarantine_requests_new_map_once_and_resets_after_epoch_change():
    c=RecoveryCoordinator(stable_seconds=0,quarantine_timeout_s=.02,timeout_action='legacy_new_map')
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    jumped=result(1_010_000_000);jumped['matrix'][0][3]=4
    c.begin_frame([sample(1_010_000_000)]);first=finish(c,jumped,body())
    assert first['await_visual_requalification'] and not first['request_new_map']
    c.begin_frame([sample(1_030_000_000)]);second=finish(c,result(1_030_000_000),body())
    assert second['request_new_map'] and second['quarantine_timed_out']
    assert second['timeout_action']=='legacy_new_map'
    c.begin_frame([sample(1_040_000_000)]);assert not finish(c,result(1_040_000_000),body())['request_new_map']
    c.begin_frame([sample(1_050_000_000)]);fresh=finish(c,result(1_050_000_000,epoch=1<<20),body())
    assert not fresh['await_visual_requalification'] and not fresh['request_new_map']

def test_default_timeout_reports_but_never_requests_map_for_missing_imu():
    from egohand.vio.pose_quality import apply_recovery_quarantine
    c=RecoveryCoordinator(stable_seconds=0,quarantine_timeout_s=.02)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_010_000_000)]);finish(c,result(1_010_000_000,'LOST'),None)
    c.begin_frame([sample(1_050_000_000)])  # IMU gap makes recovery evidence unavailable.
    first=finish(c,result(1_050_000_000),body())
    assert first['await_visual_requalification']
    assert first['quarantine_cause']=='direct_recovery_unavailable'
    assert first['evidence_status']=='unavailable' and not first['quarantine_timed_out']
    for stamp in (1_069_999_999,1_070_000_000,1_080_000_000):
        c.begin_frame([sample(stamp)])
        state=finish(c,result(stamp),body())
        assert state['await_visual_requalification'] and not state['request_new_map']
        assert state['quarantine_timed_out']==(stamp>=1_070_000_000)
        assert not apply_recovery_quarantine({**result(stamp),'recovery_preaccept':state})['pose_valid']

def test_inconsistent_candidate_keeps_distinct_evidence_and_remains_quarantined():
    c=RecoveryCoordinator(stable_seconds=0,quarantine_timeout_s=.02)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_010_000_000)])
    bad=np.eye(4);bad[0,3]=4
    assert c.check_candidate(bad,1_010_000_000,1)['status']=='inconsistent'
    first=finish(c,result(1_010_000_000),body())
    assert first['quarantine_cause']=='candidate_inconsistent'
    assert first['evidence_status']=='inconsistent'
    c.begin_frame([sample(1_030_000_000)])
    assert c.check_candidate(np.eye(4),1_030_000_000,1)['status']=='unavailable'
    timed=finish(c,result(1_030_000_000),body())
    assert timed['quarantine_timed_out'] and not timed['request_new_map']
    assert timed['quarantine_cause']=='candidate_inconsistent'
    assert timed['evidence_status']=='inconsistent'

def test_invalid_timeout_action_fails_closed():
    with pytest.raises(ValueError,match='超时策略'):
        RecoveryCoordinator(timeout_action='new_map')

def test_direct_recovery_without_relocalization_does_not_replace_anchor_on_jump():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_010_000_000)]);finish(c,result(1_010_000_000,'RECENTLY_LOST'),None)
    c.begin_frame([sample(1_020_000_000)])
    jumped=body();jumped['T_world_from_body'][0][3]=4
    status=finish(c,result(1_020_000_000),jumped)
    assert status['direct_recovery']['status']=='inconsistent'
    assert status['await_visual_requalification']
    assert c.predictor.anchor.timestamp_ns==1_000_000_000

def test_jump_while_orb_is_ok_remains_quarantined_until_map_change():
    from egohand.vio.pose_quality import apply_recovery_quarantine
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    jumped=result(1_010_000_000);jumped['matrix'][0][3]=84
    b=body();b['T_world_from_body'][0][3]=84
    c.begin_frame([sample(1_010_000_000)]);status=finish(c,jumped,b)
    assert status['await_visual_requalification']
    filtered=apply_recovery_quarantine({**jumped,'recovery_preaccept':status})
    assert not filtered['pose_valid'] and filtered['matrix'] is None
    c.begin_frame([sample(1_020_000_000)])
    later=result(1_020_000_000);later['matrix'][0][3]=84
    assert finish(c,later,b)['await_visual_requalification']
    c.begin_frame([sample(1_030_000_000)])
    assert not finish(c,result(1_030_000_000,epoch=1<<20),body())['await_visual_requalification']

def test_map_revision_jump_is_quarantined_after_old_anchor_is_discarded():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000,epoch=0),body())
    c.begin_frame([sample(1_010_000_000)])
    jump=result(1_010_000_000,epoch=1);jump['matrix'][0][3]=84
    status=finish(c,jump,None)
    assert status['await_visual_requalification'] and not status['anchor_available']
    assert status['quarantine_cause']=='pose_jump' and status['evidence_status']=='unavailable'
    c.begin_frame([sample(1_020_000_000)])
    later=result(1_020_000_000,epoch=1);later['matrix'][0][3]=84
    assert finish(c,later,body())['await_visual_requalification']

def test_real_map_change_does_not_compare_with_old_anchor():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000,epoch=0),body())
    c.begin_frame([sample(1_010_000_000)])
    fresh=result(1_010_000_000,epoch=1<<40);fresh['matrix'][0][3]=84
    status=finish(c,fresh,None)
    assert not status['await_visual_requalification'] and not status['anchor_available']

def test_world_revision_change_does_not_claim_cross_frame_jump():
    c=RecoveryCoordinator(stable_seconds=0)
    first=result(1_000_000_000,epoch=0);first['world_frame_revision']=0
    c.begin_frame([sample(1_000_000_000)]);finish(c,first,body())
    changed=result(1_010_000_000,epoch=1);changed['world_frame_revision']=1
    changed['matrix'][0][3]=84
    c.begin_frame([sample(1_010_000_000)]);status=finish(c,changed,None)
    assert not status['await_visual_requalification'] and not status['anchor_available']


@pytest.mark.parametrize('event', ['map_revision','world_revision','unknown_epoch'])
def test_reference_change_during_loss_cannot_clear_recovery_evidence(event):
    c=RecoveryCoordinator(stable_seconds=0)
    first=result(1_000_000_000,epoch=0);first['world_frame_revision']=0
    c.begin_frame([sample(1_000_000_000)]);finish(c,first,body())
    lost=result(1_010_000_000,'LOST',epoch=0);lost['world_frame_revision']=0
    c.begin_frame([sample(1_010_000_000)]);finish(c,lost,None)
    changed=result(1_020_000_000,epoch=1 if event=='map_revision' else 0)
    changed['world_frame_revision']=1 if event=='world_revision' else 0
    if event=='unknown_epoch':changed['map_epoch']=None
    c.begin_frame([sample(1_020_000_000)]);status=finish(c,changed,None)
    assert status['await_visual_requalification'] and not status['anchor_available']
    assert status['quarantine_cause']==('identity_snapshot_unverified' if event=='unknown_epoch' else 'reference_changed_during_loss')
    assert status['evidence_status']=='unavailable' and not status['request_new_map']


def test_malformed_snapshot_cannot_become_trusted_anchor():
    c=RecoveryCoordinator(stable_seconds=0)
    malformed=body();malformed['velocity_world_mps']=[float('nan'),0,0]
    c.begin_frame([sample(1_000_000_000)])
    status=finish(c,result(1_000_000_000),malformed)
    assert not status['body_snapshot_consistent'] and not status['anchor_available']


@pytest.mark.parametrize('epoch', [1<<40, 1<<20])
def test_explicit_new_map_with_world_revision_opens_separate_segment(epoch):
    c=RecoveryCoordinator(stable_seconds=0)
    first=result(1_000_000_000,epoch=0);first['world_frame_revision']=0
    c.begin_frame([sample(1_000_000_000)]);finish(c,first,body())
    lost=result(1_010_000_000,'LOST',epoch=0);lost['world_frame_revision']=0
    c.begin_frame([sample(1_010_000_000)]);finish(c,lost,None)
    fresh=result(1_020_000_000,epoch=epoch);fresh['world_frame_revision']=1
    c.begin_frame([sample(1_020_000_000)]);status=finish(c,fresh,None)
    assert not status['await_visual_requalification'] and not status['anchor_available']


@pytest.mark.parametrize('callback', [False, True])
def test_legacy_dll_loss_without_snapshot_anchor_cannot_recover_by_waiting(callback):
    from egohand.vio.pose_quality import apply_recovery_quarantine
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)])
    assert not c.finish_frame(result(1_000_000_000),body())['anchor_available']
    c.begin_frame([sample(1_010_000_000)])
    c.finish_frame(result(1_010_000_000,'LOST'),None)
    c.begin_frame([sample(1_020_000_000)])
    if callback:
        assert c.check_candidate(np.eye(4),1_020_000_000,1)['reason']=='no_trusted_anchor'
    recovered=result(1_020_000_000)
    status=c.finish_frame(recovered,body())
    assert status['await_visual_requalification'] and status['evidence_status']=='unavailable'
    assert not apply_recovery_quarantine({**recovered,'recovery_preaccept':status})['pose_valid']
    c.begin_frame([sample(1_030_000_000)])
    assert c.finish_frame(result(1_030_000_000),body())['await_visual_requalification']


def test_imu_consistency_alone_does_not_qualify_direct_recovery():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_010_000_000)]);finish(c,result(1_010_000_000,'LOST'),None)
    c.begin_frame([sample(1_020_000_000)]);status=finish(c,result(1_020_000_000),body())
    assert status['direct_recovery']['status']=='consistent'
    assert status['await_visual_requalification'] and status['anchor_available']
    assert status['quarantine_cause']=='independent_visual_verification_unavailable'
    assert status['evidence_status']=='unavailable'


def test_imu_consistent_callback_does_not_replace_independent_visual_check():
    c=RecoveryCoordinator(stable_seconds=0)
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_010_000_000)]);finish(c,result(1_010_000_000,'LOST'),None)
    c.begin_frame([sample(1_020_000_000)])
    candidate=c.check_candidate(np.eye(4),1_020_000_000,1)
    assert candidate['status']=='consistent' and candidate['visual_requalification_required']
    assert candidate['independent_visual_verification']=='unavailable'
    status=finish(c,result(1_020_000_000),body())
    assert status['await_visual_requalification']
    assert status['quarantine_cause']=='independent_visual_verification_unavailable'


def test_candidate_marks_preaccept_visual_requirement_after_tracking_not_at_startup():
    c=RecoveryCoordinator(stable_seconds=0)
    startup=c.check_candidate(np.eye(4),1_000_000_000,1)
    assert startup['status']=='unavailable' and not startup['visual_requalification_required']
    c.begin_frame([sample(1_000_000_000)]);finish(c,result(1_000_000_000),body())
    c.begin_frame([sample(1_010_000_000)])
    candidate=c.check_candidate(np.eye(4),1_010_000_000,1)
    assert candidate['status']=='consistent' and candidate['visual_requalification_required']
    assert candidate['independent_visual_verification']=='unavailable'

def test_unfinalized_scale_does_not_create_an_imu_anchor():
    """VIBA2 完成前地图尺度未定标，视觉位姿不能作为米制外推起点。"""
    c=RecoveryCoordinator(stable_seconds=0)
    r=result(1_000_000_000);r['inertial_scale_finalized']=False
    c.begin_frame([sample(1_000_000_000)])
    finish(c,r,body())
    assert c.predictor.anchor is None
    assert c.check_candidate(np.eye(4),1_010_000_000,1)['status']=='unavailable'

def test_finalized_scale_allows_an_imu_anchor():
    c=RecoveryCoordinator(stable_seconds=0)
    r=result(1_000_000_000);r['inertial_scale_finalized']=True
    c.begin_frame([sample(1_000_000_000)])
    finish(c,r,body())
    assert c.predictor.anchor is not None
