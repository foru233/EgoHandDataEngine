import numpy as np
from egohand.vio.imu_loss_prediction import BodyState
from egohand.vio.native_bridge import ImuMeasurement
from egohand.vio.recovery_monitor import RecoveryMonitor, observe_recovery


def sample(t):return ImuMeasurement(t,(0,0,9.81),(0,0,0))
def result(t,state='OK',x=0.,epoch=1):
    T=np.eye(4);T[0,3]=x
    from egohand.vio.map_identity import decode_map_epoch
    return {'timestamp_ns':t,'pose_valid':state=='OK','tracking_state':state,'map_epoch':epoch,
            'snapshot_available':True,'snapshot_consistent':True,'frame_id':t,'world_frame_id':epoch,
            **decode_map_epoch(epoch),
            'body_state_candidate':{'T_world_from_body':T.tolist(),'snapshot_consistent':True,
                'frame_id':t,'timestamp_ns':t,'world_frame_id':epoch,'velocity_world_mps':[0,0,0],
                'accel_bias_mps2':[0,0,0],'gyro_bias_rps':[0,0,0]}}
def anchor(t,epoch=1):return BodyState(t,np.eye(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),str(epoch))


def test_bad_snapshot_identity_cannot_clear_quarantine_or_replace_epoch():
    old=1<<40;m=RecoveryMonitor()
    m.observe(result(1_000_000_000,epoch=old),[])
    m.await_visual_requalification=True
    bogus=result(1_010_000_000,epoch=0);bogus['snapshot_consistent']=False
    assert m.observe(bogus,[])['reason']=='identity_snapshot_unverified'
    assert m.await_visual_requalification and m._last_epoch==old
    m.observe(result(1_020_000_000,epoch=old),[])
    assert m.await_visual_requalification
    m.observe(result(1_030_000_000,epoch=2<<40),[])
    assert not m.await_visual_requalification


def test_known_world_revision_becoming_unknown_cannot_replace_anchor():
    m=RecoveryMonitor();first=result(1_000_000_000);first['world_frame_revision']=0
    m.observe(first,[sample(first['timestamp_ns'])],trusted_body=anchor(first['timestamp_ns']))
    changed=result(1_010_000_000);changed['world_frame_revision']=None
    decision=m.observe(changed,[sample(changed['timestamp_ns'])],trusted_body=anchor(changed['timestamp_ns']))
    assert decision['reason']=='world_revision_unavailable'
    assert m.await_visual_requalification and m.predictor.anchor is None


def test_complete_loss_interval_and_endpoint_hold_are_auditable():
    m=RecoveryMonitor()
    m.observe(result(1_000_000_000),[sample(999_000_000)],trusted_body=anchor(1_000_000_000))
    m.observe(result(1_020_000_000,'RECENTLY_LOST'),[sample(1_010_000_000),sample(1_020_000_000)])
    decision=m.observe(result(1_041_000_000),[sample(1_030_000_000),sample(1_040_000_000)])
    assert decision['status']=='consistent' and decision['start_hold_ns']==1_000_000 and decision['endpoint_hold_ns']==1_000_000
    assert decision['mode']=='advisory_after_orb_acceptance'


def test_bad_recovery_cannot_automatically_replace_anchor():
    m=RecoveryMonitor();a=anchor(1_000_000_000)
    m.observe(result(a.timestamp_ns),[sample(a.timestamp_ns)],trusted_body=a)
    m.observe(result(1_010_000_000,'LOST'),[sample(1_010_000_000)])
    decision=m.observe(result(1_020_000_000,x=4),[sample(1_020_000_000)],trusted_body=anchor(1_020_000_000))
    assert decision['status']=='inconsistent' and m.await_visual_requalification
    assert m.predictor.anchor.timestamp_ns==a.timestamp_ns
    m.observe(result(1_030_000_000),[sample(1_030_000_000)],trusted_body=anchor(1_030_000_000))
    assert m.predictor.anchor.timestamp_ns==a.timestamp_ns
    assert m.requalify_after_independent_visual_check() is False
    m.observe(result(1_040_000_000),[sample(1_040_000_000)],trusted_body=anchor(1_040_000_000))
    assert m.predictor.anchor.timestamp_ns==a.timestamp_ns
    new_epoch=1<<40
    m.observe(result(1_050_000_000,epoch=new_epoch),[sample(1_050_000_000)],
              trusted_body=anchor(1_050_000_000,new_epoch))
    assert m.predictor.anchor.timestamp_ns==1_050_000_000 and not m.await_visual_requalification


def test_real_adapter_requires_ba2_and_uses_orb_body_coordinates():
    m=RecoveryMonitor();r=result(1_000_000_000)
    r.update(snapshot_available=True,snapshot_consistent=True,frame_id=7,world_frame_id=1,
             inertial_scale_finalized=True)
    r['body_state_candidate'].update(snapshot_consistent=True,frame_id=7,timestamp_ns=r['timestamp_ns'],world_frame_id=1)
    r['body_state_candidate'].update(ba2_complete=False,velocity_world_mps=[0,0,0],accel_bias_mps2=[0,0,0],gyro_bias_rps=[0,0,0])
    observe_recovery(m,r,[(1_000_000_000,0,0,9.81,0,0,0)])
    assert m.predictor.anchor is None
    r['body_state_candidate']['ba2_complete']=True
    observe_recovery(m,r,[])
    assert m.predictor.anchor is not None and m.predictor.anchor.map_id=='1'


def test_recovery_is_checked_before_display_stabilization():
    m=RecoveryMonitor();a=anchor(1_000_000_000)
    m.observe(result(a.timestamp_ns),[sample(a.timestamp_ns)],trusted_body=a)
    m.observe(result(1_010_000_000,'LOST'),[sample(1_010_000_000)])
    r=result(1_020_000_000);r.update(pose_valid=False,raw_pose_valid=True)
    decision=observe_recovery(m,r,[(1_020_000_000,0,0,9.81,0,0,0)])
    assert decision['status']=='consistent'
    assert m.predictor.anchor.timestamp_ns==a.timestamp_ns


def test_recovery_gap_and_new_map_never_claim_consistency():
    m=RecoveryMonitor();a=anchor(1_000_000_000)
    m.observe(result(a.timestamp_ns),[sample(a.timestamp_ns)],trusted_body=a)
    m.observe(result(1_010_000_000,'LOST'),[sample(1_010_000_000)])
    decision=m.observe(result(1_060_000_000),[sample(1_060_000_000)])
    assert decision['status']=='unavailable' and m.await_visual_requalification
    new_epoch=1<<40
    m.observe(result(1_070_000_000,epoch=new_epoch),[sample(1_070_000_000)],trusted_body=anchor(1_070_000_000,new_epoch))
    m.observe(result(1_080_000_000,'LOST',epoch=new_epoch),[sample(1_080_000_000)])
    switched=m.observe(result(1_090_000_000,epoch=2<<40),[sample(1_090_000_000)])
    assert switched['status']=='unavailable' and switched['reason']=='map_changed'


def test_internal_revision_cannot_clear_recovery_quarantine():
    m=RecoveryMonitor();a=anchor(1_000_000_000)
    m.observe(result(a.timestamp_ns),[sample(a.timestamp_ns)],trusted_body=a)
    m.observe(result(1_010_000_000,'LOST'),[sample(1_010_000_000)])
    assert m.observe(result(1_020_000_000,x=4),[sample(1_020_000_000)])['status']=='inconsistent'
    decision=m.observe(result(1_030_000_000,epoch=2),[sample(1_030_000_000)],
                       trusted_body=anchor(1_030_000_000,2))
    assert decision['reason']=='reference_revision_unverified'
    assert m.await_visual_requalification and m.predictor.anchor is None
    m.observe(result(1_040_000_000,epoch=2),[sample(1_040_000_000)],
              trusted_body=anchor(1_040_000_000,2))
    assert m.await_visual_requalification and m.predictor.anchor is None


def test_world_revision_invalidates_anchor_and_does_not_clear_quarantine():
    m=RecoveryMonitor();a=anchor(1_000_000_000)
    first=result(a.timestamp_ns);first['world_frame_revision']=0
    m.observe(first,[sample(a.timestamp_ns)],trusted_body=a)
    m.observe({**result(1_010_000_000,'LOST'),'world_frame_revision':0},[sample(1_010_000_000)])
    m.observe({**result(1_020_000_000,x=4),'world_frame_revision':0},[sample(1_020_000_000)])
    decision=m.observe({**result(1_030_000_000),'world_frame_revision':1},[sample(1_030_000_000)],
                       trusted_body=anchor(1_030_000_000))
    assert decision['reason']=='reference_revision_unverified'
    assert m.await_visual_requalification and m.predictor.anchor is None


def test_wrong_body_snapshot_during_recovery_is_unavailable_and_quarantined():
    m=RecoveryMonitor();a=anchor(1_000_000_000)
    m.observe(result(a.timestamp_ns),[sample(a.timestamp_ns)],trusted_body=a)
    m.observe(result(1_010_000_000,'LOST'),[sample(1_010_000_000)])
    candidate=result(1_020_000_000)
    candidate['body_state_candidate']['timestamp_ns']=1_000_000_000
    decision=m.observe(candidate,[sample(1_020_000_000)])
    assert decision['status']=='unavailable'
    assert decision['reason']=='body_snapshot_unavailable_or_mismatched'
    assert m.await_visual_requalification and m.predictor.anchor.timestamp_ns==a.timestamp_ns


def test_reference_change_during_loss_cannot_replace_trusted_anchor():
    m=RecoveryMonitor();a=anchor(1_000_000_000)
    m.observe(result(a.timestamp_ns),[sample(a.timestamp_ns)],trusted_body=a)
    m.observe(result(1_010_000_000,'LOST'),[sample(1_010_000_000)])
    candidate=result(1_020_000_000,epoch=2)
    decision=m.observe(candidate,[sample(1_020_000_000)],trusted_body=anchor(1_020_000_000,2))
    assert decision['status']=='unavailable' and decision['reason']=='reference_changed_during_loss'
    assert m.await_visual_requalification and m.predictor.anchor is None
    m.observe(result(1_030_000_000,epoch=2),[sample(1_030_000_000)],trusted_body=anchor(1_030_000_000,2))
    assert m.predictor.anchor is None and m.await_visual_requalification
