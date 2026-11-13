"""A real reference change opens a qualification window, never clears quarantine."""
import copy
import numpy as np
import pytest
from egohand.vio.map_identity import invalid_reference_frame
from egohand.vio.pose_quality import PoseQualityGate
from egohand.vio.hybrid_quality import HybridPoseQualityGate
from egohand.vio.recovery_coordinator import RecoveryCoordinator
from egohand.vio.recovery_contract import application_feedback, validate_feedback, validate_report, RecoveryProtocolError
from egohand.vio.localization_state import LocalizationPipeline, consumer_map_pose
from egohand.vio.hand_world import conservative_world_reference


def frame(t, generation=0, *, turn=False, revision=0, state='OK'):
    matrix=np.eye(4)
    if turn:matrix[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
    stamp=round(t*1e9)
    row={'timestamp_ns':stamp,'frame_id':stamp,'map_id':0,'reset_id':0,'map_revision':revision,
         'map_epoch':revision,'world_frame_id':revision,'world_frame_revision':0,'map_change_index':revision,
         'snapshot_available':True,'snapshot_consistent':True,'matrix':matrix.tolist(),
         'pose_valid':state=='OK','tracking_state':state,'imu_initialized':generation>0,
         'ba2_complete':generation>0,'inertial_scale_finalized':generation>0,'reference_generation':generation}
    row['reference_frame']={k:row[k] for k in ('frame_id','map_id','reset_id','map_revision','map_change_index','timestamp_ns')}
    row['reference_frame'].update(schema_version=1,generation=generation,operation_sequence=generation,
        atlas_generation=0,geometry_sequence=2*generation,frame_bound=True,snapshot_consistent=True,
        basis='successful_initializer_commit',coordinate_continuity_verified=False)
    return row


def test_boundary_starts_new_window_without_transporting_pose():
    gate=HybridPoseQualityGate(stable_seconds=1)
    assert not gate.check(frame(1))['pose_valid']
    old=gate.check(frame(2));assert old['pose_valid']
    boundary=frame(2.03,1,turn=True,revision=3)
    waiting=gate.check(boundary)
    assert not waiting['pose_valid'] and not waiting['jump_detected']
    assert waiting['raw_matrix']==boundary['matrix']
    assert waiting['quality_gate']['reason_code']=='stable_window_pending'
    new=gate.check(frame(3.04,1,turn=True,revision=3))
    assert new['pose_valid'] and new['pose_source']=='vio' and new['display_segment']>old['display_segment']
    assert new['matrix']==boundary['matrix']
    # A real jump inside the new reference is still rejected.
    jumped=frame(3.05,1,turn=True,revision=3);jumped['matrix'][0][3]=10
    assert gate.check(jumped)['jump_detected']


def test_same_operation_ring_or_ba_flags_cannot_hide_jump():
    gate=PoseQualityGate(stable_seconds=0,require_imu=False)
    gate.check(frame(1))
    jump=frame(1.03,0,turn=True,revision=3)
    jump['imu_initialized']=True;jump['ba2_complete']=True
    jump['map_event_frame']={'events':[{'type':1,'rotation':[0]*9}]}
    assert gate.check(jump)['jump_detected']


@pytest.mark.parametrize('damage',['wrong_frame','missing','unbound','backwards','wrong_generation','wrong_operation'])
def test_unbound_missing_or_regressing_reference_rejected(damage):
    gate=PoseQualityGate(stable_seconds=0,require_imu=False)
    assert gate.check(frame(1,1,revision=3))['pose_valid']
    row=frame(1.03,2,turn=True,revision=6)
    if damage=='wrong_frame':row['reference_frame']['frame_id']+=1
    elif damage=='missing':row.pop('reference_generation');row.pop('reference_frame')
    elif damage=='unbound':row['reference_frame']['frame_bound']=False
    elif damage=='backwards':row=frame(1.03,0,turn=True,revision=6)
    elif damage=='wrong_generation':row['reference_frame']['generation']=1
    else:row['reference_frame']['operation_sequence']=0
    rejected=gate.check(row)
    assert not rejected['pose_valid'] and rejected['quality_gate']['reason_code']=='reference_frame_unverified'


@pytest.mark.parametrize('prior_cause',['jump','loss'])
def test_reference_boundary_cannot_clear_existing_recovery(prior_cause):
    coordinator=RecoveryCoordinator(application_admission=True)
    def finish(row,jump=False):
        coordinator.begin_frame([]);native=coordinator.finish_frame(row,None)
        gated={**row,'mode':'hybrid','pose_source':'visual_fallback','quality_gate':{'passed':not jump},
               'jump_detected':jump,'pose_valid':False,'recovery_preaccept':native}
        return coordinator.finish_application_frame(application_feedback(gated))
    finish(frame(1))
    if prior_cause=='jump':assert finish(frame(1.03),True)['await_visual_requalification']
    else:finish(frame(1.03,state='LOST'))
    assert finish(frame(1.06,1,turn=True,revision=3))['await_visual_requalification']
    assert coordinator.predictor.anchor is None


def test_optional_reference_identity_is_bound_in_both_ipc_directions():
    c=RecoveryCoordinator(application_admission=True);row=frame(1)
    native=c.finish_frame(row,None)
    assert native['reference_generation']==0
    damaged=dict(native,reference_generation=1)
    with pytest.raises(RecoveryProtocolError):validate_report(damaged,row,stage='native')
    feedback=application_feedback({**row,'mode':'hybrid','quality_gate':{'passed':False},'pose_valid':False,'recovery_preaccept':native})
    feedback['reference_generation']=1
    with pytest.raises(RecoveryProtocolError):validate_feedback(feedback,native)
    c.finish_application_frame({**feedback,'reference_generation':0})


def test_pipeline_publishes_new_reference_without_continuity_or_odom():
    c=RecoveryCoordinator(application_admission=True)
    def commit(row):return c.finish_application_frame(application_feedback(row))
    pipeline=LocalizationPipeline('hybrid',HybridPoseQualityGate(stable_seconds=0),recovery_commit=commit)
    def run(row):
        c.begin_frame([]);row['recovery_preaccept']=c.finish_frame(row,None)
        return pipeline.process(row,[])
    old=run(frame(1));new=run(frame(1.03,1,turn=True,revision=3))
    assert new['pose_valid'] and new['localization']['reference_generation']==1
    assert new['map_revision_decision']['event']=='inertial_reference_changed'
    assert new['localization']['coordinate_continuity_verified'] is False
    assert new['localization']['odom']['status']=='unavailable'
    assert new['localization']['admission']['cross_frame_merge_permitted'] is False
    assert consumer_map_pose(new) is not None
    bad=copy.deepcopy(new);bad['localization']['reference_generation']=0
    assert consumer_map_pose(bad) is None
    reference=conservative_world_reference(new)
    assert reference['reference_generation']==1 and reference['cross_frame_merge_permitted'] is False


def test_reference_generation_resets_only_with_distinct_map_identity():
    old=frame(1,2,revision=6);row=frame(2,0)
    assert invalid_reference_frame(2,row,old['map_epoch'])
    row.update(map_id=1,map_epoch=1<<40,world_frame_id=1<<40)
    row['reference_frame']['map_id']=1
    assert not invalid_reference_frame(2,row,old['map_epoch'])
