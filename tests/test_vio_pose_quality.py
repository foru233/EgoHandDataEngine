import numpy as np
import pytest
from egohand.vio.pose_quality import PoseQualityGate


def test_advisory_recovery_quarantine_also_blocks_gui_pose():
    from egohand.vio.pose_quality import apply_recovery_quarantine
    raw=np.eye(4).tolist()
    result=apply_recovery_quarantine({'pose_valid':True,'matrix':raw,'raw_matrix':raw,
        'recovery_monitor_awaiting_visual_requalification':True})
    assert not result['pose_valid'] and result['matrix'] is None
    assert result['raw_matrix']==raw


def pose(t, x=0., *, initialized=True, state='OK', epoch=0):
    matrix=np.eye(4);matrix[0,3]=x
    return {'timestamp_ns':t,'tracking_state':state,'imu_initialized':initialized,
            'pose_valid':True,'matrix':matrix.tolist(),'map_epoch':epoch}


@pytest.mark.parametrize('mutation', [
    lambda m: m.__setitem__((0, 0), -1.0),  # reflection
    lambda m: m.__setitem__((0, 0), 2.0),   # non-orthogonal
    lambda m: m.__setitem__((3, 0), 0.5),   # invalid homogeneous row
    lambda m: m.__setitem__((0, 3), np.nan),
])
def test_invalid_rigid_pose_is_rejected_even_on_first_frame(mutation):
    gate=PoseQualityGate(stable_seconds=0)
    row=pose(1_000_000_000)
    matrix=np.asarray(row['matrix']);mutation(matrix);row['matrix']=matrix.tolist()
    rejected=gate.check(row)
    assert rejected['raw_pose_valid'] and not rejected['pose_valid']
    assert rejected['matrix'] is None and rejected['quality_status']=='位姿数据无效'
    assert gate.previous is None
    assert gate.check(pose(1_033_333_333))['pose_valid']


def test_malformed_pose_is_rejected_instead_of_raising():
    gate=PoseQualityGate(stable_seconds=0)
    row=pose(1_000_000_000);row['matrix']=[[1], ['bad']]
    assert not gate.check(row)['pose_valid']


def test_inconsistent_native_snapshot_cannot_authorize_output():
    gate=PoseQualityGate(stable_seconds=0)
    row=pose(1_000_000_000);row['snapshot_consistent']=False
    rejected=gate.check(row)
    assert rejected['raw_pose_valid'] and not rejected['pose_valid']
    assert rejected['matrix'] is None and rejected['quality_status']=='原生快照不一致'
    assert gate.previous is None
    good=pose(1_033_333_333);good['snapshot_consistent']=True
    assert gate.check(good)['pose_valid']


@pytest.mark.parametrize('revision', [-1, True, 0.0, '0', 1 << 64])
def test_malformed_world_revision_cannot_authorize_first_pose(revision):
    gate=PoseQualityGate(stable_seconds=0)
    row=pose(1_000_000_000);row['world_frame_revision']=revision
    checked=gate.check(row)
    assert checked['raw_pose_valid'] and not checked['pose_valid']
    assert checked['matrix'] is None and checked['quality_status']=='世界坐标版本无效'


def test_snapshot_with_contradictory_identity_cannot_authorize_pose():
    gate=PoseQualityGate(stable_seconds=0)
    row=pose(1_000_000_000,epoch=0)
    row.update(snapshot_available=True,snapshot_consistent=True,frame_id=1,
               world_frame_id=0,map_id=9,reset_id=0,map_revision=0)
    checked=gate.check(row)
    assert checked['raw_pose_valid'] and not checked['pose_valid']
    assert checked['quality_status']=='原生快照地图身份不一致'


def test_static_stabilizes_jump_is_rejected_and_recovery_is_separate():
    gate=PoseQualityGate(stable_seconds=1.0)
    assert gate.check(pose(1_000_000_000, initialized=False))['quality_status']=='等待 IMU 初始化'
    assert not gate.check(pose(2_000_000_000))['pose_valid']
    assert gate.check(pose(3_000_000_000,.001))['pose_valid']
    jump=gate.check(pose(3_100_000_000,1.0))
    assert not jump['pose_valid'] and jump['raw_pose_valid'] and jump['raw_matrix'] is not None
    assert '突变' in jump['quality_status'] and jump['display_segment']==1
    assert not gate.check(pose(3_200_000_000,1.0))['pose_valid']
    stable=gate.check(pose(4_200_000_000,1.001))
    assert stable['pose_valid'] and stable['display_segment']==1
    lost=gate.check(pose(4_300_000_000,1.001,state='LOST'))
    assert not lost['pose_valid'] and lost['display_segment']==2


def test_map_change_and_time_reversal_invalidate_history():
    gate=PoseQualityGate(stable_seconds=0)
    assert gate.check(pose(1_000_000_000))['pose_valid']
    changed=gate.check(pose(2_000_000_000,10.,epoch=1 << 40))
    assert changed['pose_valid'] and changed['display_segment']==1
    reverse=gate.check(pose(1_500_000_000,10.,epoch=1 << 40))
    assert not reverse['pose_valid'] and '时间' in reverse['quality_status']


def test_live_same_map_resume_requires_two_consistent_frames():
    gate = PoseQualityGate(stable_seconds=1.5, resume_same_map=True)
    assert not gate.check(pose(1_000_000_000))['pose_valid']
    assert gate.check(pose(2_500_000_000))['pose_valid']
    assert not gate.check(pose(2_600_000_000, state='RECENTLY_LOST'))['pose_valid']
    assert not gate.check(pose(2_700_000_000))['pose_valid']
    resumed = gate.check(pose(2_800_000_000, .01))
    assert resumed['pose_valid'] and resumed['display_segment'] == 1
    assert not gate.check(pose(2_900_000_000, 10.))['pose_valid']
    assert not gate.check(pose(3_000_000_000, 10.))['pose_valid']
    assert gate.check(pose(3_100_000_000, 10.01))['pose_valid']
    assert not gate.check(pose(3_200_000_000, 10., epoch=1))['pose_valid']
    assert not gate.check(pose(3_300_000_000, 10., epoch=1))['pose_valid']


def test_recovered_map_pose_is_not_reanchored_to_last_position():
    from egohand.vio.pose_continuity import PoseContinuityCorrector
    gate=PoseQualityGate(stable_seconds=1.0,resume_same_map=False)
    continuity=PoseContinuityCorrector(enabled=False)
    gate.check(pose(1_000_000_000));first=continuity.process(gate.check(pose(2_000_000_000)))
    assert first['pose_valid']
    lost=continuity.process(gate.check(pose(2_100_000_000,state='LOST')))
    assert not lost['pose_valid']
    assert not gate.check(pose(4_000_000_000,2.0))['pose_valid']
    recovered=continuity.process(gate.check(pose(5_000_000_000,2.0)))
    assert recovered['pose_valid'] and recovered['display_segment']!=first['display_segment']
    assert recovered['matrix'][0][3]==2.0 and recovered['display_map_coordinates_preserved']


def revision_pose(t, x=0., *, epoch=0, world=0, state='OK', evidence=None):
    result=pose(t, x, epoch=epoch, state=state)
    result.update(world_frame_revision=world, snapshot_consistent=True)
    if evidence is not None:
        result['map_revision_continuity']=evidence
    return result


def verified_revision(*, before=0, after=1, old_t=2_500_000_000,
                      new_t=2_533_000_000, world=0):
    # A test double for the contract, not a claim that the current bridge
    # supplies or independently validates this evidence.
    return {'schema':'map_revision_continuity_v1', 'source':'native_geometry_verifier',
            'complete':True, 'coordinates_continuous':True, 'geometry_consistent':True,
            'from_map_epoch':before, 'to_map_epoch':after,
            'from_timestamp_ns':old_t, 'to_timestamp_ns':new_t,
            'world_frame_revision':world}


def qualified_gate(**options):
    gate=PoseQualityGate(stable_seconds=1.5, **options)
    assert not gate.check(revision_pose(1_000_000_000))['pose_valid']
    assert gate.check(revision_pose(2_500_000_000))['pose_valid']
    return gate


def test_untrusted_continuity_claim_cannot_keep_stability():
    gate=qualified_gate()
    result=gate.check(revision_pose(2_533_000_000, .003, epoch=1,
                                    evidence=verified_revision()))
    assert not result['pose_valid'] and result['display_segment']==1
    assert result['raw_matrix'][0][3]==.003


@pytest.mark.parametrize('mutation', [
    lambda e: None,
    lambda e: e.update(complete=False),
    lambda e: e.update(geometry_consistent=False),
    lambda e: e.update(coordinates_continuous=False),
    lambda e: e.update(from_timestamp_ns=1),
    lambda e: e.update(to_map_epoch=2),
    lambda e: e.update(world_frame_revision=1),
    lambda e: e.update(source='unverified'),
])
def test_unverified_revision_restarts_stability(mutation):
    gate=qualified_gate(resume_same_map=True)
    evidence=verified_revision()
    missing=mutation(evidence)
    if missing is None and evidence==verified_revision():
        evidence=None
    result=gate.check(revision_pose(2_533_000_000, .003, epoch=1, evidence=evidence))
    assert not result['pose_valid'] and not result['jump_detected']
    assert result['display_segment']==1
    assert not gate.check(revision_pose(2_566_000_000, .004, epoch=1))['pose_valid']
    assert gate.check(revision_pose(4_033_000_000, .004, epoch=1))['pose_valid']


@pytest.mark.parametrize('provide_evidence', [False, True])
def test_revision_does_not_hide_large_jump(provide_evidence):
    gate=qualified_gate(resume_same_map=True)
    evidence=verified_revision() if provide_evidence else None
    result=gate.check(revision_pose(2_533_000_000, 1., epoch=1, evidence=evidence))
    assert result['jump_detected'] and not result['pose_valid']
    assert result['translation_step_m']==1.
    assert not gate.check(revision_pose(2_566_000_000, 1., epoch=1))['pose_valid']
    assert not gate.check(revision_pose(2_599_000_000, 1., epoch=1))['pose_valid']


@pytest.mark.parametrize('epoch,world', [(1 << 40, 0), (1 << 20, 0), (1, 1)])
def test_map_reset_and_world_change_do_not_preserve_stability(epoch, world):
    gate=qualified_gate()
    changed=gate.check(revision_pose(2_533_000_000, .003, epoch=epoch, world=world,
                                    evidence=verified_revision(after=epoch, world=world)))
    assert not changed['pose_valid'] and changed['display_segment']==1


def test_tracking_loss_at_revision_prevents_resume_and_evidence_reuse():
    gate=qualified_gate(resume_same_map=True)
    lost=gate.check(revision_pose(2_533_000_000, .003, epoch=1, state='LOST',
                                  evidence=verified_revision()))
    assert not lost['pose_valid']
    assert not gate.check(revision_pose(2_566_000_000, .004, epoch=1))['pose_valid']
    assert not gate.check(revision_pose(2_599_000_000, .005, epoch=1))['pose_valid']


def test_revision_time_reversal_cannot_hide_behind_revision_evidence():
    gate=qualified_gate()
    result=gate.check(revision_pose(2_400_000_000, epoch=1,
        evidence=verified_revision(new_t=2_400_000_000)))
    assert not result['pose_valid'] and '时间' in result['quality_status']


@pytest.mark.parametrize('which', ['previous', 'current', 'unknown_world'])
def test_revision_requires_consistent_snapshots_and_known_world(which):
    gate=PoseQualityGate(stable_seconds=1.5)
    first=revision_pose(1_000_000_000)
    previous=revision_pose(2_500_000_000)
    current=revision_pose(2_533_000_000, .003, epoch=1, evidence=verified_revision())
    if which=='previous': previous['snapshot_consistent']=False
    if which=='current': current['snapshot_consistent']=False
    if which=='unknown_world':
        first.pop('world_frame_revision');previous.pop('world_frame_revision')
        current.pop('world_frame_revision')
    gate.check(first);gate.check(previous)
    result=gate.check(current)
    assert not result['pose_valid'] and result['display_segment']==1


def test_rotation_jump_on_revision_is_rejected_with_evidence():
    gate=qualified_gate()
    result=revision_pose(2_533_000_000, epoch=1, evidence=verified_revision())
    result['matrix'][0][0]=-1.;result['matrix'][1][1]=-1.
    checked=gate.check(result)
    assert checked['jump_detected'] and not checked['pose_valid']
    assert checked['rotation_step_deg']==180.
