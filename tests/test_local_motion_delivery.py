"""Exercise admitted localization -> segment math -> persisted hand/camera output."""
import csv
import json

import numpy as np
import pytest

from egohand.core.types import Hand3D, StereoHandResult
from egohand.storage.exporter import export_all,load_json
from egohand.vio.hybrid_quality import make_quality_gate
from egohand.vio.localization_state import LocalizationPipeline,consumer_map_pose
from egohand.vio.local_segments import consumer_segment_pose,segment_hands_for_result
from egohand.vio.recovery_contract import application_feedback
from egohand.vio.recovery_coordinator import RecoveryCoordinator


class Stream:
    def __init__(self):
        self.coordinator=RecoveryCoordinator(application_admission=True)
        self.pipeline=LocalizationPipeline('stereo',make_quality_gate('stereo',stable_seconds=0),
            recovery_commit=lambda r:self.coordinator.finish_application_frame(application_feedback(r)))
        self.frame=0

    def send(self,transform,*,epoch=0,tracking='OK',world=0,source=None):
        self.frame+=1
        row={'type':'pose','frame_id':self.frame,'timestamp_ns':1_000_000_000+self.frame*50_000_000,
            'map_epoch':epoch,'map_id':epoch>>40,'reset_id':(epoch>>20)&((1<<20)-1),
            'map_revision':epoch&((1<<20)-1),'world_frame_id':epoch,
            'world_frame_revision':world,'snapshot_available':True,'snapshot_consistent':True,
            'tracking_state':tracking,'pose_valid':tracking=='OK','matrix':transform.tolist(),
            'imu_initialized':False,'ba1_complete':False,'ba2_complete':False}
        if source is not None:row['pose_source']=source
        self.coordinator.begin_frame([])
        row['recovery_preaccept']=self.coordinator.finish_frame(row,None)
        return self.pipeline.process(row,[])


def test_native_frame_gap_and_source_change_start_independent_segments():
    stream=Stream()
    first=stream.send(np.eye(4),source='visual_fallback')
    stream.frame+=1
    gap=stream.send(np.eye(4),source='visual_fallback')
    changed=stream.send(np.eye(4),source='vio')
    assert gap['local_motion']['segment_start_reason']=='frame_sequence_gap'
    assert [p['local_motion']['segment_index'] for p in (first,gap,changed)]==[0,1,2]
    assert changed['local_motion']['segment_start_reason']=='reported_reference_or_source_changed'
    assert changed['local_motion']['T_session_from_camera'] is None


def test_unknown_or_invalid_world_revision_never_exports_local_transform():
    from egohand.vio.recovery_contract import RecoveryProtocolError
    stream=Stream();stream.send(np.eye(4))
    pose=stream.send(np.eye(4),world=None)
    assert pose['local_motion']['pose_valid'] is False
    assert pose['local_motion']['T_segment_from_camera'] is None
    assert consumer_segment_pose(pose) is None
    # Malformed known identities are rejected earlier by the transaction owner.
    for revision in (-1,True):
        with pytest.raises(RecoveryProtocolError):Stream().send(np.eye(4),world=revision)


def test_local_hand_consumer_preserves_permission_and_exact_frame_join():
    from copy import deepcopy
    from egohand.vio.local_segments import consumer_display_pose
    pose=Stream().send(np.eye(4))
    denied=deepcopy(pose)
    denied['localization']['admission']['world_hands_permitted']=False
    assert consumer_display_pose(denied) is not None
    assert consumer_display_pose(denied,world_hands=True) is None
    result=StereoHandResult(1,pose['timestamp_ns'],'cal','vendor_vs030')
    stale={'status':'available','pose_timestamp_ns':pose['timestamp_ns']-1,'join':{}}
    assert segment_hands_for_result(result,pose,stale)['status']=='unavailable'


def test_segment_origin_and_rotated_camera_point_roundtrip(tmp_path):
    stream=Stream()
    origin=np.array([[0.,-1,0,2],[1,0,0,3],[0,0,1,1],[0,0,0,1]])
    first=stream.send(origin)
    relative=np.eye(4);relative[0,3]=.02
    pose=stream.send(origin@relative)
    np.testing.assert_allclose(consumer_segment_pose(first),np.eye(4),atol=1e-12)
    np.testing.assert_allclose(consumer_segment_pose(pose),relative,atol=1e-12)
    assert pose['localization']['admission']['cross_frame_merge_permitted'] is False
    assert pose['localization']['odom']['status']=='unavailable'
    hand=Hand3D('Left',1.,4);hand.joints[0].valid=True
    hand.joints[0].xyz_m=np.array([.1,.2,1]);hand.valid_joint_count=1
    result=StereoHandResult(2,pose['timestamp_ns'],'cal','vendor_vs030',hands=[hand],session_id='hands')
    result.local_motion=pose['local_motion']
    world={'status':'available','join':{'source':'recorded_localization'},'pose_timestamp_ns':pose['timestamp_ns']}
    result.segment_hands=segment_hands_for_result(result,pose,world)
    np.testing.assert_allclose(result.segment_hands['hands'][0]['xyz_segment_m'][0],[.12,.2,1])
    path=export_all([result],tmp_path/'bundle')
    loaded=load_json(path/'results.json')[0]
    assert loaded.local_motion==result.local_motion
    np.testing.assert_allclose(loaded.segment_hands['hands'][0]['xyz_segment_m'][0],[.12,.2,1])
    with np.load(path/'segment_hands.npz',allow_pickle=False) as a:
        np.testing.assert_allclose(a['xyz_segment_m'][0,0,0],[.12,.2,1])
        assert a['valid'].sum()==1 and np.isnan(a['xyz_segment_m'][0,0,1]).all()
        assert a['segment_id'][0]==pose['local_motion']['segment_id']
    with np.load(path/'camera_trajectory.npz',allow_pickle=False) as a:
        np.testing.assert_allclose(a['T_segment_from_camera'][0],relative,atol=1e-12)
    with (path/'segment_hands.csv').open(encoding='utf-8-sig',newline='') as f:
        rows=list(csv.DictReader(f))
    np.testing.assert_allclose(float(rows[0]['x_m']),.12,atol=1e-12)
    assert rows[1]['x_m']==''
    assert json.loads((path/'local_coordinate_policy.json').read_text())['cross_segment_merge_permitted'] is False


def test_reference_change_starts_unrelated_segment_and_quarantine_stays_closed():
    stream=Stream();T=np.eye(4)
    before=stream.send(T)
    changed=stream.send(T,epoch=1)
    assert consumer_segment_pose(changed) is not None
    assert changed['local_motion']['segment_id']!=before['local_motion']['segment_id']
    assert changed['local_motion']['T_session_from_camera'] is None
    assert changed['local_motion']['T_session_from_segment'] is None
    lost=stream.send(T,epoch=1,tracking='LOST')
    assert consumer_segment_pose(lost) is None
    recovered=stream.send(T,epoch=1)
    assert consumer_map_pose(recovered) is None
    assert recovered['local_motion']['pose_valid'] is False


def test_segment_consumer_rejects_stale_or_corrupt_transform():
    import copy
    pose=Stream().send(np.eye(4))
    assert consumer_segment_pose(pose,expected_timestamp_ns=pose['timestamp_ns']+1) is None
    damaged=copy.deepcopy(pose);damaged['local_motion']['T_segment_from_camera'][0][3]=1
    assert consumer_segment_pose(damaged) is None
    damaged=copy.deepcopy(pose);damaged['local_motion']['world_frame_revision']=4
    assert consumer_segment_pose(damaged) is None


def test_join_rejection_preserves_camera_hand_and_suppresses_segment_hand():
    pose=Stream().send(np.eye(4));hand=Hand3D('Right',1.,8)
    hand.joints[0].valid=True;hand.joints[0].xyz_m=np.array([.1,.2,.5])
    result=StereoHandResult(1,pose['timestamp_ns'],'cal','vendor_vs030',hands=[hand])
    out=segment_hands_for_result(result,pose,{'status':'unavailable','reason':'calibration_mismatch'})
    assert not out['hands'] and out['status']=='unavailable'
    np.testing.assert_array_equal(hand.joints[0].xyz_m,[.1,.2,.5])
