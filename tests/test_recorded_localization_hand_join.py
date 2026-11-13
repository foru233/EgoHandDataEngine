"""Exact-frame Map admission must survive world-hand recording/export joins."""
import copy
from dataclasses import replace
import json
import queue
import time
from types import SimpleNamespace

import numpy as np
import pytest

from egohand.core.types import Hand3D, StereoFrame, StereoHandResult, SourceMetadata
from egohand.vio.hand_world import RecordedLocalizationHandJoin, calibration_geometry_fingerprint
from egohand.vio.hybrid_quality import make_quality_gate
from egohand.vio.localization_state import LocalizationPipeline
from egohand.vio.recovery_contract import application_feedback
from egohand.vio.recovery_coordinator import RecoveryCoordinator


def admitted_pose(stamp, *, epoch=0, x=.3):
    coordinator=RecoveryCoordinator(application_admission=True)
    pipeline=LocalizationPipeline('hybrid',make_quality_gate('hybrid',stable_seconds=0),
        recovery_commit=lambda row:coordinator.finish_application_frame(application_feedback(row)))
    transform=np.eye(4);transform[0,3]=x
    row={'type':'pose','timestamp_ns':stamp,'frame_id':9007,'map_epoch':epoch,
         'map_id':epoch>>40,'reset_id':(epoch>>20)&((1<<20)-1),'map_revision':epoch&((1<<20)-1),
         'world_frame_id':epoch,'snapshot_available':True,
         'world_frame_revision':0,'snapshot_consistent':True,'tracking_state':'OK',
         'pose_valid':True,'matrix':transform.tolist(),'imu_initialized':False,
         'ba1_complete':False,'ba2_complete':False,'inertial_scale_finalized':False}
    coordinator.begin_frame([])
    row['recovery_preaccept']=coordinator.finish_frame(row,None)
    return pipeline.process(row,[])


@pytest.fixture
def context(tmp_path,calibration):
    stamp=1_000_000_000
    hand=Hand3D('Left',1.,17)
    hand.joints[0].valid=True;hand.joints[0].xyz_m=np.array([.1,.2,1.])
    result=StereoHandResult(7,stamp,calibration.calibration_id,'vendor_vs030',
        hands=[hand],session_id='hand-session')
    image=np.zeros((4,4,3),np.uint8)
    frame=StereoFrame(7,stamp,image,image,calibration_id=calibration.calibration_id,
        source_type='vendor_vs030',metadata={'clock_domain':'sdk_device',
            'timestamp_kind':'sdk_device_timestamp_raw_us_to_ns','recording_dir':str(tmp_path),
            'sdk_device_timestamp_raw':stamp//1000,'sdk_sequence':42,
            'acquired_perf_counter_ns':time.perf_counter_ns()})
    pose=admitted_pose(stamp)
    pose['replay']={'source':'offline_replay','recording':str(tmp_path),'frame_index':7,
        'device_ts_raw':stamp//1000,'sequence':42}
    pose['replay_config']={'schema_version':1,'fingerprint':'a'*64,
        'calibration_file_sha256':'b'*64,'camera_frame':'raw_cam0_channel4',
        'camera_geometry_sha256':calibration_geometry_fingerprint(calibration)}
    join=RecordedLocalizationHandJoin(tmp_path,calibration,result_session_id=result.session_id,
        vio_calibration=calibration,vio_config_fingerprint='a'*64)
    return SimpleNamespace(join=join,result=result,frame=frame,pose=pose,calibration=calibration)


def apply(c):return c.join.apply(c.result,c.frame,c.pose,calibration=c.calibration)


def test_exact_source_frame_uses_committed_map_without_changing_camera_points(context):
    c=context;world=apply(c)
    assert world['status']=='available'
    np.testing.assert_allclose(world['hands'][0]['xyz_world_m'][0],[.4,.2,1.])
    np.testing.assert_array_equal(c.result.hands[0].joints[0].xyz_m,[.1,.2,1.])
    assert world['hands'][0]['track_id']==17
    assert not world['hands'][0]['valid'][1] and np.isnan(world['hands'][0]['xyz_world_m'][1]).all()
    assert world['join']['source_frame_index']==7 and world['join']['native_frame_id']==9007
    assert world['join']['exposure_synchrony_verified'] is False
    assert world['world_reference']['cross_frame_merge_permitted'] is False
    assert world['world_reference']['coordinate_continuity']=='unknown'
    assert c.result.world_hands is None


@pytest.mark.parametrize('target,key,value,reason',[
    ('result','session_id','other','session_mismatch'),
    ('result','unit','mm','invalid_camera_frame_or_calibration'),
    ('result','coordinate_frame','rectified','invalid_camera_frame_or_calibration'),
    ('result','calibration_id','other','invalid_camera_frame_or_calibration'),
    ('result','frame_id',8,'source_frame_mismatch'),
    ('result','timestamp_ns',1_000_001_000,'source_frame_mismatch'),
    ('metadata','clock_domain','host','clock_domain_mismatch'),
    ('metadata','timestamp_kind','host_time','clock_domain_mismatch'),
    ('metadata','recording_dir','/other','recording_mismatch'),
    ('metadata','sdk_sequence',43,'source_frame_mismatch'),
    ('metadata','sdk_device_timestamp_raw',12,'source_frame_mismatch'),
    ('replay','frame_index',8,'source_frame_mismatch'),
    ('replay','sequence',43,'source_frame_mismatch'),
    ('replay','device_ts_raw',12,'source_frame_mismatch'),
    ('replay','recording','/other','recording_mismatch'),
    ('pose','timestamp_ns',1_000_001_000,'source_frame_mismatch'),
    ('pose','localization',None,'localization_contract_required'),
    ('pose','replay_config',None,'localization_config_mismatch'),
    ('config','fingerprint','c'*64,'localization_config_mismatch'),
    ('config','schema_version',2,'localization_config_mismatch'),
    ('config','camera_geometry_sha256','c'*64,'localization_config_mismatch'),
    ('config','camera_frame','rectified_cam1','localization_config_mismatch'),
    ('recovery','stage','native','recovery_binding_invalid'),
    ('recovery','frame_id',8,'recovery_binding_invalid'),
])
def test_mismatched_provenance_never_emits_world_points(context,target,key,value,reason):
    c=context
    if target=='result':setattr(c.result,key,value)
    else:
        dest={'metadata':c.frame.metadata,'replay':c.pose['replay'],'pose':c.pose,
              'recovery':c.pose['recovery_preaccept'],'config':c.pose['replay_config']}[target]
        dest[key]=value
    world=apply(c)
    assert world['status']=='unavailable' and world['reason']==reason and world['hands']==[]
    assert c.result.hands[0].joints[0].valid


def test_same_named_but_changed_calibration_is_rejected(context):
    c=context
    K=c.calibration.K_left.copy();K[0,0]+=1
    c.calibration=replace(c.calibration,K_left=K)
    assert apply(c)['reason']=='calibration_mismatch'


def test_saved_and_kalibr_ids_can_differ_only_if_numeric_geometry_matches(context):
    c=context
    other_name=replace(c.calibration,calibration_id='other-filename',calibration_source='other-path')
    assert calibration_geometry_fingerprint(other_name)==calibration_geometry_fingerprint(c.calibration)
    c.join=RecordedLocalizationHandJoin(c.join.recording_dir,c.calibration,
        result_session_id=c.result.session_id,vio_calibration=other_name,vio_config_fingerprint='a'*64)
    assert apply(c)['status']=='available'
    changed=replace(other_name,t_right_left=other_name.t_right_left*2)
    c.join=RecordedLocalizationHandJoin(c.join.recording_dir,c.calibration,
        result_session_id=c.result.session_id,vio_calibration=changed,vio_config_fingerprint='a'*64)
    assert apply(c)['reason']=='calibration_mismatch'


@pytest.mark.parametrize('change',['quarantine','invalid_snapshot','unknown_contract','not_admitted'])
def test_no_legacy_valid_bit_can_override_consumer_admission(context,change):
    c=context
    if change=='quarantine':c.pose['recovery_preaccept']['await_visual_requalification']=True
    elif change=='invalid_snapshot':c.pose['snapshot_consistent']=False
    elif change=='unknown_contract':c.pose['localization']['contract_version']=99
    else:c.pose['localization']['admission']['world_hands_permitted']=False
    assert c.pose['pose_valid'] is True
    world=apply(c)
    reason='localization_contract_required' if change=='unknown_contract' else 'localization_not_admitted'
    assert world['status']=='unavailable' and world['reason']==reason
    assert world['hands']==[]


def test_duplicate_or_changed_worker_session_cannot_reuse_previous_join(context):
    c=context
    assert apply(c)['status']=='available'
    assert apply(c)['reason']=='non_monotonic_join'
    c.result.timestamp_ns+=1000;c.frame.timestamp_ns+=1000
    c.frame.metadata['sdk_device_timestamp_raw']+=1
    later=admitted_pose(c.result.timestamp_ns)
    later['replay']={**c.pose['replay'],'device_ts_raw':c.result.timestamp_ns//1000}
    later['replay_config']=c.pose['replay_config']
    c.pose=later
    assert apply(c)['reason']=='recovery_binding_invalid'


def test_effective_phase_config_latches_once_and_calibration_file_must_match(context):
    c=context
    c.join=RecordedLocalizationHandJoin(c.join.recording_dir,c.calibration,
        result_session_id=c.result.session_id,vio_calibration=c.calibration,
        vio_config_fingerprint=None,vio_calibration_sha256='b'*64)
    c.pose['replay_config']['calibration_file_sha256']='c'*64
    assert apply(c)['reason']=='localization_config_mismatch'
    assert c.join.recovery_session_id is None and c.join.vio_config_fingerprint is None
    c.pose['replay_config']['calibration_file_sha256']='b'*64
    world=apply(c)
    assert world['status']=='available' and world['join']['vio_config_fingerprint']=='a'*64
    assert c.join.vio_config_fingerprint=='a'*64


def test_unavailable_localization_keeps_camera_joints_and_export_identity(context,tmp_path):
    from egohand.storage.exporter import export_all,load_json
    c=context
    c.result.world_hands=apply(c)
    later=copy.deepcopy(c.result);later.frame_id+=1;later.timestamp_ns+=1000
    later.world_hands={'frame_id':later.frame_id,'timestamp_ns':later.timestamp_ns,
        'status':'unavailable','reason':'localization_not_admitted','hands':[]}
    out=export_all([c.result,later],tmp_path/'export')
    loaded=load_json(out/'results.json')
    assert loaded[0].world_hands['status']=='available'
    assert loaded[0].world_hands['join']['recovery_session_id']==c.pose['recovery_preaccept']['session_id']
    assert loaded[1].world_hands['hands']==[]
    for row in loaded:
        np.testing.assert_array_equal(row.hands[0].joints[0].xyz_m,[.1,.2,1.])
        assert row.world_hands['world_reference']['cross_frame_merge_permitted'] is False
    policy=json.loads((out/'world_reference_policy.json').read_text(encoding='utf8'))
    assert policy['cross_frame_merge_permitted'] is False


def test_worker_attaches_world_result_before_recording(context):
    from egohand.gui.workers import PipelineWorker
    c=context
    class Engine:
        detectors=None;rectifier=None;last_timestamp=None
        calibration=c.calibration
        def process(self,frame,**kwargs):self.last_frame=frame;return c.result
    class Sink:
        error=None;started=time.perf_counter();bytes_written=0
        queue=queue.Queue()
        def submit(self,kind,value,offline):
            assert kind=='result' and value.world_hands['status']=='available'
            self.saved=copy.deepcopy(value)
    source=SimpleNamespace(loop=False,get_metadata=lambda:SourceMetadata('vs030_recorded'))
    worker=PipelineWorker(source,offline=True)
    worker.engine=Engine();worker._recorded_world_join=c.join;worker.recorder=Sink()
    c.frame.metadata['vio_pose']=c.pose
    worker._process_frame(c.frame,time.perf_counter())
    assert worker.recorder.saved.world_hands['join']['source_frame_index']==c.result.frame_id
