"""Synthetic analysis fixtures verify that geometry changes cannot become recovery approval."""
import importlib.util
from pathlib import Path
import sys

import numpy as np

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
try:
    spec=importlib.util.spec_from_file_location('vio_commit_analysis',SCRIPTS/'analyze_vio_commit_log.py')
    analysis=importlib.util.module_from_spec(spec);spec.loader.exec_module(analysis)
finally:
    sys.path.remove(str(SCRIPTS))


def pose(x=0,y=0,z=0,angle=0):
    m=np.eye(4);c,s=np.cos(angle),np.sin(angle)
    m[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];m[:3,3]=[x,y,z]
    return m


def commit_pair(deform=False):
    transform=pose(2,3,4,.4)
    positions=[pose(),pose(1),pose(0,1),pose(0,0,1)]
    before={'schema_version':1,'sequence':1,'type':'local_ba','event':1,'kind':'local_inertial_ba',
            'phase':'before','map_id':0,'revision':3,'map_change_index':4,
            'keyframes':[{'id':i,'world_from_camera':m.flatten().tolist()} for i,m in enumerate(positions)],
            'sampled_points':[{'id':i,'bad':False,'position':m[:3,3].tolist()} for i,m in enumerate(positions)]}
    after={**before,'sequence':2,'phase':'after','map_change_index':5,
           'keyframes':[{'id':i,'world_from_camera':(transform@m).flatten().tolist()} for i,m in enumerate(positions)],
           'sampled_points':[{'id':i,'bad':False,'position':(transform@m)[:3,3].tolist()} for i,m in enumerate(positions)]}
    if deform: after['keyframes'][2]['world_from_camera'][3]+=.5
    return before,after


def test_rigid_and_nonrigid_geometry_are_distinguishable_without_approval():
    rigid=analysis.analyze(commit_pair())
    deformed=analysis.analyze(commit_pair(True))
    a=rigid['paired_local_commits'][0]['keyframe_pose_fit']
    b=deformed['paired_local_commits'][0]['keyframe_pose_fit']
    assert a['position_residual_p50_p95_max_m'][-1]<1e-12
    assert b['position_residual_p50_p95_max_m'][-1]>.49
    assert rigid['integrity_errors']==[]
    assert 'recovery_approved' not in rigid


def test_truncated_duplicate_and_incoherent_commits_are_not_evidence():
    before,after=commit_pair()
    assert analysis.analyze([before])['integrity_errors'][0]['reason']=='missing_commit_after'
    assert analysis.analyze([before,before,after])['integrity_errors']
    assert analysis.analyze([before,{**after,'map_change_index':8}])['paired_local_commits']==[]


def frame(seq,id,ref,world):
    return {'schema_version':1,'sequence':seq,'type':'frame','frame_id':id,'timestamp_sec':float(id+1),
            'map_id':0,'reset_generation':0,'atlas_generation':2,'revision':3,'map_change_index':4,
            'tracking_state':2,'consistent':True,'pose_valid':True,'reference_kf_id':1,
            'reference_world_from_camera':ref.flatten().tolist(),'world_from_camera':world.flatten().tolist()}


def published(f,jump=False):
    return {'frame_id':f['frame_id'],'timestamp_ns':9000000000000+round(f['timestamp_sec']*1e9),'map_id':f['map_id'],
            'reset_id':0,'map_revision':3,'map_change_index':4,'snapshot_consistent':True,
            'jump_detected':jump,'replay':{'frame_index':100+f['frame_id']}}


def bridged(frames):
    records=[]
    for f in frames:
        records.append({**f,'sequence':len(records)+1})
        records.append({**f,'type':'bridge_frame','sequence':len(records)+1,
                        'timestamp_ns':published(f)['timestamp_ns'],'epoch_ns':9000000000000,
                        'native_timestamp_sec':f['timestamp_sec']})
    return records


def test_reference_transport_is_counterfactual_and_identity_bound():
    a,b=frame(1,0,pose(),pose()),frame(2,1,pose(2),pose(2.01))
    poses=[published(a),published(b,True)]
    report=analysis.analyze(bridged([a,b]),poses)
    assert report['integrity_errors']==[]
    jump=report['jump_contexts'][0]
    assert jump['source_frame_index']==101
    assert jump['same_reference_counterfactual']['residual_pose_step']['translation_m']<.011
    assert jump['coordinate_continuity_verified'] is False
    mismatch=analysis.analyze(bridged([a,{**b,'reference_kf_id':2}]),poses)
    assert 'same_reference_counterfactual' not in mismatch['jump_contexts'][0]
    mismatch=analysis.analyze(bridged([a,{**b,'atlas_generation':4}]),poses)
    assert mismatch['jump_contexts'][0]['same_map_lifetime'] is False
    assert 'same_reference_counterfactual' not in mismatch['jump_contexts'][0]


def test_missing_mismatched_and_malformed_frame_evidence_is_rejected():
    f=frame(1,0,pose(),pose())
    assert analysis.analyze([],[published(f)])['integrity_errors']
    assert analysis.analyze(bridged([f]),[{**published(f),'map_id':1}])['integrity_errors']
    records=bridged([f]);records[1]['epoch_ns']+=100
    assert analysis.analyze(records,[published(f)])['integrity_errors']
    assert analysis.matrix([None]*16) is None
    assert analysis.matrix(np.zeros((4,4))) is None
    reflection=np.eye(4);reflection[0,0]=-1
    assert analysis.matrix(reflection) is None


def test_getter_invalidation_blocks_use_of_earlier_tracking_capture():
    a,b=frame(1,0,pose(),pose()),frame(2,1,pose(),pose(.01))
    records=bridged([a,b]);records[1]['consistent']=False;records[1]['pose_valid']=False
    rows=[{**published(a),'snapshot_consistent':False},published(b,True)]
    report=analysis.analyze(records,rows)
    assert report['integrity_errors']==[]
    assert report['getter_invalidated_captures']==1
    assert 'native_consecutive_pose_step' not in report['jump_contexts'][0]
