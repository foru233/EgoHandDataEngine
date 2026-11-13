import importlib.util
from pathlib import Path
import sys

import numpy as np

scripts=Path(__file__).parents[1]/'scripts'
spec=importlib.util.spec_from_file_location('audit_vio_frame_snapshots',scripts/'audit_vio_frame_snapshots.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def row(t,x,change=1):
    matrix=np.eye(4);matrix[0,3]=x
    return {'timestamp_ns':t,'frame_id':t,'reset_id':0,'map_epoch':1,'world_frame_id':1,
            'snapshot_consistent':True,'snapshot_available':True,'imu_initialized':False,
            'raw_pose_valid':True,'raw_matrix':matrix.tolist(),'map_change_index':change}


def test_local_revision_changes_are_not_confused_with_exact_epoch():
    report=audit.audit_snapshots([row(1,0),row(100_000_001,1,2),row(200_000_001,2,2)])
    assert report['same_epoch_short_gap_jump_revision_classes']=={'local_revision_changed':1,'local_revision_unchanged':1}


def test_rejected_snapshot_count_is_separate_from_raw_valid_rate():
    rejected=row(1,0);rejected.update(snapshot_consistent=False,raw_pose_valid=False,raw_matrix=None)
    report=audit.audit_snapshots([rejected,row(2,0)])
    assert report['inconsistent_snapshot_count']==1 and report['raw_valid_count']==1
    assert not report['violations']


def test_mismatched_body_identity_is_detected_even_with_consistency_flag():
    r=row(1,0);r['imu_initialized']=True
    r['body_state_candidate']={'frame_id':2,'timestamp_ns':1,'world_frame_id':1,'snapshot_consistent':True}
    report=audit.audit_snapshots([r])
    assert report['violations'][0]['reason']=='body_frame_id_mismatch'
