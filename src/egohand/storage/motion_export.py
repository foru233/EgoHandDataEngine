"""Add local camera and hand outputs to the existing export bundle."""
import csv
import json
from pathlib import Path

import numpy as np

from ..core.types import serializable


def export_local_motion(results, directory):
    directory=Path(directory)
    rows=[]
    n=len(results); slots=max(2,max((len((r.segment_hands or {}).get('hands',[])) for r in results),default=0))
    matrices=np.full((n,4,4),np.nan);session_matrices=matrices.copy()
    valid=np.zeros(n,dtype=bool)
    xyz=np.full((n,slots,21,3),np.nan); joint_valid=np.zeros((n,slots,21),dtype=bool)
    tracks=np.full((n,slots),-1,dtype=np.int64)
    handedness=np.full((n,slots),'',dtype='U32')
    hand_rows=[]
    for i,r in enumerate(results):
        motion=r.local_motion or {'status':'unavailable','pose_valid':False,'reason':'not_recorded'}
        row={**motion,'frame_id':r.frame_id,'result_timestamp_ns':r.timestamp_ns,
             'result_session_id':r.session_id,'calibration_id':r.calibration_id,
             'source_type':r.source_type}
        rows.append(row)
        if motion.get('pose_valid') is True:
            matrices[i]=motion['T_segment_from_camera'];valid[i]=True
            if motion.get('T_session_from_camera') is not None:
                session_matrices[i]=motion['T_session_from_camera']
        segment=r.segment_hands or {}
        for k,h in enumerate(segment.get('hands',[])):
            points=np.asarray(h['xyz_segment_m'],dtype=float)
            mask=np.asarray(h['valid'],dtype=bool) & np.isfinite(points).all(axis=1)
            xyz[i,k,mask]=points[mask];joint_valid[i,k]=mask
            tracks[i,k]=h['track_id'];handedness[i,k]=h['handedness']
            for j in range(21):
                point=points[j] if mask[j] else [None]*3
                hand_rows.append({'frame_id':r.frame_id,'timestamp_ns':r.timestamp_ns,
                    'pose_timestamp_ns':segment.get('pose_timestamp_ns'),
                    'result_session_id':r.session_id,'session_id':segment.get('localization_session_id'),
                    'segment_id':segment.get('segment_id'),'calibration_id':r.calibration_id,
                    'track_id':h['track_id'],'handedness':h['handedness'],'joint_id':j,
                    'unit':'m','coordinate_frame':'segment','valid':bool(mask[j]),
                    'x_m':point[0],'y_m':point[1],'z_m':point[2],
                    'reference_continuity_status':segment.get('reference_continuity_status')})
    (directory/'camera_trajectory.json').write_text(json.dumps(serializable(rows),ensure_ascii=False,allow_nan=False,indent=2),encoding='utf8')
    names=['frame_id','result_timestamp_ns','timestamp_ns','result_session_id','session_id','segment_id',
        'calibration_id','pose_source','pose_valid','unit','camera_frame','status','reason',
        'map_id','reset_id','map_revision','world_frame_revision','reference_generation',
        'reference_continuity_status','T_segment_from_camera','T_session_from_camera','T_session_from_segment']
    with (directory/'camera_trajectory.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=names);w.writeheader()
        for row in rows:
            w.writerow({k:json.dumps(row[k],allow_nan=False) if k.startswith('T_') and row.get(k) is not None else row.get(k) for k in names})
    common={'frame_id':np.array([r.frame_id for r in results],dtype=np.int64),
        'timestamp_ns':np.array([r.timestamp_ns for r in results],dtype=np.int64),
        'pose_timestamp_ns':np.array([(r.local_motion or {}).get('timestamp_ns') if type((r.local_motion or {}).get('timestamp_ns')) is int else -1 for r in results],dtype=np.int64),
        'session_id':np.array([(r.local_motion or {}).get('session_id') or '' for r in results],dtype=str),
        'result_session_id':np.array([r.session_id for r in results],dtype=str),
        'segment_id':np.array([(r.local_motion or {}).get('segment_id') or '' for r in results],dtype=str),
        'calibration_id':np.array([r.calibration_id or '' for r in results],dtype=str),
        'unit':np.array('m'),'schema_version':np.array(1),
        'reference_continuity_status':np.array([(r.local_motion or {}).get('reference_continuity_status') or 'unavailable' for r in results],dtype=str),
        'cross_segment_merge_permitted':np.array(False)}
    np.savez_compressed(directory/'camera_trajectory.npz',**common,pose_valid=valid,
        T_segment_from_camera=matrices,T_session_from_camera=session_matrices,
        camera_frame=np.array('camera_left_optical'))
    np.savez_compressed(directory/'segment_hands.npz',**common,xyz_segment_m=xyz,valid=joint_valid,
        track_id=tracks,handedness=handedness,coordinate_frame=np.array('segment'))
    hand_names=['frame_id','timestamp_ns','pose_timestamp_ns','result_session_id','session_id','segment_id',
        'calibration_id','track_id','handedness','joint_id','unit','coordinate_frame','valid',
        'x_m','y_m','z_m','reference_continuity_status']
    with (directory/'segment_hands.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=hand_names);w.writeheader();w.writerows(hand_rows)
    (directory/'local_coordinate_policy.json').write_text(json.dumps({
        'schema_version':1,'transform_convention':'T_A_from_B maps B points to A',
        'segment_origin':'first admitted raw cam0 pose in each uninterrupted reported reference',
        'session_origin':'segment 0 only; all later segment-to-session transforms unavailable',
        'boundaries':'admission gap, recovery session, native frame/transaction sequence gap, reported map/reset/world/reference/Atlas or pose-source change',
        'cross_segment_merge_permitted':False,'coordinate_continuity_verified':False,
        'independent_odom':False,'metric_accuracy_verified':False,
        'invalid_coordinates':'JSON null; NPZ NaN plus valid mask; CSV empty',
        'legacy_world_policy':'unchanged; local segment estimates do not authorize legacy world concatenation'},indent=2),encoding='utf8')
