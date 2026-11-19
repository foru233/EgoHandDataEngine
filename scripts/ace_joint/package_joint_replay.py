"""Publish joint forward outputs to legacy replay schema, without post-fit wrist shifts."""
from pathlib import Path
import argparse,errno,json,sys,os,shutil,hashlib,sqlite3
import numpy as np
from itertools import zip_longest
from joint_work_paths import PROJECT
sys.dont_write_bytecode=True;sys.path[:0]=[str(PROJECT/'src'),str(PROJECT/'scripts')]
from egohand.storage.replay import load_recorded_results
from egohand.core.types import Hand3D,serializable,result_from_dict
from egohand.vio.hand_pose_binding import consumer_result_display_pose
from egohand.vio.hand_world import _transform_hands
from egohand.vio.local_segments import segment_hands_for_result
from egohand.body.smpl_mapping import SMPL_JOINT_NAMES,SMPL_PARENT_IDS
from egohand.body.smpl_mount import camera_forehead_offset_m, forehead_target
from smpl_body_mesh import SmplBody
from offline_mano import make_model

def file_sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def link_or_copy(source,target):
    try:
        os.link(source,target)
    except OSError as exc:
        if exc.errno!=errno.EXDEV and getattr(exc,'winerror',None)!=17:
            raise
        shutil.copyfile(source,target)

def finalize_existing(a):
    if (a.output/'unified_smpl_manifest.json').exists():raise FileExistsError('Already published')
    jm=json.loads((a.run/'joint_manifest.json').read_text())
    assert jm['status']=='completed' and jm['configuration']['start']==0 and jm['configuration']['frames']==0
    with np.load(a.run/'joint_params.npz') as f:q={k:f[k] for k in f.files}
    valid=np.zeros(len(q['valid']),bool);counts={'left':0,'right':0};floor=[];head=[];wrists=[]
    with (a.output/'derived_pose.jsonl').open(encoding='utf8') as f:
        for i,line in enumerate(f):
            row=json.loads(line);assert (row['frame_id'],row['timestamp_ns'])==(int(q['frame_id'][i]),int(q['timestamp_ns'][i]))
            if row['body']:
                valid[i]=True;d=row['body']['diagnostics'];floor.extend(map(abs,d['foot_contact_errors_m']));head.append(d['head_error_m'])
            for m in row['mano']['predicted']:
                counts[m['side']]+=1;wrists.append(m['wrist_connection_residual_m'])
    assert i+1==len(valid)
    with (a.output/'results/frames.jsonl').open(encoding='utf8') as f:
        for i,line in enumerate(f):
            row=json.loads(line);assert (row['frame_id'],row['timestamp_ns'])==(int(q['frame_id'][i]),int(q['timestamp_ns'][i]))
    assert i+1==len(valid)
    digest=file_sha(a.output/'results/frames.jsonl')
    with sqlite3.connect(a.output/'display_cache.sqlite3') as db:
        ids=db.execute('SELECT idx,frame_id,timestamp_ns FROM frames ORDER BY idx').fetchall()
        assert len(ids)==len(valid) and all((fid,ts)==(int(q['frame_id'][i]),int(q['timestamp_ns'][i])) for i,fid,ts in ids)
        db.execute('UPDATE metadata SET value=? WHERE key=?',(json.dumps(digest),'result_sha256'))
    keys=['body_pose','global_orient','transl','betas','joints_world','rotations_world','frame_id','timestamp_ns','world_to_smpl']
    np.savez_compressed(a.output/'smpl_pose.npz',**{k:q[k] for k in keys},valid=valid)
    m=json.loads((a.source/'unified_smpl_manifest.json').read_text())
    m.update(status='completed',source=str(a.source),algorithm='joint_smpl_mano_v2',valid=int(valid.sum()),
        source_results_sha256=file_sha(a.source/'results/frames.jsonl'),result_sha256=digest,posthoc_wrist_shift=False,initialization_only='previous_body_and_hand_fit',
        body_objective='new_joint_stereo_pixel_body_hand_forward',joint_run=str(a.run),experimental=True)
    m['camera_forehead_offset_mm']=jm['configuration'].get('camera_forehead_offset_mm',0.)
    shutil.copyfile(a.run/'joint_manifest.json',a.output/'joint_manifest.json')
    report=dict(status='passed',frames=len(valid),valid_body_frames=int(valid.sum()),hand_mesh_frames=counts,
        max_floor_residual_m=max(floor,default=0),max_head_residual_m=max(head,default=0),
        max_wrist_connection_residual_m=max(wrists,default=0),posthoc_wrist_shift=False,independent_3d_GT=False,only_right_hand_is_expected=True)
    (a.output/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    (a.output/'unified_smpl_manifest.json').write_text(json.dumps(m,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--resume-finalize',action='store_true');p.add_argument('--share-image-cache',action='store_true');p.add_argument('--cache-source',type=Path);a=p.parse_args()
    if a.resume_finalize:return finalize_existing(a)
    if a.output.exists():raise FileExistsError(a.output)
    jm=json.loads((a.run/'joint_manifest.json').read_text())
    assert jm['status']=='completed' and jm['configuration']['start']==0 and jm['configuration']['frames']==0,'Pilot output cannot be published as full replay'
    offset_mm=jm['configuration'].get('camera_forehead_offset_mm',0.)
    offset_m=camera_forehead_offset_m(offset_mm)
    with np.load(a.run/'joint_params.npz') as f:q={k:f[k] for k in f.files}
    source_manifest=json.loads((a.source/'unified_smpl_manifest.json').read_text())
    direct=source_manifest.get('body_initialization')=='neutral_smpl'
    if direct:
        with np.load(Path(jm['configuration']['hands'])/'associated_observations.npz') as observations:
            measured=observations['valid'].any(axis=1)
            measured_stereo=observations['stereo_valid']
            detection_indices=observations['detection_indices'] if 'detection_indices' in observations else None
    def source_rows():
        if direct:
            with (a.source/'results/frames.jsonl').open(encoding='utf8') as frames:
                for line in frames:yield result_from_dict(json.loads(line))
            return
        with (a.source/'results/frames.jsonl').open(encoding='utf8') as frames,(a.source/'derived_pose.jsonl').open(encoding='utf8') as poses:
            count=0
            for line,pose_line in zip_longest(frames,poses):
                assert line is not None and pose_line is not None,'Mismatched sidecar length'
                r=result_from_dict(json.loads(line));d=json.loads(pose_line)
                assert (r.frame_id,r.timestamp_ns)==(d['frame_id'],d['timestamp_ns'])
                r.diagnostics['offline_derived_pose']=d;count+=1;yield r
            assert count==len(q['valid'])
            assert not frames.read().strip() and not poses.read().strip()
    model=SmplBody().model;weights=model.lbs_weights.detach().numpy()
    feet=[np.flatnonzero(weights[:,ankle]+weights[:,toe]>.75) for ankle,toe in ((7,10),(8,11))]
    body_vertices=np.load(a.run/'smpl_vertices.npy',mmap_mode='r');hand_vertices=np.load(a.run/'hand_vertices.npy',mmap_mode='r')
    faces={side:make_model(side,'cpu').faces.tolist() for side in ('left','right')}
    valid=q['valid']&q['computed'];floor_errors=[];heads=[];wrist_errors=[];mesh_counts={'left':0,'right':0};rejected={};n=len(q['valid'])
    a.output.mkdir(parents=True);(a.output/'results').mkdir()
    for name in ['processing_calibration.yaml','reprocessing.json','smpl_mesh.json']:
        link_or_copy(a.source/name,a.output/name)
    link_or_copy(a.run/'smpl_vertices.npy',a.output/'smpl_vertices.npy')
    with (a.output/'results/frames.jsonl').open('x',encoding='utf8') as resultfile,(a.output/'derived_pose.jsonl').open('x',encoding='utf8') as derived:
        for i,result in enumerate(source_rows()):
            assert (result.frame_id,result.timestamp_ns)==(int(q['frame_id'][i]),int(q['timestamp_ns'][i]))
            previous=result.diagnostics.pop('offline_derived_pose',{}) or {};T=consumer_result_display_pose(result)
            original_body=previous.get('body');body=None;meshes=[];reason=[]
            if valid[i]:
                d=dict((original_body or {}).get('diagnostics') or {})
                ground=float(q['ground_y_m'][i]) if 'ground_y_m' in q else float(d['ground_y_m'])
                ferr=[float(body_vertices[i,ids,1].max()-ground) for ids in feet]
                herr=float(np.linalg.norm(body_vertices[i,source_manifest['head_anchor_vertex']]-forehead_target(T,offset_m)))
                angle=float(np.degrees(np.linalg.norm(q['body_pose'][i].reshape(23,3)[14])))
                if max(map(abs,ferr))>.012:reason.append('joint_floor_residual')
                if herr>1e-4:reason.append('joint_head_attachment')
                if angle>85:reason.append('joint_neck_angle')
                if reason:
                    valid[i]=False
                    for why in reason:rejected[why]=rejected.get(why,0)+1
                else:
                    floor_errors.extend(map(abs,ferr));heads.append(herr)
                    d.update(source='joint_smpl_mano_v2',body_mesh_vertices_path=str(a.output/'smpl_vertices.npy'),
                        ground_y_m=ground,head_anchor_version=source_manifest['head_anchor_version'],unified_smpl=True,
                        body_mesh_index=i,body_mesh_coordinate_frame='body_world',height_m=1.75,
                        foot_contact_errors_m=ferr,head_error_m=herr,head_local_angle_deg=angle,
                        head_anchor='camera_mount_offset_optical_y',camera_forehead_offset_mm=offset_mm,
                        observed_body_joints=[],body_is_inferred=True,posthoc_wrist_shift=False,
                        supervision='stereo_hand_pixels_and_camera_head_pose_with_body_priors',
                        smpl_wrist_rotations_world={side:q['rotations_world'][i,j].tolist() for side,j in [('left',20),('right',21)]})
                    original_body=original_body or dict(frame_id=result.frame_id,timestamp_ns=result.timestamp_ns,
                        reference_key=None,status='estimated_preview',measured_joints=[])
                    body={**original_body,'joints':dict(zip(SMPL_JOINT_NAMES,q['joints_world'][i].tolist())),
                        'bones':[[SMPL_JOINT_NAMES[parent],SMPL_JOINT_NAMES[j]] for j,parent in enumerate(SMPL_PARENT_IDS) if parent>=0],
                        'diagnostics':d}
                    d['mano_attached_sides']=[]
                    for h,side in enumerate(['left','right']):
                        if not q['hand_valid'][i,h]:continue
                        v=hand_vertices[i,h];j=q['hand_joints'][i,h,[0]+list(range(2,21))]
                        world=v@T[:3,:3].T+T[:3,3];worldj=j@T[:3,:3].T+T[:3,3]
                        wrist=q['joints_world'][i,20+h];R=q['rotations_world'][i,20+h]
                        error=float(np.linalg.norm(worldj[0]-wrist));assert error<1e-5
                        local=(world-wrist)@R;localj=(worldj-wrist)@R
                        transform=np.eye(4);transform[:3,:3]=R;transform[:3,3]=wrist
                        meshes.append(dict(side=side,track_id=h+1,vertices=world.tolist(),joints20=worldj.tolist(),faces=faces[side],
                            vertices_wrist_local=local.tolist(),joints_wrist_local=localj.tolist(),T_world_from_smpl_wrist=transform.tolist(),
                            coordinate_frame='body_world',smpl_parent_joint=20+h,reconstruction_source='joint_smpl_mano_v2',
                            observed_wrist_offset_m=0.,posthoc_wrist_shift=False,wrist_connection_residual_m=error))
                        mesh_counts[side]+=1;wrist_errors.append(error);d['mano_attached_sides'].append(side)
            if direct:
                result.hands=[]
                accepted=[]
                for h,side in enumerate(('Left','Right')):
                    if not q['computed'][i] or not q['hand_valid'][i,h]:continue
                    hand=Hand3D(side,None,h+1)
                    hand.tracking_status='offline_model_fitted'
                    indices=detection_indices[i,h].tolist() if detection_indices is not None else [-1,-1]
                    pixels=[]
                    for index,detections in zip(indices,(result.left_detections,result.right_detections)):
                        pixels.append(detections[index] if 0<=index<len(detections) else None)
                    for j,joint in enumerate(hand.joints):
                        for attr,detection in zip(('left_uv_px','right_uv_px'),pixels):
                            if detection is not None and detection.valid[j] and np.isfinite(detection.pixel_landmarks[j]).all():
                                setattr(joint,attr,tuple(detection.pixel_landmarks[j].tolist()))
                        point=q['hand_joints'][i,h,j]
                        if not np.isfinite(point).all() or point[2]<=.01:continue
                        joint.xyz_m=point;joint.valid=True
                        joint.status=('observed_stereo' if measured_stereo[i,h,j] else
                                      'inferred_single_view' if measured[i,h,j] else 'inferred_mano_prior')
                        joint.uncertainty_status='MODEL_FITTED_UNCALIBRATED'
                    hand.valid_joint_count=sum(joint.valid for joint in hand.joints)
                    if hand.valid_joint_count:
                        result.hands.append(hand);accepted.append(indices)
                result.diagnostics['valid_joints']=sum(hand.valid_joint_count for hand in result.hands)
                quality=result.diagnostics.get('association_quality') or {}
                result.diagnostics['association_quality']={**quality,'accepted_pairs':accepted}
            for hand in result.hands:
                h=['left','right'].index(hand.handedness.lower())
                if q['computed'][i] and q['hand_valid'][i,h]:
                    for j,joint in enumerate(hand.joints):
                        if joint.valid:joint.xyz_m=q['hand_joints'][i,h,j]
            if T is not None and isinstance(result.world_hands,dict):
                result.world_hands={**result.world_hands,'hands':_transform_hands(result,T),
                                    'status':'available' if result.hands else 'no_valid_hand_joints'}
                pose=result.diagnostics.get('vio_pose') or result.diagnostics.get('live_vio_pose') or {}
                result.segment_hands=segment_hands_for_result(result,pose,result.world_hands)
            elif direct:
                result.world_hands=None;result.segment_hands=None
            result.diagnostics['reconstruction_algorithm']='joint_smpl_mano_v2'
            result.diagnostics['offline_hand_reconstruction']={**result.diagnostics.get('offline_hand_reconstruction',{}),'source':str(a.run),'algorithm':'joint_smpl_mano_v2'}
            resultfile.write(json.dumps(serializable(result),allow_nan=False)+'\n')
            row=dict(frame_id=result.frame_id,timestamp_ns=result.timestamp_ns,body=body,mano=dict(predicted=meshes,coordinate_frame='body_world',binding='joint_forward_exact_wrist'),
                unified_smpl=True,head_anchor_version=source_manifest['head_anchor_version'],algorithm='joint_smpl_mano_v2')
            if body:row['body_mesh_ref']='smpl_mesh.json'
            else:row['body_unavailable_reason']=reason or ['no_admitted_initial_camera_body_state']
            derived.write(json.dumps(row,allow_nan=False)+'\n')
    # Cache is copied before metadata mutation; source images and metadata remain untouched.
    cache_source=a.cache_source or a.source
    digest=file_sha(a.output/'results/frames.jsonl')
    if a.share_image_cache:
        marker=dict(schema_version=1,source=str(cache_source.resolve()),result_sha256=digest,
            calibration_sha256=file_sha(a.output/'processing_calibration.yaml'),read_only=True)
        assert file_sha(cache_source/'processing_calibration.yaml')==marker['calibration_sha256']
        (a.output/'shared_image_cache.json').write_text(json.dumps(marker,indent=2),encoding='utf8')
        cache_path=cache_source/'display_cache.sqlite3'
    else:
        shutil.copyfile(cache_source/'display_cache.sqlite3',a.output/'display_cache.sqlite3')
        cache_path=a.output/'display_cache.sqlite3'
    with sqlite3.connect('file:'+cache_path.as_posix()+'?mode=ro',uri=True) as db:
        ids=db.execute('SELECT idx,frame_id,timestamp_ns FROM frames ORDER BY idx').fetchall()
        assert len(ids)==n
        assert all((fid,ts)==(int(q['frame_id'][i]),int(q['timestamp_ns'][i])) for i,fid,ts in ids)
    if not a.share_image_cache:
        with sqlite3.connect(cache_path) as db:db.execute('UPDATE metadata SET value=? WHERE key=?',(json.dumps(digest),'result_sha256'))
    keys=['body_pose','global_orient','transl','betas','joints_world','rotations_world','frame_id','timestamp_ns','world_to_smpl']
    np.savez_compressed(a.output/'smpl_pose.npz',**{k:q[k] for k in keys},valid=valid)
    sm={**source_manifest,'status':'completed','source':str(a.source),'algorithm':'joint_smpl_mano_v2','joint_run':str(a.run),
        'frames':n,'valid':int(valid.sum()),'posthoc_wrist_shift':False,'body_and_hands_jointly_optimized':True,
        'body_without_hand':'joint_body_prior_only','network_training':False,'hand_mesh_frames':mesh_counts,
        'source_results_sha256':file_sha(a.source/'results/frames.jsonl'),'result_sha256':digest,'experimental':True,
        'shared_image_cache':a.share_image_cache,'head_mount':'camera_mount_offset_optical_y',
        'camera_forehead_offset_mm':offset_mm,'complete_reconstruction_passes':1 if direct else None,
        'initialization_only':False}
    shutil.copyfile(a.run/'joint_manifest.json',a.output/'joint_manifest.json')
    report=dict(status='passed',frames=n,valid_body_frames=int(valid.sum()),hand_mesh_frames=mesh_counts,rejected=rejected,
        max_floor_residual_m=max(floor_errors,default=0),max_head_residual_m=max(heads,default=0),
        max_wrist_connection_residual_m=max(wrist_errors,default=0),posthoc_wrist_shift=False,
        independent_3d_GT=False)
    (a.output/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    (a.output/'unified_smpl_manifest.json').write_text(json.dumps(sm,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
