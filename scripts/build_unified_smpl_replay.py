"""Build a new replay with one SMPL pose for official joints and surface.

Never overwrites an existing directory. Uses observed camera pose and valid
same-frame hands, not the legacy body estimator's inferred target skeleton.
"""
import argparse,json,sys,os,time,hashlib
from pathlib import Path
import numpy as np
import torch
torch.set_num_threads(1)
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from smpl_body_mesh import SmplBody
from egohand.body.smpl_unified import UnifiedSmpl
from egohand.body.smpl_mapping import SMPL_JOINT_NAMES,SMPL_PARENT_IDS,SMPL_FROM_BODY
from egohand.storage.replay import load_recorded_results
from egohand.vio.hand_pose_binding import consumer_result_display_pose
from egohand.vio.local_segments import display_reference_key
from egohand.body.smpl_runtime import GroundReference,STANDING_FOREHEAD_HEIGHT_M
from egohand.body.smpl_mount import (HEAD_ANCHOR_VERSION, DEFAULT_CAMERA_FOREHEAD_OFFSET_MM,
                                    camera_forehead_offset_m, forehead_target)

from egohand.body.smpl_binding import attach_mano,rejection_reasons,mesh_asset


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('output',type=Path)
    ap.add_argument('--indices',help='comma-separated diagnostic frame indices; default full sequence')
    ap.add_argument('--iterations',type=int)
    ap.add_argument('--ground-y-m',type=float,help='Explicit floor Y in body world; otherwise infer per segment')
    ap.add_argument('--initial-camera-height-m',type=float,default=STANDING_FOREHEAD_HEIGHT_M,help='Camera-to-floor distance at segment initialization (default: standing 1.75 m SMPL forehead)')
    ap.add_argument('--camera-forehead-offset-mm',type=float,default=DEFAULT_CAMERA_FOREHEAD_OFFSET_MM)
    ap.add_argument('--in-place',action='store_true',help='Add only new derived artifacts to a newly computed result directory')
    ap.add_argument('--mano',action='store_true',help='Fit missing MANO meshes using the existing configured fitter')
    ap.add_argument('--strict-body',action='store_true',help='Only publish body on frames with head pose and both observed wrists')
    args=ap.parse_args()
    source=args.source.resolve();dest=args.output.resolve()
    if args.in_place:
        if dest!=source:raise ValueError('--in-place requires identical source and output')
        for name in ['derived_pose.jsonl','smpl_vertices.npy','smpl_pose.npz','smpl_mesh.json','unified_smpl_manifest.json']:
            if (dest/name).exists():raise FileExistsError('Preserve existing derived artifact: '+name)
    elif dest.exists():raise FileExistsError('Preserve existing candidate; choose a new output directory')
    else:dest.mkdir(parents=True)
    results,truncated=load_recorded_results(source)
    if truncated:raise ValueError('Input results truncated')
    # Read-only inputs may be hardlinked; none of these are opened for writes.
    if not args.in_place:
        (dest/'results').mkdir()
        for name in ['results/frames.jsonl','processing_calibration.yaml','reprocessing.json','display_cache.sqlite3','depth_results.sqlite3']:
            if (source/name).is_file():os.link(source/name,dest/name)
    offset_m=camera_forehead_offset_m(args.camera_forehead_offset_mm)
    smpl=SmplBody();solver=UnifiedSmpl(smpl.model)
    n=len(results)
    verts=np.lib.format.open_memmap(dest/'smpl_vertices.npy',mode='w+',dtype=np.float32,shape=(n,6890,3))
    verts[:]=0
    poses=np.zeros((n,69),np.float32);orient=np.zeros((n,3),np.float32);trans=np.zeros((n,3),np.float32)
    allj=np.zeros((n,24,3),np.float32);valid=np.zeros(n,bool)
    allr=np.zeros((n,24,3,3),np.float32)
    selected=set(map(int,args.indices.split(','))) if args.indices else None
    started=time.monotonic();reports=[];rows=[];mano_engine=None
    floor_reference=GroundReference(args.initial_camera_height_m) if args.initial_camera_height_m is not None else None
    for i,r in enumerate(results):
        row={'frame_id':r.frame_id,'timestamp_ns':r.timestamp_ns,'body':None,'mano':{'predicted':[]},'unified_smpl':True,
            'head_anchor_version':HEAD_ANCHOR_VERSION}
        if selected is not None and i not in selected:rows.append(row);continue
        T=consumer_result_display_pose(r)
        if T is None:rows.append(row);solver.previous=None;continue
        wrists={};directions={}
        for hand in r.hands:
            if hand.handedness in ('Left','Right') and hand.joints[0].valid:
                p=np.asarray(hand.joints[0].xyz_m)
                if np.isfinite(p).all():wrists[hand.handedness.lower()]=T[:3,:3]@p+T[:3,3]
                if hand.joints[9].valid:
                    d=np.asarray(hand.joints[9].xyz_m)-p
                    if np.isfinite(d).all() and np.linalg.norm(d)>.02:
                        directions[hand.handedness.lower()]=T[:3,:3]@(d/np.linalg.norm(d))
        ref=display_reference_key(r.diagnostics.get('vio_pose') or r.diagnostics.get('live_vio_pose') or {})
        ground=args.ground_y_m if args.ground_y_m is not None else floor_reference.floor_for(r) if floor_reference else None
        target=forehead_target(T,offset_m)
        fit=solver.fit(target,T[:3,:3],wrists,ref,iterations=args.iterations,
                       ground_y_m=ground,hand_directions=directions)
        reasons=rejection_reasons(fit)
        if reasons:
            fit=solver.fit(target,T[:3,:3],wrists,ref,iterations=max(args.iterations or 0,100),
                           ground_y_m=ground,hand_directions=directions)
            reasons=rejection_reasons(fit)
        if reasons:
            row['body_unavailable_reason']=reasons;rows.append(row)
            reports.append({'index':i,'frame_id':r.frame_id,'accepted':False,'reasons':reasons,
                'foot_contact_errors_m':fit['foot_contact_errors_m'],'wrist_errors_m':fit['wrist_errors_m'],
                'head_local_angle_deg':fit['head_local_angle_deg']})
            solver.previous=None
            continue
        verts[i]=fit['vertices'];allj[i]=fit['joints'];allr[i]=fit['rotations'];valid[i]=True
        poses[i]=fit['body_pose'];orient[i]=fit['global_orient'];trans[i]=fit['transl']
        diag={'source':'single_pose_smpl','model':'SMPL_NEUTRAL','unobserved_body':'conservative_pose_prior',
            'head_rotation_world':T[:3,:3].tolist(),'ground_y_m':fit['ground_y_m'],
            'head_anchor':'camera_mount_offset_optical_y','head_anchor_vertex':fit['head_anchor_vertex'],
            'camera_forehead_offset_mm':args.camera_forehead_offset_mm,
            'head_anchor_version':HEAD_ANCHOR_VERSION,
            'head_error_m':fit['head_error_m'],'wrist_errors_m':fit['wrist_errors_m'],
            'foot_contact_errors_m':fit['foot_contact_errors_m'],'head_angle_error_deg':fit['head_angle_error_deg'],
            'head_local_angle_deg':fit['head_local_angle_deg'],
            'bounds_source':'application_anatomical_prior_not_official_smpl_limits',
            'body_mesh_index':i,'body_mesh_coordinate_frame':'body_world',
            'body_mesh_vertices_path':str(dest/'smpl_vertices.npy'),'unified_smpl':True,
            'smpl_wrist_rotations_world':{side:fit['rotations'][j].tolist() for side,j in [('left',20),('right',21)]}}
        body={'frame_id':r.frame_id,'timestamp_ns':r.timestamp_ns,'reference_key':ref,
            'status':'estimated_preview','joints':dict(zip(SMPL_JOINT_NAMES,fit['joints'].tolist())),
            'bones':[[SMPL_JOINT_NAMES[p],SMPL_JOINT_NAMES[j]] for j,p in enumerate(SMPL_PARENT_IDS) if p>=0],
            'measured_joints':[],'diagnostics':diag}
        previous=(r.diagnostics.get('offline_derived_pose') or {}).get('mano') or {}
        observed_meshes=previous.get('predicted',[])
        if not observed_meshes and args.mano and r.hands:
            if mano_engine is None:
                from mano_worker import Engine
                import yaml
                cfg=yaml.safe_load((ROOT/'configs/mano.yaml').read_text(encoding='utf8'))
                mano_engine=Engine(ROOT/cfg['models'],cfg)
            hand_inputs=[]
            for h in r.hands:
                points=np.asarray([j.xyz_m for j in h.joints],float)
                mask=np.asarray([j.valid for j in h.joints],bool)&np.isfinite(points).all(axis=1)
                hand_inputs.append({'side':h.handedness,'track_id':h.track_id,
                    'points':np.where(mask[:,None],points,0.).tolist(),'valid':mask.tolist()})
            observed_meshes=mano_engine.run({'frame':r.frame_id,'sequence':repr(ref),'prediction':True,'hands':hand_inputs})['predicted']
        meshes=attach_mano(observed_meshes,T,fit)
        diag['mano_attached_sides']=[m['side'] for m in meshes]
        row.update(body=body,mano={'predicted':meshes,'coordinate_frame':'body_world','binding':'smpl_wrist_local'},
            body_mesh_ref='smpl_mesh.json')
        if args.strict_body and not {'left','right'}.issubset(wrists):
            row['body']=None;row['body_unavailable_reason']=['strict_body_missing_wrist']
            row.pop('body_mesh_ref',None);valid[i]=False
        reports.append({'index':i,'frame_id':r.frame_id,'accepted':bool(valid[i]),
            'reasons':row.get('body_unavailable_reason',[]),'head_error_m':fit['head_error_m'],
            'wrist_errors_m':fit['wrist_errors_m'],'mano_meshes':len(meshes),
            'foot_contact_errors_m':fit['foot_contact_errors_m'],'head_angle_error_deg':fit['head_angle_error_deg'],
            'head_local_angle_deg':fit['head_local_angle_deg']})
        rows.append(row)
        if (i+1)%100==0:print(f'{i+1}/{n} valid={valid.sum()} elapsed={time.monotonic()-started:.1f}s',flush=True)
    verts.flush();del verts
    np.savez_compressed(dest/'smpl_pose.npz',body_pose=poses,global_orient=orient,transl=trans,betas=np.repeat(solver.betas.numpy(),n,axis=0),
        joints_world=allj,rotations_world=allr,valid=valid,frame_id=np.array([r.frame_id for r in results]),
        timestamp_ns=np.array([r.timestamp_ns for r in results]),world_to_smpl=SMPL_FROM_BODY)
    (dest/'smpl_mesh.json').write_text(json.dumps({**mesh_asset(smpl,solver),'vertices':[[0.,0.,0.]]}),encoding='utf8')
    with (dest/'derived_pose.jsonl').open('x',encoding='utf8') as f:
        for row in rows:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    (dest/'fit_report.json').write_text(json.dumps(reports,indent=2),encoding='utf8')
    model_path=ROOT/'models/smpl/SMPL_NEUTRAL.pkl'
    manifest={'schema_version':1,'status':'completed','source':str(source),'frames':n,'valid':int(valid.sum()),
        'model':'SMPL_NEUTRAL','model_sha256':hashlib.sha256(model_path.read_bytes()).hexdigest(),
        'joint_tree':list(SMPL_PARENT_IDS),'joint_names':list(SMPL_JOINT_NAMES),
        'vertices_coordinate_frame':'body_world','world_to_smpl':SMPL_FROM_BODY.tolist(),
        'surface_and_joints':'same_smpl_forward_evaluation','custom_skeleton_targets':False,
        'pose_limits_source':'application_prior_not_official','head_mount':'camera_mount_offset_optical_y',
        'camera_forehead_offset_mm':args.camera_forehead_offset_mm,
        'floor':'local_initial_foot_height_prior','head_anchor_vertex':solver.head_vertex,
        'head_anchor_version':HEAD_ANCHOR_VERSION,
        'vertices_file':'smpl_vertices.npy','pose_file':'smpl_pose.npz','mesh_file':'smpl_mesh.json',
        'explicit_ground_y_m':args.ground_y_m,
        'initial_camera_height_m':args.initial_camera_height_m,
        'strict_body':args.strict_body,
        'quality_limits':{'foot_contact_abs_m':.012,'wrist_error_m':.10,'head_local_angle_deg':85.},
        'body_shape':{'height_target_m':1.75,'betas':solver.betas.numpy()[0].tolist(),
            'individual_bone_lengths_fixed':False,'demographic_shape':'neutral_asset_approximation'},
        'source_results_sha256':hashlib.sha256((source/'results/frames.jsonl').read_bytes()).hexdigest(),
        'mean_template_height_m':solver.rest_height,'elapsed_s':time.monotonic()-started}
    (dest/'unified_smpl_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    print(json.dumps(manifest,indent=2),flush=True)

if __name__=='__main__':main()
