"""Persistent JSON-line SMPL solver. Model computation stays outside Qt."""
import contextlib
import json
import sys
import time
from pathlib import Path
import numpy as np
import torch
torch.set_num_threads(1)
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from smpl_body_mesh import SmplBody
from egohand.body.smpl_unified import UnifiedSmpl
from egohand.body.smpl_binding import attach_mano,rejection_reasons,mesh_asset
from egohand.body.smpl_mount import HEAD_ANCHOR_VERSION


def emit(value):print(json.dumps(value,allow_nan=False,separators=(',',':')),flush=True)


def main():
    with contextlib.redirect_stdout(sys.stderr):
        smpl=SmplBody();solver=UnifiedSmpl(smpl.model);asset=mesh_asset(smpl,solver)
    emit({'type':'ready','asset':asset,'height_m':solver.rest_height})
    cache_path=None;cache=None;cache_vertices=None;seed_key=None;mano_engine=None;seed_compatible=False
    for line in sys.stdin:
        req={}
        try:
            req=json.loads(line);started=time.perf_counter()
            path=req.get('seed_vertices_path');index=req.get('seed_index')
            if path and type(index) is int and (path,index)!=seed_key:
                seed_key=(path,index)
                p=Path(path).with_name('smpl_pose.npz')
                if p.is_file():
                    if cache_path!=p:
                        if cache is not None:cache.close()
                        cache=np.load(p);cache_path=p;cache_vertices=np.load(path,mmap_mode='r')
                        manifest=p.with_name('unified_smpl_manifest.json')
                        seed_compatible=(manifest.is_file() and json.loads(manifest.read_text(encoding='utf8')).get('head_anchor_version')==HEAD_ANCHOR_VERSION)
                    if cache['valid'][index] and seed_compatible:
                        source_floor=float(np.mean([cache_vertices[index,ids.numpy(),1].max() for ids in solver.feet]))
                        solver.previous=(tuple(torch.tensor(cache[k][index:index+1]) for k in ['global_orient','body_pose','transl'])
                            if abs(source_floor-req['ground_y_m'])<.03 else None)
                        solver.reference=req['reference']
                        solver.previous_joints=None;solver.previous_head=None
            if solver.ground is not None and abs(-solver.ground-req['ground_y_m'])>.03:
                # A large user floor change is a new constrained solve, not
                # a temporal motion observation. Seed a squat at this height.
                solver.previous=None;solver.previous_joints=None;solver.previous_head=None
            fast=bool(req.get('streaming'))
            fit=solver.fit(req['head_world'],req['head_rotation'],req['wrists'],req['reference'],
                ground_y_m=req['ground_y_m'],hand_directions=req.get('hand_directions'),iterations=(8 if solver.previous is not None else 100) if fast else 70)
            reasons=rejection_reasons(fit)
            if reasons:
                fit=solver.fit(req['head_world'],req['head_rotation'],req['wrists'],req['reference'],
                    ground_y_m=req['ground_y_m'],hand_directions=req.get('hand_directions'),iterations=140)
                reasons=rejection_reasons(fit)
            reply={k:req[k] for k in ['request_id','frame_id','timestamp_ns','reference','head_rotation']}
            reply['stream_session']=req.get('stream_session')
            reply.update(type='result',accepted=not reasons,reasons=reasons,elapsed_ms=(time.perf_counter()-started)*1000.,
                residuals={k:fit[k] for k in ['foot_contact_errors_m','wrist_errors_m','head_local_angle_deg']})
            if not reasons:
                meshes=req.get('mano',[])
                if not meshes and req.get('hands'):
                    if mano_engine is None:
                        from mano_worker import Engine
                        import yaml
                        cfg=yaml.safe_load((ROOT/'configs/mano.yaml').read_text(encoding='utf8'))
                        mano_engine=Engine(ROOT/cfg['models'],{**cfg,'iterations':20 if fast else cfg['iterations']})
                    meshes=mano_engine.run({'frame':req['frame_id'],'sequence':repr(req['reference']),
                        'prediction':True,'hands':req['hands']})['predicted']
                reply['mano']=attach_mano(meshes,np.asarray(req['camera_transform']),fit)
                reply['fit']={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in fit.items()}
            else:solver.previous=None
            emit(reply)
        except Exception as exc:
            emit({'type':'error','request_id':req.get('request_id'),'error':str(exc)})


if __name__=='__main__':main()
