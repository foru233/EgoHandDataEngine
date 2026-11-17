"""Build replayable offline body/MANO derived results beside a recording.

The original frame results remain untouched. Body pose is computed from the
same estimator used by the GUI and stored as JSONL keyed by frame identity.
MANO is represented by the already triangulated 21-point hand input; an
optional MANO worker can be run later without changing the replay contract.
"""
from __future__ import annotations
import argparse, json
import subprocess
from pathlib import Path
import numpy as np

from egohand.storage.replay import load_recorded_results
from egohand.body.body_pose_estimator import BodyPoseEstimator
from egohand.core.types import serializable

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('recording', type=Path)
    ap.add_argument('--output', type=Path)
    ap.add_argument('--mano', action='store_true', help='run the configured MANO fitter')
    args=ap.parse_args()
    root=args.recording.resolve(); out=(args.output or root).resolve(); out.mkdir(parents=True,exist_ok=True)
    results,truncated=load_recorded_results(root)
    est=BodyPoseEstimator(); rows=[]; valid=0; mano_proc=None
    if args.mano:
        import yaml
        cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs'/'mano.yaml').read_text(encoding='utf8'))
        py=Path(cfg['python'])
        if not py.is_file(): raise FileNotFoundError(f'MANO Python 不存在: {py}')
        dep=str(Path(__file__).resolve().parents[1]/'tools'/'mano_deps').replace('\\','/')
        probe=subprocess.run([str(py),'-c',f"import sys;sys.path.append(r'{dep}');import torch, smplx"],cwd=str(Path(__file__).resolve().parents[1]),
                             stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
        if probe.returncode: raise RuntimeError('MANO 环境缺少 torch/smplx，未生成伪造网格结果')
        mano_proc=subprocess.Popen([str(py),'-u',str(Path(__file__).resolve().parent/'mano_worker.py')],
            cwd=str(Path(__file__).resolve().parents[1]),stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,text=True,bufsize=1)
    for r in results:
        pose=r.diagnostics.get('vio_pose') or r.diagnostics.get('live_vio_pose')
        estimate,gate=est.update(r,pose,strict_gate=False,now_ns=r.timestamp_ns)
        if estimate is not None:
            valid+=1
            body={'frame_id':estimate.frame_id,'timestamp_ns':estimate.timestamp_ns,
                  'reference_key':estimate.reference_key,'status':estimate.status,
                  'joints':{k:np.asarray(v).tolist() for k,v in estimate.joints.items()},
                  'measured_joints':sorted(estimate.measured_joints),
                  'bones':[list(x) for x in estimate.bones],
                  'diagnostics':serializable(estimate.diagnostics)}
        else: body=None
        hands=[]
        for h in r.hands:
            points=[j.xyz_m.tolist() if j.valid and np.isfinite(j.xyz_m).all() else None for j in h.joints]
            hands.append({'side':h.handedness,'track_id':h.track_id,'points':points,
                          'valid_count':h.valid_joint_count})
        mano={'source':'triangulated_hand_21','hands':hands,'status':'ready' if hands else 'empty'}
        if mano_proc is not None and hands:
            req={'sequence':r.session_id or str(root),'frame':r.frame_id,'prediction':True,'ground_truth':False,
                 'request_id':f'{r.frame_id}:{r.timestamp_ns}','hands':[{'side':h['side'],'track_id':h['track_id'],
                 'points':h['points'],'valid':[p is not None for p in h['points']]} for h in hands]}
            mano_proc.stdin.write(json.dumps(req,ensure_ascii=False)+'\n');mano_proc.stdin.flush()
            reply=json.loads(mano_proc.stdout.readline())
            mano.update({'source':'mano_worker','predicted':reply.get('predicted',[]),
                         'errors':reply.get('errors',[]),'status':'ready' if reply.get('predicted') else 'error'})
        rows.append({'frame_id':r.frame_id,'timestamp_ns':r.timestamp_ns,'body':body,'mano':mano})
    if mano_proc is not None:
        mano_proc.stdin.close();mano_proc.terminate();mano_proc.wait(timeout=5)
    target=out/'derived_pose.jsonl'; tmp=target.with_suffix('.tmp')
    with tmp.open('w',encoding='utf8') as f:
        for row in rows:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    tmp.replace(target)
    manifest={'schema_version':1,'source_recording':str(root),'result_count':len(results),
              'body_valid_count':valid,'truncated_input':truncated,'mano_source':'triangulated_hand_21',
              'note':'MANO worker 可在此文件基础上异步替换 mano 字段；原始结果未修改'}
    (out/'derived_pose_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(manifest,ensure_ascii=False))

if __name__=='__main__':main()
