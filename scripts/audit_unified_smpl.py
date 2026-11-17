"""Reconstruct exported poses with the official model; audit the replay contract."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import torch
torch.set_num_threads(1)
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from smpl_body_mesh import SmplBody
from egohand.body.smpl_mapping import SMPL_FROM_BODY as C,SMPL_PARENT_IDS
from egohand.storage.replay import load_recorded_results
from egohand.vio.hand_pose_binding import consumer_result_display_pose


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    args=parser.parse_args();root=args.directory.resolve()
    manifest=json.loads((root/'unified_smpl_manifest.json').read_text(encoding='utf8'))
    params=np.load(root/manifest.get('pose_file','smpl_pose.npz'));verts=np.load(root/manifest.get('vertices_file','smpl_vertices.npy'),mmap_mode='r')
    rows=[json.loads(l) for l in (root/'derived_pose.jsonl').open(encoding='utf8')]
    results,truncated=load_recorded_results(root);assert not truncated
    model=SmplBody().model
    assert tuple(model.parents.tolist())==SMPL_PARENT_IDS
    assert np.isclose(np.linalg.det(C),1.)
    assert hashlib.sha256((root/'results/frames.jsonl').read_bytes()).hexdigest()==hashlib.sha256(
        (Path(manifest['source'])/'results/frames.jsonl').read_bytes()).hexdigest()
    metrics={k:[] for k in ['vertices_rebuild_m','joints_rebuild_m','head_anchor_m','head_angle_deg',
        'foot_abs_m','wrist_m','head_local_deg','mano_wrist_m','body_step_m']}
    failed=[];last=None;last_ref=None
    for i in np.flatnonzero(params['valid']):
        p={key:torch.tensor(params[key][i:i+1]) for key in ['body_pose','global_orient','transl','betas']}
        with torch.no_grad():out=model(**p)
        v=out.vertices[0].numpy()@C;j=out.joints[0,:24].numpy()@C
        metrics['vertices_rebuild_m'].append(float(np.max(np.linalg.norm(v-verts[i],axis=1))))
        metrics['joints_rebuild_m'].append(float(np.max(np.linalg.norm(j-params['joints_world'][i],axis=1))))
        row=rows[i];body=row['body'];diag=body['diagnostics'];T=consumer_result_display_pose(results[i])
        metrics['head_anchor_m'].append(float(np.linalg.norm(v[manifest['head_anchor_vertex']]-T[:3,3])))
        metrics['head_angle_deg'].append(diag['head_angle_error_deg'])
        metrics['foot_abs_m'].extend(map(abs,diag['foot_contact_errors_m']))
        metrics['wrist_m'].extend(diag['wrist_errors_m'].values())
        metrics['head_local_deg'].append(diag['head_local_angle_deg'])
        assert len(body['joints'])==24 and len(body['bones'])==23
        np.testing.assert_allclose(np.array(list(body['joints'].values())),j,atol=2e-5)
        a=params['body_pose'][i].reshape(23,3)
        if not ((a[[3,4],0]>=-1e-5).all() and (a[[3,4],0]<=2.65001).all()):failed.append(int(i))
        for m in row['mano']['predicted']:
            wr=np.asarray(m['joints20'])[0];parent=m['smpl_parent_joint']
            metrics['mano_wrist_m'].append(float(np.linalg.norm(wr-j[parent])))
            local=np.asarray(m['vertices_wrist_local']);M=np.asarray(m['T_world_from_smpl_wrist'])
            np.testing.assert_allclose(local@M[:3,:3].T+M[:3,3],m['vertices'],atol=1e-6)
        if last is not None and i==last+1 and last_ref==body['reference_key']:
            metrics['body_step_m'].append(float(np.max(np.linalg.norm(j-params['joints_world'][last],axis=1))))
        last=i;last_ref=body['reference_key']
    summary={k:{'n':len(v),'p50':float(np.median(v)) if v else None,
        'p95':float(np.percentile(v,95)) if v else None,'max':max(v) if v else None} for k,v in metrics.items()}
    passed=(not failed and summary['vertices_rebuild_m']['max']<2e-5 and
        summary['joints_rebuild_m']['max']<2e-5 and summary['head_anchor_m']['max']<2e-5 and
        summary['foot_abs_m']['max']<=.012001 and summary['head_local_deg']['max']<=85.001 and
        (summary['wrist_m']['max'] or 0)<=.100001 and (summary['mano_wrist_m']['max'] or 0)<2e-5)
    report={'passed':passed,'frames':len(rows),'valid':int(params['valid'].sum()),'knee_failures':failed,
        'metrics':summary,'invalid_reasons':{},'note':'Sparse body is inferred; no full-body ground truth.'}
    for row in rows:
        if row['body'] is None:
            key=','.join(row.get('body_unavailable_reason',['missing_visual_camera_pose']))
            report['invalid_reasons'][key]=report['invalid_reasons'].get(key,0)+1
    (root/'audit_report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2));assert passed,report


if __name__=='__main__':main()
