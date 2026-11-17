"""Isolated consecutive-frame fitting audit; never changes a replay cache."""
import argparse,json
from pathlib import Path
import numpy as np
import torch
from smpl_body_mesh import SmplBody
from fit_smpl_sequence import MAP

torch.set_num_threads(1)
ap=argparse.ArgumentParser();ap.add_argument('root',type=Path);ap.add_argument('--start',type=int,default=1586);ap.add_argument('--count',type=int,default=16);ap.add_argument('--temporal-weight',type=float,default=1e-3);ap.add_argument('--vertex-weight',type=float,default=0.);ap.add_argument('--head-weight',type=float,default=1.);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--vertices-output',type=Path);args=ap.parse_args()
rows=[]
with (args.root/'derived_pose.jsonl').open(encoding='utf8') as src:
    for i,line in enumerate(src):
        if i>=args.start+args.count:break
        if i>=args.start:rows.append(json.loads(line))
model=SmplBody().model
orient=torch.nn.Parameter(torch.zeros((1,3)));pose=torch.nn.Parameter(torch.zeros((1,69)));trans=torch.nn.Parameter(torch.zeros((1,3)));betas=torch.nn.Parameter(torch.zeros((1,10)))
report=[];last_vertices=None;last_target=None; saved_vertices=[];last_reference=None
for n,row in enumerate(rows):
    b=row.get('body');
    if not b: report.append({'index':args.start+n,'valid':False});saved_vertices.append(np.full((6890,3),np.nan,dtype=np.float32));last_vertices=None;last_target=None;last_reference=None;continue
    reference=tuple(b.get('reference_key') or ())
    restarting=last_vertices is None or reference!=last_reference
    if restarting:
        with torch.no_grad(): orient.zero_();pose.zero_();trans.zero_();betas.zero_()
        last_vertices=None;last_target=None
    j=b['joints']; names=[k for k in MAP if k in j]; ids=torch.tensor([MAP[k] for k in names])
    target=torch.tensor([[-j[k][0],-j[k][1],j[k][2]] for k in names],dtype=torch.float32)
    weights=torch.tensor([args.head_weight if k=='Head' else 1. for k in names])
    if restarting:
        with torch.no_grad():trans.copy_(target[names.index('Pelvis')].reshape(1,3))
    prev=pose.detach().clone(); params=[orient,pose,trans]+([betas] if restarting else [])
    opt=torch.optim.Adam(params,lr=.025 if restarting else .008)
    for _ in range(200 if restarting else 35):
        opt.zero_grad();out=model(global_orient=orient,body_pose=pose,betas=betas,transl=trans)
        loss=(((out.joints[0,ids]-target)**2).sum(dim=1)*weights).mean()+1e-4*pose.square().mean()+1e-4*betas.square().mean()
        if not restarting:loss=loss+args.temporal_weight*(pose-prev).square().mean()
        if last_vertices is not None and args.vertex_weight:
            # Compensate for target pelvis translation so regularization does
            # not incorrectly resist global walking motion.
            shift=target[names.index('Pelvis')]-torch.as_tensor(last_target[names.index('Pelvis')])
            prior=torch.as_tensor(last_vertices)+shift
            loss=loss+args.vertex_weight*(out.vertices[0]-prior).square().mean()
        loss.backward();opt.step()
        with torch.no_grad():betas.clamp_(-3,3)
    with torch.no_grad():
        out=model(global_orient=orient,body_pose=pose,betas=betas,transl=trans)
        v=out.vertices[0].numpy().copy();errors=torch.linalg.vector_norm(out.joints[0,ids]-target,dim=1).numpy()
        # Experiment fits a proper 180-degree rotation around Z. Store
        # candidate vertices in BodyPose's Y-down coordinate convention.
        saved_vertices.append(v*np.array([-1.,-1.,1.],dtype=np.float32))
        changes=np.linalg.norm(v-last_vertices,axis=1) if last_vertices is not None else None
        motion=np.linalg.norm(target.numpy()-last_target,axis=1) if last_target is not None else None
        report.append({'index':args.start+n,'frame_id':row['frame_id'],'valid':True,'mean_joint_error_m':float(errors.mean()),'head_error_m':float(errors[names.index('Head')]) if 'Head' in names else None,'right_wrist_error_m':float(errors[names.index('RightWrist')]) if 'RightWrist' in names else None,'vertex_step_p95_m':float(np.percentile(changes,95)) if changes is not None else None,'target_step_p95_m':float(np.percentile(motion,95)) if motion is not None else None})
        last_vertices=v;last_target=target.numpy().copy();last_reference=reference
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(report,ensure_ascii=False))
if args.vertices_output:
    args.vertices_output.parent.mkdir(parents=True,exist_ok=True)
    np.save(args.vertices_output,np.asarray(saved_vertices,dtype=np.float32),allow_pickle=False)
