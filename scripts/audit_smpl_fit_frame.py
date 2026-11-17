"""Compare isolated fits without modifying any replay data."""
import json,sys
from pathlib import Path
import numpy as np
import torch
from smpl_body_mesh import SmplBody
from fit_smpl_sequence import MAP

torch.set_num_threads(1)
root=Path(sys.argv[1]); frame=int(sys.argv[2]); row=None
with (root/'derived_pose.jsonl').open(encoding='utf8') as stream:
    for i,line in enumerate(stream):
        if i==frame: row=json.loads(line); break
if row is None or not row.get('body'): raise ValueError('missing body')
joints=row['body']['joints']; names=[k for k in MAP if k in joints]; ids=torch.tensor([MAP[k] for k in names])
target=torch.tensor([[-joints[k][0],-joints[k][1],joints[k][2]] for k in names],dtype=torch.float32)
model=SmplBody().model
for weight in (1.,5.,15.):
    orient=torch.nn.Parameter(torch.zeros((1,3))); pose=torch.nn.Parameter(torch.zeros((1,69))); trans=torch.nn.Parameter(target[names.index('Pelvis')].reshape(1,3).clone()); betas=torch.nn.Parameter(torch.zeros((1,10)))
    opt=torch.optim.Adam([orient,pose,trans,betas],lr=.025)
    weights=torch.tensor([weight if 'Wrist' in k else 1. for k in names])
    for _ in range(200):
        opt.zero_grad(); out=model(global_orient=orient,body_pose=pose,betas=betas,transl=trans)
        loss=(((out.joints[0,ids]-target)**2).sum(dim=1)*weights).mean()+1e-4*pose.square().mean()+1e-4*betas.square().mean(); loss.backward(); opt.step()
        with torch.no_grad(): betas.clamp_(-3,3)
    with torch.no_grad(): out=model(global_orient=orient,body_pose=pose,betas=betas,transl=trans); errors=torch.linalg.vector_norm(out.joints[0,ids]-target,dim=1).numpy()
    print(json.dumps({'wrist_weight':weight,'mean_m':float(errors.mean()),'errors_m':dict(zip(names,map(float,errors)))},ensure_ascii=False),flush=True)
