"""Generate a real SMPL surface mesh from a cached body pose.

The current sparse body estimator supplies translation and height; the SMPL
neutral surface is kept as a real 6890-vertex mesh. Pose fitting can be
extended without changing the sidecar format.
"""
from pathlib import Path
import sys
import inspect, collections
if not hasattr(inspect,'getargspec'):
    _ArgSpec=collections.namedtuple('ArgSpec','args varargs keywords defaults')
    def _getargspec(fn):
        v=inspect.getfullargspec(fn); return _ArgSpec(v.args,v.varargs,v.varkw,v.defaults)
    inspect.getargspec=_getargspec
import numpy as np
for _name,_value in {'bool':bool,'int':int,'float':float,'complex':complex,'object':object,'unicode':str,'str':str}.items():
    if not hasattr(np,_name): setattr(np,_name,_value)
import torch
sys.path.append(str(Path(__file__).resolve().parents[1]/'tools'/'mano_deps'))
import smplx

ROOT=Path(__file__).resolve().parents[1]
MODEL=ROOT/'models'/'smpl'

class SmplBody:
    def __init__(self):
        self.model=smplx.create(str(MODEL),model_type='smpl',gender='neutral',num_betas=10,
                                use_pca=False,ext='pkl',batch_size=1)
        self.faces=self.model.faces.astype(np.int32).tolist()
        with torch.no_grad():
            out=self.model(global_orient=torch.zeros((1,3)),body_pose=torch.zeros((1,69)),
                           betas=torch.zeros((1,10)),transl=torch.zeros((1,3)))
        v=out.vertices[0].detach().cpu().numpy(); self.base_height=float(v[:,1].max()-v[:,1].min())
    def mesh(self, body):
        joints=body.get('joints',{})
        pelvis=np.asarray(joints.get('Pelvis',[0,0,0]),float)
        height=float((body.get('diagnostics') or {}).get('height_m',1.75) or 1.75)
        scale=height/max(self.base_height,1e-6)
        with torch.no_grad():
            out=self.model(global_orient=torch.zeros((1,3)),body_pose=torch.zeros((1,69)),
                           betas=torch.zeros((1,10)),transl=torch.tensor((pelvis*scale)[None,:],dtype=torch.float32))
        vertices=(out.vertices[0].detach().cpu().numpy()*scale).tolist()
        return {'vertices':vertices,'faces':self.faces,'model':'SMPL_NEUTRAL','scale':scale}
