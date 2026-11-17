from pathlib import Path
import sys, numpy as np, torch
sys.path.append(str(Path(__file__).resolve().parents[1]/'tools'/'mano_deps'))
from smpl_body_mesh import SmplBody
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path('models/smpl')
body=SmplBody(); m=body.model
with torch.no_grad():
 out=m(global_orient=torch.zeros((1,3)),body_pose=torch.zeros((1,69)),betas=torch.zeros((1,10)),transl=torch.zeros((1,3)))
np.savez_compressed(root/'body_mesh_smpl.npz',vertices=out.vertices[0].numpy(),joints=out.joints[0].numpy(),faces=m.faces.astype(np.int32),weights=m.lbs_weights.detach().numpy())
print(root/'body_mesh_smpl.npz')
