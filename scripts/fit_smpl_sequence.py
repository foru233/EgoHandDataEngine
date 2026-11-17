from pathlib import Path
import json,sys,numpy as np,torch
torch.set_num_threads(1)
sys.path.append(str(Path(__file__).resolve().parents[1]/'tools'/'mano_deps'))
from smpl_body_mesh import SmplBody
from egohand.body.smpl_mapping import SMPL_INDEX, SMPL_PARENTS, SMPL_FROM_BODY, mapped_targets, rest_bone_lengths

ROOT=Path(__file__).resolve().parents[1]
MAP=SMPL_INDEX

def main(root):
 root=Path(root); lines=(root/'derived_pose.jsonl').read_text(encoding='utf8').splitlines(); body=SmplBody(); m=body.model
 rest=rest_bone_lengths(m)
 # Write to separate files until all frames complete, so a failed job cannot
 # truncate a previously usable cache.  The manifest is published last.
 tmp=root/'body_vertices.build.npy'; valid_tmp=root/'body_vertices_valid.build.npy'
 verts=np.lib.format.open_memmap(tmp,mode='w+',dtype='float16',shape=(len(lines),6890,3))
 valid_rows=np.lib.format.open_memmap(valid_tmp,mode='w+',dtype='bool',shape=(len(lines),))
 valid_rows[:]=False
 prev_pose=torch.zeros((1,69));prev_orient=torch.zeros((1,3)); prev_trans=torch.zeros((1,3)); valid=0
 for i,line in enumerate(lines):
  row=json.loads(line); b=row.get('body');
  if not b: continue
  targets=mapped_targets(b.get('joints',{})); indices=list(targets); target=np.asarray([targets[k] for k in indices])
  if len(target)<6:continue
  target=torch.tensor(target,dtype=torch.float32)
  orient=torch.nn.Parameter(prev_orient.clone()); pose=torch.nn.Parameter(prev_pose.clone()); trans=torch.nn.Parameter(prev_trans.clone())
  opt=torch.optim.Adam([orient,pose,trans],lr=.04)
  idx=torch.tensor(indices,dtype=torch.long)
  for _ in range(24):
   opt.zero_grad(); out=m(global_orient=orient,body_pose=pose,betas=torch.zeros((1,10)),transl=trans)
   loss=((out.joints[0,idx]-target)**2).mean()
   # Fit directions and root translation while SMPL's own bone lengths stay
   # authoritative; this prevents the optimizer from stretching limbs.
   for child,parent in SMPL_PARENTS.items():
    if child in indices and parent in indices:
     a=out.joints[0,child]-out.joints[0,parent]; bvec=target[indices.index(child)]-target[indices.index(parent)]
     loss += .25*(1.-torch.nn.functional.cosine_similarity(a[None],bvec[None]).mean())
     loss += .5*(torch.linalg.vector_norm(a)-rest[child])**2
   # The torso is canonical upright.  Do not let sparse head/hand targets
   # introduce a global or spine twist into the SMPL waist.
   loss += .25*orient.square().mean()
   for start in (6, 15, 24):  # SMPL spine1/spine2/spine3 axis-angle blocks
    loss += .25*pose[:, start:start+3].square().mean()
   # Sparse head/hand observations leave the waist underconstrained.  Keep
   # the SMPL shoulder and hip belts facing the mapped detector belts instead
   # of allowing an arbitrary axial twist through the pelvis.
   for left,right in ((16,17),(1,2)):
    if left in indices and right in indices:
     model_axis=out.joints[0,right]-out.joints[0,left]
     target_axis=target[indices.index(right)]-target[indices.index(left)]
     loss += .35*(1.-torch.nn.functional.cosine_similarity(model_axis[None],target_axis[None]).mean())
   loss=loss+1e-4*pose.square().mean()+1e-4*orient.square().mean(); loss.backward();opt.step()
  with torch.no_grad(): out=m(global_orient=orient,body_pose=pose,betas=torch.zeros((1,10)),transl=trans)
  # Legacy readers decode by flipping Y only. First undo the proper fitting
  # rotation, then encode that legacy storage convention explicitly.
  array=(out.vertices[0].cpu().numpy()@SMPL_FROM_BODY)*np.array([1.,-1.,1.])
  if not np.isfinite(array).all():continue
  verts[i]=array.astype('float16');valid_rows[i]=True; prev_pose=pose.detach();prev_orient=orient.detach();prev_trans=trans.detach();valid+=1
 verts.flush();valid_rows.flush();del verts,valid_rows
 # Existing readers on Windows may hold the prior cache open; fail without
 # destroying the old files if the final rename cannot be performed.
 tmp.replace(root/'body_vertices.npy');valid_tmp.replace(root/'body_vertices_valid.npy')
 (root/'body_mesh_fit_manifest.json').write_text(json.dumps({'schema_version':2,'model':'SMPL_NEUTRAL','frames':len(lines),'valid':valid,'dtype':'float16','validity_file':'body_vertices_valid.npy'},indent=2),encoding='utf8'); print(valid)
if __name__=='__main__': main(sys.argv[1])
