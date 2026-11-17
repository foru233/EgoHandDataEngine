"""隔离 MANO 推理进程。stdin/stdout 使用逐行 JSON；诊断写 stderr。"""
import collections
import contextlib
import inspect
import json
import sys
import traceback
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
# Keep the Torch environment's compiled NumPy first; the vendored MANO
# dependencies provide smplx/chumpy without shadowing that ABI.
sys.path.append(str(ROOT/'tools'/'mano_deps'))
import numpy as np

# 官方旧版模型文件含 Chumpy 对象，加载时提供兼容别名，不修改主进程。
if not hasattr(inspect,'getargspec'):
    _ArgSpec=collections.namedtuple('ArgSpec','args varargs keywords defaults')
    def _getargspec(fn):
        v=inspect.getfullargspec(fn)
        return _ArgSpec(v.args,v.varargs,v.varkw,v.defaults)
    inspect.getargspec=_getargspec
for _name,_value in {'bool':bool,'int':int,'float':float,'complex':complex,'object':object,'unicode':str,'str':str}.items():
    if _name not in np.__dict__:setattr(np,_name,_value)
import torch
import smplx
torch.set_num_threads(1)

# MediaPipe 对 MANO 的可比点：拇指 CMC (1) 没有官方明确对应，跳过。
MP_INDICES=np.array([0,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20])
MODEL_JOINTS=np.array([0,14,15,-1,1,2,3,-2,4,5,6,-3,10,11,12,-4,7,8,9,-5])
TIP_VERTICES=(745,317,444,556,673)

def landmark_tensor(output,side):
    tips=list(TIP_VERTICES)
    if side=='left':tips[2]=445
    vertices=output.vertices[0]; joints=output.joints[0]
    return torch.stack([joints[i] if i>=0 else vertices[tips[-i-1]] for i in MODEL_JOINTS])

class Engine:
    def __init__(self,model_dir,config):
        self.cfg=config; self.previous={};self.show3d_scenes={};self.model_dir=Path(model_dir)
        with contextlib.redirect_stdout(sys.stderr):
            self.models={side:smplx.MANO(str(model_dir),is_rhand=side=='right',use_pca=False,num_pca_comps=45,
                            flat_hand_mean=False) for side in ('left','right')}
        for model in self.models.values():
            for parameter in model.parameters():parameter.requires_grad_(False)

    def truth(self,entry):
        if entry.get('kind')=='show3d':
            # 使用 SHOW3D 官方 MANO 解码器：世界毫米转原始左相机米。
            sys.path.insert(0,str(ROOT/'tools'/'show3d_recovered'))
            from show3d.hand_mesh import HandMeshScene
            from show3d.camera import world_to_camera
            key=(entry['root'],entry['subject'],entry['scene'])
            if key not in self.show3d_scenes:
                with contextlib.redirect_stdout(sys.stderr):
                    self.show3d_scenes[key]=HandMeshScene(*key,'mano',asset_dir=self.model_dir)
            mesh=self.show3d_scenes[key].mesh(int(entry['frame_index']),int(entry['slot']))
            if mesh is None:return None
            camera=np.asarray(entry['camera_world_from'],dtype=float)
            if camera.shape!=(4,4) or not np.isfinite(camera).all():raise ValueError('SHOW3D 左相机位姿无效')
            return {'vertices':(world_to_camera(mesh.vertices_world_mm,camera)/1000.).tolist(),
                    'joints20':(world_to_camera(mesh.landmarks_world_mm,camera)/1000.).tolist(),
                    'faces':mesh.faces.tolist(),'side':'left' if entry['slot']==0 else 'right',
                    'kind':'show3d_official_mano'}
        side=entry['side']; pose=np.asarray(entry['pose'],np.float32); betas=np.asarray(entry['betas'],np.float32)
        if side not in self.models or pose.shape!=(51,) or betas.shape!=(10,) or not np.isfinite(pose).all() or not np.isfinite(betas).all() or not np.any(pose):
            raise ValueError('MANO 官方标注参数无效或缺失')
        model=self.models[side]
        # DexYCB 使用前 3 维全局旋转、45 维 PCA 以及米制平移，非 flat mean。
        hand=torch.tensor(pose[3:48][None,:]) @ torch.tensor(model.np_hand_components,dtype=torch.float32)
        with torch.no_grad():
            out=model(global_orient=torch.tensor(pose[None,:3]),hand_pose=hand,
                      transl=torch.tensor(pose[None,48:51]),betas=torch.tensor(betas[None,:]))
        return {'vertices':out.vertices[0].tolist(),'joints20':landmark_tensor(out,side).tolist(),
                'faces':model.faces.tolist(),'side':side}

    def fit(self,entry,frame,sequence):
        side=entry['side'].lower(); model=self.models.get(side)
        if model is None:return {'error':'左右手不明确，不能选择 MANO 模型'}
        data=np.asarray(entry['points'],float); valid=np.asarray(entry['valid'],bool)
        if data.shape!=(21,3) or valid.shape!=(21,):return {'error':'骨架格式错误'}
        selected=data[MP_INDICES]; mask=valid[MP_INDICES]&np.isfinite(selected).all(axis=1)
        actual_valid=valid&np.isfinite(data).all(axis=1)
        fingers=sum(bool(actual_valid[a:b].any()) for a,b in ((2,5),(5,9),(9,13),(13,17),(17,21)))
        if mask.sum()<self.cfg['min_joints'] or fingers<self.cfg['min_fingers']:
            return {'error':f"有效关节不足：{mask.sum()} 个、{fingers} 根手指"}
        target=torch.tensor(selected[mask],dtype=torch.float32)
        indices=np.flatnonzero(mask)
        # 有序、相近帧的同一轨迹才使用上帧姿态；跳帧和切换序列均重置。
        key=(sequence,str(entry.get('track_id')),side); prev=self.previous.get(key)
        warm=prev is not None and 0<frame-prev['frame']<=self.cfg['temporal_max_frame_gap']
        orient=torch.nn.Parameter(torch.tensor(prev['orient'] if warm else [[0.,0.,0.]],dtype=torch.float32))
        pose=torch.nn.Parameter(torch.tensor(prev['pose'] if warm else [[0.]*45],dtype=torch.float32))
        betas=torch.nn.Parameter(torch.tensor(prev['betas'] if warm else [[0.]*10],dtype=torch.float32))
        with torch.no_grad():
            base=model(global_orient=orient,hand_pose=pose,betas=betas,transl=torch.zeros((1,3)))
            if not warm:
                # 使用共同关节的刚体配准初始化整体旋转，降低从零角度开始的局部极小值。
                from scipy.spatial.transform import Rotation
                src=landmark_tensor(base,side)[indices].numpy(); dst=target.numpy()
                u,_,vt=np.linalg.svd((src-src.mean(0)).T@(dst-dst.mean(0)))
                correction=np.eye(3);correction[-1,-1]=np.linalg.det(vt.T@u.T)
                matrix=vt.T@correction@u.T
                orient.copy_(torch.tensor(Rotation.from_matrix(matrix).as_rotvec()[None,:],dtype=torch.float32))
                base=model(global_orient=orient,hand_pose=pose,betas=betas,transl=torch.zeros((1,3)))
            initial=target.mean(0)-landmark_tensor(base,side)[indices].mean(0)
        trans=torch.nn.Parameter(initial.reshape(1,3))
        optimizer=torch.optim.Adam([orient,pose,betas,trans],lr=self.cfg['learning_rate'])
        for _ in range(self.cfg['iterations']):
            optimizer.zero_grad(); out=model(global_orient=orient,hand_pose=pose,betas=betas,transl=trans)
            diff=landmark_tensor(out,side)[indices]-target
            loss=(diff*diff).sum(dim=1).mean()+self.cfg['pose_prior_weight']*pose.square().mean()+self.cfg['shape_prior_weight']*betas.square().mean()
            if warm:loss=loss+self.cfg['temporal_weight']*(pose-torch.tensor(prev['pose'])).square().mean()
            if not torch.isfinite(loss):return {'error':'拟合出现非有限值'}
            loss.backward();optimizer.step()
            with torch.no_grad():
                # 形状和局部轴角限制在有限范围内，避免低残差但明显异常的解。
                betas.clamp_(-3.,3.);pose.clamp_(-3.14,3.14)
        with torch.no_grad():
            out=model(global_orient=orient,hand_pose=pose,betas=betas,transl=trans)
            err=float(torch.linalg.vector_norm(landmark_tensor(out,side)[indices]-target,dim=1).mean())
        if not np.isfinite(err):return {'error':'拟合残差不是有限值'}
        if err>self.cfg['max_fit_error_m']:
            return {'error':f'拟合残差 {err*1000:.1f} mm，超过阈值', 'error_m':err}
        if entry.get('track_id',-1)>0:
            self.previous[key]={'frame':frame,'orient':orient.detach().tolist(),'pose':pose.detach().tolist(),'betas':betas.detach().tolist()}
        return {'vertices':out.vertices[0].detach().tolist(),'faces':model.faces.tolist(),
                'joints20':landmark_tensor(out,side).detach().tolist(),'side':side,
                'track_id':entry.get('track_id'),'error_m':err,'joints_used':int(mask.sum()),'temporal_used':warm}

    def run(self,request):
        frame=request['frame']; sequence=request['sequence']; result={'frame':frame,'sequence':sequence,'request_id':request.get('request_id'),'predicted':[],'truth':[],'errors':[]}
        if request.get('prediction'):
            if not request.get('hands'):result['errors'].append('本帧没有可拟合的预测骨架')
            for hand in request.get('hands',[]):
                item=self.fit(hand,frame,sequence)
                if 'error' in item:result['errors'].append(item['error'])
                else:result['predicted'].append(item)
        if request.get('ground_truth'):
            for entry in request.get('truth',[]):
                try:
                    mesh=self.truth(entry)
                    if mesh is not None:result['truth'].append(mesh)
                except (ValueError,KeyError) as exc:result['errors'].append(str(exc))
            if not request.get('truth'):result['errors'].append('本帧没有可解析的官方 MANO 标注')
        return result

def main():
    import yaml
    config=yaml.safe_load((ROOT/'configs'/'mano.yaml').read_text(encoding='utf8'))
    engine=Engine(ROOT/config['models'],config)
    for line in sys.stdin:
        try:
            request=json.loads(line); response=engine.run(request)
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            response={'error':str(exc)}
        print(json.dumps(response,allow_nan=False),flush=True)

if __name__=='__main__':main()
