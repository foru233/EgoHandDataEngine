"""Bounded offline MANO optimizer; actual stereo pixels + SO(3) ACE priors.

Inputs describe ONE already-associated physical hand. Association must not be
inferred from array slot number by callers. 3D triangulations are diagnostic /
initialization only: no second likelihood is added for the same stereo pixels.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import time
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from smplx.lbs import batch_rodrigues
import smplx
from joint_work_paths import MANO

MP_MODEL=[0,13,14,15,-1,1,2,3,-2,4,5,6,-3,10,11,12,-4,7,8,9,-5]
LEGACY_TIPS=[745,317,444,556,673]


@dataclass
class Config:
    window: int=81
    overlap: int=20
    iterations: int=80
    lr: float=.012
    ace_rotation: float=.15
    ace_finger_rotation: float=0.0
    ace_translation: float=0.0
    refine_shape: bool=True
    articulation: float=.002
    shape_prior: float=.002
    acceleration: float=.5
    positive_depth: float=.1
    max_unobserved_s: float=.5
    robust_px: float=5.


def geodesic(R,Q):
    """SO(3) angle stable at identity and pi; independent of AA branch cuts."""
    M=R.transpose(-1,-2)@Q
    c=((M.diagonal(dim1=-2,dim2=-1).sum(-1)-1)*.5).clamp(-1,1)
    v=torch.stack([M[...,2,1]-M[...,1,2],M[...,0,2]-M[...,2,0],M[...,1,0]-M[...,0,1]],-1)
    s=torch.sqrt(v.square().sum(-1)+1e-12)*.5
    return torch.atan2(s,c)


def robust(x,delta=1.):
    a=x.abs();return torch.where(a<=delta,.5*x.square(),delta*(a-.5*delta))


def mean_rotations(matrices,weights):
    """Weighted chordal SO(3) mean, never arithmetic AA averaging."""
    M=np.einsum('n,n...ij->...ij',weights,matrices)/weights.sum()
    u,_,v=np.linalg.svd(M);fix=np.ones(M.shape[:-2]+(3,));fix[...,-1]=np.linalg.det(u@v)
    return (u*fix[...,None,:])@v


def landmarks(output,side):
    tips=list(LEGACY_TIPS)
    if side=='left':tips[2]=445
    return torch.stack([output.joints[:,i] if i>=0 else output.vertices[:,tips[-i-1]] for i in MP_MODEL],1)


def make_model(side,device):
    model=smplx.create(str(MANO),model_type='mano',is_rhand=side=='right',
                       use_pca=False,flat_hand_mean=False).to(device).eval()
    for p in model.parameters():p.requires_grad_(False)
    return model


def transform_pose_to_common(rotations,translation,betas,T,model):
    """MANO rotates about its shape-dependent root, not the coordinate origin."""
    with torch.no_grad():
        output=model(betas=torch.as_tensor(betas,dtype=torch.float32,device=model.v_template.device),
                     hand_pose=torch.zeros((len(betas),45),device=model.v_template.device),
                     global_orient=torch.zeros((len(betas),3),device=model.v_template.device))
    pivot=output.joints[:,0].cpu().numpy()
    result=np.array(rotations,copy=True);result[:,0]=T[:3,:3]@result[:,0]
    trans=np.asarray(translation)@T[:3,:3].T+T[:3,3]+pivot@(T[:3,:3]-np.eye(3)).T
    return result,trans


def project(joints,K,T_view_from_common):
    xyz=torch.einsum('vij,tkj->tvki',T_view_from_common[:,:3,:3],joints)+T_view_from_common[None,:,:3,3,None].transpose(-1,-2)
    p=torch.einsum('vij,tvkj->tvki',K,xyz)
    return p[...,:2]/p[...,2:].clamp_min(.01),xyz[...,2]


def window_ranges(n,window,overlap):
    if window<1 or not 0<=overlap<window:raise ValueError('Invalid overlap/window')
    if n<=window:return [(0,n)]
    starts=list(range(0,n-window+1,window-overlap))
    if starts[-1]!=n-window:starts.append(n-window)
    return [(s,min(s+window,n)) for s in starts]


def inference_age(timestamps,observed):
    t=(timestamps-timestamps[0])/1e9
    indices=np.flatnonzero(observed)
    if not len(indices):return np.full(len(t),np.inf)
    # Offline bidirectional support: nearest actual observation, no frame copies.
    return np.min(abs(t[:,None]-t[indices][None,:]),axis=1)


def optimize(side,observations,valid,timestamps,K,T_common_from_view,prior_R,prior_t,prior_weight,initial_betas,
             config=Config(),confidence=None,device='cuda',stereo_xyz=None,stereo_mask=None,stereo_weight=0.0):
    started=time.monotonic();model=make_model(side,device);cfg=config
    n=len(timestamps);timestamps=np.asarray(timestamps,dtype=np.int64)
    if n<1 or (np.diff(timestamps)<=0).any():raise ValueError('Timestamp order invalid')
    if observations.shape!=(n,2,21,2) or valid.shape!=(n,2,21):raise ValueError('Observation contract mismatch')
    if prior_R.shape!=(n,2,16,3,3) or prior_t.shape!=(n,2,3):raise ValueError('Prior contract mismatch')
    K=np.asarray(K,float);T_common_from_view=np.asarray(T_common_from_view,float)
    if K.shape!=(2,3,3) or not np.isfinite(K).all() or (K[:,[0,1],[0,1]]<=0).any():raise ValueError('Invalid pinhole intrinsics')
    if T_common_from_view.shape!=(2,4,4) or not np.isfinite(T_common_from_view).all():raise ValueError('Invalid camera transform')
    for transform in T_common_from_view:
        r=transform[:3,:3]
        if not np.allclose(transform[3],[0,0,0,1],atol=1e-8) or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:
            raise ValueError('Camera transform must have a proper orthonormal rotation')
    if np.asarray(initial_betas).shape!=(10,) or not np.isfinite(initial_betas).all():raise ValueError('Invalid shared shape initialization')
    mask=np.asarray(valid,bool)&np.isfinite(observations).all(-1)
    mask[:,:,1]=False # thumb CMC is not an audited MediaPipe/MANO GT correspondence
    weights=np.asarray(prior_weight,float)
    good=np.isfinite(prior_R).all(axis=(-1,-2,-3))&np.isfinite(prior_t).all(-1)&np.isfinite(weights)&(weights>0)
    weights=np.where(good,weights,0.)
    if not good.any():raise ValueError('No finite ACE initialization; refusing fabricated priors')
    I=np.tile(np.eye(3),(n,2,16,1,1));safeR=np.where(good[:,:,None,None,None],prior_R,I)
    safet=np.where(good[:,:,None],prior_t,0)
    initialR=np.zeros((n,16,3,3));initialt=np.zeros((n,3))
    for i in range(n):
        if not good[i].any():
            initialR[i]=np.eye(3); initialt[i]=[0,0,.5]
        else:
            initialR[i]=mean_rotations(safeR[i,good[i]],weights[i,good[i]])
            initialt[i]=np.average(safet[i,good[i]],weights=weights[i,good[i]],axis=0)
    # Missing initialization rows do not inherit the last hand; rejected below.
    initialized=good.any(1)
    # Keep the original ACE motion as the acceleration reference. Replacing
    # this by noisy framewise triangulations would preserve stereo jitter.
    temporal_initialt=initialt.copy()
    # Stereo XYZ is derived from these same pixels: use it ONLY to initialize
    # rigid location. The final likelihood remains the two pixel measurements.
    if stereo_weight != 0:
        raise ValueError('Correlated stereo XYZ likelihood is disabled; use initialization only')
    stereo_initialized=np.zeros(n,bool)
    if stereo_xyz is not None:
        stereo_xyz=np.asarray(stereo_xyz,float)
        if stereo_xyz.shape!=(n,21,3):raise ValueError('Stereo position contract mismatch')
        stereo_valid=np.isfinite(stereo_xyz).all(-1)
        if stereo_mask is not None:
            if np.asarray(stereo_mask).shape!=(n,21):raise ValueError('Stereo mask contract mismatch')
            stereo_valid &= np.asarray(stereo_mask,bool)
        stereo_valid &= mask.all(1)
        stereo_valid[:,1]=False
        with torch.no_grad():
            init_aa=Rotation.from_matrix(initialR.reshape(-1,3,3)).as_rotvec().reshape(n,48)
            ref=model(global_orient=torch.as_tensor(init_aa[:,:3],dtype=torch.float32,device=device),
                      hand_pose=torch.as_tensor(init_aa[:,3:],dtype=torch.float32,device=device),
                      betas=torch.as_tensor(initial_betas,dtype=torch.float32,device=device).reshape(1,10).expand(n,-1),
                      transl=torch.zeros((n,3),device=device))
            local=landmarks(ref,side).cpu().numpy()
        for i in range(n):
            usable=stereo_valid[i]
            if initialized[i] and usable.sum()>=6:
                initialt[i]=np.median(stereo_xyz[i,usable]-local[i,usable],axis=0)
                stereo_initialized[i]=True
    for matrix in safeR[good].reshape(-1,3,3):
        if not np.allclose(matrix.T@matrix,np.eye(3),atol=1e-3) or abs(np.linalg.det(matrix)-1)>1e-3:
            raise ValueError('ACE supplied an invalid rotation matrix')
    tensor=lambda x,dtype=torch.float32:torch.as_tensor(x,dtype=dtype,device=device)
    Ks=tensor(K);Ts=tensor(np.linalg.inv(T_common_from_view))
    uv=tensor(np.nan_to_num(observations));vm=tensor(mask)
    conf=np.ones_like(mask,dtype=np.float32) if confidence is None else np.asarray(confidence,float)
    if np.any(~np.isfinite(conf)) or np.any(conf<0) or np.any(conf>1):raise ValueError('Invalid per-joint confidence')
    cw=tensor(conf)*vm
    rprior=tensor(safeR);tprior=tensor(safet);wp=tensor(weights)
    shared=tensor(initial_betas).reshape(1,10)
    shape_initial=shared.clone();records=[]
    allR=[[] for _ in range(n)];allt=[[] for _ in range(n)];allw=[[] for _ in range(n)]
    # Fit one shared shape on a bounded sample, then freeze across the sequence.
    # A second global pass may be added after held-out validation of this design.
    sample=np.linspace(0,n-1,min(n,125),dtype=int)
    shape=torch.nn.Parameter(shared.clone())
    opt=torch.optim.Adam([shape],lr=.02)
    aa=tensor(Rotation.from_matrix(initialR[sample].reshape(-1,3,3)).as_rotvec().reshape(len(sample),48))
    trans=tensor(initialt[sample])
    for _ in range(30 if cfg.refine_shape else 0):
        opt.zero_grad();o=model(global_orient=aa[:,:3],hand_pose=aa[:,3:],betas=shape.expand(len(sample),-1),transl=trans)
        p,_=project(landmarks(o,side),Ks,Ts)
        loss=(robust((p-uv[sample])/cfg.robust_px).sum(-1)*cw[sample]).sum()/cw[sample].sum().clamp_min(1)
        loss=loss+cfg.shape_prior*(shape-shape_initial).square().mean()
        if not torch.isfinite(loss):raise RuntimeError('Shared shape fit nonfinite')
        loss.backward();opt.step()
        with torch.no_grad():shape.clamp_(-3,3)
    shared=shape.detach()
    for start,end in window_ranges(n,cfg.window,cfg.overlap):
        length=end-start;sl=slice(start,end)
        aa=torch.nn.Parameter(tensor(Rotation.from_matrix(initialR[sl].reshape(-1,3,3)).as_rotvec().reshape(length,48)))
        trans=torch.nn.Parameter(tensor(initialt[sl]));opt=torch.optim.Adam([aa,trans],lr=cfg.lr)
        times=tensor((timestamps[sl]-timestamps[start])/1e9);dt=times[1:]-times[:-1]
        # Temporal target retains the ACE sequence's rapid motion. Penalize
        # deviations from that acceleration rather than driving motion to zero.
        with torch.no_grad():
            ref_aa=tensor(Rotation.from_matrix(initialR[sl].reshape(-1,3,3)).as_rotvec().reshape(length,48))
            ref_o=model(global_orient=ref_aa[:,:3],hand_pose=ref_aa[:,3:],betas=shared.expand(length,-1),transl=tensor(temporal_initialt[sl]))
            ref_joints=landmarks(ref_o,side)
            if length>=3:
                ref_v=(ref_joints[1:]-ref_joints[:-1])/dt[:,None,None]
                ref_a=(ref_v[1:]-ref_v[:-1])/((dt[1:]+dt[:-1])*.5)[:,None,None]
        failed=False;history=[]
        for iteration in range(cfg.iterations):
            opt.zero_grad();o=model(global_orient=aa[:,:3],hand_pose=aa[:,3:],betas=shared.expand(length,-1),transl=trans)
            joints=landmarks(o,side);p,z=project(joints,Ks,Ts)
            l2=(robust((p-uv[sl])/cfg.robust_px).sum(-1)*cw[sl]).sum()/cw[sl].sum().clamp_min(1)
            Rs=batch_rodrigues(aa.reshape(-1,3)).reshape(length,16,3,3)
            angles=geodesic(Rs[:,None],rprior[sl])
            lace=(angles.square().mean(-1)*wp[sl]).sum()/wp[sl].sum().clamp_min(1)
            lfinger=(angles[:,:,1:].square().mean(-1)*wp[sl]).sum()/wp[sl].sum().clamp_min(1)
            lt=(robust((trans[:,None]-tprior[sl])/.05).mean(-1)*wp[sl]).sum()/wp[sl].sum().clamp_min(1)
            lm=aa[:,3:].square().mean()+torch.relu(aa[:,3:].abs()-2.8).square().mean()*10
            temporal=joints.sum()*0
            if length>=3:
                velocity=(joints[1:]-joints[:-1])/dt[:,None,None]
                acceleration=(velocity[1:]-velocity[:-1])/((dt[1:]+dt[:-1])*.5)[:,None,None]
                edge=(dt[1:]<.2)&(dt[:-1]<.2)
                if edge.any():temporal=robust((acceleration[edge]-ref_a[edge])/30).mean()
            depth=(torch.relu(.03-z).square()*vm[sl]).sum()/vm[sl].sum().clamp_min(1)
            loss=l2+cfg.ace_rotation*lace+cfg.ace_finger_rotation*lfinger+cfg.ace_translation*lt+cfg.articulation*lm+cfg.acceleration*temporal+cfg.positive_depth*depth
            if not torch.isfinite(loss):failed=True;break
            loss.backward()
            if any(p.grad is None or not torch.isfinite(p.grad).all() for p in [aa,trans]):failed=True;break
            torch.nn.utils.clip_grad_norm_([aa,trans],10.)
            opt.step()
            if iteration in [0,cfg.iterations-1]:history.append(dict(iteration=iteration,total=float(loss.detach()),pixels=float(l2.detach()),ace=float(lace.detach()),temporal=float(temporal.detach())))
        records.append(dict(start=start,end=end,failed=failed,loss=history))
        if failed:continue
        with torch.no_grad():
            o=model(global_orient=aa[:,:3],hand_pose=aa[:,3:],betas=shared.expand(length,-1),transl=trans)
            p,z=project(landmarks(o,side),Ks,Ts)
            residual=torch.linalg.norm(p-uv[sl],dim=-1)
            pixel_quality=(residual*cw[sl]).sum((1,2))/cw[sl].sum((1,2)).clamp_min(1)
            Rs=batch_rodrigues(aa.reshape(-1,3)).reshape(length,16,3,3).cpu().numpy();tr=trans.cpu().numpy()
            q=1/(1+pixel_quality.cpu().numpy()/cfg.robust_px)
            center_weight=np.minimum(np.arange(1,length+1),np.arange(length,0,-1)).astype(float)
            for j,i in enumerate(range(start,end)):
                allR[i].append(Rs[j]);allt[i].append(tr[j]);allw[i].append(q[j]*center_weight[j])
    fittedR=np.full((n,16,3,3),np.nan);fittedt=np.full((n,3),np.nan)
    for i in range(n):
        if allw[i] and initialized[i]:
            fittedR[i]=mean_rotations(np.array(allR[i]),np.array(allw[i]));fittedt[i]=np.average(allt[i],weights=allw[i],axis=0)
    fitted=np.isfinite(fittedR).all(axis=(-1,-2,-3))
    joints=np.full((n,21,3),np.nan,np.float32);vertices=np.full((n,778,3),np.nan,np.float32)
    reproj=np.full((n,2,21),np.nan);depth=np.full((n,2,21),np.nan)
    with torch.no_grad():
        indices=np.flatnonzero(fitted)
        for group in [indices[j:j+125] for j in range(0,len(indices),125)]:
            aa=tensor(Rotation.from_matrix(fittedR[group].reshape(-1,3,3)).as_rotvec().reshape(len(group),48))
            o=model(global_orient=aa[:,:3],hand_pose=aa[:,3:],betas=shared.expand(len(group),-1),transl=tensor(fittedt[group]))
            j=landmarks(o,side);p,z=project(j,Ks,Ts)
            joints[group]=j.cpu().numpy();vertices[group]=o.vertices.cpu().numpy()
            reproj[group]=torch.linalg.norm(p-uv[group],dim=-1).cpu().numpy();depth[group]=z.cpu().numpy()
    age=np.stack([inference_age(timestamps,mask[:,:,j].any(1)) for j in range(21)],1)
    # No direct thumb CMC evidence; support comes from observed wrist/fingers.
    age[:,1]=inference_age(timestamps,mask.any((1,2)))
    reliable=fitted[:,None]&(age<=cfg.max_unobserved_s)
    reliable&=(depth>0).all(1)&np.isfinite(joints).all(-1)
    observed_error=np.where(mask,reproj,np.nan)
    rejected=np.any(mask&(reproj>25),axis=1)
    reliable&=~rejected
    states=np.full((n,21),'inferred_ace_temporal',dtype='<U40')
    states[mask[:,0]&~mask[:,1]]='observed_left_only'
    states[mask[:,1]&~mask[:,0]]='observed_right_only'
    # A caller must supply independent geometry validation before promoting stereo.
    states[mask[:,0]&mask[:,1]]='observed_both_geometry_unverified'
    states[~reliable]='invalid'
    worst=np.max(np.where(mask,np.nan_to_num(reproj,nan=25),0),axis=1)
    worst=np.where(mask.any(1),worst,15)
    score=np.exp(-age/.25)/(1+worst/5)
    score[~reliable]=0
    joints[~reliable]=np.nan
    meshvalid=reliable[:,0]&(reliable.sum(1)>=8)
    vertices[~meshvalid]=np.nan
    return dict(rotations=fittedR,translation=fittedt,betas=shared.cpu().numpy()[0],joints=joints,vertices=vertices,
                valid=reliable,mesh_valid=meshvalid,source=states,quality_score_uncalibrated=score,
                reprojection_error_px=observed_error,inference_age_s=age,
                report=dict(config=asdict(cfg),windows=records,elapsed_s=time.monotonic()-started,
                    fitted_frames=int(fitted.sum()),mesh_valid_frames=int(meshvalid.sum()),
                    shape_constant=True,triangulated_xyz_likelihood_added=False,world_coordinates=None,
                    confidence_scope='uncalibrated quality score; no absolute covariance estimated',
                    position_source='measured pixel reprojection; stereo XYZ initialization only; monocular depth remains prior-dependent',
                    stereo_initialized_frames=int(stereo_initialized.sum()),
                    temporal_reference='ACE motion acceleration before stereo position initialization',
                    joint_convention='legacy MANO tips + native thumb base; thumb CMC excluded from measured losses'))
