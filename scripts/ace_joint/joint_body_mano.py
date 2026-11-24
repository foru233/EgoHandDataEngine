"""Differentiable SMPL + MANO bundle refinement from stereo pixels.

New body objective; previous SMPL/MANO serves only as initialization / weak prior.
Wrist position is an exact shared variable derived from SMPL, never post-shifted.
Network weights and shape stay frozen. Unobserved body is explicitly inferred.
"""
from pathlib import Path
import argparse,json,sys,time,hashlib
import numpy as np
from scipy.spatial.transform import Rotation

from joint_work_paths import PROJECT
sys.path[:0]=[str(PROJECT/'scripts'),str(PROJECT/'src')]
from egohand.body.smpl_mount import DEFAULT_CAMERA_FOREHEAD_OFFSET_MM, camera_forehead_offset_m, forehead_target
from smpl_body_mesh import SmplBody
import torch
import smplx
from smplx.lbs import batch_rodrigues,batch_rigid_transform,blend_shapes
from offline_mano import make_model,landmarks,project,robust,geodesic
torch.set_num_threads(1)
C_NP=np.diag([-1.,-1.,1.]).astype('float32')

def archive(path):
    with np.load(path) as f:return {k:f[k] for k in f.files}

class Body:
    def __init__(self,betas,head_vertex,device,offset_m=0.):
        self.model=SmplBody().model.to(device).eval()
        for p in self.model.parameters():p.requires_grad_(False)
        self.device=device;self.head_vertex=head_vertex;self.offset_m=offset_m
        self.betas=torch.as_tensor(betas[None],device=device,dtype=torch.float32)
        m=self.model
        with torch.no_grad():
            self.shaped=m.v_template[None]+blend_shapes(self.betas,m.shapedirs)
            self.rest=torch.einsum('jv,bvc->bjc',m.J_regressor,self.shaped)
        w=m.lbs_weights
        self.feet=[torch.where(w[:,a]+w[:,b]>.75)[0] for a,b in ((7,10),(8,11))]
        self.ids=torch.unique(torch.cat([*self.feet,torch.tensor([head_vertex],device=device)]))
        self.head_slot=int(torch.where(self.ids==head_vertex)[0][0])
        self.foot_slots=[torch.searchsorted(self.ids,ids) for ids in self.feet]
        self.posedirs=m.posedirs.reshape(207,-1,3)[:,self.ids].reshape(207,-1)
        self.weights=m.lbs_weights[self.ids]
        self.C=torch.tensor(C_NP,device=device)

    def forward(self,aa,T,full=False):
        b=len(aa);local=batch_rodrigues(aa.reshape(-1,3)).reshape(b,24,3,3)
        headR=self.C@T[:,:3,:3]@self.C
        rotations=[local[:,0]];parts=list(local.unbind(1))
        for j in range(1,24):
            parent=int(self.model.parents[j])
            if j==15:parts[j]=rotations[parent].transpose(-1,-2)@headR
            rotations.append(rotations[parent]@parts[j])
        local=torch.stack(parts,1);R=torch.stack(rotations,1)
        joints,A=batch_rigid_transform(local,self.rest.expand(b,-1,-1),self.model.parents)
        posefeat=(local[:,1:]-torch.eye(3,device=self.device)).reshape(b,-1)
        if full:
            posed=self.shaped+ (posefeat@self.model.posedirs).reshape(b,-1,3)
            weights=self.model.lbs_weights
        else:
            posed=self.shaped[:,self.ids]+(posefeat@self.posedirs).reshape(b,-1,3)
            weights=self.weights
        skin=(weights[None]@A.reshape(b,24,16)).reshape(b,-1,4,4)
        homo=torch.cat([posed,torch.ones((*posed.shape[:2],1),device=self.device)],-1)
        vertices=(skin@homo[...,None])[...,:3,0]
        slot=self.head_vertex if full else self.head_slot
        shift=forehead_target(T,self.offset_m)@self.C-vertices[:,slot]
        vertices=(vertices+shift[:,None])@self.C
        joints=(joints+shift[:,None])@self.C
        return joints,vertices,self.C@R@self.C,local,shift

def acceleration(x,t):
    dt=(t[1:]-t[:-1]).float()
    v=(x[1:]-x[:-1])/dt.reshape(-1,*([1]*(x.ndim-1)))
    return (v[1:]-v[:-1])/((dt[1:]+dt[:-1])*.5).reshape(-1,*([1]*(x.ndim-1)))

def motion_penalty(points, times, scale, valid):
    """Huber loss on world-space curvature, using actual frame intervals."""
    deviation=acceleration(points,times).norm(dim=-1)/scale
    per_frame=robust(deviation.reshape(len(valid),-1)).mean(-1)
    return (per_frame*valid).sum()/valid.sum().clamp_min(1)

def minimal_swing(source,target):
    source=source/np.linalg.norm(source);target=target/np.linalg.norm(target)
    axis=np.cross(source,target);sine=np.linalg.norm(axis);cosine=np.dot(source,target)
    if sine<1e-7:
        if cosine>0:return np.eye(3)
        axis=np.cross(source,np.eye(3)[np.argmin(np.abs(source))]);angle=np.pi
    else:angle=np.arctan2(sine,cosine)
    return Rotation.from_rotvec(axis*(angle/np.linalg.norm(axis))).as_matrix()

def aim_arms(body,initial,transforms,targets):
    """Analytic two-bone reach; no fitting and no new pose output."""
    with torch.no_grad():
        joints,_,rotations,_,_=body.forward(initial,transforms)
    joints=joints.cpu().numpy()
    rotations=rotations.cpu().numpy()
    camera=transforms.cpu().numpy()
    result=initial.cpu().numpy().copy()
    for row, hand in np.argwhere(np.isfinite(targets).all(-1)&(targets[:,:,2]>.03)):
        shoulder,elbow,wrist=16+hand,18+hand,20+hand
        parent=int(body.model.parents[shoulder])
        root=joints[row,shoulder]
        upper=joints[row,elbow]-root
        lower=joints[row,wrist]-joints[row,elbow]
        upper_length,lower_length=np.linalg.norm(upper),np.linalg.norm(lower)
        displacement=camera[row,:3,:3]@targets[row,hand]+camera[row,:3,3]-root
        distance=np.linalg.norm(displacement)
        if distance<1e-5:continue
        direction=displacement/distance
        reach=np.clip(distance,abs(upper_length-lower_length)+.005,upper_length+lower_length-.005)
        bend=upper-direction*np.dot(upper,direction)
        if np.linalg.norm(bend)<1e-5:
            axis=np.eye(3)[np.argmin(np.abs(direction))]
            bend=axis-direction*np.dot(axis,direction)
        bend/=np.linalg.norm(bend)
        along=(upper_length**2+reach**2-lower_length**2)/(2.*reach)
        toward_elbow=direction*along+bend*np.sqrt(max(0.,upper_length**2-along**2))
        toward_wrist=direction*reach-toward_elbow
        swing=minimal_swing(upper,toward_elbow)
        shoulder_world=swing@rotations[row,shoulder]
        elbow_swing=minimal_swing(swing@lower,toward_wrist)
        elbow_world=elbow_swing@swing@rotations[row,elbow]
        for index, local in ((shoulder,rotations[row,parent].T@shoulder_world),
                             (elbow,shoulder_world.T@elbow_world)):
            result[row,index]=Rotation.from_matrix(C_NP@local@C_NP).as_rotvec()
    return torch.as_tensor(result,dtype=initial.dtype,device=initial.device)

def context_windows(segment,window,context):
    first=0
    while first<len(segment):
        start=max(0,first-context)
        stop=min(len(segment),start+window)
        yield segment[start:stop],first-start
        first=stop

def hand_continuity_edges(valid, identities, timestamps):
    previous,current=np.flatnonzero(valid)[:-1],np.flatnonzero(valid)[1:]
    delta=timestamps[current]-timestamps[previous]
    same=(identities[previous]>=0)&(identities[previous]==identities[current])
    connected=same&(delta>0)&(delta<=120_000_000)
    return previous[connected],current[connected],delta[connected].astype(np.float32)/1e9

def solve(a):
    if a.window<1 or not all(np.isfinite(value) and value>=0 for value in
            (a.continuity_context,a.continuity_weight,a.stability_weight)):
        raise ValueError('Invalid continuity configuration')
    if a.output.exists():raise FileExistsError(a.output)
    a.output.mkdir(parents=True)
    device=a.device;seed=archive(a.seed/'smpl_pose.npz');cam=archive(a.camera)
    obs=archive(a.hands/'associated_observations.npz');hands=archive(a.hands/'mano_params.npz')
    direct='initial_hand_mask' in hands
    if not direct:
        skeleton=archive(a.hands/'skeleton_3d.npz');mesh=np.load(a.hands/'mesh_vertices.npy',mmap_mode='r')
    manifest=json.loads((a.seed/'unified_smpl_manifest.json').read_text())
    n=len(seed['valid']);assert np.array_equal(cam['timestamp_ns'],obs['timestamps_ns'])
    assert np.array_equal(seed['timestamp_ns'],cam['timestamp_ns'])
    fitted=np.flatnonzero(seed['valid'])
    body=Body(seed['betas'][fitted[0] if fitted.size else 0],manifest['head_anchor_vertex'],device,
        camera_forehead_offset_m(a.camera_forehead_offset_mm))
    models={s:make_model(s,device) for s in ('left','right')}
    K=torch.tensor(obs['K'],device=device,dtype=torch.float32)
    Tv=torch.tensor(np.linalg.inv(obs['T']),device=device,dtype=torch.float32)
    tensor=lambda x:torch.as_tensor(x,device=device,dtype=torch.float32)
    init_aa=np.concatenate([seed['global_orient'][:,None],seed['body_pose'].reshape(n,23,3)],1)
    valid_cam=np.isfinite(cam['T_world_from_camera']).all((1,2))
    initial_prior_only=valid_cam&~seed['valid']
    for i in np.flatnonzero(initial_prior_only):
        candidates=np.flatnonzero(seed['valid']&(cam['reference']==cam['reference'][i]))
        if len(candidates):
            nearest=candidates[np.argmin(abs(candidates-i))];init_aa[i]=init_aa[nearest]
        else:
            init_aa[i]=0;init_aa[i,16,2]=-1.15;init_aa[i,17,2]=1.15
            camera=cam['T_world_from_camera'][i]
            head_rotation=C_NP@camera[:3,:3]@C_NP
            init_aa[i,0,1]=np.arctan2(head_rotation[0,2],head_rotation[2,2])
            height=cam['ground_y_m'][i]-forehead_target(camera,body.offset_m)[1]
            knee=2*np.arccos(np.clip(1-max(0.,1.693-height)/.86,.5,1.))
            init_aa[i,[1,2,4,5,7,8],0]=[-knee/2,-knee/2,knee,knee,-knee/2,-knee/2]
    output_aa=init_aa.copy();body_valid=valid_cam&np.isfinite(cam['ground_y_m']);computed=np.zeros(n,bool)
    out_hrot=hands['rotations'].copy();out_ht=hands['translation_m'].copy()
    if direct:
        out_j=np.full((n,2,21,3),np.nan,np.float32)
        hand_valid=hands['initial_hand_mask']&body_valid[:,None]
    else:
        out_j=skeleton['xyz_m'].copy();hand_valid=np.isfinite(mesh).all((2,3))
    out_v=np.lib.format.open_memmap(a.output/'hand_vertices.npy',mode='w+',dtype='float32',shape=(n,2,778,3))
    out_v[:]=np.nan if direct else mesh
    out_b=np.lib.format.open_memmap(a.output/'smpl_vertices.npy',mode='w+',dtype='float32',shape=(n,6890,3))
    out_b[:]=np.load(a.seed/'smpl_vertices.npy',mmap_mode='r') if fitted.size else 0.
    bj=seed['joints_world'].copy();br=seed['rotations_world'].copy();transl=seed['transl'].copy()
    reports=[];rejected_hands=0;started=time.monotonic()
    continuity=direct and a.continuity_weight>0
    stabilize=continuity and a.stability_weight>0
    context=min(a.continuity_context,a.window//3) if continuity else 0
    valid_cam=np.isfinite(cam['T_world_from_camera']).all((1,2))
    usable=valid_cam&np.isfinite(cam['ground_y_m'])
    usable[:a.start]=False
    if a.frames:usable[a.start+a.frames:]=False
    # Windows never cross a VIO reset, rejected body frame, or >0.2 s time gap.
    groups=[];group=[]
    for i in range(n):
        contiguous=bool(group) and i==group[-1]+1 and cam['reference'][i]==cam['reference'][group[-1]] and 0<int(cam['timestamp_ns'][i]-cam['timestamp_ns'][group[-1]])<200_000_000
        if group and (not usable[i] or not contiguous):groups.append(group);group=[]
        if usable[i]:group.append(i)
    if group:groups.append(group)
    windows=[item for segment in groups for item in context_windows(segment,a.window,context)]
    for gno,(ix,prefix) in enumerate(windows):
        b=len(ix);T=tensor(cam['T_world_from_camera'][ix]);initial=tensor(init_aa[ix])
        if direct:initial=aim_arms(body,initial,T,hands['wrist_camera_m'][ix])
        fixed_body=torch.arange(b,device=device)<prefix
        previous_body=tensor(output_aa[ix]) if prefix else initial
        # Unobserved frames still run the new body objective; they remain prior-only.
        active=[h for h in range(2) if hand_valid[ix,h].any()]
        aa=torch.nn.Parameter(initial.clone())
        params=[aa];hp={};hb={};masks={};ref={};prior={};pw={};motion_ref={};wrist_seed={}
        previous_hands={};hand_edges={}
        hand_triplets={}
        frame_ns=cam['timestamp_ns'][ix]
        delta=np.diff(frame_ns)
        core_triplets=(np.arange(max(0,b-2))+2>=prefix)&(delta[:-1]<=80_000_000)&(delta[1:]<=80_000_000)
        body_triplets=tensor(core_triplets)
        for h in active:
            good=hand_valid[ix,h]
            rot=hands['rotations'][ix,h].copy();rot[~good]=np.tile(np.eye(3),(16,1,1))
            hp[h]=torch.nn.Parameter(tensor(Rotation.from_matrix(rot.reshape(-1,3,3)).as_rotvec().reshape(b,48)))
            if prefix:
                fixed_hand=fixed_body&torch.as_tensor(good,device=device)
                previous_rot=out_hrot[ix,h].copy()
                previous_rot[~good]=np.eye(3)
                previous_pose=tensor(Rotation.from_matrix(previous_rot.reshape(-1,3,3)).as_rotvec().reshape(b,48))
                previous_hands[h]=(fixed_hand,previous_pose)
            params.append(hp[h]);beta=hands['betas'][ix,h].copy();beta[~good]=0;hb[h]=tensor(beta)
            mask=obs['valid'][ix,:,h].copy()&good[:,None,None];mask[:,:,1]=False;masks[h]=tensor(mask)
            pr=obs['prior_R'][ix,h].copy();finite=np.isfinite(pr).all((2,3,4));pr[~finite]=np.tile(np.eye(3),(16,1,1))
            prior[h]=tensor(pr);pw[h]=tensor(obs['prior_weight'][ix,h]*finite*good[:,None])
            if continuity:
                before,after,seconds=hand_continuity_edges(good,obs['identity_ids'][ix,h],cam['timestamp_ns'][ix])
                edge_mask=after>=prefix
                before,after,seconds=before[edge_mask],after[edge_mask],seconds[edge_mask]
                stereo_confident=obs['stereo_valid'][ix,h].sum(axis=1)>=6
                edge_weight=np.where(stereo_confident[before]&stereo_confident[after],.45,1.)
                hand_edges[h]=(before.tolist(),after.tolist(),tensor(seconds),tensor(edge_weight))
                ids=obs['identity_ids'][ix,h]
                hand_triplets[h]=tensor(core_triplets&good[:-2]&good[1:-1]&good[2:]&
                    (ids[1:-1]>=0)&(ids[:-2]==ids[1:-1])&(ids[1:-1]==ids[2:]))
        opt=torch.optim.Adam(params,lr=a.lr)
        times=tensor((frame_ns-frame_ns[0])/1e9)
        floor=tensor(cam['ground_y_m'][ix]);uv={h:tensor(np.nan_to_num(obs['uv'][ix,:,h])) for h in active}
        with torch.no_grad():
            init_j,_,init_R,_,_=body.forward(initial,T)
            for h in active:
                # Calibrated per-frame orientation relation is a weak consistency prior,
                # not an anatomically calibrated wrist mounting measurement.
                ref[h]=T[:,:3,:3]@batch_rodrigues(hp[h][:,:3])
                ref[h]=init_R[:,20+h].transpose(-1,-2)@ref[h]
                reference_o=models[['left','right'][h]](global_orient=hp[h][:,:3],hand_pose=hp[h][:,3:],betas=hb[h],
                    transl=tensor(np.nan_to_num(hands['translation_m'][ix,h])))
                motion_ref[h]=landmarks(reference_o,['left','right'][h]).detach()
                if direct:
                    stereo=obs['stereo_valid'][ix,h] & np.isfinite(obs['stereo_xyz'][ix,h]).all(axis=-1)
                    target=obs['stereo_xyz'][ix,h,0].copy()
                    derived=~stereo[:,0] & (stereo.sum(1)>=4)
                    target[derived]=motion_ref[h][derived,0].cpu().numpy()
                    wrist_seed[h]=(tensor(np.nan_to_num(target)),tensor((stereo[:,0]|derived)&hand_valid[ix,h]))
        def body_parameters():
            return torch.where(fixed_body[:,None,None],previous_body,aa) if prefix else aa

        def evaluate(geometry_warmup=0.):
            joints,verts,R,local,shift=body.forward(body_parameters(),T)
            sole=torch.stack([verts[:,slots,1].amax(1) for slots in body.foot_slots],1)
            loss=a.floor_weight*(sole-floor[:,None]).square().mean()
            loss=loss+a.body_prior*(aa-initial).square().mean()
            loss=loss+.05*(local[:,15]-torch.eye(3,device=device)).square().mean()
            # Retain conservative leg structure; no image evidence is claimed for legs.
            for hip,knee,ankle in ((1,4,7),(2,5,8)):
                u=joints[:,hip]-joints[:,knee];v=joints[:,ankle]-joints[:,knee]
                cos=(u*v).sum(-1)/(u.norm(dim=-1)*v.norm(dim=-1)).clamp_min(.01)
                loss=loss+2*(torch.relu(cos-.9063).square()+torch.relu(-.9962-cos).square()).mean()
            loss=loss+.25*(local[:,[7,8,10,11]]-batch_rodrigues(initial[:,[7,8,10,11]].reshape(-1,3)).reshape(b,4,3,3)).square().mean()
            if continuity and b>1:
                first_edge=max(0,prefix-1)
                angles=geodesic(local[first_edge:-1],local[first_edge+1:])
                seconds=(times[first_edge+1:]-times[first_edge:-1]).clamp_min(.005)
                loss=loss+a.continuity_weight*robust(angles/(8.*seconds[:,None])).mean()
            handout={};pixelvalues=[]
            for h in active:
                side=['left','right'][h]
                hand_pose=hp[h]
                if prefix:
                    held,previous_pose=previous_hands[h]
                    hand_pose=torch.where(held[:,None],previous_pose,hand_pose)
                o=models[side](global_orient=hand_pose[:,:3],hand_pose=hand_pose[:,3:],betas=hb[h])
                base=landmarks(o,side)
                wristcam=torch.einsum('bij,bj->bi',T[:,:3,:3].transpose(-1,-2),joints[:,20+h]-T[:,:3,3])
                delta=wristcam-base[:,0];hj=base+delta[:,None];hv=o.vertices+delta[:,None]
                if direct and geometry_warmup:
                    target,weight=wrist_seed[h]
                    loss=loss+geometry_warmup*((wristcam-target).square().sum(-1)*weight).sum()/weight.sum().clamp_min(1)
                p,z=project(hj,K,Tv);mask=masks[h]
                pixel=(robust((p-uv[h])/5).sum(-1)*mask).sum()/mask.sum().clamp_min(1)
                loss=loss+pixel;pixelvalues.append(pixel)
                rotations=batch_rodrigues(hand_pose.reshape(-1,3)).reshape(b,16,3,3)
                angle=geodesic(rotations[:,None],prior[h])
                loss=loss+.15*(angle.square().mean(-1)*pw[h]).sum()/pw[h].sum().clamp_min(1)
                loss=loss+a.finger_prior*(angle[:,:,1:].square().mean(-1)*pw[h]).sum()/pw[h].sum().clamp_min(1)
                loss=loss+.002*hp[h][:,3:].square().mean()
                framegood=tensor(hand_valid[ix,h])
                orientation=geodesic(T[:,:3,:3]@rotations[:,0],R[:,20+h]@ref[h])
                loss=loss+a.orientation_weight*(orientation.square()*framegood).sum()/framegood.sum().clamp_min(1)
                if continuity:
                    before,after,seconds,weight=hand_edges[h]
                    if before:
                        world_root=T[:,:3,:3]@rotations[:,0]
                        root=geodesic(world_root[before],world_root[after])
                        finger=geodesic(rotations[before,1:],rotations[after,1:])
                        motion=robust(root/(10.*seconds))+robust(finger/(10.*seconds[:,None])).mean(-1)
                        loss=loss+a.continuity_weight*(motion*weight).mean()
                if stabilize and b>=3:
                    # Camera-frame acceleration can penalize a still hand when the head moves.
                    # Track wrist position and wrist-relative hand geometry independently.
                    relative=(hj[:,1:]-hj[:,:1])@T[:,:3,:3].transpose(-1,-2)
                    valid=hand_triplets[h]
                    loss=loss+a.stability_weight*a.temporal_weight*(
                        motion_penalty(joints[:,20+h],times,9.,valid)+
                        motion_penalty(relative,times,7.,valid))
                elif b>=3:
                    good=tensor(hand_valid[ix,h][:-2]&hand_valid[ix,h][1:-1]&hand_valid[ix,h][2:])
                    reference=motion_ref[h]
                    # Reference acceleration is the geometrically refined seed,
                    # not raw monocular ACE translation acceleration.
                    diff=(acceleration(hj,times)-(0. if direct else acceleration(reference,times)))/30
                    loss=loss+a.temporal_weight*(robust(diff).mean((1,2))*good).sum()/good.sum().clamp_min(1)
                loss=loss+.1*(torch.relu(.03-z).square()*mask).sum()/mask.sum().clamp_min(1)
                handout[h]=(hj,hv,delta,rotations)
            if stabilize and b>=3:
                # The hard forehead anchor follows measured head motion. Penalize
                # pelvis jitter and poorly observed bends without smoothing that anchor.
                elbow=joints[:,18:20]-joints[:,16:18]
                knee=joints[:,4:6]-joints[:,1:3]
                loss=loss+a.stability_weight*(
                    a.body_temporal*motion_penalty(joints[:,0],times,7.,body_triplets)+
                    a.body_temporal*motion_penalty(knee,times,7.,body_triplets)+
                    2*a.body_temporal*motion_penalty(elbow,times,7.,body_triplets))
            elif b>=3:
                reference=0. if continuity else acceleration(init_j,times)
                relative=joints-joints[:,:1] if continuity else joints
                loss=loss+a.body_temporal*robust((acceleration(relative,times)-reference)/30).mean()
            return loss,handout,(joints,verts,R,local,shift,sole),torch.stack(pixelvalues).sum() if pixelvalues else loss*0
        with torch.no_grad():initial_loss=float(evaluate()[0])
        best=None;best_loss=float('inf')
        warmup=min(25,a.iterations//4) if direct else 0
        for iteration in range(a.iterations):
            opt.zero_grad();loss,_,_,_=evaluate(75.*(1.-iteration/warmup) if iteration<warmup else 0.)
            if not torch.isfinite(loss):raise RuntimeError(f'Nonfinite joint loss {gno}/{iteration}')
            if iteration>=warmup and float(loss)<best_loss:
                best_loss=float(loss);best=[p.detach().clone() for p in params]
            loss.backward()
            if any(p.grad is None or not torch.isfinite(p.grad).all() for p in params):raise RuntimeError('Nonfinite gradients')
            torch.nn.utils.clip_grad_norm_(params,10);opt.step()
            with torch.no_grad():aa[:,1:].clamp_(-2.8,2.8)
        with torch.no_grad():
            if best is not None:
                for p,value in zip(params,best):p.copy_(value)
            final_loss,ho,bo,pixel=evaluate();joints,_,R,local,shift,sole=bo
            _,full,_,_,_=body.forward(body_parameters(),T,full=True)
            assert torch.isfinite(full).all()
            core=ix[prefix:]
            out_b[core]=full[prefix:].cpu().numpy();bj[core]=joints[prefix:].cpu().numpy();br[core]=R[prefix:].cpu().numpy()
            actual=Rotation.from_matrix(local.cpu().numpy().reshape(-1,3,3)).as_rotvec().reshape(b,24,3)
            output_aa[core]=actual[prefix:];transl[core]=shift[prefix:].cpu().numpy();computed[core]=True
            maxfloor=float((sole-floor[:,None]).abs().max())
            for h,(hj,hv,delta,rot) in ho.items():
                good=hand_valid[ix,h].copy()
                good[:prefix]=False
                if direct:
                    pixels,depth=project(hj,K,Tv)
                    errors=(pixels-uv[h]).norm(dim=-1).cpu().numpy()
                    view_depth=depth.cpu().numpy()
                    measured=masks[h].cpu().numpy().astype(bool)
                    for row in np.flatnonzero(good):
                        mask=measured[row]
                        observed_errors=errors[row][mask]
                        if (hj[row,0,2]<=.02 or not len(observed_errors)
                                or not np.isfinite(observed_errors).all()
                                or (view_depth[row][mask]<=.01).any()
                                or np.median(observed_errors)>40.):
                            good[row]=False;rejected_hands+=1
                    hand_valid[core,h]=good[prefix:]
                gi=np.array(ix)[good]
                out_v[gi,h]=hv.cpu().numpy()[good];out_j[gi,h]=hj.cpu().numpy()[good]
                out_hrot[gi,h]=rot.cpu().numpy()[good];out_ht[gi,h]=delta.cpu().numpy()[good]
            reports.append(dict(first=core[0],last=core[-1],frames=len(core),left_context=prefix,
                initial_loss=initial_loss,final_loss=float(final_loss),
                pixel_objective=float(pixel),max_floor_residual_m=maxfloor))
        print(f'JOINT {gno+1}/{len(windows)} frames={computed.sum()} loss={initial_loss:.4f}->{float(final_loss):.4f} floor={maxfloor:.4f} elapsed={time.monotonic()-started:.1f}',flush=True)
    out_v.flush();out_b.flush()
    if direct:hand_valid &= computed[:,None]
    np.savez_compressed(a.output/'joint_params.npz',body_pose=output_aa[:,1:].reshape(n,69),global_orient=output_aa[:,0],
        transl=transl,betas=seed['betas'],joints_world=bj,rotations_world=br,valid=body_valid,computed=computed,
        frame_id=seed['frame_id'],timestamp_ns=seed['timestamp_ns'],world_to_smpl=C_NP,
        hand_rotations=out_hrot,hand_translation_m=out_ht,hand_joints=out_j,hand_valid=hand_valid,
        ground_y_m=cam['ground_y_m'],initial_prior_only=initial_prior_only)
    config={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()}
    report=dict(status='completed',algorithm='joint_smpl_mano_v2',frames=n,optimized_frames=int(computed.sum()),
        body_without_hand='new_body_objective_prior_only',wrist_constraint='exact_shared_SMPL_wrist_in_forward_model',
        posthoc_wrist_shift=False,ACE_weights_frozen=True,body_shape_frozen=True,hand_shape_frozen=True,
        ACE_translation_loss_weight=0,temporal_reference='acceleration_regularization' if direct else 'geometry_refined_seed',
        hand_initialization='direct_ace_and_stereo' if direct else 'cached_geometric_fit',
        body_initialization=manifest.get('body_initialization','cached_pose_parameter_seed'),
        hand_rejected_frames=rejected_hands,configuration=config,
        continuity=dict(enabled=continuity,left_context=context,world_space_stability=stabilize,
                        hand_link_max_s=.12,split_on_camera_reference=True),
        windows=reports,elapsed_s=time.monotonic()-started,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (a.output/'joint_manifest.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print('JOINT COMPLETE',report['optimized_frames'],flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--seed',type=Path,required=True);p.add_argument('--hands',type=Path,required=True)
    p.add_argument('--camera',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--camera-forehead-offset-mm',type=float,default=DEFAULT_CAMERA_FOREHEAD_OFFSET_MM)
    p.add_argument('--device',default='cuda');p.add_argument('--window',type=int,default=81);p.add_argument('--iterations',type=int,default=80)
    p.add_argument('--start',type=int,default=0);p.add_argument('--frames',type=int,default=0);p.add_argument('--lr',type=float,default=.004)
    p.add_argument('--finger-prior',type=float,default=.3);p.add_argument('--temporal-weight',type=float,default=.15)
    p.add_argument('--body-prior',type=float,default=.1);p.add_argument('--body-temporal',type=float,default=.1)
    p.add_argument('--floor-weight',type=float,default=2500);p.add_argument('--orientation-weight',type=float,default=.01)
    p.add_argument('--continuity-context',type=int,default=12);p.add_argument('--continuity-weight',type=float,default=.12)
    p.add_argument('--stability-weight',type=float,default=.5,help='World-space body and hand curvature regularization; 0 restores previous objective')
    solve(p.parse_args())
if __name__=='__main__':main()
