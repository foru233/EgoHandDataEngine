"""Independent ACE cross-view matching, explicit unmatched and observation join."""
from pathlib import Path
import json
import numpy as np
import torch
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation
from offline_mano import make_model,landmarks,transform_pose_to_common,mean_rotations


def assignment(cost,unmatched=3.,margin=.15):
    n,m=cost.shape
    if not n or not m:return {},[]
    def solve(c):
        mat=np.full((n+m,n+m),1e8);mat[:n,:m]=c
        for i in range(n):mat[i,m+i]=unmatched
        for j in range(m):mat[n+j,j]=unmatched
        mat[n:,m:]=0
        r,q=linear_sum_assignment(mat)
        return {(int(i),int(j)) for i,j in zip(r,q) if i<n and j<m and c[i,j]<1e6},float(mat[r,q].sum())
    pairs,total=solve(cost);disputed=set();diagnostics=[]
    for pair in pairs:
        c=cost.copy();c[pair]=1e8;other,value=solve(c);gap=value-total
        diagnostics.append(dict(pair=list(pair),alternative_margin=gap))
        if gap<margin:disputed|=pairs-other
    return dict(pairs-disputed),diagnostics


def collect_sequence(root):
    root=Path(root);manifest=json.loads((root/'sequence_manifest.json').read_text())
    if manifest['status']!='completed':raise ValueError('ACE sequence incomplete')
    start=manifest['start'];n=manifest['frames'];records=[[[] for _ in range(n)] for _ in range(2)]
    for clip in manifest['outputs']:
        view=['left','right'].index(clip['side']);a,b=clip['start'],clip['end']
        with np.load(Path(clip['path'])/'raw_predictions.npz') as data:
            length=b-a
            arrays={key:data[key] for key in ('global_orient','hand_pose','betas','cam_trans','exists_2d','exists_3d')}
        if arrays['global_orient'].shape!=(length,2,3,3):raise ValueError('Raw ACE orientation shape changed')
        for k in range(length):
            weight=min(k+1,length-k)
            records[view][a-start+k].append(({key:values[k] for key,values in arrays.items()},weight))
    result={key:[] for key in ['rotations','translation','betas','exists_2d','exists_3d']}
    for view in range(2):
        values={key:[] for key in result}
        for frame in records[view]:
            if not frame:raise ValueError('Missing ACE frame coverage')
            weights=np.array([w for _,w in frame],float)
            for key in ['cam_trans','betas','exists_2d','exists_3d']:
                values[{'cam_trans':'translation'}.get(key,key)].append(np.average(np.array([r[key] for r,_ in frame]),weights=weights,axis=0))
            rotations=np.array([np.concatenate([r['global_orient'][:,None],r['hand_pose']],axis=1) for r,_ in frame])
            values['rotations'].append(mean_rotations(rotations,weights))
        for key in result:result[key].append(np.array(values[key]))
    return {k:np.array(v) for k,v in result.items()},manifest


def decode_common(raw,T):
    _,n,h=raw['translation'].shape[:3]
    joints=np.full((2,n,2,21,3),np.nan);rot=np.array(raw['rotations'],copy=True);trans=np.array(raw['translation'],copy=True)
    for view in range(2):
        for hand,side in enumerate(['left','right']):
            model=make_model(side,'cpu');r=raw['rotations'][view,:,hand];t=raw['translation'][view,:,hand];b=raw['betas'][view,:,hand]
            finite=np.isfinite(r).all((1,2,3))&np.isfinite(t).all(1)&np.isfinite(b).all(1)
            idx=np.flatnonzero(finite)
            if not len(idx):continue
            for first in range(0,len(idx),128):
                chunk=idx[first:first+128]
                rr,tt=transform_pose_to_common(r[chunk],t[chunk],b[chunk],T[view],model)
                rot[view,chunk,hand]=rr;trans[view,chunk,hand]=tt
                aa=Rotation.from_matrix(rr.reshape(-1,3,3)).as_rotvec().reshape(len(chunk),48)
                with torch.no_grad():
                    out=model(global_orient=torch.tensor(aa[:,:3],dtype=torch.float32),hand_pose=torch.tensor(aa[:,3:],dtype=torch.float32),
                              transl=torch.tensor(tt,dtype=torch.float32),betas=torch.tensor(b[chunk],dtype=torch.float32))
                    joints[view,chunk,hand]=landmarks(out,side).numpy()
    return joints,rot,trans


def pixels(x,K,T):
    xyz=(x-T[:3,3])@T[:3,:3]
    p=xyz@K.T
    with np.errstate(divide='ignore',invalid='ignore'):uv=p[...,:2]/p[...,2:]
    return uv,xyz[...,2]


def associate(raw,manifest,observation_file):
    with np.load(observation_file) as file:obs={k:file[k] for k in file.files}
    start=manifest['start'];n=manifest['frames'];sl=slice(start,start+n)
    K=obs['K'];T=obs['T_common_from_view'];times=obs['timestamps_ns'][sl]
    joints,R,t=decode_common(raw,T)
    uv=np.full((n,2,2,21,2),np.nan);valid=np.zeros((n,2,2,21),bool)
    stereo=np.zeros((n,2,21),bool);priorR=np.full((n,2,2,16,3,3),np.nan);priort=np.full((n,2,2,3),np.nan)
    priorweight=np.zeros((n,2,2));betas=np.zeros((2,10));diagnostics=[]
    C=np.full((n,2,21,3),np.nan);B=joints[0].copy();stereo_xyz=np.full((n,2,21,3),np.nan);identities=np.full((n,2),-1,int)
    history={};obsmap=np.full((n,2,2),-1,int);viewpairs=np.full((n,2),-1,int)
    selected_detections=np.full((n,2,2),-1,int)
    # Fundamental matrix in the actual per-view pinhole pixel grids.
    inv=np.linalg.inv(T[1]);r=inv[:3,:3];tr=inv[:3,3]
    skew=np.array([[0,-tr[2],tr[1]],[tr[2],0,-tr[0]],[-tr[1],tr[0],0]])
    F=np.linalg.inv(K[1]).T@skew@r@np.linalg.inv(K[0])
    for k in range(n):
        maps=[];pixel_costs=[]
        for view in range(2):
            pred,z=pixels(joints[view,k],K[view],T[view]);cost=np.full((2,2),1e8)
            for hand in range(2):
                for candidate in range(2):
                    mask=obs['valid'][start+k,view,candidate]&np.isfinite(pred[hand]).all(1)&(z[hand]>.03)
                    mask[1]=False
                    if mask.sum()<6:continue
                    error=np.median(np.linalg.norm(pred[hand,mask]-obs['uv'][start+k,view,candidate,mask],axis=1))
                    if error>80:continue
                    label=obs['labels'][start+k,view,candidate]
                    cost[hand,candidate]=error/20+(.15 if label not in ['', ['left','right'][hand]] else 0)
            mapping,detail=assignment(cost,unmatched=2.5,margin=.2)
            maps.append(mapping);pixel_costs.append(cost)
            for h,c in mapping.items():obsmap[k,view,h]=c
        cross=np.full((2,2),1e8);details=[]
        if obs['stereo_admitted'][start+k]:
            own0,_=pixels(joints[0,k],K[0],T[0]);own1,_=pixels(joints[1,k],K[1],T[1])
            for i in range(2):
                for j in range(2):
                    a,b=joints[0,k,i],joints[1,k,j]
                    if not np.isfinite(a).all() or not np.isfinite(b).all():continue
                    distance=np.linalg.norm(a[0]-b[0]);pose=np.sqrt(np.mean(np.sum(((a-a[0])-(b-b[0]))**2,axis=1)))
                    p=np.r_[own0[i,0],1];q=np.r_[own1[j,0],1];line=F@p
                    epi=abs(q@line)/max(np.linalg.norm(line[:2]),1e-9)
                    li,ri=maps[0].get(i,-1),maps[1].get(j,-1)
                    measured=li>=0 and ri>=0 and obs['strict_pairs'][start+k,li]==ri
                    if measured:
                        cross[i,j]=.25+pose/.06+(.1 if i!=j else 0)
                    elif distance<.2 and pose<.06 and epi<20:
                        temporal=0
                        if i in history:
                            old,ts=history[i];dt=(times[k]-ts)/1e9
                            if 0<dt<=.5:temporal=min(np.linalg.norm((a[0]+b[0])*.5-old)/(.03+4*dt),2)
                        cross[i,j]=distance/.1+pose/.035+epi/10+.4*temporal+(.1 if i!=j else 0)
                    details.append(dict(left=i,right=j,wrist_distance_m=float(distance),pose_rms_m=float(pose),epipolar_px=float(epi),strict_measured_pair=bool(measured)))
        pairs,ambiguity=assignment(cross,unmatched=3.,margin=.2)
        usedright=set()
        for i in range(2):
            li=maps[0].get(i,-1);j=pairs.get(i,-1);ri=maps[1].get(j,-1) if j>=0 else -1
            if li>=0:
                selected_detections[k,i,0]=li
                uv[k,0,i]=obs['uv'][start+k,0,li];valid[k,0,i]=obs['valid'][start+k,0,li]
            if j>=0:
                usedright.add(j);viewpairs[k,i]=j
                if ri>=0:uv[k,1,i]=obs['uv'][start+k,1,ri];valid[k,1,i]=obs['valid'][start+k,1,ri]
                if ri>=0:selected_detections[k,i,1]=ri
                if li>=0 and ri>=0 and obs['strict_pairs'][start+k,li]==ri:
                    stereo[k,i]=obs['geometry_valid'][start+k,li,ri]
                    stereo_xyz[k,i]=obs['triangulated_xyz'][start+k,li,ri]
            priorR[k,i,0]=R[0,k,i];priort[k,i,0]=t[0,k,i]
            score=raw['exists_3d'][0,k,i];vis=raw['exists_2d'][0,k,i]
            priorweight[k,i,0]=(.1+.9*vis) if np.isfinite(score) and score>.5 else 0
            # A contradictory chirality model must not become an articulation prior.
            if j==i:
                priorR[k,i,1]=R[1,k,j];priort[k,i,1]=t[1,k,j]
                score=raw['exists_3d'][1,k,j];vis=raw['exists_2d'][1,k,j]
                priorweight[k,i,1]=(.1+.9*vis) if np.isfinite(score) and score>.5 else 0
            if j>=0:
                C[k,i]=np.average([joints[0,k,i],joints[1,k,j]],weights=[max(vis,.1),max(raw['exists_2d'][1,k,j],.1)],axis=0)
            else:C[k,i]=joints[0,k,i]
            if valid[k,:,i].any():
                identities[k,i]=i
                history[i]=(C[k,i,0].copy(),times[k])
        # Truly unmatched right detections remain independent right-only tracks.
        # Do not merge them into a visible left track by a handedness label.
        for j,ri in maps[1].items():
            if j in usedright or valid[k,0,j].any():continue
            uv[k,1,j]=obs['uv'][start+k,1,ri];valid[k,1,j]=obs['valid'][start+k,1,ri]
            selected_detections[k,j,1]=ri
            priorweight[k,j,0]=0;priorR[k,j,1]=R[1,k,j];priort[k,j,1]=t[1,k,j]
            priorweight[k,j,1]=max(float(raw['exists_2d'][1,k,j]),.1) if raw['exists_3d'][1,k,j]>.5 else 0
            C[k,j]=joints[1,k,j];identities[k,j]=2+j
        if not obs['stereo_admitted'][start+k]:
            # Retain unavailable-side data in original observations, never fuse it.
            for h in range(2):
                if valid[k,0,h].any():valid[k,1,h]=False;priorweight[k,h,1]=0;selected_detections[k,h,1]=-1
        diagnostics.append(dict(frame_index=start+k,view_pairs=viewpairs[k].tolist(),observation_candidates=obsmap[k].tolist(),
                          cross_view_costs=np.where(cross<1e6,cross,-1).tolist(),candidates=details,
                          alternatives=ambiguity,right_only_track_ids=identities[k].tolist(),
                          chirality_conflicts=[(i,j) for i,j in pairs.items() if i!=j]))
    for hand in range(2):
        samples=[]
        for view in range(2):
            chosen=valid[:,view,hand].any(1)&np.isfinite(raw['betas'][view,:,hand]).all(1)
            samples.extend(raw['betas'][view,chosen,hand])
        betas[hand]=np.median(samples,axis=0) if samples else np.zeros(10)
    return dict(uv=uv,valid=valid,stereo_valid=stereo,stereo_xyz=stereo_xyz,prior_R=priorR,prior_t=priort,prior_weight=priorweight,
                betas=betas,K=K,T=T,timestamps_ns=times,B=B,C=C,identity_ids=identities,
                baseline_xyz=obs['baseline_xyz'][sl],detection_indices=selected_detections,diagnostics=diagnostics)
