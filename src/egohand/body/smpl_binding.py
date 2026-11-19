"""Torch-free attachment of observed MANO geometry to official SMPL wrists."""
import numpy as np


def attach_mano(meshes,T,fit):
    output=[]
    for mesh in meshes:
        side=mesh.get('side');index=20 if side=='left' else 21 if side=='right' else None
        if index is None:continue
        v=np.asarray(mesh.get('vertices'),float);j=np.asarray(mesh.get('joints20'),float)
        if v.ndim!=2 or v.shape[1]!=3 or j.shape!=(20,3) or not np.isfinite(v).all() or not np.isfinite(j).all():continue
        if mesh.get('coordinate_frame')=='body_world':world=v;world_j=j
        else:world=v@T[:3,:3].T+T[:3,3];world_j=j@T[:3,:3].T+T[:3,3]
        rotation=fit['rotations'][index];wrist=fit['joints'][index]
        local=(world-world_j[0])@rotation;local_j=(world_j-world_j[0])@rotation
        output.append({**mesh,'vertices':(local@rotation.T+wrist).tolist(),
            'joints20':(local_j@rotation.T+wrist).tolist(),
            'vertices_wrist_local':local.tolist(),'joints_wrist_local':local_j.tolist(),
            'coordinate_frame':'body_world','smpl_parent_joint':index,
            'T_world_from_smpl_wrist':np.vstack((np.column_stack((rotation,wrist)),[0.,0.,0.,1.])).tolist(),
            'observed_wrist_offset_m':float(np.linalg.norm(wrist-world_j[0]))})
    return output


def rejection_reasons(fit):
    reasons=[]
    if not np.isfinite(fit['vertices']).all():reasons.append('nonfinite')
    if max(map(abs,fit['foot_contact_errors_m']))>.012:reasons.append('floor_residual')
    if max(fit['wrist_errors_m'].values(),default=0.)>.10:reasons.append('wrist_residual')
    if fit['head_local_angle_deg']>85.:reasons.append('head_neck_twist')
    joints=np.asarray(fit.get('joints'),float)
    if joints.shape==(24,3):
        for hip,knee,ankle,name in ((1,4,7,'left'),(2,5,8,'right')):
            u=joints[hip]-joints[knee];v=joints[ankle]-joints[knee]
            c=float(np.dot(u,v)/(max(np.linalg.norm(u)*np.linalg.norm(v),1e-8)))
            angle=float(np.degrees(np.arccos(np.clip(c,-1.,1.))))
            if not 25.<=angle<=175.:reasons.append(f'{name}_knee_angle')
        if np.linalg.norm(joints[15]-joints[0])<.32:reasons.append('collapsed_torso')
    return reasons


def mesh_asset(smpl,solver):
    import torch
    w=smpl.model.lbs_weights.detach().numpy();rest=solver.rest.numpy()
    with torch.no_grad():template=smpl.model(betas=solver.betas).vertices[0].numpy()
    masks={};faces=np.asarray(smpl.faces)
    for side,wr,tip in [('left',20,22),('right',21,23)]:
        axis=rest[tip]-rest[wr];axis/=np.linalg.norm(axis)
        distal=((template-rest[wr])@axis>.012)&(w[:,wr]+w[:,tip]>.5)
        masks[side]=np.flatnonzero(distal[faces].any(axis=1)).tolist()
    return {'faces':smpl.faces,'mano_hand_face_indices':masks,'model':'SMPL_NEUTRAL'}
