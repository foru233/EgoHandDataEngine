"""Same-frame inputs and editable floor references, without a torch dependency."""
import numpy as np
from .smpl_mapping import SMPL_JOINT_NAMES,SMPL_PARENT_IDS
from .body_model import BodyPose
from ..vio.hand_pose_binding import consumer_result_display_pose
from ..vio.local_segments import display_reference_key

# SMPL_NEUTRAL fitted to 1.75 m in the straight rest pose: forehead anchor
# 335 is 1.69345 m above the mean of the two foot-surface minima. Round to
# the GUI's 0.1 cm precision. This is a model-based mounting assumption.
STANDING_FOREHEAD_HEIGHT_M=1.693


class GroundReference:
    """Height means initial camera-to-floor distance, even when seated."""
    def __init__(self,height_m=STANDING_FOREHEAD_HEIGHT_M):
        self.height_m=float(height_m);self.origins={}

    def reset(self):self.origins.clear()

    def observe(self,result):
        if type(getattr(result,'timestamp_ns',None)) is not int:return None
        T=consumer_result_display_pose(result)
        if T is None:return None
        p=result.diagnostics.get('vio_pose') or result.diagnostics.get('live_vio_pose') or {}
        ref=display_reference_key(p)
        key=repr(ref)
        self.origins.setdefault(key,float(T[1,3]))
        return ref

    def floor_for(self,result):
        ref=self.observe(result)
        p=result.diagnostics.get('vio_pose') or result.diagnostics.get('live_vio_pose') or {}
        key=repr(display_reference_key(p))
        return self.origins[key]+self.height_m if key in self.origins else None


def request_from_result(result,ground_y_m):
    if type(getattr(result,'frame_id',None)) is not int or type(getattr(result,'timestamp_ns',None)) is not int:return None
    T=consumer_result_display_pose(result)
    if T is None:return None
    wrists={};directions={}
    for hand in getattr(result,'hands',()):
        side=hand.handedness.lower()
        if side not in ('left','right') or not hand.joints[0].valid:continue
        p=np.asarray(hand.joints[0].xyz_m,float)
        if not np.isfinite(p).all():continue
        wrists[side]=(T[:3,:3]@p+T[:3,3]).tolist()
        if hand.joints[9].valid:
            d=np.asarray(hand.joints[9].xyz_m,float)-p;n=np.linalg.norm(d)
            if np.isfinite(d).all() and n>.02:directions[side]=(T[:3,:3]@(d/n)).tolist()
    pose=result.diagnostics.get('vio_pose') or result.diagnostics.get('live_vio_pose') or {}
    derived=result.diagnostics.get('offline_derived_pose') or {}
    hand_inputs=[]
    for h in getattr(result,'hands',()):
        points=np.asarray([j.xyz_m for j in h.joints],float)
        valid=np.asarray([j.valid for j in h.joints],bool)&np.isfinite(points).all(axis=1)
        hand_inputs.append({'side':h.handedness,'track_id':h.track_id,
            'points':np.where(valid[:,None],points,0.).tolist(),'valid':valid.tolist()})
    return {'frame_id':result.frame_id,'timestamp_ns':result.timestamp_ns,
        'reference':display_reference_key(pose),'head_world':T[:3,3].tolist(),
        'head_rotation':T[:3,:3].tolist(),'wrists':wrists,'hand_directions':directions,
        'ground_y_m':float(ground_y_m),'camera_transform':T.tolist(),
        'mano':(derived.get('mano') or {}).get('predicted',[]),'hands':hand_inputs,
        'seed_vertices_path':derived.get('body_mesh_vertices_path'),
        'seed_index':derived.get('body_mesh_index')}


def pose_from_reply(reply,asset):
    fit=reply['fit']
    return BodyPose(reply['frame_id'],reply['timestamp_ns'],reply.get('reference'),'estimated_preview',
        dict(zip(SMPL_JOINT_NAMES,np.asarray(fit['joints'],float))),
        bones=tuple((SMPL_JOINT_NAMES[p],SMPL_JOINT_NAMES[j]) for j,p in enumerate(SMPL_PARENT_IDS) if p>=0),
        diagnostics={**{k:v for k,v in fit.items() if k not in ('vertices','joints','rotations')},
            'unified_smpl':True,'body_mesh_vertices':fit['vertices'],'body_mesh':asset,
            'body_mesh_coordinate_frame':'body_world','ground_source':'user_initial_camera_height',
            'mano_attached_sides':[m['side'] for m in reply.get('mano',[])],
            'head_rotation_world':reply['head_rotation'],'estimated_body_not_observed':True})
