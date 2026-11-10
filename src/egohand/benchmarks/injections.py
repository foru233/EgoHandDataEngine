# 物理运动时间偏移实验；所有结果显式标记 synthetic，不作为硬件同步验收。

"""Physical time-offset injection. Synthetic evidence never represents hardware."""
from dataclasses import replace
import itertools
import numpy as np
import cv2
from ..sources.synthetic import SyntheticStereoSource
from ..core.pipeline import EgoHandEngine,default_config
from ..core.types import HandDetection
from ..calibration.rectifier import StereoRectifier
from ..stereo.triangulation import JointTriangulator

SYNC_QUALITIES=('HARDWARE_VERIFIED','DATASET_DOCUMENTED','USER_ASSERTED','SOFTWARE_ESTIMATED','UNKNOWN')
OFFSETS_MS=(0,1,3,5,10)
SPEEDS={'static':0.,'low':.05,'medium':.3,'high':1.}
AXES={'horizontal':0,'vertical':1,'depth':2}


# 左右图在不同时刻投影同一运动手；仅二维观测进入重建，XYZ参考只用于事后误差。
def measure_case(offset_ms,speed_m_s,axis,pixel_std=1.5):
    source=SyntheticStereoSource('two_static',frames=1);source.open();source.start()
    frame=source.read();c=source.get_calibration();r=StereoRectifier(c)
    xyz=np.asarray(frame.metadata['ground_truth_xyz_m'][0]);moved=xyz.copy()
    moved[:,AXES[axis]]+=speed_m_s*offset_ms/1000
    observations=[]
    for side,X in enumerate((xyz,moved)):
        R,t=(np.eye(3),np.zeros(3)) if side==0 else (c.R_right_left,c.t_right_left)
        uv=cv2.projectPoints(X,cv2.Rodrigues(R)[0],t,c.K_left,c.D_left)[0].reshape(21,2)
        # This rectification is linear/no-distortion here; actual triangulator
        # receives observations in rectified pixels, never the xyz reference.
        uv=r.points(uv,'left' if side==0 else 'right')
        observations.append(HandDetection('Left',1.,uv/[c.image_width,c.image_height],uv,origin='synthetic_projection'))
    config=default_config();config['uncertainty']={**config['uncertainty'],'assumed_pixel_std':pixel_std}
    joints=JointTriangulator(r,config).triangulate(*observations)
    geometric=[j for j in joints if j.valid]
    engine=EgoHandEngine(config=config,calibration=c)
    frame=replace(frame,left_detections=[observations[0]],right_detections=[observations[1]],
        image_geometry='rectified',left_timestamp_ns=frame.timestamp_ns,
        right_timestamp_ns=frame.timestamp_ns+int(offset_ms*1e6),
        metadata={**frame.metadata,'rectification_id':c.calibration_id,
            'sync_quality':'SOFTWARE_ESTIMATED','synchronization_evidence':'known synthetic injected sensor offset; no physical hardware'})
    result=engine.process(frame)
    error=[np.linalg.norm(j.xyz_m-xyz[j.joint_id])*1000 for j in geometric]
    stds=[j.depth_std_m for j in joints if j.depth_std_m is not None]
    H=cv2.triangulatePoints(r.P1,r.P2,observations[0].pixel_landmarks.T.copy(),observations[1].pixel_landmarks.T.copy()).T
    solved=(H[:,:3]/H[:,3,None])@r.R1
    ungated_error=float(np.mean(np.linalg.norm(solved-xyz,axis=1))*1000) if np.isfinite(solved).all() else None
    return {'classification':'sync_injection_systematic_bias','dataset':'synthetic',
        'offset_ms':offset_ms,'speed_m_s':speed_m_s,'axis':axis,'assumed_pixel_std':pixel_std,
        'raw_geometric_error_mm':float(np.mean(error)) if error else None,
        'ungated_triangulation_error_mm':ungated_error,
        'epipolar_error_px':float(np.mean([j.epipolar_error_px for j in joints])),
        'reprojection_error_px':float(np.mean([j.reprojection_error_px for j in joints])),
        'depth_std_m':float(np.mean(stds)) if stds else None,
        'valid_rate_without_sync_gate':len(geometric)/21,
        'valid_rate_with_runtime_sync_gate':sum(h.valid_joint_count for h in result.hands)/21,
        'runtime_status':result.reconstruction_status,'runtime_reason':result.diagnostics.get('reason'),
        'sync_quality':result.diagnostics['sync_quality'],'hardware_sync_precision_verified':False}


# 固定5个时间偏移×4个速度×3个方向，共60组，不随机打乱或挑选成功组。
def run_matrix():
    return [dict(speed_label=label,**measure_case(offset,speed,axis))
        for offset,(label,speed),axis in itertools.product(OFFSETS_MS,SPEEDS.items(),AXES)]
