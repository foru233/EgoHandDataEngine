# 三角化与逐点几何门控；输入为校正后像素，对外输出原始左光学坐标、米。
# 调参入口为 configs/stereo.yaml 的几何阈值和 uncertainty，不在此处散改阈值。

import cv2
import numpy as np
import time
from ..core.types import Joint3D
from ..core.status import JointStatus as S
from ..detection.landmarks import JOINT_NAMES
from .uncertainty import estimate_uncertainty,DEFAULT_UNCERTAINTY

# 只接收左右二维关键点；不读取 GT XYZ 或模型 world_landmarks 来填充结果。
class JointTriangulator:
    def __init__(self, rectifier, config):
        self.r,self.cfg=rectifier,config
        self.elapsed_ms=0.0

    def triangulate(self,left,right):
        started=time.perf_counter()
        r,cfg=self.r,self.cfg
        l,q=left.pixel_landmarks,right.pixel_landmarks
        size=np.array([r.c.image_width,r.c.image_height])
        inside=lambda p: np.isfinite(p).all(axis=1) & (p>=0).all(axis=1) & (p<size).all(axis=1)
        # 两侧都有效、有限且在图像内的同索引关节才参与三角化；缺一侧就不生成三维。
        usable=left.valid & right.valid & inside(l) & inside(q)
        indices=np.flatnonzero(usable)
        X=np.full((21,3),np.nan)
        finite_h=np.zeros(21,dtype=bool)
        if len(indices):
            # 得到齐次坐标 [x,y,z,w]；先检查有限性和 w，再除以 w，防止退化几何溢出。
            H=cv2.triangulatePoints(r.P1,r.P2,l[indices].T.copy(),q[indices].T.copy()).T
            ok=np.isfinite(H).all(axis=1) & (np.abs(H[:,3])>1e-12)
            X[indices[ok]]=H[ok,:3]/H[ok,3,None]
            finite_h[indices[ok]]=True
        # 行向量写法：X_rect @ R1 等价于列向量 R1.T @ X_rect，回到原始左相机坐标。
        original=X@r.R1
        # 外参方向固定为 p_right = R_right_left @ p_left + t_right_left；两路都检查正深度。
        right_xyz=original@r.c.R_right_left.T+r.c.t_right_left.reshape(1,3)
        def project(P):
            hp=np.column_stack((X,np.ones(21)))@P.T
            out=np.full((21,2),np.nan)
            ok=np.isfinite(hp).all(axis=1)&(np.abs(hp[:,2])>1e-12)
            out[ok]=hp[ok,:2]/hp[ok,2,None]
            return out
        # 把重建点重新投影到两视图，量度与输入像素的距离；系统性视差偏差仍可能残差很小。
        re_l=np.linalg.norm(project(r.P1)-l,axis=1)
        re_r=np.linalg.norm(project(r.P2)-q,axis=1)
        epi=r.epipolar_errors(l,q)
        # 视差轴由校正器判断，兼容水平与垂直基线，不能无条件只比较 u。
        disparity=l[:,r.disparity_axis]-q[:,r.disparity_axis]
        joints=[]
        ucfg={**DEFAULT_UNCERTAINTY,**cfg.get('uncertainty',{})}
        if ucfg['mode'] not in ('diagnostic','reject'):raise ValueError('uncertainty.mode must be diagnostic or reject')
        # 先筛掉已经几何无效的点，再传播协方差；不会因计算协方差而改变旧几何结果。
        use_epi=cfg.get('use_epipolar_error',True)
        use_reproj=cfg.get('use_reprojection_error',True)
        min_depth,max_depth=cfg.get('min_depth_m'),cfg.get('max_depth_m')
        depth_in_range=((original[:,2]>=min_depth) if min_depth is not None else np.ones(21,dtype=bool)) & ((original[:,2]<=max_depth) if max_depth is not None else np.ones(21,dtype=bool))
        geometry_valid=(usable&finite_h&((epi<=cfg['max_epipolar_px']) if use_epi else True)&
            (np.abs(disparity)>=cfg['min_disparity_px'])&(original[:,2]>0)&(right_xyz[:,2]>0)&
            ((np.maximum(re_l,re_r)<=cfg['max_reprojection_px']) if use_reproj else True)&
            depth_in_range)
        # Avoid propagating covariance for already rejected candidate geometry.
        # This does not change any XYZ, quality score, or geometry decision.
        uncertainty=estimate_uncertainty(r.P1,r.P2,np.where(geometry_valid[:,None],l,np.nan),
            np.where(geometry_valid[:,None],q,np.nan),r.R1,ucfg)
        if ucfg['enabled']:
            for i in np.flatnonzero(~geometry_valid):uncertainty[i].status='GEOMETRY_REJECTED'
        # 拒绝顺序决定首个状态码；按 INVALID→极线→视差→深度/重投影等顺序记录。
        for i,name in enumerate(JOINT_NAMES):
            status=S.OBSERVED
            if not usable[i]: status=S.INVALID
            elif use_epi and epi[i]>cfg["max_epipolar_px"]: status=S.EPIPOLAR
            elif abs(disparity[i])<cfg["min_disparity_px"]: status=S.DISPARITY
            elif not finite_h[i]: status=S.INVALID
            elif original[i,2]<=0 or right_xyz[i,2]<=0: status=S.NEGATIVE
            elif use_reproj and max(re_l[i],re_r[i])>cfg["max_reprojection_px"]: status=S.REPROJECTION
            elif not depth_in_range[i]: status=S.DEPTH
            valid=status==S.OBSERVED
            error=float(max(re_l[i],re_r[i])) if finite_h[i] else None
            # Derived geometric score, not a model confidence.
            quality=float(np.exp(-.5*((((epi[i]/cfg["max_epipolar_px"])**2) if use_epi else 0)+(((error/cfg["max_reprojection_px"])**2) if use_reproj else 0)))) if valid and (use_epi or use_reproj) else None
            joints.append(Joint3D(i,name,original[i].copy() if valid else np.full(3,np.nan),valid,status,tuple(l[i]) if np.isfinite(l[i]).all() else None,tuple(q[i]) if np.isfinite(q[i]).all() else None,float(epi[i]) if np.isfinite(epi[i]) else None,error,quality))
            joint=joints[-1];u=uncertainty[i]
            joint.depth_std_m=u.depth_std_m;joint.position_std_m=u.position_std_m
            joint.position_covariance_m2=u.position_covariance_m2;joint.uncertainty_status=u.status
            # 默认 diagnostic 不额外拒绝；只有显式 reject 才按深度/主轴 std 门限作 HIGH_UNCERTAINTY 拒绝。
            if valid and ucfg['enabled'] and ucfg['mode']=='reject':
                unavailable=u.position_std_m is None or u.depth_std_m is None
                high=(not unavailable and (u.depth_std_m>ucfg['max_depth_std_m'] or u.position_std_m>ucfg['max_position_std_m']))
                if high or (unavailable and ucfg['reject_unavailable']):
                    joint.valid=False;joint.xyz_m[:]=np.nan;joint.status=S.UNCERTAINTY;joint.quality_score=None
        self.elapsed_ms+=(time.perf_counter()-started)*1000
        return joints
