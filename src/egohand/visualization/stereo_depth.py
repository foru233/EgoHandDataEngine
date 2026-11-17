"""Shared stereo depth computation for GUI preview and saved metric results."""
import cv2
import numpy as np

def compute_depth(frame,calibration,cache):
    """Return axial metres on the reference rectified camera's scaled grid."""
    rectifier,engine,scale,swapped,vertical=cache.acquire(calibration,frame)
    f=rectifier.rectify(frame)
    images=[cv2.resize(cv2.cvtColor(i,cv2.COLOR_BGR2GRAY),None,fx=scale,fy=scale)
            for i in (f.left_image,f.right_image)]
    if vertical:images=[np.ascontiguousarray(i.T) for i in images]
    if swapped:images=images[::-1]
    depth=engine.estimate(*images,min_depth_m=.01,max_depth_m=float('inf'))
    if vertical:depth=depth.T
    depth=np.ascontiguousarray(depth,dtype=np.float32)
    projection=rectifier.P2 if swapped else rectifier.P1
    intrinsics=projection[:,:3].copy()
    intrinsics[:2,:]*=scale
    intrinsics[0,2]=(projection[0,2]+.5)*scale-.5
    intrinsics[1,2]=(projection[1,2]+.5)*scale-.5
    info={'method':'stereo_sgbm_lr_consistency','unit':'m','quantity':'rectified_optical_z',
          'reference_camera':'right' if swapped else 'left','image_geometry':'rectified',
          'invalid_value':0,'dtype':'float32','shape':list(depth.shape),'scale':scale,
          'calibration_id':calibration.calibration_id,'disparity_axis':int(rectifier.disparity_axis),
          'K_depth':intrinsics.tolist(),'R_rectified_from_camera':(rectifier.R2 if swapped else rectifier.R1).tolist(),
          'baseline_m':float(np.linalg.norm(calibration.t_right_left)),
          'valid_fraction':float((depth>0).mean()),'num_disparities':cache._disparities}
    return depth,info

def colorize_depth(depth,near_m,far_m,filter_range=True):
    """Colorize display data; all-values mode saturates outside the color scale."""
    if not 0 < near_m < far_m:raise ValueError('深度范围的近端必须小于远端')
    valid=np.isfinite(depth)&(depth>0)
    if filter_range:valid &= (depth>=near_m)&(depth<=far_m)
    scaled=np.nan_to_num((far_m-depth)/(far_m-near_m),nan=0.,posinf=0.,neginf=0.)
    color=cv2.applyColorMap(np.uint8(np.clip(scaled,0,1)*255),cv2.COLORMAP_TURBO)
    color[~valid]=0
    return color,valid


class DepthCache:
    """按标定复用校正器与 SGBM 匹配器，避免每次刷新重建映射表。"""

    # 显示匹配器按 0.10 m 计算搜索宽度；有效距离还受视差和匹配质量限制。
    NEAR_M=.10

    def __init__(self):
        self._key=None;self._rectifier=None;self._depth=None
        self._scale=.25;self._swapped=False;self._vertical=False
        self._disparities=96

    def acquire(self,calibration,frame):
        key=(getattr(calibration,'calibration_id',None),getattr(frame,'calibration_id',None),
             int(getattr(frame.left_image,'shape',(0,0))[1] or 0))
        if key!=self._key:self._build(calibration,frame)
        return self._rectifier,self._depth,self._scale,self._swapped,self._vertical

    @staticmethod
    def required_disparities(fx, baseline, near_m=NEAR_M):
        """近端处需要的视差宽度；SGBM 要求是 16 的倍数。"""
        import math
        needed=fx*baseline/near_m
        return int(min(96,max(16,16*math.ceil(needed/16))))

    def _build(self,calibration,frame):
        import cv2
        from egohand.calibration.rectifier import StereoRectifier
        from egohand.vio.map_depth import StereoDepth
        rectifier=StereoRectifier(calibration)
        width=int(getattr(frame.left_image,'shape',(0,0))[1] or 0) or 1600
        scale=min(1.,360/width)
        projection=rectifier.P1.copy()
        vertical=rectifier.disparity_axis==1
        if vertical:projection[0,0]=rectifier.P1[1,1]
        swapped=rectifier.P2[rectifier.disparity_axis,3]>0
        baseline=float(np.linalg.norm(calibration.t_right_left))
        depth_engine=StereoDepth(projection,baseline,scale)
        # 仅对显示匹配器调整搜索窗口；其余地图和定位调用保留自己的默认参数。
        disparities=self.required_disparities(depth_engine.fx,baseline)
        if disparities!=96:
            depth_engine.matcher=cv2.StereoSGBM_create(
                minDisparity=0,numDisparities=disparities,blockSize=5,
                P1=200,P2=800,uniquenessRatio=12,speckleWindowSize=60,speckleRange=2)
        self._rectifier=rectifier;self._depth=depth_engine;self._disparities=disparities
        self._scale=scale;self._swapped=swapped;self._vertical=vertical
        self._key=(getattr(calibration,'calibration_id',None),getattr(frame,'calibration_id',None),
                   int(getattr(frame.left_image,'shape',(0,0))[1] or 0))
