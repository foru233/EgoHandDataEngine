# 双目校正：图像、预先检测像素、投影矩阵必须采用同一套几何变换。

from dataclasses import replace
import cv2
import numpy as np
from .validator import validate_calibration, CalibrationError

# 只支持明确的 pinhole 或 OpenCV 四参数 fisheye；FISHEYE624 先走 HOT3D 官方适配。
class StereoRectifier:
    def __init__(self, calibration):
        self.c = validate_calibration(calibration)
        c = self.c
        size = (c.image_width,c.image_height)
        args = (c.K_left,c.D_left,c.K_right,c.D_right,size,c.R_right_left,c.t_right_left)
        # 不同相机模型采用不同 OpenCV API，不能仅因都叫鱼眼就复用畸变参数。
        if c.distortion_model == "fisheye":
            self.R1,self.R2,self.P1,self.P2,self.Q = cv2.fisheye.stereoRectify(*args,flags=cv2.CALIB_ZERO_DISPARITY,newImageSize=size,balance=0.0,fov_scale=1.0)
            self.roi_left = self.roi_right = None  # fisheye API supplies no valid ROI
            mapper = cv2.fisheye.initUndistortRectifyMap
        else:
            self.R1,self.R2,self.P1,self.P2,self.Q,self.roi_left,self.roi_right = cv2.stereoRectify(*args,flags=cv2.CALIB_ZERO_DISPARITY,alpha=c.rectification_alpha)
            mapper = cv2.initUndistortRectifyMap
        self.new_K_left = self.P1[:,:3].copy()
        self.new_K_right = self.P2[:,:3].copy()
        self.maps_left = mapper(c.K_left,c.D_left,self.R1,self.P1,size,cv2.CV_32FC1)
        self.maps_right = mapper(c.K_right,c.D_right,self.R2,self.P2,size,cv2.CV_32FC1)
        self._remap_left = cv2.convertMaps(*self.maps_left, cv2.CV_16SC2)
        self._remap_right = cv2.convertMaps(*self.maps_right, cv2.CV_16SC2)
        # 检测前各相机独立去畸变；检测后再以针孔单应矩阵转换到极线校正坐标。
        self.undistort_maps = [mapper(K,D,np.eye(3),K,size,cv2.CV_32FC1)
            for K,D in ((c.K_left,c.D_left),(c.K_right,c.D_right))]
        self.undistorted_to_rectified = [P[:,:3]@R@np.linalg.inv(K)
            for K,R,P in ((c.K_left,self.R1,self.P1),(c.K_right,self.R2,self.P2))]
        # 根据 P2 的平移投影选择水平/垂直视差轴，另一维是极线比较轴。
        self.disparity_axis = int(abs(self.P2[1,3]) > abs(self.P2[0,3]))
        self.epipolar_axis = 1-self.disparity_axis

    # 只转换有限观测；缺失点保留 NaN；输出像素与该侧 P1/P2 对应。
    def points(self, uv, side):
        uv = np.asarray(uv,dtype=np.float64)
        out = np.full_like(uv,np.nan)
        finite = np.isfinite(uv).all(axis=1)
        if finite.any():
            K,D,R,P = (self.c.K_left,self.c.D_left,self.R1,self.P1) if side == "left" else (self.c.K_right,self.c.D_right,self.R2,self.P2)
            fn = cv2.fisheye.undistortPoints if self.c.distortion_model == "fisheye" else cv2.undistortPoints
            out[finite] = fn(uv[finite].reshape(-1,1,2),K,D,R=R,P=P).reshape(-1,2)
        return out

    # raw 图像进行重采样；已校正帧须带一致 rectification_id，避免二次校正。
    def remap_pair(self, left, right):
        return (cv2.remap(left, *self._remap_left, cv2.INTER_LINEAR),
                cv2.remap(right, *self._remap_right, cv2.INTER_LINEAR))

    def rectify(self, frame):
        c = self.c
        for img in (frame.left_image,frame.right_image):
            if img is None or img.ndim != 3 or img.shape[2] != 3 or img.dtype != np.uint8:
                raise CalibrationError("图像必须为 uint8 BGR 三通道")
            validate_calibration(c,(img.shape[1],img.shape[0]))
        if frame.calibration_id != c.calibration_id:
            raise CalibrationError("帧与标定 ID 不一致")
        if frame.image_geometry == "rectified":
            if frame.metadata.get("rectification_id") != c.calibration_id:
                raise CalibrationError("已校正图像缺少一致的 rectification_id")
            return frame
        if frame.image_geometry != "raw":
            raise CalibrationError("未知图像几何类型")
        # 预计算的关键点也要和图像一起校正；仅校正图像会让三角化使用错误坐标。
        def detections(ds, side):
            if ds is None:
                return None
            result=[]
            for d in ds:
                uv=self.points(d.pixel_landmarks,side)
                normalized=np.column_stack((uv[:,0]/c.image_width,uv[:,1]/c.image_height))
                result.append(replace(d,pixel_landmarks=uv,normalized_landmarks=normalized))
            return result
        left, right = self.remap_pair(frame.left_image, frame.right_image)
        return replace(frame,left_image=left,right_image=right,image_geometry="rectified",metadata={**frame.metadata,"rectification_id":c.calibration_id},left_detections=detections(frame.left_detections,"left"),right_detections=detections(frame.right_detections,"right"))

    # 在校正像平面比较非视差轴差值，单位 px，不是米制位置误差。
    def epipolar_errors(self, left_uv, right_uv):
        return np.abs(np.asarray(left_uv)[:,self.epipolar_axis]-np.asarray(right_uv)[:,self.epipolar_axis])

    def detection_image(self, frame, side):
        """模型使用独立去畸变图；对已校正帧逆变换到相应针孔图。"""
        i=0 if side=='left' else 1
        image=(frame.left_image,frame.right_image)[i]
        if frame.image_geometry=='raw':
            return cv2.remap(image,*self.undistort_maps[i],cv2.INTER_LINEAR)
        if frame.image_geometry=='rectified':
            return cv2.warpPerspective(image,np.linalg.inv(self.undistorted_to_rectified[i]),
                (self.c.image_width,self.c.image_height))
        raise CalibrationError('检测输入的图像几何类型未知')

    def rectified_roi_to_detection(self, roi, side):
        """把显示校正图 ROI 的四角反变换，取去畸变图内的保守包围框。"""
        if roi is None:return None
        w,h=self.c.image_width,self.c.image_height;x,y,rw,rh=roi
        corners=np.array([[x*w,y*h],[(x+rw)*w,y*h],[(x+rw)*w,(y+rh)*h],[x*w,(y+rh)*h]],np.float64)
        H=np.linalg.inv(self.undistorted_to_rectified[0 if side=='left' else 1])
        p=np.column_stack((corners,np.ones(4)))@H.T
        if not np.isfinite(p).all() or np.min(p[:,2])*np.max(p[:,2])<=0:raise CalibrationError('ROI 无法映射到去畸变检测图')
        xy=p[:,:2]/p[:,2,None];lo=np.maximum(xy.min(axis=0),0);hi=np.minimum(xy.max(axis=0),[w,h])
        if np.any(hi-lo<2):raise CalibrationError('ROI 不在去畸变检测图范围内')
        return (lo[0]/w,lo[1]/h,(hi[0]-lo[0])/w,(hi[1]-lo[1])/h)

    def detection_to_rectified(self, detections, side):
        """检测关键点按相机旋转变换，与输出图像及投影矩阵同坐标。"""
        H=self.undistorted_to_rectified[0 if side=='left' else 1]
        w,h=self.c.image_width,self.c.image_height;result=[]
        for d in detections:
            uv=np.asarray(d.pixel_landmarks,dtype=float);p=np.column_stack((uv,np.ones(len(uv))))@H.T
            with np.errstate(divide='ignore',invalid='ignore'):mapped=p[:,:2]/p[:,2,None]
            valid=d.valid&np.isfinite(mapped).all(axis=1)&(p[:,2]>0)&(mapped>=0).all(axis=1)&(mapped<[w,h]).all(axis=1)
            normalized=d.normalized_landmarks.copy();normalized[:,:2]=mapped/[w,h]
            finite=mapped[np.isfinite(mapped).all(axis=1)]
            box=(*finite.min(axis=0),*finite.max(axis=0)) if len(finite) else None
            result.append(replace(d,pixel_landmarks=mapped,normalized_landmarks=normalized,valid=valid,bounding_box=box))
        return result
