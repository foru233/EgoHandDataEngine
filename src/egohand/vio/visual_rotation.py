"""Independent short-baseline stereo rotation reference for time calibration.

This does not produce a SLAM/world pose. Failed geometry stays unavailable.
No IMU predictions, vocabulary, map optimization or display smoothing are used.
"""
import cv2
import numpy as np


def solve_relative_rotation(points3d, pixels, camera, *, min_inliers=30):
    points=np.asarray(points3d,dtype=np.float64).reshape(-1,3)
    pixels=np.asarray(pixels,dtype=np.float64).reshape(-1,2)
    if len(points)!=len(pixels) or len(points)<min_inliers:
        return {'valid':False,'reason':'insufficient_correspondences'}
    valid=np.isfinite(points).all(axis=1)&np.isfinite(pixels).all(axis=1)&(points[:,2]>0)
    points,pixels=points[valid],pixels[valid]
    if len(points)<min_inliers:
        return {'valid':False,'reason':'insufficient_positive_stereo_depth'}
    success,rv,tv,inliers=cv2.solvePnPRansac(points,pixels,camera,None,
        iterationsCount=150,reprojectionError=2.0,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
    if not success or inliers is None or len(inliers)<min_inliers:
        return {'valid':False,'reason':'visual_ransac_unavailable'}
    indices=inliers.ravel()
    rv,tv=cv2.solvePnPRefineLM(points[indices],pixels[indices],camera,None,rv,tv)
    rotation=cv2.Rodrigues(rv)[0]
    transformed=points[indices]@rotation.T+tv.reshape(3)
    projected=cv2.projectPoints(points[indices],rv,tv,camera,None)[0].reshape(-1,2)
    errors=np.linalg.norm(projected-pixels[indices],axis=1)
    if (not np.isfinite(rotation).all() or not np.isfinite(tv).all()
            or np.mean(transformed[:,2]>0)<.95 or np.percentile(errors,95)>2.0):
        return {'valid':False,'reason':'visual_geometry_unverified'}
    return {'valid':True,'reason':'verified_visual_geometry',
            'previous_from_current_rotation':rotation.T,'inliers':len(indices),
            'reprojection_p95_px':float(np.percentile(errors,95))}


class StereoRotationReference:
    def __init__(self,camera,baseline_m):
        self.camera=np.asarray(camera,dtype=np.float64)
        self.baseline=float(baseline_m)
        self.previous=None
        self.lk=dict(winSize=(21,21),maxLevel=3,
                     criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,.01))

    def _reference(self,stamp,left,right):
        corners=cv2.goodFeaturesToTrack(left,500,.01,8)
        if corners is None:
            return None
        target,status,_=cv2.calcOpticalFlowPyrLK(left,right,corners,None,**self.lk)
        if target is None:return None
        back,back_status,_=cv2.calcOpticalFlowPyrLK(right,left,target,None,**self.lk)
        if back is None:return None
        a,b=corners[:,0,:],target[:,0,:]
        disparity=a[:,0]-b[:,0]
        good=(status.ravel()!=0)&(back_status.ravel()!=0)
        good&=np.linalg.norm(back[:,0,:]-a,axis=1)<=1.0
        good&=np.abs(a[:,1]-b[:,1])<=1.5
        good&=(disparity>0)&(b[:,0]>=0)&(b[:,0]<left.shape[1])
        a,disparity=a[good],disparity[good]
        if len(a)<30:return None
        z=self.camera[0,0]*self.baseline/disparity
        xyz=np.column_stack(((a[:,0]-self.camera[0,2])*z/self.camera[0,0],
                             (a[:,1]-self.camera[1,2])*z/self.camera[1,1],z))
        return (int(stamp),left.copy(),a.reshape(-1,1,2),xyz)

    def process(self,stamp,left,right):
        if left.dtype!=np.uint8 or right.dtype!=np.uint8 or left.ndim!=2 or left.shape!=right.shape:
            raise ValueError('Time calibration needs matching rectified uint8 stereo images')
        old=self.previous
        self.previous=self._reference(stamp,left,right)
        if old is None:
            return {'valid':False,'reason':'waiting_for_textured_stereo_reference'}
        t0,image,points,xyz=old
        if not 0<int(stamp)-t0<=200_000_000:
            return {'valid':False,'reason':'visual_frame_gap'}
        target,status,_=cv2.calcOpticalFlowPyrLK(image,left,points,None,**self.lk)
        if target is None:return {'valid':False,'reason':'temporal_tracking_unavailable'}
        back,back_status,_=cv2.calcOpticalFlowPyrLK(left,image,target,None,**self.lk)
        if back is None:return {'valid':False,'reason':'temporal_tracking_unavailable'}
        good=(status.ravel()!=0)&(back_status.ravel()!=0)
        good&=np.linalg.norm(back[:,0,:]-points[:,0,:],axis=1)<=1.0
        result=solve_relative_rotation(xyz[good],target[good,0,:],self.camera)
        result.update(start_ns=t0,end_ns=int(stamp))
        return result
