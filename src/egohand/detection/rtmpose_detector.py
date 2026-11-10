"""RTMDet 手框 + RTMPose-Hand：在校正图上输出 21 个二维点。"""
from pathlib import Path
import hashlib
import numpy as np
from .roi import validate_roi,roi_pixel_box
from ..core.types import HandDetection

ROOT=Path(__file__).resolve().parents[3]
DET_MODEL=ROOT/'models/rtmdet_hand.onnx'
POSE_MODEL=ROOT/'models/rtmpose_hand.onnx'

class RTMPoseHandDetector:
    def __init__(self,mode='IMAGE',num_hands=2,box_threshold=.3,keypoint_threshold=.3,roi=None):
        from rtmlib import RTMDet,RTMPose
        for p in (DET_MODEL,POSE_MODEL):
            if not p.is_file():raise FileNotFoundError('RTMPose 模型缺失：'+str(p))
        self.det=RTMDet(str(DET_MODEL),model_input_size=(320,320),backend='onnxruntime',device='cpu',score_thr=box_threshold)
        self.pose=RTMPose(str(POSE_MODEL),model_input_size=(256,256),backend='onnxruntime',device='cpu')
        self.sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (DET_MODEL,POSE_MODEL)}
        self.num_hands=num_hands;self.keypoint_threshold=keypoint_threshold;self.roi=validate_roi(roi)
        self.last_detection_diagnostics={};self.closed=False;self.rescue_config={}
    def set_roi(self,roi):self.roi=validate_roi(roi)
    def detect(self,image,timestamp_ns=0):
        if self.closed:raise RuntimeError('检测器已关闭')
        height,width=image.shape[:2];x,y,w,h=roi_pixel_box(self.roi,image.shape)
        crop=np.ascontiguousarray(image[y:y+h,x:x+w]);boxes=self.det(crop)
        # RTMPose 没有框时默认使用全图，必须显式阻止，避免将空场景当成手。
        if len(boxes)==0:
            self.last_detection_diagnostics={'method':'rtmdet_then_rtmpose','boxes':0,'manual_roi':list(self.roi) if self.roi else None}
            return []
        boxes=boxes[:self.num_hands];points,scores=self.pose(crop,bboxes=boxes)
        out=[]
        for uv,score,box in zip(points,scores,boxes):
            uv=np.asarray(uv,dtype=float).reshape(21,2)+[x,y];score=np.asarray(score,dtype=float).reshape(21)
            valid=np.isfinite(uv).all(axis=1)&(score>=self.keypoint_threshold)&(uv>=0).all(axis=1)&(uv<[width,height]).all(axis=1)
            out.append(HandDetection('Unknown',0.,uv/[width,height],uv,valid=valid,bounding_box=tuple(np.asarray(box[:4])+[x,y,x,y]),timestamp_ns=timestamp_ns,origin='rtmpose_hand'))
        self.last_detection_diagnostics={'method':'rtmdet_then_rtmpose','boxes':len(boxes),'manual_roi':list(self.roi) if self.roi else None,'handedness':'unavailable'}
        return out
    def close(self):self.closed=True;self.det=None;self.pose=None
