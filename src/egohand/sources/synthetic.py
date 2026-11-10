# 合成源仅用于可重复软件/几何测试；GT 用来生成二维观测和事后比较，不送入重建核心。

import cv2
import numpy as np
from .base import StereoDataSource
from ..calibration.schema import synthetic_calibration
from ..core.types import SourceMetadata, StereoFrame, HandDetection
from ..detection.landmarks import BONES

SCENARIOS = ("single_static","approach","recede","two_static","crossing","different_depth","missing_joints","inconsistent_count","wrong_correspondence","zero_disparity","invalid_calibration","timestamp_anomaly")

# 生成固定关节顺序的测试手形，单位米；这不是训练模型预测。
def hand_shape():
    p = np.zeros((21,3),dtype=np.float64)
    p[1:5] = [[-.022,-.014,0],[-.038,-.03,-.003],[-.052,-.041,-.006],[-.065,-.052,-.01]]
    for start,x,length in ((5,-.025,.075),(9,0,.085),(13,.022,.077),(17,.042,.062)):
        for k in range(4):
            p[start+k]=[x*(1+.06*k),-.04-k*length/3,.003*np.sin(k)]
    return p

# 显式标记 synthetic 来源，防止测试画面被当作真实设备测量。
class SyntheticStereoSource(StereoDataSource):
    def __init__(self, scenario="crossing", frames=300, fps=30, noise_px=0.0, seed=42):
        if scenario not in SCENARIOS:
            raise ValueError(f"未知模拟场景：{scenario}")
        self.scenario,self.frames,self.fps,self.noise_px,self.seed=scenario,frames,fps,noise_px,seed
        self.calibration=synthetic_calibration()
        if scenario == "invalid_calibration":
            self.calibration.t_right_left=np.zeros(3)
        self.index=0
        self.running=False

    def open(self):
        self.index=0
    def start(self):
        self.running=True
    def stop(self):
        self.running=False
    def close(self):
        self.stop()
    def seek(self,index):
        if not 0 <= index < self.frames:
            raise IndexError(index)
        self.index=index
    def get_metadata(self):
        return SourceMetadata("synthetic",synchronized=True,metric_capable=True,ground_truth=True,synthetic=True,synchronization_evidence="synthetic_projection",timestamp_kind="synthetic",extra={"scenario":self.scenario,"fps":self.fps,"frame_count":self.frames})
    def get_calibration(self):
        return self.calibration
    def read(self):
        if not self.running or self.index>=self.frames:
            return None
        i=self.index
        self.index+=1
        t=i/self.fps
        count=1 if self.scenario in ("single_static","approach","recede") else 2
        c=self.calibration
        images=[np.full((c.image_height,c.image_width,3),(27,23,20),np.uint8) for _ in range(2)]
        detections=[[],[]]
        truth=[]
        rng=np.random.default_rng(self.seed+i)
        timestamp=1_000_000_000+round(t*1e9)
        if self.scenario=="timestamp_anomaly" and i==5:
            timestamp-=round(2e9/self.fps)
        for h in range(count):
            xyz=hand_shape().copy()
            if h:
                xyz[:,0]*=-1
            x=(-.105 if h==0 else .105) if count==2 else 0
            y=.055+(h*.025)
            z=.65
            if self.scenario=="crossing":
                x=(1 if h else -1)*.13*np.cos(t*1.1)
                z=.60+h*.14
            if self.scenario=="approach": z=.95-.35*(.5-.5*np.cos(t))
            if self.scenario=="recede": z=.55+.35*(.5-.5*np.cos(t))
            if self.scenario=="different_depth": z=.48+h*.42
            xyz += [x,y,z]
            truth.append(xyz)
            for side in range(2):
                R=np.eye(3) if side==0 else c.R_right_left
                T=np.zeros(3) if side==0 else c.t_right_left
                K,D=(c.K_left,c.D_left) if side==0 else (c.K_right,c.D_right)
                if c.distortion_model=="fisheye":
                    uv=cv2.fisheye.projectPoints(xyz.reshape(-1,1,3),cv2.Rodrigues(R)[0],T,K,D)[0].reshape(21,2)
                else:
                    uv=cv2.projectPoints(xyz,cv2.Rodrigues(R)[0],T,K,D)[0].reshape(21,2)
                if self.scenario=="zero_disparity" and side:
                    uv=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),c.K_left,c.D_left)[0].reshape(21,2)
                uv += rng.normal(0,self.noise_px,uv.shape)
                valid=np.ones(21,dtype=bool)
                if self.scenario=="missing_joints": valid[[4,8,12]]=False; uv[~valid]=np.nan
                if self.scenario=="wrong_correspondence" and side: uv=uv[np.roll(np.arange(21),5)]
                d=HandDetection("Left" if h==0 else "Right",1.0,uv/[c.image_width,c.image_height],uv,timestamp_ns=timestamp,valid=valid,origin="synthetic_projection")
                detections[side].append(d)
                col=(90,205,75) if h==0 else (65,165,250)
                for a,b in BONES:
                    if valid[a] and valid[b]: cv2.line(images[side],tuple(uv[a].astype(int)),tuple(uv[b].astype(int)),col,2,cv2.LINE_AA)
                for p in uv[valid]: cv2.circle(images[side],tuple(p.astype(int)),3,col,-1,cv2.LINE_AA)
        if self.scenario=="inconsistent_count": detections[1]=detections[1][:1]
        # Deliberately vary detector order. Ground truth is never passed to reconstruction.
        if i%2: detections[1].reverse()
        for img in images: cv2.putText(img,"SYNTHETIC GEOMETRY | NOT CAMERA MEASUREMENT",(12,25),cv2.FONT_HERSHEY_SIMPLEX,.43,(100,200,255),1,cv2.LINE_AA)
        return StereoFrame(i,timestamp,*images,timestamp,timestamp,c.calibration_id,"synthetic",True,"raw",{"synthetic":True,"synchronization_evidence":"synthetic_projection","timestamp_kind":"synthetic","ground_truth_xyz_m":np.array(truth),"scenario":self.scenario},*detections)
