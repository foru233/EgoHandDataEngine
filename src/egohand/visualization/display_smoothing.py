"""仅稳定预览的观测，不改动检测、三角化、诊断、录制或导出结果。"""
from collections import Counter
from dataclasses import replace
import numpy as np

# 较低截止频率更稳但更滞后；beta 控制运动时截止频率的自适应增益。
DEFAULT_DISPLAY_SMOOTHING={
    'min_cutoff_hz':1.2, 'derivative_cutoff_hz':1.0,
    'beta_2d':0.01, 'beta_3d':1.0, 'max_gap_s':0.35,
}

class DisplaySmoother:
    def __init__(self,config=None):
        self.cfg={**DEFAULT_DISPLAY_SMOOTHING,**(config or {})};self.states={};self.session=None
    def reset(self):self.states.clear();self.session=None
    @staticmethod
    def _alpha(cutoff,dt):
        r=2*np.pi*np.asarray(cutoff)*dt
        return r/(1+r)
    def _points(self,key,points,valid,ts,beta):
        points=np.asarray(points,dtype=float);valid=np.asarray(valid,dtype=bool)&np.isfinite(points).all(axis=1)
        old=self.states.get(key);out=points.copy()
        dt=(ts-old['ts'])/1e9 if old is not None else 0
        if old is None or dt<=0 or dt>self.cfg['max_gap_s'] or old['raw'].shape!=points.shape:
            velocity=np.zeros_like(points)
        else:
            common=valid & old['valid'];velocity=np.zeros_like(points)
            raw_velocity=(points[common]-old['raw'][common])/dt
            ad=self._alpha(self.cfg['derivative_cutoff_hz'],dt)
            velocity[common]=ad*raw_velocity+(1-ad)*old['velocity'][common]
            cutoff=self.cfg['min_cutoff_hz']+beta*np.linalg.norm(velocity[common],axis=1)
            a=self._alpha(cutoff,dt)[:,None]
            out[common]=a*points[common]+(1-a)*old['filtered'][common]
        self.states[key]={'raw':points.copy(),'filtered':out.copy(),'valid':valid.copy(),'velocity':velocity,'ts':ts}
        return out
    def update(self,result):
        session=(result.session_id,result.source_type,result.calibration_id)
        if session!=self.session:self.reset();self.session=session
        # 未观测到的点立即失效；真值和合成投影不做预览滤波。
        if result.diagnostics.get('synthetic') or result.diagnostics.get('result_kind') in ('official_ground_truth','ground_truth_unavailable'):
            self.states.clear();return result
        timestamp=result.timestamp_ns
        detections=[]
        for side,items in (('left',result.left_detections),('right',result.right_detections)):
            counts=Counter(d.handedness for d in items);copies=[]
            for d in items:
                if d.handedness not in ('Left','Right') or counts[d.handedness]!=1:
                    copies.append(d);continue
                uv=self._points(('2d',side,d.handedness),d.pixel_landmarks,d.valid,timestamp,self.cfg['beta_2d'])
                copies.append(replace(d,pixel_landmarks=uv))
            detections.append(copies)
        hands=[]
        counts=Counter(h.handedness for h in result.hands if h.valid_joint_count and h.tracking_status=='frame_only')
        for h in result.hands:
            frame_key=h.tracking_status=='frame_only' and h.handedness in ('Left','Right') and counts[h.handedness]==1
            if not h.valid_joint_count or (h.track_id<=0 and not frame_key):
                hands.append(h);continue
            xyz=np.array([j.xyz_m for j in h.joints],float);valid=np.array([j.valid for j in h.joints],bool)
            key=('3d',h.track_id) if h.track_id>0 else ('3d-frame',h.handedness)
            smooth=self._points(key,xyz,valid,timestamp,self.cfg['beta_3d'])
            hands.append(replace(h,joints=[replace(j,xyz_m=smooth[i]) if valid[i] else j for i,j in enumerate(h.joints)]))
        # 按时间清除消失的身份，防止轨迹长期积累。
        self.states={k:v for k,v in self.states.items() if 0<=(timestamp-v['ts'])/1e9<=self.cfg['max_gap_s']}
        return replace(result,left_detections=detections[0],right_detections=detections[1],hands=hands,
                       diagnostics={**result.diagnostics,'display_smoothed':True})
