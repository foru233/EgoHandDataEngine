"""对比校正图的视觉旋转与 IMU 候选角速度；仅保存数值，不保存图像。"""
import json
import queue
import time
from pathlib import Path
import cv2
import numpy as np
from egohand.sources.vendor_xdsdk import XdSdkStereoSource
from egohand.vio.vs030_imu_candidate import parse_candidate_payload
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry

ROOT=Path(__file__).resolve().parents[1]
DURATION_SECONDS=35
FRAME_INTERVAL_SECONDS=.12
MAX_CORNERS=500


def main():
    geo=prepare_vs030_vio_geometry(ROOT/'标定结果/1/data3-camchain-imucam.yaml')
    rect=geo.rectifier;scale=.5
    maps=tuple(cv2.resize(m,None,fx=scale,fy=scale,interpolation=cv2.INTER_LINEAR) for m in rect.maps_left)
    K=rect.P1[:3,:3].copy();K[:2,:]*=scale
    gyro_to_camera=geo.T_imu_from_orb_left[:3,:3].T
    source=XdSdkStereoSource();source.enable_imu_capture();imu=[];visual=[];previous=None;error=None
    try:
        source.open();source.start();start=time.monotonic();end=start+DURATION_SECONDS;last=0
        print('CAPTURE_READY',flush=True)
        while time.monotonic()<end:
            frame=source.read()
            if frame is None:break
            while True:
                try:
                    item=source.imu_vio_queue.get_nowait()
                    if len(item)==4: _,_,_,packet=item
                    elif len(item)==2: _,packet=item
                    else: raise ValueError('SDK IMU queue record has an unsupported layout')
                    imu.extend(parse_candidate_payload(packet))
                except queue.Empty:break
            ts=int(frame.metadata['sdk_device_timestamp_raw'])*1000
            if ts-last<int(FRAME_INTERVAL_SECONDS*1e9):continue
            gray=cv2.cvtColor(frame.right_image,cv2.COLOR_BGR2GRAY)
            gray=cv2.remap(gray,*maps,cv2.INTER_LINEAR)
            if previous:
                last_t,last_image,last_points=previous
                dt=(ts-last_t)*1e-9
                points,status,_=cv2.calcOpticalFlowPyrLK(last_image,gray,last_points,None)
                if points is not None and status is not None and .03<dt<.5:
                    valid=status.reshape(-1).astype(bool)
                    p0=last_points.reshape(-1,2)[valid];p1=points.reshape(-1,2)[valid]
                    if len(p0)>=30:
                        H,inliers=cv2.findHomography(p0,p1,cv2.RANSAC,2.0)
                        if H is not None and inliers is not None and int(inliers.sum())>=25:
                            M=np.linalg.inv(K)@H@K
                            if np.linalg.det(M)>0:
                                U,_,V=np.linalg.svd(M);R=U@V
                                if np.linalg.det(R)>0:
                                    rv,_=cv2.Rodrigues(R)
                                    visual.append({'t_ns':(last_t+ts)//2,'dt_s':dt,'features':int(inliers.sum()),
                                                   'optical_angular_velocity':(-rv[:,0]/dt).tolist()})
            corners=cv2.goodFeaturesToTrack(gray,MAX_CORNERS,.01,12)
            previous=(ts,gray,corners) if corners is not None and len(corners)>=30 else None
            last=ts
    except Exception as exc:error=str(exc)
    finally:source.close()
    if len(imu)<10 or len(visual)<10:raise RuntimeError(f'数据不足：imu={len(imu)} visual={len(visual)} error={error}')
    times=np.array([r.time_b_raw for r in imu],np.int64)
    gyros=np.array([r.values[3:] for r in imu],float)@gyro_to_camera.T
    vt=np.array([r['t_ns'] for r in visual],np.int64)
    vv=np.array([r['optical_angular_velocity'] for r in visual])
    candidates=[]
    for lag_ms in range(-350,351,5):
        target=vt+lag_ms*1_000_000
        valid=(target>=times[0])&(target<=times[-1])
        if valid.sum()<10:continue
        g=np.stack([np.interp(target[valid],times,gyros[:,axis]) for axis in range(3)],axis=1)
        a=vv[valid];a=a-a.mean(axis=0);g=g-g.mean(axis=0)
        denom=np.linalg.norm(a)*np.linalg.norm(g)
        if denom>1e-9:candidates.append({'lag_ms':lag_ms,'correlation':float(np.sum(a*g)/denom),'samples':int(valid.sum())})
    best=max(candidates,key=lambda x:x['correlation']) if candidates else None
    report={'duration_requested_s':DURATION_SECONDS,'visual_count':len(visual),'imu_count':len(imu),
        'visual_angular_std':vv.std(axis=0).tolist(),'gyro_camera_std':gyros.std(axis=0).tolist(),
        'best_lag':best,'zero_lag':next((x for x in candidates if x['lag_ms']==0),None),
        'old_kalibr_lag':min(candidates,key=lambda x:abs(x['lag_ms']+225)) if candidates else None,
        'visual':visual,'error':error,'sdk_diagnostics':source.sdk_diagnostics,
        'note':'单应旋转受平移、景深和图像质量影响；该相关性仅用于排查，不作为时延标定真值。'}
    path=ROOT/'results/vio/vs030_gyro_diagnostic.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print('RESULT',json.dumps({k:v for k,v in report.items() if k!='visual'},ensure_ascii=True),flush=True)

if __name__=='__main__':main()
