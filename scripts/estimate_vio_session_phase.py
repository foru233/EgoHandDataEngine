"""Estimate a recording-specific phase from independent stereo PnP and gyro.

Writes a new diagnostic model; never edits source recordings or global timing.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from egohand.storage.sdk_recording import iter_paired_frames,iter_imu_packets
from egohand.vio.config_snapshot import resolve_recording_config
from egohand.vio.visual_rotation import StereoRotationReference
from egohand.vio.imu_lattice import PacketLatticeImuBuffer
from egohand.vio.session_phase import verify_session_phase
from audit_vio_rotation_phase import evaluate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',type=Path,required=True)
    p.add_argument('--config-snapshot',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--max-frames',type=int,default=300)
    p.add_argument('--start-frame',type=int,default=0,
        help='First decoded frame for a separate temporal consistency window')
    p.add_argument('--stride',type=int,default=2)
    a=p.parse_args()
    if a.start_frame<0 or a.max_frames<1 or a.stride<1:
        p.error('start-frame must be nonnegative; max-frames and stride must be positive')
    config=resolve_recording_config(a.config_snapshot or a.recording)
    geometry=config.validate_geometry();scale=config.image_scale
    camera=geometry.rectifier.P1[:3,:3].copy();camera[:2]*=scale
    baseline=-geometry.rectifier.P2[0,3]/geometry.rectifier.P2[0,0]
    reference=StereoRotationReference(camera,baseline)
    samples=[];reasons=Counter();quality=[]
    for index,frame in enumerate(iter_paired_frames(a.recording)):
        if index>=a.start_frame+a.max_frames:break
        if index<a.start_frame or (index-a.start_frame)%a.stride:continue
        left,right=geometry.rectifier.remap_pair(frame.right_bgr,frame.left_bgr)
        left,right=[cv2.resize(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),None,
                    fx=scale,fy=scale,interpolation=cv2.INTER_AREA) for im in [left,right]]
        result=reference.process(frame.device_ts*1000,left,right)
        reasons[result['reason']]+=1
        if result['valid']:
            samples.append((result['start_ns'],result['end_ns'],np.eye(3),
                            result['previous_from_current_rotation']))
            quality.append({k:v for k,v in result.items() if k!='previous_from_current_rotation'})
    if len(samples)<30:
        raise RuntimeError(f'Visual reference unavailable: {len(samples)} pairs; {dict(reasons)}')
    capacity=max(20000,round((samples[-1][1]-samples[0][0])/1e9*480)+4000)
    buffer=PacketLatticeImuBuffer(sample_rate_hz=float(config.timing.get('imu_rate_hz',480)),
        camera_to_imu_drift_ppm=float(config.timing.get('camera_to_imu_drift_ppm',0)),
        phase_ns=0,capacity=capacity)
    for packet in iter_imu_packets(a.recording):
        # Keep a bounded diagnostic window while decoding preceding packets.
        # add() still checks packet/header continuity; no samples are invented.
        while buffer.rows and buffer.rows[0][0]<samples[0][0]-500_000_000:
            buffer.rows.popleft()
        buffer.add(packet.payload,packet.device_ts)
        if packet.device_ts*1000>samples[-1][1]+250_000_000:break
    times=np.array([r[0] for r in buffer.rows],dtype=np.int64)
    gyro=np.array([r[4:7] for r in buffer.rows])
    samples=[s for s in samples if s[0]>=times[0]+200_000_000 and s[1]<=times[-1]-150_000_000]
    camera_from_body=geometry.T_imu_from_orb_left[:3,:3].T
    coarse=[]
    for phase in range(-150,201,5):
        stats,_=evaluate(samples,times+phase*1_000_000,gyro,camera_from_body)
        coarse.append({'phase_ms':phase,'residual':stats})
    best=min(coarse,key=lambda x:x['residual']['rms_deg'])['phase_ms']
    fine=[]
    for phase in range(best-5,best+6):
        stats,_=evaluate(samples,times+phase*1_000_000,gyro,camera_from_body)
        fine.append({'phase_ms':phase,'residual':stats})
    selected=min(fine,key=lambda x:x['residual']['rms_deg'])
    verification=verify_session_phase(samples,times,gyro,camera_from_body)
    report={'recording':str(a.recording.resolve()),'config':config.describe(),
            'frame_window':{'start':a.start_frame,'maximum_count':a.max_frames,'stride':a.stride},
            'method':'independent short-baseline stereo LK/PnP; no SLAM map or IMU pose predictions',
            'pairs':len(samples),'visual_reasons':dict(reasons),'selected':selected,
            'verification':verification,
            'coarse':coarse,'fine':fine,'quality':quality,
            'limitations':['Diagnostic candidate, not physical exposure truth or a runtime confidence certificate.']}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x',encoding='utf8') as output:
        output.write(json.dumps(report,indent=2))
    print(json.dumps({'recording':str(a.recording),'pairs':len(samples),'selected':selected,'reasons':dict(reasons)}))


if __name__=='__main__':main()
