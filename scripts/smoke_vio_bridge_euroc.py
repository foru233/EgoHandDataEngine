"""在独立进程用 EuRoC 原始图像与真实 IMU 验证 native bridge。

本测试使用官方 EuRoC PinHole 配置，ORB-SLAM3 内部执行校正；
VS030 实时接口必须另行使用已校正图及对应标定，不套用本配置。
"""
import csv
import json
import time
from collections import Counter
from pathlib import Path
import numpy as np
from PIL import Image
from egohand.vio.native_bridge import ImuMeasurement,NativeVioBridge
ROOT=Path(__file__).resolve().parents[1]

def main():
    root=ROOT/'samples/date/VIO/converted/EuRoC/MH_01_easy'
    stamps=[int(x) for x in (root/'timestamps_smoke_160.txt').read_text().split()]
    samples=[]
    with (root/'mav0/imu0/data.csv').open() as f:
        for row in csv.reader(line for line in f if not line.startswith('#')):
            if not row:continue
            t=int(row[0]);g=tuple(map(float,row[1:4]));a=tuple(map(float,row[4:7]))
            samples.append(ImuMeasurement(t,a,g))
    vocabulary=ROOT/'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt'
    config=root/'calibration/EuRoC.yaml';results=[];body_states=[];i=0;previous=None
    t0=time.perf_counter()
    with NativeVioBridge(vocabulary,config) as bridge:
        version=bridge.api_version
        for stamp in stamps:
            ims=[np.asarray(Image.open(root/f'mav0/cam{k}/data/{stamp}.png').convert('L')) for k in (0,1)]
            batch=[]
            while i<len(samples) and samples[i].timestamp_ns<=stamp:
                if previous is not None and samples[i].timestamp_ns>previous:batch.append(samples[i])
                i+=1
            out=bridge.track_stereo(ims[0],ims[1],stamp,batch)
            body_states.append(bridge.get_candidate_body_state())
            results.append(out);previous=stamp
    poses=[r.T_world_from_rectified_left_camera for r in results if r.pose_valid]
    if any(not np.isfinite(p).all() or not np.allclose(p[3],[0,0,0,1],atol=1e-5) for p in poses):
        raise RuntimeError('bridge returned invalid pose')
    report={'dataset':'EuRoC MH_01_easy 160 frames','mode':'ORB-SLAM3 Stereo-Inertial',
        'geometry':'official raw EuRoC images; ORB-SLAM3 internally rectifies via official settings',
        'api_version':version,'frames':len(results),'tracking_states':dict(Counter(r.tracking_state_name for r in results)),
        'pose_valid':len(poses),'imu_initialized_frames':sum(r.imu_initialized for r in results),
        'body_state_candidates':sum(x is not None for x in body_states),
        'body_state_ba2_frames':sum(x is not None and x['ba2_complete'] for x in body_states),
        'first_frame_imu_initialized':results[0].imu_initialized,'duration_s':time.perf_counter()-t0,
        'note':'离线接口冒烟测试，不证明 VS030 实时同步、IMU 语义或定位精度。'}
    path=ROOT/'results/vio/native_bridge_euroc_smoke.json';path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    if not poses:raise RuntimeError('no valid tracking pose in smoke test')
    if results[0].imu_initialized:raise RuntimeError('IMU incorrectly reported initialized on first frame')
    if body_states[0] is not None:raise RuntimeError('uninitialized first frame has body state')
    if not any(x is not None for x in body_states):raise RuntimeError('no inertial body state observed')
    for state in (x for x in body_states if x is not None):
        if not all(np.isfinite(state[k]).all() for k in ('T_world_from_body','velocity_world_mps','accel_bias_mps2','gyro_bias_rps')):
            raise RuntimeError('non-finite candidate body state')

if __name__=='__main__':main()
