"""Exercise actual asynchronous startup collector using recordings, without a camera."""
import argparse
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from egohand.storage.sdk_recording import iter_paired_frames,iter_imu_packets
from egohand.vio.config_snapshot import resolve_recording_config
from egohand.vio.imu_lattice import PacketLatticeImuBuffer
from egohand.vio.session_phase import SessionPhasePreparation


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',type=Path,required=True)
    p.add_argument('--config-snapshot',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--offline-working',action='store_true')
    args=p.parse_args()
    config=resolve_recording_config(args.config_snapshot or args.recording)
    if args.offline_working:
        from dataclasses import replace
        from egohand.vio.session_phase import prepare_recording_phase
        before=config.describe()['hashes']
        workspace=args.output.parent/(args.output.stem+'_working')
        prepared=prepare_recording_phase(args.recording,
            replace(config,timing={**config.timing,'lattice_phase_mode':'session_auto'}),workspace)
        assert config.describe()['hashes']==before
        assert prepared.timing['session_phase_verified']
        report={'test_kind':'actual offline phase preparation of raw recording; source snapshots unchanged',
                'phase_ms':prepared.timing['lattice_phase_ms'],'source_hashes':before,
                'working_config':prepared.describe(),'workspace':str(workspace)}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps({'phase_ms':report['phase_ms'],'workspace':str(workspace)}))
        return
    geometry=config.validate_geometry();scale=config.image_scale
    k=geometry.rectifier.P1[:3,:3].copy();k[:2]*=scale
    collector=SessionPhasePreparation(k,-geometry.rectifier.P2[0,3]/geometry.rectifier.P2[0,0],
                                     geometry.T_imu_from_orb_left[:3,:3].T)
    buffer=PacketLatticeImuBuffer(sample_rate_hz=480,camera_to_imu_drift_ppm=0,phase_ns=97_000_000)
    packets=iter(iter_imu_packets(args.recording));packet=next(packets,None)
    model=None;started=time.perf_counter();first_stamp=None
    try:
        for index,frame in enumerate(iter_paired_frames(args.recording)):
            if index>=450:break
            stamp=frame.device_ts*1000
            if first_stamp is None:first_stamp=stamp
            while packet is not None and buffer.covered_until_ns<stamp:
                buffer.add(packet.payload,packet.device_ts);packet=next(packets,None)
            if not buffer.rows or buffer.rows[0][0]>stamp:continue
            left,right=geometry.rectifier.remap_pair(frame.right_bgr,frame.left_bgr)
            left,right=[cv2.resize(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),None,fx=scale,fy=scale,
                                  interpolation=cv2.INTER_AREA) for im in (left,right)]
            model=collector.process(stamp,left,right,buffer)
            assert buffer.consumed==0
            if model is not None:
                buffer.rephase_before_tracking(round(model['phase_ms']*1e6))
                break
        report={'recording':str(args.recording.resolve()),'config':config.describe(),
                'model':model,'frames':index+1,'input_span_s':(stamp-first_stamp)/1e9,
                'elapsed_s':time.perf_counter()-started,'imu_consumed_before_verification':buffer.consumed,
                'locked_phase_ms':buffer.phase_ns/1e6,
                'test_kind':'recorded input / real asynchronous startup collector; no camera or native SLAM'}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('config','model')}))
        if not model or not model['verified']:raise RuntimeError('Startup phase not verified')
    finally:collector.close()


if __name__=='__main__':main()
