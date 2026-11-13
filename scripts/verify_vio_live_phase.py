"""Recorded input through real LiveVio/native worker; no VS030 device access."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import queue
import sys
import time
from unittest.mock import patch
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from egohand.storage.sdk_recording import iter_paired_frames,iter_imu_packets
from egohand.vio.config_snapshot import resolve_recording_config
from egohand.vio import live_runtime


class RecordingInput:
    imu_vio_error=None
    def __init__(self):self.imu_vio_queue=queue.Queue()
    def enable_imu_capture(self,**kwargs):pass


def run(recording, config, worker, rounds):
    reports=[]
    for round_id in range(rounds):
        source=RecordingInput();models=[];poses=[];errors=[];sent=[]
        def status(message):
            kind=message.get('type')
            if kind=='session_phase':models.append(message)
            elif kind=='pose':poses.append(message)
            elif kind in ('error','vio_reset'):errors.append(message)
        original_send=live_runtime._send_tracking_frame
        def send(sock,stamp,left,right,measurements,**tracking_context):
            if len(models)!=1:raise AssertionError('Native frame submitted before session phase verification')
            if not sent and measurements:raise AssertionError('First native frame must have no IMU')
            sent.append(stamp)
            return original_send(sock,stamp,left,right,measurements,**tracking_context)
        vio=live_runtime.LiveVio(source,status,worker=worker)
        try:
            with patch('egohand.vio.config_snapshot.resolve_live_config',return_value=config), \
                 patch('egohand.vio.live_runtime._send_tracking_frame',side_effect=send):
                vio.start()
                if not vio.wait_ready():raise RuntimeError(vio.startup_error)
                packets=iter(iter_imu_packets(recording));packet=next(packets,None)
                for index,frame in enumerate(iter_paired_frames(recording)):
                    if index>=900 or errors:break
                    while packet is not None and packet.device_ts<=frame.device_ts+100_000:
                        source.imu_vio_queue.put((packet.device_ts,packet.payload))
                        packet=next(packets,None)
                    vio.submit(SimpleNamespace(left_image=frame.left_bgr,right_image=frame.right_bgr,
                                               metadata={'sdk_device_timestamp_raw':frame.device_ts}))
                    time.sleep(.04)
                    if len(poses)>=80 and sum(bool(r.get('pose_valid')) for r in poses)>=15:break
                # Allow the final submitted frame to finish before collecting the report.
                time.sleep(.2)
        finally:vio.close()
        report={'round':round_id,'worker_pid':worker.proc.pid if worker.proc else None,
                'phase_models':models,'native_frames':len(sent),'pose_results':len(poses),
                'valid_poses':sum(bool(r.get('pose_valid')) for r in poses),
                'imu_initialized':sum(bool(r.get('imu_initialized')) for r in poses),
                'single_owner_committed_frames':sum(
                    r.get('recovery_preaccept',{}).get('stage')=='committed'
                    and r.get('recovery_ownership',{}).get('owner')=='RecoveryCoordinator'
                    and r.get('recovery_ownership',{}).get('monitor_active') is False for r in poses),
                'frame_queue':poses[-1].get('runtime_performance',{}).get('frame_queue') if poses else None,
                'runtime_timings_ms':[r.get('runtime_performance',{}).get('timing_ms') for r in poses],
                'errors':errors,'worker_log':str(worker.log_path)}
        reports.append(report)
        if errors or not models or len(sent)<30:break
    return reports


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rounds',type=int,default=2)
    a=p.parse_args()
    config=resolve_recording_config(a.recording)
    config=replace(config,timing={**config.timing,'lattice_phase_mode':'session_auto'})
    worker=live_runtime.LiveVioWorker()
    try:reports=run(a.recording,config,worker,a.rounds)
    finally:worker.close()
    success=(len(reports)==a.rounds and all(not r['errors'] and r['native_frames']>=30
                                         and r['imu_initialized']>0 for r in reports)
             and len({r['worker_pid'] for r in reports})==1)
    result={'success':success,'test_kind':'recorded input paced into actual live runtime/native worker; no physical device',
            'dictionary_reused_same_pid':len(reports)>1 and len({r['worker_pid'] for r in reports})==1,
            'reports':reports}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k!='reports'}))
    if not success:raise RuntimeError('Live phase/native acceptance failed; inspect report')


if __name__=='__main__':main()
