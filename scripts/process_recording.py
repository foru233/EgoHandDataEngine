"""Process real SDK recordings with the existing GUI worker, export and read back.

This CLI never edits input recordings or changes the production configuration.
Example: python scripts/process_recording.py RECORDING OUTPUT --mode stereo
Use a complete explicit calibration/settings/timing/quality quartet to override
an old recording snapshot. --library selects an isolated DLL for this run only.
"""
import argparse
import csv
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))

import numpy as np
from egohand.core.types import serializable


def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def save(p,v):
    p.write_text(json.dumps(serializable(v),indent=2,ensure_ascii=False,allow_nan=False),encoding='utf8')


class RuntimeMetrics:
    """Measure the existing bridge call; never replace estimation or admission."""
    def __init__(self):
        self.calls=[];self.memory=[]

    def install(self):
        import psutil
        from egohand.vio.replay_runtime import ReplayBridge
        original=ReplayBridge.track
        owner=psutil.Process()
        def measured(bridge,stamp,*args,**kwargs):
            start=time.perf_counter()
            try:return original(bridge,stamp,*args,**kwargs)
            finally:
                self.calls.append({'timestamp_ns':int(stamp),'ipc_track_ms':(time.perf_counter()-start)*1000})
                if len(self.calls)==1 or len(self.calls)%30==0:
                    processes=[owner]+owner.children(recursive=True)
                    rss={}
                    for process in processes:
                        try:rss[str(process.pid)]=process.memory_info().rss
                        except (psutil.NoSuchProcess,psutil.AccessDenied):pass
                    self.memory.append({'frame':len(self.calls),'rss_bytes_by_pid':rss,'total_rss_bytes':sum(rss.values())})
        ReplayBridge.track=measured

    def report(self):
        values=[x['ipc_track_ms'] for x in self.calls]
        totals=[x['total_rss_bytes'] for x in self.memory]
        return {'bridge_call_count':len(values),
            'ipc_track_ms':{'p50':float(np.median(values)),'p95':float(np.percentile(values,95)),
                'max':float(max(values))} if values else None,
            'rss_first_bytes':totals[0] if totals else None,'rss_last_bytes':totals[-1] if totals else None,
            'rss_peak_sampled_bytes':max(totals) if totals else None,
            'rss_growth_bytes':totals[-1]-totals[0] if totals else None,
            'scope':'CLI + owned VIO children; RSS sampled every 30 bridge calls; IPC includes serialization/native wait, excludes decode/hand inference; no online latency or leak certification'}


def summarize(results, poses):
    hands=Counter(); reasons=Counter(); detections=Counter(); segments={}; depths=[];repro=[]
    for r in results:
        for side,ds in [('left_view',r.left_detections),('right_view',r.right_detections)]:
            for name in ('Left','Right'):
                detections[f'{side}_{name}_frames']+=any(d.handedness==name for d in ds)
        for name in ('Left','Right'):
            hs=[h for h in r.hands if h.handedness==name and any(j.valid for j in h.joints)]
            hands[f'{name}_camera_3d_frames']+=bool(hs)
            hands[f'{name}_21_valid_frames']+=any(all(j.valid for j in h.joints) for h in hs)
        hands['camera_3d_frames']+=any(j.valid for h in r.hands for j in h.joints)
        hands['both_hands_camera_3d_frames']+=all(any(h.handedness==name and any(j.valid for j in h.joints) for h in r.hands) for name in ('Left','Right'))
        hands['stereo_matched_hands']+=sum(h.valid_joint_count>0 for h in r.hands)
        hands['left_detected_hands']+=len(r.left_detections)
        hands['right_detected_hands']+=len(r.right_detections)
        hands['valid_camera_joints']+=sum(j.valid for h in r.hands for j in h.joints)
        hands['output_joint_slots']+=sum(len(h.joints) for h in r.hands)
        world=r.world_hands or {}; segment=r.segment_hands or {}
        hands['map_hand_frames']+=world.get('status')=='available'
        hands['segment_hand_frames']+=segment.get('status')=='available'
        hands['both_hands_segment_frames']+=all(any(h.get('handedness')==name and any(h.get('valid',[])) for h in segment.get('hands',[])) for name in ('Left','Right'))
        if world.get('reason'):reasons['world:'+world['reason']]+=1
        if segment.get('reason'):reasons['segment:'+segment['reason']]+=1
        if r.diagnostics.get('reason'):reasons['camera:'+r.diagnostics['reason']]+=1
        for h in r.hands:
            for j in h.joints:
                if j.valid:
                    depths.append(float(j.xyz_m[2]))
                    if j.reprojection_error_px is not None:repro.append(j.reprojection_error_px)
                else:reasons['joint:'+str(j.status)]+=1
    state=Counter();source=Counter();losses=0;previous=None
    for p in poses:
        tracking=p.get('tracking_state');state[tracking]+=1
        if tracking in ('LOST','RECENTLY_LOST') and previous not in ('LOST','RECENTLY_LOST'):
            losses+=1
        previous=tracking
        motion=p.get('local_motion') or {}
        if motion.get('pose_valid') is True:
            source[motion['pose_source']]+=1
            key=motion['segment_id'];part=segments.setdefault(key,{'frames':0,'first_ns':p['timestamp_ns'],
                'last_ns':p['timestamp_ns'],'start_reason':motion.get('segment_start_reason')})
            part['frames']+=1;part['last_ns']=p['timestamp_ns']
    for part in segments.values():part['span_s']=(part['last_ns']-part['first_ns'])/1e9
    def stats(a):
        return {'count':len(a),'min':float(min(a)),'median':float(np.median(a)),
                'p95':float(np.percentile(a,95)),'max':float(max(a))} if a else None
    return {'result_frames':len(results),'pose_frames':len(poses),'tracking':dict(state),
        'pose_admitted':sum(p.get('pose_valid') is True for p in poses),
        'local_pose_frames':sum(source.values()),'pose_sources':dict(source),
        'recording_processed_span_s':(poses[-1]['timestamp_ns']-poses[0]['timestamp_ns'])/1e9 if poses else 0,
        'loss_episodes':losses,'segments':segments,'segment_count':len(segments),
        'longest_segment_frames':max((s['frames'] for s in segments.values()),default=0),
        'longest_segment_span_s':max((s['span_s'] for s in segments.values()),default=0),
        'jump_detected_frames':sum(p.get('jump_detected') is True for p in poses),
        'quarantined_frames':sum((p.get('recovery_preaccept') or {}).get('await_visual_requalification') is True for p in poses),
        'hand_counts':dict(hands),'detected_label_counts':dict(detections),
        'valid_joint_depth_m':stats(depths),'valid_joint_reprojection_px':stats(repro),
        'hand_failure_reasons':dict(reasons),
        'pose_available_ratio':sum(p.get('pose_valid') is True for p in poses)/len(poses) if poses else 0,
        'handedness':'detector labels; no independent anatomical identity ground truth',
        'vs030_absolute_accuracy_verified':False,'cross_segment_merge_permitted':False}


def readback(results, directory):
    from egohand.storage.exporter import load_json
    loaded=load_json(directory/'results.json')
    if serializable(loaded)!=serializable(results):
        raise RuntimeError('JSON roundtrip mismatch')
    report={'json_exact_roundtrip':True,'frames':len(loaded)}
    with np.load(directory/'joints.npz',allow_pickle=False) as a:
        for i,r in enumerate(loaded):
            for k,h in enumerate(r.hands):
                for j in h.joints:
                    if bool(a['valid'][i,k,j.joint_id])!=j.valid:
                        raise RuntimeError('Camera valid mask mismatch')
                    if j.valid:np.testing.assert_allclose(a['xyz_m'][i,k,j.joint_id],j.xyz_m,rtol=0,atol=0)
    report['camera_npz_numeric_roundtrip']=True
    with (directory/'joints.csv').open(encoding='utf-8-sig',newline='') as f:
        camera_rows=list(csv.DictReader(f))
    expected=[(r,h,j) for r in loaded for h in r.hands for j in h.joints]
    if len(camera_rows)!=len(expected):raise RuntimeError('Camera CSV row count mismatch')
    for row,(r,h,j) in zip(camera_rows,expected):
        if (int(row['frame_id']),int(row['timestamp_ns']),int(row['track_id']),int(row['joint_id']))!=(r.frame_id,r.timestamp_ns,h.track_id,j.joint_id):
            raise RuntimeError('Camera CSV identity mismatch')
        if j.valid:np.testing.assert_array_equal([float(row[k]) for k in ('x_m','y_m','z_m')],j.xyz_m)
        elif any(row[k] for k in ('x_m','y_m','z_m')):raise RuntimeError('Invalid camera CSV coordinates')
    report['camera_csv_identity_numeric_roundtrip']=True
    if (directory/'camera_trajectory.npz').is_file():
        with np.load(directory/'camera_trajectory.npz',allow_pickle=False) as a:
            for i,r in enumerate(loaded):
                motion=r.local_motion or {}
                if motion.get('pose_valid') is True:
                    np.testing.assert_allclose(a['T_segment_from_camera'][i],motion['T_segment_from_camera'],rtol=0,atol=0)
                    if str(a['segment_id'][i])!=motion['segment_id']:raise RuntimeError('Segment ID mismatch')
                elif not np.isnan(a['T_segment_from_camera'][i]).all():raise RuntimeError('Invalid pose serialized as coordinates')
        with np.load(directory/'segment_hands.npz',allow_pickle=False) as a:
            for i,r in enumerate(loaded):
                for k,h in enumerate((r.segment_hands or {}).get('hands',[])):
                    mask=np.asarray(h['valid'],dtype=bool)
                    np.testing.assert_array_equal(a['valid'][i,k],mask)
                    np.testing.assert_allclose(a['xyz_segment_m'][i,k,mask],np.asarray(h['xyz_segment_m'],dtype=float)[mask],rtol=0,atol=0)
        report['local_npz_numeric_roundtrip']=True
        trajectory=json.loads((directory/'camera_trajectory.json').read_text(encoding='utf8'))
        for r,motion in zip(loaded,trajectory):
            if motion.get('pose_valid') is not True:continue
            T=np.asarray(motion['T_segment_from_camera'],dtype=float)
            for h,local in zip(r.hands,(r.segment_hands or {}).get('hands',[])):
                points=np.asarray([j.xyz_m for j in h.joints],dtype=float)
                mask=np.asarray(local['valid'],dtype=bool)
                np.testing.assert_allclose(np.asarray(local['xyz_segment_m'],dtype=float)[mask],
                    points[mask]@T[:3,:3].T+T[:3,3],rtol=0,atol=1e-12)
        with (directory/'segment_hands.csv').open(encoding='utf-8-sig',newline='') as f:
            segment_rows=list(csv.DictReader(f))
        expected=[(r,h,j) for r in loaded for h in (r.segment_hands or {}).get('hands',[]) for j in range(21)]
        if len(segment_rows)!=len(expected):raise RuntimeError('Segment CSV row count mismatch')
        for row,(r,h,j) in zip(segment_rows,expected):
            if (int(row['frame_id']),int(row['timestamp_ns']),int(row['joint_id']),row['segment_id'])!=(r.frame_id,r.timestamp_ns,j,r.segment_hands['segment_id']):
                raise RuntimeError('Segment CSV identity mismatch')
            if h['valid'][j]:np.testing.assert_array_equal([float(row[k]) for k in ('x_m','y_m','z_m')],h['xyz_segment_m'][j])
        report['local_csv_identity_numeric_roundtrip']=True
        report['independent_saved_camera_to_segment_recompute']=True
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--mode',choices=('stereo','stereo_inertial','hybrid'),default='stereo_inertial')
    for name in ('calibration','settings','timing','quality','library'):
        parser.add_argument('--'+name,type=Path)
    parser.add_argument('--image-scale',type=float)
    parser.add_argument('--session-auto-phase',action='store_true')
    parser.add_argument('--poses-only',action='store_true',help='Isolated localization comparison; no hand inference')
    parser.add_argument('--hand-config',type=Path,help='Explicit stereo hand quality overrides; does not change GUI defaults')
    parser.add_argument('--detector-config',type=Path,help='Explicit offline detector profile; does not change live defaults')
    parser.add_argument('--coordinated-stereo',action='store_true',help='Opt into existing single-owner fail-closed admission for isolated Stereo replay')
    args=parser.parse_args()
    from egohand.vio.config_snapshot import load_config_snapshot,resolve_recording_config
    quartet=[args.calibration,args.settings,args.timing,args.quality]
    if any(quartet) and not all(quartet):parser.error('显式覆盖须同时给出 calibration/settings/timing/quality')
    timing={}
    if args.image_scale is not None:timing['image_scale']=args.image_scale
    if args.session_auto_phase:timing.update(lattice_phase_mode='session_auto',session_phase_verified=False)
    if all(quartet):
        config=load_config_snapshot(calibration=args.calibration,settings=args.settings,timing=args.timing,
            quality=args.quality,timing_override=timing,source='offline_product_explicit')
    else:config=resolve_recording_config(args.recording,timing_override=timing)
    config.validate_geometry()
    args.output.mkdir(parents=True,exist_ok=False)
    root=args.recording.resolve()
    m=json.loads((root/'manifest.json').read_text(encoding='utf-8-sig'))
    paths=[root/'manifest.json']
    for key in ('sdk_records','sdk_index','imu_records','calibration'):
        if m.get(key):paths.append(root/m[key])
    hashes={str(p):sha(p) for p in paths}
    source_hashes={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/egohand').rglob('*.py')}
    config.write(args.output/'input_config')
    save(args.output/'processing_inputs.json',{'recording':str(root),'mode':args.mode,
        'poses_only':args.poses_only,'inputs':hashes,'source_hashes':source_hashes,
        'config':config.describe(),'library':str(args.library.resolve()) if args.library else None,
        'library_sha256':sha(args.library) if args.library else None,
        'production_defaults_changed':False})
    messages=[];start=time.perf_counter();results=[]
    metrics=RuntimeMetrics();metrics.install()
    def status(message):
        if message.get('type')!='pose':print(json.dumps(message,ensure_ascii=True),flush=True)
    if args.poses_only:
        from egohand.vio.replay_runtime import ReplayVio
        replay=ReplayVio(root,output_dir=args.output/'replay',config_snapshot=config,
            mode=args.mode,library_path=args.library,on_status=status,coordinated_stereo=args.coordinated_stereo)
        report=replay.run();error=report.get('error')
    else:
        import yaml
        from PySide6.QtCore import QCoreApplication
        from egohand.gui.workers import PipelineWorker
        from egohand.sources.sdk_recorded import SdkRecordedStereoSource
        from egohand.calibration.loader import load_calibration
        from egohand.storage.replay import load_recorded_results
        from egohand.storage.exporter import export_all
        app=QCoreApplication.instance() or QCoreApplication([])
        detector_path=args.detector_config or ROOT/'configs/detector.yaml'
        detector=yaml.safe_load(detector_path.read_text(encoding='utf8'))
        detector['model_path']=str(ROOT/detector['model_path'])
        hand_config=yaml.safe_load(args.hand_config.read_text(encoding='utf8')) if args.hand_config else {}
        save(args.output/'hand_processing_inputs.json',{'detector_config':detector,
            'detector_config_path':str(detector_path.resolve()),'detector_config_sha256':sha(detector_path),
            'model_sha256':sha(Path(detector['model_path'])),
            'hand_overrides':hand_config,'override_sha256':sha(args.hand_config) if args.hand_config else None,
            'default_stereo_sha256':sha(ROOT/'configs/stereo.yaml')})
        worker=PipelineWorker(SdkRecordedStereoSource(root),config=hand_config,offline=True,detector_config=detector,
            calibration=load_calibration(config.calibration,kalibr_left_camera='cam0'),
            live_vio=True,vio_mode=args.mode,vio_output_dir=args.output/'replay',
            vio_config_snapshot=config,vio_library_path=args.library,coordinated_stereo=args.coordinated_stereo)
        worker.error.connect(messages.append);worker.vio_status.connect(status)
        worker.run();app.processEvents()
        error=worker.last_error
        report_path=args.output/'replay/replay.json'
        report=json.loads(report_path.read_text(encoding='utf8')) if report_path.is_file() else {'status':'failed'}
        results,truncated=load_recorded_results(args.output/'replay')
        if truncated:raise RuntimeError('Newly processed output has truncated JSONL')
        export_all(results,args.output/'export')
        save(args.output/'readback.json',readback(results,args.output/'export'))
    poses_path=args.output/'replay/poses.jsonl'
    poses=[json.loads(s) for s in poses_path.open(encoding='utf8')] if poses_path.is_file() else []
    summary=summarize(results,poses)
    save(args.output/'runtime_samples.json',{'bridge_calls':metrics.calls,'memory':metrics.memory})
    summary['runtime_metrics']=metrics.report()
    summary['replay_frame_accounting']=report.get('frames')
    summary['raw_recording_sink_counts']=m.get('sdk_count_by_sink')
    summary['raw_pairs_minus_processed']=min(int((m.get('sdk_count_by_sink') or {}).get(str(k),0)) for k in (4,5))-len(poses)
    integrity=all(sha(Path(p))==h for p,h in hashes.items())
    source_integrity=all(sha(ROOT/p)==h for p,h in source_hashes.items())
    summary.update(status=report.get('status'),error=error,messages=messages,
        input_integrity_passed=integrity,source_integrity_passed=source_integrity,
        elapsed_s=time.perf_counter()-start,
        offline_processing_fps=len(poses)/max(time.perf_counter()-start,1e-6),
        real_time_30fps_acceptance=False,device_validation='BLOCKED_offline_only',
        pipeline='existing PipelineWorker + EgoHandEngine + ReplayVio + RecordedLocalizationHandJoin',
        first_stage_complete=False)
    summary['usable_fused_data']=bool(summary['hand_counts'].get('segment_hand_frames',0))
    summary['real_two_hand_fusion_observed']=bool(summary['hand_counts'].get('both_hands_segment_frames',0))
    save(args.output/'validation.json',summary)
    print(json.dumps(serializable(summary),ensure_ascii=True,allow_nan=False,indent=2),flush=True)
    success=(not error and not messages and report.get('status')=='completed' and integrity and source_integrity)
    return 0 if success else 1


if __name__=='__main__':
    sys.exit(main())
