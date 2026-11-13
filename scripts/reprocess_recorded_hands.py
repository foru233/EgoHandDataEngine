"""Recompute real hands against immutable saved poses with the shared engine.

This is a hands-only offline stage, not a fresh VIO run. Every SDK identity is
checked against the source results and pose stream before exact-frame fusion.
"""
import argparse
import copy
from dataclasses import replace
import json
from pathlib import Path
import shutil
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import numpy as np
import psutil
import yaml
from process_recording import sha,save,summarize,readback
from egohand.core.pipeline import EgoHandEngine
from egohand.core.types import serializable,detection_from_dict
from egohand.core.status import Status
from egohand.gui.workers import PipelineWorker
from egohand.storage.replay import load_recorded_results,SavedResultReplay
from egohand.storage.exporter import export_all
from egohand.calibration.loader import save_calibration
from egohand.vio.config_snapshot import load_config_snapshot
from egohand.vio.hand_world import RecordedLocalizationHandJoin
from egohand.vio.local_segments import segment_hands_for_result


def read(path):return json.loads(path.read_text(encoding='utf8'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pose_run',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--detector-config',type=Path,default=ROOT/'configs/detector.yaml')
    parser.add_argument('--saved-detections',type=Path,help='Reuse measured 2D observations from an exact-frame completed run')
    parser.add_argument('--stereo-refinement-selection',action='store_true')
    args=parser.parse_args()
    parent=args.pose_run.resolve();out=args.output.resolve()
    if out.exists():raise FileExistsError(out)
    validation=read(parent/'validation.json');inputs=read(parent/'processing_inputs.json')
    assert validation['status']=='completed' and validation['input_integrity_passed'] and validation['source_integrity_passed']
    saved,truncated=load_recorded_results(parent/'replay')
    assert saved and not truncated
    with (parent/'replay/poses.jsonl').open(encoding='utf8') as stream:
        poses=[json.loads(line) for line in stream]
    assert len(saved)==len(poses)
    for result,pose in zip(saved,poses):
        assert (result.frame_id,result.timestamp_ns)==(pose['replay']['frame_index'],pose['timestamp_ns'])
    dependencies=dict(inputs['inputs'])
    candidate_results=None
    if args.saved_detections:
        candidate_run=args.saved_detections.resolve()
        candidate_validation=read(candidate_run/'validation.json')
        assert candidate_validation['status']=='completed' and candidate_validation['input_integrity_passed']
        candidate_results,candidate_truncated=load_recorded_results(candidate_run/'replay')
        assert not candidate_truncated and len(candidate_results)==len(saved)
        for original,candidate in zip(saved,candidate_results):
            assert (original.frame_id,original.timestamp_ns,original.calibration_id)==(candidate.frame_id,candidate.timestamp_ns,candidate.calibration_id)
            assert candidate.diagnostics['output_geometry']=='rectified'
        for path in (candidate_run/'replay/results/frames.jsonl',candidate_run/'hand_processing_inputs.json',candidate_run/'validation.json'):
            dependencies[str(path)]=sha(path)
    for path in (parent/'replay/poses.jsonl',parent/'replay/results/frames.jsonl',
                 parent/'hand_processing_inputs.json',parent/'validation.json',args.detector_config,
                 Path(__file__),ROOT/'scripts/process_recording.py',ROOT/'configs/app.yaml',ROOT/'configs/stereo.yaml'):
        dependencies[str(path.resolve())]=sha(path)
    assert all(sha(Path(p))==h for p,h in dependencies.items())
    sources={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/egohand').rglob('*.py')}
    hand_inputs=read(parent/'hand_processing_inputs.json')
    detector=yaml.safe_load(args.detector_config.read_text(encoding='utf8'))
    detector['model_path']=str((ROOT/detector['model_path']).resolve())
    assert sha(Path(detector['model_path']))==hand_inputs['model_sha256']
    hand_config={**hand_inputs['hand_overrides'],'parallel_stereo_detection':True}
    if args.stereo_refinement_selection:hand_config['stereo_refinement_selection']=True
    cfg=parent/'input_config'
    config=load_config_snapshot(calibration=cfg/'kalibr_camchain.yaml',settings=cfg/'vio_settings.yaml',
        timing=cfg/'vio_timing.yaml',quality=cfg/'vio_quality.yaml',source='saved_pose_real_hand_reprocess')
    replay_source=SavedResultReplay(parent/'replay',saved)
    replay_source.open();replay_source.start()
    detectors=None;engine=None
    results=[];measurements=[];owner=psutil.Process();start=time.perf_counter()
    try:
        # Detector construction is shared with both GUIs, including the explicit
        # recorded-only override; no duplicate model/ROI configuration path.
        factory=PipelineWorker(replay_source.source,config=hand_config,detector_config=detector,offline=True)
        if candidate_results is None:
            detectors=factory._build_detectors(detector)
        else:
            recorded_detector=read(candidate_run/'hand_processing_inputs.json')['detector_config']
            for key in ('num_hands','min_hand_detection_confidence','min_hand_presence_confidence','min_tracking_confidence'):
                assert detector[key]==recorded_detector[key], 'Saved 2D detector settings differ'
            class SavedDetections:
                def __init__(self,side):
                    self.side=side;self.by_stamp={r.timestamp_ns:r for r in candidate_results}
                    self.last_detection_diagnostics={}
                def detect(self,image,timestamp_ns):
                    result=self.by_stamp[timestamp_ns]
                    self.last_detection_diagnostics=copy.deepcopy(result.diagnostics[self.side+'_detection_details'])
                    originals=self.last_detection_diagnostics.get('full_frame_observations')
                    if originals is not None:
                        self.last_detection_diagnostics['full_frame_observations']=[detection_from_dict(d) for d in originals]
                    return copy.deepcopy(getattr(result,self.side+'_detections'))
                def close(self):pass
            detectors=[SavedDetections(side) for side in ('left','right')]
        engine=EgoHandEngine(config=hand_config,calibration=replay_source.calibration,detectors=detectors)
        join=RecordedLocalizationHandJoin.from_recording(inputs['recording'],engine.calibration,
            result_session_id=engine.session_id,config_snapshot=config)
        out.mkdir(parents=True)
        replay=out/'replay';(replay/'results').mkdir(parents=True)
        config.write(out/'input_config')
        save_calibration(engine.calibration,replay/'processing_calibration.yaml')
        shutil.copyfile(parent/'replay/poses.jsonl',replay/'poses.jsonl')
        save(out/'processing_inputs.json',{**inputs,'inputs':dependencies,'source_hashes':sources,
            'poses_only':False,'pose_source':str(parent),'estimator_reused':True,
            'operation':('saved measured 2D + new triangulation + immutable saved poses + exact SDK join'
                if candidate_results is not None else 'real hand reinference + immutable saved poses + exact SDK join'),
            'pose_estimator_executed':False,'hand_inference_reused':False,
            'two_d_inference_reused':candidate_results is not None,
            'two_d_candidate_source':str(candidate_run) if candidate_results is not None else None})
        save(out/'hand_processing_inputs.json',{'detector_config':detector,
            'detector_config_path':str(args.detector_config.resolve()),'detector_config_sha256':sha(args.detector_config),
            'model_sha256':sha(Path(detector['model_path'])),'hand_overrides':hand_config,
            'override_sha256':hand_inputs.get('override_sha256'),'default_stereo_sha256':sha(ROOT/'configs/stereo.yaml')})
        with (replay/'results/frames.jsonl').open('x',encoding='utf8') as stream:
            for index,(original,pose) in enumerate(zip(saved,poses)):
                tick=time.perf_counter();replay_source.seek(index);frame=replay_source.read()
                decode_ms=(time.perf_counter()-tick)*1000
                frame=replace(frame,left_detections=None,right_detections=None,
                    metadata={**frame.metadata,'processing_mode':'offline'})
                result=engine.process(frame)
                if result.reconstruction_status==Status.ERROR:raise RuntimeError(result.diagnostics)
                tick=time.perf_counter()
                result.diagnostics['vio_pose']=copy.deepcopy(pose)
                result.diagnostics['saved_pose_dependency']={'pose_run':str(parent),
                    'pose_file_sha256':dependencies[str((parent/'replay/poses.jsonl').resolve())],
                    'estimator_executed_in_this_run':False}
                result.world_hands=join.apply(result,frame,pose,calibration=engine.calibration)
                result.local_motion=copy.deepcopy(pose.get('local_motion'))
                result.segment_hands=segment_hands_for_result(result,pose,result.world_hands)
                join_ms=(time.perf_counter()-tick)*1000
                if result.world_hands.get('reason') not in (None,'localization_not_admitted'):
                    raise RuntimeError(result.world_hands)
                tick=time.perf_counter()
                stream.write(json.dumps(serializable(result),ensure_ascii=False,allow_nan=False)+'\n')
                write_ms=(time.perf_counter()-tick)*1000
                measurements.append({'frame':index,'decode_ms':decode_ms,'join_ms':join_ms,'jsonl_write_ms':write_ms,
                    'rss_bytes':owner.memory_info().rss if index%30==0 else None})
                results.append(result)
                if (index+1)%100==0:
                    stream.flush();print(json.dumps({'real_hand_frames_processed':index+1}),flush=True)
    finally:
        if engine is not None:engine.close()
        elif detectors:
            for detector_instance in detectors:detector_instance.close()
        replay_source.close()
    export_start=time.perf_counter()
    export_all(results,out/'export');save(out/'readback.json',readback(results,out/'export'))
    export_s=time.perf_counter()-export_start
    elapsed=time.perf_counter()-start
    save(replay/'reprocessing.json',{'schema_version':1,'original_recording':inputs['recording'],
        'mode':inputs['mode'],'status':'completed','result_count':len(results),'result_index':'results/frames.jsonl',
        'raw_images':'original_recording (read-only)','pose_estimation_reused':True,'pose_run':str(parent)})
    replay_report=read(parent/'replay/replay.json')
    replay_report['real_hand_reprocessing']={'pose_run':str(parent),'estimator_executed':False}
    save(replay/'replay.json',replay_report)
    assert all(sha(Path(p))==h for p,h in dependencies.items())
    assert all(sha(ROOT/p)==h for p,h in sources.items())
    assert sha(replay/'poses.jsonl')==sha(parent/'replay/poses.jsonl')
    report=summarize(results,poses)
    memory=[m['rss_bytes'] for m in measurements if m['rss_bytes'] is not None]
    stage_times={key:{'total_s':sum(m[key] for m in measurements)/1000,
        'p95_ms':float(np.percentile([m[key] for m in measurements],95))}
        for key in ('decode_ms','join_ms','jsonl_write_ms')}
    report.update(status='completed',error=None,messages=[],input_integrity_passed=True,source_integrity_passed=True,
        elapsed_s=elapsed,offline_processing_fps=len(results)/elapsed,
        performance_scope=('saved real 2D + new triangulation + SDK decode + saved-pose fusion + export/readback; excludes prior inference/VIO'
            if candidate_results is not None else 'fresh hand inference + SDK decode + saved-pose fusion + export/readback; excludes prior VIO'),
        two_d_inference_reused=candidate_results is not None,
        two_d_candidate_source_elapsed_s=candidate_validation['elapsed_s'] if candidate_results is not None else None,
        prior_vio_elapsed_s=validation['elapsed_s'],stage_times=stage_times,export_and_readback_s=export_s,
        runtime_metrics={'rss_first_bytes':memory[0],'rss_last_bytes':memory[-1],
            'rss_peak_sampled_bytes':max(memory),'rss_growth_bytes':memory[-1]-memory[0],
            'scope':'hands-only process RSS sampled every 30 frames; includes saved inputs/results; no native VIO child'},
        raw_recording_sink_counts=validation['raw_recording_sink_counts'],raw_pairs_minus_processed=validation['raw_pairs_minus_processed'],
        replay_frame_accounting=validation['replay_frame_accounting'],pose_file_byte_identical=True,
        real_time_30fps_acceptance=False,device_validation='BLOCKED_offline_only',first_stage_complete=False,
        usable_fused_data=bool(report['hand_counts'].get('segment_hand_frames')),
        real_two_hand_fusion_observed=bool(report['hand_counts'].get('both_hands_segment_frames')),
        pipeline=('shared EgoHandEngine + saved measured 2D + new triangulation + immutable saved poses + existing exact SDK join'
            if candidate_results is not None else 'shared EgoHandEngine + fresh real MediaPipe + immutable saved poses + existing exact SDK join'))
    save(out/'runtime_samples.json',{'hand_stage_frames':measurements})
    save(out/'validation.json',report)
    print(json.dumps(report,ensure_ascii=True,indent=2),flush=True)


if __name__=='__main__':main()
