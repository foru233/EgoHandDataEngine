# 独立评测编排，不启动 GUI；Oracle 与真实模型两通道分开，主参数由 scripts/run_stereo_benchmark.py 提供。

import copy
from dataclasses import asdict
from datetime import datetime,timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import time
import uuid
import cv2
import numpy as np
import yaml
from ..core.pipeline import EgoHandEngine,default_config
from ..core.types import StereoFrame,HandDetection,serializable
from ..detection.mediapipe_detector import MediaPipeHandDetector
from ..detection.landmarks import JOINT_NAMES,BONES
from ..storage.exporter import export_csv,export_npz
from .types import BenchmarkBlocked
from .hot3d import HOT3DBenchmarkAdapter,CANONICAL_MP
from .stb import STBBenchmarkAdapter
from .provenance import calibration_fingerprint,sha256
from .metrics import distribution,write_csv,match_2d,grouped_joint_errors,save_plots


def save_json(path,value):
    Path(path).write_text(json.dumps(serializable(value),indent=2,ensure_ascii=False,allow_nan=False),encoding='utf8')


# 只在评测/Oracle 通道生成投影参考；真实模型推理不得使用这些 GT 像素补检。
def project_truth(sample):
    if sample.dataset_id=='hot3d':
        geometry=sample.native_annotations['geometry'];truth=sample.native_annotations['gt_hands']
        mapped={}
        projections=[{},{}]
        for side,xyz in truth.items():
            points=np.full((21,3),np.nan);points[list(CANONICAL_MP)]=xyz;mapped[side]=points
            for camera in (0,1):
                uv,mask=geometry.project_virtual(xyz,camera)
                full=np.full((21,2),np.nan);full[list(CANONICAL_MP)]=uv
                valid=np.zeros(21,bool);valid[list(CANONICAL_MP)]=mask
                projections[camera][side]=(full,valid)
        return mapped,projections
    points,mask=sample.mapped_ground_truth()
    if sample.calibration is None or not mask.any():return {},[{},{}]
    from ..calibration.rectifier import StereoRectifier
    r=StereoRectifier(sample.calibration);c=sample.calibration
    projections=[]
    for side in (0,1):
        R,t=(np.eye(3),np.zeros(3)) if side==0 else (c.R_right_left,c.t_right_left)
        fn=cv2.fisheye.projectPoints if c.distortion_model=='fisheye' else cv2.projectPoints
        uv=fn(points.reshape(-1,1,3),cv2.Rodrigues(R)[0],t,c.K_left if side==0 else c.K_right,c.D_left if side==0 else c.D_right)[0].reshape(21,2)
        uv=r.points(uv,'left' if side==0 else 'right')
        valid=mask&np.isfinite(uv).all(axis=1)&(uv>=0).all(axis=1)&(uv<[c.image_width,c.image_height]).all(axis=1)
        projections.append({'single_hand':(uv,valid)})
    return {'single_hand':points},projections


# 保存可复核图：绿色为模型 GT 投影，橙色为预测二维点；失败图也保留。
def draw_case(sample,result,truth_uv,path):
    images=[]
    for side,(image,ds) in enumerate(((sample.left_image,result.left_detections),(sample.right_image,result.right_detections))):
        canvas=image.copy()
        for uv,mask in truth_uv[side].values():
            for p in uv[mask]:cv2.circle(canvas,tuple(np.rint(p).astype(int)),3,(0,210,0),1)
        for d in ds:
            for a,b in BONES:
                if d.valid[a] and d.valid[b] and np.isfinite(d.pixel_landmarks[[a,b]]).all():
                    pa,pb=np.rint(d.pixel_landmarks[[a,b]]).astype(int)
                    # Malformed detector output must not overflow drawing primitives.
                    if np.max(np.abs([pa,pb]))<100000:cv2.line(canvas,tuple(pa),tuple(pb),(0,130,255),1)
        cv2.putText(canvas,('1201-1' if side==0 else '1201-2')+' | green=GT orange=detector',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
        images.append(canvas)
    cv2.imwrite(str(path),np.hstack(images))


# 顺序执行样本、保存配置/摘要/失败案例；没有条件的指标用 None 与 N/A 原因表示。
def run(args):
    # 不允许直接对原生鱼眼图运行检测；virtual 表示输出的校正坐标系。
    if getattr(args,'detector_input','virtual') != 'virtual' or getattr(args,'native_detection',False):
        raise ValueError('必须先去畸变并极线校正，再检测；原生鱼眼检测已停用')
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True);(output/'failure_cases').mkdir(exist_ok=True)
    config=default_config()
    if args.config:config.update(yaml.safe_load(Path(args.config).read_text(encoding='utf8')))
    run_id=str(uuid.uuid4());np.random.seed(args.seed)
    save_json(output/'environment.json',{'python':platform.python_version(),'platform':platform.platform(),
        'dependencies':{p:importlib.metadata.version(p) for p in ('numpy','scipy','opencv-contrib-python','mediapipe','PySide6')},
        'run_id':run_id,'utc':datetime.now(timezone.utc).isoformat()})
    (output/'config_snapshot.yaml').write_text(yaml.safe_dump({'engine':config,'benchmark':vars(args)},allow_unicode=True,sort_keys=False),encoding='utf8')
    selection=json.loads(Path(args.manifest).read_text(encoding='utf8')) if args.manifest else None
    ids=[f"{r['sequence_id']}/{r['frame_id']}" for r in selection] if selection else None
    adapter=(HOT3DBenchmarkAdapter(args.dataset_root,max_frames=args.max_frames,toolkit_root=args.toolkit_root,
        clips=args.clips.split(',') if args.clips else None,frame_ids=ids) if args.dataset=='hot3d' else
        STBBenchmarkAdapter(args.dataset_root,max_frames=args.max_frames,frame_ids=ids))
    records=[];rows=[];manifest=[];provenance=[];results=[];failure_candidates=[]
    detectors=None;engine=None;current_calibration=None;current_sequence=None
    counts={'frames':0,'left_any_detection':0,'right_any_detection':0,'both_any_detection':0,'either_any_detection':0,
        'native_left_any_detection':0,'native_right_any_detection':0,'valid_output_joints':0,'predicted_hands':0,
        'accepted_stereo_pairs':0,'stereo_pairs_evaluable':0,'stereo_pairs_correct':0,'unknown_gt_pairs':0,
        'detector_gt_hands_visible':0,'detector_gt_hands_found':0,'gt_hands':0,'id_switches_consecutive_observations':0}
    status='SOFTWARE_VERIFIED';blocked=None;processing_s=0;native_s=0;last_identity={};frame_results=[]
    wall_start=None;wall_elapsed=0
    try:
        if args.mode=='mediapipe-stereo':
            detector_cfg_path=Path(args.detector_config) if getattr(args,'detector_config',None) else Path('configs/detector.yaml')
            if not detector_cfg_path.is_absolute(): detector_cfg_path=Path(__file__).resolve().parents[3]/detector_cfg_path
            if not detector_cfg_path.is_file(): raise FileNotFoundError('Detector config missing: '+str(detector_cfg_path))
            detector_cfg=yaml.safe_load(detector_cfg_path.read_text(encoding='utf8'))
            detector_cfg=detector_cfg or {}
            allowed=('num_hands','min_hand_detection_confidence','min_hand_presence_confidence','min_tracking_confidence')
            detector_params={key:detector_cfg[key] for key in allowed if key in detector_cfg}
            detectors=[MediaPipeHandDetector(args.model,mode='IMAGE',rescue_config=detector_cfg.get('rescue'),**detector_params) for _ in (0,1)]
            (output/'detector_snapshot.json').write_text(json.dumps({
                'model_sha256':[d.sha256 for d in detectors],'mode':'IMAGE',
                'config_source':str(detector_cfg_path.resolve()) if detector_cfg_path.is_file() else 'constructor_defaults',
                'effective_model_parameters':detectors[0]._task_options | {'base_options':'official_model_file'},
                'rescue_config':[dict(d.rescue_config) for d in detectors]},indent=2,default=str),encoding='utf8')
        wall_start=time.perf_counter()
        for sample in adapter.iter_samples():
            if selection:
                expected=next(r for r in selection if r['sequence_id']==sample.sequence_id and r['frame_id']==sample.frame_id)
                for field in ('image_sha256','archive_sha256'):
                    if expected.get(field) and expected[field]!=sample.provenance.get(field):
                        raise BenchmarkBlocked('Fixed sample manifest hash mismatch: '+field)
            tick=time.perf_counter();counts['frames']+=1
            if args.dataset=='stb' and args.mode=='oracle-2d' and sample.calibration is None:
                raise BenchmarkBlocked('CALIBRATION_MAPPING_UNVERIFIED: Oracle-2D disabled')
            fp=calibration_fingerprint(sample.calibration)
            if engine is None or fp!=current_calibration or current_sequence!=sample.sequence_id:
                engine=EgoHandEngine(config=config,calibration=sample.calibration,detectors=detectors)
                current_calibration=fp;current_sequence=sample.sequence_id;last_identity={}
            truth,truth_uv=project_truth(sample)
            left_ds=right_ds=None
            if args.mode=='oracle-2d':
                if not truth:raise BenchmarkBlocked('No verified official FK GT for Oracle-2D')
                ds=[]
                for camera in (0,1):
                    ds.append([HandDetection(side.title(),1.,uv/[sample.left_image.shape[1],sample.left_image.shape[0]],uv,
                        valid=mask,origin='oracle_2d_projected_gt') for side,(uv,mask) in truth_uv[camera].items()])
                left_ds,right_ds=ds
            metadata={'real_stereo':True,'timestamp_kind':'dataset_sensor' if sample.left_timestamp_ns is not None else 'benchmark_order_only_not_sensor',
                'sync_quality':'DATASET_DOCUMENTED','synchronization_evidence':sample.provenance['source'],
                'rectification_id':sample.calibration.calibration_id if sample.calibration else None,
                'benchmark_dataset_id':sample.dataset_id,'benchmark_sequence_id':sample.sequence_id,'benchmark_frame_id':sample.frame_id}
            # STB unknown sensor time: order is for API monotonicity only, no temporal inference.
            ts=sample.left_timestamp_ns if sample.left_timestamp_ns is not None else counts['frames']*1_000_000_000
            if sample.left_timestamp_ns is None:engine.reset_tracking()
            f=StereoFrame(counts['frames']-1,int(ts),sample.left_image,sample.right_image,sample.left_timestamp_ns,
                sample.right_timestamp_ns,sample.calibration.calibration_id if sample.calibration else None,
                'benchmark:'+sample.dataset_id,False,sample.image_geometry,metadata,left_ds,right_ds)
            pairing={'dataset_id':sample.dataset_id,'sync_quality':'DATASET_DOCUMENTED',
                'pair_id':sample.sequence_id+'/'+sample.frame_id,'source':sample.provenance['source']}
            result=engine.process(f,dataset_pairing=pairing)
            if detectors:
                result.diagnostics['detector_rescue']=[copy.deepcopy(getattr(d,'last_detection_diagnostics',{})) for d in detectors]
                result.diagnostics['detector_input']='stereo_rectified_image'
            # Preserve engine output in virtual frame for runtime/export compatibility.
            # Benchmark result additionally exposes original left coordinates by explicit rotation.
            if args.dataset=='hot3d':result.coordinate_frame='virtual_left_optical'
            result.diagnostics.update(metadata,processing_run_id=run_id,calibration_fingerprint=fp,
                joint_mapping_version=sample.provenance.get('joint_mapping_version'),
                metric_valid=bool(truth),evaluation_mode=args.mode)
            processing_s+=time.perf_counter()-tick
            ds=[result.left_detections,result.right_detections]
            for key,yes in [('left_any_detection',bool(ds[0])),('right_any_detection',bool(ds[1])),('both_any_detection',bool(ds[0]) and bool(ds[1])),('either_any_detection',bool(ds[0]) or bool(ds[1]))]:counts[key]+=yes
            counts['valid_output_joints']+=sum(h.valid_joint_count for h in result.hands)
            counts['predicted_hands']+=len(result.hands)
            # 左右各自通过二维投影建立评测身份，仅用于打分，不修改系统的实际左右配对。
            eval_matches=[match_2d(ds[i],truth_uv[i]) for i in (0,1)]
            counts['detector_gt_hands_visible']+=sum(mask.sum()>=4 for view in truth_uv for _,mask in view.values())
            counts['detector_gt_hands_found']+=sum(len(x[0]) for x in eval_matches)
            pairs=result.diagnostics.get('association_quality',{}).get('accepted_pairs',[])
            counts['accepted_stereo_pairs']+=len(pairs)
            prediction_by_truth={};matched_sides=set()
            for hi,(li,ri) in enumerate(pairs):
                lside=eval_matches[0][0].get(li);rside=eval_matches[1][0].get(ri)
                if lside is not None and rside is not None:
                    counts['stereo_pairs_evaluable']+=1;counts['stereo_pairs_correct']+=lside==rside
                else:counts['unknown_gt_pairs']+=1
                # Evaluate error against the hand identified independently in LEFT 2D,
                # even if the system paired it to the wrong right hand.
                if lside is not None and hi<len(result.hands):
                    prediction_by_truth[lside]=result.hands[hi];matched_sides.add(lside)
                    h=result.hands[hi]
                    previous=last_identity.get(lside)
                    if h.track_id>0:
                        if previous and previous[0]==counts['frames']-1 and previous[1]!=h.track_id:
                            counts['id_switches_consecutive_observations']+=1
                        last_identity[lside]=(counts['frames'],h.track_id)
            counts['gt_hands']+=len(truth)
            frame_rows=[]
            for side,gt in truth.items():
                hand=prediction_by_truth.get(side)
                for joint_id in np.flatnonzero(np.isfinite(gt).all(axis=1)):
                    joint=hand.joints[joint_id] if hand else None
                    valid=bool(joint and joint.valid)
                    uv_l,ml=truth_uv[0][side];uv_r,mr=truth_uv[1][side]
                    disparity=float(abs(uv_l[joint_id,0]-uv_r[joint_id,0]))
                    native_xyz=sample.native_annotations['geometry'].virtual_to_original(gt[joint_id]) if args.dataset=='hot3d' else gt[joint_id]
                    row={'sequence_id':sample.sequence_id,'frame_id':sample.frame_id,'hand':side,'joint_id':int(joint_id),'joint_name':JOINT_NAMES[joint_id],
                        'depth_m':float(native_xyz[2]),'disparity_px':disparity,'both_views_valid':bool(ml[joint_id] and mr[joint_id]),
                        'valid':valid,'error_mm':float(np.linalg.norm(joint.xyz_m-gt[joint_id])*1000) if valid else None,
                        'position_std_m':joint.position_std_m if joint else None,
                        'status':str(joint.status) if joint else 'NO_EVALUABLE_STEREO_HAND',
                        'depth_bin':str(int(native_xyz[2]/.1)*.1)[:5],
                        'disparity_bin':f'{int(disparity//10)*10:04d}-{int(disparity//10)*10+10:04d}',
                        'occlusion_bin':str(int(10*min(sample.native_annotations.get('hands',{}).get(side,{}).get('visibilities_modeled',{}).get(s,0.) for s in ('1201-1','1201-2')))/10) if args.dataset=='hot3d' else 'N/A'}
                    frame_rows.append(row);rows.append(row)
            frame_error=distribution([r['error_mm'] for r in frame_rows if r['error_mm'] is not None])
            result.diagnostics['metric_valid']=bool(frame_error['count'])
            result.diagnostics['gt_available']=bool(truth)
            rejected=sum(not r['valid'] for r in frame_rows)
            frame_record={'sequence_id':sample.sequence_id,'frame_id':sample.frame_id,'gt_hands':len(truth),
                'left_detections':len(ds[0]),'right_detections':len(ds[1]),'valid_output_joints':sum(h.valid_joint_count for h in result.hands),
                'comparable_gt_joints':len(frame_rows),'comparable_valid_joints':len(frame_rows)-rejected,
                'mpjpe_mm':frame_error['mean'],'status':result.reconstruction_status,
                'sync_error_ns':abs(sample.left_timestamp_ns-sample.right_timestamp_ns) if sample.left_timestamp_ns is not None and sample.right_timestamp_ns is not None else None}
            frame_results.append(frame_record)
            manifest.append({'dataset_id':sample.dataset_id,'sequence_id':sample.sequence_id,'frame_id':sample.frame_id,
                'image_sha256':sample.provenance.get('image_sha256'),'archive_sha256':sample.provenance.get('archive_sha256')})
            if not any(p.get('sequence_id')==sample.sequence_id for p in provenance):provenance.append({'sequence_id':sample.sequence_id,**sample.provenance,'joint_mapping':[asdict(m) for m in sample.joint_mapping]})
            # 正式导出用副本回原生左坐标，避免更改仍供跟踪/失败图使用的内部虚拟结果。
            exported=copy.deepcopy(result)
            if args.dataset=='hot3d':
                g=sample.native_annotations['geometry'];rotation=g.R_virtual_left.T
                exported.coordinate_frame='camera_left_optical'
                exported.diagnostics['pixel_coordinate_frame']='virtual_rectified_pixels'
                exported.diagnostics['output_rotation_native_from_virtual']=rotation.tolist()
                for h in exported.hands:
                    for j in h.joints:
                        if j.valid:j.xyz_m=g.virtual_to_original(j.xyz_m)
                        if j.position_covariance_m2 is not None:
                            j.position_covariance_m2=rotation@j.position_covariance_m2@rotation.T
                            j.depth_std_m=float(np.sqrt(max(j.position_covariance_m2[2,2],0)))
                exported.diagnostics['uncertainty_assumptions']['output_covariance_frame']='camera_left_optical'
                native_stds=[j.depth_std_m for h in exported.hands for j in h.joints if j.depth_std_m is not None]
                exported.diagnostics.setdefault('depth_uncertainty',{})['mean_depth_std_m']=float(np.mean(native_stds)) if native_stds else None
            results.append(exported)
            # Keep a bounded candidate pool, including complete failures and severe errors.
            severity=(frame_error['max'] or 0)+1000*(rejected/max(len(frame_rows),1))
            if len(failure_candidates)<12 or severity>min(x[0] for x in failure_candidates):
                case_id=sample.sequence_id+'_'+sample.frame_id
                case={'frame':frame_record,'provenance':sample.provenance,'gt_xyz_m':truth,
                    'gt_coordinate_frame':sample.ground_truth_frame,'gt_native_annotations':sample.native_annotations.get('hands'),
                    'result':result,'per_joint':frame_rows,'classification':'rejected_or_detection_failure' if rejected else 'largest_valid_error'}
                if args.dataset=='hot3d':case['result_xyz_original_left_m']=[[sample.native_annotations['geometry'].virtual_to_original(j.xyz_m) for j in h.joints] for h in result.hands]
                save_json(output/'failure_cases'/f'{case_id}.json',case)
                draw_case(sample,result,truth_uv,output/'failure_cases'/f'{case_id}.png')
                failure_candidates.append((severity,case_id));failure_candidates=sorted(failure_candidates,reverse=True)[:12]
            if counts['frames']%25==0:print(f"processed {counts['frames']} frames; valid 3D joints {counts['valid_output_joints']}",flush=True)
        if not counts['frames']:raise BenchmarkBlocked('No samples selected')
    except BenchmarkBlocked as e:status=e.status;blocked=str(e)
    except FileNotFoundError as e:status='BLOCKED_BY_DATA';blocked=str(e)
    finally:
        if wall_start is not None:wall_elapsed=time.perf_counter()-wall_start
        if detectors:
            for detector in detectors:detector.close()
    values=[r['error_mm'] for r in rows if r['error_mm'] is not None]
    errors=distribution(values)
    metric_valid=bool(rows)
    if errors['count'] and not blocked:status='BENCHMARK_VALIDATED'
    metrics={'status':status,'blocked_reason':blocked,'dataset':args.dataset,'mode':args.mode,'run_id':run_id,
        'counts':counts,'error_mm':errors,'MPJPE_mm':errors['mean'],
        'metric_valid':bool(errors['count']),'gt_available':metric_valid,'detector_input':'stereo_rectified_image','detector_input_option':args.detector_input,'gt_comparable_joints':len(rows),
        'valid_joint_rate':sum(r['valid'] for r in rows)/len(rows) if rows else None,
        'valid_joint_rate_output':counts['valid_output_joints']/(21*counts['predicted_hands']) if counts['predicted_hands'] else None,
        'rejection_or_no_reconstruction_rate':sum(not r['valid'] for r in rows)/len(rows) if rows else None,
        'both_view_detection_rate':counts['both_any_detection']/counts['frames'] if counts['frames'] else None,
        'either_view_detection_rate':counts['either_any_detection']/counts['frames'] if counts['frames'] else None,
        'overall_both_view_detection_failure_rate':1-counts['both_any_detection']/counts['frames'] if counts['frames'] else None,
        'projected_gt_hand_detection_recall':counts['detector_gt_hands_found']/counts['detector_gt_hands_visible'] if counts['detector_gt_hands_visible'] else None,
        'stereo_match_accuracy_evaluable_subset':counts['stereo_pairs_correct']/counts['stereo_pairs_evaluable'] if counts['stereo_pairs_evaluable'] else None,
        'stereo_match_accuracy_reason':'Evaluated only where independent 2D assignments identify both hands; all other pairs separately counted',
        'ID_switch_count_consecutive_observations':counts['id_switches_consecutive_observations'] if counts['stereo_pairs_evaluable'] and args.dataset=='hot3d' else None,
        'ID_switch_reason':'Consecutive GT hand observations only; gaps and unknown GT excluded; not a proof of continuous identity',
        '2D_landmark_error':None,'2D_landmark_error_reason':'No independent native pixel landmark GT; projected model GT is a proxy, never reported as measured 2D error',
        'processing_fps':counts['frames']/wall_elapsed if wall_elapsed else None,
        'processing_time_s':wall_elapsed,'engine_plus_evaluation_fps':counts['frames']/processing_s if processing_s else None,
        'native_detection_extra_time_s':native_s,
        'throughput_scope':'Full sample iteration including archive reads, official warp, inference, evaluation and failure capture; excludes download/model initialization/final report serialization; native extra pass included if requested',
        'native_detection_rate_left':counts['native_left_any_detection']/counts['frames'] if args.native_detection and counts['frames'] and args.mode=='mediapipe-stereo' else None,
        'native_detection_rate_right':counts['native_right_any_detection']/counts['frames'] if args.native_detection and counts['frames'] and args.mode=='mediapipe-stereo' else None,
        'quality_is_calibrated_probability':False,'hardware_sync_precision_verified':False,
        'detection_rate_kind':'actual_mediapipe' if args.mode=='mediapipe-stereo' else 'oracle_observations_supplied_not_detection',
        'noise_assumption_px':config.get('uncertainty',{}).get('assumed_pixel_std'),
        'coordinate_frame':'original_left_optical rotated equivalently from declared virtual_left_optical' if args.dataset=='hot3d' else 'camera_left_optical',
        'unit':'mm for errors, m for coordinates','mapping_joints':20 if args.dataset=='hot3d' else 17,
        'comparison_scope':'Model landmark correspondence, no root alignment, no scale alignment; all annotated comparable joints in denominator including outside overlap',
        'N_A_reasons':{'MPJPE':None if errors['count'] else blocked or 'No verified GT matched to accepted 3D joints',
                      'occlusion':'HOT3D modeled per-hand visibility strata; not per-joint measured occlusion' if args.dataset=='hot3d' else 'Occlusion GT unavailable'}}
    save_json(output/'metrics.json',metrics);save_json(output/'provenance.json',provenance)
    save_json(output/'sample_manifest.json',manifest);save_json(output/'results.json',results)
    write_csv(output/'metrics.csv',[{k:v for k,v in metrics.items() if not isinstance(v,dict)}])
    write_csv(output/'per_frame.csv',frame_results);write_csv(output/'per_joint.csv',rows)
    for key,filename in [('joint_name','error_by_joint.csv'),('sequence_id','error_by_sequence.csv'),('depth_bin','error_by_depth.csv'),('disparity_bin','error_by_disparity.csv'),('occlusion_bin','error_by_occlusion.csv'),('status','error_by_type.csv')]:
        write_csv(output/filename,grouped_joint_errors(rows,key))
    save_plots(output,rows,metrics)
    if results:export_csv(results,output/'reconstruction.csv');export_npz(results,output/'reconstruction.npz')
    save_json(output/'failure_cases/index.json',[{'severity':a,'case_id':b} for a,b in failure_candidates])
    links=''.join(f'<li>{name}: <a href="failure_cases/{name}.json">JSON</a><br><img src="failure_cases/{name}.png" style="max-width:100%"></li>' for _,name in failure_candidates)
    (output/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Stereo benchmark</title><style>body{max-width:1200px;margin:auto;font:16px system-ui}pre{white-space:pre-wrap}</style><h1>'+args.dataset+' '+args.mode+'</h1><pre>'+json.dumps(serializable(metrics),indent=2)+'</pre><img src="coverage_error.png"><img src="error_distribution.png"><h2>Failures and largest errors</h2><ol>'+links+'</ol>',encoding='utf8')
    print(json.dumps({k:metrics[k] for k in ('status','blocked_reason','MPJPE_mm','valid_joint_rate','both_view_detection_rate','processing_fps')},indent=2))
    return 2 if blocked else 0

