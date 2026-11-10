"""固定均匀抽帧比较 HOT3D 检测设置；真值只用于事后评分。"""
import json, sys, tarfile, time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np, yaml
from egohand.benchmarks.hot3d import HOT3DBenchmarkAdapter
from egohand.benchmarks.runner import project_truth
from egohand.benchmarks.metrics import match_2d
from egohand.core.pipeline import EgoHandEngine
from egohand.core.types import StereoFrame
from egohand.detection.mediapipe_detector import MediaPipeHandDetector

ROOT=Path('samples/date/hot3d')
OUT=Path('benchmark-results/paired_no_roi_20261104')
FRAMES_PER_CLIP=15
SETTINGS=[(.2,.5)]
OUT.mkdir(parents=True,exist_ok=True)
roi={'left':None,'right':None}
archives=[ROOT/'train_aria'/f'{name}.tar' for name in ('clip-001849','clip-001851','clip-001894','clip-001898','clip-003365')]
ids=[]
for path in archives:
    with tarfile.open(path) as tar:
        frames=sorted(n.split('.')[0] for n in tar.getnames() if n.endswith('.info.json'))
    ids.extend(path.stem+'/'+frames[i] for i in np.linspace(0,len(frames)-1,min(FRAMES_PER_CLIP,len(frames)),dtype=int))
(OUT/'manifest.json').write_text(json.dumps({'ids':ids,'roi':roi,'settings':SETTINGS},ensure_ascii=False,indent=2),encoding='utf8')
detectors={}
for domain in ('roi','rectified'):
    for det,pres in SETTINGS:
        detectors[domain,det,pres]=[MediaPipeHandDetector('models/hand_landmarker.task',min_hand_detection_confidence=det,min_hand_presence_confidence=pres,roi=roi[side]) for side in ('left','right')]
records=[]
try:
    adapter=HOT3DBenchmarkAdapter(ROOT,max_frames=len(ids),frame_ids=ids)
    for idx,s in enumerate(adapter.iter_samples()):
        truth,uv=project_truth(s)
        for (domain,det,pres),ds in detectors.items():
            start=time.perf_counter()
            g=s.native_annotations['geometry']
            pred=[]
            if domain=='rectified':
                pred=[ds[i].detect(im,s.left_timestamp_ns) for i,im in enumerate((s.left_image,s.right_image))]
            else:
                images=g.warp_detection(s.native_annotations['raw_images'])
                for i,side in enumerate(('left','right')):
                    old=ds[i].roi
                    ds[i].roi=g.rectifier.rectified_roi_to_detection(old,side)
                    try:detected=ds[i].detect(images[i],s.left_timestamp_ns)
                    finally:ds[i].roi=old
                    pred.append(g.rectifier.detection_to_rectified(detected,side))
            duration=time.perf_counter()-start
            matches=[match_2d(pred[i],uv[i]) for i in (0,1)]
            engine=EgoHandEngine(calibration=s.calibration)
            frame=StereoFrame(idx,int(s.left_timestamp_ns),s.left_image,s.right_image,s.left_timestamp_ns,s.right_timestamp_ns,s.calibration.calibration_id,'benchmark:hot3d',False,s.image_geometry,{'timestamp_kind':'dataset_sensor','real_stereo':True,'rectification_id':s.calibration.calibration_id,'synchronization_evidence':s.provenance['source']},pred[0],pred[1])
            pair={'dataset_id':'hot3d','sync_quality':'DATASET_DOCUMENTED','pair_id':s.sequence_id+'/'+s.frame_id,'source':s.provenance['source']}
            result=engine.process(frame,dataset_pairing=pair)
            assert 'association_quality' in result.diagnostics, (s.sequence_id,s.frame_id,result.reconstruction_status,result.diagnostics.get('reason'))
            pairs=result.diagnostics['association_quality'].get('accepted_pairs',[])
            errors=[]; correct=0; evaluable=0
            for hi,(li,ri) in enumerate(pairs):
                l=matches[0][0].get(li);r=matches[1][0].get(ri)
                if l is not None and r is not None:
                    evaluable+=1;correct+=l==r
                if l is not None and hi<len(result.hands):
                    for j,gt in enumerate(truth[l]):
                        joint=result.hands[hi].joints[j]
                        if np.isfinite(gt).all() and joint.valid:errors.append(float(np.linalg.norm(joint.xyz_m-gt)*1000))
            records.append({'clip':s.sequence_id,'frame':s.frame_id,'domain':domain,'detection':det,'presence':pres,'gt':bool(truth),'gt_visible':sum(int(mask.sum()>=4) for view in uv for _,mask in view.values()),'matched':sum(len(m[0]) for m in matches),'unmatched_predictions':sum(len(pred[i])-len(matches[i][0]) for i in (0,1)) if truth else None,'left':len(pred[0]),'right':len(pred[1]),'pairs':len(pairs),'pairs_evaluable':evaluable,'pairs_correct':correct,'valid_output_joints':sum(h.valid_joint_count for h in result.hands),'gt_joints':sum(int(np.isfinite(v).all(axis=1).sum()) for v in truth.values()),'errors_mm':errors,'detection_seconds':duration})
        print(f'{idx+1}/{len(ids)} {s.sequence_id}/{s.frame_id}',flush=True)
        (OUT/'records.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf8')
finally:
    for ds in detectors.values():
        for d in ds:d.close()
print('DONE',flush=True)



