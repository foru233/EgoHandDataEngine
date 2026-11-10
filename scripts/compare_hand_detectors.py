"""固定 HOT3D 训练帧、无 ROI 的二维模型对照；GT 仅用于事后评分。"""
import sys,json,tarfile,time,statistics
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from egohand.benchmarks.hot3d import HOT3DBenchmarkAdapter
from egohand.benchmarks.runner import project_truth
from egohand.benchmarks.metrics import match_2d
from egohand.detection.mediapipe_detector import MediaPipeHandDetector
from egohand.detection.rtmpose_detector import RTMPoseHandDetector
ROOT=Path('samples/date/hot3d');OUT=Path('benchmark-results/mediapipe_vs_rtmpose_20261104');OUT.mkdir(parents=True,exist_ok=True)
clips=['clip-001849','clip-001851','clip-001894'];ids=[]
for clip in clips:
 with tarfile.open(ROOT/'train_aria'/f'{clip}.tar') as f:frames=sorted(n.split('.')[0] for n in f.getnames() if n.endswith('.info.json'))
 ids.extend(f'{clip}/{frames[i]}' for i in np.linspace(0,len(frames)-1,15,dtype=int))
ds={'mediapipe':[MediaPipeHandDetector('models/hand_landmarker.task',min_hand_detection_confidence=.2) for _ in range(2)],'rtmpose':[RTMPoseHandDetector() for _ in range(2)]}
records=[]
try:
 for k,s in enumerate(HOT3DBenchmarkAdapter(ROOT,max_frames=len(ids),frame_ids=ids).iter_samples()):
  truth,uv=project_truth(s)
  for model,detectors in ds.items():
   t=time.perf_counter();pred=[detectors[i].detect(im,s.left_timestamp_ns) for i,im in enumerate((s.left_image,s.right_image))];elapsed=time.perf_counter()-t
   found=visible=unmatched=0;errors=[]
   for side in (0,1):
    m,_=match_2d(pred[side],uv[side]);found+=len(m);visible+=sum(int(mask.sum()>=4) for _,mask in uv[side].values());unmatched+=len(pred[side])-len(m)
    for i,name in m.items():
     gt,mask=uv[side][name];d=pred[side][i];valid=mask&d.valid&np.isfinite(d.pixel_landmarks).all(axis=1)
     errors.extend(np.linalg.norm(d.pixel_landmarks[valid]-gt[valid],axis=1).tolist())
   records.append({'clip':s.sequence_id,'frame':s.frame_id,'model':model,'left':len(pred[0]),'right':len(pred[1]),'visible':visible,'matched':found,'unmatched':unmatched,'errors_px':errors,'seconds':elapsed})
  (OUT/'records.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf8')
  print(k+1,len(ids),s.sequence_id,s.frame_id,flush=True)
finally:
 for pair in ds.values():
  for d in pair:d.close()
summary={}
for model in ds:
 a=[x for x in records if x['model']==model];err=[e for x in a for e in x['errors_px']]
 summary[model]={'frames':len(a),'left_frames':sum(x['left']>0 for x in a),'right_frames':sum(x['right']>0 for x in a),'both_frames':sum(x['left']>0 and x['right']>0 for x in a),'visible_gt_hands':sum(x['visible'] for x in a),'matched_gt_hands':sum(x['matched'] for x in a),'unmatched_predictions':sum(x['unmatched'] for x in a),'comparable_points':len(err),'mean_error_px':statistics.mean(err) if err else None,'median_error_px':statistics.median(err) if err else None,'mean_pair_ms':statistics.mean(x['seconds'] for x in a)*1000}
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(summary,indent=2),flush=True)

