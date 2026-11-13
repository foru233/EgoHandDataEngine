"""EuRoC 官方双目+IMU 基准：建立锚点后遮挡，核对恢复并以刚体对齐评估机体轨迹。"""
import csv,json,os,sys
from collections import Counter
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from egohand.vio.native_bridge import NativeVioBridge,ImuMeasurement
from egohand.vio.recovery_coordinator import RecoveryCoordinator

root=ROOT/'samples/date/VIO/converted/EuRoC/MH_01_easy'
out=ROOT/'results/vio'/os.environ.get('EGOHAND_TEST_OUTPUT','euroc_real_anchor_recovery_v2');out.mkdir(parents=True,exist_ok=True)
occlusion_frames=int(os.environ.get('EGOHAND_TEST_OCCLUSION_FRAMES','8'))
shift_px=int(os.environ.get('EGOHAND_TEST_SHIFT_PX','0'))
freeze_after_loss=os.environ.get('EGOHAND_TEST_FREEZE_VIEW')=='1'
stamps=[int(x) for x in (root/'timestamps_inertial.txt').read_text().split()][:1800]
samples=[]
with (root/'mav0/imu0/data.csv').open() as f:
 for row in csv.reader(line for line in f if not line.startswith('#')):
  if row:samples.append(ImuMeasurement(int(row[0]),tuple(map(float,row[4:7])),tuple(map(float,row[1:4]))))
coord=RecoveryCoordinator();now=[0];anchor_index=None;anchor_epoch=None;first_recovery=None;recovery_epoch=None;rows=[];i=0;previous=None;frozen=None
with NativeVioBridge(ROOT/'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt',root/'calibration/EuRoC.yaml') as bridge:
 def check(T,epoch):return {'consistent':1,'inconsistent':0,'unavailable':-1}[coord.check_candidate(T,now[0],epoch)['status']]
 bridge.set_recovery_callback(check)
 for n,stamp in enumerate(stamps):
  now[0]=stamp;batch=[]
  while i<len(samples) and samples[i].timestamp_ns<=stamp:
   if previous is not None and samples[i].timestamp_ns>previous:batch.append(samples[i])
   i+=1
  coord.begin_frame(batch)
  images=[np.asarray(Image.open(root/f'mav0/cam{k}/data/{stamp}.png').convert('L')).copy() for k in (0,1)]
  altered=False
  if freeze_after_loss and anchor_index is not None and n==anchor_index+29:
   frozen=[im.copy() for im in images]
  if anchor_index is not None and anchor_index+30<=n<anchor_index+30+occlusion_frames:
   for im in images:im.fill(0)
   altered=True
  if shift_px>0 and anchor_index is not None and anchor_index+30+occlusion_frames<=n<anchor_index+40+occlusion_frames:
   images=[np.roll(im,shift_px,axis=1) for im in images]
   for im in images:im[:,:shift_px]=0
   altered=True
  if freeze_after_loss and frozen is not None and anchor_index+30+occlusion_frames<=n<anchor_index+70+occlusion_frames:
   images=[im.copy() for im in frozen];altered=True
  result=bridge.track_stereo(*images,stamp,batch)
  body=bridge.get_candidate_body_state() if result.pose_valid and result.imu_initialized else None
  if body is not None:body={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in body.items()}
  response={'timestamp_ns':stamp,'tracking_state':result.tracking_state_name,'pose_valid':result.pose_valid,
   'imu_initialized':result.imu_initialized,'map_epoch':result.map_epoch,
   'matrix':result.T_world_from_rectified_left_camera.tolist() if result.pose_valid else None}
  recovery=coord.finish_frame(response,body)
  rows.append({'n':n,'timestamp_ns':stamp,'tracking_state':response['tracking_state'],'map_epoch':result.map_epoch,'synthetic_image':altered,
   'body':body,'recovery':recovery})
  if anchor_index is None and recovery['anchor_available'] and not recovery['await_visual_requalification']:
   anchor_index=n;anchor_epoch=result.map_epoch;print('ANCHOR',n,'epoch',anchor_epoch,flush=True)
  if anchor_index is not None and n>=anchor_index+30+occlusion_frames and result.pose_valid and (recovery['direct_recovery'] or recovery['candidates']):
   if first_recovery is None:first_recovery=n;recovery_epoch=result.map_epoch;print('RECOVERY',n,'epoch',recovery_epoch,recovery['direct_recovery'],recovery['candidates'],flush=True)
  previous=stamp
  if first_recovery is not None and n>=first_recovery+80:break
 bridge.set_recovery_callback(None)
with (out/'poses.jsonl').open('w',encoding='utf8') as f:
 for r in rows:f.write(json.dumps(r,allow_nan=False)+'\n')
gt=[]
with (ROOT/'samples/date/VIO/EuRoC/MH_01_easy/MH_01_easy_gt_openvins.csv').open() as f:
 for r in csv.reader(line for line in f if not line.startswith('#')):
  if r:gt.append((int(r[0]),np.array(list(map(float,r[1:4])))))
gt_times=np.array([x[0] for x in gt],dtype=np.int64);X=[];Y=[]
epoch_counts=Counter(r['map_epoch'] for r in rows if r['body'] is not None and not r['recovery']['await_visual_requalification'])
metric_epoch=epoch_counts.most_common(1)[0][0] if epoch_counts else None
for r in rows:
 if r['body'] is None or r['recovery']['await_visual_requalification'] or r['map_epoch']!=metric_epoch or r['synthetic_image']:continue
 t=r['timestamp_ns'];j=int(np.searchsorted(gt_times,t));j=min(max(j,1),len(gt_times)-1)
 if min(abs(int(gt_times[j])-t),abs(int(gt_times[j-1])-t))>10_000_000:continue
 a=(t-int(gt_times[j-1]))/(int(gt_times[j])-int(gt_times[j-1]));p=(1-a)*gt[j-1][1]+a*gt[j][1]
 X.append(np.array(r['body']['T_world_from_body'])[:3,3]);Y.append(p)
metrics=None
if len(X)>=3:
 X=np.array(X);Y=np.array(Y);xc=X.mean(0);yc=Y.mean(0);U,_,Vt=np.linalg.svd((X-xc).T@(Y-yc));D=np.diag([1,1,np.linalg.det(Vt.T@U.T)]);R=Vt.T@D@U.T
 e=np.linalg.norm((X-xc)@R.T+yc-Y,axis=1);metrics={'pairs':len(X),'map_epoch':metric_epoch,'position_rmse_m':float(np.sqrt(np.mean(e**2))),'position_median_m':float(np.median(e)),'alignment':'post-hoc rigid SE(3), no scale; longest single-epoch non-quarantined frames'}
same_map=anchor_epoch is not None and recovery_epoch is not None and (int(anchor_epoch)>>20)==(int(recovery_epoch)>>20)
recovery_row=next((r for r in rows if r['n']==first_recovery),None)
decision=(recovery_row['recovery']['direct_recovery'] or (recovery_row['recovery']['candidates'] or [None])[-1]) if recovery_row else None
verified=bool(same_map and decision and decision.get('status')=='consistent' and not recovery_row['recovery']['await_visual_requalification'])
report={'frames':len(rows),'occlusion_frames':occlusion_frames,'synthetic_shift_px':shift_px,'freeze_after_loss':freeze_after_loss,'anchor_index':anchor_index,'anchor_epoch':anchor_epoch,'first_recovery_index':first_recovery,'recovery_epoch':recovery_epoch,'same_map':same_map,'imu_heuristic_consistent':verified,'recovery_decision':decision,'quarantined_frames':sum(x['recovery']['await_visual_requalification'] for x in rows),'ground_truth':metrics,'note':'恢复判定使用经验门限，并非统计验证；遮挡、图像平移及冻结为测试人为注入；离线对齐不能证明 VS030 实时精度。'}
(out/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8');print(json.dumps(report,ensure_ascii=False),flush=True)
if anchor_index is None or first_recovery is None or metrics is None or not same_map:
 raise SystemExit('未完成锚点、原地图遮挡恢复和同地图真值对照，结果不可判定')
