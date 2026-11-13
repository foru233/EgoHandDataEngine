"""量化 EuRoC 可信锚点后的 IMU 外推误差；不以该短片段自动改生产门限。"""
import csv,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from egohand.vio.imu_loss_prediction import BodyState,integrate_loss
from egohand.vio.native_bridge import ImuMeasurement

rows=[json.loads(line) for line in (ROOT/'results/vio/euroc_real_anchor_recovery_v2/poses.jsonl').read_text(encoding='utf8').splitlines()]
imu=[]
with (ROOT/'samples/date/VIO/converted/EuRoC/MH_01_easy/mav0/imu0/data.csv').open() as f:
 for r in csv.reader(line for line in f if not line.startswith('#')):
  if r:imu.append(ImuMeasurement(int(r[0]),tuple(map(float,r[4:7])),tuple(map(float,r[1:4]))))
it=np.array([r.timestamp_ns for r in imu],dtype=np.int64)
gt=[]
with (ROOT/'samples/date/VIO/EuRoC/MH_01_easy/MH_01_easy_gt_openvins.csv').open() as f:
 for r in csv.reader(line for line in f if not line.startswith('#')):
  if r:gt.append((int(r[0]),np.array(list(map(float,r[1:4]))),Rotation.from_quat([float(r[5]),float(r[6]),float(r[7]),float(r[4])]).as_matrix()))
gt_t=np.array([g[0] for g in gt],dtype=np.int64)
def truth(t):
 j=int(np.searchsorted(gt_t,t)); candidates=[i for i in (j-1,j) if 0<=i<len(gt)]
 if not candidates:return None
 j=min(candidates,key=lambda k:abs(gt[k][0]-t))
 return gt[j] if abs(gt[j][0]-t)<=3_000_000 else None

buckets={str(x):[] for x in (0.1,0.3,0.5,1.0,1.5,2.0)}
for i,row in enumerate(rows):
 body=row.get('body')
 if i%5 or not body or not body.get('ba2_complete') or row['recovery']['await_visual_requalification']:continue
 t0=int(row['timestamp_ns']);g0=truth(t0)
 if g0 is None:continue
 T=np.array(body['T_world_from_body']);anchor=BodyState(t0,T[:3,:3],T[:3,3],np.array(body['velocity_world_mps']),np.array(body['accel_bias_mps2']),np.array(body['gyro_bias_rps']),str(row['map_epoch']))
 alignment=g0[2]@T[:3,:3].T
 for label,results in buckets.items():
  target=t0+int(float(label)*1e9);j=min(range(i+1,len(rows)),key=lambda k:abs(rows[k]['timestamp_ns']-target),default=None)
  if j is None or abs(rows[j]['timestamp_ns']-target)>10_000_000:continue
  tail=rows[j];body_j=tail.get('body');g1=truth(int(tail['timestamp_ns']))
  if not body_j or g1 is None or tail['map_epoch']!=row['map_epoch'] or any(x['tracking_state']!='OK' or x['map_epoch']!=row['map_epoch'] or x['recovery']['await_visual_requalification'] for x in rows[i:j+1]):continue
  t1=int(tail['timestamp_ns']);lo=max(0,int(np.searchsorted(it,t0))-1);hi=min(len(imu),int(np.searchsorted(it,t1))+1)
  try:pred=integrate_loss(anchor,imu[lo:hi],t1,max_gap_ns=20_000_000,max_duration_ns=2_050_000_000)
  except ValueError:continue
  position=g0[1]+alignment@(pred.position_world_m-anchor.position_world_m)
  Rpred=alignment@pred.rotation_world_from_body
  angle=float(np.degrees(Rotation.from_matrix(g1[2].T@Rpred).magnitude()))
  results.append({'position_error_m':float(np.linalg.norm(position-g1[1])),'rotation_error_deg':angle})
report={'source':'EuRoC MH_01_easy; OpenVINS-format ground truth','method':'每个锚点用真值体位姿建立局部刚体对齐；使用 ORB 速度与偏置和真实 IMU 中点积分；跳过丢失及地图版本变化','buckets':{},'note':'短片段离线诊断，样本相关，不用于声明 VS030 的精度或自动设定统计置信门限。'}
for k,vals in buckets.items():
 entry={'samples':len(vals)}
 if vals:
  for field in ('position_error_m','rotation_error_deg'):
   a=np.array([x[field] for x in vals]);entry[field]={'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(a.max())}
 report['buckets'][k]=entry
out=ROOT/'results/vio/euroc_imu_prediction_error.json';out.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8');print(json.dumps(report,ensure_ascii=False),flush=True)
