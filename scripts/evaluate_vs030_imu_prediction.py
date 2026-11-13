"""量化 VS030 录制中 IMU 外推与 ORB 输出的自一致性，不视为真值误差。"""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from egohand.vio.imu_lattice import PacketLatticeImuBuffer
from egohand.storage.sdk_recording import iter_imu_packets
from egohand.vio.native_bridge import ImuMeasurement
from egohand.vio.imu_loss_prediction import BodyState,integrate_loss
rec=ROOT/'data/imu/session_20261110_110623_140565800'
rows=[json.loads(line) for line in (ROOT/'results/vio/recovery_real_anchor_occlusion_v4/poses.jsonl').read_text(encoding='utf8').splitlines()]
buf=PacketLatticeImuBuffer(sample_rate_hz=480,camera_to_imu_drift_ppm=0,phase_ns=97_000_000,capacity=100000)
for p in iter_imu_packets(rec):buf.add(p.payload,p.device_ts)
imu=[ImuMeasurement(row[0],tuple(row[1:4]),tuple(row[4:7])) for row in buf.rows];times=np.array([x.timestamp_ns for x in imu],dtype=np.int64)
buckets={str(x):[] for x in (0.1,0.3,0.5,1.0,1.5,2.0)}
for i,row in enumerate(rows):
 b=row.get('body_state_candidate');q=row.get('recovery_preaccept') or {}
 if i%10 or not b or not b.get('ba2_complete') or q.get('await_visual_requalification') or not row.get('pose_valid'):continue
 t0=row['timestamp_ns'];T=np.array(b['T_world_from_body']);state=BodyState(t0,T[:3,:3],T[:3,3],np.array(b['velocity_world_mps']),np.array(b['accel_bias_mps2']),np.array(b['gyro_bias_rps']),str(row['map_epoch']))
 for label,result in buckets.items():
  target=t0+int(float(label)*1e9);j=min(range(i+1,len(rows)),key=lambda k:abs(rows[k]['timestamp_ns']-target),default=None)
  if j is None or abs(rows[j]['timestamp_ns']-target)>10_000_000:continue
  tail=rows[j];end_body=tail.get('body_state_candidate')
  if not end_body or any(x['tracking_state']!='OK' or x['map_epoch']!=row['map_epoch'] or (x.get('recovery_preaccept') or {}).get('await_visual_requalification') for x in rows[i:j+1]):continue
  t1=tail['timestamp_ns'];lo=max(0,int(np.searchsorted(times,t0))-1);hi=min(len(imu),int(np.searchsorted(times,t1))+1)
  try:pred=integrate_loss(state,imu[lo:hi],t1,max_gap_ns=20_000_000,max_duration_ns=2_050_000_000)
  except ValueError:continue
  T1=np.array(end_body['T_world_from_body']);angle=float(np.degrees(Rotation.from_matrix(T1[:3,:3].T@pred.rotation_world_from_body).magnitude()))
  result.append({'position_residual_m':float(np.linalg.norm(pred.position_world_m-T1[:3,3])),'rotation_residual_deg':angle})
report={'source':'VS030 110623 录制与同一次 ORB 回放','note':'比较两种相关估计，仅为自一致性，不是独立真值、精度或置信度；使用实验性 97ms 时间对齐。','buckets':{}}
for k,v in buckets.items():
 entry={'samples':len(v)}
 if v:
  for field in ('position_residual_m','rotation_residual_deg'):
   a=np.array([x[field] for x in v]);entry[field]={'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(a.max())}
 report['buckets'][k]=entry
out=ROOT/'results/vio/vs030_imu_prediction_consistency.json';out.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8');print(json.dumps(report,ensure_ascii=False),flush=True)
