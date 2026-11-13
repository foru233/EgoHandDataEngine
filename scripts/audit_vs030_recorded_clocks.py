"""只审计已有 SDK 录制的候选时间字段，不推断曝光时刻或厂商字段语义。"""
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from egohand.storage.sdk_recording import iter_imu_packets
from egohand.vio.vs030_imu_candidate import parse_candidate_payload
out=[]
for rec in sorted((ROOT/'data/imu').glob('session_*')):
 v=[]
 try:
  for p in iter_imu_packets(rec):
   r=parse_candidate_payload(p.payload);t=int(p.device_ts)*1000
   v.append((t,r[0].time_a_raw-t,r[0].time_b_raw-t,len(r)))
 except Exception as exc:out.append({'session':rec.name,'error':str(exc)});continue
 if len(v)<3:continue
 arr=np.asarray(v,dtype=np.float64);secs=(arr[:,0]-arr[0,0])*1e-9
 result={'session':rec.name,'packets':len(v),'duration_s':float(secs[-1]),'sample_counts':{str(int(k)):int(n) for k,n in zip(*np.unique(arr[:,3],return_counts=True))}}
 for col,label in ((1,'candidate_time_a'),(2,'candidate_time_b')):
  delta=arr[:,col]/1e6;fit=np.polyfit(secs,delta,1);res=delta-np.polyval(fit,secs)
  result[label]={'offset_start_ms':float(delta[0]),'offset_end_ms':float(delta[-1]),'slope_ms_per_s':float(fit[0]),'residual_std_ms':float(np.std(res))}
 out.append(result)
report={'measurements':out,'note':'time_a/time_b 与 device_ts 的单位、时钟域、曝光/采样语义和样本顺序仍需厂商确认；此分析不证明时间同步或 97ms 相位正确。'}
p=ROOT/'results/vio/vs030_recorded_clock_audit.json';p.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8');print(json.dumps(report,ensure_ascii=False),flush=True)
