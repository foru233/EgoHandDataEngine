"""用 VS030 真实录制验证隔离超时后由原生 ORB 建图并继续处理。"""
import json,os,sys
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from egohand.vio import replay_runtime as rr
original_popen=rr.subprocess.Popen;original_track=rr.ReplayBridge.track
def worker(args,*a,**kw):
 if args[1:3]==['-m','egohand.vio.live_subprocess']:args=[args[0],str(ROOT/'scripts/test_auto_new_map_worker.py'),args[-1]]
 return original_popen(args,*a,**kw)
def track(self,stamp,left,right,measurements):
 return original_track(self,stamp,cv2.resize(left,(800,600),interpolation=cv2.INTER_AREA),cv2.resize(right,(800,600),interpolation=cv2.INTER_AREA),measurements)
if __name__=='__main__':
 os.environ['EGOHAND_VIO_CONFIG']=str(ROOT/'configs/vio/vs030_rectified_half.yaml')
 rr.subprocess.Popen=worker;rr.ReplayBridge.track=track
 output=ROOT/'results/vio/auto_new_map_real_recording_v8'
 report=rr.ReplayVio(ROOT/'data/imu/session_20261110_112020_689246900',output_dir=output,
  timing={'imu_alignment':'lattice','lattice_reference':'packet','lattice_phase_ms':97.0},max_frames=450,stride=2).run()
 rows=[json.loads(line) for line in (output/'poses.jsonl').read_text(encoding='utf8').splitlines()]
 switches=[(i,r) for i,r in enumerate(rows) if r.get('automatic_new_map')]
 resumed=False
 if switches:
  i,row=switches[0];resumed=any(r.get('tracking_state')=='OK' and r.get('imu_initialized') and r.get('pose_valid') and r.get('map_epoch')!=row.get('map_epoch') for r in rows[i+1:])
 print(json.dumps({'status':report['status'],'switches':len(switches),'resumed_in_new_map':resumed,'frames':report['frames'],'error':report['error']},ensure_ascii=False),flush=True)
 if report['status']!='completed' or len(switches)!=1 or not resumed:raise SystemExit('自动切图未通过实际回放验收')
