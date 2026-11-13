"""回放时短暂遮挡双目，验证强制拒绝由真实 ORB 重定位回调执行。"""
import json
import os
import sys
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from egohand.vio import replay_runtime as rr

original_popen=rr.subprocess.Popen
original_track=rr.ReplayBridge.track
def worker(args,*a,**kw):
    if args[1:3]==['-m','egohand.vio.live_subprocess']:
        args=[args[0],str(ROOT/'scripts/test_recovery_native_worker.py'),args[-1]]
    return original_popen(args,*a,**kw)
def track(self,stamp,left,right,measurements):
    self.test_index=getattr(self,'test_index',0)+1
    left=cv2.resize(left,(800,600),interpolation=cv2.INTER_AREA)
    right=cv2.resize(right,(800,600),interpolation=cv2.INTER_AREA)
    if 95<=self.test_index<100:left.fill(0);right.fill(0)
    return original_track(self,stamp,left,right,measurements)

if __name__=='__main__':
    os.environ['EGOHAND_VIO_CONFIG']=str(ROOT/'configs/vio/vs030_rectified_half.yaml')
    rr.subprocess.Popen=worker;rr.ReplayBridge.track=track
    report=rr.ReplayVio(ROOT/'data/imu/session_20261110_112020_689246900',
        output_dir=ROOT/'results/vio/native_recovery_forced_reject',
        timing={'imu_alignment':'lattice','lattice_reference':'packet','lattice_phase_ms':97.0},
        max_frames=150,stride=2).run()
    print(json.dumps({k:report[k] for k in ('status','frames','error')},ensure_ascii=False),flush=True)
    if report['status']!='completed':raise SystemExit(1)
    rows=[json.loads(line) for line in (ROOT/'results/vio/native_recovery_forced_reject/poses.jsonl').read_text(encoding='utf8').splitlines()]
    attempts=[candidate for row in rows for candidate in (row.get('recovery_preaccept') or {}).get('candidates',[])
              if candidate.get('reason')=='test_forced_reject']
    log=(ROOT/'results/vio/native_recovery_forced_reject/run.log').read_text(encoding='utf8',errors='replace')
    accepted=log.count('Relocalization rejected by full-interval recovery check')
    print(f'forced_rejected_candidates={len(attempts)} native_rejection_log_count={accepted}',flush=True)
    if not attempts or accepted<len(attempts):raise SystemExit('未证明原生候选拒绝链路生效')
