"""用真实 VS030 录制，在首次建立可信锚点后人为遮挡双目并观察恢复；不修改原始录制。"""
import json, os, sys
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from egohand.vio import replay_runtime as rr
original=rr.ReplayBridge.track

def track(self,stamp,left,right,measurements):
    self.test_index=getattr(self,'test_index',0)+1
    left=cv2.resize(left,(800,600),interpolation=cv2.INTER_AREA)
    right=cv2.resize(right,(800,600),interpolation=cv2.INTER_AREA)
    if hasattr(self,'occlusion_start') and self.occlusion_start<=self.test_index<self.occlusion_start+8:
        left.fill(0);right.fill(0)
    result=original(self,stamp,left,right,measurements)
    if not hasattr(self,'occlusion_start') and (result.get('recovery_preaccept') or {}).get('anchor_available'):
        self.occlusion_start=self.test_index+30
        print('trusted_anchor_frame',self.test_index,'occlusion_start',self.occlusion_start,flush=True)
    return result

if __name__=='__main__':
    os.environ['EGOHAND_VIO_CONFIG']=str(ROOT/'configs/vio/vs030_rectified_half.yaml')
    rr.ReplayBridge.track=track
    report=rr.ReplayVio(ROOT/'data/imu/session_20261110_110623_140565800',
        output_dir=ROOT/'results/vio/recovery_real_anchor_occlusion_v4',
        timing={'imu_alignment':'lattice','lattice_reference':'packet','lattice_phase_ms':97.0},
        max_frames=None,stride=1).run()
    print(json.dumps({k:report[k] for k in ('status','frames','error')},ensure_ascii=False),flush=True)
    if report['status']!='completed':raise SystemExit(1)
    rows=[json.loads(line) for line in (ROOT/'results/vio/recovery_real_anchor_occlusion_v4/poses.jsonl').read_text(encoding='utf8').splitlines()]
    anchors=sum(bool((row.get('recovery_preaccept') or {}).get('anchor_available')) for row in rows)
    quarantined=sum(bool((row.get('recovery_preaccept') or {}).get('await_visual_requalification')) for row in rows)
    predictions=sum(((((row.get('recovery_preaccept') or {}).get('loss_prediction')) or {}).get('status')=='predicted') for row in rows)
    print(json.dumps({'anchor_frames':anchors,'quarantined_frames':quarantined,'imu_prediction_frames':predictions}),flush=True)
    if not anchors or not predictions:
        raise SystemExit('未形成可信锚点后的惯性外推证据；结果不可判定')
