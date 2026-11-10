import json,statistics,collections
from pathlib import Path
p=Path('benchmark-results/parameter_sweep_rectified_current')
r=json.loads((p/'records.json').read_text(encoding='utf8'))
out=[]
for domain in ('roi','virtual'):
 for det,pres in [(.2,.5),(.3,.5),(.4,.5),(.5,.5),(.3,.3)]:
  for clip in ('GT_ALL','clip-001849','clip-001851','clip-001894','clip-001898','clip-003365'):
   a=[x for x in r if x['domain']==domain and x['detection']==det and x['presence']==pres and (x['gt'] if clip=='GT_ALL' else x['clip']==clip)]
   err=[e for x in a for e in x['errors_mm']]; visible=sum(x['gt_visible'] for x in a); gj=sum(x['gt_joints'] for x in a)
   out.append(dict(domain=domain,detection=det,presence=pres,group=clip,frames=len(a),gt_frames=sum(x['gt'] for x in a),left=sum(x['left']>0 for x in a),right=sum(x['right']>0 for x in a),both=sum(x['left']>0 and x['right']>0 for x in a),gt_visible=visible,gt_matched=sum(x['matched'] for x in a),unmatched_predictions=sum(x['unmatched_predictions'] or 0 for x in a if x['gt']),pairs=sum(x['pairs'] for x in a),pairs_evaluable=sum(x['pairs_evaluable'] for x in a),pairs_correct=sum(x['pairs_correct'] for x in a),valid_output_joints=sum(x['valid_output_joints'] for x in a),gt_joints=gj,comparable_valid_joints=len(err),mpjpe_mm=statistics.mean(err) if err else None,detection_pair_ms=statistics.mean(x['detection_seconds'] for x in a)*1000 if a else None))
(p/'summary.json').write_text(json.dumps(out,indent=2,ensure_ascii=False),encoding='utf8')
for x in out:
 if x['group']=='GT_ALL':print(x)
