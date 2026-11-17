"""Summarize a completed isolated SMPL fitting audit, including tail failures."""
import argparse,json
from pathlib import Path
import numpy as np

ap=argparse.ArgumentParser();ap.add_argument('input',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
rows=json.loads(args.input.read_text(encoding='utf8'))
valid=[r for r in rows if r.get('valid')]
summary={'frames':len(rows),'valid_frames':len(valid),'invalid_frames':len(rows)-len(valid)}
for key in ('head_error_m','right_wrist_error_m','mean_joint_error_m','vertex_step_p95_m','target_step_p95_m'):
    values=np.asarray([r[key] for r in valid if r.get(key) is not None],float)
    if len(values):
        worst=max((r for r in valid if r.get(key) is not None),key=lambda r:r[key])
        summary[key]={'count':len(values),'median':float(np.median(values)),
                      'p95':float(np.percentile(values,95)),'p99':float(np.percentile(values,99)),
                      'max':float(values.max()),'worst_index':worst['index']}
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(summary,ensure_ascii=False))
