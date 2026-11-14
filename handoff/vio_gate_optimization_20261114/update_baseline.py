import json
from pathlib import Path
p=Path('handoff/vio_gate_optimization_20261114/BASELINE_METRICS.json')
x=json.loads(p.read_text(encoding='utf8'))
x['git_status_at_baseline']='clean (verified before analysis changes)'
x['git_status_at_capture']=x.pop('git_status')
x['analysis_changes_after_baseline']=['src/egohand/vio/display_pose.py (freeze age fix)','tests/test_best_effort_display_pose.py','scripts/audit_gate_decisions.py']
p.write_text(json.dumps(x,ensure_ascii=False,indent=2),encoding='utf8')
