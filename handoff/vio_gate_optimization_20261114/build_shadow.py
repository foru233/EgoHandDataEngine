import json
from pathlib import Path
root=Path('handoff/vio_gate_optimization_20261114')
reports={k:json.loads((root/f'baseline_{k}_report.json').read_text(encoding='utf8')) for k in ('111807','111833','184714')}
out={'schema_version':1,'kind':'host_only_shadow_gate','closed_loop':False,'candidates':{},'interpretation':'These are recorded-row counterfactuals for the host stability window only. They do not rerun native VIO, RecoveryCoordinator, map creation, or hand output; no candidate is safe to merge based on this file.'}
for name,report in reports.items():
 out['candidates'][name]=report['shadow_stability']
(root/'SHADOW_GATE_RESULTS.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8')
