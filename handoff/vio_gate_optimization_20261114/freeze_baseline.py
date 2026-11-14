import hashlib, json, os, platform, subprocess, sys
from pathlib import Path
root=Path('.')
out=root/'handoff/vio_gate_optimization_20261114'
def sha(path):
 h=hashlib.sha256();
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
 return h.hexdigest()
def git(*args): return subprocess.check_output(['git',*args],text=True).strip()
files={
 'active_bridges':'src/egohand/vio/native/bin/active_bridges.json',
 'live_quality':'configs/vio/live_quality.yaml',
 'recovery_experimental':'configs/vio/recovery_experimental.yaml',
 'stereo':'configs/stereo.yaml',
 'vio_config':'configs/vio/vs030_rectified_preliminary.yaml',
 'dll':'src/egohand/vio/native/bin/egohand_vio_cheirality_v3.dll',
 'calibration_111807':'data/hand/session_20261110_111807_489019700/calibration.yaml',
 'calibration_111833':'data/hand/session_20261110_111833_589265500/calibration.yaml',
 'calibration_184714':'data/session_20261112_184714_636910800/calibration.yaml',
 'manifest_111807':'data/hand/session_20261110_111807_489019700/manifest.json',
 'manifest_111833':'data/hand/session_20261110_111833_589265500/manifest.json',
 'manifest_184714':'data/session_20261112_184714_636910800/manifest.json',
}
hashes={k:(sha(v) if Path(v).is_file() else None) for k,v in files.items()}
rows={
 '111807':json.loads((out/'baseline_111807_report.json').read_text(encoding='utf8')),
 '111833':json.loads((out/'baseline_111833_report.json').read_text(encoding='utf8')),
 '184714':json.loads((out/'baseline_184714_report.json').read_text(encoding='utf8')),
}
report={'schema_version':1,'kind':'fixed_offline_gate_baseline','git_head':git('rev-parse','HEAD'),'git_status':git('status','--short'),'python':sys.executable,'python_version':sys.version,'platform':platform.platform(),'opencv_threads':None,'cpu':platform.processor(),'files_sha256':hashes,'recordings':rows,'replay_outputs':{
 '111807':'results/hand_111807_v4_profile_20261114/replay/poses.jsonl',
 '111833':'results/hand_111833_v4_long1320_20261114/replay/poses.jsonl',
 '184714':'results/vio_repair_184714_20261114/replay/poses.jsonl'},
 'notes':['Existing completed offline replays were used; source recordings were not modified.', 'The 184714 report uses the 3283-frame vio_repair replay because gui_184714_entry_fix has no poses.jsonl at its run/results path.', 'Raw replay files contain frame-bound recovery reports; this baseline does not claim absolute accuracy.']}
try:
 import cv2; report['opencv_threads']=cv2.getNumThreads()
except Exception: pass
(out/'BASELINE_METRICS.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({'git_head':report['git_head'],'git_status':report['git_status'],'hashes':hashes,'opencv_threads':report['opencv_threads']},ensure_ascii=False,indent=2))
