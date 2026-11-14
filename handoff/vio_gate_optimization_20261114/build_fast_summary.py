import json
from pathlib import Path
root=Path('handoff/vio_gate_optimization_20261114')
def summary(path):
 rows=[json.loads(x) for x in Path(path).open(encoding='utf8')]
 admitted=sum(r.get('pose_valid') is True for r in rows)
 first=next((i for i in range(915,len(rows)) if rows[i].get('pose_valid') is True),None)
 ident=lambda r: tuple(r.get(k) for k in ('map_id','reset_id','map_revision','reference_generation','world_frame_revision'))
 return {'frames':len(rows),'admitted_frames':admitted,'first_admission_after_dropout_index':first,
   'readmission_delay_s':None if first is None else (rows[first]['timestamp_ns']-rows[915]['timestamp_ns'])/1e9,
   'same_reference':None if first is None else ident(rows[914])==ident(rows[first]),
   'pose_jump_frames':sum(r.get('jump_detected') is True for r in rows),
   'quarantined_frames':sum((r.get('recovery_preaccept') or {}).get('await_visual_requalification') is True for r in rows),
   'input_integrity':json.loads((Path(path).parent.parent/'validation.json').read_text(encoding='utf8')).get('input_integrity_passed')}
result={'schema_version':1,'same_recording':'data/session_20261112_184714_636910800','same_candidate_dll':'egohand_vio_short_recovery_candidate.dll','baseline_candidate_stable_seconds':1.5,'optimized_host_and_recovery_stable_seconds':0.5,
 'baseline_candidate':summary('handoff/vio_recovery_bridge_v3_20261113/runs/184714_stereo_inertial_candidate_mask_15/replay/poses.jsonl'),
 'optimized_candidate':summary('handoff/vio_gate_optimization_20261114/closed_loop_host_stable_0_5/replay/poses.jsonl'),
 'interpretation':['The optimized run changed both application quality and RecoveryCoordinator candidate stability configs; changing only recovery config did not change the host gate and was discarded as an invalid experiment.', 'The VS030 recording has no external GT. Same-reference identity and internal geometry/recovery contracts passed, but absolute accuracy and false-recovery-rate nonincrease remain unverified.', 'The 0.5 s profile is explicit opt-in offline candidate only; production stable profile remains 1.5 s.']}
(root/'CLOSED_LOOP_FAST_CANDIDATE.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(result,ensure_ascii=False,indent=2))
