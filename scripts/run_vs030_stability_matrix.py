"""Run an isolated 0.5/1.0/1.5 s native+host dropout matrix on 184714.

The three profiles differ only in stability duration in both gate owners.
Inputs and formal configuration are never edited. No absolute GT is available.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'handoff/vio_gate_optimization_20261114/vs030_184714_stability_matrix_20261114'
RECORDING = ROOT / 'data/session_20261112_184714_636910800'
QUALITY = ROOT / 'configs/vio/live_quality.yaml'
RECOVERY = ROOT / 'handoff/vio_recovery_bridge_v3_20261113/recovery_candidate.yaml'
WRAPPER = ROOT / 'handoff/vio_recovery_bridge_v3_20261113/process_wrapper.py'
DLL = ROOT / 'handoff/vio_recovery_bridge_20261113/bin/egohand_vio_short_recovery_candidate.dll'
CALIBRATION = ROOT / '标定结果/2/data3-camchain-imucam-measured.yaml'
SETTINGS = ROOT / 'handoff/vio_architecture_execution_20261113/bootstrap_full_settings.yaml'
START, END, FRAMES = 900, 915, 1200


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def save(path, value):
    with path.open('x', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)


def identity(row):
    return tuple(row.get(k) for k in ('map_id', 'reset_id', 'map_revision', 'reference_generation', 'world_frame_revision'))


def summarize(rows):
    if len(rows) != FRAMES:
        raise ValueError('incomplete run')
    for i, row in enumerate(rows):
        marker = row.get('controlled_visual_dropout') or {}
        if marker.get('input_frame_index') != i or marker.get('active') is not (START <= i < END):
            raise ValueError('dropout marker does not match planned frame')
    before = next((r for r in reversed(rows[:START]) if r.get('pose_valid') is True), None)
    after = next((r for r in rows[END:] if r.get('pose_valid') is True), None)
    before_id = identity(before) if before is not None else None
    after_id = identity(after) if after is not None else None
    return {'frames': len(rows), 'admitted_frames': sum(r.get('pose_valid') is True for r in rows),
            'tracking_ok': sum(r.get('tracking_state') == 'OK' for r in rows),
            'quarantined_frames': sum((r.get('recovery_preaccept') or {}).get('await_visual_requalification') is True for r in rows),
            'pose_jump_frames': sum(r.get('jump_detected') is True for r in rows),
            'last_pre_mask_frame': None if before is None else before['replay']['frame_index'],
            'first_post_mask_frame': None if after is None else after['replay']['frame_index'],
            'readmission_delay_s': None if after is None else (after['timestamp_ns'] - rows[END]['timestamp_ns']) / 1e9,
            'same_reference_as_pre_mask': (None if before_id is None or after_id is None or
                                           any(x is None for x in (*before_id, *after_id)) else before_id == after_id),
            'interpretation': 'Reference field equality is not ground truth or proof of accurate recovery.'}


def main():
    DEST.mkdir(parents=True, exist_ok=False)
    inputs = [RECORDING / p for p in ('manifest.json', 'raw/sdk_index.jsonl', 'raw/sdk_records.bin', 'raw/imu_records.bin', 'vio_timing.yaml')]
    inputs += [QUALITY, RECOVERY, WRAPPER, DLL, CALIBRATION, SETTINGS, ROOT / 'scripts/process_recording.py', Path(__file__)]
    before = {str(p): sha(p) for p in inputs}
    base_quality = yaml.safe_load(QUALITY.read_text(encoding='utf-8'))
    base_recovery = yaml.safe_load(RECOVERY.read_text(encoding='utf-8'))
    profiles = []
    for seconds in (1.5, 1.0, 0.5):
        name = str(seconds).replace('.', '_')
        q = {**base_quality, 'stable_seconds': seconds}; r = {**base_recovery, 'stable_seconds': seconds}
        qp, rp = DEST / f'quality_{name}.yaml', DEST / f'recovery_{name}.yaml'
        with qp.open('x', encoding='utf-8') as f: yaml.safe_dump(q, f, allow_unicode=True, sort_keys=True)
        with rp.open('x', encoding='utf-8') as f: yaml.safe_dump(r, f, allow_unicode=True, sort_keys=True)
        profiles.append({'seconds': seconds, 'quality': str(qp), 'recovery': str(rp),
                         'quality_sha256': sha(qp), 'recovery_sha256': sha(rp)})
    plan = {'recording': str(RECORDING), 'mask': [START, END], 'max_frames': FRAMES,
            'profiles': profiles, 'inputs_sha256': before, 'production_deployed': False,
            'scope': 'single VS030 recording and one synthetic dropout; pose-only native+host; no GT or hand/GUI evaluation',
            'caution': 'Independent runs cannot guarantee identical native map state; identify reference changes separately.'}
    save(DEST / 'plan.json', plan)
    outcomes = []
    for profile in profiles:
        name = str(profile['seconds']).replace('.', '_')
        output = DEST / ('run_' + name); mask_audit = DEST / ('mask_' + name + '.jsonl')
        env = os.environ.copy()
        for key in list(env):
            if key.startswith(('EGOHAND_TEST_', 'EGOHAND_VIO_', 'TASK_RECOVERY_')):
                env.pop(key)
        env.update(EGOHAND_VIO_SHORT_RECOVERY='1', EGOHAND_VIO_RECOVERY_CONFIG=profile['recovery'],
                   TASK_RECOVERY_MASK_FRAMES='15', TASK_RECOVERY_MAX_FRAMES=str(FRAMES), TASK_RECOVERY_MASK_AUDIT=str(mask_audit))
        command = [sys.executable, str(WRAPPER), str(RECORDING), str(output), '--mode', 'stereo_inertial',
                   '--poses-only', '--image-scale', '1', '--session-auto-phase', '--calibration', str(CALIBRATION),
                   '--settings', str(SETTINGS), '--timing', str(RECORDING / 'vio_timing.yaml'),
                   '--quality', profile['quality'], '--library', str(DLL)]
        print('RUN', profile['seconds'], flush=True)
        with (DEST / ('run_' + name + '.log')).open('x', encoding='utf-8') as log:
            code = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
        if code:
            raise RuntimeError(f'run {name} failed: exit {code}; inspect its partial output')
        if any(sha(Path(p)) != h for p, h in before.items()):
            raise RuntimeError('an input, source or DLL changed during the run')
        validation = json.loads((output / 'validation.json').read_text(encoding='utf-8'))
        replay = json.loads((output / 'replay/replay.json').read_text(encoding='utf-8'))
        if not validation['input_integrity_passed'] or not validation['source_integrity_passed']:
            raise RuntimeError('run integrity failed')
        rows = [json.loads(line) for line in (output / 'replay/poses.jsonl').read_text(encoding='utf-8').splitlines()]
        info = summarize(rows)
        info.update(profile_seconds=profile['seconds'], mask_audit_sha256=sha(mask_audit),
                    phase=replay['phase_verification'], poses_sha256=sha(output / 'replay/poses.jsonl'),
                    actual_fingerprint=replay['config_snapshot']['fingerprint'])
        outcomes.append(info)
        save(DEST / ('run_' + name + '_summary.json'), info)
        print(json.dumps(info, ensure_ascii=False), flush=True)
    if len({x['mask_audit_sha256'] for x in outcomes}) != 1 or len({x['phase']['effective_phase_ms'] for x in outcomes}) != 1:
        raise RuntimeError('masked inputs or verified phase differ: results are not same-input comparable')
    save(DEST / 'matrix_summary.json', {'outcomes': outcomes, 'input_mask_and_phase_equal': True,
         'interpretation': plan['scope'], 'no_accuracy_or_false_recovery_claim': True})


if __name__ == '__main__':
    main()
