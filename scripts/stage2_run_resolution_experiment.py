"""Run paired frozen-source VS030 experiments after competing native builds have stopped."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/vio_stage2_codex/perf_init'
FROZEN = OUT / 'frozen'
SESSIONS = ('session_20261110_111807_489019700', 'session_20261110_111833_589265500')
CANDIDATE = ROOT / ('results/vio_native_audit_20261112_codex/isolated_diagnostic_candidate/'
                    'src/egohand/vio/native/bin/egohand_vio_init_trace_candidate.dll')
EXPECTED = 'df98e8d9c32b1c1675246202df6841203c6cbe67c54b03be30abd372468c0cdf'


def main():
    if not (OUT / 'geometry_validation.json').is_file():
        raise RuntimeError('Run geometry validation first')
    if hashlib.sha256(CANDIDATE.read_bytes()).hexdigest() != EXPECTED:
        raise RuntimeError('Candidate DLL does not match verified isolated build')
    baseline = json.loads((OUT / 'preparation.json').read_text(encoding='utf8'))
    for name, expected in baseline['_frozen']['source_sha256'].items():
        p = FROZEN / name
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Frozen source modified after preparation: {name}')
    runs = [(SESSIONS[0], 'full', False), (SESSIONS[0], 'half', False),
            (SESSIONS[1], 'half', False), (SESSIONS[1], 'full', False),
            (SESSIONS[0], 'full', True), (SESSIONS[1], 'full', True)]
    plan = []
    for session, variant, fast in runs:
        rec = OUT / 'inputs' / session / variant
        name = session + '_' + variant + ('_fastInit' if fast else '_standard')
        output = OUT / 'runs' / name
        if output.exists():
            raise FileExistsError(output)
        cmd = [sys.executable, str(FROZEN / 'scripts/run_vio_stability_experiment.py'),
               '--recording', str(rec), '--library', str(CANDIDATE), '--output', str(output),
               '--gauge-constraint', '--cheirality-guard']
        if fast:
            settings = OUT / 'inputs' / session / 'vio_settings_fastInit.yaml'
            if settings.exists():
                raise FileExistsError(settings)
            original = (rec / 'vio_settings.yaml').read_text(encoding='utf8')
            if 'IMU.fastInit:' in original:
                raise ValueError('fastInit already configured in baseline')
            settings.write_text(original.rstrip() + '\nIMU.fastInit: 1\n', encoding='utf8')
            cmd.extend(['--settings', str(settings)])
        plan.append({'name':name, 'command':cmd, 'input':str(rec),
                     'output':str(output), 'variant':variant, 'fast_init':fast})
    (OUT / 'run_plan.json').write_text(json.dumps(plan, indent=2, ensure_ascii=False), encoding='utf8')
    env = os.environ.copy()
    env.pop('EGOHAND_VIO_INIT_TRACE', None)
    env.pop('EGOHAND_VIO_DIAGNOSTICS', None)
    (OUT / 'runs').mkdir(exist_ok=False)
    for row in plan:
        log = OUT / 'runs' / (row['name'] + '.console.log')
        print('START', row['name'], flush=True)
        start = time.time()
        with log.open('x', encoding='utf8') as stream:
            result = subprocess.run(row['command'], cwd=ROOT, env=env,
                                    stdout=stream, stderr=subprocess.STDOUT)
        row.update(exit_code=result.returncode, wall_seconds=time.time()-start)
        (OUT / 'run_status.json').write_text(json.dumps(plan, indent=2, ensure_ascii=False), encoding='utf8')
        print('DONE', row['name'], 'exit=', result.returncode, flush=True)
        if result.returncode:
            raise RuntimeError(f'experiment failed; inspect {log}')


if __name__ == '__main__':
    main()
