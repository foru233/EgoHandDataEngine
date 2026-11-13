"""Run named single-variable experiments serially and retain failed results."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--recording', required=True, type=Path)
    p.add_argument('--library', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--frames', type=int)
    p.add_argument('--suite', choices=['gauge', 'snapshot'], default='gauge')
    a = p.parse_args()
    cases = ([('gauge_B', ['--diagnostics', '--gauge-fix']),
              ('repeat_A', ['--diagnostics']), ('recovery_off', ['--disable-recovery']),
              ('phase_77', ['--phase-ms', '77']), ('phase_117', ['--phase-ms', '117'])]
             if a.suite == 'gauge' else
             [('snapshot_default', ['--diagnostics']), ('stereo_only', ['--stereo-only']),
              ('snapshot_repeat', []), ('snapshot_recovery_off', ['--disable-recovery'])])
    reports = []
    a.output.mkdir(parents=True, exist_ok=True)
    for name, extra in cases:
        out = a.output / name
        if (out/'experiment.json').is_file():
            reports.append({'name': name, 'retained_existing_result': True})
            continue
        cmd = [sys.executable, str(ROOT/'scripts/run_vio_stability_experiment.py'),
               '--recording', str(a.recording), '--library', str(a.library), '--output', str(out), *extra]
        if a.frames:
            cmd.extend(['--frames', str(a.frames)])
        with (a.output/f'{name}_console.log').open('w') as stream:
            result = subprocess.run(cmd, stdout=stream, stderr=subprocess.STDOUT, cwd=ROOT)
        reports.append({'name': name, 'command': cmd, 'exit_code': result.returncode})
        (a.output/'matrix.json').write_text(json.dumps(reports, indent=2), encoding='utf8')
        print(json.dumps(reports[-1]), flush=True)


if __name__ == '__main__':
    main()
