"""Freeze an isolated host fix and validate development windows and holdout."""
import argparse
import json
import os
from pathlib import Path
import sys

from run_public_recovery_acceptance import ROOT, execute, save, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--coordinator-directory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    library = args.library.resolve(strict=True)
    directory = args.coordinator_directory.resolve(strict=True)
    runner = ROOT / 'scripts/run_tumvi_geometry_recovery.py'
    source, hook = directory / 'recovery_coordinator.py', directory / 'sitecustomize.py'
    hashes = {str(p): sha(p) for p in [library, runner, source, hook,
        *sorted((ROOT / 'src/egohand').rglob('*.py'))]}
    cases = [dict(sequence=s, frames=850, mask_frames=m, stable_seconds=w, kind='candidate')
        for s in ('room1', 'room2') for m, w in ((4, .5), (10, .5), (20, .5),
            (40, .5), (20, 1.), (20, 1.5))]
    # Frozen before these results: no tuning from this whole-sequence holdout.
    cases += [dict(sequence='room3_holdout', frames=2821, mask_frames=40,
        stable_seconds=.5, kind=k) for k in ('control', 'candidate')]
    save(args.output / 'freeze.json', dict(cases=cases, hashes=hashes,
        policy='Unchanged geometry, gap, prediction duration, jump and identity thresholds.',
        comparison='Development controls remain sealed in public_v4_development_suite; verify complete run plans before comparing. Holdout uses a new same-input control.',
        selection='Existing development event550 and fixed whole room3 holdout. No parameter search.',
        production_deployed=False))
    environment = os.environ.copy()
    for key in list(environment):
        if key.startswith(('EGOHAND_COORDINATOR_', 'EGOHAND_VIO_', 'EGOHAND_TEST_')):
            environment.pop(key)
    environment.update(TUMVI_RECOVERY_LIBRARY=str(library), PYTHONIOENCODING='utf-8',
        PYTHONPATH=str(ROOT / 'src') + os.pathsep + str(directory),
        EGOHAND_COORDINATOR_CANDIDATE_SOURCE=str(source))
    outcomes = []
    for case in cases:
        assert all(sha(Path(p)) == value for p, value in hashes.items())
        name = f"{case['sequence']}_{case['kind']}_mask{case['mask_frames']}_stable{case['stable_seconds']}"
        output = args.output / name
        audit = args.output / (name + '_source.jsonl')
        environment['EGOHAND_COORDINATOR_CANDIDATE_AUDIT'] = str(audit)
        command = [sys.executable, str(runner), 'run', str(output), '--sequence', case['sequence'],
            '--frames', str(case['frames']), '--mask-start', '550', '--mask-frames', str(case['mask_frames']),
            '--stable-seconds', str(case['stable_seconds']), '--kind', case['kind']]
        print(json.dumps(dict(starting=name)), flush=True)
        result = dict(case=case, command=command,
            **execute(command, args.output / (name + '.log'), environment))
        if result['exit_code'] == 0:
            loaded = [json.loads(line) for line in audit.open(encoding='utf-8')]
            assert all(r['sha256'] == hashes[str(source)] for r in loaded)
            assert any(r['argv'][0] == '-m' and len(r['argv']) == 2 and r['argv'][1].isdigit()
                and Path(r['executable']).parent.name == 'egohand-vio' for r in loaded)
            result['worker_candidate_source_verified'] = True
            actions = [('score', runner, []),
                ('trajectory', ROOT / 'scripts/evaluate_public_recovery_trajectory.py', ['--output', str(output / 'trajectory_metrics.json')]),
                ('gate_GT', ROOT / 'scripts/audit_public_gate_gt.py', ['--output', str(output / 'gate_gt.json')]),
                ('prediction_GT', ROOT / 'scripts/audit_recovery_prediction_gt.py', ['--output', str(output / 'prediction_gt_metrics.json')])]
            for action, script, extra in actions:
                command = [sys.executable, str(script), *(['score'] if action == 'score' else []), str(output), *extra]
                result[action] = execute(command, args.output / (name + '_' + action + '.log'), environment)
        outcomes.append(result)
        save(args.output / (name + '_outcome.json'), result)
        print(json.dumps(dict(finished=name, exit_code=result['exit_code'])), flush=True)
    assert all(sha(Path(p)) == value for p, value in hashes.items())
    save(args.output / 'suite_outcomes.json', dict(outcomes=outcomes, production_deployed=False))


if __name__ == '__main__':
    main()
