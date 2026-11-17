"""Frozen room3 whole-sequence tests; no tuning or deployment."""
import argparse
import json
import os
from pathlib import Path
import sys

from run_public_recovery_acceptance import ROOT, execute, save, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    library = args.library.resolve(strict=True)
    runner = ROOT / 'scripts/run_tumvi_geometry_recovery.py'
    environment = os.environ.copy()
    environment.update(TUMVI_RECOVERY_LIBRARY=str(library), PYTHONIOENCODING='utf-8', EGOHAND_NATIVE_FAULT_TRACE='1')
    hashes = {str(p): sha(p) for p in [library, runner, *sorted((ROOT / 'src/egohand').rglob('*.py'))]}
    cases = [{'kind': k, 'mask_frames': m, 'stable_seconds': .5} for m in (0, 20) for k in ('control', 'candidate')]
    save(args.output / 'freeze.json', {'cases': cases, 'sequence': 'room3_holdout', 'frames': 2821,
        'mask_start': 550, 'hashes': hashes, 'recovery_limits': {'translation_m': .15, 'rotation_deg': 5},
        'provenance': 'Previously inspected control availability, never used for parameter selection. Whole sequence kept together. This freezes v4 without changing parameters from development.',
        'limitations': 'One holdout sequence and one injected event; does not establish statistical noninferiority or official-source checksum equivalence.',
        'production_deployed': False})
    results = []
    plans = {}
    for case in cases:
        assert all(sha(Path(p)) == value for p, value in hashes.items())
        name = f"{case['kind']}_mask{case['mask_frames']}"
        output = args.output / name
        command = [sys.executable, str(runner), 'run', str(output), '--sequence', 'room3_holdout',
                   '--frames', '2821', '--mask-start', '550', '--mask-frames', str(case['mask_frames']),
                   '--stable-seconds', '.5', '--kind', case['kind']]
        print(json.dumps({'starting': name}), flush=True)
        result = {'case': case, 'command': command, **execute(command, args.output / (name + '.log'), environment)}
        if result['exit_code'] == 0:
            for action, script, extra in [('score', runner, []),
                    ('trajectory', ROOT / 'scripts/evaluate_public_recovery_trajectory.py', ['--output', str(output / 'trajectory_metrics.json')]),
                    ('prediction', ROOT / 'scripts/audit_recovery_prediction_gt.py', ['--output', str(output / 'prediction_gt_metrics.json')])]:
                score = [sys.executable, str(script), *(['score'] if action == 'score' else []), str(output), *extra]
                result[action] = execute(score, args.output / (name + '_' + action + '.log'), environment)
            plan = json.loads((output / 'plan.json').read_text(encoding='utf-8'))
            comparable = {k: plan[k] for k in ('input_hashes', 'runtime_source_hashes', 'library_sha256', 'policy_sha256', 'quality_options', 'mask', 'frames')}
            if case['mask_frames'] in plans:
                assert comparable == plans[case['mask_frames']]
                result['same_input_policy_AB_verified'] = True
            plans[case['mask_frames']] = comparable
        results.append(result)
        save(args.output / (name + '_outcome.json'), result)
        print(json.dumps({'finished': name, 'exit_code': result['exit_code']}), flush=True)
    save(args.output / 'suite_outcomes.json', {'outcomes': results, 'production_deployed': False})


if __name__ == '__main__':
    main()
