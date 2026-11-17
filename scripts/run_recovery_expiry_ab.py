"""Same-native, single-host-change A/B via explicit isolated coordinator source."""
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
    parser.add_argument('--resume', action='store_true', help='Read sealed completed runs; never overwrite an experiment')
    parser.add_argument('--sequences', nargs='+', choices=['room1','room2','room3_holdout'], default=['room1','room2'])
    parser.add_argument('--frames', type=int, default=850)
    parser.add_argument('--mask-frames', nargs='+', type=int, choices=[4,10,20,40], default=[40,10])
    parser.add_argument('--experiment', choices=['expiry', 'loss_deadline'], default='expiry')
    parser.add_argument('--native-fixes', action='store_true',
                        help='Use corrected right observations and lost-keyframe guard in both arms')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=args.resume)
    library, directory = args.library.resolve(strict=True), args.coordinator_directory.resolve(strict=True)
    runner = ROOT / 'scripts/run_tumvi_geometry_recovery.py'
    source, hook = directory / 'recovery_coordinator.py', directory / 'sitecustomize.py'
    hashes = {str(p): sha(p) for p in (library, runner, source, hook)}
    cases = [{'sequence': s, 'mask_frames': m, 'host': h} for s in args.sequences
             for m in args.mask_frames for h in ('baseline', 'candidate')]
    frozen_plan = {'cases': cases, 'native_short_recovery': True,
        'mask_start': 550, 'frames': args.frames, 'stable_seconds': .5, 'hashes': hashes,
        'single_change': ('Start the existing quarantine/new-map deadline at actual visual loss; no timeout auto-release or threshold changes.'
            if args.experiment=='loss_deadline' else
            'Host retires expired IMU anchor instead of poisoning invalid_reason; no threshold change, geometry/identity/jump/sensor guards retained.'),
        'experiment': args.experiment, 'native_fixes_both_arms': args.native_fixes,
        'injection': 'Isolated sitecustomize substitutes only recovery_coordinator.py in owned interpreters, with per-process source audit. Repository runtime Python files unchanged.',
        'production_deployed': False}
    if args.resume:
        assert json.loads((args.output / 'plan.json').read_text(encoding='utf-8')) == frozen_plan
    else:
        save(args.output / 'plan.json', frozen_plan)
    results, paired = [], {}
    for case in cases:
        assert all(sha(Path(p)) == value for p, value in hashes.items())
        name = f"{case['sequence']}_{case['host']}_mask{case['mask_frames']}"
        output, audit = args.output / name, args.output / (name + '_loaded_source.jsonl')
        environment = os.environ.copy()
        for key in list(environment):
            if key.startswith(('EGOHAND_COORDINATOR_', 'EGOHAND_VIO_', 'EGOHAND_TEST_')):
                environment.pop(key)
        environment.update(TUMVI_RECOVERY_LIBRARY=str(library), PYTHONIOENCODING='utf-8',
            EGOHAND_NATIVE_FAULT_TRACE='1',
            PYTHONPATH=str(ROOT / 'src') + os.pathsep + str(directory))
        if case['host'] == 'candidate':
            environment.update(EGOHAND_COORDINATOR_CANDIDATE_SOURCE=str(source),
                               EGOHAND_COORDINATOR_CANDIDATE_AUDIT=str(audit))
        command = [sys.executable, str(runner), 'run', str(output), '--sequence', case['sequence'],
            '--frames', str(args.frames), '--mask-start', '550', '--mask-frames', str(case['mask_frames']),
            '--stable-seconds', '.5', '--kind', 'candidate']
        if args.native_fixes:
            command += ['--right-index-fix', '--right-weight-fix', '--lost-keyframe-guard', '--native-diagnostics']
        print(json.dumps({'starting': name}), flush=True)
        outcome = args.output / (name + '_outcome.json')
        if outcome.exists():
            result = json.loads(outcome.read_text(encoding='utf-8'))
        elif args.resume and (output / 'run_summary.json').exists():
            result = {'case': case, 'command': command, 'exit_code': 0,
                      'resumed_completed_run': True, 'performance_unavailable': 'Run completed before driver audit assertion; original stdout preserved.'}
        else:
            result = {'case': case, 'command': command, **execute(command, args.output / (name + '.log'), environment)}
        if result['exit_code'] == 0:
            if case['host'] == 'candidate':
                loaded = [json.loads(line) for line in audit.open(encoding='utf-8')]
                # sitecustomize runs before Python resolves -m; sys.argv still
                # contains '-m' and the IPC port, not the eventual module name.
                assert any(r['argv'][0] == '-m' and len(r['argv']) == 2 and r['argv'][1].isdigit()
                           and Path(r['executable']).parent.name == 'egohand-vio' for r in loaded), 'Owned VIO interpreter did not load candidate'
                assert all(r['sha256'] == hashes[str(source)] for r in loaded)
                result['loaded_candidate_source_verified'] = loaded
            for action, script, extra in [('score', runner, []),
                    ('gate_GT', ROOT / 'scripts/audit_public_gate_gt.py', ['--output', str(output / 'gate_gt.json')]),
                    ('trajectory', ROOT / 'scripts/evaluate_public_recovery_trajectory.py', ['--output', str(output / 'trajectory_metrics.json')])]:
                scoring = [sys.executable, str(script), *(['score'] if action == 'score' else []), str(output), *extra]
                score_log = args.output / (name + '_' + action + '.log')
                if action not in result:
                    result[action] = execute(scoring, score_log, environment)
            plan = json.loads((output / 'plan.json').read_text(encoding='utf-8'))
            comparable = {k: plan[k] for k in ('input_hashes', 'runtime_source_hashes', 'library_sha256', 'policy_sha256', 'quality_options', 'mask', 'frames')}
            key = (case['sequence'], case['mask_frames'])
            if key in paired:
                assert comparable == paired[key]
                result['same_input_native_policy_root_sources_verified'] = True
            paired[key] = comparable
        results.append(result)
        if not outcome.exists():
            save(outcome, result)
        print(json.dumps({'finished': name, 'exit_code': result['exit_code']}), flush=True)
    save(args.output / 'suite_outcomes.json', {'outcomes': results, 'production_deployed': False})


if __name__ == '__main__':
    main()
