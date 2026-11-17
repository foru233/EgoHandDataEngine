"""Serial native recovery A/B on fixed development events, retaining failures."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import psutil

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    with path.open('x', encoding='utf-8') as output:
        json.dump(value, output, ensure_ascii=False, indent=2, allow_nan=False)


def execute(command, log_path, environment):
    started = time.perf_counter()
    cpu_seconds, rss_peak, seen = 0.0, 0, {}
    with log_path.open('x', encoding='utf-8') as log:
        child = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT)
        while child.poll() is None:
            try:
                process = psutil.Process(child.pid)
                processes = [process, *process.children(recursive=True)]
                rss = 0
                for process in processes:
                    try:
                        cpu = process.cpu_times()
                        seen[(process.pid, process.create_time())] = cpu.user + cpu.system
                        rss += process.memory_info().rss
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                rss_peak = max(rss_peak, rss)
                cpu_seconds = sum(seen.values())
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
            try:
                child.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
    return {'exit_code': child.returncode, 'wall_s': time.perf_counter() - started,
            'sampled_cpu_seconds': cpu_seconds, 'sampled_process_tree_peak_rss_bytes': rss_peak,
            'resource_scope': 'owned run process tree only, ~1s sampling, exit CPU tail may be missed; offline throughput'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    runner = ROOT / 'scripts/run_tumvi_geometry_recovery.py'
    environment = os.environ.copy()
    environment['TUMVI_RECOVERY_LIBRARY'] = str(args.library.resolve(strict=True))
    environment['PYTHONIOENCODING'] = 'utf-8'
    environment['EGOHAND_NATIVE_FAULT_TRACE'] = '1'
    cases = []
    for sequence in ('room1', 'room2'):
        for frames, stable in [(4, .5), (10, .5), (20, .5), (40, .5), (20, 1.0), (20, 1.5)]:
            for kind in ('control', 'candidate'):
                cases.append({'sequence': sequence, 'mask_start': 550, 'mask_frames': frames,
                              'stable_seconds': stable, 'kind': kind, 'frames': 850})
    library_hash, runner_hash = sha(args.library), sha(runner)
    save(args.output / 'suite_plan.json', {'cases': cases, 'library': str(args.library.resolve()),
        'library_sha256': library_hash, 'runner_sha256': runner_hash,
        'selection': 'Development room1/room2 event550 chosen from GT timestamp availability only before this suite; see gt_event_coverage_plan.json. No parameter tuning.',
        'thresholds': {'same_reference_gap_translation_m': .15, 'rotation_deg': 5.0},
        'limits': '24 correlated runs, one event location per sequence; not 24 independent events or statistical noninferiority. Identical inputs do not force deterministic native mapping.',
        'production_deployed': False})
    outcomes = []
    plans = {}
    for index, case in enumerate(cases):
        assert sha(args.library) == library_hash and sha(runner) == runner_hash
        name = f"{case['sequence']}_{case['kind']}_mask{case['mask_frames']}_stable{case['stable_seconds']}"
        output = args.output / name
        command = [sys.executable, str(runner), 'run', str(output), '--sequence', case['sequence'],
            '--kind', case['kind'], '--frames', str(case['frames']), '--mask-start', str(case['mask_start']),
            '--mask-frames', str(case['mask_frames']), '--stable-seconds', str(case['stable_seconds'])]
        print(json.dumps({'starting': index + 1, 'total': len(cases), 'case': case}), flush=True)
        result = {'case': case, 'command': command, **execute(command, args.output / (name + '.log'), environment)}
        if result['exit_code'] == 0:
            for action, script, extra in [('score', runner, []),
                ('trajectory', ROOT / 'scripts/evaluate_public_recovery_trajectory.py', ['--output', str(output / 'trajectory_metrics.json')])]:
                scoring = [sys.executable, str(script), *(['score'] if action == 'score' else []), str(output), *extra]
                result[action] = execute(scoring, args.output / (name + '_' + action + '.log'), environment)
            plan = json.loads((output / 'plan.json').read_text(encoding='utf-8'))
            key = (case['sequence'], case['mask_frames'], case['stable_seconds'])
            comparable = {k: plan[k] for k in ('input_hashes', 'runtime_source_hashes', 'library_sha256',
                                                'policy_sha256', 'quality_options', 'mask', 'frames')}
            if key in plans:
                assert comparable == plans[key], 'A/B input, source, calibration, library or policy mismatch'
                result['same_input_policy_sources_AB_verified'] = True
            plans[key] = comparable
            if (output / 'recovery_gt_metrics.json').exists():
                metrics = json.loads((output / 'recovery_gt_metrics.json').read_text(encoding='utf-8'))
                result['recovery'] = {k: metrics[k] for k in ('pre_mask_admitted', 'application_readmission',
                    'same_reference_as_pre_mask', 'first_readmission_gap_error', 'threshold_comparison_assessable',
                    'threshold_violations', 'new_map_requests', 'native_prediction_attempts', 'native_prediction_successes')}
        outcomes.append(result)
        save(args.output / f'case_{index:02d}_outcome.json', result)
        print(json.dumps({'finished': index + 1, 'exit_code': result['exit_code'], 'recovery': result.get('recovery')}), flush=True)
    save(args.output / 'suite_outcomes.json', {'outcomes': outcomes, 'all_cases_completed': len(outcomes) == len(cases),
                                             'no_deployment': True})


if __name__ == '__main__':
    main()
