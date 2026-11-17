"""Reproduce captured native optimizations, then compare visual-only diagnostics on the same graph.

Does not read ground truth or change production admission. Frame-reference costs
use the exported previous state; the optimizer's hidden previous posterior is not
exported. Keyframe-reference costs use the actual fixed native reference.
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(value):
    return np.asarray(value['data'], dtype=float).reshape(value['rows'], value['cols'])


def center(state):
    pose = matrix(state['Tcw'])
    return -pose[:3, :3].T @ pose[:3, 3]


def visual_costs(value):
    observations = value['visual']
    costs = [row['chi2'] for row in observations]
    return dict(count=len(costs), sum_chi2=sum(costs), median_chi2=float(np.median(costs)) if costs else None,
                outliers=sum(row['outlier'] for row in observations),
                negative_depth=sum(not row['depth_positive'] for row in observations))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--jobs', type=int, choices=(1, 2), default=2)
    args = parser.parse_args()
    args.run = args.run.resolve(strict=True)
    args.replay = args.replay.resolve(strict=True)
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    plan = json.loads((args.run / 'plan.json').read_text(encoding='utf8'))
    graphs = args.run / 'exact_native_graph'
    inputs = sorted(graphs.glob('frame_*_before.json'), key=lambda p: int(p.name.split('_')[1]))
    if not inputs:
        raise ValueError('no captured graphs')
    env = os.environ.copy()
    for key in list(env):
        if key.startswith(('EGOHAND_VIO_', 'EGOHAND_TEST_')):
            env.pop(key)
    env.update({key: value for key, value in plan['environment'].items() if value in ('0', '1')})
    conda = Path(os.environ['USERPROFILE']) / 'miniconda3/envs/egohand-vio'
    env['PATH'] = str(conda) + os.pathsep + str(conda / 'Library/bin') + os.pathsep + env['PATH']
    frozen = {str(p): sha(p) for p in [args.replay, Path(__file__), *graphs.glob('*.json')]}
    with (args.output / 'freeze.json').open('x', encoding='utf8') as stream:
        json.dump(dict(hashes=frozen, solver_flags={k: v for k, v in env.items() if k.startswith('EGOHAND_VIO_')},
                       production_deployed=False, gt_loaded=False), stream, indent=2)

    def work(before_path):
        before = json.loads(before_path.read_text())
        index = int(before['frame_id'])
        after_paths = [p for p in graphs.glob(f'frame_{index}_*.json') if p != before_path]
        if len(after_paths) != 1:
            return dict(frame=index, errors=['missing_or_ambiguous_after'])
        after_path, = after_paths
        after = json.loads(after_path.read_text())
        row = dict(frame=index, phase=after['phase'], errors=[],
                   map_id=before['map_id'], map_change_index=before['map_change_index'],
                   reference_generation=before['reference_generation'],
                   reference_operation=before['reference_operation'],
                   native_optimizer_displacement_m=float(np.linalg.norm(center(after['current']) - center(before['current']))))
        if any(before[key] != after[key] for key in ('map_id', 'map_change_index', 'reference_generation', 'reference_operation')):
            row['errors'].append('reference_changed_inside_optimizer')
        if before['keyframe_delta']['available']:
            covariance = matrix(before['keyframe_delta']['C'])[:9, :9]
            eigenvalues = np.linalg.eigvalsh(covariance)
            row['keyframe_delta'] = dict(dt=before['keyframe_delta']['dt'],
                covariance_eigen_min=float(eigenvalues.min()), covariance_eigen_max=float(eigenvalues.max()),
                covariance_condition=float(np.linalg.cond(covariance)))
        for mode in ('native', 'visual'):
            output = args.output / f'frame_{index}_{mode}.json'
            command = [str(args.replay), str(before_path), str(after_path), str(output), mode]
            with (args.output / f'frame_{index}_{mode}.log').open('x', encoding='utf8') as log:
                result = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=60)
            row[mode] = dict(exit_code=result.returncode)
            if result.returncode:
                row['errors'].append(f'{mode}_exit_{result.returncode}')
            if output.exists():
                try:
                    report = json.loads(output.read_text())
                    row[mode].update({key: report[key] for key in ('inliers', 'pose_matrix_max_error',
                        'velocity_error', 'bias_max_error', 'outlier_mismatches')})
                    row[mode]['before_visual'] = visual_costs(report['before'])
                    row[mode]['after_visual'] = visual_costs(report['after'])
                    row[mode]['before_inertial_chi2'] = report['before'].get('inertial_chi2')
                    row[mode]['after_inertial_chi2'] = report['after'].get('inertial_chi2')
                    row[mode]['inertial_reference'] = report['after'].get('reference_state')
                    row[mode]['velocity_availability_matches'] = report.get('velocity_availability_matches')
                    row[mode]['marginal_posterior_reference_not_exported'] = report.get('marginal_posterior_reference_not_exported', 0)
                    row[mode]['camera_center'] = center(report['replayed']).tolist()
                except (ValueError, KeyError):
                    row['errors'].append(f'{mode}_incomplete_report')
        if 'camera_center' in row['native'] and 'camera_center' in row['visual']:
            row['native_vs_visual_center_distance_m'] = float(np.linalg.norm(
                np.asarray(row['native']['camera_center']) - row['visual']['camera_center']))
        return row

    rows = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for row in pool.map(work, inputs):
            rows.append(row)
            if len(rows) % 10 == 0 or row['errors']:
                print(json.dumps(dict(completed=len(rows), total=len(inputs), frame=row['frame'], errors=row['errors'])), flush=True)
    changed = [name for name, digest in frozen.items() if sha(Path(name)) != digest]
    failures = [dict(frame=row['frame'], errors=row['errors']) for row in rows if row['errors']]
    summary = dict(graphs=len(rows), phases=dict(Counter(row.get('phase') for row in rows)), failures=failures,
        integrity_failures=changed, results=rows, gt_loaded=False, production_deployed=False,
        acceptance=False,
        limits=['Same-graph diagnostic; visual-only result is not a VIO recovery candidate',
                'Frame-reference cost uses exported previous state, not hidden optimized previous posterior',
                'Diagnostic export can change asynchronous mapping schedule'])
    with (args.output / 'summary.json').open('x', encoding='utf8') as stream:
        json.dump(summary, stream, indent=2)
    print(json.dumps({key: summary[key] for key in ('graphs', 'phases', 'failures', 'integrity_failures')}), flush=True)
    return int(bool(failures or changed))


if __name__ == '__main__':
    raise SystemExit(main())
