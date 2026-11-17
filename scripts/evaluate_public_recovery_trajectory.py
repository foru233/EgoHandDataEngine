"""Score saved public recovery runs by contiguous, exact native reference.

This separate process reads GT. It never feeds GT to estimation, joins maps,
fits scale/time, or treats retrospective ATE alignment as recovery validation.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.vio.short_recovery import visual_reference
from evaluate_vio_trajectory import evaluate, digest


def reference_intervals(rows):
    """Keep returns to a previous identity separate from its earlier interval."""
    groups = []
    for row in rows:
        key = visual_reference(row['raw'])
        if not groups or groups[-1][0] != key:
            groups.append((key, []))
        groups[-1][1].append(row)
    return groups


def score_interval(rows, truth, camera_from_body, population):
    timeline = np.asarray([r['raw']['timestamp_ns'] for r in rows], dtype=np.int64)
    estimates = []
    for row in rows:
        value = row['raw'] if population == 'native' else row['result']
        if value.get('pose_valid') is True and value.get('tracking_state') == 'OK':
            estimates.append((row['raw']['timestamp_ns'], np.asarray(value['matrix']) @ camera_from_body))
    coverage = {'expected_frames': len(rows), 'estimate_available': len(estimates)}
    if len(estimates) < 3 or len(rows) < 3:
        return {'coverage': coverage, 'assessment': 'insufficient_estimates'}
    timestamps = np.asarray([t for t, _ in estimates], dtype=np.int64)
    poses = np.asarray([p for _, p in estimates])
    try:
        result = evaluate(timeline, (timestamps, poses[:, :3, 3], Rotation.from_matrix(poses[:, :3, :3])),
                          truth, 0, 20_000_000, [0.1, 0.5, 1.0], 100_000_000)
    except ValueError as error:
        return {'coverage': coverage, 'assessment': 'unassessable', 'reason': str(error)}
    return {'assessment': 'scored', **result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    plan_path = args.run / 'plan.json'
    frames_path = args.run / 'frames.jsonl'
    summary_path = args.run / 'run_summary.json'
    plan, summary = (json.loads(p.read_text(encoding='utf-8')) for p in (plan_path, summary_path))
    assert summary['frames_sha256'] == digest(frames_path)
    assert summary['plan_sha256'] == digest(plan_path)
    rows = [json.loads(line) for line in frames_path.open(encoding='utf-8')]
    assert len(rows) == plan['frames']
    assert all(b['raw']['timestamp_ns'] > a['raw']['timestamp_ns'] for a, b in zip(rows, rows[1:]))
    loader = ROOT / 'handoff/goal_delivery_20261113/benchmark_public_vio.py'
    spec = importlib.util.spec_from_file_location('public_recovery_gt_loader', loader)
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    _, settings_path, _, gt_path = public.paths(plan['sequence'])

    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix',
                           lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(settings_path.read_text(encoding='utf-8').splitlines()[1:]), Loader=Loader)
    camera_from_body = np.linalg.inv(np.asarray(config['IMU.T_b_c1']['data']).reshape(4, 4))
    times, poses = public.load_gt(plan['sequence'])
    truth = (times, poses[:, :3, 3], Rotation.from_matrix(poses[:, :3, :3]))
    intervals = []
    for key, group in reference_intervals(rows):
        intervals.append({'reference': key, 'first_index': group[0]['index'], 'last_index': group[-1]['index'],
                          'masked_frames': sum(r['masked'] for r in group),
                          **{population: score_interval(group, truth, camera_from_body, population)
                             for population in ('native', 'application')}})
    result = {'schema_version': 1, 'sequence': plan['sequence'], 'frames': len(rows),
              'native_ok': sum(r['raw'].get('tracking_state') == 'OK' for r in rows),
              'application_admitted': sum(r['result'].get('pose_valid') is True for r in rows),
              'reference_intervals': intervals,
              'input_sha256': {'plan': digest(plan_path), 'frames': digest(frames_path),
                              'gt': digest(gt_path), 'settings': digest(settings_path),
                              'public_loader': digest(loader), 'evaluator': digest(Path(__file__)),
                              'trajectory_evaluator': digest(ROOT / 'scripts/evaluate_vio_trajectory.py')},
              'frame_contract': 'raw camera1 pose @ inverse(IMU.T_b_c1) equals world_from_body; GT body frame',
              'limits': ['Each contiguous exact-reference interval aligned separately by SE(3); scale=1, time offset=0.',
                         'No transform between intervals is established; no stitched global ATE.',
                         'All-camera coverage includes loss/rejection; RPE never bridges missing estimates or GT gaps.',
                         'Retrospective ATE is not evidence of correct recovery; use pre-loss anchored event scorer.',
                         'Shared frames within a sequence are correlated; no statistical noninferiority claim.']}
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({'sequence': result['sequence'], 'frames': len(rows), 'intervals': len(intervals),
                      'native_ok': result['native_ok'], 'admitted': result['application_admitted']}))


if __name__ == '__main__':
    main()
