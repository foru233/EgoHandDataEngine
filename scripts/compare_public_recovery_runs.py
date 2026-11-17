"""Compare public runs on identical timestamps inside intersected reference spans.

Every span retains each run's own map identity. Separate retrospective SE3 fits
are reported explicitly; this is neither map stitching nor recovery validation.
"""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from evaluate_public_recovery_trajectory import score_interval
from evaluate_vio_trajectory import digest
from egohand.vio.short_recovery import visual_reference

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('control', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    runs = [args.control, args.candidate]
    plans, rows = [], []
    for run in runs:
        plan = json.loads((run / 'plan.json').read_text(encoding='utf8'))
        summary = json.loads((run / 'run_summary.json').read_text(encoding='utf8'))
        assert summary['plan_sha256'] == digest(run / 'plan.json')
        assert summary['frames_sha256'] == digest(run / 'frames.jsonl')
        plans.append(plan)
        rows.append([json.loads(line) for line in (run / 'frames.jsonl').open(encoding='utf8')])
    for field in ('sequence', 'frames', 'mask', 'stable_seconds', 'input_hashes', 'policy_sha256',
                  'quality_options', 'runtime_source_hashes', 'library_sha256'):
        assert plans[0][field] == plans[1][field], field
    def environment(plan):
        return {k: v for k, v in plan['environment'].items()
                if k not in ('EGOHAND_VIO_RIGHT_INDEX_FIX', 'EGOHAND_VIO_RIGHT_WEIGHT_FIX', 'EGOHAND_VIO_LOST_KEYFRAME_GUARD', 'EGOHAND_VIO_VISUAL_VELOCITY_SEED', 'EGOHAND_VIO_GEOMETRIC_RETRY', 'EGOHAND_VIO_MERGE_OBSERVATION_FIX', 'EGOHAND_VIO_FRAME_SEARCH_SEED', 'EGOHAND_VIO_KF_COVARIANCE_PRIOR', 'EGOHAND_VIO_EXACT_GRAPH_DIR', 'EGOHAND_VIO_TRACKING_PROBE',
                             'EGOHAND_VIO_COMMIT_LOG', 'EGOHAND_VIO_DIAGNOSTICS')}
    assert environment(plans[0]) == environment(plans[1])
    varied = [k for k in ('EGOHAND_VIO_RIGHT_INDEX_FIX', 'EGOHAND_VIO_RIGHT_WEIGHT_FIX', 'EGOHAND_VIO_LOST_KEYFRAME_GUARD', 'EGOHAND_VIO_VISUAL_VELOCITY_SEED', 'EGOHAND_VIO_GEOMETRIC_RETRY', 'EGOHAND_VIO_MERGE_OBSERVATION_FIX', 'EGOHAND_VIO_FRAME_SEARCH_SEED', 'EGOHAND_VIO_KF_COVARIANCE_PRIOR')
              if plans[0]['environment'].get(k, '0') != plans[1]['environment'].get(k, '0')]
    assert len(varied) == 1, 'Require exactly one native algorithm flag difference'
    assert [r['raw']['timestamp_ns'] for r in rows[0]] == [r['raw']['timestamp_ns'] for r in rows[1]]
    loader = ROOT / 'handoff/goal_delivery_20261113/benchmark_public_vio.py'
    spec = importlib.util.spec_from_file_location('paired_public_gt', loader)
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    _, settings, _, gt = public.paths(plans[0]['sequence'])
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(settings.read_text(encoding='utf8').splitlines()[1:]), Loader=Loader)
    camera_from_body = np.linalg.inv(np.asarray(config['IMU.T_b_c1']['data']).reshape(4, 4))
    times, poses = public.load_gt(plans[0]['sequence'])
    truth = (times, poses[:, :3, 3], Rotation.from_matrix(poses[:, :3, :3]))
    intervals = []
    for pair in zip(*rows):
        key = tuple(visual_reference(r['raw']) for r in pair)
        if not intervals or intervals[-1][0] != key:
            intervals.append((key, []))
        intervals[-1][1].append(pair)
    scored = []
    for key, pairs in intervals:
        populations = {}
        for population in ('native', 'application'):
            groups = [[], []]
            field = 'raw' if population == 'native' else 'result'
            for pair in pairs:
                common = all(r[field].get('pose_valid') is True and r[field].get('tracking_state') == 'OK' for r in pair)
                for arm, row in enumerate(pair):
                    value = dict(row[field])
                    value['pose_valid'] = common
                    groups[arm].append({**row, field: value})
            populations[population] = {arm: score_interval(groups[i], truth, camera_from_body, population)
                                       for i, arm in enumerate(('control', 'candidate'))}
        scored.append({'first_index': pairs[0][0]['index'], 'last_index': pairs[-1][0]['index'],
                       'references': key, **populations})
    report = {'sequence': plans[0]['sequence'], 'same_inputs_library_host_policy': True,
              'varied_native_flag': varied[0],
              'frames': len(rows[0]), 'common_reference_intervals': scored,
              'raw_native_ok': [sum(r['raw'].get('tracking_state') == 'OK' for r in group) for group in rows],
              'application_admitted': [sum(r['result'].get('pose_valid') is True for r in group) for group in rows],
              'jump_flags': [sum(r['result'].get('jump_detected') is True for r in group) for group in rows],
              'hashes': {str(p): digest(p) for run in runs for p in (run / 'plan.json', run / 'frames.jsonl')},
              'evaluator_sha256': digest(Path(__file__)), 'gt_sha256': digest(gt),
              'limits': ['Same native/application timestamp subsets in both arms, split at either reference change.',
                         'Each interval gets its own retrospective SE3 fit; no scale/time fit and no stitched global ATE.',
                         'Short intervals and resets must be reviewed with coverage; no significance or noninferiority certification.',
                         'Use preloss anchored GT scorer for recovery correctness, not these retrospective fits.']}
    with args.output.open('x', encoding='utf8') as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps({k: report[k] for k in ('sequence', 'frames', 'jump_flags', 'application_admitted')}))


if __name__ == '__main__':
    main()
