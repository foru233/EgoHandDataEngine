"""Score recorded raw poses before admission for research; never changes gates."""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / 'handoff/vio_recovery_gt_20261113/benchmark_application_recovery.py'
spec = importlib.util.spec_from_file_location('recovery_gt_runner', RUNNER)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def analyze(directory):
    plan = json.loads((directory / 'plan.json').read_text(encoding='utf8'))
    run = json.loads((directory / 'run_summary.json').read_text(encoding='utf8'))
    scored = json.loads((directory / 'recovery_gt_metrics.json').read_text(encoding='utf8'))
    frames = directory / 'frames.jsonl'
    if (runner.sha(frames) != run['frames_sha256'] or
            runner.sha(directory / 'plan.json') != run['plan_sha256']):
        raise ValueError('Run provenance does not match the recorded frames or plan')
    if not scored['pre_mask_admitted'] or not scored['threshold_comparison_assessable']:
        raise ValueError('Ground truth score lacks admitted same-reference recovery')
    if runner.sha(runner.public.paths(plan['sequence'])[3]) != scored['gt_sha256']:
        raise ValueError('Ground truth has changed')
    rows = [json.loads(line) for line in frames.open(encoding='utf8')]
    times, poses = runner.public.load_gt(plan['sequence'])

    def truth(t):
        j = int(np.searchsorted(times, t))
        if j < len(times) and times[j] == t:
            return poses[j]
        if (j == 0 or j == len(times) or times[j] - times[j-1] > 20_000_000
                or min(t-times[j-1], times[j]-t) > 5_000_000):
            return None
        alpha = (t-times[j-1]) / (times[j]-times[j-1])
        result = np.eye(4)
        result[:3, 3] = (1-alpha)*poses[j-1, :3, 3] + alpha*poses[j, :3, 3]
        result[:3, :3] = Slerp([0, 1], Rotation.from_matrix(poses[j-1:j+1, :3, :3]))([alpha]).as_matrix()[0]
        return result

    before = rows[plan['mask'][0] - 1]
    ref = runner.reference(before['result'])
    before_body = np.asarray(before['raw']['body_state_candidate']['T_world_from_body'])
    before_gt = truth(before['result']['timestamp_ns'])
    if before_gt is None:
        raise ValueError('Pre-mask GT unavailable')
    admitted_index = scored['application_readmission']['index']
    observed = []
    for row in rows[plan['mask'][1]:admitted_index+1]:
        raw = row['raw']
        body = raw.get('body_state_candidate') or {}
        if (raw.get('tracking_state') != 'OK' or raw.get('pose_valid') is not True
                or runner.reference(raw) != ref or body.get('snapshot_consistent') is not True
                or body.get('timestamp_ns') != raw.get('timestamp_ns')
                or body.get('frame_id') != raw.get('frame_id')):
            continue
        gt = truth(raw['timestamp_ns'])
        if gt is None:
            continue
        predicted = np.linalg.inv(before_body) @ np.asarray(body['T_world_from_body'])
        expected = np.linalg.inv(before_gt) @ gt
        error = np.linalg.inv(expected) @ predicted
        observed.append({'index': row['index'], 'admitted': row['result']['pose_valid'],
                         'quality_reason': (row['result'].get('quality_gate') or {}).get('reason_code'),
                         'quarantined': row['result']['recovery_preaccept']['await_visual_requalification'],
                         'translation_error_m': float(np.linalg.norm(error[:3, 3])),
                         'rotation_error_deg': float(Rotation.from_matrix(error[:3, :3]).magnitude()*180/np.pi)})
    last = next((item for item in observed if item['index'] == admitted_index), None)
    official = scored['first_readmission_gap_error']
    if (last is None or abs(last['translation_error_m']-official['translation_m']) > 1e-4
            or abs(last['rotation_error_deg']-official['rotation_deg']) > .01):
        raise ValueError('Body-snapshot result does not agree with independent admitted-pose scorer')
    return {'frames_sha256': run['frames_sha256'], 'gt_sha256': scored['gt_sha256'],
            'method': 'pre-mask GT-relative body motion; raw snapshot, no post-loss fit',
            'limitation': 'diagnostic counterfactual, not an authorization or statistical safety validation',
            'samples': observed}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = analyze(args.directory)
    with args.output.open('x', encoding='utf8') as output:
        json.dump(result, output, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
