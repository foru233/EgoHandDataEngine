"""Score saved GUI/host IMU-only predictions using pre-loss anchored public GT.

Native search seeds have no independently exported pose here and are not scored.
No estimate or configuration is modified. No covariance interpretation is made.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.vio.short_recovery import visual_reference
from evaluate_vio_trajectory import digest, summary


def relative_motion_error(anchor_estimate, predicted, anchor_truth, actual):
    estimated_delta = np.linalg.inv(anchor_estimate) @ predicted
    actual_delta = np.linalg.inv(anchor_truth) @ actual
    translation = float(np.linalg.norm(estimated_delta[:3, 3]-actual_delta[:3, 3]))
    angle = float(np.degrees(Rotation.from_matrix(actual_delta[:3, :3].T @ estimated_delta[:3, :3]).magnitude()))
    return translation, angle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads((args.run / 'plan.json').read_text(encoding='utf-8'))
    run_summary = json.loads((args.run / 'run_summary.json').read_text(encoding='utf-8'))
    frames = args.run / 'frames.jsonl'
    assert digest(frames) == run_summary['frames_sha256']
    rows = [json.loads(line) for line in frames.open(encoding='utf-8')]
    spec = importlib.util.spec_from_file_location('public_prediction_gt', ROOT / 'handoff/goal_delivery_20261113/benchmark_public_vio.py')
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    _, settings, _, gt_path = public.paths(plan['sequence'])
    times, poses = public.load_gt(plan['sequence'])

    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(settings.read_text(encoding='utf-8').splitlines()[1:]), Loader=Loader)
    camera_from_body = np.linalg.inv(np.asarray(config['IMU.T_b_c1']['data']).reshape(4, 4))

    def truth(t):
        j = int(np.searchsorted(times, t))
        if j < len(times) and times[j] == t:
            return poses[j]
        if j == 0 or j == len(times) or times[j] - times[j-1] > 20_000_000 or min(t-times[j-1], times[j]-t) > 5_000_000:
            return None
        fraction = (t-times[j-1]) / (times[j]-times[j-1])
        result = np.eye(4)
        result[:3, 3] = (1-fraction)*poses[j-1, :3, 3] + fraction*poses[j, :3, 3]
        result[:3, :3] = Slerp([0, 1], Rotation.from_matrix(poses[j-1:j+1, :3, :3]))([fraction]).as_matrix()[0]
        return result

    anchor = None
    samples, unavailable = [], []
    event = 0
    previously_lost = False
    for row in rows:
        raw, result = row['raw'], row['result']
        lost = raw.get('tracking_state') != 'OK'
        if lost and not previously_lost:
            event += 1
        prediction = (result.get('recovery_preaccept') or {}).get('loss_prediction') or {}
        if prediction.get('status') == 'predicted':
            stamp = raw['timestamp_ns']
            actual = truth(stamp)
            if anchor is None or actual is None or anchor['truth'] is None or visual_reference(raw) != anchor['reference']:
                unavailable.append({'index': row['index'], 'event': event, 'reason': 'preloss_anchor_GT_or_reference_unavailable'})
            else:
                translation, angle = relative_motion_error(anchor['body'], np.asarray(prediction['T_world_from_body']),
                                                          anchor['truth'], actual)
                samples.append({'index': row['index'], 'event': event, 'duration_s': prediction['duration_s'],
                                'translation_error_m': translation, 'rotation_error_deg': angle,
                                'published_as_visual_pose': result.get('pose_valid') is True})
        elif lost:
            unavailable.append({'index': row['index'], 'event': event, 'reason': prediction.get('reason', 'no_prediction')})
        if not lost and result.get('pose_valid') is True:
            anchor = {'body': np.asarray(result['matrix']) @ camera_from_body,
                      'truth': truth(raw['timestamp_ns']), 'reference': visual_reference(raw)}
        previously_lost = lost
    bins = []
    for lo, hi in [(0, .2), (.2, .5), (.5, 1), (1, 1.5), (1.5, 2.000001)]:
        selected = [s for s in samples if lo < s['duration_s'] <= hi]
        bins.append({'duration_bounds_s': [lo, hi], 'samples': len(selected),
                     'independent_events': len({s['event'] for s in selected}),
                     'translation_m': summary([s['translation_error_m'] for s in selected]),
                     'rotation_deg': summary([s['rotation_error_deg'] for s in selected])})
    output = {'schema_version': 1, 'samples': samples, 'unavailable': unavailable, 'duration_bins': bins,
              'prediction_ever_published_as_visual_pose': any(s['published_as_visual_pose'] for s in samples),
              'hashes': {'frames': digest(frames), 'gt': digest(gt_path), 'settings': digest(settings),
                         'scorer': digest(Path(__file__))},
              'limits': ['Host/GUI prediction only; native internal search pose is not exported independently.',
                         'Pre-loss admitted body pose anchors relative GT motion; no post-loss fit, scale or time fit.',
                         'Samples within an event are correlated. Empirical errors are not a covariance/confidence envelope.',
                         'Unavailable prediction or GT is retained; cannot claim coverage/accuracy for omitted states.']}
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2, allow_nan=False)
    print(json.dumps({'scored_predictions': len(samples), 'unavailable': len(unavailable),
                      'prediction_ever_published_as_visual_pose': output['prediction_ever_published_as_visual_pose']}))


if __name__ == '__main__':
    main()
