"""Label one recovery event with fixed pre-loss motion GT, never a post-loss fit."""
import argparse
from collections import Counter
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
from audit_recovery_prediction_gt import relative_motion_error
from evaluate_vio_trajectory import digest


def associate_truth(stamp, times, poses):
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and int(times[index]) == stamp:
        return poses[index]
    if (index == 0 or index == len(times) or int(times[index])-int(times[index-1]) > 20_000_000
            or min(stamp-int(times[index-1]), int(times[index])-stamp) > 5_000_000):
        return None
    fraction = (stamp-int(times[index-1])) / (int(times[index])-int(times[index-1]))
    result = np.eye(4)
    result[:3, 3] = (1-fraction)*poses[index-1, :3, 3] + fraction*poses[index, :3, 3]
    result[:3, :3] = Slerp([0, 1], Rotation.from_matrix(poses[index-1:index+1, :3, :3]))([fraction]).as_matrix()[0]
    return result


def labels(rows, mask, truth, camera_from_body, translation_limit=.15, rotation_limit=5.):
    start, end = mask
    before = rows[start-1]
    anchor = before['result']
    anchor_truth = truth(before['raw']['timestamp_ns'])
    anchor_ready = anchor.get('pose_valid') is True and anchor_truth is not None
    identity = visual_reference(anchor)
    samples = []
    # Stop fixed-horizon evaluation before subsequent reference transitions;
    # a new map is a distinct coordinate system, not a successful old-map fix.
    deadline = rows[end]['raw']['timestamp_ns'] + 2_000_000_000
    for row in rows[start:]:
        raw, result = row['raw'], row['result']
        if raw['timestamp_ns'] > deadline:
            break
        label = {'index': row['index'], 'timestamp_ns': raw['timestamp_ns'],
                 'admitted': result.get('pose_valid') is True,
                 'quality_reason': (result.get('quality_gate') or {}).get('reason_code'),
                 'quarantine_cause': (result.get('recovery_preaccept') or {}).get('quarantine_cause'),
                 'classification': 'unassessable'}
        actual = truth(raw['timestamp_ns'])
        if not anchor_ready:
            label['reason'] = 'immediately_pre_mask_admission_or_GT_unavailable'
        elif visual_reference(raw) != identity:
            label['reason'] = 'different_reference_no_stitching'
        elif raw.get('pose_valid') is not True or raw.get('tracking_state') != 'OK':
            label['reason'] = 'native_visual_pose_unavailable'
        elif actual is None:
            label['reason'] = 'GT_unavailable_or_gap'
        else:
            translation, rotation = relative_motion_error(np.asarray(anchor['matrix']) @ camera_from_body,
                np.asarray(raw['matrix']) @ camera_from_body, anchor_truth, actual)
            good = translation <= translation_limit and rotation <= rotation_limit
            label.update(translation_error_m=translation, rotation_error_deg=rotation,
                classification=('within_envelope_admitted' if label['admitted'] else 'potential_false_rejection')
                    if good else ('outside_envelope_admitted' if label['admitted'] else 'outside_envelope_rejected'))
        samples.append(label)
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    plan_path, summary_path = args.run / 'plan.json', args.run / 'run_summary.json'
    plan, summary = [json.loads(p.read_text(encoding='utf-8')) for p in (plan_path, summary_path)]
    frames = args.run / 'frames.jsonl'
    assert digest(frames) == summary['frames_sha256'] and digest(plan_path) == summary['plan_sha256']
    rows = [json.loads(line) for line in frames.open(encoding='utf-8')]
    loader = ROOT / 'handoff/goal_delivery_20261113/benchmark_public_vio.py'
    spec = importlib.util.spec_from_file_location('public_gate_truth', loader)
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    _, settings, _, gt = public.paths(plan['sequence'])
    times, poses = public.load_gt(plan['sequence'])
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(settings.read_text(encoding='utf-8').splitlines()[1:]), Loader=Loader)
    camera_from_body = np.linalg.inv(np.asarray(config['IMU.T_b_c1']['data']).reshape(4, 4))
    samples = labels(rows, plan['mask'], lambda t: associate_truth(t, times, poses), camera_from_body)
    report = {'schema_version': 1, 'sequence': plan['sequence'], 'mask': plan['mask'],
        'classifications': dict(Counter(s['classification'] for s in samples)),
        'unassessable_reasons': dict(Counter(s.get('reason') for s in samples if s['classification'] == 'unassessable')),
        'potential_false_rejection_reasons': dict(Counter(s['quality_reason'] for s in samples if s['classification'] == 'potential_false_rejection')),
        'samples': samples, 'event_count': 1, 'independent_sample_count_is_not_frame_count': True,
        'limits': 'Fixed immediately pre-mask admitted pose anchors relative GT; no post-loss fit, scale/time fit or map stitching. Within-envelope rejected poses are potential false rejections: valid safety requirements can still justify rejection. No statistical risk claim.',
        'hashes': {'frames': digest(frames), 'plan': digest(plan_path), 'gt': digest(gt), 'settings': digest(settings), 'script': digest(Path(__file__))}}
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(report, output, indent=2, allow_nan=False)
    print(json.dumps({k: report[k] for k in ('sequence', 'classifications', 'unassessable_reasons')}))


if __name__ == '__main__':
    main()
