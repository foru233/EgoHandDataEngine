"""Evaluate every same-reference native camera step against public GT.

Read-only diagnosis. Local motion agreement cannot establish global map accuracy
or authorize a pose that other admission requirements reject.
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml

from audit_public_gate_gt import associate_truth
from audit_recovery_prediction_gt import relative_motion_error
from evaluate_vio_trajectory import digest, summary
from egohand.vio.short_recovery import visual_reference

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads((args.run / 'plan.json').read_text(encoding='utf8'))
    sealed = json.loads((args.run / 'run_summary.json').read_text(encoding='utf8'))
    assert sealed['frames_sha256'] == digest(args.run / 'frames.jsonl')
    assert sealed['plan_sha256'] == digest(args.run / 'plan.json')
    rows = [json.loads(line) for line in (args.run / 'frames.jsonl').open(encoding='utf8')]
    spec = importlib.util.spec_from_file_location('step_public_gt', ROOT / 'handoff/goal_delivery_20261113/benchmark_public_vio.py')
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    _, settings, _, gt = public.paths(plan['sequence'])
    times, poses = public.load_gt(plan['sequence'])
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(settings.read_text(encoding='utf8').splitlines()[1:]), Loader=Loader)
    body_from_camera = np.asarray(config['IMU.T_b_c1']['data']).reshape(4, 4)
    quality = plan['quality_options']
    speed = quality.get('max_translation_speed_mps', 1.5)
    allowance = quality.get('translation_noise_allowance_m', .02)
    rot_speed = quality.get('max_rotation_speed_deg_s', 360.)
    rot_allowance = quality.get('rotation_noise_allowance_deg', 3.)
    samples = []
    counts = Counter()
    for previous, current in zip(rows, rows[1:]):
        a, b = previous['raw'], current['raw']
        jump = current['result'].get('jump_detected') is True
        row = dict(index=current['index'], jump_detected=jump, admitted=current['result'].get('pose_valid') is True,
            geometry_qualified=(b.get('recovery_observation') or {}).get('qualified'), assessment='unassessable')
        reason = None
        if not all(v.get('pose_valid') is True and v.get('tracking_state') == 'OK' for v in (a, b)):
            reason = 'consecutive_native_visual_pose_unavailable'
        elif visual_reference(a) != visual_reference(b):
            reason = 'reference_transition_no_stitching'
        else:
            before_gt = associate_truth(a['timestamp_ns'], times, poses)
            after_gt = associate_truth(b['timestamp_ns'], times, poses)
            if before_gt is None or after_gt is None:
                reason = 'GT_unavailable_or_gap'
            else:
                dt = (b['timestamp_ns'] - a['timestamp_ns']) / 1e9
                assert dt > 0
                first, last = np.asarray(a['matrix']), np.asarray(b['matrix'])
                gt_first, gt_last = before_gt @ body_from_camera, after_gt @ body_from_camera
                trans_error, rot_error = relative_motion_error(first, last, gt_first, gt_last)
                native_step = float(np.linalg.norm(last[:3, 3] - first[:3, 3]))
                gt_step = float(np.linalg.norm(gt_last[:3, 3] - gt_first[:3, 3]))
                def angle(first, last):
                    return float(np.rad2deg(np.arccos(np.clip((np.trace(first[:3, :3].T @ last[:3, :3])-1)/2, -1, 1))))
                native_angle, gt_angle = angle(first, last), angle(gt_first, gt_last)
                gt_exceeds = gt_step > allowance + speed*dt or gt_angle > rot_allowance + rot_speed*dt
                local_agrees = trans_error <= allowance and rot_error <= rot_allowance
                row.update(assessment='scored', dt_s=dt, native_camera_step_m=native_step,
                    GT_camera_step_m=gt_step, GT_camera_speed_mps=gt_step/dt,
                    local_relative_translation_error_m=trans_error, local_relative_rotation_error_deg=rot_error,
                    translation_limit_m=allowance+speed*dt, GT_motion_exceeds_fixed_envelope=gt_exceeds,
                    local_motion_within_noise_allowance=local_agrees,
                    recorded_quality_translation_step_m=current['result'].get('translation_step_m'),
                    recorded_quality_rotation_step_deg=current['result'].get('rotation_step_deg'))
                recorded = row['recorded_quality_translation_step_m']
                if jump and recorded is not None and abs(recorded-native_step) > 1e-5:
                    row['assessment'] = 'unassessable'
                    reason = 'quality_anchor_differs_from_consecutive_raw_pose'
                else:
                    row['classification'] = ('GT_fast_motion_local_agreement' if gt_exceeds and local_agrees
                        else 'local_motion_disagrees' if not local_agrees else 'GT_within_fixed_envelope_local_agreement')
        if reason:
            row['reason'] = reason
        counts[row['assessment']] += 1
        samples.append(row)
    scored = [r for r in samples if r['assessment'] == 'scored']
    jumps = [r for r in samples if r['jump_detected']]
    result = dict(sequence=plan['sequence'], frames=len(rows), assessed_counts=dict(counts),
        jump_assessment=dict(Counter(r.get('reason', r.get('classification')) for r in jumps)),
        all_scored_classifications=dict(Counter(r['classification'] for r in scored)),
        local_relative_translation_error_m=summary([r['local_relative_translation_error_m'] for r in scored]),
        local_relative_rotation_error_deg=summary([r['local_relative_rotation_error_deg'] for r in scored]),
        GT_camera_speed_mps=summary([r['GT_camera_speed_mps'] for r in scored]), jump_events=jumps,
        fast_motion_frame_indices=[r['index'] for r in scored if r['classification']=='GT_fast_motion_local_agreement'],
        thresholds_from_quality_options=quality, samples=samples,
        hashes={str(p): digest(p) for p in (args.run/'plan.json', args.run/'frames.jsonl', settings, gt, Path(__file__))},
        production_deployed=False, acceptance=False,
        limits=['Read-only local-step diagnosis; no global alignment, scale/time fit or cross-reference stitching.',
            'Local motion agreement does not validate accumulated map position, bias or reference identity.',
            'GT_fast_motion_local_agreement is only a hypothesis about a fixed-speed rejection, not proof to release quarantine.',
            'No threshold, native pose, quality decision or production source is changed.',
            'Correlated frames are not independent recovery events or statistical safety trials.'])
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('sequence', 'assessed_counts', 'jump_assessment', 'GT_camera_speed_mps')}))


if __name__ == '__main__':
    main()
