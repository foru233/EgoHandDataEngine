"""Compare recorded KF propagation with one-frame IMU propagation offline.

Neither estimator state nor admission is modified. GT is read only by this
auditor, after native replay. This is not a replacement VIO or closed-loop test.
"""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml

from audit_public_gate_gt import associate_truth
from audit_recovery_prediction_gt import relative_motion_error
from evaluate_vio_trajectory import digest, summary


def mat(value):
    return np.asarray(value, dtype=float).reshape(4, 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    paths = [args.run / name for name in
             ('plan.json', 'run_summary.json', 'native_commits.jsonl', 'frames.jsonl')]
    hashes = {str(p): digest(p) for p in paths}
    plan, sealed = [json.loads(p.read_text(encoding='utf8')) for p in paths[:2]]
    assert sealed['worker_exit_code'] == 0
    assert sealed['plan_sha256'] == hashes[str(paths[0])]
    assert sealed['frames_sha256'] == hashes[str(paths[3])]
    rows = [json.loads(line) for line in paths[3].open(encoding='utf8')]
    by_frame = {r['raw']['frame_id']: r for r in rows}
    records = [json.loads(line) for line in paths[2].open(encoding='utf8')]
    assert [r['sequence'] for r in records] == list(range(1, len(records) + 1))
    loader = root / 'handoff/goal_delivery_20261113/benchmark_public_vio.py'
    spec = importlib.util.spec_from_file_location('prediction_branch_truth', loader)
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    _, settings, _, gt = public.paths(plan['sequence'])
    times, poses = public.load_gt(plan['sequence'])

    class Loader(yaml.SafeLoader):
        pass

    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix',
                           lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(settings.read_text(encoding='utf8').splitlines()[1:]), Loader=Loader)
    camera_from_body = np.linalg.inv(np.asarray(config['IMU.T_b_c1']['data']).reshape(4, 4))
    before, samples, crosschecks = {}, [], []
    for record in records:
        if record['type'] != 'frame_stage':
            continue
        frame, phase = record['frame_id'], record['phase']
        if phase == 'prediction_before':
            before[frame] = record
            continue
        if phase != 'prediction_after' or frame not in before:
            continue
        initial = before[frame]
        previous = initial['previous']
        delta = initial['frame_preintegration']
        if (not previous['pose_valid'] or not previous['velocity_valid'] or delta is None
                or previous['id'] + 1 != frame or frame not in by_frame):
            continue
        dt = float(delta['dt_s'])
        stamp_dt = initial['current']['timestamp_sec'] - previous['timestamp_sec']
        if not 0 < stamp_dt <= .1 or abs(dt-stamp_dt) > .005:
            continue
        old = mat(previous['world_from_camera']) @ camera_from_body
        predicted = mat(record['current']['world_from_camera']) @ camera_from_body
        alternate = np.eye(4)
        alternate[:3, :3] = old[:3, :3] @ np.asarray(delta['delta_rotation']).reshape(3, 3)
        alternate[:3, 3] = (old[:3, 3] + np.asarray(previous['velocity'])*dt
                             + .5*dt*dt*np.array([0, 0, -9.81])
                             + old[:3, :3] @ np.asarray(delta['delta_position']))
        difference = float(np.max(np.abs(predicted-alternate)))
        if not initial['map_updated']:
            # In this branch native already uses this exact one-frame formula.
            # Cross-check its reconstruction before interpreting updated maps.
            crosschecks.append(difference)
            continue
        raw = by_frame[frame]['raw']
        gt_now = associate_truth(raw['timestamp_ns'], times, poses)
        previous_row = by_frame.get(previous['id'])
        gt_before = (associate_truth(previous_row['raw']['timestamp_ns'], times, poses)
                     if previous_row is not None else None)
        item = dict(frame_id=frame, index=by_frame[frame]['index'],
                    sequence_before=initial['sequence'], sequence_after=record['sequence'],
                    map_id=initial['map_id'], reference_generation=initial['reference_generation'],
                    geometry_sequence=initial['geometry_sequence'], dt_s=dt,
                    jump_rejected=by_frame[frame]['result'].get('jump_detected') is True,
                    actual_kf_prediction_step_m=float(np.linalg.norm(predicted[:3, 3]-old[:3, 3])),
                    hypothetical_frame_prediction_step_m=float(np.linalg.norm(alternate[:3, 3]-old[:3, 3])),
                    previous_velocity_m_s=previous['velocity'], last_keyframe=initial['last_keyframe'],
                    actual_body_pose=predicted.tolist(), hypothetical_body_pose=alternate.tolist(),
                    gt_assessable=gt_before is not None and gt_now is not None)
        if item['gt_assessable']:
            for name, candidate in [('actual', predicted), ('hypothetical', alternate)]:
                translation, rotation = relative_motion_error(old, candidate, gt_before, gt_now)
                item[name+'_relative_translation_error_m'] = translation
                item[name+'_relative_rotation_error_deg'] = rotation
        samples.append(item)
    assert hashes == {str(p): digest(p) for p in paths}
    failures = ([] if crosschecks and max(crosschecks) < 1e-4 else
                ['One-frame reconstruction did not match the actual no-map-update native branch'])
    report = dict(sequence=plan['sequence'], native_branch_crosscheck=summary(crosschecks),
                  updated_map_predictions=len(samples), samples=samples,
                  jump_samples=[s for s in samples if s['jump_rejected']], failures=failures,
                  hashes=hashes, gt_sha256=digest(gt), settings_sha256=digest(settings),
                  auditor_sha256=digest(Path(__file__)),
                  limits=['Hypothetical one-frame propagation only; no native state or host admission changes.',
                          'Local motion GT uses the internal previous pose as the estimate anchor; no post-loss fit, scale/time fit or map stitching.',
                          'A better prediction does not prove the resulting visual search or optimization would recover safely.',
                          'A changed reference, rebase or inertial prior can invalidate adopting the alternative; full identity/prior handling remains necessary.',
                          'Samples are correlated, not independent recovery trials.'])
    with args.output.open('x', encoding='utf8') as output:
        json.dump(report, output, indent=2, allow_nan=False)
    print(json.dumps(dict(updated_map_predictions=len(samples),
                         jump_samples=[{k: v for k, v in s.items() if k in
                             ('index', 'actual_relative_translation_error_m', 'hypothetical_relative_translation_error_m',
                              'actual_kf_prediction_step_m', 'hypothetical_frame_prediction_step_m')}
                                       for s in report['jump_samples']], failures=failures)))
    assert not failures, 'Reconstruction check failed; report preserved'


if __name__ == '__main__':
    main()
