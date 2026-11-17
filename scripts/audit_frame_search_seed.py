"""Verify actual native feature-search seeds in a sealed public replay."""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml

from evaluate_vio_trajectory import digest, summary


def matrix(value):
    return np.asarray(value, dtype=float).reshape(4, 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.run / n for n in ('plan.json', 'run_summary.json', 'frames.jsonl', 'native_commits.jsonl')]
    hashes = {str(p): digest(p) for p in paths}
    plan, sealed = [json.loads(p.read_text(encoding='utf8')) for p in paths[:2]]
    assert sealed['worker_exit_code'] == 0
    assert sealed['plan_sha256'] == digest(paths[0]) and sealed['frames_sha256'] == digest(paths[2])
    rows = [json.loads(s) for s in paths[2].open(encoding='utf8')]
    by_frame = {r['raw']['frame_id']: r for r in rows}
    records = [json.loads(s) for s in paths[3].open(encoding='utf8')]
    assert [r['sequence'] for r in records] == list(range(1, len(records) + 1))
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('search_calibration', root / 'handoff/goal_delivery_20261113/benchmark_public_vio.py')
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    _, settings, _, _ = public.paths(plan['sequence'])

    class Loader(yaml.SafeLoader):
        pass

    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix',
                           lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(settings.read_text(encoding='utf8').splitlines()[1:]), Loader=Loader)
    camera_from_body = np.linalg.inv(np.asarray(config['IMU.T_b_c1']['data']).reshape(4, 4))
    enabled = plan['environment'].get('EGOHAND_VIO_FRAME_SEARCH_SEED', '0') == '1'
    before, choices, seeds, failures = {}, {}, [], []
    for r in records:
        frame = r.get('frame_id')
        if r['type'] == 'frame_search_seed_choice':
            assert enabled and frame not in choices
            choices[frame] = r
        if r['type'] != 'frame_stage':
            continue
        if r['phase'] == 'prediction_before':
            before[frame] = r
        if r['phase'] != 'prediction_after' or not choices.get(frame, {}).get('selected'):
            continue
        initial = before[frame]
        previous = initial['previous']
        old = by_frame[frame - 1]['raw']
        evidence = old['recovery_observation']
        current_raw = by_frame[frame]['raw']
        delta = initial['frame_preintegration']
        dt = delta['dt_s']
        body = matrix(previous['world_from_camera']) @ camera_from_body
        expected = np.eye(4)
        expected[:3, :3] = body[:3, :3] @ np.asarray(delta['delta_rotation']).reshape(3, 3)
        expected[:3, 3] = (body[:3, 3] + np.asarray(previous['velocity']) * dt
                           + .5 * dt * dt * np.array([0, 0, -9.81])
                           + body[:3, :3] @ np.asarray(delta['delta_position']))
        velocity = (np.asarray(previous['velocity']) + dt * np.array([0, 0, -9.81])
                    + body[:3, :3] @ np.asarray(delta['delta_velocity']))
        actual = matrix(r['current']['world_from_camera']) @ camera_from_body
        pose_error = float(np.max(np.abs(expected - actual)))
        velocity_error = float(np.max(np.abs(velocity - r['current']['velocity'])))
        checks = dict(
            enabled=enabled, ordinary_update=initial['map_updated'] is True,
            initialized=initial['imu_initialized'] is True,
            current_and_previous_ok=initial['tracking_state'] == 2 and old['tracking_state'] == 'OK',
            previous_visual_geometry=evidence['available'] and evidence['qualified'],
            evidence_frame_time=(evidence['frame_id'] == previous['id'] == frame - 1 and
                                 evidence['timestamp_sec'] == previous['timestamp_sec']),
            map_identity=(evidence['map_id'] == initial['map_id'] == current_raw['map_id'] and
                          evidence['reset_id'] == current_raw['reset_id'] and
                          evidence['atlas_generation'] == current_raw['reference_frame']['atlas_generation']),
            reference_identity=(evidence['reference_generation'] == initial['reference_generation'] and
                                evidence['map_revision'] == initial['revision'] and
                                initial['reference_generation'] > 0 and initial['reference_operation'] > 0),
            geometry_writer_fence=(evidence['geometry_sequence'] == initial['geometry_sequence'] and
                                   initial['geometry_active'] == 0),
            continuous_interval=(0 < dt <= .1 and abs(dt - (initial['current']['timestamp_sec'] -
                                                         previous['timestamp_sec'])) <= .005),
            pose_formula=pose_error < 1e-4, velocity_formula=velocity_error < 1e-4,
            bias_from_previous=r['current']['bias_gyro_acc'] == previous['bias_gyro_acc'],
            previous_unchanged=initial['previous'] == r['previous'],
            keyframe_unchanged=initial['last_keyframe'] == r['last_keyframe'],
            frame_preintegration_unchanged=initial['frame_preintegration'] == r['frame_preintegration'],
            keyframe_preintegration_unchanged=initial['keyframe_preintegration'] == r['keyframe_preintegration'],
            prior_unchanged=initial['current']['has_inertial_prior'] == r['current']['has_inertial_prior'])
        item = dict(frame_id=frame, index=by_frame[frame]['index'], pose_error_max=pose_error,
                    velocity_error_max=velocity_error, checks=checks)
        seeds.append(item)
        if not all(checks.values()):
            failures.append(item)
    assert len(seeds) == sum(bool(r['selected']) for r in choices.values())
    assert hashes == {str(p): digest(p) for p in paths}
    report = dict(sequence=plan['sequence'], enabled=enabled, choices=len(choices), seed_count=len(seeds),
                  reasons=dict(Counter(r['reason'] for r in choices.values())), seeds=seeds,
                  pose_error=summary([r['pose_error_max'] for r in seeds]),
                  velocity_error=summary([r['velocity_error_max'] for r in seeds]), failures=failures,
                  hashes=hashes, auditor_sha256=digest(Path(__file__)), settings_sha256=digest(settings),
                  limits=['Search initialization only; no trusted map or GUI prediction claim.',
                          'Final visual optimization and fixed preloss GT must be assessed separately.',
                          'No seeds means this feature was not exercised. Samples are correlated.'])
    with args.output.open('x', encoding='utf8') as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps({k: report[k] for k in ('sequence', 'enabled', 'choices', 'seed_count', 'failures')}))
    assert not failures, 'Seed invariants failed; report preserved'


if __name__ == '__main__':
    main()
