"""Audit actual native visual-to-inertial handoff; read-only, never estimator input."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from evaluate_vio_trajectory import digest, summary


def matrix(value):
    return np.asarray(value, dtype=float).reshape(4, 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.run / name for name in ('plan.json', 'frames.jsonl', 'native_commits.jsonl', 'run_summary.json')]
    hashes = {str(p): digest(p) for p in paths}
    plan = json.loads(paths[0].read_text(encoding='utf8'))
    sealed = json.loads(paths[3].read_text(encoding='utf8'))
    assert sealed['worker_exit_code'] == 0
    assert hashes[str(paths[1])] == sealed['frames_sha256']
    assert hashes[str(paths[0])] == sealed['plan_sha256']
    rows = [json.loads(line) for line in paths[1].open(encoding='utf8')]
    by_frame = {r['raw']['frame_id']: r for r in rows}
    records = [json.loads(line) for line in paths[2].open(encoding='utf8')]
    assert [r['sequence'] for r in records] == list(range(1, len(records) + 1))
    enabled = plan['environment'].get('EGOHAND_VIO_VISUAL_VELOCITY_SEED', '0') == '1'
    seeds, transitions = [], []
    last_visual, last_phase = {}, None
    failures = []
    for r in records:
        if r['type'] != 'frame_stage':
            continue
        frame, phase = r['frame_id'], r['phase']
        if phase == 'local_visual':
            last_visual[frame] = r
        elif phase == 'recovery_visual_velocity':
            assert enabled, 'Seed ran in disabled arm'
            assert frame in last_visual and frame in by_frame and frame - 1 in by_frame
            row, previous = by_frame[frame], by_frame[frame - 1]
            cur = row['raw']['recovery_observation']
            old = previous['raw']['recovery_observation']
            dt = cur['timestamp_sec'] - old['timestamp_sec']
            fields = ('map_id', 'reset_id', 'atlas_generation', 'map_revision',
                      'reference_generation', 'geometry_sequence')
            qualified = all(o['available'] and o['qualified'] for o in (cur, old))
            same_geometry = all(cur[k] == old[k] for k in fields)
            expected = (matrix(cur['T_world_from_body'])[:3, 3] -
                        matrix(old['T_world_from_body'])[:3, 3]) / dt
            velocity = np.asarray(r['current']['velocity'])
            error = float(np.linalg.norm(velocity - expected))
            visual = last_visual[frame]['current']
            pose_error = float(np.max(np.abs(matrix(visual['world_from_camera']) -
                                            matrix(r['current']['world_from_camera']))))
            bias_same = visual['bias_gyro_acc'] == r['current']['bias_gyro_acc']
            checks = dict(qualified=qualified, same_geometry=same_geometry,
                          local_ba_version_unchanged=(frame - 1 in last_visual and
                              last_visual[frame - 1]['map_change_index'] == r['map_change_index']),
                          previous_tracking_ok=previous['raw']['tracking_state'] == 'OK',
                          consecutive_frames=old['frame_id'] + 1 == cur['frame_id'],
                          continuous_time=0 < dt <= .1, seed_matches_visual_difference=error < 1e-4,
                          visual_pose_unchanged=pose_error == 0, bias_unchanged=bias_same,
                          prior_unchanged=visual['has_inertial_prior'] == r['current']['has_inertial_prior'])
            if not all(checks.values()):
                failures.append({'frame_id': frame, 'checks': checks})
            seeds.append(dict(frame_id=frame, index=row['index'], sequence=r['sequence'],
                              velocity_m_s=velocity.tolist(), expected_m_s=expected.tolist(),
                              velocity_error_m_s=error, pose_change_max=pose_error, checks=checks))
        if phase in ('local_visual', 'local_inertial_keyframe', 'local_inertial_frame'):
            if last_phase and last_phase['phase'] == 'local_visual' and phase.startswith('local_inertial'):
                transitions.append({k: r[k] for k in ('frame_id', 'sequence', 'phase',
                                                       'current', 'previous', 'last_keyframe', 'map_id')})
            last_phase = r
    assert hashes == {str(p): digest(p) for p in paths}, 'Sealed run changed during audit'
    report = dict(sequence=plan['sequence'], native_flag_enabled=enabled,
                  native_log_contiguous=True, seed_count=len(seeds), seeds=seeds,
                  seed_error_m_s=summary([r['velocity_error_m_s'] for r in seeds]),
                  transitions=transitions, phase_counts=dict(Counter(r['phase'] for r in records if r['type'] == 'frame_stage')),
                  failures=failures, hashes=hashes, auditor_sha256=digest(Path(__file__)),
                  limits=['This verifies actual state handoff and unchanged visual pose, not absolute accuracy.',
                          'Velocity difference uses two genuine visual body poses from one unchanged geometry version.',
                          'No velocity observation means the feature was not exercised; it is not a successful recovery.',
                          'Review preloss anchored GT and complete post-reentry accuracy separately.'])
    with args.output.open('x', encoding='utf8') as output:
        json.dump(report, output, indent=2, allow_nan=False)
    print(json.dumps({k: report[k] for k in ('sequence', 'native_flag_enabled', 'seed_count', 'failures')}))
    assert not failures, 'Actual seed broke visual/identity/time/state invariants; report preserved'


if __name__ == '__main__':
    main()
