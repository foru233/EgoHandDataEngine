"""Audit actual two-frame visual retry from preserved native search matches."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from evaluate_vio_trajectory import digest


def matrix(value):
    return np.asarray(value, dtype=float).reshape(4, 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.run / name for name in ('plan.json', 'run_summary.json', 'frames.jsonl', 'native_commits.jsonl')]
    hashes = {str(p): digest(p) for p in paths}
    plan, sealed = [json.loads(p.read_text(encoding='utf8')) for p in paths[:2]]
    assert sealed['worker_exit_code'] == 0
    assert sealed['plan_sha256'] == digest(paths[0]) and sealed['frames_sha256'] == digest(paths[2])
    rows = [json.loads(s) for s in paths[2].open(encoding='utf8')]
    by_frame = {r['raw']['frame_id']: r for r in rows}
    records = [json.loads(s) for s in paths[3].open(encoding='utf8')]
    assert [r['sequence'] for r in records] == list(range(1, len(records) + 1))
    enabled = plan['environment'].get('EGOHAND_VIO_GEOMETRIC_RETRY', '0') == '1'
    initial, rejected, visual, decisions, events, failures = {}, {}, {}, {}, [], []
    for r in records:
        frame = r.get('frame_id')
        if r['type'] == 'visual_retry_geometry':
            visual.setdefault(frame, {})[r['which']] = r['observation']
        if r['type'] == 'visual_retry_decision':
            decisions[frame] = r['reason']
        if r['type'] != 'frame_stage':
            continue
        phase = r['phase']
        if phase == 'local_before':
            initial[frame] = r
        if phase == 'inertial_geometry_rejected':
            assert frame not in rejected
            rejected[frame] = r
        if phase not in ('preoptimizer_retry_committed', 'preoptimizer_retry_failed'):
            continue
        before = rejected.pop(frame)
        search = initial[frame]
        success = phase == 'preoptimizer_retry_committed'
        fields = ('map_id', 'map_change_index', 'revision', 'geometry_sequence',
                  'geometry_active', 'reference_generation', 'reference_operation')
        checks = dict(flag_enabled=enabled,
                      same_identity=all(search[k] == before[k] == r[k] for k in fields),
                      previous_unchanged=search['previous'] == before['previous'] == r['previous'],
                      keyframe_unchanged=search['last_keyframe'] == before['last_keyframe'] == r['last_keyframe'],
                      frame_delta_unchanged=search['frame_preintegration'] == before['frame_preintegration'] == r['frame_preintegration'],
                      keyframe_delta_unchanged=search['keyframe_preintegration'] == before['keyframe_preintegration'] == r['keyframe_preintegration'])
        detail = {}
        if success:
            old, cur = visual[frame]['previous'], visual[frame]['current']
            dt = cur['timestamp_sec'] - old['timestamp_sec']
            expected = (matrix(cur['T_world_from_body'])[:3, 3] - matrix(old['T_world_from_body'])[:3, 3]) / dt
            velocity_error = float(np.linalg.norm(expected - r['current']['velocity']))
            pose_error = float(np.max(np.abs(matrix(cur['T_world_from_camera']) - matrix(r['current']['world_from_camera']))))
            identity = ('map_id', 'reset_id', 'atlas_generation', 'map_revision', 'reference_generation', 'geometry_sequence')
            final = by_frame[frame]['raw']['recovery_observation']
            checks.update(both_visual_geometries=old['available'] and old['qualified'] and cur['available'] and cur['qualified'],
                          same_visual_reference=all(old[k] == cur[k] == final[k] for k in identity),
                          frame_time_continuous=old['frame_id'] + 1 == cur['frame_id'] == frame and 0 < dt <= .1,
                          actual_visual_velocity=velocity_error < 1e-4,
                          actual_visual_pose=pose_error < 1e-5,
                          bias_restored_to_search=r['current']['bias_gyro_acc'] == search['current']['bias_gyro_acc'],
                          prior_fresh_before_rejection=before['current']['has_inertial_prior'] and not search['current']['has_inertial_prior'],
                          rejected_prior_retired=not r['current']['has_inertial_prior'],
                          final_visual_geometry=final['qualified'] and final['available'],
                          decision_committed=decisions[frame] == 'committed_visual_retry')
            detail = dict(velocity_error_m_s=velocity_error, pose_error_max=pose_error,
                          previous_visual=old, current_visual=cur)
        else:
            checks['failed_retry_preserved_current'] = before['current'] == r['current']
        event = dict(frame_id=frame, index=by_frame[frame]['index'], success=success,
                     reason=decisions.get(frame), checks=checks, **detail)
        events.append(event)
        if not all(checks.values()):
            failures.append(event)
    failures += [dict(frame_id=f, reason='rejection_without_result') for f in rejected]
    assert hashes == {str(p): digest(p) for p in paths}
    report = dict(sequence=plan['sequence'], enabled=enabled, attempts=len(events),
                  successful_retries=sum(e['success'] for e in events), events=events,
                  reasons=dict(Counter(decisions.values())), failures=failures, hashes=hashes,
                  auditor_sha256=digest(Path(__file__)),
                  limits=['Actual transactional and geometry checks, not absolute accuracy or independent map proof.',
                          'Fixed preloss GT and full subsequent trajectory must be assessed separately.',
                          'Repeated frames are correlated. No attempt means the retry was not exercised.'])
    with args.output.open('x', encoding='utf8') as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps({k: report[k] for k in ('sequence', 'enabled', 'attempts', 'successful_retries', 'reasons', 'failures')}))
    assert not failures, 'Actual retry invariant failure; report preserved'


if __name__ == '__main__':
    main()
