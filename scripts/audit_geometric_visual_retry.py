"""Read-only audit of actual inertial rejection and transactional visual retry.

The runtime journal does not contain the re-estimated previous copy. This audit
therefore does not claim to independently verify the two-pose velocity formula.
Absolute accuracy must be assessed with the separate, fixed preloss GT anchor.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from evaluate_vio_trajectory import digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.run / name for name in
             ('plan.json', 'frames.jsonl', 'native_commits.jsonl', 'run_summary.json')]
    hashes = {str(p): digest(p) for p in paths}
    plan = json.loads(paths[0].read_text(encoding='utf8'))
    sealed = json.loads(paths[3].read_text(encoding='utf8'))
    assert sealed['worker_exit_code'] == 0, 'Only completed worker runs are assessable'
    assert hashes[str(paths[1])] == sealed['frames_sha256']
    assert hashes[str(paths[0])] == sealed['plan_sha256']
    rows = [json.loads(line) for line in paths[1].open(encoding='utf8')]
    by_frame = {r['raw']['frame_id']: r for r in rows}
    records = [json.loads(line) for line in paths[2].open(encoding='utf8')]
    assert [r['sequence'] for r in records] == list(range(1, len(records) + 1))
    enabled = plan['environment'].get('EGOHAND_VIO_GEOMETRIC_RETRY', '0') == '1'
    attempts, failures, pending = [], [], {}
    for record in records:
        if record['type'] != 'frame_stage':
            continue
        frame, phase = record['frame_id'], record['phase']
        if phase == 'inertial_geometry_rejected':
            if frame in pending:
                failures.append(dict(frame_id=frame, reason='duplicate_rejection'))
            pending[frame] = record
            continue
        if phase == 'local_visual_retry' and frame not in pending:
            failures.append(dict(frame_id=frame, reason='retry_without_rejected_inertial_result'))
        if frame not in pending or phase not in (
                'local_visual_retry', 'local_inertial_keyframe', 'local_inertial_frame'):
            continue
        before = pending.pop(frame)
        success = phase == 'local_visual_retry'
        identity = ('map_id', 'map_change_index', 'revision', 'geometry_sequence',
                    'geometry_active', 'reference_generation', 'reference_operation')
        checks = dict(flag_enabled=enabled,
                      same_identity=all(before[k] == record[k] for k in identity),
                      previous_original_unchanged=before['previous'] == record['previous'],
                      keyframe_unchanged=before['last_keyframe'] == record['last_keyframe'],
                      frame_preintegration_unchanged=before['frame_preintegration'] == record['frame_preintegration'],
                      keyframe_preintegration_unchanged=before['keyframe_preintegration'] == record['keyframe_preintegration'],
                      bias_unchanged=before['current']['bias_gyro_acc'] == record['current']['bias_gyro_acc'])
        raw = by_frame[frame]['raw']
        geometry = raw.get('recovery_observation') or {}
        if success:
            velocity = record['current']['velocity']
            dt = record['current']['timestamp_sec'] - record['previous']['timestamp_sec']
            checks.update(pose_valid=record['current']['pose_valid'],
                          finite_velocity=record['current']['velocity_valid'] and
                              bool(np.isfinite(np.asarray(velocity, dtype=float)).all()),
                          consecutive_frames=record['previous']['id'] + 1 == frame,
                          continuous_time=0 < dt <= .1,
                          rejected_prior_existed=before['current']['has_inertial_prior'],
                          rejected_prior_retired=not record['current']['has_inertial_prior'],
                          current_geometry_qualified=geometry.get('available') is True and
                              geometry.get('qualified') is True and geometry.get('frame_id') == frame)
        else:
            checks['failed_retry_preserved_current'] = before['current'] == record['current']
        if not all(checks.values()):
            failures.append(dict(frame_id=frame, checks=checks))
        attempts.append(dict(frame_id=frame, index=by_frame[frame]['index'],
                             rejected_sequence=before['sequence'], result_sequence=record['sequence'],
                             success=success, checks=checks,
                             before=before, after=record,
                             returned_tracking_state=raw['tracking_state'],
                             returned_geometry=geometry))
    failures += [dict(frame_id=f, reason='rejection_without_result') for f in pending]
    assert hashes == {str(p): digest(p) for p in paths}, 'Sealed run changed during audit'
    report = dict(sequence=plan['sequence'], native_flag_enabled=enabled,
                  attempts=len(attempts), successful_retries=sum(a['success'] for a in attempts),
                  unsuccessful_retries=sum(not a['success'] for a in attempts),
                  events=attempts, failures=failures, hashes=hashes,
                  phase_counts=dict(Counter(r['phase'] for r in records if r['type'] == 'frame_stage')),
                  auditor_sha256=digest(Path(__file__)),
                  limits=['Transactional audit, not absolute accuracy or map correctness.',
                          'The previous re-estimated copy is not journaled; real-run velocity formula is not independently verified here.',
                          'Native pinhole/fisheye optimizer units separately exercise two-frame visual velocity.',
                          'Judge recovery with preloss anchored GT beyond first readmission, including subsequent jumps.',
                          'Repeated frames and retries are correlated, not independent recovery samples.'])
    with args.output.open('x', encoding='utf8') as output:
        json.dump(report, output, indent=2, allow_nan=False)
    print(json.dumps({k: report[k] for k in
                      ('sequence', 'native_flag_enabled', 'attempts', 'successful_retries', 'unsuccessful_retries', 'failures')}))
    assert not failures, 'Retry invariant failure; diagnostic report preserved'


if __name__ == '__main__':
    main()
