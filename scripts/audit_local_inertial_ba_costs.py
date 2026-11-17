"""Audit opt-in native local BA observations without changing estimator state."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np


PARTS = ('mono', 'stereo', 'inertial', 'gyro_rw', 'acc_rw')


def audit(path):
    rows = [json.loads(line) for line in path.read_text(encoding='utf8').splitlines()]
    errors, pairs, pending = [], [], {}
    previous_sequence = 0
    commits = {}
    for row in rows:
        sequence = row['sequence']
        if sequence != previous_sequence + 1:
            errors.append(['journal_sequence', previous_sequence, sequence])
        previous_sequence = sequence
        if row['type'] == 'local_ba' and row['kind'] == 'local_inertial_ba':
            commits.setdefault(row['event'], {})[row['phase']] = row
        if row['type'] != 'local_inertial_solver_cost':
            continue
        kid = row['latest_keyframe_id']
        total = sum(row[key]['robust_chi2_sum'] for key in PARTS)
        cached = row['cached_robust_cost']
        if cached is None or abs(total - cached) > max(1e-4, abs(total) * 2e-7):
            errors.append(['cost_sum', sequence, total, cached])
        if row['invalid_state_count'] or any(row[key]['nonfinite'] for key in PARTS):
            errors.append(['nonfinite', sequence])
        if row['phase'] == 'before':
            if kid in pending:
                errors.append(['unpaired_before', kid])
            pending[kid] = row
        elif row['phase'] == 'after':
            before = pending.pop(kid, None)
            if before is None:
                errors.append(['unpaired_after', kid])
                continue
            changes = {}
            for key in PARTS:
                if before[key]['count'] != row[key]['count']:
                    errors.append(['edge_count_changed', kid, key])
                changes[key] = dict(before=before[key]['robust_chi2_sum'],
                                    after=row[key]['robust_chi2_sum'], count=row[key]['count'])
            pair = dict(latest_keyframe_id=kid, before_sequence=before['sequence'],
                        after_sequence=sequence, large=row['large'],
                        recovery_initialization=row['recovery_initialization'],
                        stop_before=before['stop_requested'], stop_after=row['stop_requested'],
                        iterations=row['iterations_returned'], costs=changes,
                        total_before=before['cached_robust_cost'], total_after=cached,
                        max_camera_center_change_m=row['max_camera_center_change_m'],
                        max_velocity_change_mps=row['max_velocity_change_mps'],
                        max_gyro_bias_change=row['max_gyro_bias_change'],
                        max_acc_bias_change=row['max_acc_bias_change'])
            pair['visual_before'] = sum(changes[k]['before'] for k in ('mono', 'stereo'))
            pair['visual_after'] = sum(changes[k]['after'] for k in ('mono', 'stereo'))
            pair['native_cost_gate_would_reject'] = bool(
                cached > 2 * before['cached_robust_cost'] and not row['large'])
            pair['_states'] = row['states']
            pairs.append(pair)
    if pending:
        errors.append(['unpaired_before_end', list(pending)])
    ordered = sorted(commits.values(), key=lambda c: c.get('before', c.get('after'))['sequence'])
    for i, pair in enumerate(pairs):
        end = pairs[i + 1]['before_sequence'] if i + 1 < len(pairs) else previous_sequence + 1
        matches = [c for c in ordered if 'before' in c and
                   pair['after_sequence'] < c['before']['sequence'] < end]
        if len(matches) > 1:
            errors.append(['ambiguous_commit', pair['latest_keyframe_id']])
        pair['commit_event'] = None
        states = pair.pop('_states')
        if len(matches) == 1:
            commit = matches[0]
            pair['commit_event'] = commit['before']['event']
            pair['map_id'] = commit['before']['map_id']
            if 'after' not in commit:
                errors.append(['unpaired_commit', pair['commit_event']])
                continue
            actual = {s['id']: s for s in commit['after']['keyframes']}
            prior = {s['id']: s for s in commit['before']['keyframes']}
            differences, displacements = [], []
            for state in states:
                if state['id'] not in actual:
                    errors.append(['missing_committed_kf', pair['commit_event'], state['id']])
                    continue
                expected = np.array(state['world_from_camera']).reshape(4, 4)
                got = np.array(actual[state['id']]['world_from_camera']).reshape(4, 4)
                differences.append(float(np.abs(got - expected).max()))
                if state['id'] in prior:
                    old = np.array(prior[state['id']]['world_from_camera']).reshape(4, 4)
                    displacements.append(dict(id=state['id'], change_m=float(np.linalg.norm(got[:3, 3] - old[:3, 3]))))
            pair['commit_matrix_max_error'] = max(differences, default=None)
            pair['largest_committed_kf_changes'] = sorted(displacements, key=lambda s: s['change_m'], reverse=True)[:3]
            if differences and max(differences) > 1e-4:
                errors.append(['solver_commit_mismatch', pair['commit_event'], max(differences)])
    summary = dict(solver_pairs=len(pairs), committed=sum(p['commit_event'] is not None for p in pairs),
                   stop_before=sum(p['stop_before'] for p in pairs),
                   stop_after=sum(p['stop_after'] for p in pairs),
                   iterations=dict(Counter(p['iterations'] for p in pairs)),
                   cost_increased=sum(p['total_after'] > p['total_before'] for p in pairs),
                   visual_cost_increased=sum(p['visual_after'] > p['visual_before'] for p in pairs),
                   max_camera_center_change_m=max((p['max_camera_center_change_m'] for p in pairs), default=None))
    return dict(summary=summary, errors=errors, pairs=pairs, input=str(path.resolve()),
                input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                limits=['Cached edge errors may reflect a rejected LM trial after vertex rollback.',
                        'Cost decrease does not establish physical correctness or ground-truth accuracy.',
                        'Opt-in logging changes asynchronous scheduling; no speedup claim.',
                        'This is an observation audit, not a deployed algorithm change.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('journal', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = audit(args.journal)
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(dict(summary=result['summary'], errors=result['errors']), ensure_ascii=False))
    raise SystemExit(bool(result['errors']))
