"""Audit actual native KF marginal use without altering a completed replay."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from opencv_graph_json import read_graph, nonfinite_paths, tracking_depth_only


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(value):
    return np.asarray(value['data'], dtype=float).reshape(value['rows'], value['cols'])


def stats(values):
    return dict(count=len(values), **({name: float(np.percentile(values, p))
        for name, p in [('p50', 50), ('p95', 95), ('p99', 99), ('max', 100)]} if values else {}))


def audit(run):
    plan = json.loads((run / 'plan.json').read_text())
    summary = json.loads((run / 'run_summary.json').read_text())
    errors = []
    for field, name in [('plan_sha256', 'plan.json'), ('frames_sha256', 'frames.jsonl')]:
        if summary[field] != sha(run / name):
            errors.append(dict(error='completed_run_hash_mismatch', file=name))
    if summary['worker_exit_code'] != 0 or not summary['input_and_source_integrity_passed']:
        errors.append(dict(error='worker_integrity_or_exit_failed'))
    journal = [json.loads(line) for line in (run / 'native_commits.jsonl').open()]
    if [r['sequence'] for r in journal] != list(range(1, len(journal) + 1)):
        errors.append(dict(error='noncontiguous_native_journal'))
    publications = {}
    before = {}
    uses = []
    phases = Counter()
    for row in journal:
        if row.get('type') == 'keyframe_marginal':
            publications[(row['map_id'], row['map_change_index'], row['keyframe_id'])] = row
        if row.get('type') != 'frame_stage':
            continue
        phases[row['phase']] += 1
        if row['phase'] == 'local_before':
            before[row['frame_id']] = row
        if row['phase'] != 'local_inertial_keyframe_marginal':
            continue
        failures = []
        prior = before.get(row['frame_id'])
        pub = publications.get((row['map_id'], row['map_change_index'], row['last_keyframe']['id']))
        if not pub or not pub['valid']:
            failures.append('no_preceding_valid_publication_at_same_map_version_and_kf')
        if not row['imu_initialized']:
            failures.append('imu_uninitialized')
        dt = row['current']['timestamp_sec'] - row['last_keyframe']['timestamp_sec']
        if not 0 < dt <= 2 or abs(dt-row['keyframe_preintegration']['dt_s']) > .005:
            failures.append('invalid_keyframe_integration_interval')
        if not row['current']['has_inertial_prior'] or row['previous']['has_inertial_prior']:
            failures.append('fresh_current_prior_or_previous_retirement_failed')
        if not prior:
            failures.append('local_before_missing')
        else:
            for key in ('map_id', 'map_change_index', 'revision', 'reference_generation', 'reference_operation'):
                if row[key] != prior[key]:
                    failures.append('reference_changed_' + key)
            if prior['current']['has_inertial_prior']:
                failures.append('current_prior_already_present_before_solve')
            if row['last_keyframe'] != prior['last_keyframe']:
                failures.append('shared_keyframe_mutated_inside_solve')
        uses.append(dict(frame=row['frame_id'], sequence=row['sequence'], map_id=row['map_id'],
            map_change_index=row['map_change_index'], keyframe_id=row['last_keyframe']['id'],
            ba2=row['ba2'], dt_s=dt, errors=failures))
        errors.extend(dict(frame=row['frame_id'], error=e) for e in failures)
    snapshots = []
    nonfinite_diagnostics = []
    for path in sorted((run / 'exact_native_graph').glob('*_before.json')):
        pre = read_graph(path)
        paths = nonfinite_paths(pre)
        if paths:
            nonfinite_diagnostics.append(dict(file=path.name, paths=paths))
            if not tracking_depth_only(paths):
                errors.append(dict(file=path.name, error='nonfinite_outside_tracking_depth', paths=paths))
        marginal = pre.get('keyframe', {}).get('state_marginal', {})
        if not marginal.get('available'):
            continue
        frame = int(pre['frame_id'])
        after_paths = [p for p in path.parent.glob(f'frame_{frame}_*.json') if p != path]
        failures = []
        info, cov = matrix(marginal['information']), matrix(marginal['covariance'])
        finite = info.shape == cov.shape == (15, 15) and np.isfinite(info).all() and np.isfinite(cov).all()
        inverse_error = None
        eig_min = None
        if not finite:
            failures.append('invalid_marginal_matrix')
        else:
            eig_min = [float(np.linalg.eigvalsh((m + m.T)*.5).min()) for m in (info, cov)]
            inverse_error = float(np.abs(info @ cov - np.eye(15)).max())
            if min(eig_min) <= 0:
                failures.append('marginal_not_positive_definite')
            if inverse_error > 1e-5:
                failures.append('marginal_inverse_inconsistent')
        phase = None
        if len(after_paths) != 1:
            failures.append('missing_or_ambiguous_after_snapshot')
        else:
            post = read_graph(after_paths[0])
            phase = post['phase']
            if phase == 'local_inertial_keyframe_marginal':
                if not post['current']['prior']['available'] or post['previous']['prior']['available']:
                    failures.append('snapshot_prior_lifecycle_failed')
                if pre['keyframe'] != post['keyframe']:
                    failures.append('snapshot_shared_keyframe_mutated')
        snapshots.append(dict(frame=frame, phase=phase, eigen_min_information_covariance=eig_min,
            inverse_max_error=inverse_error, errors=failures))
        errors.extend(dict(frame=frame, error=e) for e in failures)
    records = [r for r in journal if r.get('type') == 'keyframe_marginal']
    enabled = plan['environment'].get('EGOHAND_VIO_KF_COVARIANCE_PRIOR') == '1'
    if not enabled and uses:
        errors.append(dict(error='disabled_arm_used_marginal'))
    return dict(run=str(run), enabled=enabled, journal_records=len(journal), phases=dict(phases),
        publications=dict(Counter('valid' if r['valid'] else 'invalid' for r in records)),
        marginal_extraction_ms=stats([r['elapsed_ms'] for r in records]),
        actual_marginal_uses=len(uses), uses_before_ba2=sum(not r['ba2'] for r in uses), uses=uses,
        captured_available_marginals=snapshots, errors=errors,
        nonfinite_tracking_diagnostics=nonfinite_diagnostics,
        hashes={str(p): sha(p) for p in [run / 'plan.json', run / 'run_summary.json', run / 'native_commits.jsonl']},
        acceptance=False, production_deployed=False,
        limits=['Checks actual publication and prior lifecycle, not just environment flags.',
                'Selected graph captures only; no full-map posterior or recovery correctness claim.',
                'Unexported KF posterior reference prevents assessing its final inertial residual.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.run.resolve(strict=True))
    result['auditor_sha256'] = sha(Path(__file__))
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('actual_marginal_uses', 'publications', 'marginal_extraction_ms', 'errors')}))
    raise SystemExit(bool(result['errors']))


if __name__ == '__main__':
    main()
