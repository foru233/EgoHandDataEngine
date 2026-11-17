"""Inspect native inertial edge intervals, information and actual-state residuals."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def audit(path):
    errors, reports, active = [], [], {}
    for line in path.open(encoding='utf8'):
        row = json.loads(line)
        if row['type'] != 'local_inertial_solver_cost':
            continue
        kid, phase = row['latest_keyframe_id'], row['phase']
        report = dict(sequence=row['sequence'], latest_keyframe_id=kid, phase=phase,
                      evaluated_total=0., cached_total=row['inertial']['robust_chi2_sum'],
                      max_cache_error=0., newest_edge_cost=0., edges=[])
        for edge in row['inertial_details']:
            info = np.array(edge['information']).reshape(9, 9)
            covariance = np.array(edge['preintegration_covariance']).reshape(15, 15)
            evaluated = np.array(edge['evaluated_error'])
            cached = np.array(edge['cached_error'])
            if not all(np.isfinite(x).all() for x in (info, covariance, evaluated, cached)):
                errors.append(['nonfinite', row['sequence'], edge['to_id']])
                continue
            span = edge['timestamp_span_s']
            if span is None or abs(edge['dt_s'] - span) > max(1e-4, abs(span) * 1e-5):
                errors.append(['interval_mismatch', row['sequence'], edge['to_id'], edge['dt_s'], span])
            eigenvalues = np.linalg.eigvalsh((info + info.T) / 2)
            if eigenvalues[0] < -max(1e-5, eigenvalues[-1] * 1e-12):
                errors.append(['indefinite_information', row['sequence'], edge['to_id']])
            chi2 = float(evaluated @ info @ evaluated)
            if abs(chi2 - edge['evaluated_chi2']) > max(1e-5, abs(chi2) * 1e-8):
                errors.append(['evaluated_chi2', row['sequence'], edge['to_id']])
            cache_error = float(np.max(np.abs(evaluated - cached)))
            if phase == 'before' and cache_error > 1e-7:
                errors.append(['before_cache_inconsistent', row['sequence'], edge['to_id'], cache_error])
            report['evaluated_total'] += edge['evaluated_robust_chi2']
            report['max_cache_error'] = max(report['max_cache_error'], cache_error)
            if edge['to_id'] == kid:
                report['newest_edge_cost'] += edge['evaluated_robust_chi2']
            details = dict(from_id=edge['from_id'], to_id=edge['to_id'], dt_s=edge['dt_s'],
                           timestamp_span_s=span, fixed_from=edge['fixed_from'],
                           evaluated_robust_chi2=edge['evaluated_robust_chi2'],
                           evaluated_rotation_error_rad=float(np.linalg.norm(evaluated[:3])),
                           evaluated_velocity_error_mps=float(np.linalg.norm(evaluated[3:6])),
                           evaluated_position_error_m=float(np.linalg.norm(evaluated[6:])),
                           covariance_diagonal=covariance.diagonal().tolist(),
                           information_min_eigenvalue=float(eigenvalues[0]),
                           information_max_eigenvalue=float(eigenvalues[-1]), cache_error=cache_error)
            if phase == 'before':
                previous = active.get((edge['from_id'], edge['to_id']))
                if previous is not None:
                    details['previous_after_residual_change'] = float(np.linalg.norm(evaluated - previous))
            else:
                active[(edge['from_id'], edge['to_id'])] = evaluated
            report['edges'].append(details)
        report['newest_edge_cost_fraction'] = report['newest_edge_cost'] / max(report['evaluated_total'], 1e-30)
        reports.append(report)
    before = [r for r in reports if r['phase'] == 'before']
    after = [r for r in reports if r['phase'] == 'after']
    all_before = [dict(latest_keyframe_id=r['latest_keyframe_id'], **e) for r in before for e in r['edges']]
    summary = dict(before_records=len(before), after_records=len(after),
                   before_edges=len(all_before),
                   after_stale_cache_records=sum(r['max_cache_error'] > 1e-7 for r in after),
                   max_after_cache_error=max((r['max_cache_error'] for r in after), default=None),
                   newest_edge_over_99pct=sum(r['newest_edge_cost_fraction'] > .99 for r in before),
                   median_newest_edge_cost_fraction=float(np.median([r['newest_edge_cost_fraction'] for r in before])) if before else None,
                   max_before_position_error_m=max((e['evaluated_position_error_m'] for e in all_before), default=None),
                   max_before_velocity_error_mps=max((e['evaluated_velocity_error_mps'] for e in all_before), default=None))
    return dict(summary=summary, errors=errors, reports=reports,
                largest_before_cost_edges=sorted(all_before, key=lambda e: e['evaluated_robust_chi2'], reverse=True)[:20],
                input=str(path.resolve()), input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                limits=['An internally consistent residual is not ground-truth accuracy.',
                        'After-state evaluation does not mutate native caches or solver decisions.',
                        'Newest-edge dominance alone does not establish an incorrect tracking estimate.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('journal', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = audit(args.journal)
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(dict(summary=result['summary'], errors=result['errors'])))
    raise SystemExit(bool(result['errors']))
