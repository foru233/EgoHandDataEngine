"""Summarize completed runs, without mistaking gated output for raw tracking."""
import argparse
import csv
import json
from pathlib import Path
from audit_vio_stability import audit_rows, audit_worker, write_csv, plot_series


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    a = p.parse_args()
    table = []
    for report_path in sorted(a.root.rglob('experiment.json')):
        run = report_path.parent
        rows = [json.loads(l) for l in (run/'poses.jsonl').read_text(encoding='utf8').splitlines()]
        audit, series, pairs = audit_rows(rows, max_gap_s=.2, jump_m=.5)
        analysis = run/'analysis'; analysis.mkdir(exist_ok=True)
        (analysis/'report.json').write_text(json.dumps(audit, indent=2, ensure_ascii=False), encoding='utf8')
        write_csv(analysis/'series.csv', series); write_csv(analysis/'pairs.csv', pairs)
        plot_series(analysis/'timeline.png', series, pairs, audit['events'])
        experiment = json.loads(report_path.read_text())
        same = audit['pairs']['same_exact_epoch_consecutive_records_short_gap']
        resources = [r['resources'] for r in rows if r.get('resources')]
        table.append({'run': str(run.relative_to(a.root)), 'frames': len(rows),
            'raw_ok': audit['raw_valid_count'], 'gated_valid': audit['gated_valid_count'],
            'same_epoch_large_steps': same['investigate_jump_count'],
            'same_epoch_max_step_m': same['translation_m'].get('max'),
            'same_epoch_step_p95_m': same['translation_m'].get('p95'),
            'latency_p50_p95_ms': experiment.get('track_ms_p50_p95'),
            'error': experiment.get('error'), 'exit_code': experiment.get('worker_exit_code'),
            'rss_first_bytes': resources[0]['rss_bytes'] if resources else None,
            'rss_last_bytes': resources[-1]['rss_bytes'] if resources else None,
            'rss_max_bytes': max((r['rss_bytes'] for r in resources), default=None)})
    (a.root/'experiment_comparison.json').write_text(json.dumps(table, indent=2), encoding='utf8')
    with (a.root/'experiment_comparison.csv').open('w', newline='') as stream:
        if table:
            writer = csv.DictWriter(stream, fieldnames=list(table[0])); writer.writeheader(); writer.writerows(table)
    print(json.dumps(table, indent=2))


if __name__ == '__main__':
    main()
