"""Audit frozen landmark shadow evidence against actual application admission.

Read-only diagnostics, never runtime evidence or a recovery decision.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

from evaluate_vio_trajectory import digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.run / n for n in ('plan.json', 'run_summary.json', 'frames.jsonl',
                                   'native_commits.jsonl', 'gate_gt.json')]
    hashes = {str(p): digest(p) for p in paths}
    plan, summary = [json.loads(p.read_text(encoding='utf8')) for p in paths[:2]]
    assert summary['worker_exit_code'] == 0
    assert summary['frames_sha256'] == digest(paths[2])
    assert summary['plan_sha256'] == digest(paths[0])
    rows = [json.loads(line) for line in paths[2].open(encoding='utf8')]
    records = [json.loads(line) for line in paths[3].open(encoding='utf8')]
    assert [r['sequence'] for r in records] == list(range(1, len(records) + 1))
    by_stamp = {r['raw']['timestamp_ns']: r for r in rows}
    by_frame = {r['frame_id']: by_stamp[r['timestamp_ns']]
                for r in records if r['type'] == 'bridge_frame'}
    assert len(by_frame) == len(rows)
    gt = json.loads(paths[4].read_text(encoding='utf8'))
    labels = {r['index']: r for r in gt['samples']}
    samples = []
    for rec in records:
        if rec['type'] != 'frozen_map_shadow':
            continue
        assert rec['admission_effect'] is False
        row, anchor = by_frame[rec['frame_id']], by_frame[rec['anchor_frame_id']]
        # Every record binds a snapshot to the exact returned native frame.
        assert anchor['raw']['map_id'] == row['raw']['map_id'] == rec['map_id']
        sample = {**rec, 'index': row['index'], 'anchor_index': anchor['index'],
                  'anchor_application_admitted': anchor['result'].get('pose_valid') is True,
                  'application_admitted': row['result'].get('pose_valid') is True,
                  'native_tracking_state': row['raw']['tracking_state'],
                  'live_geometry_report_qualified': (row['raw'].get('recovery_observation') or {}).get('qualified'),
                  'gt_label': labels.get(row['index'])}
        samples.append(sample)
    assert samples, 'No frozen shadow evidence captured'
    live_comparable = [s for s in samples if s['native_tracking_state'] == 'OK']
    assert all(s['live_geometry_qualified'] == s['live_geometry_report_qualified'] for s in live_comparable), 'Shadow changed the reported live geometry'
    event = [s for s in samples if s['gt_label'] is not None]
    report = {'sequence': plan['sequence'], 'mask': plan['mask'], 'shadow_only': True,
              'live_observation_qualification_preserved': True,
              'shadow_records': len(samples),
              'anchors': sorted({(s['anchor_index'], s['anchor_application_admitted']) for s in samples}),
              'event_classifications': dict(Counter((s['gt_label']['classification'],
                  'frozen_pass' if s['frozen_geometry_qualified'] else
                  'insufficient_frozen_correspondences' if s['frozen_associated'] < 50 else
                  'frozen_geometry_failed') for s in event)),
              'event_samples': event, 'hashes': hashes,
              'limits': ['Native-OK shadow anchors are not authorized runtime admission anchors.',
                         'Retrospective host admission crosscheck does not implement native acknowledgement.',
                         'Frozen geometry may reject valid pose when landmarks disappear or are corrected.',
                         'A frozen pass alone does not prove correctness or statistical false-recovery safety.']}
    # JSON object keys must be strings, preserve the two categorical axes explicitly.
    report['event_classifications'] = [
        {'gt_classification': key[0], 'frozen_decision': key[1], 'frames': value}
        for key, value in report['event_classifications'].items()]
    assert hashes == {str(p): digest(p) for p in paths}
    report['hashes'][str(Path(__file__))] = digest(Path(__file__))
    with args.output.open('x', encoding='utf8') as output:
        json.dump(report, output, indent=2, allow_nan=False)
    print(json.dumps({k: report[k] for k in ('shadow_records', 'anchors', 'event_classifications')}))


if __name__ == '__main__':
    main()
