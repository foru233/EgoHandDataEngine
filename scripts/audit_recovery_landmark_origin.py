"""Audit observed keyframe birth frames and frozen landmark overlap in one run.

Read-only. Last-keyframe observations are partial coverage, not a complete birth
journal. Point IDs bound creation ordering; they do not explain historical runs.
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
    paths = [args.run / n for n in ('plan.json', 'run_summary.json', 'frames.jsonl', 'native_commits.jsonl')]
    hashes = {str(p): digest(p) for p in paths}
    plan, summary = [json.loads(p.read_text(encoding='utf8')) for p in paths[:2]]
    assert summary['worker_exit_code'] == 0
    assert summary['frames_sha256'] == digest(paths[2]) and summary['plan_sha256'] == digest(paths[0])
    rows = [json.loads(line) for line in paths[2].open(encoding='utf8')]
    records = [json.loads(line) for line in paths[3].open(encoding='utf8')]
    assert [r['sequence'] for r in records] == list(range(1, len(records) + 1))
    by_stamp = {r['raw']['timestamp_ns']: r for r in rows}
    bridge = [r for r in records if r['type'] == 'bridge_frame']
    assert len(bridge) == len(rows)
    by_frame = {b['frame_id']: by_stamp[b['timestamp_ns']] for b in bridge}
    by_time = {b['native_timestamp_sec']: by_stamp[b['timestamp_ns']] for b in bridge}
    observed = {}
    for rec in records:
        if rec['type'] != 'frame_stage' or not rec.get('last_keyframe'):
            continue
        kf = rec['last_keyframe']
        key = rec['map_id'], kf['id']
        if key in observed:
            continue
        born = by_time.get(kf['timestamp_sec'])
        observed[key] = {'map_id': rec['map_id'], 'keyframe_id': kf['id'],
            'timestamp_sec': kf['timestamp_sec'], 'first_observed_frame': rec['frame_id'],
            'birth_input_index': born['index'] if born else None,
            'birth_native_tracking_state': born['raw']['tracking_state'] if born else None,
            'birth_native_pose_valid': born['raw']['pose_valid'] if born else None}
        if born:
            assert born['raw']['map_id'] == rec['map_id']
    lost_births = [r for r in observed.values() if r['birth_native_tracking_state'] in ('RECENTLY_LOST', 'LOST')]
    guard = plan['environment'].get('EGOHAND_VIO_LOST_KEYFRAME_GUARD', '0') == '1'
    if guard:
        assert not lost_births, 'Observed keyframe born on a visually lost frame despite guard'
    shadows = []
    for rec in records:
        if rec['type'] != 'frozen_map_shadow':
            continue
        row = by_frame[rec['frame_id']]
        if row['raw']['tracking_state'] != 'OK' or not row['raw']['pose_valid']:
            continue
        shadows.append({**rec, 'index': row['index'],
            'application_admitted': row['result']['pose_valid'],
            'old_landmark_overlap_available': 'point_id_overlap' in rec,
            'all_current_points_newer_than_anchor': bool(rec.get('current_frame_points', 0) and
                rec.get('current_min_point_id', 0) > rec.get('frozen_max_point_id', float('inf')))})
    report = {'sequence': plan['sequence'], 'lost_keyframe_guard_enabled': guard,
        'observed_keyframes': len(observed), 'keyframes_born_on_visually_lost_frames': lost_births,
        'birth_tracking_counts': dict(Counter(r['birth_native_tracking_state'] for r in observed.values())),
        'native_ok_shadow_frames': len(shadows),
        'native_ok_shadow_frames_using_only_new_point_ids': sum(s['all_current_points_newer_than_anchor'] for s in shadows),
        'native_ok_shadow_samples': shadows,
        'hashes': hashes, 'limits': ['Only keyframes later observed as last_keyframe are counted; this is not complete birth-event coverage.',
            'Point-ID ordering does not identify every creation call or certify a pose.',
            'Frozen-point absence is insufficient visual evidence, not an inconsistency residual.',
            'Use same-run GT for correctness and compare identical frozen inputs and native flags.']}
    assert hashes == {str(p): digest(p) for p in paths}
    report['hashes'][str(Path(__file__))] = digest(Path(__file__))
    with args.output.open('x', encoding='utf8') as output:
        json.dump(report, output, indent=2, allow_nan=False)
    print(json.dumps({k: report[k] for k in ('sequence', 'lost_keyframe_guard_enabled', 'observed_keyframes',
        'keyframes_born_on_visually_lost_frames', 'native_ok_shadow_frames_using_only_new_point_ids')}))


if __name__ == '__main__':
    main()
