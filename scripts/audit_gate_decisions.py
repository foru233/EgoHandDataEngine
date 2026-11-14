"""Reconstruct per-frame VIO gate decisions from an immutable poses.jsonl.

This is a descriptive audit and shadow-gate study. It never changes a replay or
authorizes a pose. Shadow stability numbers are explicitly not closed-loop A/B
results because they do not rerun the native estimator and recovery state.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def identity(row):
    return tuple(row.get(key) for key in ('map_epoch', 'world_frame_revision', 'reference_generation'))


def _event(row, index):
    return {'index': index, 'frame_id': row.get('frame_id'), 'timestamp_ns': row.get('timestamp_ns'),
            'tracking_state': row.get('tracking_state'), 'pose_valid': row.get('pose_valid'),
            'map_epoch': row.get('map_epoch'), 'world_frame_revision': row.get('world_frame_revision'),
            'reference_generation': row.get('reference_generation')}


def _geometry(row):
    observation = row.get('recovery_observation') or {}
    return {'available': observation.get('available'), 'qualified': observation.get('qualified'),
            'geometric_inliers': observation.get('geometric_inliers'),
            'stereo_inliers': observation.get('stereo_inliers'),
            'occupied_cells': observation.get('occupied_cells'),
            'associated': observation.get('associated'),
            'stereo_p95_normalized': observation.get('stereo_p95_normalized'),
            'phase': observation.get('phase')}


def _decision(row, index):
    quality = row.get('quality_gate') or {}
    recovery = row.get('recovery_gate') or {}
    pre = row.get('recovery_preaccept') or {}
    localization = row.get('localization') or {}
    reasons = []
    if quality.get('passed') is not True:
        reasons.append(str(quality.get('reason_code') or 'quality_gate_missing_or_failed'))
    if recovery.get('passed') is not True and recovery:
        reasons.append(str(recovery.get('reason_code') or 'recovery_gate_failed'))
    if pre.get('await_visual_requalification') is True:
        reasons.append('await_visual_requalification')
    if row.get('pose_valid') is not True and not reasons:
        reasons.append(str(row.get('quality_status') or 'pose_not_admitted'))
    return {'index': index, 'frame_id': row.get('frame_id'), 'timestamp_ns': row.get('timestamp_ns'),
            'session_id': (pre.get('session_id') or row.get('session_id')),
            'tracking_state': row.get('tracking_state'), 'imu_initialized': row.get('imu_initialized'),
            'ba2_complete': row.get('ba2_complete'), 'inertial_scale_finalized': row.get('inertial_scale_finalized'),
            'map_epoch': row.get('map_epoch'), 'world_frame_revision': row.get('world_frame_revision'),
            'reference_generation': row.get('reference_generation'), 'identity': identity(row),
            'raw_pose_valid': row.get('raw_pose_valid'), 'pose_valid': row.get('pose_valid'),
            'tracking_ok': row.get('tracking_state') == 'OK',
            'quality_passed': quality.get('passed'), 'quality_reason': quality.get('reason_code'),
            'recovery_quarantined': pre.get('await_visual_requalification'),
            'recovery_cause': pre.get('quarantine_cause'),
            'quarantine_elapsed_s': pre.get('quarantine_elapsed_s'),
            'visual_release_status': (pre.get('visual_release') or {}).get('status'),
            'request_new_map': pre.get('request_new_map'),
            'automatic_new_map': row.get('automatic_new_map'),
            'localization_state': localization.get('state'),
            'world_hands_permitted': (localization.get('admission') or {}).get('world_hands_permitted'),
            'geometry': _geometry(row),
            'prediction_status': (pre.get('loss_prediction') or {}).get('status'),
            'prediction_duration_s': (pre.get('loss_prediction') or {}).get('duration_s'),
            'jump_detected': row.get('jump_detected'), 'all_reject_reasons': reasons}


def _episode_timeline(decisions):
    episodes = []
    current = None
    had_admitted = False
    last_identity = None
    for decision in decisions:
        if current is None and had_admitted and decision['tracking_state'] in ('LOST', 'RECENTLY_LOST'):
            current = {'loss_start': decision, 'first_tracking_ok': None,
                       'first_geometry_valid': None, 'first_quality_passed': None,
                       'first_visual_release': None, 'first_admitted': None,
                       'first_new_map': None, 'identity_transitions': [],
                       'rejection_reasons': Counter(), 'censored': True}
            episodes.append(current)
        if current is not None:
            if current['first_tracking_ok'] is None and decision['tracking_state'] == 'OK':
                current['first_tracking_ok'] = decision
            geo = decision['geometry']
            if (current['first_geometry_valid'] is None and
                    geo.get('available') is True and geo.get('qualified') is True):
                current['first_geometry_valid'] = decision
            if current['first_quality_passed'] is None and decision['quality_passed'] is True:
                current['first_quality_passed'] = decision
            if current['first_visual_release'] is None and decision['visual_release_status'] == 'qualified':
                current['first_visual_release'] = decision
            if current['first_new_map'] is None and decision.get('automatic_new_map') is True:
                current['first_new_map'] = decision
            if not current['identity_transitions'] or decision['identity'] != current['identity_transitions'][-1]['identity']:
                current['identity_transitions'].append(decision)
            for reason in decision['all_reject_reasons']:
                current['rejection_reasons'][reason] += 1
            if decision['pose_valid'] is True:
                current['first_admitted'] = decision
                current['censored'] = False
                current = None
        if decision['pose_valid'] is True:
            had_admitted = True
        last_identity = decision['identity']
    for episode in episodes:
        episode['rejection_reasons'] = dict(episode['rejection_reasons'])
        for key in ('loss_start', 'first_tracking_ok', 'first_geometry_valid', 'first_quality_passed',
                    'first_visual_release', 'first_admitted', 'first_new_map'):
            value = episode[key]
            episode[key] = value if value is None else _event(value, value['index'])
        episode['identity_transitions'] = [_event(item, item['index']) | {'identity': item['identity']}
                                           for item in episode['identity_transitions']]
        start = episode['loss_start'].get('timestamp_ns')
        end = episode['first_admitted'].get('timestamp_ns') if episode['first_admitted'] else None
        for name in ('first_tracking_ok', 'first_geometry_valid', 'first_quality_passed', 'first_visual_release'):
            item = episode[name]
            episode[name+'_delay_ms'] = ((item['timestamp_ns'] - start) / 1e6
                                         if item and isinstance(start, int) and isinstance(item.get('timestamp_ns'), int)
                                         else None)
        episode['loss_to_admission_ms'] = ((end-start)/1e6 if isinstance(start, int) and isinstance(end, int) else None)
    return episodes


def _shadow_stability(decisions, rows, seconds):
    """Approximate only the host quality stability gate on recorded raw rows."""
    stable_ns = int(seconds * 1e9)
    previous = None
    stable_start = None
    passed = []
    for decision, row in zip(decisions, rows):
        raw_ok = row.get('raw_pose_valid') is True and row.get('tracking_state') == 'OK'
        stamp = row.get('timestamp_ns')
        if (not raw_ok or not isinstance(stamp, int)
                or (previous is not None and row.get('map_epoch') != previous.get('map_epoch'))):
            previous = None; stable_start = None; passed.append(False); continue
        if previous is None:
            stable_start = stamp
        previous = row
        passed.append(stamp - stable_start >= stable_ns)
    earlier = sum(1 for value, decision in zip(passed, decisions) if value and decision['pose_valid'] is not True)
    actual = sum(1 for decision in decisions if decision['pose_valid'] is True)
    return {'stable_seconds': seconds, 'shadow_quality_passed_frames': sum(passed),
            'baseline_application_admitted_frames': actual, 'shadow_earlier_than_baseline_frames': earlier,
            'closed_loop': False,
            'limitation': 'replays recorded raw snapshots through a host-only stability approximation; it does not rerun native VIO or recovery state'}


def analyze(path, *, shadow_seconds=(0.0, .25, .5, 1.0, 1.5, 2.0)):
    rows = [json.loads(line) for line in path.open(encoding='utf8') if line.strip()]
    decisions = [_decision(row, index) for index, row in enumerate(rows)]
    reasons = Counter(reason for decision in decisions for reason in decision['all_reject_reasons'])
    tracking = Counter(str(decision['tracking_state']) for decision in decisions)
    identities = []
    for decision in decisions:
        if not identities or decision['identity'] != identities[-1]: identities.append(decision['identity'])
    episodes = _episode_timeline(decisions)
    return {'input': str(path), 'input_sha256': sha256(path), 'frames': len(rows),
            'admitted_frames': sum(decision['pose_valid'] is True for decision in decisions),
            'raw_pose_frames': sum(decision['raw_pose_valid'] is True for decision in decisions),
            'tracking_counts': dict(tracking), 'reject_reason_counts': dict(reasons),
            'identity_transition_count': max(0, len(identities)-1), 'identities': identities,
            'recovery_episode_count': len(episodes), 'recovery_episodes': episodes,
            'shadow_stability': [_shadow_stability(decisions, rows, seconds) for seconds in shadow_seconds],
            'limitations': ['Recorded poses are evidence of the configured run, not an external accuracy ground truth',
                            'Shadow stability is not a closed-loop A/B result',
                            'Correlated frames inside one recovery episode are not independent samples']}, decisions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('poses', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--decisions', type=Path)
    args = parser.parse_args()
    report, decisions = analyze(args.poses)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf8')
    if args.decisions:
        with args.decisions.open('w', encoding='utf8') as stream:
            for decision in decisions:
                stream.write(json.dumps(decision, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({key: report[key] for key in ('input', 'frames', 'admitted_frames', 'raw_pose_frames',
        'tracking_counts', 'reject_reason_counts', 'identity_transition_count', 'recovery_episode_count',
        'shadow_stability')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
