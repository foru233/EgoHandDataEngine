"""Join a product Qt observation with its immutable offline pose archive."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from egohand.vio.local_segments import consumer_display_pose
from run_public_recovery_acceptance import save, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    files = [args.run/n for n in ('observation_summary.json', 'qt_events.jsonl',
                                  'replay/poses.jsonl', 'replay/replay.json',
                                  'replay/gui_launch.json', 'world_hand_audit.json')]
    hashes = {str(p):sha(p) for p in files}
    summary = json.loads(files[0].read_text(encoding='utf8'))
    events = [json.loads(s) for s in files[1].open(encoding='utf8')]
    poses = [json.loads(s) for s in files[2].open(encoding='utf8')]
    replay = json.loads(files[3].read_text(encoding='utf8'))
    hands = json.loads(files[5].read_text(encoding='utf8'))
    observed = [e for e in events if e['type'] == 'pose_display']
    errors = []
    if not summary['finished'] or summary['exit_code'] or summary['errors'] or summary['last_error']:
        errors.append('gui_did_not_complete_without_error')
    if hands['errors']:
        errors.append('world_hand_archive_error')
    if [e['timestamp_ns'] for e in observed] != [r['timestamp_ns'] for r in poses]:
        errors.append('displayed_pose_timestamp_coverage_mismatch')
    if replay['native_process_exit_code'] != 0 or replay['status'] != 'completed':
        errors.append('native_replay_not_successful')
    counts = Counter()
    predictions = []
    transitions = []
    previous = None
    for i, (e, r) in enumerate(zip(observed, poses)):
        transform = consumer_display_pose(r, world_hands=True, expected_timestamp_ns=r['timestamp_ns'])
        source = e['display_source']
        counts[source] += 1
        if source == 'trusted' and transform is None:
            errors.append(dict(index=i, error='trusted_display_without_consumer_pose'))
        if source in ('imu_prediction', 'frozen', 'visual_candidate', 'origin') and transform is not None:
            errors.append(dict(index=i, error='nontrusted_display_has_world_hand_pose'))
        if source == 'imu_prediction':
            if not isinstance(e['prediction_duration_s'], (int,float)) or not 0 <= e['prediction_duration_s'] <= 2:
                errors.append(dict(index=i, error='invalid_prediction_duration'))
            predictions.append(e['prediction_duration_s'])
        if previous != source:
            transitions.append(dict(index=i, timestamp_ns=r['timestamp_ns'], source=source,
                                    native_tracking_state=r['tracking_state'], pose_valid=r['pose_valid'],
                                    map_id=r['map_id'], reset_id=r['reset_id'],
                                    quarantine=(r.get('recovery_preaccept') or {}).get('quarantined')))
        previous = source
    screenshots = summary['screenshots']
    if not all(s['saved'] and Path(s['path']).is_file() for s in screenshots):
        errors.append('screenshot_missing_or_not_saved')
    hashes.update({s['path']:sha(Path(s['path'])) for s in screenshots})
    assert all(sha(Path(p)) == h for p,h in hashes.items())
    save(args.output,dict(frames=len(poses), observed_pose_frames=len(observed), display_sources=dict(counts),
        prediction_duration_max_s=max(predictions, default=None), transitions=transitions, errors=errors,
        screenshots=screenshots, hashes=hashes, source_sha256=sha(Path(__file__)),
        valid_world_joints_checked=hands['valid_world_joints_checked'],
        max_world_hand_transform_error_m=hands['max_transform_error_m'],
        visible_heartbeat_samples=summary['visible_samples'],
        qt_max_heartbeat_gap_s=summary['max_heartbeat_gap_s'],
        production_deployed=False, accepted=False,
        limits=['Product Qt callbacks and QWidget capture; no OS automation or manual interaction acceptance.',
                'hand_has_current_pose observer field reflects diagnostic presence, not validated hand admission; ignored.',
                'Native state/admitted observer fields were null; use joined immutable pose fields.',
                'No external position truth, hardware alignment, realtime or statistical safety certification.']))
    print(json.dumps(dict(frames=len(poses), observed=len(observed), sources=dict(counts), errors=errors)))
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
