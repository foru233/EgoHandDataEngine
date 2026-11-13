"""Summarize recorded admission reasons and host IPC timings, without replaying data.

This is descriptive evidence for one run, not an accuracy or recovery-rate test.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def summarize(poses, runtime):
    reasons, tracking, quarantine, releases = Counter(), Counter(), Counter(), Counter()
    count = admitted = 0
    first_stamp = first_admitted = None
    episodes = []
    episode = None
    had_admitted = False

    def event(row):
        return {'timestamp_ns': row.get('timestamp_ns'), 'frame_id': row.get('frame_id'),
                'map_epoch': row.get('map_epoch'),
                'world_frame_revision': row.get('world_frame_revision'),
                'reference_generation': row.get('reference_generation')}

    for row in poses:
        count += 1
        stamp = row.get('timestamp_ns')
        if first_stamp is None and type(stamp) is int:
            first_stamp = stamp
        tracking[str(row.get('tracking_state', 'missing'))] += 1
        gate = row.get('quality_gate') or {}
        reasons[str(gate.get('reason_code', 'missing'))] += 1
        pre = row.get('recovery_preaccept') or {}
        if pre.get('await_visual_requalification') is True:
            quarantine[str(pre.get('quarantine_cause') or 'unspecified')] += 1
        release = pre.get('visual_release')
        if isinstance(release, dict):
            releases[str(release.get('status', 'missing'))] += 1
        lost = row.get('tracking_state') in ('LOST', 'RECENTLY_LOST')
        if episode is None and had_admitted and lost:
            episode = {'loss': event(row), 'first_tracking_ok': None,
                       'first_qualified_visual_release': None, 'first_admitted': None,
                       'quality_reasons_before_admission': Counter(),
                       'lost_frames_after_first_tracking_ok': 0,
                       'identity_transitions': [], 'censored': True}
            episodes.append(episode)
        if episode is not None:
            if episode['first_tracking_ok'] is None and row.get('tracking_state') == 'OK':
                episode['first_tracking_ok'] = event(row)
            elif episode['first_tracking_ok'] is not None and lost:
                episode['lost_frames_after_first_tracking_ok'] += 1
            current = event(row)
            identity = tuple(current.get(k) for k in ('map_epoch', 'world_frame_revision', 'reference_generation'))
            transitions = episode['identity_transitions']
            if not transitions or identity != tuple(transitions[-1].get(k) for k in
                    ('map_epoch', 'world_frame_revision', 'reference_generation')):
                transitions.append(current)
            if (episode['first_qualified_visual_release'] is None and isinstance(release, dict)
                    and release.get('status') == 'qualified'):
                episode['first_qualified_visual_release'] = event(row)
            if row.get('pose_valid') is True:
                episode['first_admitted'] = event(row)
                episode['censored'] = False
                episode = None
            else:
                episode['quality_reasons_before_admission'][str(gate.get('reason_code', 'missing'))] += 1
        if row.get('pose_valid') is True:
            admitted += 1
            had_admitted = True
            if first_admitted is None and type(stamp) is int:
                first_admitted = stamp
    samples = []
    for item in runtime.get('bridge_calls', []):
        value = item.get('ipc_track_ms')
        if type(value) in (int, float) and math.isfinite(value) and value >= 0:
            samples.append(float(value))
    samples.sort()

    def percentile(p):
        if not samples:
            return None
        position = (len(samples) - 1) * p
        lo, hi = math.floor(position), math.ceil(position)
        return samples[lo] + (samples[hi] - samples[lo]) * (position - lo)

    return {'frames': count, 'admitted_frames': admitted, 'tracking': dict(tracking),
            'quality_reason_counts': dict(reasons), 'quarantined_frame_causes': dict(quarantine),
            'visual_release_status_counts': dict(releases),
            'loss_episodes_after_first_admission': episodes,
            'first_admission_recorded_seconds': ((first_admitted-first_stamp)/1e9
                if first_stamp is not None and first_admitted is not None else None),
            'ipc_track_ms': {'count': len(samples), 'mean': sum(samples)/len(samples) if samples else None,
                'p50': percentile(.50), 'p95': percentile(.95), 'p99': percentile(.99),
                'max': samples[-1] if samples else None},
            'limitations': ['IPC timing includes native tracking and communication, not isolated ORB time',
                'frame counts are not independent recovery episodes',
                'episodes use reported LOST/RECENTLY_LOST after an admitted pose, not all possible tracking failures',
                'events with different identities cannot establish same-map recovery',
                'missing fields do not demonstrate absence of events',
                'no ground truth or exposure-to-output latency is inferred']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('poses', type=Path)
    parser.add_argument('runtime', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    with args.poses.open(encoding='utf8') as stream:
        result = summarize((json.loads(line) for line in stream if line.strip()),
                           json.loads(args.runtime.read_text(encoding='utf8')))
    result['input_sha256'] = {'poses': digest(args.poses), 'runtime': digest(args.runtime)}
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
