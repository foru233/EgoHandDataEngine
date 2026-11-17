"""Separate native loss from host rejection in sealed offline replay records."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def normalize(record, index, trace=None):
    public = 'raw' in record and 'result' in record
    result = record['result'] if public else record
    raw = record['raw'] if public else record
    geometry = raw.get('recovery_observation') or {}
    if trace is not None:
        if (trace['timestamp_ns'] != raw['timestamp_ns'] or
                trace['frame_id'] != raw.get('frame_id')):
            raise ValueError('Native diagnostic trace is not bound to this frame')
        geometry = trace.get('stages') or {}
    recovery = result.get('recovery_preaccept') or {}
    quality = result.get('quality_gate') or {}
    native_valid = raw.get('pose_valid') if public else result.get('raw_pose_valid')
    admitted = result.get('pose_valid') is True
    reference = raw.get('reference_frame') or {}
    return dict(index=record.get('index', index), timestamp_ns=raw['timestamp_ns'],
        frame_id=raw.get('frame_id'), tracking_state=raw.get('tracking_state'),
        native_pose_valid=native_valid, application_admitted=admitted,
        imu_initialized=raw.get('imu_initialized'), ba2_complete=raw.get('ba2_complete'),
        prediction_attempted=geometry.get('prediction_attempted'),
        prediction_succeeded=geometry.get('prediction_succeeded'),
        prediction_duration_s=geometry.get('prediction_duration_s'),
        visual_relocalization_attempts=geometry.get('relocalization_attempts'),
        visual_relocalization_successes=geometry.get('relocalization_successes'),
        reference_matches=geometry.get('reference_matches'),
        local_map_attempts=geometry.get('local_map_attempts'),
        local_map_successes=geometry.get('local_map_successes'),
        local_map_inliers=geometry.get('local_map_inliers'),
        tracked_inliers=raw.get('tracked_inliers'),
        geometry_qualified=geometry.get('qualified'),
        geometric_inliers=geometry.get('geometric_inliers'),
        stereo_inliers=geometry.get('stereo_inliers'),
        geometry_phase=geometry.get('phase'),
        map_id=raw.get('map_id'), reset_id=raw.get('reset_id'),
        map_revision=raw.get('map_revision'), reference_generation=raw.get('reference_generation'),
        world_frame_revision=raw.get('world_frame_revision'),
        atlas_generation=reference.get('atlas_generation'),
        segment_id=(result.get('local_motion') or {}).get('segment_id'),
        quality_reason=quality.get('reason_code'),
        jump_detected=result.get('jump_detected'),
        quarantined=recovery.get('await_visual_requalification'),
        quarantine_cause=recovery.get('quarantine_cause'),
        evidence_status=recovery.get('evidence_status'),
        direct_recovery=recovery.get('direct_recovery'),
        visual_release=recovery.get('visual_release') is not None,
        loss_prediction=recovery.get('loss_prediction'),
        request_new_map=recovery.get('request_new_map'),
        native_recovered_host_rejected=bool(native_valid is True and
            raw.get('tracking_state') == 'OK' and not admitted))


def identity(row):
    return tuple(row.get(k) for k in ('map_id', 'reset_id', 'map_revision',
        'reference_generation', 'world_frame_revision', 'atlas_generation'))


def summarize(rows):
    if any(b['timestamp_ns'] <= a['timestamp_ns'] for a, b in zip(rows, rows[1:])):
        raise ValueError('Replay timestamps must be strictly increasing')
    episodes = []
    starts = [i for i, r in enumerate(rows) if r['tracking_state'] in ('LOST', 'RECENTLY_LOST')
        and (i == 0 or rows[i-1]['tracking_state'] not in ('LOST', 'RECENTLY_LOST'))]
    for n, start in enumerate(starts):
        stop = starts[n+1] if n+1 < len(starts) else len(rows)
        before = next((r for r in reversed(rows[:start]) if r['application_admitted']), None)
        native = next((r for r in rows[start:stop] if r['tracking_state'] == 'OK'
            and r['native_pose_valid'] is True), None)
        geometry = next((r for r in rows[start:stop] if r['geometry_qualified'] is True
            and r['tracking_state'] == 'OK' and r['native_pose_valid'] is True), None)
        application = next((r for r in rows[start:stop] if r['application_admitted']), None)
        waiting = [r for r in rows[start:stop] if application is None or
            r['timestamp_ns'] < application['timestamp_ns']]
        lost_rows = [r for r in rows[start:stop] if r['tracking_state'] in ('LOST', 'RECENTLY_LOST')]
        def event(row):
            if row is None:
                return None
            return {k: row[k] for k in ('index', 'timestamp_ns', 'map_id', 'reset_id',
                'reference_generation', 'world_frame_revision', 'segment_id')}
        episode = dict(loss=event(rows[start]), native_recovery=event(native),
            first_qualified_geometry=event(geometry), application_readmission=event(application),
            prior_admitted=event(before),
            same_reference_readmission=(identity(before) == identity(application)
                if before is not None and application is not None else None),
            native_to_application_s=((application['timestamp_ns']-native['timestamp_ns'])*1e-9
                if application is not None and native is not None else None),
            geometry_to_application_s=((application['timestamp_ns']-geometry['timestamp_ns'])*1e-9
                if application is not None and geometry is not None else None),
            native_prediction_attempts=sum(r['prediction_attempted'] is True for r in rows[start:stop]),
            native_prediction_successes=sum(r['prediction_succeeded'] is True for r in rows[start:stop]),
            native_loss_prediction_attempts=sum(r['prediction_attempted'] is True for r in lost_rows),
            native_loss_prediction_successes=sum(r['prediction_succeeded'] is True for r in lost_rows),
            native_loss_prediction_max_duration_s=max((r['prediction_duration_s'] or 0 for r in lost_rows), default=0),
            rejection_reasons=dict(Counter((r['quarantine_cause'] or r['quality_reason'] or 'unreported')
                for r in waiting if r['native_recovered_host_rejected'])))
        episodes.append(episode)
    windows = []
    for row in rows:
        if row['native_recovered_host_rejected']:
            if not windows or windows[-1]['last_index'] != row['index']-1:
                windows.append(dict(first_index=row['index'], last_index=row['index'],
                    first_ns=row['timestamp_ns'], last_ns=row['timestamp_ns'], reasons=Counter()))
            window = windows[-1]
            window['last_index'], window['last_ns'] = row['index'], row['timestamp_ns']
            window['reasons'][row['quarantine_cause'] or row['quality_reason'] or 'unreported'] += 1
    for window in windows:
        window['span_s'] = (window['last_ns']-window['first_ns'])*1e-9
        window['reasons'] = dict(window['reasons'])
    return dict(frames=len(rows), tracking=dict(Counter(r['tracking_state'] for r in rows)),
        application_admitted=sum(r['application_admitted'] for r in rows),
        native_ok_host_rejected=sum(r['native_recovered_host_rejected'] for r in rows),
        prediction_diagnostics_available=any(r['prediction_attempted'] is not None for r in rows),
        native_prediction_attempts=sum(r['prediction_attempted'] is True for r in rows),
        native_prediction_successes=sum(r['prediction_succeeded'] is True for r in rows),
        prediction_max_duration_s=max((r['prediction_duration_s'] or 0 for r in rows), default=0),
        jump_rejections=[r['index'] for r in rows if r['jump_detected'] is True],
        new_map_requests=[r['index'] for r in rows if r['request_new_map'] is True],
        native_ok_host_rejection_windows=windows,
        loss_episodes=episodes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('poses', type=Path)
    parser.add_argument('--native-trace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {'poses': digest(args.poses), 'script': digest(Path(__file__))}
    traces = {}
    if args.native_trace:
        hashes['native_trace'] = digest(args.native_trace)
        for line in args.native_trace.open(encoding='utf-8'):
            trace = json.loads(line)
            key = trace['timestamp_ns'], trace['frame_id']
            if key in traces:
                raise ValueError('Duplicate native trace association')
            traces[key] = trace
    rows = []
    for index, line in enumerate(args.poses.open(encoding='utf-8')):
        record = json.loads(line)
        raw = record.get('raw', record)
        trace = traces.get((raw['timestamp_ns'], raw.get('frame_id')))
        if args.native_trace and trace is None:
            raise ValueError('Missing matching native trace')
        rows.append(normalize(record, index, trace))
    if rows:
        with (args.output / 'timeline.csv').open('x', encoding='utf-8', newline='') as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0]))
            writer.writeheader()
            for row in rows:
                writer.writerow({k: json.dumps(v, ensure_ascii=False) if isinstance(v, (dict, list)) else v
                    for k, v in row.items()})
    report = dict(schema_version=1, **summarize(rows), hashes=hashes,
        limits=['Native OK and qualified geometry are observations, not independent truth.',
            'Actual image observability cannot be inferred from tracking alone.',
            'Absent native stage diagnostics remain unknown, never zero-count claims.',
            'Readmission is searched only before the next loss episode; no map stitching.',
            'Display contract fields do not certify an actual rendered GUI.'])
    assert digest(args.poses) == hashes['poses']
    if args.native_trace:
        assert digest(args.native_trace) == hashes['native_trace']
    with (args.output / 'summary.json').open('x', encoding='utf-8') as output:
        json.dump(report, output, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps({k: report[k] for k in ('frames', 'application_admitted', 'native_ok_host_rejected')}))


if __name__ == '__main__':
    main()
