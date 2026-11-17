"""Separate native loss/recovery from application readmission in sealed recordings."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.vio.short_recovery import geometry_matches_frame, visual_reference
from run_public_recovery_acceptance import sha


def raw(row):
    return {**row, 'pose_valid': row.get('raw_pose_valid') is True, 'matrix': row.get('raw_matrix')}


def point(index, rows):
    if index is None:
        return None
    row = rows[index]
    return dict(index=index, timestamp_ns=row['timestamp_ns'], frame_id=row.get('frame_id'),
        reference=visual_reference(row), map_id=row.get('map_id'), reset_id=row.get('reset_id'),
        segment_id=(row.get('local_motion') or {}).get('segment_id'))


def audit(run, journal):
    validation = json.loads((run / 'validation.json').read_text(encoding='utf8'))
    assert validation['status'] == 'completed' and validation['input_integrity_passed'] and validation['source_integrity_passed']
    path = run / 'replay/poses.jsonl'
    rows = [json.loads(line) for line in path.open(encoding='utf8')]
    assert len(rows) == validation['pose_frames']
    assert all(b['timestamp_ns'] > a['timestamp_ns'] for a, b in zip(rows, rows[1:]))
    starts = [i for i in range(1, len(rows)) if rows[i-1]['tracking_state'] == 'OK' and rows[i]['tracking_state'] != 'OK']
    episodes = []
    def qualified(i):
        r = raw(rows[i])
        return geometry_matches_frame(r, r.get('recovery_observation'), visual_reference(r))
    for position, start in enumerate(starts):
        end = starts[position+1] if position+1 < len(starts) else len(rows)
        recovered = next((i for i in range(start, end) if rows[i]['tracking_state']=='OK' and rows[i].get('raw_pose_valid') is True), None)
        geometry = next((i for i in range(start, end) if qualified(i)), None)
        admitted = next((i for i in range(start, end) if rows[i].get('pose_valid') is True), None)
        prior = next((i for i in range(start-1, -1, -1) if rows[i].get('pose_valid') is True), None)
        lost_rows = rows[start:recovered if recovered is not None else end]
        prediction = []
        stale_prediction_records = []
        for r in lost_rows:
            o = r.get('recovery_observation') or {}
            if o.get('prediction_attempted') is not True:
                continue
            if o.get('frame_id') != r.get('frame_id') or o.get('timestamp_ns') != r['timestamp_ns']:
                stale_prediction_records.append(r.get('frame_id'))
            else:
                prediction.append(o)
        def gap(first, last):
            return (rows[last]['timestamp_ns']-rows[first]['timestamp_ns'])/1e9 if first is not None and last is not None else None
        episodes.append(dict(loss=point(start, rows), native_recovery=point(recovered, rows),
            first_qualified_geometry=point(geometry, rows), application_readmission=point(admitted, rows),
            prior_admitted=point(prior, rows),
            same_reference_readmission=(visual_reference(rows[prior])==visual_reference(rows[admitted])) if prior is not None and admitted is not None else None,
            loss_to_native_s=gap(start, recovered), loss_to_application_s=gap(start, admitted),
            native_to_application_s=gap(recovered, admitted), geometry_to_application_s=gap(geometry, admitted),
            native_loss_prediction_attempts=len(prediction),
            native_loss_prediction_successes=sum(o.get('prediction_succeeded') is True for o in prediction),
            native_loss_prediction_max_duration_s=max((o.get('prediction_duration_s', 0) for o in prediction), default=0),
            stale_prediction_records=stale_prediction_records,
            quality_rejection_reasons=dict(Counter((r.get('quality_gate') or {}).get('reason_code') for r in rows[start:end] if not r.get('pose_valid'))),
            quarantine_rejection_reasons=dict(Counter((r.get('recovery_gate') or {}).get('reason_code') for r in rows[start:end] if not r.get('pose_valid') and r.get('recovery_gate')))))
    native = [json.loads(line) for line in journal.open(encoding='utf8')]
    assert [r['sequence'] for r in native] == list(range(1, len(native)+1))
    phases = Counter(r.get('phase') for r in native if r.get('type')=='frame_stage')
    used = [r for r in native if r.get('phase')=='local_inertial_keyframe_marginal']
    return dict(run=str(run.resolve()), frames=len(rows), loss_episodes=episodes,
        native_ok_host_rejected=sum(r['tracking_state']=='OK' and not r.get('pose_valid') for r in rows),
        jumps=[point(i, rows) for i, r in enumerate(rows) if r.get('jump_detected') is True],
        native_record_counts=dict(Counter(r.get('type') for r in native)), native_phase_counts=dict(phases),
        actual_keyframe_marginal_uses=len(used), marginal_use_frame_ids=[r['frame_id'] for r in used],
        protection_input_source_verified=True,
        hashes={str(p.resolve()): sha(p) for p in (path, journal, run/'validation.json', run/'processing_inputs.json', Path(__file__))},
        production_deployed=False, acceptance=False,
        limits=['No external VS030 GT: cannot establish absolute drift, correct old-map reentry or physical hand accuracy.',
            'Loss episodes end before subsequent loss starts; no admission from a later episode is assigned to an earlier recovery.',
            'First qualified geometry is verified native evidence, not an independently measured time when images became clear.',
            'Frame-bound native IMU prediction is counted only while native tracking is unavailable; normal tracking predictions excluded.',
            'Separate executions vary with asynchronous scheduling; more admits do not establish causal improvement.',
            'Full worker/archive contract; not rendered customer GUI verification.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--journal', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run, args.journal)
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('frames','native_ok_host_rejected','actual_keyframe_marginal_uses','loss_episodes')}))


if __name__ == '__main__':
    main()
