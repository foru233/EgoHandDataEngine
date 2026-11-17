"""Summarize completed same-DLL KF-uncertainty hand/VIO recording pairs."""
import argparse
from collections import Counter
import json
from pathlib import Path

from audit_recorded_recovery_events import audit
from compare_recorded_fusion_runs import compare
from run_public_recovery_acceptance import save, sha


def lifecycle(journal):
    before = {}
    publications = {}
    uses = []
    errors = []
    rows = [json.loads(line) for line in journal.open(encoding='utf8')]
    for r in rows:
        if r.get('type') == 'keyframe_marginal':
            publications[(r['map_id'], r['map_change_index'], r['keyframe_id'])] = r
        if r.get('type') != 'frame_stage':
            continue
        if r['phase'] == 'local_before':
            before[r['frame_id']] = r
        if r['phase'] != 'local_inertial_keyframe_marginal':
            continue
        failures = []
        p = before.get(r['frame_id'])
        pub = publications.get((r['map_id'], r['map_change_index'], r['last_keyframe']['id']))
        if not pub or not pub['valid']:
            failures.append('valid_publication_not_observed')
        if not r['imu_initialized'] or not r['current']['has_inertial_prior'] or r['previous']['has_inertial_prior']:
            failures.append('initialization_or_prior_lifecycle_failed')
        dt = r['current']['timestamp_sec'] - r['last_keyframe']['timestamp_sec']
        if not 0 < dt <= 2 or abs(dt-r['keyframe_preintegration']['dt_s']) > .005:
            failures.append('integration_interval_invalid')
        if not p:
            failures.append('before_missing')
        else:
            if p['current']['has_inertial_prior'] or p['last_keyframe'] != r['last_keyframe']:
                failures.append('previous_current_prior_or_keyframe_mutation')
            if any(p[k] != r[k] for k in ('map_id','map_change_index','revision','reference_generation','reference_operation')):
                failures.append('reference_changed_inside_optimizer')
        uses.append(dict(frame_id=r['frame_id'], before_ba2=not r['ba2'], errors=failures))
        errors.extend(dict(frame_id=r['frame_id'], error=e) for e in failures)
    return dict(uses=len(uses), uses_before_ba2=sum(r['before_ba2'] for r in uses), errors=errors,
        publications=dict(Counter('valid' if r['valid'] else 'invalid' for r in rows if r.get('type')=='keyframe_marginal')),
        journal_sha256=sha(journal))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    outcomes = json.loads((args.suite/'suite_outcomes.json').read_text(encoding='utf8'))
    assert len(outcomes['outcomes']) == 6
    plan = json.loads((args.suite/'suite_plan.json').read_text(encoding='utf8'))
    assert plan['experiment'] == 'keyframe_uncertainty'
    assert all(sha(Path(p)) == h for p, h in plan['hashes'].items())
    assert all(r['exit_code'] == 0 and r.get('world_hand_audit',{}).get('exit_code') == 0 for r in outcomes['outcomes'])
    pairs = {}
    errors = []
    for tag in ('184714','111807','111833'):
        pair = compare(args.suite/(tag+'_control'), args.suite/(tag+'_candidate'))
        pair['recovery'] = {}
        pair['native_lifecycle'] = {}
        for arm in ('control','candidate'):
            journal = args.suite/(tag+'_'+arm+'_native_commits.jsonl')
            r = audit(args.suite/(tag+'_'+arm), journal)
            pair['recovery'][arm] = r
            life = lifecycle(journal)
            pair['native_lifecycle'][arm] = life
            errors.extend(dict(recording=tag, arm=arm, **e) for e in life['errors'])
            if arm == 'control' and life['uses']:
                errors.append(dict(recording=tag, arm=arm, error='disabled_control_used_marginal'))
            if any(e['stale_prediction_records'] for e in r['loss_episodes']):
                errors.append(dict(recording=tag, arm=arm, error='stale_loss_prediction_records'))
            episodes = r['loss_episodes']
            if episodes:
                first = episodes[0]['loss']
                returned = next((e['application_readmission'] for e in episodes if e['application_readmission']), None)
                r['first_loss_through_repeated_initialization_to_application_s'] = (
                    (returned['timestamp_ns']-first['timestamp_ns'])/1e9 if returned else None)
        if not pair['same_input_configuration_library_root_sources'] or not pair['camera_hand_numeric_and_identity_preserved']:
            errors.append(dict(recording=tag, error='pair_input_or_camera_hand_preservation_failed'))
        pairs[tag] = pair
        print(json.dumps(dict(recording=tag, camera_hands_equal=pair['camera_hand_numeric_and_identity_preserved'],
            control_marginal_uses=pair['native_lifecycle']['control']['uses'],
            candidate_marginal_uses=pair['native_lifecycle']['candidate']['uses'])), flush=True)
    save(args.output, dict(pairs=pairs, errors=errors, production_deployed=False, acceptance=False,
        all_six_replays_and_hand_audits_exit0=True,
        source_integrity=True, script_sha256=sha(Path(__file__)),
        limits=['Recorded contracts only; no VS030 external GT or rendered GUI acceptance.',
            'No cross-map stitching or native/IMU prediction substitution for current visual pose.',
            'Native asynchronous scheduling differs; admission totals alone do not establish recovery benefit.',
            'More local quality or fewer jumps is not statistical safety noninferiority.']))
    raise SystemExit(bool(errors))


if __name__ == '__main__':
    main()
