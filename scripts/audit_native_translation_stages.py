"""Join one run's native stage/commit observations to its output and public GT.

Read-only retrospective diagnosis, never a recovery-admission signal. Optimizer
logging can alter scheduling. Different runs cannot establish event causality.
"""
import argparse
from collections import Counter, defaultdict
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml

from audit_public_gate_gt import associate_truth
from audit_recovery_prediction_gt import relative_motion_error
from evaluate_vio_trajectory import digest, summary

ROOT = Path(__file__).resolve().parents[1]


def pose(value):
    if value is None:
        return None
    result = np.asarray(value, dtype=float).reshape(4, 4)
    return result if np.isfinite(result).all() else None


def same_reference(a, b):
    fields = ('map_id', 'reset_id', 'map_revision', 'reference_generation', 'world_frame_revision')
    return all(a.get(k) == b.get(k) for k in fields)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads((args.run / 'plan.json').read_text(encoding='utf8'))
    sealed = json.loads((args.run / 'run_summary.json').read_text(encoding='utf8'))
    assert digest(args.run / 'frames.jsonl') == sealed['frames_sha256']
    assert digest(args.run / 'plan.json') == sealed['plan_sha256']
    records = [json.loads(line) for line in (args.run / 'native_commits.jsonl').open(encoding='utf8')]
    assert [r['sequence'] for r in records] == list(range(1, len(records) + 1)), 'incomplete/duplicated native log'
    worker = (args.run / 'worker.log').read_text(encoding='utf8', errors='replace')
    assert not any(s in worker for s in ('DIAGNOSTICS_INCOMPLETE', 'DIAGNOSTICS_IO_FAILED',
                                        'DIAGNOSTICS_ENQUEUE_FAILED', 'CAPTURE_FAILED',
                                        'DIAGNOSTICS_SHUTDOWN_NOT_QUIESCENT'))
    rows = [json.loads(line) for line in (args.run / 'frames.jsonl').open(encoding='utf8')]
    by_stamp = {r['raw']['timestamp_ns']: r for r in rows}
    bridge = {r['frame_id']: r for r in records if r['type'] == 'bridge_frame'}
    final = {r['frame_id']: r for r in records if r['type'] == 'frame'}
    stages = defaultdict(list)
    for r in records:
        if r['type'] == 'frame_stage':
            stages[r['frame_id']].append(r)
    assert len(bridge) == len(rows) and len(stages) > 0
    spec = importlib.util.spec_from_file_location('translation_public', ROOT / 'handoff/goal_delivery_20261113/benchmark_public_vio.py')
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    _, settings, _, gt_path = public.paths(plan['sequence'])
    times, truth_poses = public.load_gt(plan['sequence'])
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(settings.read_text(encoding='utf8').splitlines()[1:]), Loader=Loader)
    camera_from_body = np.linalg.inv(np.asarray(config['IMU.T_b_c1']['data']).reshape(4, 4))
    def truth(stamp):
        return associate_truth(stamp, times, truth_poses)

    previous = None
    events, crosschecks = [], []
    phase_errors = defaultdict(list)
    for frame_id, b in sorted(bridge.items(), key=lambda item: item[1]['timestamp_ns']):
        row = by_stamp[b['timestamp_ns']]
        f = final.get(frame_id)
        current_pose = pose(row['raw'].get('matrix'))
        if f and f['pose_valid'] and current_pose is not None and b['pose_valid']:
            crosschecks.append(float(np.max(np.abs(current_pose - pose(f['world_from_camera'])))))
        event = {'index': row['index'], 'frame_id': frame_id, 'timestamp_ns': b['timestamp_ns'],
                 'jump_detected': row['result'].get('jump_detected') is True,
                 'native_tracking_state': row['raw'].get('tracking_state'),
                 'admitted': row['result'].get('pose_valid') is True,
                 'quarantine_cause': (row['result'].get('recovery_preaccept') or {}).get('quarantine_cause'),
                 'stages': []}
        previous_pose = pose(previous['row']['raw'].get('matrix')) if previous else None
        gt = truth(b['timestamp_ns'])
        previous_gt = truth(previous['bridge']['timestamp_ns']) if previous else None
        comparable = bool(previous and previous_pose is not None and same_reference(previous['row']['raw'], row['raw']))
        event['same_reference_previous_output'] = comparable
        if gt is not None and previous_gt is not None:
            event['GT_body_step_m'] = float(np.linalg.norm(gt[:3, 3] - previous_gt[:3, 3]))
        for stage in stages[frame_id]:
            estimate = pose(stage['current']['world_from_camera'])
            anchor = pose(stage['previous']['world_from_camera'])
            detail = {key: stage[key] for key in ('sequence', 'phase', 'map_id', 'map_change_index',
                      'geometry_sequence', 'geometry_active', 'reference_generation', 'map_updated',
                      'imu_initialized', 'ba2', 'short_search', 'map_matches', 'nonoutlier_matches')}
            detail['velocity'] = stage['current']['velocity']
            detail['previous_velocity'] = stage['previous']['velocity']
            detail['last_keyframe'] = stage['last_keyframe']
            detail['bias_gyro_acc'] = stage['current']['bias_gyro_acc']
            detail['frame_preintegration'] = stage['frame_preintegration']
            if estimate is not None and anchor is not None:
                detail['step_from_internal_previous_m'] = float(np.linalg.norm(estimate[:3, 3] - anchor[:3, 3]))
                # This rebased previous frame is diagnostic only, never a new
                # application anchor. It may itself contain optimizer error.
                if comparable and stage['map_id'] == row['raw'].get('map_id') and gt is not None and previous_gt is not None and stage['previous']['id'] == previous['bridge']['frame_id']:
                    te, re = relative_motion_error(anchor @ camera_from_body, estimate @ camera_from_body, previous_gt, gt)
                    detail.update(internal_relative_translation_error_m=te, internal_relative_rotation_error_deg=re)
            if estimate is not None and comparable and stage['map_id'] == row['raw'].get('map_id'):
                detail['step_from_previous_output_m'] = float(np.linalg.norm(estimate[:3, 3] - previous_pose[:3, 3]))
                if anchor is not None:
                    detail['internal_previous_rebase_from_output_m'] = float(np.linalg.norm(anchor[:3, 3] - previous_pose[:3, 3]))
                if gt is not None and previous_gt is not None:
                    te, re = relative_motion_error(previous_pose @ camera_from_body, estimate @ camera_from_body, previous_gt, gt)
                    detail.update(relative_translation_error_m=te, relative_rotation_error_deg=re)
                    phase_errors[stage['phase']].append(te)
            event['stages'].append(detail)
        if current_pose is not None and comparable:
            event['raw_camera_step_m'] = float(np.linalg.norm(current_pose[:3, 3] - previous_pose[:3, 3]))
        if event['jump_detected']:
            lower = previous['final']['sequence'] if previous and previous['final'] else b['sequence']
            upper = f['sequence'] if f else b['sequence']
            event['commits_between_frame_outputs'] = [
                {k: r.get(k) for k in ('sequence', 'type', 'kind', 'event', 'phase', 'revision', 'map_change_index')}
                for r in records if lower < r['sequence'] <= upper and r.get('map_id') == row['raw'].get('map_id')
                and r['type'] in ('local_ba', 'counter')]
            local = [r for r in records if lower < r['sequence'] <= upper and r.get('map_id') == row['raw'].get('map_id') and r['type'] == 'local_ba']
            event['local_commit_pose_changes'] = []
            for before in local:
                if before['phase'] != 'before':
                    continue
                after = next((r for r in local if r['event'] == before['event'] and r['phase'] == 'after'), None)
                if after is None:
                    continue
                old = {k['id']: pose(k['world_from_camera']) for k in before['keyframes']}
                changes = [float(np.linalg.norm(pose(k['world_from_camera'])[:3, 3] - old[k['id']][:3, 3]))
                           for k in after['keyframes'] if k['id'] in old]
                event['local_commit_pose_changes'].append({'event': before['event'], 'kind': before['kind'],
                    'keyframe_position_correction_m': summary(changes)})
            events.append(event)
        previous = {'row': row, 'bridge': b, 'final': f}
    assert crosschecks and max(crosschecks) < 1e-4, 'native frame/output pose mismatch'
    weight_records = [r for r in records if r['type'] == 'local_right_weights']
    if weight_records:
        expected_weight_fix = plan['environment'].get('EGOHAND_VIO_RIGHT_WEIGHT_FIX', '0') == '1'
        assert all(r['corrected_enabled'] == expected_weight_fix for r in weight_records)
    report = {'sequence': plan['sequence'], 'record_counts': dict(Counter(r['type'] for r in records)),
              'native_log_contiguous': True, 'snapshot_output_crosscheck_max': max(crosschecks),
              'stage_translation_errors_m': {key: summary(values) for key, values in phase_errors.items()},
              'jump_events': events,
              'right_weight_diagnostics': {
                  'observed_enabled': sorted({r['corrected_enabled'] for r in weight_records}),
                  'local_ba_calls': sum(r['type'] == 'local_right_weights' for r in records),
                  'repeated_right_edge_observations': sum(r.get('right_edges', 0) for r in records if r['type'] == 'local_right_weights'),
                  'mismatched_octaves': sum(r.get('mismatched_octaves', 0) for r in records if r['type'] == 'local_right_weights'),
                  'right_only_edges': sum(r.get('right_only_edges', 0) for r in records if r['type'] == 'local_right_weights'),
                  'legacy_information_sum': sum(r.get('legacy_information_sum', 0) for r in records if r['type'] == 'local_right_weights'),
                  'corrected_information_sum': sum(r.get('corrected_information_sum', 0) for r in records if r['type'] == 'local_right_weights'),
                  'note': 'Repeated constraints across local BA calls are not independent observations or recovery events.'},
              'hashes': {str(p): digest(p) for p in (args.run / 'plan.json', args.run / 'frames.jsonl',
                        args.run / 'native_commits.jsonl', settings, gt_path, Path(__file__))},
              'limits': ['GT only at covered timestamps. Each stage is scored against the previous returned pose; map geometry changes can move that anchor.',
                         'Consecutive relative accuracy does not establish accumulated map or recovery accuracy.',
                         'Commit journal covers selected writers only. No listed commit does not prove no map change.',
                         'Observation changes thread timing; this diagnoses this run, not causality in historical runs.',
                         'No thresholds, admission decisions, coordinates, scale or timestamps are modified.']}
    with args.output.open('x', encoding='utf8') as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps({'records': len(records), 'jumps': len(events), 'snapshot_max_error': max(crosschecks)}))


if __name__ == '__main__':
    main()
