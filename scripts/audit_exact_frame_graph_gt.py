"""Post-run GT diagnosis of exact keyframe graphs; no estimator changes or fits."""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from audit_exact_native_frame_graph import matrix, sha
from audit_public_gate_gt import associate_truth
from audit_recovery_prediction_gt import relative_motion_error


def body(state, calibration):
    return np.linalg.inv(matrix(state['Tcw'])) @ matrix(calibration['Tcb'])


def propagated_position(keyframe, calibration, delta):
    pose = body(keyframe, calibration)
    bias = np.asarray(keyframe['bias']) - np.asarray(delta['original_bias'])
    dp = matrix(delta['dP']).ravel() + matrix(delta['JPg']) @ bias[:3] + matrix(delta['JPa']) @ bias[3:]
    dt = delta['dt']
    return pose[:3, 3] + matrix(keyframe['velocity']).ravel() * dt + .5 * dt**2 * np.array([0, 0, -9.81]) + pose[:3, :3] @ dp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    plan = json.loads((args.run / 'plan.json').read_text())
    sealed = json.loads((args.run / 'run_summary.json').read_text())
    assert sealed['worker_exit_code'] == 0
    assert sealed['frames_sha256'] == sha(args.run / 'frames.jsonl')
    rows = [json.loads(line) for line in (args.run / 'frames.jsonl').open()]
    by_id = {row['raw']['frame_id']: row for row in rows}
    loader = root / 'handoff/goal_delivery_20261113/benchmark_public_vio.py'
    spec = importlib.util.spec_from_file_location('exact_graph_truth', loader)
    public = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(public)
    times, truth = public.load_gt(plan['sequence'])
    snapshots = [json.loads(path.read_text()) for path in sorted(
        (args.run / 'exact_native_graph').glob('*_before.json'), key=lambda p: int(p.name.split('_')[1]))]
    previous_kf = {}
    samples = []
    paths = [args.run / 'plan.json', args.run / 'run_summary.json', args.run / 'frames.jsonl',
             *sorted((args.run / 'exact_native_graph').glob('*.json'))]
    hashes = {str(path): sha(path) for path in paths}
    for graph in snapshots:
        frame = int(graph['frame_id'])
        expected = list((args.run / 'exact_native_graph').glob(f'frame_{frame}_local_*.json'))
        if len(expected) != 1 or 'keyframe' not in expected[0].name:
            previous_kf[graph['keyframe']['id']] = graph
            continue
        previous = graph['previous']
        if not previous['pose_valid']:
            continue
        raw = by_id[frame]['raw']
        offset = raw['timestamp_ns'] - round(graph['current']['timestamp'] * 1e9)
        stamp = lambda state: offset + round(state['timestamp'] * 1e9)
        gt_previous = associate_truth(stamp(previous), times, truth)
        gt_now = associate_truth(stamp(graph['current']), times, truth)
        gt_kf = associate_truth(stamp(graph['keyframe']), times, truth)
        if any(value is None for value in (gt_previous, gt_now, gt_kf)):
            continue
        previous_body = body(previous, graph['imu_calibration'])
        entry = dict(frame=frame, jump_rejected=bool(by_id[frame]['result'].get('jump_detected')),
                     keyframe_id=graph['keyframe']['id'], map_change_index=graph['map_change_index'])
        for mode in ('native', 'visual'):
            path = args.audit / f'frame_{frame}_{mode}.json'
            report = json.loads(path.read_text())
            estimate = body(report['replayed'], graph['imu_calibration'])
            translation, angle = relative_motion_error(previous_body, estimate, gt_previous, gt_now)
            entry[mode] = dict(relative_translation_error_m=translation, relative_rotation_error_deg=angle)
        keyframe = graph['keyframe']
        kf_body = body(keyframe, keyframe['imu_calibration'])
        truth_before = associate_truth(stamp(keyframe) - 10_000_000, times, truth)
        truth_after = associate_truth(stamp(keyframe) + 10_000_000, times, truth)
        if truth_before is not None and truth_after is not None:
            truth_velocity = (truth_after[:3, 3] - truth_before[:3, 3]) / .02
            aligned_velocity = kf_body[:3, :3] @ gt_kf[:3, :3].T @ truth_velocity
            entry['keyframe_velocity_gt_local_m_s'] = aligned_velocity.tolist()
            entry['keyframe_velocity_error_m_s'] = float(np.linalg.norm(matrix(keyframe['velocity']).ravel() - aligned_velocity))
        old = previous_kf.get(keyframe['id'])
        if old is not None and old['map_change_index'] != graph['map_change_index']:
            old_kf = old['keyframe']
            # Sequential substitution on the SAME captured preintegration. This
            # is a numerical attribution, not an alternative state estimator.
            trial = dict(old_kf)
            prior_prediction = propagated_position(trial, keyframe['imu_calibration'], graph['keyframe_delta'])
            components = {}
            for key in ('Tcw', 'velocity', 'bias'):
                trial[key] = keyframe[key]
                prediction = propagated_position(trial, keyframe['imu_calibration'], graph['keyframe_delta'])
                difference = prediction - prior_prediction
                components[key] = dict(delta_m=difference.tolist(), norm_m=float(np.linalg.norm(difference)))
                prior_prediction = prediction
            entry['keyframe_update_attribution'] = dict(previous_capture_frame=old['frame_id'],
                previous_map_change_index=old['map_change_index'], components=components,
                previous_keyframe_velocity=matrix(old_kf['velocity']).ravel().tolist(),
                updated_keyframe_velocity=matrix(keyframe['velocity']).ravel().tolist())
        samples.append(entry)
        previous_kf[keyframe['id']] = graph
    assert hashes == {str(path): sha(path) for path in paths}
    report = dict(samples=samples, input_hashes=hashes, scorer_sha256=sha(Path(__file__)),
        gt_sha256=sha(public.paths(plan['sequence'])[3]), production_deployed=False, acceptance=False,
        limits=['Post-run relative-motion diagnosis anchored only to the previous internal pose; no post-loss fit',
                'KF velocity truth uses 20ms central pose differences and local orientation alignment',
                'KF substitution only attributes propagation changes; nonrigid BA forbids blindly reusing old states'])
    with args.output.open('x', encoding='utf8') as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps(dict(samples=len(samples), jumps=[row for row in samples if row['jump_rejected']])))


if __name__ == '__main__':
    main()
