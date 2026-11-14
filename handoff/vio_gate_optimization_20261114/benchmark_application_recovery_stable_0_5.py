"""Public exact-input recovery evaluation through the production IPC/host path.

The run action never opens GT. Score is a separate process. All writes exclusive.
No settings, raw inputs, runtime sources, DLLs or sealed prior results are edited.
"""
import argparse
from collections import Counter
import csv
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[1]
V3 = BASE.parent / 'vio_recovery_bridge_v3_20261113'
LIBRARY = BASE.parent / 'vio_recovery_bridge_20261113/bin/egohand_vio_short_recovery_candidate.dll'
sys.path.insert(0, str(ROOT / 'src'))
spec = importlib.util.spec_from_file_location('public_data', BASE.parent / 'goal_delivery_20261113/benchmark_public_vio.py')
public = importlib.util.module_from_spec(spec)
spec.loader.exec_module(public)
save, sha = public.save, public.sha


def run(args):
    import numpy as np
    import yaml
    from egohand.vio.native_bridge import ImuMeasurement
    from egohand.vio.replay_runtime import ReplayBridge
    from egohand.vio.hybrid_quality import make_quality_gate
    from egohand.vio.localization_state import LocalizationPipeline, consumer_map_pose
    from egohand.vio.recovery_contract import PROTOCOL
    sequence = 'V1_02_medium'
    directory, settings, imu_path, _ = public.paths(sequence)
    calibration_check = public.check_calibration(sequence)
    stamps = [int(x) for x in (directory / 'timestamps_inertial.txt').read_text().split()][:args.frames]
    assert len(stamps) == args.frames and all(b > a for a, b in zip(stamps, stamps[1:]))
    assert args.mask_start + args.mask_frames + 60 < args.frames
    measurements = []
    with imu_path.open() as f:
        for row in csv.reader(line for line in f if line.strip() and not line.startswith('#')):
            measurements.append(ImuMeasurement(int(row[0]), tuple(map(float, row[4:7])), tuple(map(float, row[1:4]))))
    for key in list(os.environ):
        if key.startswith(('EGOHAND_VIO_', 'EGOHAND_TEST_')):
            os.environ.pop(key)
    flags = {'STEREO_ONLY': '0', 'GAUGE_CONSTRAINT': '1', 'CHEIRALITY_GUARD': '1',
             'WORLD_FRAME_DISABLE': '1', 'DEPTH_GATE': '0', 'POINT_CLAMP': '0',
             'ENTRY_PRUNE': '0', 'GAUGE_FIX': '0', 'DISABLE_RECOVERY': '0',
             'SHORT_RECOVERY': '1' if args.kind == 'candidate' else '0',
             'RECOVERY_PROTOCOL': str(PROTOCOL)}
    os.environ.update({'EGOHAND_VIO_' + key: value for key, value in flags.items()})
    # Explicit DLL prevents active_bridges.json from changing benchmark flags.
    os.environ['EGOHAND_VIO_LIBRARY'] = str(LIBRARY)
    if args.kind == 'candidate':
        os.environ['EGOHAND_VIO_RECOVERY_CONFIG'] = str(V3 / 'recovery_candidate.yaml')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['EGOHAND_VIO_TRACKING_PROBE'] = str(args.output / 'tracking_stages.csv')
    options = yaml.safe_load((ROOT / 'handoff/vio_gate_optimization_20261114/live_quality_stable_0_5.yaml').read_text(encoding='utf8'))
    options = {key: value for key, value in options.items() if not key.startswith(('pose_continuity', 'continuity_'))}
    sources = {str(p.relative_to(ROOT)): sha(p) for p in (ROOT / 'src/egohand').rglob('*.py')}
    inputs = public.input_hashes(sequence)
    policy = Path(os.environ.get('EGOHAND_VIO_RECOVERY_CONFIG', ROOT / 'configs/vio/recovery_experimental.yaml'))
    plan = {'sequence': sequence, 'kind': args.kind, 'frames': args.frames,
            'mask': [args.mask_start, args.mask_start + args.mask_frames], 'mask_seconds': args.mask_frames / 20,
            'policy': str(policy), 'policy_sha256': sha(policy), 'quality_options': options,
            'input_hashes': inputs, 'runtime_source_hashes': sources, 'runner_sha256': sha(__file__),
            'public_loader_sha256': sha(spec.origin), 'library_sha256': sha(LIBRARY),
            'environment': {k: v for k, v in os.environ.items() if k.startswith('EGOHAND_VIO_')},
            'calibration_check': calibration_check, 'first_timestamp_ns': stamps[0],
            'path': 'ReplayBridge -> live_subprocess -> RecoveryCoordinator -> LocalizationPipeline -> committed feedback',
            'gt_loaded_by_estimation': False, 'production_deployed': False,
            'scoring_predeclared': {'same_reference_gap_translation_limit_m': 0.15,
                'same_reference_gap_rotation_limit_deg': 5.0, 'post_reentry_horizon_s': 1.0,
                'scale': 1.0, 'time_offset_ns': 0,
                'alignment': 'same-reference gap motion from last pre-mask admitted pose; no post-loss fit',
                'independent_new_reference': 'no cross-map GT recovery claim; local motion only',
                'purpose': 'observed threshold violations, not statistical false-recovery-rate certification'}}
    save(args.output / 'plan.json', plan)
    bridge = ReplayBridge(args.output / 'worker.log', settings_path=settings,
                         mode='stereo_inertial', library_path=LIBRARY)
    counts = Counter()
    cursor = 0
    previous = None
    start = time.perf_counter()
    try:
        bridge.start()
        assert bridge.recovery_protocol == PROTOCOL
        pipeline = LocalizationPipeline('stereo_inertial', make_quality_gate('stereo_inertial', **options),
                                        recovery_commit=bridge.commit_recovery)
        with (args.output / 'frames.jsonl').open('x', encoding='utf8') as out:
            for index, (stamp, images) in enumerate(public.stereo_images(sequence, stamps)):
                assert stamp == stamps[index]
                batch = []
                while cursor < len(measurements) and measurements[cursor].timestamp_ns <= stamp:
                    if previous is not None and measurements[cursor].timestamp_ns > previous:
                        m = measurements[cursor]
                        batch.append((m.timestamp_ns, *m.acceleration_mps2, *m.angular_velocity_rps))
                    cursor += 1
                masked = args.mask_start <= index < args.mask_start + args.mask_frames
                if masked:
                    images = [np.zeros_like(im) for im in images]
                raw = bridge.track(stamp, *images, batch)
                result = pipeline.process(raw, batch)
                admitted = consumer_map_pose(result) is not None
                assert admitted == bool(result['pose_valid'])
                assert not admitted or result['tracking_state'] == 'OK'
                assert not admitted or not result['recovery_preaccept']['await_visual_requalification']
                row = {'index': index, 'masked': masked, 'raw': raw, 'result': result}
                out.write(json.dumps(row, allow_nan=False) + '\n')
                previous = stamp
                counts['frames'] += 1
                counts[result['tracking_state']] += 1
                counts['admitted'] += admitted
                counts['quarantined'] += result['recovery_preaccept']['await_visual_requalification']
                if index % 100 == 99:
                    out.flush()
                    print(json.dumps({'index': index, **counts}), flush=True)
    finally:
        bridge.close()
    assert counts['frames'] == args.frames
    assert inputs == public.input_hashes(sequence)
    assert all(sha(ROOT / name) == digest for name, digest in sources.items())
    assert sha(LIBRARY) == plan['library_sha256'] and sha(__file__) == plan['runner_sha256']
    assert sha(policy) == plan['policy_sha256'] and sha(spec.origin) == plan['public_loader_sha256']
    save(args.output / 'run_summary.json', {'counts': dict(counts), 'elapsed_s': time.perf_counter() - start,
        'frames_sha256': sha(args.output / 'frames.jsonl'), 'plan_sha256': sha(args.output / 'plan.json'),
        'input_and_source_integrity_passed': True, 'ground_truth_loaded_by_worker': False,
        'worker_exit_code': bridge.exit_code})


def reference(frame):
    ref = frame.get('reference_frame') or {}
    return tuple(frame.get(k) for k in ('map_id', 'reset_id', 'map_revision', 'reference_generation', 'world_frame_revision')) + (ref.get('atlas_generation'),)


def score(args):
    import cv2
    import numpy as np
    import yaml
    from scipy.spatial.transform import Rotation, Slerp
    plan = json.loads((args.output / 'plan.json').read_text())
    summary = json.loads((args.output / 'run_summary.json').read_text())
    assert summary['frames_sha256'] == sha(args.output / 'frames.jsonl')
    assert summary['plan_sha256'] == sha(args.output / 'plan.json')
    rows = [json.loads(line) for line in (args.output / 'frames.jsonl').open()]
    gt_digest = sha(public.paths(plan['sequence'])[3])
    times, poses = public.load_gt(plan['sequence'])
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(public.paths(plan['sequence'])[1].read_text().splitlines()[1:]), Loader=Loader)
    # ORB Settings::precomputeRectificationMaps rotates the left camera and
    # updates Tbc. Mirror that calibrated operation; the raw-camera Tbc would
    # be a frame error. Cross-check against immutable native body snapshots.
    def K(k):
        return np.array([[config[f'Camera{k}.fx'], 0, config[f'Camera{k}.cx']],
                         [0, config[f'Camera{k}.fy'], config[f'Camera{k}.cy']], [0, 0, 1]], dtype=float)
    D = [np.array([config[f'Camera{k}.{s}'] for s in ('k1', 'k2', 'p1', 'p2')], dtype=float) for k in (1, 2)]
    Tlr = np.array(config['Stereo.T_c1_c2']['data'], dtype=np.float32).reshape(4, 4)
    Trl = np.linalg.inv(Tlr).astype(float)
    size = (int(config['Camera.width']), int(config['Camera.height']))
    Rrect, *_ = cv2.stereoRectify(K(1), D[0], K(2), D[1], size, Trl[:3, :3], Trl[:3, 3],
                                 flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=size)
    T_rectified_from_raw_camera = np.eye(4)
    T_rectified_from_raw_camera[:3, :3] = Rrect
    T_camera_from_body = T_rectified_from_raw_camera @ np.linalg.inv(np.array(config['IMU.T_b_c1']['data']).reshape(4, 4))
    snapshot_calibration_errors = []
    for row in rows:
        r = row['raw']
        b = r.get('body_state_candidate')
        if r.get('pose_valid') and b is not None:
            error = float(np.max(np.abs(np.asarray(r['matrix']) @ T_camera_from_body - np.asarray(b['T_world_from_body']))))
            snapshot_calibration_errors.append(error)
    assert snapshot_calibration_errors and max(snapshot_calibration_errors) < 5e-5
    def truth(t):
        j = int(np.searchsorted(times, t))
        if j < len(times) and times[j] == t:
            return poses[j]
        if j == 0 or j == len(times) or times[j] - times[j-1] > 20_000_000 or min(t-times[j-1], times[j]-t) > 5_000_000:
            return None
        alpha = (t-times[j-1]) / (times[j]-times[j-1])
        T = np.eye(4)
        T[:3, 3] = (1-alpha)*poses[j-1, :3, 3] + alpha*poses[j, :3, 3]
        T[:3, :3] = Slerp([0, 1], Rotation.from_matrix(poses[j-1:j+1, :3, :3]))([alpha]).as_matrix()[0]
        return T
    def body(r):
        # Score the actual admitted camera measurement, transformed with the
        # verified static calibration, including pre-IMU visual bootstrap.
        return np.asarray(r['result']['matrix']) @ T_camera_from_body
    def pair_error(before, after):
        a, b = truth(before['result']['timestamp_ns']), truth(after['result']['timestamp_ns'])
        if a is None or b is None:
            return None
        error = np.linalg.inv(np.linalg.inv(a) @ b) @ (np.linalg.inv(body(before)) @ body(after))
        return {'translation_m': float(np.linalg.norm(error[:3, 3])),
                'rotation_deg': float(Rotation.from_matrix(error[:3, :3]).magnitude()*180/np.pi)}
    start, end = plan['mask']
    admitted = lambda r: r['result'].get('pose_valid') is True
    before = rows[start-1]
    first = next((r for r in rows[end:] if admitted(r)), None)
    first_ok = next((r for r in rows[end:] if r['raw']['pose_valid'] and r['raw']['tracking_state'] == 'OK'), None)
    from egohand.vio.short_recovery import geometry_matches_frame, visual_reference
    first_geometry = next((r for r in rows[end:] if geometry_matches_frame(r['raw'], r['raw'].get('recovery_observation'), visual_reference(r['raw']))), None)
    def event(r):
        if r is None:
            return None
        return {'index': r['index'], 'timestamp_ns': r['result']['timestamp_ns'], 'reference': reference(r['result']),
                'from_unmask_s': (r['result']['timestamp_ns']-rows[end]['result']['timestamp_ns'])*1e-9}
    same = first is not None and reference(before['result']) == reference(first['result'])
    post = []
    if admitted(before) and first is not None and same:
        deadline = first['result']['timestamp_ns'] + 1_000_000_000
        for r in rows[first['index'] :]:
            if r['result']['timestamp_ns'] > deadline:
                break
            if admitted(r) and reference(r['result']) == reference(before['result']):
                error = pair_error(before, r)
                if error is not None:
                    post.append({'index': r['index'], **error})
    limits = plan['scoring_predeclared']
    violations = [r for r in post if r['translation_m'] > limits['same_reference_gap_translation_limit_m'] or r['rotation_deg'] > limits['same_reference_gap_rotation_limit_deg']]
    rpe = []
    for a, b in zip(rows, rows[1:]):
        if admitted(a) and admitted(b) and reference(a['result']) == reference(b['result']):
            err = pair_error(a, b)
            if err is not None:
                rpe.append(err)
    def stats(values):
        return {'count': len(values), 'rmse': float(np.sqrt(np.mean(np.square(values)))),
                'p95': float(np.percentile(values, 95)), 'max': float(max(values))} if values else None
    observations = [(r['raw'].get('recovery_observation') or {}) for r in rows]
    loss = next((r for r in rows[start:] if r['raw']['tracking_state'] in ('RECENTLY_LOST', 'LOST')), None)
    report = {'kind': plan['kind'], 'mask': plan['mask'], 'mask_seconds': plan['mask_seconds'],
        'pre_mask_admitted': admitted(before), 'pre_mask': event(before), 'first_loss': event(loss),
        'native_recovery': event(first_ok), 'first_geometry': event(first_geometry), 'application_readmission': event(first),
        'same_reference_as_pre_mask': same,
        'first_readmission_gap_error': pair_error(before, first) if admitted(before) and first is not None and same else None,
        'same_reference_post_reentry_gap_errors': post, 'threshold_violations': violations,
        'threshold_comparison_assessable': bool(post),
        'independent_reference_readmission': first is not None and not same,
        'native_prediction_attempts': sum(o.get('prediction_attempted') is True for o in observations),
        'native_prediction_successes': sum(o.get('prediction_succeeded') is True for o in observations),
        'native_prediction_max_duration_s': max([o.get('prediction_duration_s', 0) for o in observations] + [0]),
        'consecutive_admitted_rpe_translation_m': stats([r['translation_m'] for r in rpe]),
        'consecutive_admitted_rpe_rotation_deg': stats([r['rotation_deg'] for r in rpe]),
        'new_map_requests': [r['index'] for r in rows if r['result']['recovery_preaccept'].get('request_new_map')],
        'step_rejections': [r['index'] for r in rows if r['result'].get('jump_detected')],
        'estimated_body_source': 'actual admitted rectified camera transform times calibrated T_rectified_camera_from_body',
        'camera_body_snapshot_crosscheck': {'count': len(snapshot_calibration_errors), 'max_element_error': max(snapshot_calibration_errors)},
        'scoring_predeclared': limits, 'gt_sha256': gt_digest, 'run_summary_sha256': sha(args.output / 'run_summary.json'),
        'scorer_sha256': sha(__file__), 'vs030_absolute_accuracy_verified': False,
        'statistical_false_recovery_rate_nonincrease_established': False,
        'limits': 'Finite public synthetic dropout tests; distinct new maps cannot prove original-map recovery; no post-loss alignment or reference stitching.'}
    assert sha(public.paths(plan['sequence'])[3]) == gt_digest
    save(args.output / 'recovery_gt_metrics.json', report)
    print(json.dumps({k: report[k] for k in ('mask', 'pre_mask_admitted', 'application_readmission', 'same_reference_as_pre_mask', 'first_readmission_gap_error', 'threshold_comparison_assessable')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['run', 'score'])
    p.add_argument('output', type=Path)
    p.add_argument('--kind', choices=['control', 'candidate'], default='candidate')
    p.add_argument('--mask-start', type=int, default=700)
    p.add_argument('--mask-frames', type=int, choices=[0, 4, 10, 20, 40], default=0)
    p.add_argument('--frames', type=int, default=1000)
    a = p.parse_args()
    a.output = a.output.resolve()
    (run if a.action == 'run' else score)(a)

