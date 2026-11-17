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

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'handoff/vio_gate_optimization_20261114'
V3 = BASE / 'vs030_184714_stability_matrix_20261114'  # isolated stable-window profiles
LIBRARY = Path(os.environ.get('TUMVI_RECOVERY_LIBRARY',
    str(BASE / 'tumvi_recovery_fisheye_candidate/src/egohand/vio/native/bin/egohand_vio_tumvi_fisheye_candidate.dll'))).resolve()
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
    sequence = args.sequence
    directory, settings, imu_path, _ = public.paths(sequence)
    calibration_check = public.check_calibration(sequence)
    stamps = [int(x) for x in (directory / 'timestamps_inertial.txt').read_text().split()][:args.frames]
    assert len(stamps) == args.frames and all(b > a for a, b in zip(stamps, stamps[1:]))
    assert args.mask_start + args.mask_frames + 60 < args.frames
    graph_range = getattr(args, 'exact_graph_range', None)
    if getattr(args, 'kf_covariance_prior', False) and not getattr(args, 'kf_covariance_capture', False):
        raise ValueError('KF covariance prior requires explicit native covariance capture')
    if graph_range is not None:
        first, last = graph_range
        if not (0 <= first <= last < args.frames and last - first < 256):
            raise ValueError('exact graph range must be within the run and contain at most 256 frames')
        if not args.native_diagnostics:
            raise ValueError('exact graph export requires --native-diagnostics')
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
             'RIGHT_INDEX_FIX': '1' if args.right_index_fix else '0',
             'RIGHT_WEIGHT_FIX': '1' if args.right_weight_fix else '0',
             'LOST_KEYFRAME_GUARD': '1' if args.lost_keyframe_guard else '0',
             'VISUAL_VELOCITY_SEED': '1' if args.visual_velocity_seed else '0',
             'GEOMETRIC_RETRY': '1' if args.geometric_retry else '0',
             'MERGE_OBSERVATION_FIX': '1' if args.merge_observation_fix else '0',
             'FRAME_SEARCH_SEED': '1' if args.frame_search_seed else '0',
             'KF_COVARIANCE_CAPTURE': '1' if getattr(args, 'kf_covariance_capture', False) else '0',
             'KF_COVARIANCE_PRIOR': '1' if getattr(args, 'kf_covariance_prior', False) else '0',
             'SHORT_RECOVERY': '1' if args.kind == 'candidate' else '0',
             'RECOVERY_PROTOCOL': str(PROTOCOL)}
    os.environ.update({'EGOHAND_VIO_' + key: value for key, value in flags.items()})
    # Explicit DLL prevents active_bridges.json from changing benchmark flags.
    os.environ['EGOHAND_VIO_LIBRARY'] = str(LIBRARY)
    # A/B uses identical recovery policy; only SHORT_RECOVERY differs.
    os.environ['EGOHAND_VIO_RECOVERY_CONFIG'] = str(V3 / ('recovery_' + str(args.stable_seconds).replace('.', '_') + '.yaml'))
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['EGOHAND_VIO_TRACKING_PROBE'] = str(args.output / 'tracking_stages.csv')
    if args.native_diagnostics:
        # Opt-in, run-local observation. Never inherit a stale external log path.
        os.environ['EGOHAND_VIO_COMMIT_LOG'] = str(args.output / 'native_commits.jsonl')
        diagnostics = args.output / 'native_optimization'
        diagnostics.mkdir()
        os.environ['EGOHAND_VIO_DIAGNOSTICS'] = str(diagnostics)
    if graph_range is not None:
        graph_directory = args.output / 'exact_native_graph'
        graph_directory.mkdir()
        os.environ['EGOHAND_VIO_EXACT_GRAPH_DIR'] = str(graph_directory)
        os.environ['EGOHAND_VIO_EXACT_GRAPH_FIRST'] = str(graph_range[0])
        os.environ['EGOHAND_VIO_EXACT_GRAPH_LAST'] = str(graph_range[1])
    options = yaml.safe_load((V3 / ('quality_' + str(args.stable_seconds).replace('.', '_') + '.yaml')).read_text(encoding='utf8'))
    options = {key: value for key, value in options.items() if not key.startswith(('pose_continuity', 'continuity_'))}
    sources = {str(p.relative_to(ROOT)): sha(p) for p in (ROOT / 'src/egohand').rglob('*.py')}
    inputs = public.input_hashes(sequence)
    policy = Path(os.environ.get('EGOHAND_VIO_RECOVERY_CONFIG', ROOT / 'configs/vio/recovery_experimental.yaml'))
    plan = {'sequence': sequence, 'kind': args.kind, 'stable_seconds': args.stable_seconds, 'frames': args.frames,
            'mask': [args.mask_start, args.mask_start + args.mask_frames], 'mask_seconds': args.mask_frames / 20,
            'policy': str(policy), 'policy_sha256': sha(policy), 'quality_options': options,
            'input_hashes': inputs, 'runtime_source_hashes': sources, 'runner_sha256': sha(__file__),
            'public_loader_sha256': sha(spec.origin), 'library_sha256': sha(LIBRARY),
            'environment': {k: v for k, v in os.environ.items() if k.startswith('EGOHAND_VIO_')},
            'calibration_check': calibration_check, 'first_timestamp_ns': stamps[0],
            'path': 'ReplayBridge -> live_subprocess -> RecoveryCoordinator -> LocalizationPipeline -> committed feedback',
            'gt_loaded_by_estimation': False, 'production_deployed': False,
            'timing_scope': 'wall-clock VIO IPC and host process/commit per frame; excludes image decode, IMU batching, startup and output writes; not customer GUI latency',
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
    timings = {'vio_ipc_ms': [], 'host_commit_ms': [], 'vio_and_host_ms': []}
    cursor = 0
    previous = None
    start = time.perf_counter()
    failure = None
    index = -1
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
                track_started = time.perf_counter_ns()
                raw = bridge.track(stamp, *images, batch)
                track_finished = time.perf_counter_ns()
                result = pipeline.process(raw, batch)
                commit_finished = time.perf_counter_ns()
                timing = {'vio_ipc_ms': (track_finished-track_started)*1e-6,
                          'host_commit_ms': (commit_finished-track_finished)*1e-6,
                          'vio_and_host_ms': (commit_finished-track_started)*1e-6}
                for name, duration in timing.items():
                    timings[name].append(duration)
                admitted = consumer_map_pose(result) is not None
                assert admitted == bool(result['pose_valid'])
                assert not admitted or result['tracking_state'] == 'OK'
                assert not admitted or not result['recovery_preaccept']['await_visual_requalification']
                row = {'index': index, 'masked': masked, 'raw': raw, 'result': result, 'timing': timing}
                out.write(json.dumps(row, allow_nan=False) + '\n')
                previous = stamp
                counts['frames'] += 1
                counts[result['tracking_state']] += 1
                counts['admitted'] += admitted
                counts['quarantined'] += result['recovery_preaccept']['await_visual_requalification']
                if index % 100 == 99:
                    out.flush()
                    print(json.dumps({'index': index, **counts}), flush=True)
    except Exception as error:
        failure = {'type': type(error).__name__, 'message': str(error),
                   'attempted_index': index,
                   'worker_exit_code_before_cleanup': bridge.proc.poll() if bridge.proc else None}
        raise
    finally:
        bridge.close()
        if failure is not None:
            # Preserve incomplete native runs explicitly; scorers still require
            # the separate successful run_summary and full input/source checks.
            partial = [args.output / name for name in
                       ('plan.json', 'frames.jsonl', 'native_commits.jsonl', 'worker.log')]
            save(args.output / 'run_failure.json', {
                'failure': failure, 'counts': dict(counts),
                'elapsed_s': time.perf_counter() - start,
                'worker_exit_code_after_cleanup': bridge.exit_code,
                'completed_replay': False, 'accuracy_assessable_as_complete_run': False,
                'hashes': {str(path): sha(path) for path in partial if path.exists()},
                'ground_truth_loaded_by_worker': False, 'production_deployed': False})
    assert counts['frames'] == args.frames
    assert inputs == public.input_hashes(sequence)
    assert all(sha(ROOT / name) == digest for name, digest in sources.items())
    assert sha(LIBRARY) == plan['library_sha256'] and sha(__file__) == plan['runner_sha256']
    assert sha(policy) == plan['policy_sha256'] and sha(spec.origin) == plan['public_loader_sha256']
    latency = {name: {'count': len(values), 'p50': float(np.percentile(values, 50)),
                     'p95': float(np.percentile(values, 95)), 'p99': float(np.percentile(values, 99)),
                     'max': float(max(values))} for name, values in timings.items()}
    save(args.output / 'run_summary.json', {'counts': dict(counts), 'elapsed_s': time.perf_counter() - start,
        'per_frame_latency_ms': latency, 'timing_scope': plan['timing_scope'],
        'frames_sha256': sha(args.output / 'frames.jsonl'), 'plan_sha256': sha(args.output / 'plan.json'),
        'input_and_source_integrity_passed': True, 'ground_truth_loaded_by_worker': False,
        'worker_exit_code': bridge.exit_code})
    assert bridge.exit_code == 0, f'Native worker failed after replay: {bridge.exit_code}'


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
    assert summary['worker_exit_code'] == 0, 'Cannot score a failed native worker as a completed replay'
    assert summary['frames_sha256'] == sha(args.output / 'frames.jsonl')
    assert summary['plan_sha256'] == sha(args.output / 'plan.json')
    rows = [json.loads(line) for line in (args.output / 'frames.jsonl').open()]
    gt_digest = sha(public.paths(plan['sequence'])[3])
    times, poses = public.load_gt(plan['sequence'])
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(public.paths(plan['sequence'])[1].read_text().splitlines()[1:]), Loader=Loader)
    # TUM-VI uses raw Kannala-Brandt camera1, not EuRoC's rectified pinhole.
    # T_b_c1 maps camera coordinates into body; its inverse maps body to camera.
    T_camera_from_body = np.linalg.inv(np.array(config['IMU.T_b_c1']['data']).reshape(4, 4))
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
        'estimated_body_source': 'actual admitted raw fisheye camera transform times calibrated T_camera_from_body',
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
    p.add_argument('--sequence', choices=['room1', 'room2', 'corridor4', 'room3_holdout'], default='room1')
    p.add_argument('--mask-start', type=int, default=700)
    p.add_argument('--mask-frames', type=int, choices=[0, 4, 10, 20, 40], default=0)
    p.add_argument('--frames', type=int, default=1000)
    p.add_argument('--stable-seconds', type=float, choices=[0.5, 1.0, 1.5], default=1.5)
    p.add_argument('--native-diagnostics', action='store_true',
                   help='Capture isolated native commit and optimizer observations; may affect scheduling')
    p.add_argument('--exact-graph-range', type=int, nargs=2, metavar=('FIRST', 'LAST'),
                   help='Export bounded native optimizer graphs; requires native diagnostics and a supporting DLL')
    p.add_argument('--kf-covariance-capture', action='store_true',
                   help='Isolated native experiment: extract current local BA keyframe Schur marginal')
    p.add_argument('--kf-covariance-prior', action='store_true',
                   help='Use a fresh validated KF marginal in native frame optimization; requires capture')
    p.add_argument('--right-index-fix', action='store_true',
                   help='Enable the isolated native combined-to-right observation index experiment')
    p.add_argument('--right-weight-fix', action='store_true',
                   help='Enable isolated local inertial BA right-camera octave/uncertainty correction')
    p.add_argument('--lost-keyframe-guard', action='store_true',
                   help='Isolated experiment: prevent keyframe insertion from visually lost predictions')
    p.add_argument('--visual-velocity-seed', action='store_true',
                   help='Isolated native metric body velocity initialization after visual relocalization')
    p.add_argument('--geometric-retry', action='store_true',
                   help='Retry inertial geometry failure with real visual optimization and transactional state commit')
    p.add_argument('--frame-search-seed', action='store_true',
                   help='Isolated experiment: qualified previous-frame IMU feature-search initial pose')
    p.add_argument('--merge-observation-fix', action='store_true',
                   help='Isolated bounds-checked camera-owned visual edges in MergeInertialBA')
    a = p.parse_args()
    a.output = a.output.resolve()
    (run if a.action == 'run' else score)(a)
