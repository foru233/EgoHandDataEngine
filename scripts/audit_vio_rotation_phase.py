"""Offline relative stereo rotation / gyro consistency, not a physical clock calibration.

Uses already recorded stereo-only poses. Never starts SLAM or changes source inputs.
Diagnostic integration linearly interpolates gyro at midpoint quadrature nodes; this
does not add samples to the runtime VIO stream or certify exposure timestamps.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.storage.sdk_recording import iter_imu_packets
from egohand.vio.imu_lattice import PacketLatticeImuBuffer


def stats(values):
    values = np.asarray(values)
    return {'count': len(values), 'median_deg': float(np.median(values)),
            'p95_deg': float(np.percentile(values, 95)), 'rms_deg': float(np.sqrt(np.mean(values**2))),
            'max_deg': float(values.max())} if len(values) else {'count': 0}


def pairs(rows, step):
    selected = []
    for i in range(0, len(rows) - step):
        group = rows[i:i + step + 1]
        if any(not r.get('raw_pose_valid', r.get('pose_valid')) or r.get('raw_matrix') is None
               or r.get('tracking_state') != 'OK' or r.get('snapshot_consistent') is False
               or int(r.get('tracked_inliers', 30)) < 30 for r in group):
            continue
        if len({r.get('map_epoch') for r in group}) != 1:
            continue
        a, b = group[0], group[-1]
        dt = (b['timestamp_ns'] - a['timestamp_ns']) / 1e9
        if not 0 < dt <= .2:
            continue
        selected.append((a['timestamp_ns'], b['timestamp_ns'],
                         np.asarray(a['raw_matrix'])[:3, :3], np.asarray(b['raw_matrix'])[:3, :3]))
    return selected


def evaluate(samples, times, gyro, camera_from_body, bias=None, gyro_sign=1):
    residuals, vectors = [], []
    bias = np.zeros(3) if bias is None else np.asarray(bias)
    for t0, t1, R0, R1 in samples:
        if t0 < times[0] or t1 > times[-1]:
            continue
        duration = (t1 - t0) / 1e9
        count = max(1, round(duration * 480))
        nodes = t0 + (np.arange(count) + .5) * ((t1 - t0) / count)
        omega = np.stack([np.interp(nodes, times, gyro[:, k]) for k in range(3)], axis=1)
        body0 = R0 @ camera_from_body
        body1 = R1 @ camera_from_body
        visual = body0.T @ body1
        visual_velocity = Rotation.from_matrix(visual).as_rotvec() / duration
        vectors.append((omega.mean(0) * gyro_sign - visual_velocity).tolist())
        predicted = np.eye(3)
        for velocity in omega * gyro_sign - bias:
            predicted = predicted @ cv2.Rodrigues(velocity * (duration / count))[0]
        residuals.append(float(np.degrees(Rotation.from_matrix(predicted.T @ visual).magnitude())))
    return stats(residuals), np.asarray(vectors)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording', required=True, type=Path)
    p.add_argument('--poses', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--phase-min-ms', type=int, default=57)
    p.add_argument('--phase-max-ms', type=int, default=137)
    p.add_argument('--phase-step-ms', type=int, default=5)
    p.add_argument('--report-name', default='rotation_phase.json')
    args = p.parse_args()
    rows = [json.loads(line) for line in args.poses.read_text(encoding='utf8').splitlines() if line.strip()]
    config = yaml.safe_load((args.recording / 'kalibr_camchain.yaml').read_text(encoding='utf8'))
    camera_from_body = np.asarray(config['cam0']['T_cam_imu'])[:3, :3]
    buffer = PacketLatticeImuBuffer(sample_rate_hz=480, camera_to_imu_drift_ppm=0,
                                  phase_ns=0, capacity=100000)
    for packet in iter_imu_packets(args.recording):
        buffer.add(packet.payload, packet.device_ts)
    base_times = np.array([row[0] for row in buffer.rows], dtype=np.int64)
    gyro = np.array([row[4:7] for row in buffer.rows])
    if args.phase_step_ms <= 0 or args.phase_min_ms > args.phase_max_ms:
        p.error('Invalid phase grid')
    phase_grid = list(range(args.phase_min_ms, args.phase_max_ms + 1, args.phase_step_ms))
    sweep = []
    # Compare precisely the same visual pairs at every phase, including boundaries.
    common_start = base_times[0] + max(phase_grid) * 1_000_000
    common_end = base_times[-1] + min(phase_grid) * 1_000_000
    prepared = {step: [sample for sample in pairs(rows, step)
                       if sample[0] >= common_start and sample[1] <= common_end] for step in [1, 4]}
    for phase in phase_grid:
        times = base_times + phase * 1_000_000
        item = {'phase_ms': phase, 'windows': {}}
        for step, samples in prepared.items():
            middle = len(samples) // 2
            train, holdout = samples[:middle], samples[middle:]
            raw_all, _ = evaluate(samples, times, gyro, camera_from_body)
            raw_train, bias_vectors = evaluate(train, times, gyro, camera_from_body)
            bias = np.median(bias_vectors, axis=0)
            raw_holdout, _ = evaluate(holdout, times, gyro, camera_from_body)
            bias_holdout, _ = evaluate(holdout, times, gyro, camera_from_body, bias)
            item['windows'][str(step)] = {'all_no_bias': raw_all,
                'train_no_bias': raw_train,
                'holdout_no_bias': raw_holdout, 'holdout_train_bias': bias_holdout,
                'train_median_bias_candidate_rps': bias.tolist(),
                'note': 'Bias fitted only on chronological first half; evaluated on second half.'}
        sweep.append(item)
        print(json.dumps({'phase_ms': phase, 'one_frame': item['windows']['1']['all_no_bias'],
                          'four_frames': item['windows']['4']['all_no_bias']}), flush=True)
    conventions = {}
    times = base_times + 97_000_000
    for name, transform, sign in [('calibrated_T_cam0_imu', camera_from_body, 1),
                                 ('wrong_transpose_control', camera_from_body.T, 1),
                                 ('identity_axes_control', np.eye(3), 1),
                                 ('negative_gyro_control', camera_from_body, -1)]:
        conventions[name], _ = evaluate(prepared[4], times, gyro, transform, gyro_sign=sign)
    grid_summary = {}
    for step in prepared:
        data = {}
        for metric in ['all_no_bias', 'train_no_bias', 'holdout_no_bias', 'holdout_train_bias']:
            best = min(sweep, key=lambda x: x['windows'][str(step)][metric]['rms_deg'])
            threshold = best['windows'][str(step)][metric]['rms_deg'] * 1.05
            near = [x['phase_ms'] for x in sweep if x['windows'][str(step)][metric]['rms_deg'] <= threshold]
            data[metric] = {'best_phase_ms': best['phase_ms'], 'residual': best['windows'][str(step)][metric],
                'within_5_percent_of_best_rms_phase_ms': [min(near), max(near)],
                'best_is_grid_boundary': best['phase_ms'] in [min(phase_grid), max(phase_grid)]}
        grid_summary[str(step)] = data
    report = {'source_recording': str(args.recording.resolve()), 'source_poses': str(args.poses.resolve()),
        'poses_sha256': hashlib.sha256(args.poses.read_bytes()).hexdigest(),
        'calibration_sha256': hashlib.sha256((args.recording / 'kalibr_camchain.yaml').read_bytes()).hexdigest(),
        'rotation_convention': 'GUI Twc0 raw poses: R_world_body = R_world_cam0 * R_cam0_imu. '
            'Gyro right-integrates R_body0_body1; relative visual = Rwb0.T * Rwb1.',
        'visual_pair_criteria': 'All intervening records raw-valid, OK, snapshot consistent, >=30 inliers, same full epoch, dt<=.2s.',
        'samples': {str(k): len(v) for k, v in prepared.items()},
        'common_visual_coverage_ns': [int(common_start), int(common_end)],
        'phase_grid_ms': phase_grid, 'grid_summary': grid_summary,
        'interval_note': 'Within-5%-of-best ranges describe this finite grid objective, not a 95% confidence interval or physical timestamp accuracy.',
        'bias_policy': 'No runtime bias is available from pure stereo. Compare zero bias and a first-half median log-velocity fit; this candidate is not an independent sensor calibration.',
        'sweep': sweep, 'convention_controls_phase_97ms': conventions,
        'limitations': ['Stereo rotations are estimates, not independent truth.',
            'SLAM visual pose optimization timing, dynamic motion, rolling exposure and gyro noise can shift the apparent optimum.',
            'No absolute exposure/clock semantics or physical extrinsic convention is proved by this consistency score.',
            'Diagnostic interpolation is only numerical quadrature; runtime IMU gap handling is unchanged.']}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / args.report_name).write_text(json.dumps(report, indent=2), encoding='utf8')
    best = {str(k): min(sweep, key=lambda x: x['windows'][str(k)]['holdout_no_bias']['rms_deg'])['phase_ms'] for k in prepared}
    print(json.dumps({'best_holdout_no_bias_phase_ms': best, 'convention_controls': conventions}), flush=True)


if __name__ == '__main__':
    main()
