"""Conservative metric trajectory evaluation on a specified camera timeline.

Inputs use TUM pose columns: timestamp tx ty tz qx qy qz qw.  The estimated
pose must already represent the same sensor body as GT, in one known map.
No time offset or scale is fitted. Missing frames and GT gaps remain visible.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from decimal import Decimal
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def timestamp(value, unit):
    scale = Decimal(1000000000) if unit == 's' else Decimal(1)
    result = Decimal(value) * scale
    if not result.is_finite() or result != result.to_integral_value():
        raise ValueError(f'timestamp is not integral nanoseconds: {value}')
    return int(result)


def load_timeline(path, unit):
    rows = [line.split()[0] for line in path.read_text(encoding='utf-8-sig').splitlines()
            if line.strip() and not line.lstrip().startswith('#')]
    ts = np.asarray([timestamp(x, unit) for x in rows], dtype=np.int64)
    if len(ts) < 2 or np.any(np.diff(ts) <= 0):
        raise ValueError(f'{path}: timestamps must be strictly increasing')
    return ts


def load_pose(path, unit):
    times, positions, quats = [], [], []
    for line in path.read_text(encoding='utf-8-sig').splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f'{path}: expected eight columns')
        times.append(timestamp(fields[0], unit))
        values = np.asarray(fields[1:], dtype=np.float64)
        if not np.all(np.isfinite(values)) or abs(np.linalg.norm(values[3:]) - 1) > 0.01:
            raise ValueError(f'{path}: invalid pose or quaternion')
        positions.append(values[:3])
        quats.append(values[3:] / np.linalg.norm(values[3:]))
    ts = np.asarray(times, dtype=np.int64)
    if len(ts) < 2 or np.any(np.diff(ts) <= 0):
        raise ValueError(f'{path}: pose times must be strictly increasing')
    return ts, np.asarray(positions), Rotation.from_quat(quats)


def summary(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return None
    return {'count': len(a), 'rmse': float(np.sqrt(np.mean(a * a))),
            'mean': float(np.mean(a)), 'median': float(np.median(a)),
            'p95': float(np.percentile(a, 95)), 'max': float(np.max(a))}


def evaluate(timeline, estimated, truth, max_est_dt_ns, max_gt_gap_ns, horizons, max_frame_gap_ns=100_000_000):
    et, ep, er = estimated
    gt, gp, gr = truth
    # Associate each expected camera frame with at most one estimate. Estimates
    # further than the specified tolerance remain unavailable, not interpolated.
    right = np.searchsorted(et, timeline)
    lo, hi = np.clip(right - 1, 0, len(et) - 1), np.clip(right, 0, len(et) - 1)
    nearest = np.where(np.abs(et[lo] - timeline) <= np.abs(et[hi] - timeline), lo, hi)
    has_est = np.abs(et[nearest] - timeline) <= max_est_dt_ns
    if len(np.unique(nearest[has_est])) != np.count_nonzero(has_est):
        raise ValueError('One estimated pose matched multiple camera frames; reduce tolerance')
    r = np.searchsorted(gt, timeline, side='left')
    exact = (r < len(gt)) & (gt[np.minimum(r, len(gt) - 1)] == timeline)
    left, right = np.clip(r - 1, 0, len(gt) - 1), np.clip(r, 0, len(gt) - 1)
    has_gt = exact | ((r > 0) & (r < len(gt)) & ((gt[right] - gt[left]) <= max_gt_gap_ns))
    valid = has_est & has_gt
    ids = np.flatnonzero(valid)
    if len(ids) < 3:
        raise ValueError('fewer than three camera frames have both estimate and valid GT')
    gt_pos = np.empty((len(ids), 3))
    gt_rot = []
    for k, i in enumerate(ids):
        if exact[i]:
            gt_pos[k] = gp[r[i]]
            gt_rot.append(gr[r[i]].as_quat())
        else:
            a, b = left[i], right[i]
            fraction = (int(timeline[i]) - int(gt[a])) / (int(gt[b]) - int(gt[a]))
            gt_pos[k] = (1 - fraction) * gp[a] + fraction * gp[b]
            gt_rot.append(Slerp([0, 1], gr[[a, b]])([fraction]).as_quat()[0])
    gt_rot = Rotation.from_quat(gt_rot)
    est_pos = ep[nearest[ids]]
    est_rot = er[nearest[ids]]
    x, y = est_pos.mean(axis=0), gt_pos.mean(axis=0)
    u, singular, vt = np.linalg.svd((est_pos - x).T @ (gt_pos - y))
    if singular[1] < 1e-12:
        raise ValueError('alignment is underconstrained: positions are collinear')
    d = np.eye(3); d[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ d @ u.T
    translation = y - rotation @ x
    aligned = est_pos @ rotation.T + translation
    errors = np.linalg.norm(aligned - gt_pos, axis=1)
    gap_mask = np.diff(gt) > max_gt_gap_ns
    gap_starts, gap_ends = gt[:-1][gap_mask], gt[1:][gap_mask]
    horizon_tolerance = int(np.median(np.diff(timeline)) / 2)
    rpes = {}
    for seconds in horizons:
        target_ns = int(round(seconds * 1e9))
        pos_errors, rot_errors, gt_distance, est_distance = [], [], 0.0, 0.0
        for a in range(len(ids)):
            desired = int(timeline[ids[a]]) + target_ns
            insertion = int(np.searchsorted(timeline[ids], desired))
            options = [b for b in (insertion - 1, insertion) if a < b < len(ids)]
            if not options:
                continue
            b = min(options, key=lambda j: abs(int(timeline[ids[j]]) - desired))
            # No pair may bridge a missing camera output, GT gap, or large frame gap.
            if ids[b] - ids[a] != b - a or abs(int(timeline[ids[b]]) - desired) > horizon_tolerance:
                continue
            if np.any(np.diff(timeline[ids[a]:ids[b] + 1]) > max_frame_gap_ns):
                continue
            gap = np.searchsorted(gap_ends, timeline[ids[a]], side='right')
            if gap < len(gap_starts) and gap_starts[gap] < timeline[ids[b]]:
                continue
            g_rel = gt_rot[a].inv() * gt_rot[b]
            e_rel = est_rot[a].inv() * est_rot[b]
            g_delta = gt_rot[a].inv().apply(gt_pos[b] - gt_pos[a])
            e_delta = est_rot[a].inv().apply(est_pos[b] - est_pos[a])
            pos_errors.append(np.linalg.norm(e_delta - g_delta))
            rot_errors.append(np.degrees((g_rel.inv() * e_rel).magnitude()))
            gt_distance += np.linalg.norm(g_delta)
            est_distance += np.linalg.norm(e_delta)
        rpes[str(seconds)] = {'translation_m': summary(pos_errors), 'rotation_deg': summary(rot_errors),
                               'sum_relative_distance_ratio_minus_one': (est_distance / gt_distance - 1) if gt_distance > 0 else None}
    return {'coverage': {'expected_frames': len(timeline), 'estimate_available': int(has_est.sum()),
                         'gt_available': int(has_gt.sum()), 'paired': len(ids),
                         'estimate_fraction': float(has_est.mean()), 'paired_fraction': float(valid.mean())},
            'alignment': {'method': 'SE(3) least squares on all paired positions; no scale',
                          'rotation': rotation.tolist(), 'translation_m': translation.tolist(),
                          'scale_applied': 1.0, 'singular_values': singular.tolist()},
            'ate_translation_m': summary(errors), 'rpe_by_horizon_s': rpes,
            'rpe_horizon_tolerance_ns': horizon_tolerance,
            'limitations': ['Alignment on all paired frames is retrospective and optimistic.',
                            'Map identity, tracking, recovery and GT correctness are not established by this pose-only evaluation.',
                            'No claim about accuracy during missing estimates or GT gaps.']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeline', required=True, type=Path)
    p.add_argument('--estimate', required=True, type=Path)
    p.add_argument('--gt', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--estimate-unit', required=True, choices=['s', 'ns'])
    p.add_argument('--timeline-unit', required=True, choices=['s', 'ns'])
    p.add_argument('--gt-unit', required=True, choices=['s', 'ns'])
    p.add_argument('--frame-contract', required=True, help='Document estimate/GT body frames and extrinsics used')
    p.add_argument('--map-contract', required=True, help='Document evidence estimates share a single map')
    p.add_argument('--max-est-diff-ms', type=float, default=1)
    p.add_argument('--max-gt-gap-ms', type=float, default=50)
    p.add_argument('--max-frame-gap-ms', type=float, default=100)
    p.add_argument('--horizons', type=float, nargs='+', default=[0.1, 0.5, 1.0])
    args = p.parse_args()
    if args.max_est_diff_ms <= 0 or args.max_gt_gap_ms <= 0 or args.max_frame_gap_ms <= 0 or any(h <= 0 for h in args.horizons):
        p.error('tolerances and horizons must be positive')
    result = evaluate(load_timeline(args.timeline, args.timeline_unit), load_pose(args.estimate, args.estimate_unit),
                      load_pose(args.gt, args.gt_unit), int(args.max_est_diff_ms * 1e6),
                      int(args.max_gt_gap_ms * 1e6), args.horizons, int(args.max_frame_gap_ms * 1e6))
    result['provenance'] = {'inputs': {name: {'path': str(path.resolve()), 'sha256': digest(path)} for name, path in
                                      [('timeline', args.timeline), ('estimate', args.estimate), ('gt', args.gt)]},
                            'frame_contract_user_asserted': args.frame_contract, 'map_contract_user_asserted': args.map_contract,
                            'time_offset_ns': 0, 'max_est_diff_ms': args.max_est_diff_ms,
                            'max_gt_gap_ms': args.max_gt_gap_ms, 'max_frame_gap_ms': args.max_frame_gap_ms}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
        f.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
