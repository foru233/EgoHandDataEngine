"""Compare paired, same-input native snapshots without joining reference frames.

Uses exact image timestamps, independently aligns both runs at the same first
paired GT body pose, and never fits scale or time. This is exploratory: one
run per candidate does not establish statistical non-inferiority.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ROOT / 'handoff/goal_delivery_20261113/benchmark_public_vio.py'


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def read_run(path):
    plan = json.loads((path / 'plan.json').read_text(encoding='utf-8'))
    summary = json.loads((path / 'run_summary.json').read_text(encoding='utf-8'))
    frames_path = path / 'native_frames.jsonl'
    if sha(frames_path) != summary['native_frames_sha256'] or sha(path / 'plan.json') != summary['plan_sha256']:
        raise ValueError('run provenance or frames hash mismatch')
    with frames_path.open(encoding='utf-8') as f:
        rows = [json.loads(line) for line in f]
    if len(rows) != summary['counts']['frames'] or len(rows) != plan['expected_frames']:
        raise ValueError('incomplete run')
    return plan, summary, rows


def body(row):
    s = row['snapshot']
    b = s.get('body_state')
    ref = s.get('reference_frame') or {}
    if (row['tracking_state'] != 'OK' or not row['raw_pose_valid'] or s.get('snapshot_consistent') is not True
            or b is None or ref.get('frame_bound') is not True or ref.get('generation') != s.get('reference_generation')
            or any(b.get(k) != s.get(k) for k in ('frame_id', 'timestamp_ns', 'world_frame_id'))):
        return None
    T = np.asarray(b['T_world_from_body'], dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-5)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1.0, atol=1e-4)):
        raise ValueError('invalid body matrix')
    return (s['map_id'], s['reset_id'], ref['atlas_generation'], s['reference_generation'], row['world_frame_revision']), T


def stat(values):
    a = np.asarray(values, dtype=float)
    return {'count': len(a), 'rmse': float(np.sqrt(np.mean(a*a))), 'mean': float(np.mean(a)),
            'median': float(np.median(a)), 'p95': float(np.percentile(a, 95)), 'max': float(np.max(a))} if len(a) else None


def metrics(items):
    truth = np.stack([x[3] for x in items])
    a = np.stack([x[4] for x in items]); b = np.stack([x[5] for x in items])
    indices = np.array([x[0] for x in items]); aligned = []
    for estimate in (a, b):
        transform = truth[0] @ np.linalg.inv(estimate[0])
        p = transform[None] @ estimate
        error = np.linalg.norm(p[:, :3, 3] - truth[:, :3, 3], axis=1)
        rotation = np.degrees(Rotation.from_matrix(np.transpose(truth[:, :3, :3], (0, 2, 1)) @ p[:, :3, :3]).magnitude())
        keep = (np.diff(indices) == 1) & np.array([x[6] for x in items[1:]], dtype=bool)
        dg = np.linalg.inv(truth[:-1]) @ truth[1:]
        de = np.linalg.inv(estimate[:-1]) @ estimate[1:]
        residual = np.linalg.inv(dg[keep]) @ de[keep]
        aligned.append({'ape_translation_m': stat(error), 'ape_rotation_deg': stat(rotation),
                        'rpe_translation_m': stat(np.linalg.norm(residual[:, :3, 3], axis=1)),
                        'rpe_rotation_deg': stat(np.degrees(Rotation.from_matrix(residual[:, :3, :3]).magnitude())) if len(residual) else None})
    return {'first_input_sequence': int(indices[0]), 'last_input_sequence': int(indices[-1]),
            'paired_frames': len(items), 'baseline_identity': list(items[0][1][0]),
            'candidate_identity': list(items[0][1][1]), 'baseline': aligned[0], 'candidate': aligned[1]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline', type=Path); p.add_argument('candidate', type=Path); p.add_argument('output', type=Path)
    args = p.parse_args()
    pa, sa, a = read_run(args.baseline); pb, sb, b = read_run(args.candidate)
    if pa['dataset'] != pb['dataset'] or pa['input_hashes'] != pb['input_hashes'] or len(a) != len(b):
        raise ValueError('runs do not use identical input data')
    spec = importlib.util.spec_from_file_location('public_benchmark', PUBLIC)
    public = importlib.util.module_from_spec(spec); spec.loader.exec_module(public)
    gt_path = public.paths(pa['dataset'])[3]; gt_hash = sha(gt_path)
    gt_times, gt_poses = public.load_gt(pa['dataset'])
    gaps = np.diff(gt_times) > 20_000_000
    gap_starts, gap_ends = gt_times[:-1][gaps], gt_times[1:][gaps]
    groups, current, excluded = [], [], Counter()
    for ra, rb in zip(a, b):
        if (ra['input_sequence'], ra['timestamp_ns']) != (rb['input_sequence'], rb['timestamp_ns']):
            raise ValueError('image timelines differ')
        aa, bb = body(ra), body(rb)
        if aa is None or bb is None:
            excluded['one_or_both_native_bodies_unavailable'] += 1
            if current:
                groups.append(current); current = []
            continue
        t = ra['timestamp_ns']; j = int(np.searchsorted(gt_times, t))
        if j < len(gt_times) and int(gt_times[j]) == t:
            truth = gt_poses[j]
        elif j == 0 or j == len(gt_times):
            excluded['outside_gt_range'] += 1
            continue
        else:
            gap = int(gt_times[j] - gt_times[j-1])
            if gap > 20_000_000 or min(t-int(gt_times[j-1]), int(gt_times[j])-t) > 5_000_000:
                excluded['gt_gap_or_distance'] += 1
                continue
            alpha = (t-int(gt_times[j-1])) / gap
            truth = np.eye(4)
            truth[:3, 3] = (1-alpha)*gt_poses[j-1, :3, 3] + alpha*gt_poses[j, :3, 3]
            truth[:3, :3] = Slerp([0, 1], Rotation.from_matrix(gt_poses[j-1:j+1, :3, :3]))([alpha]).as_matrix()[0]
        key = (aa[0], bb[0])
        if current and key != current[-1][1]:
            groups.append(current); current = []
        previous = current[-1][2] if current else t
        gap_index = np.searchsorted(gap_ends, previous, side='right')
        continuous_gt = not (gap_index < len(gap_starts) and gap_starts[gap_index] < t)
        current.append((ra['input_sequence'], key, t, truth, aa[1], bb[1], continuous_gt))
    if current:
        groups.append(current)
    report = {'dataset': pa['dataset'], 'input_frames': len(a), 'paired_frames': sum(map(len, groups)),
              'excluded': dict(excluded), 'segments': [metrics(g) for g in groups],
              'provenance': {'baseline_frames_sha256': sa['native_frames_sha256'], 'candidate_frames_sha256': sb['native_frames_sha256'],
                             'ground_truth_sha256': gt_hash, 'input_hashes_equal': True, 'time_offset_ns': 0, 'scale': 1.0},
              'interpretation': ['First common GT body pose determines independent SE(3) alignment for each run.',
                                 'Reference identities are tracked separately; identical numbers across runs do not imply identical maps.',
                                 'Segments restart after a missing native body, which can hide long-term drift across a loss.',
                                 'RPE excludes nonconsecutive input frames; no inference is made about missing frames.',
                                 'One pair of runs is not a statistical non-inferiority test; this is native-only, not application recovery.']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
