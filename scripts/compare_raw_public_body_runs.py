"""Compare immutable native body poses on common timestamps and reference spans."""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from run_public_recovery_acceptance import ROOT, save, sha


def stats(values):
    a = np.asarray(values)
    return dict(count=len(a), rmse=float(np.sqrt(np.mean(a*a))),
                p95=float(np.percentile(a, 95)), max=float(a.max())) if len(a) else None


def identity(row):
    s = row['snapshot']
    ref = s.get('reference_frame') or {}
    return tuple([s.get(k) for k in ('map_id', 'reset_id')] +
                 [ref.get('atlas_generation'), s.get('reference_generation'), row['world_frame_revision']])


def usable(row):
    s = row['snapshot']
    b = s.get('body_state')
    r = s.get('reference_frame') or {}
    return (row['tracking_state'] == 'OK' and row['raw_pose_valid'] and s.get('snapshot_consistent') is True
            and b is not None and r.get('frame_bound') is True
            and r.get('generation') == s.get('reference_generation')
            and all(b.get(k) == s.get(k) for k in ('frame_id', 'timestamp_ns', 'world_frame_id')))


def errors(est, truth):
    return (np.linalg.norm(est[:, :3, 3]-truth[:, :3, 3], axis=1),
            np.rad2deg(Rotation.from_matrix(np.swapaxes(truth[:, :3, :3], 1, 2) @ est[:, :3, :3]).magnitude()))


def evaluate(items, arm):
    est = np.asarray([r[1+arm] for r in items])
    truth = np.asarray([r[3] for r in items])
    seq = np.asarray([r[4] for r in items])
    align = truth[0] @ np.linalg.inv(est[0])
    ep, er = errors(align[None] @ est, truth)
    adjacent = np.diff(seq) == 1
    de = np.linalg.inv(est[:-1]) @ est[1:]
    dg = np.linalg.inv(truth[:-1]) @ truth[1:]
    relative = np.linalg.inv(dg[adjacent]) @ de[adjacent]
    result = dict(initial_alignment_translation_m=stats(ep), initial_alignment_rotation_deg=stats(er),
                  consecutive_rpe_translation_m=stats(np.linalg.norm(relative[:, :3, 3], axis=1)),
                  excluded_nonconsecutive_pairs=int(np.sum(~adjacent)))
    t = np.asarray([r[0] for r in items], dtype=np.int64)
    train = t-t[0] < 5_000_000_000
    result['first5s_alignment_later_errors'] = dict(status='insufficient_span_or_degenerate')
    if sum(train) >= 3 and sum(~train):
        x, y = est[train, :3, 3], truth[train, :3, 3]
        cx, cy = x.mean(0), y.mean(0)
        u, sv, vt = np.linalg.svd((x-cx).T @ (y-cy))
        if sv[0] > 0 and sv[1]/sv[0] > 1e-6:
            rot = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
            a = np.eye(4)
            a[:3, :3], a[:3, 3] = rot, cy-rot@cx
            hp, hr = errors(a[None] @ est[~train], truth[~train])
            result['first5s_alignment_later_errors'] = dict(train_pairs=int(sum(train)), later_pairs=int(sum(~train)),
                                                           translation_m=stats(hp), rotation_deg=stats(hr))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    plans, rows, frozen = [], [], {}
    for arm in ('control', 'candidate'):
        run = args.suite/arm
        files = [run/n for n in ('plan.json', 'run_summary.json', 'native_frames.jsonl')]
        frozen.update({str(p):sha(p) for p in files})
        plan, summary = [json.loads(p.read_text(encoding='utf8')) for p in files[:2]]
        assert summary['plan_sha256'] == sha(files[0]) and summary['native_frames_sha256'] == sha(files[2])
        assert summary['input_integrity_passed']
        plans.append(plan)
        rows.append([json.loads(s) for s in files[2].open(encoding='utf8')])
    for field in ('dataset', 'expected_frames', 'first_ns', 'last_ns', 'settings', 'input_hashes',
                  'library_sha256', 'runner_sha256', 'normal_python', 'scoring'):
        assert plans[0][field] == plans[1][field], field
    envs = [{k:v for k,v in p['effective_native_environment'].items()
             if k not in ('EGOHAND_VIO_EXACT_GRAPH_DIR', 'EGOHAND_VIO_KF_COVARIANCE_PRIOR')} for p in plans]
    assert envs[0] == envs[1]
    assert [p['effective_native_environment']['EGOHAND_VIO_KF_COVARIANCE_PRIOR'] for p in plans] == ['0', '1']
    assert [r['timestamp_ns'] for r in rows[0]] == [r['timestamp_ns'] for r in rows[1]]
    spec = importlib.util.spec_from_file_location('raw_body_gt', ROOT/'handoff/goal_delivery_20261113/benchmark_public_vio.py')
    benchmark = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark)
    gt_file = benchmark.paths(plans[0]['dataset'])[3]
    frozen[str(gt_file)] = sha(gt_file)
    times, gt = benchmark.load_gt(plans[0]['dataset'])
    intervals, rejected = [], Counter()
    for pair in zip(*rows):
        key = tuple(identity(r) for r in pair)
        if not intervals or intervals[-1]['references'] != key:
            intervals.append(dict(references=key, items=[]))
        if not all(usable(r) for r in pair):
            rejected['either_body_unavailable'] += 1
            continue
        t = pair[0]['timestamp_ns']
        j = int(np.searchsorted(times, t))
        if j < len(times) and times[j] == t:
            truth = gt[j]
        else:
            scoring = plans[0]['scoring']
            if not 0 < j < len(times):
                rejected['outside_gt'] += 1
                continue
            gap = int(times[j]-times[j-1])
            if gap > scoring['max_interpolation_gap_ns'] or min(t-int(times[j-1]), int(times[j])-t) > scoring['max_nearest_time_ns']:
                rejected['gt_gap'] += 1
                continue
            alpha = (t-int(times[j-1]))/gap
            truth = np.eye(4)
            truth[:3, 3] = (1-alpha)*gt[j-1,:3,3] + alpha*gt[j,:3,3]
            truth[:3,:3] = Slerp([0.,1.], Rotation.from_matrix(gt[j-1:j+1,:3,:3]))([alpha]).as_matrix()[0]
        estimates = [np.asarray(r['snapshot']['body_state']['T_world_from_body']) for r in pair]
        assert all(np.isfinite(a).all() and a.shape == (4,4) for a in estimates)
        intervals[-1]['items'].append((t, *estimates, truth, pair[0]['input_sequence']))
    reports = [dict(references=v['references'], pairs=len(v['items']), first_sequence=v['items'][0][4],
                    last_sequence=v['items'][-1][4],
                    **{a:evaluate(v['items'],i) for i,a in enumerate(('control','candidate'))})
               for v in intervals if v['items']]
    assert all(sha(Path(p)) == h for p,h in frozen.items())
    save(args.output,dict(intervals=reports, hashes=frozen, rejections=dict(rejected),
         common_scored_pairs=sum(r['pairs'] for r in reports), source_sha256=sha(Path(__file__)),
         same_inputs_and_single_algorithm_variable=True, production_deployed=False, scale=1., time_offset_ns=0,
         limits='Raw body accuracy on development data; split on either reference change. No app recovery, stitched trajectory, independent final holdout, or statistical noninferiority claim.'))
    print(json.dumps(dict(common_pairs=sum(r['pairs'] for r in reports), intervals=len(reports))))


if __name__ == '__main__':
    main()
