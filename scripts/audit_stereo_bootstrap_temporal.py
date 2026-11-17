"""Read-only adjacent-stereo feasibility probe; never authorizes map admission."""
from collections import Counter
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from egohand.storage.sdk_recording import iter_paired_frames
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry


def mutual(a, b, matcher):
    if a is None or b is None or len(a) < 2 or len(b) < 2:
        return {}
    def selected(x, y):
        return {m.queryIdx: m.trainIdx for pair in matcher.knnMatch(x, y, k=2)
                if len(pair) == 2 for m, n in [pair] if m.distance < .75 * n.distance and m.distance <= 50}
    forward, reverse = selected(a, b), selected(b, a)
    return {i: j for i, j in forward.items() if reverse.get(j) == i}


def main(args):
    poses = [json.loads(l) for l in args.poses.open(encoding='utf8')]
    wanted = {p['timestamp_ns']: (i, p) for i, p in enumerate(poses) if args.first <= i <= args.last}
    geometry = prepare_vs030_vio_geometry(args.calibration)
    rectifier = geometry.rectifier
    K = rectifier.P1[:, :3]
    baseline = -rectifier.P2[0, 3] / K[0, 0]
    orb = cv2.ORB_create(nfeatures=1200, scaleFactor=1.2, nlevels=8, fastThreshold=7)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    previous, results = None, []
    first_ts, last_ts = min(wanted), max(wanted)
    for pair in iter_paired_frames(args.recording):
        timestamp = int(pair.device_ts) * 1000
        if timestamp < first_ts - 100_000_000:
            continue
        if timestamp > last_ts:
            break
        left, right = rectifier.remap_pair(pair.right_bgr, pair.left_bgr)
        left, right = [cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) for im in (left, right)]
        kl, dl = orb.detectAndCompute(left, None)
        kr, dr = orb.detectAndCompute(right, None)
        stereo, points = {}, {}
        for i, j in mutual(dl, dr, matcher).items():
            u, v = kl[i].pt
            ur, vr = kr[j].pt
            disparity = u - ur
            if abs(v - vr) > 2 * 1.2 ** max(kl[i].octave, kr[j].octave) or disparity <= 1:
                continue
            z = K[0, 0] * baseline / disparity
            if not np.isfinite(z) or z <= 0:
                continue
            points[i] = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])
            stereo[i] = j
        row = dict(timestamp_ns=timestamp, raw_pair_index=int(pair.index), left_features=len(kl),
                   stereo_points=len(stereo), temporal_stereo_matches=0, pnp_inliers=0,
                   stereo_geometric_inliers=0, occupied_cells=0, stereo_chi2_p95=None,
                   adjacent_candidate_supported=False)
        if previous is not None and 0 < timestamp - previous['timestamp'] < 100_000_000:
            matches = [(i, j) for i, j in mutual(previous['descriptors'], dl, matcher).items()
                       if i in previous['points'] and j in stereo]
            row['temporal_stereo_matches'] = len(matches)
            if len(matches) >= 6:
                xyz = np.array([previous['points'][i] for i, j in matches], np.float64)
                uv = np.array([kl[j].pt for i, j in matches], np.float64)
                cv2.setRNGSeed(173)
                ok, rvec, tvec, inliers = cv2.solvePnPRansac(xyz, uv, K, None, iterationsCount=300,
                    reprojectionError=3., confidence=.999, flags=cv2.SOLVEPNP_EPNP)
                if ok and inliers is not None and len(inliers) >= 6:
                    ix = inliers.ravel()
                    ok, rvec, tvec = cv2.solvePnP(xyz[ix], uv[ix], K, None, rvec, tvec, True,
                                               flags=cv2.SOLVEPNP_ITERATIVE)
                    R = cv2.Rodrigues(rvec)[0]
                    current = xyz @ R.T + tvec.ravel()
                    projected = current @ K.T
                    projected = projected[:, :2] / projected[:, 2:]
                    right_projected = projected.copy()
                    right_projected[:, 0] -= K[0, 0] * baseline / current[:, 2]
                    right_uv = np.array([kr[stereo[j]].pt for i, j in matches])
                    sigma2 = np.array([1.2 ** (2 * kl[j].octave) for i, j in matches])
                    # Same 3-coordinate stereo residual convention as native pinhole edges.
                    chi2 = ((projected - uv) ** 2).sum(axis=1) + (right_projected[:, 0] - right_uv[:, 0]) ** 2
                    chi2 /= sigma2
                    good = (chi2 <= 7.815) & (current[:, 2] > 0)
                    pixels = uv[good]
                    cell_counts = Counter((min(3, int(u * 4 / left.shape[1])), min(2, int(v * 3 / left.shape[0]))) for u, v in pixels)
                    cells = {cell for cell, count in cell_counts.items() if count >= 3}
                    row.update(pnp_inliers=len(inliers), stereo_geometric_inliers=int(good.sum()),
                               occupied_cells=len(cells), stereo_chi2_p95=float(np.percentile(chi2[good], 95)) if good.any() else None,
                               adjacent_candidate_supported=bool(good.sum() >= 50 and len(cells) >= 6 and good.sum() * 2 >= len(matches)),
                               relative_camera_translation_m=float(np.linalg.norm(tvec)))
        if timestamp in wanted:
            index, native = wanted[timestamp]
            row.update(pose_index=index, native_state=native['tracking_state'],
                       native_map_id=native['map_id'], native_reset_id=native['reset_id'],
                       native_stereo_depths=native['stereo_valid_depths'])
            results.append(row)
        previous = dict(timestamp=timestamp, descriptors=dl, points=points)
    missing = sorted(set(range(args.first, args.last + 1)) - {r['pose_index'] for r in results})
    result = dict(results=results, missing_pose_indices=missing,
                  calibration_sha256=hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
                  poses_sha256=hashlib.sha256(args.poses.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  production_admission_authorized=False,
                  limits=['Independent OpenCV ORB/triangulation/PnP, not the native ORB-SLAM3 tracker.',
                          'Adjacent pair support is insufficient to establish stable initialization, map identity, or physical accuracy.',
                          'No IMU consistency, map continuity, or production quarantine release is inferred.'])
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(dict(frames=len(results), missing=missing,
                          supported=sum(r['adjacent_candidate_supported'] for r in results))))
    return bool(missing)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('recording', 'calibration', 'poses', 'output'):
        p.add_argument(name, type=Path)
    p.add_argument('--first', type=int, default=130)
    p.add_argument('--last', type=int, default=290)
    raise SystemExit(main(p.parse_args()))
