"""Export raw input pairs around largest raw pose steps, without changing replay timing."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'map_creat'))
from egohand.vio.config_snapshot import resolve_recording_config
from atlas_replay import recording_frames


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--recording', type=Path, required=True)
    p.add_argument('--poses', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--cases', type=int, default=8)
    a = p.parse_args()
    rows = [json.loads(l) for l in a.poses.read_text(encoding='utf8').splitlines()]
    pairs = []
    for i in range(1, len(rows)):
        prev, cur = rows[i-1:i+1]
        x, y = prev.get('raw_matrix'), cur.get('raw_matrix')
        dt = (cur['timestamp_ns'] - prev['timestamp_ns'])/1e9
        if x is not None and y is not None and 0 < dt <= .2:
            distance = float(np.linalg.norm(np.array(x)[:3, 3]-np.array(y)[:3, 3]))
            pairs.append((distance, i))
    cases = sorted(pairs, reverse=True)[:a.cases]
    targets = {}
    for d, i in cases:
        dest = a.output / f'frame_{i:05d}'
        dest.mkdir(parents=True, exist_ok=True)
        (dest/'state.json').write_text(json.dumps({'step_m': d,
            'same_epoch': rows[i-1]['map_epoch'] == rows[i]['map_epoch'],
            'rows': rows[max(0, i-2):i+3],
            'limitation': 'Feature overlays below are independent OpenCV diagnostics, not native ORB inliers.'}, indent=2), encoding='utf8')
        for r in rows[max(0, i-1):i+2]:
            targets.setdefault(r['timestamp_ns'], []).append(dest)
    config = resolve_recording_config(a.recording)
    geometry = config.validate_geometry()
    orb = cv2.ORB_create(nfeatures=1200)
    for stamp, left, right, _ in recording_frames(a.recording, geometry, config.timing):
        if stamp not in targets:
            continue
        kp1, ds1 = orb.detectAndCompute(left, None)
        kp2, ds2 = orb.detectAndCompute(right, None)
        matches = []
        if ds1 is not None and ds2 is not None:
            matches = [m for candidates in cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(ds1, ds2, k=2)
                       if len(candidates) == 2 for m, n in [candidates] if m.distance < .75*n.distance]
        errors = [abs(kp1[m.queryIdx].pt[1]-kp2[m.trainIdx].pt[1]) for m in matches]
        disparity = [kp1[m.queryIdx].pt[0]-kp2[m.trainIdx].pt[0] for m in matches]
        overlay = cv2.drawMatches(left, kp1, right, kp2, matches[:150], None)
        for dest in targets[stamp]:
            cv2.imwrite(str(dest/f'{stamp}_stereo.png'), np.hstack([left, right]))
            cv2.imwrite(str(dest/f'{stamp}_independent_matches.png'), overlay)
            (dest/f'{stamp}_features.json').write_text(json.dumps({'left_features': len(kp1),
                'right_features': len(kp2), 'ratio_matches': len(matches),
                'epipolar_error_p50_p95_px': np.percentile(errors, [50,95]).tolist() if errors else None,
                'disparity_p05_p50_p95_px': np.percentile(disparity, [5,50,95]).tolist() if disparity else None,
                'note': 'Independent feature diagnostic; not a measurement of native frontend inliers.'}, indent=2))
    print(f'Exported {len(cases)} cases to {a.output}')


if __name__ == '__main__':
    main()
