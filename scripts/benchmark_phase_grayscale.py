"""Compare identity-resize removal on actual phase-calibration input frames."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.storage.sdk_recording import iter_paired_frames
from egohand.vio.live_recovery import LiveRecoveryProfile
from egohand.vio.session_phase import _phase_gray_pair


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--pairs', type=int, default=60)
    args = parser.parse_args()
    candidate = LiveRecoveryProfile()
    config = candidate.recording_options(args.recording)['config_snapshot']
    if config.image_scale != 1.0:
        raise ValueError('This comparison requires identity scale')
    rectifier = config.validate_geometry().rectifier
    old_ms, new_ms, digest = [], [], hashlib.sha256()
    mismatch = count = 0
    for index, frame in enumerate(iter_paired_frames(args.recording, limit=args.pairs, stride=2)):
        left, right = rectifier.remap_pair(frame.right_bgr, frame.left_bgr)
        def old():
            return [cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), None, fx=1., fy=1.,
                               interpolation=cv2.INTER_AREA) for im in (left, right)]
        def new():
            return _phase_gray_pair(left, right, 1.)
        results = {}
        for name, func, timing in ((('old', old, old_ms), ('new', new, new_ms)) if index % 2 == 0
                                   else (('new', new, new_ms), ('old', old, old_ms))):
            start = time.perf_counter_ns(); results[name] = func()
            timing.append((time.perf_counter_ns()-start)/1e6)
        for before, after in zip(results['old'], results['new']):
            mismatch += int(not np.array_equal(before, after))
            digest.update(before.tobytes())
        count += 1
    if not count: raise ValueError('No frames decoded')
    def stats(values):
        return {'count':len(values), 'mean':float(np.mean(values)),
                'p50':float(np.percentile(values,50)), 'p95':float(np.percentile(values,95)),
                'p99':float(np.percentile(values,99)), 'max':float(np.max(values))}
    report = {'sampled_pairs':count, 'requested_pairs':args.pairs,
              'mismatched_images':mismatch, 'sha256':digest.hexdigest(),
              'opencv_threads':cv2.getNumThreads(), 'old_ms':stats(old_ms), 'new_ms':stats(new_ms),
              'calibration_sha256':candidate.provenance['files']['calibration']['sha256'],
              'limitation':'isolated conversion only; not end-to-end latency or physical accuracy'}
    with args.output.open('x',encoding='utf8') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2))
    return 0 if count==args.pairs and mismatch==0 else 1


if __name__ == '__main__':
    sys.exit(main())
