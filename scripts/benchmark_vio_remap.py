"""Compare VIO float/fixed OpenCV maps on recorded frames; no VIO policy changes."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import cv2
import numpy as np
from egohand.storage.sdk_recording import iter_paired_frames
from egohand.vio.live_recovery import LiveRecoveryProfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--pairs', type=int, default=60)
    args = parser.parse_args()
    profile = LiveRecoveryProfile()
    rect = profile.recording_options(args.recording)['config_snapshot'].validate_geometry().rectifier
    timings = {'float_ms': [], 'fixed_ms': []}
    digest = hashlib.sha256()
    different_frames = 0
    max_difference = 0
    sampled = 0
    for i, frame in enumerate(iter_paired_frames(args.recording, limit=args.pairs)):
        sampled += 1
        def floating():
            return (cv2.remap(frame.right_bgr, *rect.maps_left, cv2.INTER_LINEAR),
                    cv2.remap(frame.left_bgr, *rect.maps_right, cv2.INTER_LINEAR))

        def fixed():
            return rect.remap_pair(frame.right_bgr, frame.left_bgr)

        results = {}
        # Alternate ordering to reduce a systematic first/second bias.
        for name, fn in ((('float_ms', floating), ('fixed_ms', fixed)) if i % 2 == 0
                         else (('fixed_ms', fixed), ('float_ms', floating))):
            start = time.perf_counter_ns()
            results[name] = fn()
            timings[name].append((time.perf_counter_ns() - start) / 1e6)
        for old, new in zip(results['float_ms'], results['fixed_ms']):
            delta = np.abs(old.astype(np.int16) - new.astype(np.int16))
            max_difference = max(max_difference, int(delta.max()))
            different_frames += int(np.any(delta))
            digest.update(old.tobytes())
    def distribution(values):
        return {'n': len(values), 'mean': float(np.mean(values)),
                'p50': float(np.percentile(values, 50)), 'p95': float(np.percentile(values, 95)),
                'p99': float(np.percentile(values, 99)), 'max': float(np.max(values))}
    if sampled == 0:
        raise ValueError('No paired frames were decoded')
    report = {'requested_pairs': args.pairs, 'sampled_pairs': sampled, 'opencv_threads': cv2.getNumThreads(),
              'different_images': different_frames, 'max_pixel_difference': max_difference,
              'output_sha256': digest.hexdigest(), 'timing_ms': {k: distribution(v) for k,v in timings.items()},
              'profile_files': profile.provenance['files'],
              'limitation': 'single offline process; remap only, not end-to-end FPS or accuracy'}
    with args.output.open('x', encoding='utf8') as f:
        json.dump(report, f, indent=2, allow_nan=False)
    print(json.dumps(report, indent=2))
    return 0 if different_frames == 0 and sampled == args.pairs else 1


if __name__ == '__main__':
    sys.exit(main())
