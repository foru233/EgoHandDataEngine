"""Summarize recorded hand-pipeline timings, stratified by actual output."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def summarize(rows):
    groups = {'all': [], 'either_detector': [], 'both_detectors': [],
              'valid_3d_joint': [], 'no_detected_hands': []}
    failures = Counter()
    for row in rows:
        diag = row.get('diagnostics') or {}
        left = diag.get('detected_hands_left')
        right = diag.get('detected_hands_right')
        known = type(left) is int and type(right) is int and left >= 0 and right >= 0
        valid = diag.get('valid_joints')
        groups['all'].append(row)
        if known and (left > 0 or right > 0): groups['either_detector'].append(row)
        if known and left > 0 and right > 0: groups['both_detectors'].append(row)
        if known and left == 0 and right == 0: groups['no_detected_hands'].append(row)
        if type(valid) is int and valid > 0: groups['valid_3d_joint'].append(row)
        if diag.get('reason'): failures[str(diag['reason'])] += 1

    def distribution(values):
        values = [float(v) for v in values if type(v) in (int, float) and math.isfinite(v) and v >= 0]
        if not values: return {'count': 0, 'mean': None, 'p50': None, 'p95': None, 'p99': None, 'max': None}
        return {'count': len(values), 'mean': float(np.mean(values)),
                'p50': float(np.percentile(values, 50)), 'p95': float(np.percentile(values, 95)),
                'p99': float(np.percentile(values, 99)), 'max': max(values)}

    def metrics(group):
        return {'frames': len(group),
                'engine_total_ms': distribution([r.get('processing_time_ms') for r in group]),
                **{key: distribution([(r.get('diagnostics') or {}).get(key) for r in group])
                   for key in ('left_detection_ms', 'right_detection_ms',
                               'matching_and_triangulation_ms', 'triangulation_ms')}}

    return {'groups': {name: metrics(group) for name, group in groups.items()},
            'reason_counts': dict(failures),
            'limitations': ['Groups overlap and are selected by observed output, not ground truth',
                'Left/right detector timings can overlap and must not be added as wall latency',
                'Engine time excludes separate VIO, acquisition, GUI rendering, and recording',
                'No pose accuracy, exposure latency, or device FPS is established']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    digest = hashlib.sha256()
    with args.frames.open('rb') as source:
        def rows():
            for line in source:
                digest.update(line)
                if line.strip(): yield json.loads(line)
        report = summarize(rows())
    report['frames_sha256'] = digest.hexdigest()
    with args.output.open('x', encoding='utf8') as target:
        json.dump(report, target, indent=2, allow_nan=False)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
