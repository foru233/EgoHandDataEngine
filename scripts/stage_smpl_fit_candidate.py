"""Stage an audited experimental fit for GUI evaluation without modifying the source recording."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('report', type=Path)
    parser.add_argument('vertices', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    dest = args.destination.resolve()
    target, mask_target = dest / 'body_vertices.npy', dest / 'body_vertices_valid.npy'
    if target.exists() or mask_target.exists():
        raise FileExistsError('Refusing to overwrite an existing mesh cache')
    with (dest / 'derived_pose.jsonl').open(encoding='utf8') as stream:
        identities = [(r['frame_id'], r['timestamp_ns']) for r in map(json.loads, stream)]
    report = json.loads(args.report.read_text(encoding='utf8'))
    source = np.load(args.vertices, mmap_mode='r', allow_pickle=False)
    if source.shape != (len(identities), 6890, 3) or len(report) != len(identities):
        raise ValueError('Fit, report and sidecar frame counts do not match')
    valid = np.zeros(len(report), dtype=bool)
    temp = dest / 'body_vertices.candidate-build.npy'
    try:
        output = np.lib.format.open_memmap(temp, mode='w+', dtype='float16', shape=source.shape)
        for index, row in enumerate(report):
            if row.get('index') != index:
                raise ValueError(f'Fit index mismatch at {index}')
            if not row.get('valid'):
                output[index] = 0
                continue
            if row.get('frame_id') != identities[index][0] or not np.isfinite(source[index]).all():
                raise ValueError(f'Invalid fit frame at {index}')
            # The audited source is in BodyPose coordinates; the existing GUI
            # cache contract stores Y-up and converts it on read.
            output[index] = source[index] * np.array([1., -1., 1.], dtype=np.float32)
            valid[index] = True
        output.flush()
        del output
        with mask_target.open('xb') as stream:
            np.save(stream, valid, allow_pickle=False)
        temp.rename(target)
    finally:
        if temp.exists():
            temp.unlink()
        if mask_target.exists() and not target.exists():
            mask_target.unlink()
    print(json.dumps({'frames': len(report), 'valid': int(valid.sum()), 'directory': str(dest)}))


if __name__ == '__main__':
    main()
