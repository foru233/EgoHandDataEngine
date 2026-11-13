"""Compare full conversion with selective conversion for identical HEVC pairs."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from egohand.storage.sdk_recording import iter_paired_frames


def measure(root, selected_count, stride):
    digest = hashlib.sha256()
    started = time.perf_counter()
    count = 0
    limit = selected_count if stride == 2 else selected_count * 2 - 1
    for pair in iter_paired_frames(root, limit=limit, stride=stride):
        if pair.index % 2:
            continue
        digest.update(f'{pair.index}:{pair.sequence}:{pair.device_ts}:'.encode('ascii'))
        digest.update(pair.left_bgr.tobytes())
        digest.update(pair.right_bgr.tobytes())
        count += 1
    return {'selected_pairs': count, 'elapsed_s': time.perf_counter() - started,
            'sha256': digest.hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--pairs', type=int, default=136)
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    results = []
    for index in range(args.repeats):
        for stride in ((1, 2) if index % 2 == 0 else (2, 1)):
            results.append({'run': index, 'stride': stride, **measure(args.recording, args.pairs, stride)})
    expected = results[0]['sha256']
    equivalent = all(item['selected_pairs'] == args.pairs and item['sha256'] == expected for item in results)
    with args.output.open('x', encoding='utf8') as output:
        json.dump({'equivalent': equivalent, 'results': results}, output, indent=2)
    print(json.dumps({'equivalent': equivalent, 'results': results}, indent=2))
    return 0 if equivalent else 1


if __name__ == '__main__':
    sys.exit(main())
