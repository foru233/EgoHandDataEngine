"""Copy captured graphs with lossless floating tracking-depth number spelling.

OpenCV can emit a float as an integer outside the reader's integer range.
Keep originals and nonfinite values; no replacement, clamping or optimizer edit.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

from opencv_graph_json import read_graph, nonfinite_paths, tracking_depth_only


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def equal(a, b):
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(equal(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return len(a) == len(b) and all(equal(x, y) for x, y in zip(a, b))
    if isinstance(a, float) and math.isnan(a):
        return isinstance(b, float) and math.isnan(b)
    return a == b


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    graphs = args.output / 'exact_native_graph'
    graphs.mkdir()
    shutil.copyfile(args.run / 'plan.json', args.output / 'plan.json')
    records = []
    for source in sorted((args.run / 'exact_native_graph').glob('*.json')):
        original = read_graph(source)
        paths = nonfinite_paths(original)
        if not tracking_depth_only(paths):
            raise ValueError(f'nonfinite outside tracking_depth: {source}: {paths}')
        value = read_graph(source)
        promoted = []
        for i, point in enumerate(value['map_points']):
            depth = point['tracking_depth']
            if isinstance(depth, int):
                if int(float(depth)) != depth:
                    raise ValueError(f'inexact float promotion: {source}: {depth}')
                promoted.append(i)
                point['tracking_depth'] = float(depth)
        # Nonfinite tokens only occur in verified numeric tracking_depth fields.
        text = json.dumps(value, indent=2).replace(': NaN', ': .Nan').replace(': Infinity', ': .Inf').replace(': -Infinity', ': -.Inf')
        dest = graphs / source.name
        with dest.open('x', encoding='utf8') as stream:
            stream.write(text)
        if not equal(original, read_graph(dest)):
            raise ValueError(f'numeric values changed: {source}')
        records.append(dict(source=str(source.resolve()), source_sha256=sha(source),
            copy=str(dest.resolve()), copy_sha256=sha(dest),
            tracking_depth_integer_to_float_indices=promoted, nonfinite_paths=paths,
            decoded_values_and_nonfinite_masks_equal=True))
    if any(sha(Path(r['source'])) != r['source_sha256'] for r in records):
        raise ValueError('original graph changed')
    with (args.output / 'conversion_manifest.json').open('x', encoding='utf8') as stream:
        json.dump(dict(records=records, converter_sha256=sha(Path(__file__)),
            original_run=str(args.run.resolve()), production_deployed=False,
            purpose='Input number spelling compatibility only; all values preserved including NaN/Inf.',
            limits='Not a new native replay or an algorithm change.'), stream, indent=2)
    print(json.dumps(dict(graph_files=len(records), original_graphs_changed=False)))


if __name__ == '__main__':
    main()
