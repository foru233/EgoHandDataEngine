"""Summarize sampled native-worker resources without inferring an unbounded leak."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def summarize(rows):
    samples = []
    for index, row in enumerate(rows):
        resource = row.get('resources')
        if resource is None:
            continue
        samples.append({'frame_index': row.get('replay', {}).get('frame_index', index),
                        'elapsed_s': row['timing']['elapsed_s'], **resource})
    if not samples:
        return {'samples': 0, 'limitation': 'No native worker resource samples recorded'}, []
    elapsed = np.array([sample['elapsed_s'] for sample in samples])
    rss = np.array([sample['rss_bytes'] for sample in samples]) / 2**20
    commit = np.array([sample['vms_bytes'] for sample in samples]) / 2**20
    threads = [sample['threads'] for sample in samples]
    duration = float(elapsed[-1] - elapsed[0])
    last_quarter = elapsed >= elapsed[0] + duration * .75
    def slope(t, y):
        return float(np.polyfit(t - t[0], y, 1)[0]) if len(t) >= 2 and t[-1] > t[0] else None
    report = {'samples': len(samples), 'sample_first_frame': samples[0]['frame_index'],
        'sample_last_frame': samples[-1]['frame_index'], 'sampled_duration_s': duration,
        'rss_mib': {'first': float(rss[0]), 'last': float(rss[-1]), 'peak': float(rss.max()),
                    'growth': float(rss[-1] - rss[0]), 'linear_slope_mib_s': slope(elapsed, rss),
                    'last_quarter_slope_mib_s': slope(elapsed[last_quarter], rss[last_quarter])},
        'commit_mib': {'first': float(commit[0]), 'last': float(commit[-1]), 'peak': float(commit.max())},
        'threads': {'first': threads[0], 'last': threads[-1], 'peak': max(threads)},
        'cpu_average_cores': ((samples[-1]['cpu_seconds'] - samples[0]['cpu_seconds']) / duration
                              if duration > 0 else None),
        'limitation': 'Finite replay samples show observed growth only. Map/keyframe growth, allocator caches, '
                      'OS residency and a leak cannot be separated from RSS alone. This is not a 30-minute live-device test.'}
    return report, samples


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--poses', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    rows = [json.loads(line) for line in args.poses.read_text(encoding='utf8').splitlines() if line.strip()]
    report, samples = summarize(rows)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'resources.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    if samples:
        with (args.output / 'resources.csv').open('w', newline='', encoding='utf8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
            writer.writeheader(); writer.writerows(samples)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        time = [sample['elapsed_s'] for sample in samples]
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
        for key in ['rss_bytes', 'vms_bytes']:
            axes[0].plot(time, [sample[key] / 2**20 for sample in samples], label=key)
        axes[0].legend(); axes[0].set_ylabel('MiB')
        axes[1].plot(time, [sample['threads'] for sample in samples]); axes[1].set_ylabel('Threads')
        axes[2].plot(time, [sample['cpu_seconds'] for sample in samples]); axes[2].set_ylabel('CPU seconds')
        axes[2].set_xlabel('Replay elapsed wall time (s)')
        for ax in axes: ax.grid(alpha=.25)
        fig.savefig(args.output / 'resources.png', dpi=140)
        plt.close(fig)
    print(json.dumps(report))


if __name__ == '__main__':
    main()
