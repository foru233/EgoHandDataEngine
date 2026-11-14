"""Inventory local recordings without modifying their contents.

Hashes the complete files of raw SDK recordings; replay readiness and timing
figures are explicitly labelled as historical checks, not a fresh full replay.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXCLUDE = {'.git', '.venv', '__pycache__', '.pytest_cache', '.pytest_tmp', 'node_modules'}
MARKERS = {'sdk_index.jsonl', 'camera_frames.jsonl', 'poses.jsonl', 'CameraTrajectory.txt',
           'groundtruth.tum', 'groundtruth_from_bag.tum', 'sensor.yaml'}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read_json(path: Path) -> dict:
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError, ValueError):
        return {}


def recording(path: Path) -> dict:
    manifest = read_json(path / 'manifest.json')
    check = read_json(path / 'replay_check.json')
    raw = path / 'raw'
    hashes = {}
    for name in ('manifest.json', 'calibration.yaml', 'kalibr_camchain.yaml',
                 'vio_timing.yaml', 'raw/sdk_index.jsonl', 'raw/sdk_records.bin',
                 'raw/imu_records.bin', 'raw/camera_frames.jsonl'):
        file = path / name
        if file.is_file():
            hashes[name] = {'bytes': file.stat().st_size, 'sha256': sha256(file)}
    camera = check.get('frames', {})
    imu = check.get('imu', {})
    sample = None
    frame_file = raw / 'camera_frames.jsonl'
    if frame_file.is_file():
        with frame_file.open(encoding='utf-8') as f:
            try:
                sample = json.loads(next(f))
            except (StopIteration, ValueError):
                pass
    meta = manifest.get('metadata', {})
    return {
        'session_id': path.name, 'path': str(path.relative_to(ROOT)),
        'format': 'VS030 SDK binary/index' if (raw / 'sdk_index.jsonl').exists() else 'other recording',
        'source_type': meta.get('source_type'), 'synthetic': meta.get('synthetic'),
        'ground_truth_claimed': meta.get('ground_truth'),
        'resolution_left_shape': (sample or {}).get('left_shape'),
        'resolution_right_shape': (sample or {}).get('right_shape'),
        'camera_pairs_historical': camera.get('index_pairs'),
        'duration_s_historical': ((camera['paired_device_ts_last'] - camera['paired_device_ts_first']) / 1e6
                                  if camera.get('paired_device_ts_first') is not None and camera.get('paired_device_ts_last') is not None else None),
        'imu_rate_hz_historical': imu.get('rate_hz'), 'imu_gaps_historical': imu.get('gap_count'),
        'camera_gaps_historical': camera.get('device_ts_gap_count'),
        'sync_claimed': meta.get('synchronized'), 'timestamp_semantics': meta.get('timestamp_kind'),
        'historical_replay_ready_sample_only': check.get('replay_ready'),
        'full_replay_verified': None,
        'calibration_files_present': [x for x in ('calibration.yaml', 'kalibr_camchain.yaml') if (path / x).is_file()],
        'calibration_completeness_verified': False,
        'loss_recovery_events': 'not yet evaluated',
        'hashes': hashes,
        'limitations': ['Historical replay check samples decoding; it does not prove complete replay.',
                        'Transport timestamps do not establish physical exposure/IMU synchronization.',
                        'No external GT established for VS030.' if not meta.get('ground_truth') else 'GT requires independent verification.'],
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    roots, manifests, findings, errors = set(), set(), [], []
    counts = collections.Counter()
    for dirname, dirs, files in os.walk(ROOT, onerror=lambda e: errors.append(str(e))):
        dirs[:] = [d for d in dirs if d.lower() not in EXCLUDE and not (Path(dirname) / d).is_symlink()]
        current = Path(dirname)
        counts['directories'] += 1
        counts['files'] += len(files)
        if current.name == 'raw' and 'sdk_index.jsonl' in files:
            roots.add(current.parent)
        if 'manifest.json' in files and ('raw' in dirs or 'frames.jsonl' in files):
            manifests.add(current)
        for name in files:
            suffix = Path(name).suffix.lower()
            if name in MARKERS or suffix in {'.bag', '.db3', '.mcap'}:
                findings.append(str((current / name).relative_to(ROOT)))
                counts['marker:' + (name if name in MARKERS else suffix)] += 1
    records = []
    for path in sorted(roots):
        try:
            records.append(recording(path))
        except (OSError, ValueError) as e:
            errors.append(f'{path}: {e}')
    duplicates = collections.defaultdict(list)
    for rec in records:
        data = rec['hashes'].get('raw/sdk_records.bin', {})
        if data:
            duplicates[data['sha256']].append(rec['path'])
    other = []
    for path in sorted(manifests - roots):
        meta = read_json(path / 'manifest.json').get('metadata', {})
        other.append({'path': str(path.relative_to(ROOT)), 'session_id': path.name,
                      'source_type': meta.get('source_type'), 'synthetic': meta.get('synthetic'),
                      'contains_imu_claimed': meta.get('contains_imu'),
                      'ground_truth_claimed': meta.get('ground_truth'),
                      'manifest_sha256': sha256(path / 'manifest.json'),
                      'assessment': 'Not yet assessed; manifest metadata is a claim, not a VIO or GT validation.'})
    output = {'schema_version': 1, 'root': str(ROOT), 'excluded_directories': sorted(EXCLUDE),
              'counts': dict(counts), 'recordings': records, 'markers': sorted(findings),
              'other_recording_manifests': other,
              'same_sdk_payload_sha256': [v for v in duplicates.values() if len(v) > 1], 'errors': errors,
              'scope_note': 'SDK roots and marker discovery only; image-only, archive-only and other formats require further classification.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'recordings': len(records), 'markers': len(findings), 'duplicates': output['same_sdk_payload_sha256'], 'errors': errors}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
