"""Measure actual independent recording phase verification with fixed provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.vio.live_recovery import LiveRecoveryProfile
from egohand.vio.session_phase import prepare_recording_phase


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    profile = LiveRecoveryProfile()
    config = profile.recording_options(args.recording)['config_snapshot']
    paths = [args.recording / name for name in
             ('manifest.json','vio_timing.yaml','raw/sdk_records.bin','raw/imu_records.bin')]
    before = {p.name: sha(p) for p in paths}
    args.output.mkdir(parents=True, exist_ok=False)
    start = time.perf_counter()
    prepared = prepare_recording_phase(args.recording, config, args.output)
    elapsed = time.perf_counter() - start
    after = {p.name: sha(p) for p in paths}
    phase = json.loads((args.output/'session_phase_preparation.json').read_text(encoding='utf8'))
    report = {'elapsed_s': elapsed, 'verified': prepared.timing.get('session_phase_verified'),
              'phase_ms': prepared.timing.get('lattice_phase_ms'), 'input_sha256_before': before,
              'input_sha256_after': after, 'inputs_unchanged': before == after,
              'configuration': config.describe(), 'verification': phase['verification'],
              'limitation':'phase validation only; not absolute pose accuracy or live performance'}
    (args.output/'measurement.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('configuration','verification')},indent=2))
    return 0 if report['verified'] is True and before == after else 1


if __name__ == '__main__':
    sys.exit(main())
