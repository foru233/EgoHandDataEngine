"""Check loaded native fisheye parameters, not just the input YAML."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = ROOT / 'samples10-15gb/VIO/TUM-VI/calibration/ORB_SLAM3_TUM-VI.yaml'
VOCAB = ROOT / 'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt'


def digest(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--library', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--probe', action='store_true')
    args = parser.parse_args()
    if args.probe:
        sys.path.insert(0, str(ROOT / 'src'))
        from egohand.vio.native_bridge import NativeVioBridge
        with NativeVioBridge(VOCAB, SETTINGS, library_path=args.library):
            pass
        return
    if args.output is None:
        parser.error('--output is required')
    import yaml
    env = os.environ.copy()
    for key in list(env):
        if key.startswith('EGOHAND_VIO_'):
            env.pop(key)
    env['EGOHAND_VIO_STEREO_ONLY'] = '0'
    result = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--probe', '--library',
                             str(args.library.resolve())], env=env, text=True, encoding='utf-8',
                             errors='replace', capture_output=True, timeout=120)
    match = re.search(r'Camera 2 parameters \(Kannala-Brandt\)?:\s*\[([^\]]+)\]', result.stdout)
    if result.returncode or not match:
        raise RuntimeError(f'native settings probe failed: code={result.returncode}\n{result.stdout}\n{result.stderr}')
    actual = [float(x) for x in match.group(1).split()]
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('tag:yaml.org,2002:opencv-matrix', lambda loader, node: loader.construct_mapping(node, deep=True))
    config = yaml.load('\n'.join(SETTINGS.read_text(encoding='utf-8').splitlines()[1:]), Loader=Loader)
    expected = [float(config[f'Camera2.{k}']) for k in ('fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'k3', 'k4')]
    tolerances = [0.001] * 4 + [5e-8] * 4  # native ostream defaults to six significant digits
    good = len(actual) == 8 and all(abs(a - b) <= tol for a, b, tol in zip(actual, expected, tolerances))
    report = {'library': str(args.library.resolve()), 'library_sha256': digest(args.library),
              'settings_sha256': digest(SETTINGS), 'loaded_camera2': actual,
              'expected_camera2': expected, 'pass': good,
              'scope': 'native settings loading and bridge ABI only; no tracking or full ABI safety certification'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    if not good:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
