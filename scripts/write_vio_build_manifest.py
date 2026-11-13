"""Record the exact local source tree and CMake inputs used for a bridge build."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import datetime

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--library', required=True, type=Path)
    p.add_argument('--build-dir', required=True, type=Path)
    a = p.parse_args()
    paths = sorted(set(
        list((ROOT/'third_party/ORB_SLAM3/src').glob('*.cc')) +
        list((ROOT/'third_party/ORB_SLAM3/include').rglob('*.h')) +
        list((ROOT/'src/egohand/vio/native').glob('*.cpp')) +
        list((ROOT/'src/egohand/vio/native').glob('*.h')) +
        [ROOT/'src/egohand/vio/native/CMakeLists.txt']))
    sources = {str(p.relative_to(ROOT)): digest(p) for p in paths}
    capabilities = ['stereo_inertial', 'atlas_build', 'atlas_localize', 'body_state',
                    'recovery_callback', 'optimization_diagnostics', 'gauge_fix_opt_in',
                    'world_frame_revision']
    if 'egohand_vio_snapshot_size' in (ROOT/'src/egohand/vio/native/vio_bridge.cpp').read_text(encoding='utf8'):
        capabilities.extend(['frame_snapshot_v1', 'stereo_diagnostic'])
    if 'egohand_vio_numerical_fault_count' in (ROOT/'src/egohand/vio/native/vio_bridge.cpp').read_text(encoding='utf8'):
        capabilities.append('numerical_fault_fail_closed')
    if 'egohand_vio_invalid_depth_removed' in (ROOT/'src/egohand/vio/native/vio_bridge.cpp').read_text(encoding='utf8'):
        capabilities.append('cheirality_guard')
    manifest = {'schema_version': 1, 'built_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'library': a.library.name, 'sha256': digest(a.library),
                'source_tree_sha256': hashlib.sha256(json.dumps(sources, sort_keys=True).encode()).hexdigest(),
                'git_head': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
                'sources': sources, 'cmake_cache_sha256': digest(a.build_dir/'CMakeCache.txt'),
                'abi_contract': 'legacy EgohandVioResult unchanged; result_size validated at load',
                'capabilities': capabilities}
    a.library.with_suffix('.build.json').write_text(json.dumps(manifest, indent=2), encoding='utf8')
    print(json.dumps({k: v for k, v in manifest.items() if k != 'sources'}))


if __name__ == '__main__':
    main()
