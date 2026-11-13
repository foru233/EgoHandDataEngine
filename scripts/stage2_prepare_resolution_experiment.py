"""Create immutable, independent VS030 resolution experiment inputs; never edit a recording."""
from pathlib import Path
import hashlib
import json
import re
import shutil

import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/vio_stage2_codex/perf_init'
SESSIONS = ('session_20261110_111807_489019700', 'session_20261110_111833_589265500')
BASE = ROOT / 'results/vio_stability_20261112_baseline_agent/working_inputs'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True, exist_ok=False)
    frozen = OUT / 'frozen'
    for folder in ('src', 'configs', '标定结果'):
        shutil.copytree(ROOT / folder, frozen / folder, ignore=shutil.ignore_patterns(
            '__pycache__', '*.pyc', '*.dll', '*.pdb', '*.lib', '*.exp', '*.obj', '*.exe'))
    (frozen / 'scripts').mkdir(parents=True)
    shutil.copy2(ROOT / 'scripts/run_vio_stability_experiment.py', frozen / 'scripts')
    (frozen / 'map_creat').mkdir()
    shutil.copy2(ROOT / 'map_creat/atlas_replay.py', frozen / 'map_creat')
    vocab = frozen / 'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt'
    vocab.parent.mkdir(parents=True)
    shutil.copy2(ROOT / 'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt', vocab)
    manifest = {}
    for session in SESSIONS:
        src = BASE / session
        if not src.is_dir():
            raise FileNotFoundError(src)
        for variant, scale in (('full', 1.0), ('half', .5)):
            dst = OUT / 'inputs' / session / variant
            shutil.copytree(src, dst)
            timing_path = dst / 'vio_timing.yaml'
            timing = yaml.safe_load(timing_path.read_text(encoding='utf8'))
            if timing['image_scale'] != 1.0 or timing['lattice_phase_ms'] != 105:
                raise ValueError(f'unexpected baseline configuration: {session}')
            timing['image_scale'] = scale
            timing_path.write_text(yaml.safe_dump(timing, sort_keys=True), encoding='utf8')
            settings_path = dst / 'vio_settings.yaml'
            raw_settings = settings_path.read_text(encoding='utf8')
            if scale != 1:
                keys = ('Camera1.fx', 'Camera1.fy', 'Camera1.cx', 'Camera1.cy',
                        'Camera.width', 'Camera.height')
                for key in keys:
                    pattern = rf'(?m)^({re.escape(key)}:\s*)([0-9.eE+-]+)(\s*)$'
                    def half(match):
                        value = float(match.group(2)) * scale
                        output = str(int(value)) if key in ('Camera.width', 'Camera.height') else repr(value)
                        return match.group(1) + output + match.group(3)
                    raw_settings, count = re.subn(pattern, half, raw_settings)
                    if count != 1:
                        raise ValueError(f'{key}: expected exactly one scalar, got {count}')
                settings_path.write_text(raw_settings, encoding='utf8')
            raw_source = {str(p.relative_to(src)): sha(p) for p in (src / 'raw').rglob('*') if p.is_file()}
            raw_copy = {str(p.relative_to(dst)): sha(p) for p in (dst / 'raw').rglob('*') if p.is_file()}
            if raw_source != raw_copy:
                raise ValueError(f'raw copy mismatch: {dst}')
            manifest[f'{session}/{variant}'] = {
                'source': str(src), 'input': str(dst), 'scale': scale, 'phase_ms': 105,
                'raw_sha256': raw_copy,
                'config_sha256': {p.name: sha(p) for p in dst.glob('*.yaml')},
            }
    source_files = {str(p.relative_to(frozen)): sha(p) for p in frozen.rglob('*')
                    if p.is_file() and p.suffix.lower() in ('.py', '.yaml', '.json')}
    manifest['_frozen'] = {'root': str(frozen), 'source_sha256': source_files,
                           'vocabulary_sha256': sha(vocab),
                           'bridge': str(ROOT / 'src/egohand/vio/native/bin/egohand_vio_cheirality_v3.dll'),
                           'bridge_sha256': sha(ROOT / 'src/egohand/vio/native/bin/egohand_vio_cheirality_v3.dll')}
    (OUT / 'preparation.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf8')
    print(f'Created {OUT}; {len(source_files)} frozen sources; 4 copied inputs.')


if __name__ == '__main__':
    main()
