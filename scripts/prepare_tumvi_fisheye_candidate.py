"""Prepare a non-production ORB-SLAM3 candidate with right fisheye calibration fixed."""
from __future__ import annotations

import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'handoff/goal_delivery_20261113/sim3_candidate'
DEST = ROOT / 'handoff/vio_gate_optimization_20261114/fisheye_calibration_candidate'


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def ignore(folder, names):
    excluded = {'.git', '.vs', '__pycache__', 'build_msvc_initial', 'bridge_build', 'build_windows', 'Vocabulary'}
    return {name for name in names if name in excluded or name.lower().endswith(
        ('.obj', '.pdb', '.exe', '.lib', '.dll', '.ilk', '.ninja_log', '.ninja_deps'))}


def main():
    if DEST.exists():
        raise FileExistsError(DEST)
    for folder in ('third_party/ORB_SLAM3', 'src'):
        shutil.copytree(SOURCE / folder, DEST / folder, ignore=ignore)
    path = DEST / 'third_party/ORB_SLAM3/src/Settings.cc'
    original = path.read_text(encoding='utf-8')
    anchor = 'float cy = readParameter<float>(fSettings,"Camera2.cy",found);'
    start = original.index(anchor)
    end = original.index('calibration2_ = new KannalaBrandt8', start)
    fragment = original[start:end]
    for k in range(1, 5):
        old = f'"Camera1.k{k}"'
        if fragment.count(old) != 1:
            raise RuntimeError(f'unexpected right-camera calibration reader: {old}')
        fragment = fragment.replace(old, f'"Camera2.k{k}"')
    changed = original[:start] + fragment + original[end:]
    path.write_text(changed, encoding='utf-8', newline='')
    (DEST / 'fisheye.patch').write_text(''.join(difflib.unified_diff(
        original.splitlines(True), changed.splitlines(True), fromfile='parent/Settings.cc',
        tofile='candidate/Settings.cc')), encoding='utf-8')
    helper = (ROOT / 'handoff/goal_delivery_20261113/fix_sim3_msvc_deps.py').read_text(encoding='utf-8')
    helper = helper.replace("base=Path(__file__).resolve().parent/'sim3_candidate'", "base=Path(__file__).resolve().parent")
    (DEST / 'fix_candidate_msvc_deps.py').write_text(helper, encoding='utf-8')
    build = (SOURCE / 'build_candidate.cmd').read_text(encoding='utf-8')
    build = build.replace('egohand_vio_sim3_candidate', 'egohand_vio_fisheye_candidate')
    build = build.replace('..\\fix_sim3_msvc_deps.py', 'fix_candidate_msvc_deps.py')
    (DEST / 'build_candidate.cmd').write_text(build, encoding='utf-8')
    manifest = {'parent': str(SOURCE), 'parent_settings_sha256': sha(SOURCE / 'third_party/ORB_SLAM3/src/Settings.cc'),
                'candidate_settings_sha256': sha(path), 'changed_scope': 'right fisheye distortion reader only',
                'production_deployed': False,
                'source_sha256': {str(p.relative_to(DEST)): sha(p) for folder in (DEST / 'third_party/ORB_SLAM3', DEST / 'src')
                                  for p in folder.rglob('*') if p.is_file()}}
    (DEST / 'preparation.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(DEST)


if __name__ == '__main__':
    main()
