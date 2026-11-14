"""Prepare a full recovery bridge candidate with corrected right fisheye keys."""
from __future__ import annotations

import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'handoff/vio_gate_optimization_20261114/tumvi_recovery_fisheye_candidate'
ORB = ROOT / 'third_party/ORB_SLAM3'
NATIVE = ROOT / 'src/egohand/vio/native'


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def ignore(folder, names):
    blocked = {'.git', '.vs', '__pycache__', 'build_msvc_initial', 'build_windows',
               'bridge_build', 'Vocabulary', 'lib'}
    return {n for n in names if n in blocked or n.lower().endswith(
        ('.obj', '.pdb', '.exe', '.lib', '.dll', '.ilk', '.ninja_log', '.ninja_deps'))}


def main():
    if DEST.exists():
        raise FileExistsError(DEST)
    shutil.copytree(ORB, DEST / 'third_party/ORB_SLAM3', ignore=ignore)
    shutil.copytree(NATIVE, DEST / 'src/egohand/vio/native', ignore=ignore)
    settings = DEST / 'third_party/ORB_SLAM3/src/Settings.cc'
    original = settings.read_text(encoding='utf-8')
    anchor = 'float cy = readParameter<float>(fSettings,"Camera2.cy",found);'
    start, end = original.index(anchor), original.index('calibration2_ = new KannalaBrandt8', original.index(anchor))
    fragment = original[start:end]
    for k in range(1, 5):
        old = f'"Camera1.k{k}"'
        if fragment.count(old) != 1:
            raise ValueError(f'unexpected key count for {old}')
        fragment = fragment.replace(old, f'"Camera2.k{k}"')
    changed = original[:start] + fragment + original[end:]
    settings.write_text(changed, encoding='utf-8', newline='')
    (DEST / 'fisheye.patch').write_text(''.join(difflib.unified_diff(
        original.splitlines(True), changed.splitlines(True), fromfile='parent/Settings.cc',
        tofile='candidate/Settings.cc')), encoding='utf-8')
    build = r'''@echo off
setlocal
set "VSLANG=1033"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
set "TASK_VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%TASK_VIO_CONDA%;%TASK_VIO_CONDA%\Scripts;%TASK_VIO_CONDA%\Library\bin;%PATH%"
set "TASK_CMAKE=%TASK_VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe"
set "TASK_ROOT=%~dp0"
"%TASK_CMAKE%" -G Ninja -S "%TASK_ROOT%third_party\ORB_SLAM3" -B "%TASK_ROOT%third_party\ORB_SLAM3\build_msvc_initial" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="/DWIN32 /D_WINDOWS /W3 /GR /EHsc /utf-8" -DCMAKE_PREFIX_PATH="%TASK_VIO_CONDA%\Library" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
if errorlevel 1 exit /b 11
python "%TASK_ROOT%fix_candidate_msvc_deps.py" "%TASK_ROOT%third_party\ORB_SLAM3\build_msvc_initial\CMakeFiles\rules.ninja"
if errorlevel 1 exit /b 15
"%TASK_CMAKE%" --build "%TASK_ROOT%third_party\ORB_SLAM3\build_msvc_initial" --target ORB_SLAM3 --parallel 2
if errorlevel 1 exit /b 12
"%TASK_CMAKE%" -G Ninja -S "%TASK_ROOT%src\egohand\vio\native" -B "%TASK_ROOT%bridge_build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="/DWIN32 /D_WINDOWS /EHsc /utf-8" -DCMAKE_PREFIX_PATH="%TASK_VIO_CONDA%\Library" -DEGOHAND_RECOVERY_BRIDGE=ON -DEGOHAND_RECOVERY_OUTPUT_NAME=egohand_vio_tumvi_fisheye_candidate -DCMAKE_POLICY_VERSION_MINIMUM=3.5
if errorlevel 1 exit /b 13
python "%TASK_ROOT%fix_candidate_msvc_deps.py" "%TASK_ROOT%bridge_build\CMakeFiles\rules.ninja"
if errorlevel 1 exit /b 15
"%TASK_CMAKE%" --build "%TASK_ROOT%bridge_build" --parallel 2
if errorlevel 1 exit /b 14
exit /b 0
'''
    (DEST / 'build_candidate.cmd').write_text(build, encoding='utf-8')
    helper = (ROOT / 'handoff/vio_gate_optimization_20261114/fisheye_calibration_candidate/fix_candidate_msvc_deps.py').read_text(encoding='utf-8')
    helper = helper.replace("base=Path(__file__).resolve().parent", "base=Path(__file__).resolve().parent")
    (DEST / 'fix_candidate_msvc_deps.py').write_text(helper, encoding='utf-8')
    files = [p for folder in (DEST / 'third_party/ORB_SLAM3', DEST / 'src/egohand/vio/native')
             for p in folder.rglob('*') if p.is_file() and 'build_' not in str(p) and 'bridge_build' not in str(p)]
    manifest = {'source_orb': str(ORB), 'source_native': str(NATIVE), 'changed': 'third_party/ORB_SLAM3/src/Settings.cc right-camera k1-k4',
                'source_files': len(files), 'production_deployed': False,
                'short_recovery_abi': 'exported_conservative_unavailable',
                'settings_parent_sha256': sha(ORB / 'src/Settings.cc'), 'settings_candidate_sha256': sha(settings),
                'source_manifest': {str(p.relative_to(DEST)): sha(p) for p in files}}
    (DEST / 'preparation.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
