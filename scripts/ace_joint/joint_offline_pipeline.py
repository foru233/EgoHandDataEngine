"""Process an original SDK recording with one joint body-and-hand reconstruction."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from joint_work_paths import ACE, JOINT_WORK, MANO, PROJECT, SCRIPT_DIR
from ace_prediction_cache import fingerprint, lookup, publish

sys.path.insert(0, str(PROJECT / 'src'))
from egohand.body.smpl_mount import DEFAULT_CAMERA_FOREHEAD_OFFSET_MM, camera_forehead_offset_m
from egohand.body.smpl_runtime import STANDING_FOREHEAD_HEIGHT_M

GIB = 1024**3


def runtime_python(variable):
    value=os.environ.get(variable,sys.executable)
    found=shutil.which(value)
    if not found:raise FileNotFoundError(f'{variable}: Python interpreter not found')
    return Path(found).absolute()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera-forehead-offset-mm', type=float, default=DEFAULT_CAMERA_FOREHEAD_OFFSET_MM)
    parser.add_argument('--initial-camera-height-m', type=float, default=STANDING_FOREHEAD_HEIGHT_M)
    parser.add_argument('--iterations', type=int, default=150)
    parser.add_argument('--no-ace-cache', action='store_true')
    args = parser.parse_args()
    recording, output = args.recording.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError('Choose a new output directory')
    if recording == output or recording in output.parents:
        raise ValueError('Output must be outside the original recording')
    camera_forehead_offset_m(args.camera_forehead_offset_mm)
    if not .3 <= args.initial_camera_height_m <= 2.1 or args.iterations < 1:
        raise ValueError('Invalid camera height or iteration count')
    source_manifest = json.loads((recording / 'manifest.json').read_text(encoding='utf-8-sig'))
    counts = source_manifest.get('sdk_count_by_sink') or {}
    frames = min(int(counts.get(str(sink), 0)) for sink in (4, 5))
    if not frames:
        raise ValueError('Recording has no VS030 stereo frames')
    for required in (ACE/'options/ace_ego_hand_k.yml', ACE/'checkpoints/ace_ego_hand_k.pt', MANO,
                     PROJECT/'models/smpl/SMPL_NEUTRAL.pkl'):
        if not required.exists():raise FileNotFoundError(f'Missing joint reconstruction asset: {required}')
    ace_python=runtime_python('EGOHAND_JOINT_ACE_PYTHON')
    body_python=runtime_python('EGOHAND_JOINT_BODY_PYTHON')
    output.parent.mkdir(parents=True, exist_ok=True)
    JOINT_WORK.mkdir(parents=True, exist_ok=True)
    scratch_bytes = GIB + frames * 3_600_000
    output_bytes = GIB + frames * 1_950_000
    same_drive = os.stat(JOINT_WORK).st_dev == os.stat(output.parent).st_dev
    if shutil.disk_usage(JOINT_WORK).free < scratch_bytes + (output_bytes if same_drive else 0):
        raise OSError('Not enough space in the joint workspace for this recording and its caches')
    if not same_drive and shutil.disk_usage(output.parent).free < output_bytes:
        raise OSError('Not enough space on the result drive')
    work = JOINT_WORK / f'job_{time.time_ns()}'
    work.mkdir()
    env = {**os.environ, 'PYTHONIOENCODING': 'utf-8', 'PYTHONDONTWRITEBYTECODE': '1'}

    def run(python, script, *parameters):
        path = SCRIPT_DIR / script
        print('STAGE', path.name, flush=True)
        subprocess.run([str(python), str(path), *map(str, parameters)], cwd=PROJECT, env=env, check=True)

    measurements, video, observations = (work / name for name in ('measurements', 'input', 'observations'))
    camera, hands, fit = (work / name for name in ('camera.npz', 'hand_initialization', 'fit'))
    run(sys.executable, 'extract_joint_measurements.py', recording, measurements)
    run(body_python, 'prepare_neutral_joint_seed.py', measurements, camera,
        '--initial-camera-height-m', args.initial_camera_height_m)
    run(sys.executable, 'export_recording.py', '--recording', recording, '--output', video,
        '--calibration', measurements / 'processing_calibration.yaml')
    run(sys.executable, 'prepare_recording_observations.py', '--source', measurements,
        '--input', video, '--recording', recording, '--output', observations)
    ace_window=81
    frame_count=json.loads((video/'manifest.json').read_text(encoding='utf8'))['frames']
    cache_key=None
    cached=None
    if not args.no_ace_cache:
        print('STAGE ace_prediction_cache.py',flush=True)
        try:
            cache_key=fingerprint(video,ace_python,ace_window)
            cached=lookup(cache_key,frame_count,ace_window)
        except (OSError,ValueError,KeyError,subprocess.CalledProcessError) as exc:
            print(f'ACE CACHE BYPASS: {exc}',flush=True)
    ace_predictions=cached or work/'ace'
    if cached:
        print('ACE CACHE HIT',cache_key,flush=True)
    else:
        run(ace_python, 'encode_windows.py', '--input', video, '--out', work / 'latents', '--window', ace_window, '--overlap', 0)
        run(ace_python, 'infer_latents.py', '--latents', work / 'latents', '--out', ace_predictions)
        if cache_key:
            print('STAGE ace_prediction_cache.py',flush=True)
            try:
                if fingerprint(video,ace_python,ace_window)==cache_key:
                    publish(ace_predictions,cache_key,frame_count,ace_window)
                else:print('ACE CACHE BYPASS: inference inputs changed during the run',flush=True)
            except (OSError,ValueError,KeyError,subprocess.CalledProcessError) as exc:
                print(f'ACE CACHE BYPASS: {exc}',flush=True)
    run(ace_python, 'initialize_joint_hands.py', '--ace-run', ace_predictions,
        '--observations', observations / 'observations.npz', '--output', hands)
    run(ace_python, 'joint_body_mano.py', '--seed', measurements, '--hands', hands, '--camera', camera,
        '--output', fit, '--iterations', args.iterations, '--temporal-weight', .35, '--finger-prior', .3,
        '--camera-forehead-offset-mm', args.camera_forehead_offset_mm)
    run(sys.executable, PROJECT / 'scripts/build_saved_replay_cache.py', measurements)
    run(body_python, 'package_joint_replay.py', '--run', fit, '--source', measurements, '--output', output,
        '--share-image-cache', '--cache-source', measurements)
    run(sys.executable, 'build_joint_fast_replay.py', output, '--source', measurements)
    manifest = json.loads((output / 'unified_smpl_manifest.json').read_text(encoding='utf8'))
    if (manifest.get('complete_reconstruction_passes') != 1 or manifest.get('status') != 'completed'
            or not manifest.get('valid')):
        raise RuntimeError('Joint reconstruction did not complete')
    report = dict(status='completed', recording=str(recording), work=str(work), complete_reconstruction_passes=1,
                  initialization='neutral_smpl_and_ace_stereo', previous_reconstruction_required=False,
                  camera_observations=json.loads((measurements/'measurement_manifest.json').read_text(encoding='utf8')),
                  ace_cache_reused=cached is not None, ace_cache_key=cache_key,
                  camera_forehead_offset_mm=args.camera_forehead_offset_mm,
                  initial_camera_height_m=args.initial_camera_height_m, standalone_fast_replay=True)
    with (output / 'pipeline.json').open('x', encoding='utf8') as stream:
        json.dump(report, stream, indent=2)
    print('COMPLETED', output, flush=True)


if __name__ == '__main__':
    main()
