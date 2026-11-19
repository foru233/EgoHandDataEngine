"""Read original SDK frames, emitting only camera poses and stereo observations."""
import argparse
import json
from pathlib import Path
import sys

from joint_work_paths import PROJECT
sys.path[:0] = [str(PROJECT / 'src'), str(PROJECT / 'userrr')]

import yaml
from PySide6.QtCore import QCoreApplication

from egohand.calibration.loader import load_calibration
from egohand.gui.workers import PipelineWorker
from egohand.sources.sdk_recorded import SdkRecordedStereoSource
from egohand.vio.config_snapshot import resolve_recording_config
from egohand.vio.hand_world import calibration_geometry_fingerprint
from egohand.vio.session_phase import prepare_recording_phase
from offline_recovery import OfflineRecoveryProfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    app = QCoreApplication([])
    config = yaml.safe_load((PROJECT / 'configs/stereo.yaml').read_text(encoding='utf8'))
    detector = yaml.safe_load((PROJECT / 'configs/detector.yaml').read_text(encoding='utf8'))
    detector['model_path'] = str(PROJECT / detector['model_path'])
    try:
        profile = OfflineRecoveryProfile()
    except FileNotFoundError:
        profile = None
    expected = calibration_geometry_fingerprint(load_calibration(args.recording / 'calibration.yaml'))
    profile_geometry = calibration_geometry_fingerprint(
        load_calibration(profile.paths['calibration'], kalibr_left_camera='cam0')) if profile else None
    use_profile = expected == profile_geometry
    if use_profile:
        options = profile.worker_options(args.recording, args.output, isolated_environment=True)
    else:
        snapshot = resolve_recording_config(args.recording, timing_override={'lattice_phase_mode': 'session_auto'})
        snapshot.validate_geometry()
        calibration = load_calibration(snapshot.calibration, kalibr_left_camera='cam0')
        if calibration_geometry_fingerprint(calibration) != expected:
            raise ValueError('原始图像标定与录制相机定位快照不匹配')
        args.output.mkdir(parents=True, exist_ok=False)
        options = dict(calibration=calibration, vio_config_snapshot=snapshot)
    timing_work = args.output / 'timing_preflight'
    timing_work.mkdir()
    mode = 'stereo_inertial'
    fallback_reason = None
    try:
        options['vio_config_snapshot'] = prepare_recording_phase(
            args.recording, options['vio_config_snapshot'], timing_work, on_progress=lambda text: print(text, flush=True))
    except ValueError as error:
        if not str(error).startswith('录制时序无法确认'):
            raise
        mode = 'stereo'
        fallback_reason = str(error)
        options['coordinated_stereo'] = use_profile
        print('TIMING Visual-only stereo fallback:', fallback_reason, flush=True)
    worker = PipelineWorker(SdkRecordedStereoSource(args.recording), config=config,
                            detector_config=detector, offline=True, live_vio=True,
                            vio_mode=mode, vio_output_dir=args.output, **options)
    worker.run()
    app.processEvents()
    if worker.last_error:
        raise RuntimeError(worker.last_error)
    report = json.loads((args.output / 'reprocessing.json').read_text(encoding='utf8'))
    if report['status'] != 'completed' or report['result_count'] < 1:
        raise ValueError('Raw recording did not produce complete frame observations')
    (args.output / 'measurement_manifest.json').write_text(json.dumps(
        dict(mode=mode, calibration_source='matched_delivery_profile' if use_profile else 'recording_snapshot',
             imu_fallback_reason=fallback_reason, result_count=report['result_count'],
             reconstruction_performed=False), ensure_ascii=False, indent=2), encoding='utf8')
    print('MEASUREMENTS', report['result_count'], flush=True)


if __name__ == '__main__':
    main()
