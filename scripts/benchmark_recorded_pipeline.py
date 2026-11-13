"""Bounded actual GUI worker replay, without rendering; source recording is read-only."""
import argparse
import cProfile
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import cv2
import numpy as np
import psutil
import yaml
from PySide6.QtCore import QCoreApplication
from egohand.gui.workers import PipelineWorker
from egohand.sources.sdk_recorded import SdkRecordedStereoSource


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def snapshot_files(paths):
    """Hash every expected path; missing files remain visible as null."""
    return {str(path): file_sha256(path) if path.is_file() else None for path in paths}


def source_snapshot():
    paths = sorted((ROOT / 'src/egohand').rglob('*.py')) + [Path(__file__).resolve()]
    return snapshot_files(paths)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--profile', action='store_true')
    parser.add_argument('--legacy-policy', action='store_true', help='Reproduce pre-fix serial/four-crop policy')
    parser.add_argument('--recovery-v4', action='store_true', help='Use the same candidate profile as customer recording replay')
    parser.add_argument('--opencv-threads', type=int, help='Experiment-only OpenCV thread count (must be positive)')
    args = parser.parse_args()
    if args.opencv_threads is not None:
        if args.opencv_threads < 1:
            parser.error('--opencv-threads must be positive')
        cv2.setNumThreads(args.opencv_threads)
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = ('manifest.json', 'vio_timing.yaml', 'raw/sdk_records.bin',
              'raw/sdk_index.jsonl', 'raw/imu_records.bin')
    input_paths = {name: args.recording / name for name in inputs}
    def input_snapshot():
        return {name: file_sha256(path) if path.is_file() else None for name, path in input_paths.items()}
    input_hashes_before = input_snapshot()
    source_hashes_before = source_snapshot()
    app = QCoreApplication([])
    samples = []
    arrivals = []
    resources = []
    root_process = psutil.Process()

    def resource_snapshot(frame_id):
        processes = []
        for process in [root_process, *root_process.children(recursive=True)]:
            try:
                with process.oneshot():
                    cpu = process.cpu_times()
                    processes.append({'pid': process.pid, 'name': process.name(),
                        'rss_bytes': process.memory_info().rss, 'threads': process.num_threads(),
                        'cpu_seconds': cpu.user + cpu.system})
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return {'frame': frame_id, 'host_monotonic_ns': time.perf_counter_ns(),
                'processes': processes}

    class MeasuredWorker(PipelineWorker):
        def _new_detectors(self):
            if args.legacy_policy:
                self.runtime['vs030_rescue_enabled'] = True
                self.runtime['vs030_recorded_rescue_crops_per_frame'] = 4
            super()._new_detectors()
            if args.legacy_policy:
                self.engine.config['parallel_stereo_detection'] = False

        def _process_frame(self, frame, started):
            arrivals.append(time.perf_counter())
            result = super()._process_frame(frame, started)
            d = result.diagnostics
            samples.append({'frame': result.frame_id, 'process_ms': result.processing_time_ms,
                'left_ms': d.get('left_detection_ms'), 'right_ms': d.get('right_detection_ms'),
                'left_crops': d.get('left_detection_details', {}).get('crop_attempts'),
                'right_crops': d.get('right_detection_details', {}).get('crop_attempts'),
                'left_hands': d.get('detected_hands_left'), 'right_hands': d.get('detected_hands_right')})
            if len(samples) == 1 or len(samples) % 30 == 0:
                resources.append(resource_snapshot(result.frame_id))
            if len(samples) >= args.frames:
                self.stop()
            return result

    detector = yaml.safe_load((ROOT / 'configs/detector.yaml').read_text(encoding='utf8'))
    detector['model_path'] = str(ROOT / detector['model_path'])
    config_paths = (ROOT/'configs/detector.yaml', ROOT/'configs/app.yaml',
                    ROOT/'configs/stereo.yaml', Path(detector['model_path']))
    def config_snapshot():
        return {str(path.relative_to(ROOT)): file_sha256(path) if path.is_file() else None
                for path in config_paths}
    config_hashes = config_snapshot()
    options = {}
    provenance = None
    recording_config = None
    candidate = None
    if args.recovery_v4:
        from egohand.vio.live_recovery import LiveRecoveryProfile
        candidate = LiveRecoveryProfile()
        binding = candidate.recording_options(args.recording)
        recording_config = binding['config_snapshot'].describe()
        options = {'vio_config_snapshot': binding['config_snapshot'],
                   'vio_library_path': binding['library_path'],
                   'vio_environment': binding['environment'], 'calibration': candidate.calibration}
        provenance = candidate.provenance
    worker = MeasuredWorker(SdkRecordedStereoSource(args.recording), offline=True,
        detector_config=detector, live_vio=True, vio_output_dir=args.output / 'replay', **options)
    worker.error.connect(lambda message: print(message, flush=True))
    profile = cProfile.Profile()
    start = time.perf_counter()
    if args.profile:
        profile.enable()
    worker.run()
    if args.profile:
        profile.disable()
        profile.dump_stats(str(args.output / 'profile.pstats'))
    elapsed_s = time.perf_counter() - start
    input_hashes = input_snapshot()
    source_hashes_after = source_snapshot()
    config_hashes_after = config_snapshot()
    candidate_hashes_after = snapshot_files(candidate.paths.values()) if candidate is not None else None
    candidate_unchanged = (candidate is None or all(
        candidate_hashes_after[str(path)] == candidate.data['files'][key]['sha256']
        for key, path in candidate.paths.items()))
    inputs_unchanged = (input_hashes_before == input_hashes and
                        all(value is not None for value in input_hashes.values()))
    configuration_unchanged = (config_hashes == config_hashes_after and
                               all(value is not None for value in config_hashes.values()) and candidate_unchanged)
    sources_unchanged = source_hashes_before == source_hashes_after
    report = {'frames': len(samples), 'error': worker.last_error, 'legacy_policy': args.legacy_policy,
        'recovery_v4': args.recovery_v4, 'profiled': args.profile, 'profile_provenance': provenance,
        'recording_config': recording_config, 'input_sha256_before': input_hashes_before,
        'input_sha256_after': input_hashes, 'inputs_unchanged': inputs_unchanged,
        'opencv_threads': cv2.getNumThreads(), 'configuration_sha256': config_hashes,
        'configuration_sha256_after': config_hashes_after, 'candidate_sha256_after': candidate_hashes_after,
        'configuration_unchanged': configuration_unchanged,
        'source_sha256_before': source_hashes_before, 'source_sha256_after': source_hashes_after,
        'sources_unchanged': sources_unchanged, 'elapsed_s': elapsed_s,
        'resource_sampling': {'clock': 'host_perf_counter_ns', 'every_n_frames': 30,
            'scope': 'GUI worker process and currently visible child processes',
            'rss_is_not_unique_memory': True, 'gpu_usage_unavailable': True,
            'snapshots': resources},
        'steady_fps': (len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else None,
        'process_p50_ms': float(np.median([s['process_ms'] for s in samples])) if samples else None,
        'samples': samples}
    (args.output / 'benchmark.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps({k:v for k,v in report.items()
                      if k not in ('samples', 'profile_provenance', 'recording_config', 'resource_sampling',
                                   'source_sha256_before', 'source_sha256_after')}), flush=True)
    return 0 if (len(samples)==args.frames and not worker.last_error and inputs_unchanged
                 and configuration_unchanged and sources_unchanged) else 1


if __name__ == '__main__':
    sys.exit(main())
