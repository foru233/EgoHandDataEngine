"""Bounded actual GUI worker replay, without rendering; source recording is read-only."""
import argparse
import cProfile
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import cv2
import numpy as np
import yaml
from PySide6.QtCore import QCoreApplication
from egohand.gui.workers import PipelineWorker
from egohand.sources.sdk_recorded import SdkRecordedStereoSource


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--profile', action='store_true')
    parser.add_argument('--legacy-policy', action='store_true', help='Reproduce pre-fix serial/four-crop policy')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    app = QCoreApplication([])
    samples = []
    arrivals = []

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
            if len(samples) >= args.frames:
                self.stop()
            return result

    detector = yaml.safe_load((ROOT / 'configs/detector.yaml').read_text(encoding='utf8'))
    detector['model_path'] = str(ROOT / detector['model_path'])
    worker = MeasuredWorker(SdkRecordedStereoSource(args.recording), offline=True,
        detector_config=detector, live_vio=True, vio_output_dir=args.output / 'replay')
    worker.error.connect(lambda message: print(message, flush=True))
    profile = cProfile.Profile()
    start = time.perf_counter()
    if args.profile:
        profile.enable()
    worker.run()
    if args.profile:
        profile.disable()
        profile.dump_stats(str(args.output / 'profile.pstats'))
    report = {'frames': len(samples), 'error': worker.last_error, 'legacy_policy': args.legacy_policy,
        'opencv_threads': cv2.getNumThreads(), 'elapsed_s': time.perf_counter() - start,
        'steady_fps': (len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else None,
        'process_p50_ms': float(np.median([s['process_ms'] for s in samples])) if samples else None,
        'samples': samples}
    (args.output / 'benchmark.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}), flush=True)
    return 0 if len(samples)==args.frames and not worker.last_error else 1


if __name__ == '__main__':
    sys.exit(main())
