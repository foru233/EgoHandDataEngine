"""Offline completeness must not force serial eyes or exhaustive empty-scene rescue."""
from types import SimpleNamespace

import pytest

from egohand.gui.workers import PipelineWorker


@pytest.mark.parametrize('kind,key', [
    ('vendor_vs030', 'vs030_rescue_crops_per_frame'),
    ('vs030_recorded', 'vs030_recorded_rescue_crops_per_frame'),
])
def test_vs030_detector_policy_is_shared_and_configurable(qtbot, monkeypatch, kind, key):
    calls = []
    source = SimpleNamespace(get_metadata=lambda: SimpleNamespace(source_type=kind, synthetic=False))
    worker = PipelineWorker(source, offline=True)
    worker.runtime[key] = 1
    monkeypatch.setattr('egohand.gui.workers.MediaPipeHandDetector',
                        lambda **options: calls.append(options) or object())
    assert len(worker._build_detectors({'algorithm': 'mediapipe'})) == 2
    assert all(c['mode'] == 'VIDEO' and c['rescue_config']['max_crops_per_frame'] == 1 for c in calls)
    assert all(c['rescue_config']['enabled'] is False for c in calls)
    calls.clear()
    worker.runtime['vs030_rescue_enabled'] = True
    worker.runtime[key] = 4
    worker._build_detectors({'algorithm': 'mediapipe'})
    assert all(c['rescue_config']['max_crops_per_frame'] == 4 for c in calls)
    assert all(c['rescue_config']['enabled'] is True for c in calls)


def test_dataset_rescue_policy_is_not_overridden(qtbot, monkeypatch):
    source = SimpleNamespace(get_metadata=lambda: SimpleNamespace(source_type='hot3d_clips', synthetic=False))
    worker = PipelineWorker(source, offline=True)
    calls = []
    monkeypatch.setattr('egohand.gui.workers.MediaPipeHandDetector',
                        lambda **options: calls.append(options) or object())
    worker._build_detectors({'algorithm': 'mediapipe'})
    assert all('rescue_config' not in c and c['mode'] == 'IMAGE' for c in calls)


@pytest.mark.parametrize('kind,enabled', [('vs030_recorded',True),('vendor_vs030',False)])
def test_explicit_recorded_missing_hand_profile_does_not_enable_live_rescue(qtbot,monkeypatch,kind,enabled):
    source=SimpleNamespace(get_metadata=lambda:SimpleNamespace(source_type=kind,synthetic=False))
    worker=PipelineWorker(source,offline=True)
    calls=[]
    monkeypatch.setattr('egohand.gui.workers.MediaPipeHandDetector',
        lambda **options:calls.append(options) or object())
    worker._build_detectors({'algorithm':'mediapipe','vs030_recorded_rescue':
        {'enabled':True,'retry_partial':True,'max_crops_per_frame':4,'partial_retry_interval_frames':3}})
    assert all(c['rescue_config']['enabled'] is enabled for c in calls)
    assert all(c['rescue_config'].get('retry_partial',False) is enabled for c in calls)


def test_recorded_offline_worker_enables_parallel_eyes_before_replay(qtbot, monkeypatch):
    source = SimpleNamespace(get_metadata=lambda: SimpleNamespace(source_type='vs030_recorded', synthetic=False))
    worker = PipelineWorker(source, offline=True, live_vio=True)
    monkeypatch.setattr(worker, '_start_source', lambda: True)
    observed = []
    monkeypatch.setattr(worker, '_run_recorded_vio',
                        lambda started: observed.append(worker.engine.config['parallel_stereo_detection']))
    worker.run()
    assert observed == [True]
    assert worker.last_error is None


def test_processing_fps_excludes_startup(qtbot, source, monkeypatch):
    from egohand.core.pipeline import EgoHandEngine
    worker = PipelineWorker(source, offline=True)
    worker.engine = EgoHandEngine()
    worker.engine.start(source)
    # Long vocabulary startup must not dilute throughput for the whole session.
    ticks = iter([100.0, 100.1, 100.2, 100.3])
    monkeypatch.setattr('egohand.gui.workers.time.perf_counter', lambda: next(ticks, 100.3))
    try:
        # Engine timings use the same time module; bypass it to isolate stats.
        from egohand.core.types import StereoHandResult
        def process(frame, **kwargs):
            worker.engine.last_frame = frame
            return StereoHandResult(frame.frame_id, frame.timestamp_ns, frame.calibration_id, frame.source_type)
        monkeypatch.setattr(worker.engine, 'process', process)
        for _ in range(2):
            frame = source.read()
            frame.metadata['acquired_perf_counter_ns'] = 0
            worker._process_frame(frame, 0.0)
        assert worker.stats['processing_fps'] == pytest.approx(5.0)
    finally:
        worker.engine.close()
