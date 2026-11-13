"""Shared services must preserve both shells' session and image semantics."""
import json
from dataclasses import replace
from threading import Event
from types import SimpleNamespace

import numpy as np

from egohand.calibration.loader import save_calibration
from egohand.core.pipeline import EgoHandEngine
from egohand.gui.shared.presentation import display_stereo, session_state
from egohand.gui.shared.session import SessionController
from egohand.gui.shared.sources import open_recording_source
from egohand.sources.sdk_recorded import SdkRecordedStereoSource
from egohand.storage.sdk_recording import PairedFrame


def test_session_state_stopping_overrides_pause():
    worker = SimpleNamespace(isRunning=lambda: True, stop_event=Event())
    assert session_state(worker, paused=True).text == '已暂停'
    worker.stop_event.set()
    state = session_state(worker, paused=True)
    assert state.text == '正在停止' and not state.active


def test_shared_overlay_uses_same_pair_policy_for_both_cameras():
    calls = []
    view = SimpleNamespace(display=lambda *a, **kw: calls.append((a, kw)))
    result = SimpleNamespace(diagnostics={'association_quality': {'accepted_pairs': [1]}},
                             left_detections=['left'], right_detections=['right'], hands=[])
    frame = SimpleNamespace(left_image='L', right_image='R')
    display_stereo(frame, result, view, view, epipolar_axis=0)
    assert [call[0][0] for call in calls] == ['L', 'R']
    assert all(call[1]['accepted_pairs'] == [1] for call in calls)
    assert all(call[0][-1] == 0 for call in calls)
    result.diagnostics['result_kind'] = 'official_ground_truth'
    display_stereo(frame, result, view, view)
    assert all(call[1]['accepted_pairs'] is None for call in calls[-2:])


def test_controller_lifecycle_and_stale_signal_isolation(qtbot):
    from egohand.sources.synthetic import SyntheticStereoSource

    controller = SessionController()
    errors = []
    controller.error.connect(errors.append)
    try:
        old = controller.start(SyntheticStereoSource('two_static', frames=3), offline=True)
        qtbot.waitUntil(lambda: bool(old.output), timeout=6000)
        assert controller.take_latest()[1].frame_id >= 0
        assert controller.stop()
        current = controller.start(SyntheticStereoSource('two_static', frames=3), offline=True)
        # A late signal from an old worker cannot change the new session.
        old.error.emit('stale')
        qtbot.waitUntil(lambda: bool(current.output), timeout=6000)
        assert not errors
        assert controller.stop()
        assert not current.isRunning() and not current.capture_thread.is_alive()
    finally:
        controller.stop()


def test_debug_gui_ignores_queued_vio_from_old_source(qtbot, monkeypatch):
    from PySide6.QtCore import Qt
    from egohand.gui.main_window import MainWindow
    from egohand.gui.workers import PipelineWorker

    window = MainWindow()
    qtbot.addWidget(window)
    updates = []
    monkeypatch.setattr(window.vio_panel, 'update_live', updates.append)
    monkeypatch.setattr(PipelineWorker, 'start', lambda self: None)
    try:
        window.source_panel.kind.setCurrentIndex(0)
        window.start_pipeline()
        old = window.worker
        old.vio_status.disconnect(window.on_vio_status)
        old.vio_status.connect(window.on_vio_status, Qt.QueuedConnection)
        old.vio_status.emit({'type': 'old_session'})
        window.source_panel.kind.setCurrentIndex(3)
        qtbot.wait(30)
        assert updates == []
    finally:
        window.close()


def _sdk_recording(tmp_path, calibration):
    (tmp_path / 'raw').mkdir()
    (tmp_path / 'raw/sdk_records.bin').write_bytes(b'raw packet placeholder')
    save_calibration(calibration, tmp_path / 'calibration.yaml')
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'sdk_records': 'raw/sdk_records.bin', 'calibration': 'calibration.yaml',
        'metadata': {'source_type': 'vendor_vs030', 'contains_imu': True},
    }), encoding='utf-8')


def test_sdk_adapter_seek_and_device_metadata(tmp_path, calibration, monkeypatch):
    from egohand.sources import sdk_recorded

    _sdk_recording(tmp_path, calibration)
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    pairs = [PairedFrame(i, 10+i, 1000+i*33333, image, image, 100+i, 101+i, 110+i, 111+i)
             for i in range(3)]
    monkeypatch.setattr(sdk_recorded, 'iter_paired_frames', lambda *a: iter(pairs))
    source = open_recording_source(tmp_path)
    assert isinstance(source, SdkRecordedStereoSource)
    source.open()
    source.start()
    try:
        frame = source.read()
        assert frame.frame_id == 0 and frame.timestamp_ns == 1_000_000
        assert frame.left_timestamp_ns is None and not frame.synchronized
        assert frame.metadata['hardware_sync_precision_verified'] is False
        assert source.get_metadata().source_type == 'vs030_recorded'
        source.seek(2)
        assert source.read().frame_id == 2
        assert source.read() is None
        source.seek(0)
        assert source.read().frame_id == 0
        source.interrupt_read()
        assert source.read() is None
    finally:
        source.close()


def test_sdk_hand_geometry_retains_existing_user_confirmed_sync(tmp_path, calibration, source):
    # Real-style metadata with controlled projection pixels exercises the gate,
    # without asserting accuracy of a physical camera from synthetic test data.
    real_calibration = replace(calibration, synthetic=False, calibration_source='controlled_test_fixture')
    _sdk_recording(tmp_path, real_calibration)
    adapter = SdkRecordedStereoSource(tmp_path)
    adapter.open()
    original = source.read()
    pair = PairedFrame(0, 42, 1000, original.left_image, original.right_image, 10, 11, 12, 13)
    frame = adapter.frame_from_pair(pair)
    frame.left_detections = [replace(d, origin='controlled_test_projection') for d in original.left_detections]
    frame.right_detections = [replace(d, origin='controlled_test_projection') for d in original.right_detections]
    engine = EgoHandEngine(config={'vs030_user_confirmed_sync': True}, calibration=real_calibration)
    result = engine.process(frame)
    assert result.diagnostics['sync_quality'] == 'USER_CONFIRMED_SAME_BUNDLE'
    assert not result.diagnostics['hardware_sync_precision_verified']
    assert result.reconstruction_status != '2D Only'
    assert sum(h.valid_joint_count for h in result.hands) > 0
    adapter.close()
