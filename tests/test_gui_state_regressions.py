"""GUI 异常路径回归：会话切换、异步反馈和回放图像必须保持一致。"""
import threading

import numpy as np
import pytest
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFileDialog

from egohand.gui.main_window import MainWindow
from egohand.gui.workers import PipelineWorker
from egohand.storage.recorder import Recorder


@pytest.fixture
def window(qtbot):
    w = MainWindow()
    qtbot.addWidget(w)
    yield w
    w.stop_pipeline()
    w.close()


def test_failed_stop_keeps_source_and_session(window, monkeypatch, calibration):
    w = window
    original = w.source_panel.kind.currentIndex()
    w.loaded_calibration = calibration
    w.saved_results = [object()]
    saved = w.saved_results
    with monkeypatch.context() as patch:
        patch.setattr(w, 'stop_pipeline', lambda: False)
        w.source_panel.kind.setCurrentIndex(0)
        assert w.source_panel.kind.currentIndex() == original
        assert w.loaded_calibration is calibration
        assert w.saved_results is saved


@pytest.mark.parametrize('action', ['load_model', 'load_calibration', 'open_results'])
def test_failed_stop_cancels_file_actions(window, monkeypatch, calibration, action):
    w = window
    w.loaded_calibration = calibration
    model = w.model_path
    saved = w.saved_results
    monkeypatch.setattr(QFileDialog, 'getOpenFileName', lambda *a: ('missing-file', ''))
    monkeypatch.setattr(QFileDialog, 'getExistingDirectory', lambda *a: 'missing-directory')
    with monkeypatch.context() as patch:
        patch.setattr(w, 'stop_pipeline', lambda: False)
        getattr(w, action)()
        assert w.model_path == model
        assert w.loaded_calibration is calibration
        assert w.saved_results is saved
        assert not w.banner.text().startswith('Error')


def test_failed_worker_resets_controls(window, qtbot, monkeypatch):
    from egohand.sources.vendor_xdsdk import XdSdkStereoSource

    def fail_open(self):
        raise RuntimeError('测试相机连接失败')

    monkeypatch.setattr(XdSdkStereoSource, 'open', fail_open)
    w = window
    w.source_panel.kind.setCurrentIndex(3)
    w.start_pipeline()
    qtbot.waitUntil(lambda: not w.worker.isRunning(), timeout=5000)
    qtbot.wait(50)
    assert not w.pause_button.isEnabled()
    assert not w.record_button.isEnabled()
    assert '测试相机连接失败' in w.banner.text()


def test_stop_during_camera_discovery_does_not_block_gui(window, qtbot, monkeypatch):
    from egohand.sources.vendor_xdsdk import XdSdkStereoSource

    discovery_started = threading.Event()
    release_discovery = threading.Event()
    stream_starts = []

    def open_source(self):
        discovery_started.set()
        release_discovery.wait(3)

    monkeypatch.setattr(XdSdkStereoSource, 'open', open_source)
    monkeypatch.setattr(XdSdkStereoSource, 'start', lambda self: stream_starts.append(True))
    window.source_panel.kind.setCurrentIndex(3)
    window.start_pipeline()
    worker = window.worker
    qtbot.waitUntil(discovery_started.is_set, timeout=2000)
    with qtbot.waitSignal(worker.finished, timeout=2000):
        try:
            window.stop_button.click()
            assert worker.isRunning()
            assert '正在停止' in window.banner.text()
        finally:
            release_discovery.set()
    assert not stream_starts


def test_queued_old_worker_error_does_not_change_new_session(window, qtbot, monkeypatch):
    w = window
    w.source_panel.kind.setCurrentIndex(0)
    # 不启动计算线程，只复现 Qt 排队信号晚于来源切换到达的顺序。
    monkeypatch.setattr(PipelineWorker, 'start', lambda self: None)
    w.start_pipeline()
    old = w.worker
    old.error.disconnect(w._error)
    old.error.connect(w._error, Qt.QueuedConnection)
    old.error.emit('旧会话错误')
    w.source_panel.kind.setCurrentIndex(3)
    qtbot.wait(30)
    assert '旧会话错误' not in w.banner.text()
    assert '旧会话错误' not in w.statusBar().currentMessage()


def test_failed_record_start_restores_toolbar(window, qtbot, monkeypatch, tmp_path):
    import egohand.gui.workers as workers
    import egohand.gui.main_window as main_window

    def fail(*args, **kwargs):
        raise OSError('测试写入失败')

    monkeypatch.setattr(workers, 'AsyncRecorder', fail)
    monkeypatch.setattr(main_window, 'ROOT', tmp_path)
    w = window
    w.source_panel.kind.setCurrentIndex(0)
    w.start_pipeline()
    qtbot.waitUntil(lambda: w.latest is not None, timeout=6000)
    w.start_recording()
    qtbot.waitUntil(lambda: '测试写入失败' in w.statusBar().currentMessage(), timeout=3000)
    assert not w.recording
    assert w.record_start_action.isEnabled()
    assert w.record_button.text() == '录制'
    assert w.worker.isRunning()


def test_import_sets_replay_range_and_clears_unavailable_images(window, source, engine, tmp_path, monkeypatch):
    dest = tmp_path / 'replay'
    writer = Recorder(dest, source.get_metadata(), source.get_calibration())
    for i in range(3):
        frame = source.read()
        if i != 1:  # 中间结果存在，但原始图像缺失，不能继续显示上一帧。
            writer.write_raw(frame)
        writer.write_result(engine.process(frame))
    writer.close()
    w = window
    monkeypatch.setattr(QFileDialog, 'getExistingDirectory', lambda *a: str(dest))
    w.source_panel.seek_index.setMaximum(0)
    w.open_results()
    assert w.source_panel.seek_index.maximum() == 2
    assert w.left_view.pix is not None
    w.display_saved(1)
    assert w.latest.frame_id == 1
    assert w.left_view.pix is None and w.right_view.pix is None
    assert w.left_view.last_bgr is None and w._debug_frame is None
    assert '图像' in w.banner.text()


def test_show3d_replay_uses_same_rectified_pixels(window, tmp_path, monkeypatch):
    from egohand.core.pipeline import ROOT, EgoHandEngine
    from egohand.sources.show3d import SHOW3DSource, SHOW3DFrameProcessor, _verified_scenes

    root = ROOT / 'samples/date/SHOW3D'
    if not _verified_scenes(root):
        pytest.skip('本机无已验收 SHOW3D 场景')
    source = SHOW3DSource(root)
    engine = EgoHandEngine()
    engine.start(source)
    try:
        frame = source.read()
        result = SHOW3DFrameProcessor().process(engine, frame, 'ground_truth')
        expected = engine.last_frame
        dest = tmp_path / 'show3d_replay'
        writer = Recorder(dest, source.get_metadata())
        try:
            writer.write_raw(frame)
            writer.write_result(result)
        finally:
            writer.close()
    finally:
        engine.close()
    monkeypatch.setattr(QFileDialog, 'getExistingDirectory', lambda *a: str(dest))
    window.open_results()
    assert window._debug_frame.image_geometry == 'rectified'
    np.testing.assert_array_equal(window._debug_frame.left_image, expected.left_image)
    np.testing.assert_array_equal(window._debug_frame.right_image, expected.right_image)
    assert '已校正' in window.left_card.meta.text()
