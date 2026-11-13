import pytest
from egohand.gui.main_window import MainWindow

@pytest.mark.gui
def test_default_camera_and_optional_error_gates(qtbot):
    w=MainWindow();qtbot.addWidget(w)
    assert w.source_panel.kind.currentText()=='VS030 · 双通道预览'
    assert not w.reconstruction_panel.use_epipolar.isChecked()
    assert not w.reconstruction_panel.use_reprojection.isChecked()
    w.reconstruction_panel.use_epipolar.setChecked(True)
    w.reconstruction_panel.use_reprojection.setChecked(True)
    assert w.stereo_config['use_epipolar_error'] and w.stereo_config['use_reprojection_error']

@pytest.mark.gui
def test_window_lifecycle(qtbot,tmp_path,monkeypatch):
    w=MainWindow();qtbot.addWidget(w);w.show()
    try:
        w.source_panel.kind.setCurrentIndex(0)  # 生命周期测试明确使用合成数据。
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=12000)
        assert w.left_view.pix is not None and w.right_view.pix is not None
        assert len(w.skeleton.dynamic)>0
        w.toggle_pause();assert w.paused
        qtbot.wait(150)
        w.step_frame();qtbot.wait(100)
        w.toggle_pause()
        assert w.stop_pipeline()
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=12000)
        worker=w.worker
        assert w.stop_pipeline() and not worker.isRunning()
        assert not worker.capture_thread.is_alive()
        # 测试失败路径时不访问实际相机，确保连接状态不会影响结果。
        from egohand.sources.vendor_xdsdk import XdSdkStereoSource
        def fail_open(self):raise RuntimeError('模拟相机连接失败')
        monkeypatch.setattr(XdSdkStereoSource,'open',fail_open)
        w.source_panel.kind.setCurrentIndex(w.source_panel.kind.findText('VS030 · 双通道预览'))
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.worker is not None and not w.worker.isRunning(),timeout=5000)
        assert w.worker.last_error=='模拟相机连接失败'
    finally:w.stop_pipeline();w.close()
