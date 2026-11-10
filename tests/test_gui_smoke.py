import pytest
from egohand.gui.main_window import MainWindow

@pytest.mark.gui
def test_window_lifecycle(qtbot,tmp_path):
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
        w.source_panel.kind.setCurrentIndex(3)
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.worker is not None and not w.worker.isRunning(),timeout=5000)
        assert w.worker.last_error=="真实相机适配器尚未配置"
    finally:w.stop_pipeline();w.close()
