import pytest
from egohand.gui.main_window import MainWindow

@pytest.mark.gui
def test_adaptive_viewports_preserve_content(qtbot):
    w=MainWindow();qtbot.addWidget(w);w.show()
    try:
        w.source_panel.kind.setCurrentIndex(0)  # 此测试验证合成源的 4:3 视图。
        w.start_pipeline();qtbot.waitUntil(lambda:w.latest is not None,timeout=8000)
        for size in ((1500,960),(1280,820),(1100,720),(1280,1000)):
            w.resize(*size);qtbot.wait(120)
            assert w.width()==size[0] and w.height()==size[1]
            assert w.stat_fps.height()<=40
            assert w.skeleton.height()>=220
            assert w.skeleton.width()>=300
            for view in (w.left_view,w.right_view):
                pix=view.pixmap()
                assert pix is not None
                assert abs(pix.width()/pix.height()-4/3)<.015
                occupancy=pix.width()*pix.height()/(view.width()*view.height())
                assert occupancy>=.76, (size,occupancy)
                assert pix.width()<=view.width() and pix.height()<=view.height()
        w.settings_button.setChecked(False);qtbot.wait(120)
        assert not w.inspector.isVisible()
        w.settings_button.setChecked(True);qtbot.wait(120)
        assert w.inspector.isVisible()
        w.theme.setCurrentIndex(1);qtbot.wait(100)
        assert w.skeleton.isValid()
    finally:w.stop_pipeline();w.close()
