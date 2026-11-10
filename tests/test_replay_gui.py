import json
from egohand.gui.main_window import MainWindow
from egohand.storage.recorder import Recorder,AsyncRecorder

def test_saved_result_images_and_seek(qtbot,source,engine,tmp_path,monkeypatch):
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QFileDialog
    path=tmp_path/"replay"
    writer=Recorder(path,source.get_metadata(),source.get_calibration())
    for _ in range(3):
        f=source.read();writer.write_raw(f);writer.write_result(engine.process(f))
    writer.close()
    monkeypatch.setattr(QFileDialog,"getExistingDirectory",lambda *a,**k:str(path))
    w=MainWindow();qtbot.addWidget(w);w.show()
    try:
        assert [w.inspector_tabs.tabText(i) for i in range(w.inspector_tabs.count())]==["采集","录制","检测","区域","标定","坐标","诊断"]
        qtbot.mouseClick(w.import_button,Qt.LeftButton)
        assert w.latest.frame_id==0 and w.left_view.pix is not None and w.right_view.pix is not None
        w.source_panel.seek_index.setValue(2);w.seek_frame()
        assert w.latest.frame_id==2
        w.source_panel.kind.setCurrentIndex(1);w.source_panel.path.setText(str(path))
        w.source_panel.offline.setChecked(True);w.start_pipeline()
        qtbot.waitUntil(lambda:w.worker.processed==3,timeout=6000)
        old=w.worker.engine.session_id
        w.worker.seek(0)
        qtbot.waitUntil(lambda:w.worker.processed>=4,timeout=6000)
        assert w.worker.engine.session_id!=old
        assert w.worker.history[-1].frame_id==0
    finally:w.stop_pipeline();w.close()

def test_pause_recording_events(source,tmp_path):
    w=AsyncRecorder(tmp_path/"events",source.get_metadata(),source.get_calibration())
    assert w.submit("raw",source.read(),True)
    w.set_paused(True)
    assert not w.submit("raw",source.read())
    w.set_paused(False)
    assert w.submit("raw",source.read(),True)
    w.close()
    events=[json.loads(line) for line in (tmp_path/"events/events.jsonl").read_text().splitlines()]
    assert [e["event"] for e in events]==["pause","resume"]
    assert events[0]["last_raw_submitted"]["frame_id"]==0
    assert w.writer.manifest["raw_count"]==2

def test_toolbar_recording_controls(qtbot,tmp_path):
    from PySide6.QtCore import Qt,QPoint

    w=MainWindow();qtbot.addWidget(w);w.show()
    recording_root=tmp_path/"recordings"
    # 面板预设保存目录：离屏/无人值守环境不能弹 QFileDialog。
    w.recording_panel.set_directory(recording_root)
    try:
        w.source_panel.kind.setCurrentIndex(0)  # 合成源：不启动相机采集
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=6000)
        qtbot.mouseClick(w.record_button,Qt.LeftButton,pos=QPoint(10,w.record_button.height()//2))
        qtbot.waitUntil(lambda:w.worker.recorder is not None and w.worker.recorder.writer.manifest["raw_count"]>0,timeout=6000)
        assert w.recording and w.record_button.isEnabled()
        path=w.worker.recorder.writer.path
        assert path.parent==recording_root
        assert "录制中" in w.recording_panel.status.text()
        assert w.record_pause_action.isEnabled() and w.record_stop_action.isEnabled()
        w.record_pause_action.trigger()
        qtbot.waitUntil(lambda:w.worker.recorder.paused,timeout=3000)
        qtbot.waitUntil(lambda:w.recording_panel.paused,timeout=3000)
        assert w.recording_paused and not w.paused
        assert w.record_pause_action.text()=="继续录制"
        assert w.recording_panel.pause_button.text()=="继续"
        qtbot.mouseClick(w.record_button,Qt.LeftButton,pos=QPoint(10,w.record_button.height()//2))
        qtbot.waitUntil(lambda:not w.worker.recorder.paused,timeout=3000)
        qtbot.waitUntil(lambda:not w.recording_panel.paused,timeout=3000)
        assert not w.recording_paused
        w.record_stop_action.trigger()
        qtbot.waitUntil(lambda:w.worker.recorder is None,timeout=3000)
        # 后台写盘线程先结束；录制状态通过排队信号回到 GUI，需要等事件循环处理。
        qtbot.waitUntil(lambda:not w.recording and not w.recording_paused,timeout=3000)
        assert w.worker.isRunning()
        assert json.loads((path/"manifest.json").read_text(encoding="utf-8"))["recording_status"]=="complete"
        events=[json.loads(line)["event"] for line in (path/"events.jsonl").read_text(encoding="utf-8").splitlines()]
        # 录制开始/停止事件也会写入 events.jsonl（旧断言只期望 pause/resume，已过期）。
        assert events==["recording_started","pause","resume","recording_stopped"]
        # 停止后主窗口读取该会话 manifest.json 并回填面板摘要。
        qtbot.waitUntil(lambda:w.recording_panel.last_session==path,timeout=3000)
        assert "已停止" in w.recording_panel.status.text()
        assert "未提供回放校验信息" in w.recording_panel.summary.text()
    finally:w.stop_pipeline();w.close()
