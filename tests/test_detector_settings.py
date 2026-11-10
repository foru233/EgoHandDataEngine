from pathlib import Path
import pytest
from PySide6.QtCore import Qt
from egohand.gui.main_window import MainWindow
from egohand.gui.workers import PipelineWorker
from egohand.core.pipeline import EgoHandEngine
from egohand.sources.image import SingleImageSource


@pytest.mark.gui
def test_apply_idle_and_synthetic_running(qtbot):
    w=MainWindow();qtbot.addWidget(w);w.show();w.inspector_tabs.setCurrentIndex(1)
    w.source_panel.kind.setCurrentIndex(0)  # 本测试覆盖合成源运行中的设置应用。
    p=w.reconstruction_panel
    try:
        original_threshold=w.detector_config["min_hand_detection_confidence"]
        assert not p.apply_button.isEnabled()
        p.confidence[0].setValue(.4)
        assert p.apply_button.isEnabled() and w.detector_config["min_hand_detection_confidence"]==original_threshold
        qtbot.mouseClick(p.apply_button,Qt.LeftButton)
        assert w.detector_config["min_hand_detection_confidence"]==.4
        assert not p.apply_button.isEnabled() and "下次" in p.apply_hint.text()
        w.start_pipeline();qtbot.waitUntil(lambda:w.latest is not None,timeout=6000)
        p.num_hands.setValue(1)
        qtbot.mouseClick(p.apply_button,Qt.LeftButton)
        qtbot.waitUntil(lambda:not p._busy and "几何模拟" in p.apply_hint.text(),timeout=3000)
        assert w.worker.detector_config["num_hands"]==1
        assert w.worker.engine.detectors is None
        assert w.worker.isRunning()
        assert sum(h.valid_joint_count for h in w.latest.hands)==42
    finally:w.stop_pipeline();w.close()


@pytest.mark.assets
def test_real_detectors_replaced_atomically():
    model=Path("models/hand_landmarker.task");image=Path("samples/public/hand.jpg")
    if not model.is_file() or not image.is_file():pytest.skip("缺少官方模型或真实照片")
    source=SingleImageSource(image)
    config={"model_path":str(model),"num_hands":2,"min_hand_detection_confidence":.5,"min_hand_presence_confidence":.5,"min_tracking_confidence":.5}
    worker=PipelineWorker(source,detector_config=config)
    worker.engine=EgoHandEngine();worker.engine.start(source)
    failures=[];worker.detector_settings_failed.connect(failures.append)
    try:
        worker._new_detectors();old=list(worker.engine.detectors)
        new_config={**config,"num_hands":1,"min_hand_detection_confidence":.4}
        worker.command("detector_config",new_config);worker._commands()
        assert all(d.closed for d in old)
        assert worker.engine.detectors[0] is not worker.engine.detectors[1]
        assert all(d.num_hands==1 for d in worker.engine.detectors)
        result=worker.engine.process_next()
        assert result.reconstruction_status=="2D Only" and not result.left_detections
        assert result.diagnostics['model_status']=='RECTIFICATION_REQUIRED'
        current=worker.engine.detectors;session=worker.engine.session_id
        worker.command("detector_config",{**new_config,"model_path":"missing-model.task"});worker._commands()
        assert failures and "模型未加载" in failures[0]
        assert worker.engine.detectors is current and all(not d.closed for d in current)
        assert worker.engine.session_id==session and worker.detector_config==new_config
    finally:worker.engine.close()
