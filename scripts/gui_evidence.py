"""Native Qt acceptance capture. No offscreen platform; exercises the real GL widget."""
import json
import logging
import sys
import time
from pathlib import Path
from PySide6.QtCore import QTimer,QPoint,Qt,QCoreApplication
from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QTest
from egohand.gui.main_window import MainWindow
from egohand.core.pipeline import ROOT
from egohand.storage.exporter import export_all

QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
app=QApplication(sys.argv[:1]);w=MainWindow();w.show()
root=ROOT/"logs";root.mkdir(exist_ok=True)
evidence={"qt_platform":app.platformName(),"checks":{},"errors":[]}

def safe(callback):
    def run():
        try:callback()
        except Exception as e:
            evidence["errors"].append(str(e));logging.exception("GUI evidence failure")
    return run

def snapshot():
    if w.latest is None:raise RuntimeError("模拟流水线没有输出")
    w.toggle_pause()
    evidence["checks"]["synthetic_valid_joints"]=sum(h.valid_joint_count for h in w.latest.hands)
    evidence["checks"]["left_2d_visible"]=w.left_view.pix is not None
    evidence["checks"]["right_2d_visible"]=w.right_view.pix is not None
    evidence["checks"]["gl_items"]=len(w.skeleton.dynamic)
    evidence["checks"]["runtime_metrics"]=w.worker.stats
    w.grab().save(str(root/"gui_synthetic.png"))
    w.skeleton.grabFramebuffer().save(str(root/"skeleton.png"))
    evidence["checks"]["gl_valid"]=w.skeleton.isValid()
    export_all([w.latest],ROOT/"exports"/time.strftime("gui_selected_%Y%m%d_%H%M%S"))
    w.start_recording()

def resume():w.toggle_pause()
def recorded():
    w.stop_recording()
    worker=w.worker
    evidence["checks"]["stop_success"]=w.stop_pipeline()
    evidence["checks"]["capture_thread_exited"]=not worker.capture_thread.is_alive()
    w.start_pipeline()
def real_image():
    evidence["checks"]["restart_has_result"]=w.latest is not None
    w.source_panel.kind.setCurrentIndex(2)
    w.source_panel.path.setText(str(ROOT/"samples/public/hand.jpg"))
    w.start_pipeline()
def real_snapshot():
    if not w.latest:raise RuntimeError("真实图片推理没有输出")
    evidence["checks"]["real_image_status"]=w.latest.reconstruction_status
    evidence["checks"]["real_image_detected_hands"]=len(w.latest.left_detections)
    evidence["checks"]["real_image_has_no_metric_joints"]=not any(h.valid_joint_count for h in w.latest.hands)
    w.grab().save(str(root/"gui_real_image.png"))
    w.theme.setCurrentIndex(1)
    w.grab().save(str(root/"gui_light.png"))
def finish():
    w.stop_pipeline();w.close()
    (root/"gui_evidence.json").write_text(json.dumps(evidence,indent=2,ensure_ascii=False),encoding="utf-8")
    print(json.dumps(evidence,indent=2,ensure_ascii=False))

QTimer.singleShot(200,w.start_pipeline)
QTimer.singleShot(1800,safe(snapshot))
QTimer.singleShot(2200,safe(resume))
QTimer.singleShot(3300,safe(recorded))
QTimer.singleShot(4300,safe(real_image))
QTimer.singleShot(11000,safe(real_snapshot))
QTimer.singleShot(12000,safe(finish))
sys.exit(app.exec())
