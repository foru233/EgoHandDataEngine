"""Native Qt visual QA for the redesigned workspace, with bounded self-closing run."""
import json
import sys
from pathlib import Path
from PySide6.QtCore import Qt,QCoreApplication,QTimer
from PySide6.QtWidgets import QApplication
from egohand.gui.main_window import MainWindow
from egohand.core.pipeline import ROOT

QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
app=QApplication(sys.argv[:1]);app.setStyle("Fusion")
w=MainWindow();w.show()
out=ROOT/"logs/ui-redesign";out.mkdir(parents=True,exist_ok=True)
evidence={"platform":app.platformName(),"captures":[],"errors":[]}

def capture(name):
    app.processEvents()
    w.grab().save(str(out/f"{name}.png"))
    evidence["captures"].append({"name":name,"window_size":[w.width(),w.height()],"left_view":[w.left_view.width(),w.left_view.height()],"skeleton_view":[w.skeleton.width(),w.skeleton.height()],"gl_valid":w.skeleton.isValid(),"has_result":w.latest is not None})

def safe(fn):
    def run():
        try:fn()
        except Exception as e:evidence["errors"].append(repr(e))
    return run

def empty():capture("01-empty-dark");w.start_pipeline()
def dark():
    if w.latest is None:raise RuntimeError("No synthetic result")
    w.toggle_pause();capture("02-studio-dark")
def light():w.theme.setCurrentIndex(1)
def light_capture():capture("03-studio-light");w.resize(1280,820)
def compact():capture("04-compact-light");w.theme.setCurrentIndex(0);w.resize(1600,1040);w.inspector_tabs.setCurrentIndex(1)
def detection_tab():capture("05-detection-tab");w.inspector_tabs.setCurrentIndex(2)
def recording_tab():capture("06-recording-tab");w.inspector_tabs.setCurrentIndex(0)
def finish():
    w.stop_pipeline();w.close()
    (out/"evidence.json").write_text(json.dumps(evidence,indent=2),encoding="utf-8")
    print(json.dumps(evidence,indent=2))

QTimer.singleShot(300,safe(empty))
QTimer.singleShot(1800,safe(dark))
QTimer.singleShot(2100,safe(light))
QTimer.singleShot(2500,safe(light_capture))
QTimer.singleShot(2800,safe(compact))
QTimer.singleShot(3200,safe(detection_tab))
QTimer.singleShot(3500,safe(recording_tab))
QTimer.singleShot(3900,safe(finish))
sys.exit(app.exec())
