"""Bounded native Qt validation with actual local HOT3D clips."""
import json,sys,time
from pathlib import Path
from PySide6.QtCore import Qt,QCoreApplication
from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QTest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from egohand.gui.main_window import MainWindow
QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
app=QApplication(sys.argv[:1]);w=MainWindow();w.show()
out=Path(json.loads(Path("logs/active_hot3d_gui_run.json").read_text())["path"])/"screenshots";out.mkdir(exist_ok=True)
def wait(predicate):
    for _ in range(400):
        app.processEvents()
        if predicate():return
        time.sleep(.05)
    raise RuntimeError("HOT3D GUI wait timed out: "+w.banner.text())
report={"platform":app.platformName(),"captures":[]}
try:
    w.source_panel.kind.setCurrentIndex(4);w.source_panel.path.setText(str(Path("samples/date/hot3d/train_aria/clip-001849.tar").resolve()));w.start_pipeline()
    wait(lambda:w.latest is not None);w.worker.seek(110);w.paused=True;wait(lambda:w.latest.frame_id==110)
    assert sum(h.valid_joint_count for h in w.latest.hands)==40
    for theme in (0,1):
        w.theme.setCurrentIndex(theme)
        for width,height in [(1500,960),(1280,820),(1100,720),(1280,1000)]:
            w.resize(width,height);time.sleep(.12);app.processEvents()
            name=f"gt_{theme}_{width}x{height}.png";w.grab().save(str(out/name))
            report["captures"].append({"file":name,"size":[w.width(),w.height()],"gl_valid":w.skeleton.isValid(),"points":len(w.skeleton.points)})
    w.source_panel.hot3d_mode.setCurrentIndex(1);wait(lambda:w.latest.diagnostics.get("result_kind")=="algorithm_prediction")
    w.grab().save(str(out/"prediction.png"));report["prediction_valid_joints"]=sum(h.valid_joint_count for h in w.latest.hands)
    w.stop_pipeline();w.source_panel.hot3d_mode.setCurrentIndex(0);w.source_panel.path.setText(str(Path("samples/date/hot3d/train_aria/clip-003365.tar").resolve()));w.start_pipeline()
    wait(lambda:w.latest is not None);assert w.latest.reconstruction_status=="Ground Truth Unavailable"
    w.grab().save(str(out/"test_gt_unavailable.png"));report["test_status"]=w.latest.reconstruction_status
finally:
    w.stop_pipeline();w.close()
(out/"evidence.json").write_text(json.dumps(report,indent=2),encoding="utf8");print(json.dumps(report,indent=2))
