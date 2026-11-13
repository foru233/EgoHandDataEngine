"""Capture the real Qt/OpenGL customer window using clearly labelled synthetic input."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from PySide6.QtCore import Qt,QCoreApplication,QTimer
from PySide6.QtWidgets import QApplication
QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
from client_gui import ClientMainWindow
app=QApplication([]);app.setStyle('Fusion');w=ClientMainWindow();w.show()
w.source.setCurrentIndex(w.source.findData('synthetic'));w.start_pipeline()
out=Path(__file__).resolve().parent/'screenshots';out.mkdir(exist_ok=True)
def capture():
    w.grab().save(str(out/'client-preview.png'))
    (out/'preview-evidence.json').write_text(json.dumps({'has_result':w.latest is not None,'gl_valid':w.skeleton.isValid(),'size':[w.width(),w.height()],'synthetic':True}),encoding='utf8')
    w.resize(1000,700)
def finish():
    w.grab().save(str(out/'client-compact.png'));w.close()
QTimer.singleShot(3500,capture);QTimer.singleShot(4200,finish)
sys.exit(app.exec())
