"""Bounded native Qt regression: four sizes, dark/light, real GL, no UI edits."""
import json
from pathlib import Path
import sys
from PySide6.QtCore import Qt,QCoreApplication,QTimer
from PySide6.QtWidgets import QApplication
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from egohand.gui.main_window import MainWindow

out=Path('logs/validation_20261103_114805/gui_final');out.mkdir(parents=True,exist_ok=True)
QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
app=QApplication(sys.argv[:1]);w=MainWindow();w.show()
report={'platform':app.platformName(),'captures':[],'errors':[]}
sizes=[(1500,960),(1280,820),(1100,720),(1280,1000)]


def capture(i):
    try:
        theme=i//4;size=sizes[i%4];w.theme.setCurrentIndex(theme);w.resize(*size);app.processEvents()
        def actual():
            name=f'{"dark" if theme==0 else "light"}_{size[0]}x{size[1]}'
            w.grab().save(str(out/(name+'.png')))
            report['captures'].append({'name':name,'requested_size':size,'actual_size':[w.width(),w.height()],
                'left_view':[w.left_view.width(),w.left_view.height()],
                'skeleton_view':[w.skeleton.width(),w.skeleton.height()],
                'gl_valid':w.skeleton.isValid(),'valid_synthetic_joints':sum(h.valid_joint_count for h in w.latest.hands) if w.latest else 0})
        QTimer.singleShot(180,actual)
    except Exception as e:report['errors'].append(str(e))


def freeze():
    if w.latest:w.toggle_pause()
    else:report['errors'].append('No synthetic output')


def finish():
    w.stop_pipeline();w.close();(out/'evidence.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report,indent=2));app.quit()

QTimer.singleShot(150,w.start_pipeline);QTimer.singleShot(1300,freeze)
for i in range(8):QTimer.singleShot(1600+i*450,lambda i=i:capture(i))
QTimer.singleShot(5500,finish)
sys.exit(app.exec())
