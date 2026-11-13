"""Bounded real-recording GUI integration check; source recordings stay read-only."""
import argparse,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from PySide6.QtCore import Qt,QCoreApplication,QTimer
from PySide6.QtWidgets import QApplication
QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
from client_gui import ClientMainWindow
p=argparse.ArgumentParser();p.add_argument('recording');p.add_argument('--mode',default='stereo');p.add_argument('--frames',type=int,default=8);args=p.parse_args()
app=QApplication([]);w=ClientMainWindow();w.show();w.select_recording(args.recording)
w.mode.setCurrentIndex(w.mode.findData(args.mode));w.start_pipeline();started=time.monotonic()
def check():
    worker=w.session.worker
    if (worker and worker.processed>=args.frames) or time.monotonic()-started>110 or (worker and not worker.isRunning()):
        data={'mode':args.mode,'processed':worker.processed if worker else 0,'has_image':w.left_view.pix is not None,'has_pose_message':bool(w.latest and w.latest.diagnostics.get('vio_pose')),'error':worker.last_error if worker else None,'elapsed':time.monotonic()-started}
        data['displayable_poses']=sum(bool(r.diagnostics.get('vio_pose',{}).get('pose_valid')) for r in worker.history) if worker else 0
        out=Path(__file__).resolve().parent
        w.grab().save(str(out/'screenshots'/('recording-'+args.mode+'.png')))
        data['stopped']=w.stop_pipeline();w.close()
        (out/('recording-'+args.mode+'.json')).write_text(json.dumps(data,indent=2),encoding='utf8');print(json.dumps(data));app.exit(0 if data['processed']>=8 and data['has_pose_message'] and data['stopped'] else 1)
timer=QTimer();timer.timeout.connect(check);timer.start(200)
sys.exit(app.exec())
