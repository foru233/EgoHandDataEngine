import argparse
import logging
import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt,QCoreApplication
from .core.pipeline import ROOT

def main():
    p=argparse.ArgumentParser();p.add_argument("--demo",action="store_true");p.add_argument("--quit-after",type=float)
    args=p.parse_args()
    (ROOT/"logs").mkdir(exist_ok=True)
    logging.basicConfig(filename=ROOT/"logs/studio.log",level=logging.INFO,encoding="utf-8",format="%(asctime)s %(levelname)s %(message)s")
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app=QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    from .gui.main_window import MainWindow
    window=MainWindow();window.show()
    from PySide6.QtCore import QTimer
    if args.demo:QTimer.singleShot(250,window.start_pipeline)
    if args.quit_after:QTimer.singleShot(int(args.quit_after*1000),window.close)
    return app.exec()

if __name__=="__main__":sys.exit(main())
