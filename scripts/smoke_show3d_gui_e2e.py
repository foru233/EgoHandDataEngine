"""End-to-end Qt SHOW3D smoke: actually start the worker and render a GT frame."""
import os, time
os.environ["QT_QPA_PLATFORM"]="offscreen"
from PySide6.QtWidgets import QApplication
from egohand.core.pipeline import ROOT
from egohand.gui.main_window import MainWindow
app=QApplication([])
window=MainWindow()
window.source_panel.kind.setCurrentIndex(5)
window.source_panel.path.setText(str(ROOT/"samples/date/SHOW3D/scenes/LWA828/bbq_tapping_33e9"))
window.source_panel.hot3d_mode.setCurrentIndex(window.source_panel.hot3d_mode.findData("ground_truth"))
window.start_pipeline()
end=time.monotonic()+14
try:
    while window.latest is None and time.monotonic()<end:
        app.processEvents()
        time.sleep(.04)
    if window.latest is None:
        raise RuntimeError("GUI worker did not produce a frame: "+window.banner.text())
    print("SHOW3D_GUI_E2E_OK",window.latest.source_type,window.latest.reconstruction_status,
          window.latest.diagnostics.get("show3d_sequence_id"),
          "gt_hands",len(window.latest.hands),flush=True)
    assert window.latest.source_type=="show3d"
    assert window.latest.diagnostics["result_kind"] in ("official_ground_truth","ground_truth_unavailable")
finally:
    window.stop_pipeline()
    window.close()
    app.processEvents()
