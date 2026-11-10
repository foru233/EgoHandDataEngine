import pytest
from PySide6.QtCore import QCoreApplication,Qt
# PyQtGraph caches shader programs; windows in one QApplication must share GL.
QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
from egohand.calibration.schema import synthetic_calibration
from egohand.core.pipeline import EgoHandEngine
from egohand.sources.synthetic import SyntheticStereoSource
from pathlib import Path

@pytest.fixture(autouse=True)
def isolate_user_roi_config(monkeypatch,tmp_path):
    # GUI 测试使用独立配置，不读取或覆盖用户保存的真实 ROI。
    from egohand.gui.roi_panel import RoiPanel
    original=RoiPanel.__init__
    default=(Path(__file__).resolve().parents[1]/'configs/roi.yaml').resolve()
    def init(panel,config_path=None,parent=None):
        if config_path is not None and Path(config_path).resolve()==default:
            config_path=tmp_path/'gui_roi.yaml'
        original(panel,config_path,parent)
    monkeypatch.setattr(RoiPanel,'__init__',init)

@pytest.fixture
def calibration(): return synthetic_calibration()

@pytest.fixture
def source():
    s=SyntheticStereoSource("two_static",frames=10)
    s.open();s.start()
    yield s
    s.close()

@pytest.fixture
def engine(calibration): return EgoHandEngine(calibration=calibration)
