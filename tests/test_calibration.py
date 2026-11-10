from dataclasses import replace
import numpy as np
import pytest
from egohand.calibration.loader import save_calibration,load_calibration
from egohand.calibration.validator import validate_calibration,CalibrationError

def test_roundtrip(calibration,tmp_path):
    p=tmp_path/"c.yaml";save_calibration(calibration,p)
    c=load_calibration(p)
    np.testing.assert_equal(c.K_left,calibration.K_left)

@pytest.mark.parametrize("field,value",[("K_left",np.eye(2)),("t_right_left",np.zeros(3)),("translation_unit",""),("R_right_left",np.ones((3,3))),("K_left",np.diag([-1.,600,1])),("D_right",np.array([np.nan]*5)),("distortion_model","unknown"),("synthetic",False)])
def test_reject(calibration,field,value):
    with pytest.raises(CalibrationError): validate_calibration(replace(calibration,**{field:value}))

def test_size(calibration):
    with pytest.raises(CalibrationError): validate_calibration(calibration,(320,240))

def test_missing_unit(tmp_path):
    p=tmp_path/"x.json";p.write_text("{}")
    with pytest.raises(CalibrationError): load_calibration(p)
