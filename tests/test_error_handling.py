from dataclasses import replace
import numpy as np
import pytest
from egohand.calibration.validator import CalibrationError
from egohand.core.pipeline import EgoHandEngine

def test_no_calibration(source):
    r=EgoHandEngine().process(source.read())
    assert r.reconstruction_status=="2D Only"

def test_fake_calibration(engine,source):
    f=source.read();f.metadata["synthetic"]=False
    f.left_detections=[];f.right_detections=[]
    r=engine.process(f)
    assert r.reconstruction_status=="2D Only" and not r.hands

def test_timestamp(engine,source):
    f=source.read();engine.process(f)
    assert engine.process(f).diagnostics["reason"]=="TIMESTAMP_NON_MONOTONIC"

def test_bad_size(engine,source):
    f=source.read();f.left_image=np.zeros((20,20,3),np.uint8)
    r=engine.process(f)
    assert r.reconstruction_status=="2D Only"

def test_bad_calibration_clears(engine,calibration,source):
    with pytest.raises(CalibrationError):engine.set_calibration(replace(calibration,t_right_left=np.zeros(3)))
    assert engine.rectifier is None
    assert engine.process(source.read()).reconstruction_status=="2D Only"

def test_sync_gate(engine,source):
    f=source.read();f.right_timestamp_ns+=10_000_000
    assert engine.process(f).reconstruction_status=="2D Only"
