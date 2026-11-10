from dataclasses import replace
import cv2
import numpy as np
import pytest
from egohand.calibration.rectifier import StereoRectifier
from egohand.calibration.validator import CalibrationError

@pytest.mark.parametrize("model",["pinhole","fisheye"])
def test_paths(calibration,source,model):
    c=replace(calibration,distortion_model=model,D_left=np.zeros(4),D_right=np.zeros(4))
    r=StereoRectifier(c)
    source.calibration=c
    f=r.rectify(source.read())
    assert f.image_geometry=="rectified"
    assert r.rectify(f) is f
    assert r.Q.shape==(4,4)
    assert r.new_K_left.shape==(3,3)
    assert f.left_image.shape==(480,640,3)
    assert np.nanmax(r.epipolar_errors(f.left_detections[0].pixel_landmarks,f.right_detections[0].pixel_landmarks))<1e-8
    with pytest.raises(CalibrationError): r.rectify(replace(f,metadata={}))

def test_rotated_coordinate_frame(calibration,source):
    from egohand.core.pipeline import EgoHandEngine
    c=replace(calibration,R_right_left=cv2.Rodrigues(np.array([.015,.025,-.01]))[0],D_left=np.array([.02,-.01,0,0,0]))
    s=source;s.calibration=c
    f=s.read();r=EgoHandEngine(calibration=c).process(f)
    assert r.hands[0].valid_joint_count==21
    np.testing.assert_allclose([j.xyz_m for j in r.hands[0].joints],f.metadata["ground_truth_xyz_m"][0],atol=1e-5)


@pytest.mark.parametrize('model', ['pinhole','fisheye'])
def test_detection_point_transform_agrees_with_direct_rectification(calibration,model):
    from egohand.core.types import HandDetection
    d=np.array([.04,-.01,.001,-.001])
    c=replace(calibration,distortion_model=model,D_left=d,D_right=d,
              R_right_left=cv2.Rodrigues(np.array([.015,.025,-.01]))[0])
    rectifier=StereoRectifier(c)
    raw=np.column_stack((np.linspace(180,440,21),np.linspace(150,330,21)))
    fn=cv2.fisheye.undistortPoints if model=='fisheye' else cv2.undistortPoints
    for side in ('left','right'):
        K=c.K_left if side=='left' else c.K_right
        undist=fn(raw.reshape(-1,1,2),K,d,P=K).reshape(-1,2)
        hand=HandDetection('Left',1.,undist/[640,480],undist)
        actual=rectifier.detection_to_rectified([hand],side)[0].pixel_landmarks
        np.testing.assert_allclose(actual,rectifier.points(raw,side),atol=1e-7)
