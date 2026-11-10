from pathlib import Path
import cv2
import numpy as np
import pytest
from egohand.detection.mediapipe_detector import MediaPipeHandDetector,bgr_to_rgb
from egohand.detection.landmarks import JOINT_NAMES,BONES,FINGERTIPS

MODEL=Path("models/hand_landmarker.task")
IMAGE=Path("samples/public/hand.jpg")

def test_color():
    np.testing.assert_array_equal(bgr_to_rgb(np.array([[[1,2,3]]],dtype=np.uint8)),[[[3,2,1]]])
def test_indices():
    assert len(JOINT_NAMES)==21 and JOINT_NAMES[0]=="WRIST" and JOINT_NAMES[20]=="PINKY_TIP"
    assert FINGERTIPS==(4,8,12,16,20)
    assert all(0<=a<21 and 0<=b<21 for a,b in BONES)
def test_missing_model():
    with pytest.raises(FileNotFoundError,match="模型未加载"):MediaPipeHandDetector("not-present.task")

@pytest.mark.assets
def test_real_model_image_video():
    if not MODEL.is_file() or not IMAGE.is_file():pytest.skip("缺少官方模型或许可可核实的真实照片")
    image=cv2.imread(str(IMAGE));assert image is not None
    a=MediaPipeHandDetector(MODEL,"IMAGE");b=MediaPipeHandDetector(MODEL,"VIDEO")
    try:
        assert a.landmarker is not b.landmarker and a.num_hands==b.num_hands==2
        hands=a.detect(image)
        assert len(hands)>=1,"真实照片没有检测到手，不能宣称 M04 通过"
        assert hands[0].pixel_landmarks.shape==(21,2) and hands[0].world_landmarks.shape==(21,3)
        assert b.detect(image,1_000_000_000)
        assert b.detect(image,1_033_000_000)
        with pytest.raises(ValueError,match="递增"):b.detect(image,1_033_000_001)
        assert a.detect(np.zeros((256,256,3),np.uint8))==[]
    finally:a.close();b.close()
