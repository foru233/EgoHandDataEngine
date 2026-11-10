from pathlib import Path
import numpy as np
import pytest
from egohand.detection.rtmpose_detector import RTMPoseHandDetector


def test_no_hand_box_does_not_infer_whole_image():
    d=RTMPoseHandDetector.__new__(RTMPoseHandDetector);d.closed=False;d.roi=None;d.num_hands=2;d.det=lambda im:[]
    d.pose=lambda *a,**k:pytest.fail('empty boxes must not trigger whole image pose')
    assert d.detect(np.zeros((480,640,3),np.uint8))==[]


def test_roi_points_are_restored_and_scores_gate_joints():
    d=RTMPoseHandDetector.__new__(RTMPoseHandDetector);d.closed=False;d.roi=(.25,.25,.5,.5);d.num_hands=2;d.keypoint_threshold=.3
    d.det=lambda im:np.array([[10,20,100,120]])
    d.pose=lambda im,bboxes:(np.full((1,21,2),[50.,60.]),np.array([[.2]+[.9]*20]))
    result=d.detect(np.zeros((480,640,3),np.uint8))[0]
    np.testing.assert_allclose(result.pixel_landmarks[0],[210,180]);assert not result.valid[0] and result.valid[1]
    assert result.handedness=='Unknown' and result.handedness_score==0


def test_real_rtmpose_model_on_photo():
    if not Path('models/rtmpose_hand.onnx').exists():pytest.skip('缺少 RTMPose 权重')
    import cv2
    d=RTMPoseHandDetector()
    try:
        hands=d.detect(cv2.imread('samples/public/hand.jpg'))
        assert hands and hands[0].pixel_landmarks.shape==(21,2) and hands[0].handedness=='Unknown'
    finally:d.close()


def test_algorithm_settings_switch(qtbot):
    from egohand.gui.reconstruction_panel import ReconstructionPanel
    p=ReconstructionPanel();qtbot.addWidget(p);p.show()
    assert p.algorithm.currentData()=='mediapipe'
    p.algorithm.setCurrentIndex(1)
    assert p.settings()['algorithm']=='rtmpose' and p.rtm_page.isVisible() and not p.mp_page.isVisible()

@pytest.mark.gui
def test_rtmpose_real_hot3d_gui(qtbot):
    from egohand.gui.main_window import MainWindow
    p=Path('samples/date/hot3d/train_aria/clip-001851.tar')
    if not p.is_file() or not Path('models/rtmpose_hand.onnx').exists():pytest.skip('缺少模型或数据')
    w=MainWindow();qtbot.addWidget(w);w.show()
    try:
        w.source_panel.kind.setCurrentIndex(4);w.source_panel.path.setText(str(p.resolve()));w.source_panel.hot3d_mode.setCurrentIndex(1)
        w.reconstruction_panel.algorithm.setCurrentIndex(1);w.apply_detection_settings();w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=20000)
        assert w.latest.diagnostics['left_detection_origin']=='rtmpose'
        assert w.worker.engine.detectors[0].__class__.__name__=='RTMPoseHandDetector'
        assert w.latest.diagnostics['detector_input']=='virtual_rectified_grayscale'
        w.reconstruction_panel.algorithm.setCurrentIndex(0);w.apply_detection_settings()
        qtbot.waitUntil(lambda:w.worker.engine.detectors[0].__class__.__name__=='MediaPipeHandDetector' and w.detector_config['algorithm']=='mediapipe',timeout=20000)
    finally:w.stop_pipeline();w.close()
