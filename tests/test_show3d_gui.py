"""SHOW3D integration smoke and regression; requires local verified SHOW3D files."""
import os
import sys
from pathlib import Path
import numpy as np

# Windows 使用原生 Qt 平台，以便其他测试正常创建 OpenGL 上下文。
# 无显示服务器的 Linux 环境仍可显式使用 offscreen。
if sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from egohand.core.pipeline import EgoHandEngine, ROOT
from egohand.sources.show3d import SHOW3DSource, SHOW3DFrameProcessor, _verified_scenes, world_to_camera
from egohand.gui.main_window import MainWindow
from PySide6.QtWidgets import QApplication


def test_show3d_verified_scene_and_gt():
    root = ROOT / "samples/date/SHOW3D"
    valid = _verified_scenes(root)
    assert valid, "No verified SHOW3D scenes available"
    s, scene, count = valid[0]
    source = SHOW3DSource(root / "scenes" / s / scene)
    engine = EgoHandEngine()
    engine.start(source)
    processor = SHOW3DFrameProcessor()
    try:
        assert source.get_metadata().extra["annotation_version"] == "v3"
        assert len(source.rows) == count
        frame = None
        result = None
        # Some frames may have missing headset pose/hand; search a few candidates.
        for n in (0, count // 3, count // 2, count - 1):
            source.seek(n)
            f = source.read()
            r = processor.process(engine, f, "ground_truth")
            if r.hands:
                frame, result = f, r
                break
        assert result is not None, "No valid GT among smoke frames"
        assert result.diagnostics["result_kind"] == "official_ground_truth"
        assert result.source_type == "show3d"
        assert result.coordinate_frame == "camera_left_optical"
        assert all(h.valid_joint_count == 20 for h in result.hands)
        assert engine.last_frame.image_geometry == "rectified"
        assert engine.last_frame.debug_ground_truth_uv
        for h in result.hands:
            src = frame.metadata["show3d"]["ground_truth"][h.handedness.lower()]["world_mm"]
            gt = world_to_camera(src, frame.metadata["show3d"]["calibration"][0].t_world_from_camera) / 1000.
            assert np.allclose(h.joints[4].xyz_m, gt[0], atol=1.e-8)
            assert np.allclose(h.joints[0].xyz_m, gt[5], atol=1.e-8)
        print("SHOW3D_GT_PASS", s, scene, "frame", frame.metadata["show3d"]["frame_index"],
              "hands",len(result.hands))
    finally:
        engine.close()


def test_show3d_prediction_pipeline_without_gt_leakage():
    root = ROOT / "samples/date/SHOW3D"
    s, scene, count = _verified_scenes(root)[0]
    source = SHOW3DSource(root / "scenes" / s / scene, view_mode="prediction")
    engine = EgoHandEngine()
    engine.start(source)
    try:
        frame = source.read()
        result = SHOW3DFrameProcessor().process(engine, frame, "prediction")
        assert result.diagnostics["result_kind"] == "algorithm_prediction"
        assert result.source_type == "show3d"
        assert result.diagnostics["left_detection_origin"] == "none"
        assert not result.hands
        assert result.diagnostics["ground_truth_available"] in (True, False)
        assert result.diagnostics.get("dataset_pairing",{}).get("dataset_id") == "show3d"
        print("SHOW3D_NO_GT_LEAK_PASS")
    finally:
        engine.close()


def test_show3d_monochrome_rectified_before_detection_and_invalid_pose_blocks():
    from dataclasses import replace
    root = ROOT / 'samples/date/SHOW3D'
    s, scene, _ = _verified_scenes(root)[0]
    source = SHOW3DSource(root / 'scenes' / s / scene, view_mode='prediction')
    class Detector:
        last_detection_diagnostics = {}
        def __init__(self): self.images = []
        def detect(self, image, timestamp): self.images.append(image.copy()); return []
        def close(self): pass
    detectors = [Detector(), Detector()]
    engine = EgoHandEngine(detectors=detectors); engine.start(source)
    try:
        frame = source.read()
        assert frame.metadata['show3d']['pose_valid']
        for image in (frame.left_image,frame.right_image):
            assert image.shape[:2] == (1280,1024)
            assert np.array_equal(image[:,:,0],image[:,:,1])
            assert np.array_equal(image[:,:,1],image[:,:,2])
        processor = SHOW3DFrameProcessor()
        result = processor.process(engine,frame,'prediction')
        assert result.diagnostics['detector_input']=='rectified_monochrome_pinhole'
        assert engine.last_frame.image_geometry=='rectified'
        np.testing.assert_array_equal(detectors[0].images[0],engine.last_frame.left_image)
        np.testing.assert_array_equal(detectors[1].images[0],engine.last_frame.right_image)
        show = {**frame.metadata['show3d'],'pose_valid':False}
        invalid = replace(frame,timestamp_ns=frame.timestamp_ns+1,metadata={**frame.metadata,'show3d':show})
        blocked = processor.process(engine,invalid,'prediction')
        assert blocked.diagnostics['model_status']=='RECTIFICATION_REQUIRED'
        assert blocked.diagnostics['detector_input']=='unavailable_rectification_required'
        assert [len(d.images) for d in detectors]==[1,1]
    finally:
        engine.close()


def test_show3d_gui_source_selector():
    app = QApplication.instance() or QApplication([])
    window = MainWindow()
    try:
        idx = window.source_panel.kind.findText("SHOW3D · v3 双目")
        assert idx >= 0
        window.source_panel.kind.setCurrentIndex(idx)
        assert '等待启动' in window.calibration_panel.label.text()
        assert window.source_panel.hot3d_mode.isVisible() or not window.isVisible()
        source = window.make_source()
        assert isinstance(source, SHOW3DSource)
        assert source.path == ROOT / "samples/date/SHOW3D"
        source.open()
        try:
            assert source.get_metadata().extra["frame_count"] > 0
            window.on_ready({"metadata":source.get_metadata(),"calibration":None,"model_loaded":False})
            assert '等待首帧' in window.calibration_panel.label.text()
            assert window.source_panel.seek_index.maximum() == len(source.rows)-1
        finally:
            source.close()
        print("SHOW3D_GUI_PASS",len(source.rows))
    finally:
        window.close()


def test_show3d_reports_actual_frame_calibration(qtbot):
    window = MainWindow()
    qtbot.addWidget(window)
    window.show()
    try:
        window.source_panel.kind.setCurrentIndex(5)
        window.start_pipeline()
        qtbot.waitUntil(lambda: bool(window.latest is not None and
            window.latest.source_type == 'show3d' and window.latest.calibration_id and
            '本帧标定可用' in window.calibration_panel.label.text()), timeout=15000)
    finally:
        window.stop_pipeline()
        window.close()
