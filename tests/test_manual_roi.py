from pathlib import Path
import numpy as np
import pytest
from PySide6.QtCore import Qt,QPoint
from egohand.detection.roi import validate_roi,roi_pixel_box
from egohand.detection.mediapipe_detector import MediaPipeHandDetector
from egohand.core.types import HandDetection
from egohand.visualization.image_view import ImageView
from egohand.gui.roi_panel import RoiPanel


@pytest.mark.parametrize('roi',[(.9,0,.2,.5),(0,-.1,.5,.5),(0,0,0,.5),(0,0,float('nan'),.5),(1,0,1e-12,.5)])
def test_invalid_roi_rejected(roi):
    with pytest.raises(ValueError):validate_roi(roi)


def test_roi_full_and_pixel_conversion():
    assert validate_roi((0,0,1,1)) is None
    assert roi_pixel_box((.25,.25,.5,.5),(480,640,3))==(160,120,320,240)


def test_roi_model_receives_crop_and_returns_original_coordinates():
    detector=MediaPipeHandDetector.__new__(MediaPipeHandDetector);detector.roi=(.25,.25,.5,.5)
    seen=[]
    def infer(image,ts):
        seen.append(image.shape);detector.last_detection_diagnostics={'method':'full_frame'}
        uv=np.full((21,2),[50.,60.])
        return [HandDetection('Left',.9,np.column_stack((uv/[320,240],np.full(21,.2))),uv)]
    detector._detect_with_rescue=infer
    ds=detector.detect(np.zeros((480,640,3),np.uint8),100)
    assert seen==[(240,320,3)]
    np.testing.assert_allclose(ds[0].pixel_landmarks,np.tile([210,180],(21,1)))
    np.testing.assert_allclose(ds[0].normalized_landmarks[:,:2],ds[0].pixel_landmarks/[640,480])
    np.testing.assert_allclose(ds[0].normalized_landmarks[:,2],.1)
    assert ds[0].origin.endswith('manual_roi') and detector.last_detection_diagnostics['pixel_roi']==[160,120,320,240]


def test_draw_roi_excludes_letterbox_and_survives_resize(qtbot):
    view=ImageView('test');qtbot.addWidget(view);view.resize(800,800);view.show()
    view.display(np.zeros((480,640,3),np.uint8));qtbot.wait(20)
    changes=[];view.roi_changed.connect(changes.append);view.set_roi_selection(True)
    rect=view.image_rect()
    outside=None
    for candidate in (QPoint(1,1),QPoint(view.width()-1,1),QPoint(1,view.height()-1),QPoint(view.width()-1,view.height()-1)):
        if not rect.contains(candidate):outside=candidate;break
    if outside is not None:
        qtbot.mousePress(view,Qt.LeftButton,pos=outside);qtbot.mouseRelease(view,Qt.LeftButton,pos=outside)
        assert not changes
    a=QPoint(round(rect.x()+rect.width()*.25),round(rect.y()+rect.height()*.25))
    b=QPoint(round(rect.x()+rect.width()*.75),round(rect.y()+rect.height()*.75))
    qtbot.mousePress(view,Qt.LeftButton,pos=a);qtbot.mouseMove(view,b);qtbot.mouseRelease(view,Qt.LeftButton,pos=b)
    np.testing.assert_allclose(view.roi,(.25,.25,.5,.5),atol=.003)
    assert len(changes)==1 and not view.roi_selecting
    original=view.roi;view.resize(1000,600);qtbot.wait(20);assert view.roi==original
    view.set_roi_selection(True);qtbot.keyClick(view,Qt.Key_Escape);assert view.roi==original and not view.roi_selecting


def test_roi_numeric_independent_views_save_clear(qtbot,tmp_path):
    p=RoiPanel(tmp_path/'roi.yaml');qtbot.addWidget(p)
    p.set_image_size('left',640,480);p.set_image_size('right',640,480)
    for field,value in zip(p.fields,(160,120,320,240)):field.setValue(value)
    p.apply_coordinates();np.testing.assert_allclose(p.rois['left'],(.25,.25,.5,.5))
    assert p.rois['right'] is None
    p.set_roi('right',(.1,.2,.4,.5));p.save_settings()
    loaded=RoiPanel(tmp_path/'roi.yaml');qtbot.addWidget(loaded);assert loaded.settings()==p.settings()
    p.set_roi('left',None);assert p.rois['left'] is None and p.rois['right'] is not None
    p.reset();assert p.settings()=={'left':None,'right':None}


def test_roi_zoom_reselect_cancel_clear_and_new_frames(qtbot):
    view=ImageView('test');qtbot.addWidget(view);view.resize(800,600);view.show()
    frame=np.zeros((480,640,3),np.uint8);view.display(frame)
    view.set_roi((.25,.25,.5,.25))
    assert (view._display_pix().width(),view._display_pix().height())==(320,120)
    assert (view.pix.width(),view.pix.height())==(640,480)
    view.display(frame);assert (view._display_pix().width(),view._display_pix().height())==(320,120)
    view.set_roi_selection(True);assert (view._display_pix().width(),view._display_pix().height())==(640,480)
    qtbot.keyClick(view,Qt.Key_Escape);assert (view._display_pix().width(),view._display_pix().height())==(320,120)
    view.set_roi((.1,.1,.3,.4));assert view._display_pix().width()==192
    view.set_roi(None);assert (view._display_pix().width(),view._display_pix().height())==(640,480)


@pytest.mark.parametrize('rois',[(None,None),((.25,.25,.5,.5),None),((.25,.25,.5,.5),(.25,.25,.5,.5))])
def test_hot3d_roi_uses_displayed_virtual_image_not_native(qtbot,rois):
    from egohand.sources.hot3d import HOT3DSource,HOT3DFrameProcessor
    from egohand.core.pipeline import EgoHandEngine
    path=Path('samples/date/hot3d/train_aria/clip-001851.tar')
    if not path.exists():pytest.skip('Actual HOT3D sample unavailable')
    class Detector:
        last_detection_diagnostics={}
        def __init__(self,roi):self.images=[];self.roi=roi
        def detect(self,image,ts):self.images.append(image.copy());return []
    ds=[Detector(roi) for roi in rois];e=EgoHandEngine(detectors=ds);s=HOT3DSource(path);s.open();s.start()
    try:
        p=HOT3DFrameProcessor();frame=s.read();r=p.process(e,frame,'prediction')
        assert r.diagnostics['detector_input_by_view']==['virtual_rectified_manual_roi' if roi else 'virtual_rectified_full_frame' for roi in rois]
        np.testing.assert_array_equal(ds[0].images[0],e.last_frame.left_image)
        np.testing.assert_array_equal(ds[1].images[0],e.last_frame.right_image)
        assert not np.array_equal(ds[0].images[0],frame.left_image)
        assert 'debug_native_detections' not in r.diagnostics
        n=len(ds[0].images);p.process(e,frame,'ground_truth');assert len(ds[0].images)==n
    finally:s.close()


def test_gui_apply_roi_to_paused_hot3d_reprocesses_and_clears(qtbot):
    from egohand.gui.main_window import MainWindow
    path=Path('samples/date/hot3d/train_aria/clip-001851.tar')
    if not path.exists():pytest.skip('Actual HOT3D sample unavailable')
    w=MainWindow();qtbot.addWidget(w);w.show()
    try:
        w.source_panel.kind.setCurrentIndex(4);w.source_panel.path.setText(str(path.resolve()))
        w.source_panel.hot3d_mode.setCurrentIndex(1);w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=12000)
        if not w.paused:w.toggle_pause()
        roi=(.25,.5,.5,.5);w.roi_panel.set_roi('left',roi)
        qtbot.waitUntil(lambda:w.latest.diagnostics.get('manual_rois',{}).get('left')==roi,timeout=12000)
        assert w.worker.engine.detectors[0].roi==roi and w.worker.engine.detectors[1].roi is None
        assert w.roi_panel.current_side()=='left'
        assert w.latest.diagnostics['detector_input_by_view'][0]=='virtual_rectified_manual_roi'
        w.roi_panel.set_roi('left',None)
        qtbot.waitUntil(lambda:w.worker.engine.detectors[0].roi is None and w.latest.diagnostics.get('manual_rois',{}).get('left') is None,timeout=12000)
        assert w.left_view.roi is None
    finally:w.stop_pipeline();w.close()


def test_single_image_roi_does_not_bypass_rectification(qtbot):
    from egohand.gui.main_window import MainWindow
    path=Path('samples/public/hand.jpg')
    if not path.exists():pytest.skip('Real image unavailable')
    w=MainWindow();qtbot.addWidget(w);w.show()
    try:
        w.source_panel.kind.setCurrentIndex(2);w.source_panel.path.setText(str(path.resolve()));w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None and not w.worker.isRunning(),timeout=12000)
        roi=(.1,.1,.8,.8);w.roi_panel.set_roi('left',roi)
        qtbot.waitUntil(lambda:w.latest is not None and w.latest.diagnostics.get('manual_rois',{}).get('left')==roi,timeout=12000)
        assert w.latest.source_type=='real_single_image'
        assert w.latest.diagnostics['model_status']=='RECTIFICATION_REQUIRED'
        assert not w.latest.left_detections
    finally:w.stop_pipeline();w.close()


def test_paused_roi_keeps_unprocessed_offline_queue_frames():
    from egohand.gui.workers import PipelineWorker
    from egohand.sources.synthetic import SyntheticStereoSource
    from egohand.core.pipeline import EgoHandEngine
    source=SyntheticStereoSource('two_static',frames=3);source.open();source.start()
    worker=PipelineWorker(source,offline=True);worker.engine=EgoHandEngine()
    worker.last_input_frame=source.read();pending=source.read();worker.queue.put(pending)
    worker.pause(True);worker.command('roi_config',{'left':(.25,.25,.5,.5),'right':None});worker._commands()
    assert worker.queue.qsize()==1 and worker.queue.get_nowait() is pending
    assert worker.roi_reprocess_pending and worker.roi_hold_paused
    worker.pause(False);assert not worker.roi_hold_paused


@pytest.mark.parametrize('content',['- 0\n- 1\n','left: [0, null, 0.5, 0.5]\n'])
def test_invalid_roi_config_reports_error_without_crashing(qtbot,tmp_path,content):
    path=tmp_path/'roi.yaml';path.write_text(content,encoding='utf8')
    panel=RoiPanel(path);qtbot.addWidget(panel)
    assert panel.settings()=={'left':None,'right':None}
    assert '未加载' in panel.hint.text()
