from dataclasses import replace
from egohand.gui.debug_page import debug_snapshot,DebugPage
from egohand.core.types import StereoHandResult


def test_debug_candidates_and_final_output(engine,source):
    r=engine.process(source.read());summary,ds,js,_=debug_snapshot(r)
    assert len(ds)==84 and len(js)==84
    assert sum(row[9] for row in js)==42
    assert '非模型' in summary
    assert all(row[10]=='STEREO_OBSERVED' for row in js if row[9])


def test_debug_missing_view_does_not_invent_candidates(engine,source):
    r=engine.process(replace(source.read(),right_detections=[]))
    summary,ds,js,_=debug_snapshot(r)
    assert len(ds)==42 and len(js)==42 and '无逐关节候选证据' in summary
    assert all(row[10]=='UNMATCHED' and '缺少另一侧' in row[11] for row in js)


def test_debug_gt_not_mediapipe_and_old_record():
    r=StereoHandResult(0,0,None,'hot3d_clips',diagnostics={'result_kind':'official_ground_truth'})
    summary,ds,js,_=debug_snapshot(r)
    assert '未运行检测模型' in summary and not ds and not js


def test_debug_window_freeze_and_empty(qtbot,engine,source):
    from PySide6.QtWidgets import QWidget
    owner=QWidget();owner.latest=engine.process(source.read());qtbot.addWidget(owner)
    page=DebugPage(owner);qtbot.addWidget(page);page.show();page.refresh(force=True)
    assert page.detections.rowCount()==84 and page.joints.rowCount()==84
    page.freeze.setChecked(True);owner.latest=None;page.refresh()
    assert page.detections.rowCount()==84
    page.refresh(force=True)
    assert page.detections.rowCount()==0 and page.json.toPlainText()==''
    page.close()

def test_debug_window_and_columns_resize_with_content(qtbot,engine,source):
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QWidget
    owner=QWidget();owner.latest=engine.process(replace(source.read(),right_detections=[]));qtbot.addWidget(owner)
    page=DebugPage(owner);qtbot.addWidget(page);page.show();page.refresh(force=True)
    assert page.windowFlags() & Qt.WindowMaximizeButtonHint
    assert page.isSizeGripEnabled()
    page.resize(950,650)
    assert page.width()==950 and page.height()==650
    reason_column=page.joints.columnCount()-1
    text=page.joints.item(0,reason_column).text()
    assert page.joints.columnWidth(reason_column)>=page.joints.fontMetrics().horizontalAdvance(text)
    page.joints.setColumnWidth(reason_column,100)
    page.refresh(force=True)
    assert page.joints.columnWidth(reason_column)==100
    page.fit_columns()
    assert page.joints.columnWidth(reason_column)>=page.joints.fontMetrics().horizontalAdvance(text)
    page.close()


def test_four_views_accept_only_final_valid_points(engine,source):
    from egohand.gui.debug_page import render_debug_views
    import numpy as np
    f=source.read();r=engine.process(f);images=render_debug_views(r,engine.last_frame)
    assert len(images)==4 and all(i.shape==f.left_image.shape for i in images)
    green=lambda img:np.sum((img[:,:,1]>200)&(img[:,:,0]<120)&(img[:,:,2]<120))
    assert green(images[2])>0 and green(images[3])>0
    for h in r.hands:
        for j in h.joints:j.valid=False
    rejected=render_debug_views(r,engine.last_frame)
    assert green(rejected[2])==0 and green(rejected[3])==0


def test_truth_comparison_uses_matched_same_frame_pixels(engine,source):
    import numpy as np
    from egohand.gui.debug_page import render_truth_comparison
    f=source.read();r=engine.process(f);frame=engine.last_frame
    truths=[]
    for ds in (r.left_detections,r.right_detections):
        truths.append({d.handedness:(d.pixel_landmarks.copy()+[2,0],d.valid.copy()) for d in ds})
    frame.debug_ground_truth_uv=truths
    image=render_truth_comparison(r,frame)
    assert image.shape==(960,640,3)
    assert np.sum((image[:,:,0]>180)&(image[:,:,1]<120)&(image[:,:,2]>180))>0
    assert np.array_equal(truths[0]['Left'][0],r.left_detections[0].pixel_landmarks+[2,0])


def test_truth_comparison_no_gt_and_gt_mode(engine,source):
    from egohand.gui.debug_page import render_truth_comparison
    f=source.read();r=engine.process(f);frame=engine.last_frame
    assert render_truth_comparison(r,frame).shape==(960,640,3)
    r.diagnostics['result_kind']='official_ground_truth'
    assert render_truth_comparison(r,frame).shape==(960,640,3)

