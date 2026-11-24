"""Delivery lifecycle regression; run with tests/conftest.py loaded via tests directory."""
import sys
import pytest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from client_gui import ClientMainWindow


def test_compact_toolbar_and_tiles_keep_controls_readable(qtbot):
    from PySide6.QtCore import QPoint
    from PySide6.QtWidgets import QPushButton, QToolButton
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        w.resize(900,620);w.show();qtbot.wait(150)
        assert not w._toolbar_wide
        bounds=w.command_strip.rect()
        assert len({group.y() for group in w.command_groups})>1
        for group in w.command_groups:
            assert bounds.contains(group.geometry())
            for button in group.findChildren(QPushButton)+group.findChildren(QToolButton):
                if button.isVisible() and button.text():
                    assert button.width() >= button.sizeHint().width()-2, button.text()
        for control in (w.camera_mount_controls,w.ground_controls,w.stop_button):
            assert bounds.contains(control.mapTo(w.command_strip,QPoint(control.width()-1,control.height()-1)))
        assert w.raw_left_tile.geometry().bottom()+1 == w.raw_right_tile.geometry().top()
        assert w.raw_right_tile.geometry().bottom()+1 == w.depth_tile.geometry().top()
        assert w.left_tile.geometry().bottom()+1 == w.right_tile.geometry().top()
        assert w.record_output_parent.name=='ori'
        assert w.result_output_parent is None
        w.resize(1920,900);qtbot.wait(100)
        assert w._toolbar_wide
        record=w.record_controls.geometry();compute=w.compute_controls.geometry()
        assert w.fit_controls.geometry().left()>compute.right()
        assert record.left()>w.fit_controls.geometry().right()
        assert record.height()==compute.height()
        assert record.top()==compute.top()
        assert {widget.height() for widget in (w.start_button,w.compute_button,w.view_menu_button,
                w.source,w.hand_filter_toggle,w.body_gate_toggle,w.ground_adjust)}=={32}
    finally:w.close()


def test_rotation_pivot_uses_display_coordinates(qtbot):
    import numpy as np
    from types import SimpleNamespace
    from delivery_views import DISPLAY_WORLD_ROTATION
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        w._world_pose=np.eye(4);w._world_pose[:3,3]=[1.,2.,3.]
        w._displayed_hands=[SimpleNamespace(joints=[SimpleNamespace(valid=True,xyz_m=[.2,.3,.4])])]
        w.body_pose=SimpleNamespace(joints={'Pelvis':[1.,3.,3.],'Chest':[1.,1.,3.]})
        for key,world in (('world',[0,0,0]),('camera',[1,2,3]),('hands',[1.2,2.3,3.4]),('body',[1,2,3])):
            w.pivot_choice.setCurrentIndex(w.pivot_choice.findData(key))
            p=w.preview.opts['center']
            np.testing.assert_allclose([p.x(),p.y(),p.z()],DISPLAY_WORLD_ROTATION@np.asarray(world),atol=1e-6)
    finally:w.close()


def test_head_wireframe_follows_pose_and_short_direction():
    import numpy as np
    from delivery_views import head_wireframe
    center=np.array([1.,2.,3.]); rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    points=head_wireframe(center,rotation)
    assert points.shape==(146,3)
    np.testing.assert_allclose(points[-2],center)
    np.testing.assert_allclose(points[-1],center+rotation[:,2]*.03)
    assert np.isfinite(points).all()


def test_saved_depth_rejects_old_frame_and_coalesces_pending(qtbot,monkeypatch):
    from types import SimpleNamespace
    import numpy as np
    w=ClientMainWindow();qtbot.addWidget(w)
    active=SimpleNamespace(isRunning=lambda:True)
    try:
        w.depth_task=active;w.generation=10
        w._request_saved_depth('old_frame','cal')
        w.generation=11
        w._request_saved_depth('current_frame','cal')
        assert w._pending_saved_depth==(11,'current_frame','cal')
        image=np.full((8,8,3),100,dtype=np.uint8)
        w.on_depth(10,image,'old')
        assert w.depth_view.pix is None
        started=[]
        monkeypatch.setattr(w,'_start_depth_task',lambda *args:started.append(args))
        w._depth_finished(active)
        assert started==[(11,'current_frame','cal')]
        w.on_depth(11,image,'深度·已完成')
        assert w.depth_view.pix is not None
    finally:
        w.depth_task=None;w.close()


def test_saved_frame_requests_depth_from_matching_replay(qtbot,monkeypatch):
    from types import SimpleNamespace
    import numpy as np
    from egohand.core.types import StereoFrame,StereoHandResult
    w=ClientMainWindow();qtbot.addWidget(w)
    frame=StereoFrame(4,123,np.zeros((48,64,3),np.uint8),np.zeros((48,64,3),np.uint8),'cal','recorded')
    result=StereoHandResult(4,123,'cal','recorded')
    requests=[]
    class Replay:
        calibration=object()
        def seek(self,index):assert index==0
        def read(self):return frame
        def close(self):pass
    try:
        w.saved=[result];w.saved_replay=Replay()
        monkeypatch.setattr(w,'_request_saved_depth',lambda f,c:requests.append((f,c)))
        w.display_saved(0)
        assert not w.depth_tile.isHidden()
        assert requests==[(frame,w.saved_replay.calibration)]
        assert w.depth_view.title=='深度计算中'
        # Missing raw stereo must not retain a previous frame's depth.
        w.saved_replay=None
        w.display_saved(0)
        assert w.depth_view.pix is None and '不可用' in w.depth_tile.caption.text()
    finally:w.close()


def test_saved_playback_waits_for_depth_and_pause_cancels_advance(qtbot,monkeypatch):
    from types import SimpleNamespace
    import numpy as np
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        w.saved=[SimpleNamespace(timestamp_ns=i*33_333_333) for i in range(4)]
        w.result_slider.valueChanged.disconnect(w.display_saved)
        w.result_slider.setRange(0,3)
        w.depth_task=SimpleNamespace(isRunning=lambda:True,generation=w.generation)
        w.saved_play.setText('暂停')
        w._advance_saved()
        assert w._saved_depth_advance is not None and not w.saved_timer.isActive()
        w.on_depth(w.generation,np.zeros((8,8,3),np.uint8),'深度')
        assert w.saved_timer.isActive() and w._saved_depth_advance is None
        w.saved_timer.stop();w._advance_saved()
        assert w._saved_depth_advance is not None
        w.toggle_saved_playback()
        assert w._saved_depth_advance is None and w.saved_play.text()=='播放'
        w.on_depth(w.generation,np.zeros((8,8,3),np.uint8),'深度')
        assert not w.saved_timer.isActive()
    finally:w.depth_task=None;w.close()


def test_saved_playback_speed_and_seek(qtbot, monkeypatch):
    from types import SimpleNamespace
    import client_gui
    import numpy as np
    now=[0.]
    monkeypatch.setattr(client_gui,'time',SimpleNamespace(perf_counter=lambda:now[0]))
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        w.saved=[SimpleNamespace(timestamp_ns=i*100_000_000,diagnostics={}) for i in range(12)]
        w.result_slider.valueChanged.disconnect(w.display_saved)
        monkeypatch.setattr(w,'display_saved',lambda index:None)
        w.result_slider.setRange(0,11)
        w.toggle_saved_playback()
        w.saved_timer.stop();w._saved_depth_advance=(0,now[0])
        speed_index=w.saved_speed.findData(2.)
        w.saved_speed.setCurrentIndex(speed_index);w.saved_speed.activated.emit(speed_index)
        assert w._saved_depth_advance is not None and not w.saved_timer.isActive()
        w.on_depth(w.generation,np.zeros((8,8,3),np.uint8),'深度')
        assert w.saved_timer.isActive() and w.saved_timer.interval()==50
        now[0]=.18
        w.saved_timer.stop();w._advance_saved()
        assert w.result_slider.value()==1 and w.saved_timer.isActive()
        w._saved_seek_started();w.result_slider.setValue(6);w._saved_seek_finished()
        assert w.result_slider.value()==6 and w.saved_timer.isActive()
        fastest=w.saved_speed.findText('最快')
        w.saved_speed.setCurrentIndex(fastest);w.saved_speed.activated.emit(fastest)
        w._normalize_saved_speed()
        assert w.saved_speed.currentText()=='最快' and w.saved_timer.interval()==1
        displayed=[]
        w.result_slider.valueChanged.connect(displayed.append)
        for _ in range(5):
            now[0]+=1.
            w.saved_timer.stop();w._advance_saved()
        assert displayed==[7,8,9,10,11]
        assert w.result_slider.value()==11 and not w.saved_timer.isActive()
        assert w.saved_play.text()=='播放'
        w.saved_speed.setEditText('12.5')
        assert abs(w._saved_speed_value()-12.5)<1e-6
        w.saved_speed.setEditText('999')
        assert w._saved_speed_value()==32.
    finally:w.close()


def test_saved_seek_release_commits_position_even_without_value_changed(qtbot, monkeypatch):
    from types import SimpleNamespace
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        w.saved=[SimpleNamespace(timestamp_ns=i*100_000_000) for i in range(4)]
        calls=[]
        monkeypatch.setattr(w,'display_saved',lambda index:calls.append(index))
        w.result_slider.setRange(0,3)
        w.result_slider.setValue(2)
        w._saved_seek_finished()
        assert calls == [2]
    finally:w.close()


def test_choose_output_parent(qtbot, monkeypatch, tmp_path):
    from PySide6.QtWidgets import QFileDialog
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        monkeypatch.setattr(QFileDialog,'getExistingDirectory',lambda *args: str(tmp_path))
        w.choose_result_output()
        assert w.result_output_parent==tmp_path
        assert str(tmp_path) in w.output_label.text()
        assert w.output_button.text()=='结果目录'
    finally:w.close()


def test_body_switches_are_independent_and_default_safe(qtbot):
    w = ClientMainWindow(); qtbot.addWidget(w)
    try:
        assert not w.body_gate_toggle.isChecked()
        assert w.body_toggle.isChecked()
        assert w.hand_filter_toggle.isChecked()
        assert w.ground_adjust.minimum() == 30
        assert w.ground_adjust.maximum() == 210
        w.ground_adjust.setValue(60)
        assert w.ground_reference.height_m == .6 and w._ground_override
        w.body_toggle.setChecked(False)
        assert not w.body_gate_toggle.isChecked() and not w.body_toggle.isChecked()
        w.body_toggle.setChecked(True)
        assert not w.body_gate_toggle.isChecked() and w.body_toggle.isChecked()
        assert w.body_overlay.visible
        w.body_gate_toggle.setChecked(True)
        assert w.body_gate_toggle.isChecked() and w.body_toggle.isChecked()
        w.body_toggle.setChecked(False)
        assert w.body_gate_toggle.isChecked() and not w.body_toggle.isChecked()
        assert not w.body_overlay.visible
    finally:
        w.close()


def test_ground_slider_and_number_stay_in_sync_and_refresh(qtbot, monkeypatch):
    from types import SimpleNamespace
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        result=SimpleNamespace()
        w.latest=result
        updates=[]
        monkeypatch.setattr(w, '_update_body_pose', lambda value: updates.append(value))
        w.ground_slider.setValue(300)
        assert w.ground_adjust.value()==30 and w.ground_reference.height_m==.30
        assert updates==[result]
        w.ground_adjust.setValue(42)
        assert w.ground_slider.value()==420 and w.ground_reference.height_m==.42
        assert updates==[result,result]
        assert w.ground_slider.hasTracking()
    finally:
        w.latest=None
        w.close()


def test_saved_slider_defers_decoding_until_release(qtbot, monkeypatch):
    from types import SimpleNamespace
    from PySide6.QtCore import Qt
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        w.saved=[SimpleNamespace(timestamp_ns=i*33_000_000) for i in range(10)]
        w.result_slider.setRange(0,9)
        calls=[]
        w.result_slider.valueChanged.disconnect(w.display_saved)
        w.result_slider.valueChanged.connect(calls.append)
        assert not w.result_slider.hasTracking()
        w.result_slider.setSliderDown(True)
        w.result_slider.setSliderPosition(3)
        w.result_slider.setSliderPosition(7)
        assert calls==[]
        w.result_slider.setSliderDown(False)
        assert calls==[7]
    finally:w.close()


def test_body_overlay_submits_exact_camera_and_dual_hand_input(qtbot,monkeypatch):
    import numpy as np
    from types import SimpleNamespace
    w = ClientMainWindow(); qtbot.addWidget(w)
    points = np.zeros((21, 3), dtype=float)
    valid = np.zeros(21, dtype=bool); valid[[0, 5, 9, 13, 17]] = True
    hands = [
        {'handedness': 'Left', 'track_id': 1, 'xyz_world_m': (points + [-.35, .15, 1.0]).tolist(), 'valid': valid.tolist()},
        {'handedness': 'Right', 'track_id': 2, 'xyz_world_m': (points + [.35, .15, 1.0]).tolist(), 'valid': valid.tolist()},
    ]
    result = SimpleNamespace(frame_id=4, timestamp_ns=100, session_id='s', world_hands={
        'status': 'available', 'frame_id': 4, 'timestamp_ns': 100, 'session_id': 's',
        'hands': hands, 'world_reference': {'map_epoch': None, 'world_frame_revision': None,
                                             'reference_generation': None}},
        diagnostics={'vio_pose': {'pose_valid': True, 'matrix': np.eye(4).tolist(), 'timestamp_ns': 100}})
    from egohand.core.types import Hand3D
    result.hands=[]
    for side,x in [('Left',-.35),('Right',.35)]:
        h=Hand3D(side,1.,len(result.hands)+1)
        h.joints[0].valid=True;h.joints[0].xyz_m=np.array([x,.15,1.])
        result.hands.append(h)
    requests=[]
    monkeypatch.setattr(w.smpl_controller,'request',lambda p:requests.append(p) or 1)
    try:
        w.body_toggle.setChecked(True)
        w._update_body_pose(result)
        assert w.body_pose is None and w.body_meta.text()=='SMPL · 求解中'
        assert len(requests)==1
        np.testing.assert_allclose(requests[0]['head_world'],[0,0,0])
        np.testing.assert_allclose(requests[0]['wrists']['left'],[-.35,.15,1.])
        np.testing.assert_allclose(requests[0]['wrists']['right'],[.35,.15,1.])
    finally:
        w.close()


def test_completed_recording_starts_background_cache(qtbot, monkeypatch, tmp_path):
    import json
    from types import SimpleNamespace
    import client_gui
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        (tmp_path/'reprocessing.json').write_text(json.dumps({'status':'completed','result_count':3}),encoding='utf8')
        jobs=[]
        monkeypatch.setattr(client_gui.subprocess,'Popen',lambda args,**kwargs: jobs.append((args,kwargs)) or SimpleNamespace(poll=lambda:None))
        assert w.camera_forehead_offset.value()==7.5
        w._computed_options={'strict_body':False,'initial_camera_height_m':1.65,'camera_forehead_offset_mm':12.5}
        w._computed_output=tmp_path
        w.on_finished()
        assert len(jobs)==2 and all(str(tmp_path) in job[0] for job in jobs)
        assert any('build_saved_replay_cache.py' in job[0][1] for job in jobs)
        pose_job=next(job for job in jobs if 'build_unified_smpl_replay.py' in job[0][1])
        assert '--in-place' in pose_job[0] and '--mano' in pose_job[0]
        assert '--initial-camera-height-m' in pose_job[0]
        assert pose_job[0][pose_job[0].index('--camera-forehead-offset-mm')+1]=='12.5'
        assert w._cache_jobs and '缓存' in w.output_label.text()
    finally:w.close()


def test_successful_record_stop_precomputes_meshes_once(qtbot,monkeypatch,tmp_path):
    w=ClientMainWindow();qtbot.addWidget(w)
    (tmp_path/'results').mkdir();(tmp_path/'results/frames.jsonl').write_text('',encoding='utf8')
    jobs=[];monkeypatch.setattr(w,'_start_pose_derivation',jobs.append)
    w.recording_path=tmp_path
    w.on_recording(True,False,'录制中');assert not jobs
    w.on_recording(True,True,'录制暂停');assert not jobs
    w.on_recording(False,False,'录制已停止');assert jobs==[tmp_path]
    w._finish_recording_mesh_cache();assert jobs==[tmp_path]
    w.close()


def test_background_pose_and_replay_jobs_report_their_own_results(qtbot, tmp_path):
    from types import SimpleNamespace
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        (tmp_path/'display_cache.sqlite3').touch()
        notes=[]
        w._output_note=notes.append
        w._cache_jobs=[(SimpleNamespace(poll=lambda:0),tmp_path,'pose'),
                       (SimpleNamespace(poll=lambda:0),tmp_path,'replay')]
        w._poll_cache_jobs()
        assert any('SMPL / MANO 网格缓存' in x and '失败' in x for x in notes)
        assert any('拖动缓存' in x and '完成' in x for x in notes)
        assert not w._cache_jobs
    finally:w.close()


def test_live_hands_bind_only_exact_frame_in_either_arrival_order(qtbot):
    import numpy as np
    from types import SimpleNamespace
    w=ClientMainWindow();qtbot.addWidget(w);w.vio_panel.begin_live()
    def pose(stamp,epoch=0,valid=True):
        matrix=np.eye(4);matrix[0,3]=stamp
        return dict(type='pose',timestamp_ns=stamp,pose_valid=valid,tracking_state='OK',
                    imu_initialized=True,map_epoch=epoch,matrix=matrix.tolist() if valid else None)
    def frame(stamp,**diagnostics):
        return SimpleNamespace(timestamp_ns=stamp,diagnostics=diagnostics)
    try:
        w.on_vio(pose(1))
        w._bind_displayed_hands([],frame(2))
        assert w._world_pose is None  # Newest available pose belongs to another frame.
        w.on_vio(pose(2))
        assert w._world_pose[0,3]==2  # Late exact result refreshes current hands.
        w.on_vio(pose(3))
        assert w._world_pose[0,3]==2  # A newer pose must not move an older hand frame.
        w._bind_displayed_hands([],frame(3))
        assert w._world_pose[0,3]==3
        w._bind_displayed_hands([],frame(3,vio_pose=pose(3,valid=False)))
        assert w._world_pose is None  # Explicit frame rejection wins over cache.
        w.on_vio(pose(3))
        assert w._world_pose is None  # Nor can a later callback replace that rejection.
        w.on_vio(pose(4,epoch=1))
        w._bind_displayed_hands([],frame(2))
        assert w._world_pose is None  # Old world entries were discarded.
        w.on_vio({'type':'vio_reset'})
        w._bind_displayed_hands([],frame(4))
        assert w._world_pose is None
        for stamp in range(100,180):w.on_vio(pose(stamp,epoch=1))
        assert len(w._hand_poses)==64 and 100 not in w._hand_poses
    finally:w.close()


def test_saved_invalid_or_unpaired_hand_frame_never_reuses_prior_pose(qtbot):
    import numpy as np
    from types import SimpleNamespace
    w=ClientMainWindow();qtbot.addWidget(w)
    transform=np.eye(4);transform[0,3]=5
    def result(stamp,pose):
        return SimpleNamespace(timestamp_ns=stamp,hands=[],diagnostics={'vio_pose':pose} if pose else {})
    def pose(stamp,valid=True):
        return dict(type='pose',timestamp_ns=stamp,pose_valid=valid,tracking_state='OK' if valid else 'LOST',
                    map_epoch=0,imu_initialized=True,matrix=transform.tolist() if valid else None)
    w.saved=[result(1,pose(1)),result(2,pose(2,False)),result(3,None),result(4,pose(1))]
    try:
        w.display_saved(0)
        np.testing.assert_array_equal(w._world_pose,transform)
        for index in (1,2,3):
            w.display_saved(index)
            assert w._world_pose is None
        assert len(w.vio_panel.poses)==2  # Camera history is independent of current hand binding.
    finally:w.close()


def test_saved_unavailable_local_pose_keeps_camera_data_without_segment_overlay(qtbot):
    import numpy as np
    from egohand.core.types import Hand3D,StereoHandResult
    hand=Hand3D('Right',1.,1)
    hand.joints[0].valid=True;hand.joints[0].xyz_m=np.array([.1,.2,.3]);hand.valid_joint_count=1
    result=StereoHandResult(7,123,'cal','vendor_vs030',hands=[hand])
    result.local_motion={'status':'unavailable','pose_valid':False}
    result.diagnostics['vio_pose']={'type':'pose','timestamp_ns':123,'pose_valid':False,
        'tracking_state':'LOST','map_epoch':0,'matrix':None,'local_motion':result.local_motion}
    w=ClientMainWindow();qtbot.addWidget(w);w.saved=[result]
    try:
        w.hand_overlay.update(result.hands,np.eye(4))
        assert w.hand_overlay.has_geometry()
        w.display_saved(0)
        assert w._world_pose is None and not w.hand_overlay.has_geometry()
        assert w.latest.hands[0].joints[0].valid
        np.testing.assert_array_equal(w.latest.hands[0].joints[0].xyz_m,[.1,.2,.3])
    finally:w.close()


def test_customer_world_pose_obeys_contract_even_when_legacy_bit_is_true(qtbot):
    import numpy as np
    w=ClientMainWindow();qtbot.addWidget(w);w.vio_panel.begin_live()
    row=dict(type='pose',timestamp_ns=1,tracking_state='OK',pose_valid=True,
             imu_initialized=True,map_epoch=0,matrix=np.eye(4).tolist(),
             localization={'schema_version':1,'contract_version':2,'admission':{'map_pose_available':False}})
    try:
        w.on_vio(row)
        assert w._world_pose is None and not w.vio_panel.poses
        assert '等待稳定跟踪' in w.world_meta.text()
    finally:w.close()


def test_customer_lifecycle_and_mode_lock(qtbot):
    w=ClientMainWindow();qtbot.addWidget(w);w.show()
    try:
        assert w.mode.currentData()=='live_recovery_v4'
        assert len(w.column_widgets)==3
        w.enable_demo()
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=15000)
        assert w.left_view.pix is not None and w.right_view.pix is not None
        assert not w.mode.isEnabled()
        # 手部骨架现在画在与相机位姿同一个三维视图里。
        assert w.hand_overlay.items
        assert w.hands_toggle.isChecked() and w.trajectory_toggle.isChecked()
        w.hands_toggle.setChecked(False)
        assert not any(item.visible() for item in w.hand_overlay.items)
        w.hands_toggle.setChecked(True)
        assert all(item.visible() for item in w.hand_overlay.items)
        worker=w.session.worker
        assert w.stop_pipeline() and not worker.isRunning()
        assert w.mode.isEnabled()
        w.mode.addItem('测试纯双目','stereo');w.mode.setCurrentIndex(w.mode.findData('stereo'))
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=15000)
        assert w.session.worker.vio_mode=='stereo'
        assert w.stop_pipeline()
    finally:w.stop_pipeline();w.close()


def test_depth_display_search_window_covers_near_limit():
    """显示深度的搜索窗口按 0.25 m 实际所需反推，且必须是 16 的倍数。"""
    from delivery_views import DepthCache
    # fx=89.25 px、baseline=65.55 mm：0.25 m 需要 23.4 px，向上取到 16 的倍数 32。
    assert DepthCache.required_disparities(89.25, 0.06555, .25) == 32
    for fx in (10.0, 44.6, 89.25, 198.7885, 1000.0):
        window = DepthCache.required_disparities(fx, 0.06555)
        assert window % 16 == 0
        assert 16 <= window <= 96
        ideal = fx * 0.06555 / 0.10
        if ideal <= 96:
            # 窗口必须覆盖 0.25 m 处的视差，否则近端会被静默截断。
            assert window >= ideal
        else:
            # 超出上限时保持原厂默认窗口，不静默放大。
            assert window == 96
    assert DepthCache.required_disparities(10.0, 0.06555) == 16
    assert DepthCache.required_disparities(1000.0, 0.06555) == 96


def test_depth_range_controls_only_display_and_filter_valid_values(qtbot):
    import numpy as np
    from delivery_views import colorize_depth
    values=np.array([[0.,.12,.15,1.,3.,4.,np.nan]],dtype=np.float32)
    color,mask=colorize_depth(values,.15,3.,True)
    assert mask.tolist()==[[False,False,True,True,True,False,False]]
    assert not color[0,1].any() and color[0,2].any()
    _,all_mask=colorize_depth(values,.15,3.,False)
    assert all_mask.tolist()==[[False,True,True,True,True,True,False]]
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        assert not hasattr(w,'depth_near') and not hasattr(w,'depth_filter')
        assert w.depth_tile.caption.text() == '深度'
        assert '全部有效值' in w.depth_tile.caption.toolTip()
    finally:w.close()


def test_depth_display_can_include_20cm_without_changing_mapping_default():
    import numpy as np
    from egohand.vio.map_depth import StereoDepth
    class Matcher:
        def compute(self,left,right):return np.full(left.shape,80,dtype=np.int16)
    engine=StereoDepth(np.eye(3),1.,1.);engine.matcher=Matcher()
    image=np.zeros((3,12),dtype=np.uint8)
    assert engine.estimate(image,image)[1,8]==0
    assert engine.estimate(image,image,min_depth_m=.01,max_depth_m=float('inf'))[1,8]==pytest.approx(.2)


def test_hands_are_placed_in_world_frame_with_current_pose(qtbot):
    """手部必须按当前 T_world_camera 变换；无位姿时退回相机原点，不冒充世界坐标。"""
    import numpy as np
    from egohand.core.types import Hand3D
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        hand=Hand3D(handedness='Right',handedness_score=1.0,track_id=1)
        for joint in hand.joints:
            joint.xyz_m=np.array([0.0,0.05,0.5])   # 相机前方 0.5 m
            joint.valid=True
        transform=np.eye(4);transform[:3,3]=[1.0,2.0,3.0]
        w.hand_overlay.update([hand],transform)
        assert w.hand_overlay.items
        assert w.hand_overlay.items[-1].pxMode is False
        assert w.hand_overlay.items[-1].size == .012
        points=np.asarray(w.hand_overlay.items[-1].pos)
        np.testing.assert_allclose(points,np.tile([1.0,3.5,-2.05],(len(points),1)),atol=1e-6)
        # 逆变换应回到相机系，确认不是随便平移
        from delivery_views import DISPLAY_WORLD_ROTATION
        back=(points-transform[:3,3]@DISPLAY_WORLD_ROTATION.T)@DISPLAY_WORLD_ROTATION
        np.testing.assert_allclose(back,np.tile([0.0,0.05,0.5],(len(back),1)),atol=1e-6)
        # 无有效位姿：按相机原点显示
        w.hand_overlay.update([hand],None)
        points=np.asarray(w.hand_overlay.items[-1].pos)
        np.testing.assert_allclose(points,np.tile([0.0,0.5,-0.05],(len(points),1)),atol=1e-6)
        # 手部开关只影响可见性，不改变坐标
        w.hands_toggle.setChecked(False)
        assert not any(item.visible() for item in w.hand_overlay.items)
    finally:w.close()


def test_three_columns_are_world_native_depth_and_rectified_pairs(qtbot):
    w=ClientMainWindow();qtbot.addWidget(w)
    try:
        world,middle,right=w.column_widgets
        # 左列：唯一的二维/三维世界视图（相机轨迹 + 手部）。
        assert world.isAncestorOf(w.preview)
        # 中列（小）：原始鱼眼左/右 + 深度，纵向一列。
        assert middle.isAncestorOf(w.raw_left_tile) and middle.isAncestorOf(w.raw_right_tile)
        assert middle.isAncestorOf(w.depth_tile)
        # 右列（大）：校正后的左右图及手部识别。
        assert right.isAncestorOf(w.left_tile) and right.isAncestorOf(w.right_tile)
        assert w.left_view is not w.raw_left_view and w.right_view is not w.raw_right_view
        # 中列的深度与原始左右图是纵向一列，不是一行并列。
        w.show();qtbot.wait(50)
        assert w.raw_left_tile.y()<w.raw_right_tile.y()<w.depth_tile.y()
        assert w.raw_left_tile.x()==w.depth_tile.x()
        # 右列按两张图排版，因此比中列的三张图更宽。
        assert right.width()>middle.width()
        # 研究用控件（滑块、详情、地图、对比）在客户界面保持隐藏。
        for name in ('slider','details','raw_container','filtered_status'):
            assert not getattr(w.vio_panel,name).isVisible()
    finally:w.close()


def test_missing_recording_is_friendly(qtbot,tmp_path):
    w=ClientMainWindow();qtbot.addWidget(w)
    w.select_recording(tmp_path/'missing');w.start_pipeline()
    qtbot.waitUntil(lambda:not w.session.worker or not w.session.worker.isRunning(),timeout=10000)
    assert w.stop_pipeline()
    w.close()


def test_customer_pause_freezes_and_resumes_live_preview(qtbot):
    w=ClientMainWindow();qtbot.addWidget(w);w.show()
    try:
        assert not w.pause_button.isEnabled()
        w.enable_demo()
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=15000)
        worker=w.session.worker
        w.pause_button.click()
        assert w.paused and worker.pause_event.is_set()
        assert w.pause_button.text()=='恢复' and '已暂停' in w.state_label.text()
        qtbot.wait(300)  # allow an in-flight frame to finish
        frozen=w.latest.frame_id
        qtbot.wait(350)
        assert w.latest.frame_id==frozen
        w.pause_button.click()
        assert not w.paused and not worker.pause_event.is_set()
        assert w.pause_button.text()=='暂停'
        qtbot.waitUntil(lambda:w.latest.frame_id!=frozen,timeout=5000)
        assert w.stop_pipeline() and not w.pause_button.isEnabled()
    finally:w.stop_pipeline();w.close()


def test_recorded_pause_keeps_display_and_restores_worker(qtbot):
    import threading
    import numpy as np
    class FakeWorker:
        def __init__(self):
            self.stop_event=threading.Event();self.pause_event=threading.Event()
            self.history=[];self.output=[];self.running=True
        def isRunning(self):return self.running
        def pause(self,value):
            self.pause_event.set() if value else self.pause_event.clear()
        def stop(self):self.stop_event.set();self.running=False
        def wait(self,_timeout):return True
        def deleteLater(self):pass
    w=ClientMainWindow();qtbot.addWidget(w)
    worker=FakeWorker();w.session.worker=worker
    w.source.setCurrentIndex(w.source.findData('recorded'));w.vio_panel.begin_live();w.sync_controls()
    def pose(t):
        return {'type':'pose','timestamp_ns':t,'tracking_state':'OK','pose_valid':True,
                'imu_initialized':True,'map_epoch':0,'matrix':np.eye(4).tolist()}
    try:
        w.on_vio(pose(1));assert len(w.vio_panel.poses)==1
        w.pause_button.click();assert worker.pause_event.is_set()
        w.on_vio(pose(2));assert len(w.vio_panel.poses)==1
        w.pause_button.click();assert not worker.pause_event.is_set()
        w.on_vio(pose(3));assert len(w.vio_panel.poses)==2
        assert not w.vio_panel._live_gap_pending
    finally:w.stop_pipeline();w.close()


def test_recorded_worker_waits_at_frame_boundary_until_resume(qtbot,monkeypatch,tmp_path):
    import threading
    from types import SimpleNamespace
    from egohand.gui.workers import PipelineWorker
    from egohand.vio import replay_runtime

    first_done=threading.Event();release=threading.Event();finished=threading.Event()
    frames=[];errors=[]
    class FakeReplay:
        def __init__(self,*args,on_frame,**kwargs):self.on_frame=on_frame
        def run(self):
            self.on_frame(0,{'timestamp_ns':1});first_done.set()
            if not release.wait(5):raise RuntimeError('test timeout')
            self.on_frame(1,{'timestamp_ns':2})
            return {'status':'completed'}
        def cancel(self):release.set()
    monkeypatch.setattr(replay_runtime,'ReplayVio',FakeReplay)
    source=SimpleNamespace(recording_dir=tmp_path,
        frame_from_pair=lambda pair:SimpleNamespace(metadata={},index=pair))
    worker=PipelineWorker(source,offline=True,live_vio=True,vio_mode='stereo',
                          vio_output_dir=tmp_path/'out')
    def process(frame,_started):
        frames.append(frame.index);worker.processed+=1;return {'index':frame.index}
    monkeypatch.setattr(worker,'_process_frame',process)
    def run():
        try:worker._run_recorded_vio(0)
        except Exception as exc:errors.append(exc)
        finally:finished.set()
    thread=threading.Thread(target=run);thread.start()
    monkeypatch.setattr(worker,'isRunning',lambda:thread.is_alive())
    w=ClientMainWindow();qtbot.addWidget(w);w.session.worker=worker
    w.source.setCurrentIndex(w.source.findData('recorded'));w.sync_controls()
    try:
        qtbot.waitUntil(first_done.is_set,timeout=3000)
        w.pause_button.click();release.set()
        qtbot.wait(200)
        assert frames==[0] and worker.pause_event.is_set()
        w.pause_button.click()
        qtbot.waitUntil(finished.is_set,timeout=3000)
        assert frames==[0,1] and not errors
    finally:
        worker.stop();release.set();thread.join(timeout=3);w.stop_pipeline();w.close()


def test_live_reset_while_paused_is_applied_before_resuming(qtbot):
    import threading
    import numpy as np
    class FakeWorker:
        def __init__(self):
            self.stop_event=threading.Event();self.pause_event=threading.Event()
            self.history=[];self.output=[];self.running=True
        def isRunning(self):return self.running
        def pause(self,value):self.pause_event.set() if value else self.pause_event.clear()
        def stop(self):self.stop_event.set();self.running=False
        def wait(self,_timeout):return True
        def deleteLater(self):pass
    w=ClientMainWindow();qtbot.addWidget(w);w.session.worker=FakeWorker()
    w.source.setCurrentIndex(w.source.findData('vendor'));w.vio_panel.begin_live();w.sync_controls()
    def pose(t,epoch=0):
        return {'type':'pose','timestamp_ns':t,'tracking_state':'OK','pose_valid':True,
                'imu_initialized':True,'map_epoch':epoch,'matrix':np.eye(4).tolist()}
    try:
        w.on_vio(pose(1));assert len(w.vio_panel.poses)==1
        w.pause_button.click()
        w.on_vio(pose(2,epoch=1));w.on_vio(pose(3,epoch=0))
        assert len(w.vio_panel.poses)==1
        w.pause_button.click();assert not w.vio_panel.poses
        w.on_vio(pose(4));assert len(w.vio_panel.poses)==1
        w.pause_button.click()
        w.on_vio({'type':'vio_reset','message':'原生进程重启'})
        w.on_vio(pose(5));assert len(w.vio_panel.poses)==1
        w.pause_button.click();assert not w.vio_panel.poses
        w.on_vio(pose(6));assert len(w.vio_panel.poses)==1
        w.pause_button.click();w.on_vio(pose(7));w.pause_button.click()
        w.on_vio(pose(8));assert len(w.vio_panel.poses)==2
        assert w.vio_panel._live_breaks==[1]
    finally:w.stop_pipeline();w.close()


def test_hybrid_source_switch_keeps_same_world_history_but_breaks_segments(qtbot):
    """Visual -> VIO -> visual -> VIO is one map, not four new maps."""
    import numpy as np

    w = ClientMainWindow()
    qtbot.addWidget(w)
    w.vio_panel.begin_live()
    try:
        def send(stamp, source, segment, *, map_epoch=7, revision=2, valid=True):
            transform = np.eye(4)
            transform[0, 3] = stamp * 0.1
            w.on_vio({
                'type': 'pose', 'mode': 'hybrid', 'timestamp_ns': int(stamp * 1e9),
                'tracking_state': 'OK' if valid else 'LOST',
                'pose_valid': valid, 'imu_initialized': source == 'vio',
                'map_epoch': map_epoch, 'world_frame_revision': revision,
                'display_segment': segment, 'pose_source': source,
                'matrix': transform.tolist() if valid else None,
            })

        for stamp, source, segment in (
            (1, 'visual_fallback', 0), (2, 'vio', 1),
            (3, 'visual_fallback', 2), (4, 'vio', 3)
        ):
            send(stamp, source, segment)

        assert len(w.vio_panel.poses) == 4
        assert w.vio_panel._live_breaks == [1, 2, 3]
        assert w.vio_panel._live_map_epoch == (7, 2)
        np.testing.assert_allclose(
            [pose.T_world_from_camera_left[0, 3] for pose in w.vio_panel.poses],
            [0.1, 0.2, 0.3, 0.4],
        )

        # A temporary lost pose breaks the visible line but not the map/history.
        send(5, 'vio', 3, valid=False)
        assert len(w.vio_panel.poses) == 4
        send(6, 'vio', 4)
        assert len(w.vio_panel.poses) == 5
        assert w.vio_panel._live_breaks == [1, 2, 3, 4]

        # A world-frame reanchor, even within the same map epoch, must clear.
        send(7, 'vio', 5, revision=3)
        assert len(w.vio_panel.poses) == 1
        assert w.vio_panel._live_breaks == []
        assert w.vio_panel._live_map_epoch == (7, 3)

        # A true map switch must also clear.
        send(8, 'visual_fallback', 6, map_epoch=8, revision=3)
        assert len(w.vio_panel.poses) == 1
        assert w.vio_panel._live_map_epoch == (8, 3)

        # Native restart is different from a mere pose-source change.
        w.on_vio({'type': 'vio_reset', 'message': '原生进程重启'})
        assert not w.vio_panel.poses
        assert w.vio_panel._live_map_epoch is None
    finally:
        w.close()
