"""Delivery lifecycle regression; run with tests/conftest.py loaded via tests directory."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from client_gui import ClientMainWindow


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
        w.source.setCurrentIndex(w.source.findData('synthetic'))
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
        w.mode.setCurrentIndex(w.mode.findData('stereo'))
        w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=15000)
        assert w.session.worker.vio_mode=='stereo'
        assert w.stop_pipeline()
    finally:w.stop_pipeline();w.close()


def test_depth_display_search_window_covers_near_limit():
    """显示深度的搜索窗口按 0.25 m 实际所需反推，且必须是 16 的倍数。"""
    from delivery_views import DepthCache
    # fx=89.25 px、baseline=65.55 mm：0.25 m 需要 23.4 px，向上取到 16 的倍数 32。
    assert DepthCache.required_disparities(89.25, 0.06555) == 32
    for fx in (10.0, 44.6, 89.25, 198.7885, 1000.0):
        window = DepthCache.required_disparities(fx, 0.06555)
        assert window % 16 == 0
        assert 16 <= window <= 96
        ideal = fx * 0.06555 / 0.25
        if ideal <= 96:
            # 窗口必须覆盖 0.25 m 处的视差，否则近端会被静默截断。
            assert window >= ideal
        else:
            # 超出上限时保持原厂默认窗口，不静默放大。
            assert window == 96
    assert DepthCache.required_disparities(10.0, 0.06555) == 16
    assert DepthCache.required_disparities(1000.0, 0.06555) == 96


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
        points=np.asarray(w.hand_overlay.items[-1].pos)
        np.testing.assert_allclose(points,np.tile([1.0,2.05,3.5],(len(points),1)),atol=1e-6)
        # 逆变换应回到相机系，确认不是随便平移
        back=(points-transform[:3,3])@transform[:3,:3]
        np.testing.assert_allclose(back,np.tile([0.0,0.05,0.5],(len(back),1)),atol=1e-6)
        # 无有效位姿：按相机原点显示
        w.hand_overlay.update([hand],None)
        points=np.asarray(w.hand_overlay.items[-1].pos)
        np.testing.assert_allclose(points,np.tile([0.0,0.05,0.5],(len(points),1)),atol=1e-6)
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
        w.source.setCurrentIndex(w.source.findData('synthetic'))
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
