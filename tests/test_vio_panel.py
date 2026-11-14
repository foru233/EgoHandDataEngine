import numpy as np
import pyqtgraph.opengl as gl
from PySide6.QtWidgets import QApplication, QFileDialog

from egohand.gui.vio_panel import VioPanel

def test_camera_axes_follow_world_pose():
    from egohand.gui.vio_panel import pose_axes
    pose=np.eye(4);pose[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];pose[:3,3]=[1,2,3]
    points=pose_axes(pose,length=.2)
    np.testing.assert_allclose(points,[[1,2,3],[1,2.2,3],[.8,2,3],[1,2,3.2]],atol=1e-7)

def test_live_camera_pose_readout_uses_current_valid_camera_pose():
    from egohand.gui.vio_panel import live_camera_pose_text
    pose=np.eye(4);pose[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];pose[:3,3]=[1,-2,3]
    text=live_camera_pose_text({'pose_valid':True,'matrix':pose.tolist()})
    assert '+1.000, -2.000, +3.000 m' in text
    assert '+0.0°, +0.0°, +90.0°' in text
    assert '当前不可用' in live_camera_pose_text({'pose_valid':False,'matrix':pose.tolist()})
    assert '显示重锚坐标' in live_camera_pose_text({'pose_valid':True,'matrix':pose.tolist(),'display_map_coordinates_preserved':False})


def test_panel_and_readout_obey_versioned_map_admission(qtbot):
    from egohand.gui.vio_panel import live_camera_pose_text
    from egohand.vio.localization_state import LocalizationPipeline
    from egohand.vio.hybrid_quality import make_quality_gate
    from egohand.vio.map_identity import decode_map_epoch
    pipeline=LocalizationPipeline('stereo',make_quality_gate('stereo',stable_seconds=0))
    def message(stamp):
        return pipeline.process(dict(type='pose',timestamp_ns=stamp,frame_id=stamp,map_epoch=0,
            world_frame_id=0,world_frame_revision=0,**decode_map_epoch(0),snapshot_available=True,
            snapshot_consistent=True,pose_valid=True,tracking_state='OK',matrix=np.eye(4).tolist()),[])
    panel=VioPanel();qtbot.addWidget(panel);panel.begin_live()
    allowed=message(1_000_000_000)
    panel.update_live(allowed)
    assert len(panel.poses)==1
    denied=message(2_000_000_000)
    denied['localization']['admission']['map_pose_available']=False
    assert denied['pose_valid']  # A contradictory legacy bit cannot grant admission.
    panel.update_live(denied)
    assert len(panel.poses)==1 and panel._live_gap_pending
    assert '当前不可用' in live_camera_pose_text(denied)
    assert '暂停追加' in panel.filtered_status.text()

def test_live_pose_status_and_session_separation():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.poses=[object()];panel.begin_live()
    assert not panel.poses and len(panel._live_previous[0])==1
    panel.update_live({'type':'pose','timestamp_ns':1000,'tracking_state':'NOT_INITIALIZED','pose_valid':False,'imu_initialized':False,'matrix':None})
    assert '初始化中' in panel.details.text() and not panel.poses
    panel.update_live({'type':'pose','timestamp_ns':2000,'tracking_state':'OK','pose_valid':True,'imu_initialized':True,'map_epoch':1,'matrix':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]})
    assert len(panel.poses)==1 and '已初始化' in panel.details.text()
    panel.update_live({'type':'pose','timestamp_ns':3000,'tracking_state':'NOT_INITIALIZED','pose_valid':False,'imu_initialized':False,'map_epoch':2,'matrix':None})
    assert not panel.poses and '初始化中' in panel.details.text()

def test_native_restart_clears_old_world_trajectory():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live()
    panel.update_live({'type':'pose','timestamp_ns':2000,'tracking_state':'OK','pose_valid':True,
                       'imu_initialized':True,'map_epoch':0,'matrix':np.eye(4).tolist()})
    assert len(panel.poses)==1
    panel.update_live({'type':'vio_reset','message':'重启并新建地图'})
    assert not panel.poses and panel._live_map_epoch is None
    assert '新建地图' in panel.details.text() and not panel.slider.isEnabled()

def test_imu_only_loss_prediction_is_not_added_as_visual_pose():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live()
    panel.update_live({'type':'pose','timestamp_ns':1000,'tracking_state':'LOST','pose_valid':False,
        'imu_initialized':True,'matrix':None,'recovery_preaccept':{'loss_prediction':{
            'status':'predicted','T_world_from_body':np.eye(4).tolist(),'duration_s':0.1}}})
    assert '纯 IMU 外推' in panel.details.text() and '非视觉位姿' in panel.details.text()
    assert not panel.poses


def test_best_effort_prediction_moves_camera_axes_without_formal_track_append():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live();panel.preview.resize(640,420)
    identity=np.eye(4).tolist()
    panel.update_live({'type':'pose','timestamp_ns':1,'frame_id':1,'tracking_state':'OK',
        'pose_valid':True,'imu_initialized':True,'map_epoch':1,'matrix':identity})
    body=np.eye(4);body[0,3]=.8
    panel.update_live({'type':'pose','timestamp_ns':2,'frame_id':2,'tracking_state':'LOST',
        'pose_valid':False,'imu_initialized':True,'map_epoch':1,'matrix':None,
        'display_pose_contract':{'T_body_from_camera':identity},
        'recovery_preaccept':{'loss_prediction':{'status':'predicted','duration_s':.5,
            'T_world_from_body':body.tolist()}}})
    assert len(panel.poses)==1
    assert panel._display_pose.source=='imu_prediction'
    assert panel._prediction_points and panel._prediction_points[-1][0]==.8
    assert 'IMU 临时预测' in panel.details.text()
    camera_x=panel.preview.axis_items[6]
    np.testing.assert_allclose(camera_x.pos[0],[.8,0,0])
    frozen=panel._display_pose.matrix.copy()
    panel.update_live({'type':'pose','timestamp_ns':3,'frame_id':3,'tracking_state':'LOST',
        'pose_valid':False,'imu_initialized':True,'map_epoch':1,'matrix':None,
        'display_pose_contract':{'T_body_from_camera':identity},
        'recovery_preaccept':{'loss_prediction':{'status':'predicted','duration_s':3.0,
            'T_world_from_body':body.tolist()}}})
    assert panel._display_pose.source=='frozen'
    np.testing.assert_allclose(panel._display_pose.matrix,frozen)

def test_automatic_new_map_status_is_visible():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live()
    panel.update_live({'type':'pose','timestamp_ns':1000,'tracking_state':'OK','pose_valid':False,
        'imu_initialized':True,'matrix':None,'automatic_new_map':True,
        'recovery_preaccept':{'await_visual_requalification':True,'quarantine_elapsed_s':3.0}})
    assert '正在建立新地图' in panel.details.text()

def test_quarantine_report_only_does_not_claim_automatic_new_map():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live()
    panel.update_live({'type':'pose','timestamp_ns':1000,'tracking_state':'OK','pose_valid':False,
        'imu_initialized':True,'matrix':None,
        'recovery_preaccept':{'await_visual_requalification':True,'quarantine_elapsed_s':3.0,
                              'quarantine_timed_out':True,'timeout_action':'report_only'}})
    assert '超时后继续隔离' in panel.details.text()
    assert '自动新建地图' not in panel.details.text()


def test_live_quality_failure_keeps_history_and_breaks_line():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live()
    panel.preview.resize(640, 420)
    def update(stamp, segment, x):
        matrix = np.eye(4)
        matrix[0, 3] = x if x is not None else 0
        panel.update_live({
            'type': 'pose', 'timestamp_ns': stamp, 'tracking_state': 'OK',
            'pose_valid': x is not None, 'imu_initialized': True, 'map_epoch': 1,
            'display_segment': segment, 'quality_status': '位姿突变已拒绝' if x is None else '可信度待验证',
            'matrix': matrix.tolist() if x is not None else None,
        })

    update(1, 0, 0)
    update(2, 0, 1)
    update(3, 1, None)
    assert len(panel.poses) == 2 and panel.slider.isEnabled()
    assert '位姿突变已拒绝' in panel.details.text()

    assert not panel.preview.contains_position([10, 0, 0])
    update(4, 1, 10)
    assert panel.preview.contains_position([10, 0, 0])
    update(5, 1, 11)
    line, = [item for item in panel.preview.trajectory_items if isinstance(item, gl.GLLinePlotItem)]
    assert line.mode == 'lines'
    np.testing.assert_array_equal(line.pos, [[0, 0, 0], [1, 0, 0], [10, 0, 0], [11, 0, 0]])
    assert panel.slider.maximum() == 3
    panel.slider.setValue(0)
    camera_x=panel.preview.axis_items[6]
    np.testing.assert_allclose(camera_x.pos[0], [0, 0, 0])
    panel.clear()
    assert not panel.poses and not panel.preview.trajectory_items

def test_kalibr_camera_imu_transform_is_inverted(tmp_path):
    import numpy as np
    import yaml
    app=QApplication.instance() or QApplication([])
    transform=np.eye(4);transform[:3,3]=[.1,.2,.3]
    path=tmp_path/'camchain.yaml'
    path.write_text(yaml.safe_dump({'cam0':{'T_cam_imu':transform.tolist()}}),encoding='utf8')
    panel=VioPanel();panel.set_calibration(path,kalibr_camera='cam0')
    np.testing.assert_allclose(panel.calibration,np.linalg.inv(transform))


def test_inertial_trajectory_import_and_clear(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    trajectory = tmp_path / 'CameraTrajectory.txt'
    trajectory.write_text('1000000000 1 2 3 0 0 0 1\n1100000000 2 3 4 0 0 0 1\n')
    calibration = tmp_path / 'calibration.yaml'
    calibration.write_text('%YAML:1.0\nIMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [1, 0, 0, 0.1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n')
    files = iter([(str(calibration), ''), (str(trajectory), '')])
    monkeypatch.setattr(QFileDialog, 'getOpenFileName', lambda *args: next(files))
    panel = VioPanel()
    panel.select_calibration()
    panel.import_trajectory()
    assert len(panel.poses) == 2
    assert '1.100' in panel.details.text()
    assert '手部同步：不可用' in panel.details.text()
    panel.slider.setValue(1)
    assert '2.100' in panel.details.text()
    panel.resize(320, 500)
    assert not panel.grab().isNull()
    panel.clear()
    assert not panel.slider.isEnabled()
    assert '未加载' in panel.details.text()
    assert panel.calibration_path==str(calibration.resolve())


def test_estimate_and_truth_can_be_aligned_and_shown_independently(tmp_path,monkeypatch):
    from egohand.gui.vio_panel import NAVIGATION_ROOT
    import numpy as np
    app=QApplication.instance() or QApplication([])
    points=np.array([[0,0,0],[1,0,0],[0,1,0],[1,1,1]],float)
    rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]],float);offset=np.array([2.,3.,4.])
    truth=points@rotation.T+offset
    estimate_file=tmp_path/'CameraTrajectory.txt'
    truth_file=tmp_path/'groundtruth.tum'
    estimate_file.write_text(''.join(f'{(i+1)*1000000000} {p[0]} {p[1]} {p[2]} 0 0 0 1\n' for i,p in enumerate(points)))
    truth_file.write_text(''.join(f'{i+1}.0 {p[0]} {p[1]} {p[2]} 0 0 0 1\n' for i,p in enumerate(truth)))
    cal=tmp_path/'cal.yaml';cal.write_text('%YAML:1.0\nIMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n')
    choices=iter([(str(cal),''),(str(estimate_file),''),(str(truth_file),'')]);directories=[]
    def choose(*args):directories.append(args[2]);return next(choices)
    monkeypatch.setattr(QFileDialog,'getOpenFileName',choose)
    panel=VioPanel();panel.select_calibration();panel.import_trajectory();panel.import_truth()
    assert directories[0]==str(NAVIGATION_ROOT)
    assert panel.alignment and panel.alignment[2]==4
    np.testing.assert_allclose(panel.alignment[0],rotation,atol=1e-6)
    np.testing.assert_allclose(panel.alignment[1],offset,atol=1e-6)
    assert len(panel.preview.trajectory_items)==3
    panel.show_truth.setChecked(False)
    assert len(panel.preview.trajectory_items)==1
    panel.show_estimate.setChecked(False)
    assert len(panel.preview.trajectory_items)==0
    assert len(panel.preview.items)==7  # 网格与世界三轴及其标签保留。
    panel.clear()

def test_calibration_must_be_loaded_and_cannot_change_with_existing_trajectory(tmp_path,monkeypatch):
    app=QApplication.instance() or QApplication([])
    panel=VioPanel()
    monkeypatch.setattr(QFileDialog,'getOpenFileName',lambda *args: (_ for _ in ()).throw(AssertionError('不应提前打开轨迹对话框')))
    panel.import_trajectory()
    assert '请先在' in panel.details.text()
    cal=tmp_path/'cal.yaml';cal.write_text('%YAML:1.0\nIMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n')
    panel.set_calibration(cal);panel.poses=[object()]
    import pytest
    with pytest.raises(ValueError,match='清除轨迹'):panel.set_calibration(tmp_path/'another.yaml')
    assert panel.calibration_path==str(cal.resolve())

def test_no_alignment_when_timestamps_do_not_overlap(tmp_path):
    from egohand.vio.trajectory import load_euroc_trajectory,load_tum_body_ground_truth,align_positions_to_ground_truth
    import numpy as np
    a=tmp_path/'a.txt';a.write_text('1000000000 0 0 0 0 0 0 1\n2000000000 1 0 0 0 0 0 1\n3000000000 0 1 0 0 0 0 1\n')
    b=tmp_path/'b.tum';b.write_text('10 0 0 0 0 0 0 1\n11 1 0 0 0 0 0 1\n12 0 1 0 0 0 0 1\n')
    x=load_euroc_trajectory(a,output_frame='body',calibration_id='test',T_body_from_camera=np.eye(4))
    y=load_tum_body_ground_truth(b,calibration_id='test',T_body_from_camera=np.eye(4))
    assert align_positions_to_ground_truth(x,y) is None

def test_main_window_switches_between_hand_and_pose_views(qtbot,tmp_path):
    from egohand.gui.main_window import MainWindow
    w=MainWindow();qtbot.addWidget(w);w.show()
    assert w.calibration_panel.title()=='双目几何标定'
    assert w.vio_calibration_panel.title()=='相机—IMU 标定'
    before=w.calibration_panel.label.text()
    cal=tmp_path/'cal.yaml';cal.write_text('%YAML:1.0\nIMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n')
    w.vio_panel.set_calibration(cal)
    assert '已加载' in w.vio_calibration_status.text()
    assert w.calibration_panel.label.text()==before
    assert w.scene_stack.currentWidget() is not w.vio_panel
    w.scene_selector.setCurrentIndex(1)
    assert w.scene_stack.currentWidget() is w.vio_panel
    assert w.scene_card.tag.text()=='WORLD · m'
    assert w.joint_info.isHidden()
    w.scene_selector.setCurrentIndex(0)
    assert w.scene_stack.currentWidget() is not w.vio_panel
    assert not w.joint_info.isHidden()
    w.close()
