from PySide6.QtWidgets import QApplication, QFileDialog

from egohand.gui.vio_panel import VioPanel

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

def test_live_quality_failure_splits_trajectory_and_reports_reason():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live()
    m=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    panel.update_live({'type':'pose','timestamp_ns':1,'tracking_state':'OK','pose_valid':True,'imu_initialized':True,'map_epoch':1,'display_segment':0,'matrix':m})
    panel.update_live({'type':'pose','timestamp_ns':2,'tracking_state':'OK','pose_valid':False,'imu_initialized':True,'map_epoch':1,'display_segment':1,'quality_status':'位姿突变已拒绝','matrix':None})
    assert not panel.poses and '位姿突变已拒绝' in panel.details.text()

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
    assert len(panel.preview.trajectory_items)==4
    panel.show_truth.setChecked(False)
    assert len(panel.preview.trajectory_items)==2
    panel.show_estimate.setChecked(False)
    assert len(panel.preview.trajectory_items)==0
    assert len(panel.preview.items)==1  # 网格不会随重复刷新被移除或重复添加。
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
