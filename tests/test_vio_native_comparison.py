import numpy as np
from PySide6.QtWidgets import QApplication
from egohand.gui.vio_panel import VioPanel


def message(t,x,accepted=True,epoch=1):
    pose=np.eye(4);pose[0,3]=x
    return dict(type='pose',timestamp_ns=t,tracking_state='OK',imu_initialized=True,
                map_epoch=epoch,raw_pose_valid=True,raw_matrix=pose.tolist(),
                pose_valid=accepted,matrix=pose.tolist() if accepted else None,
                quality_status='可信度待验证' if accepted else '位姿突变已拒绝')


def test_raw_rejected_jump_is_visible_only_on_raw_side():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live();panel.compare_native.setChecked(True)
    panel.update_live(message(1,0))
    panel.update_live(message(2,8,False))
    assert len(panel.poses)==1
    assert panel._raw_poses[-1][0,3]==8
    assert '+8.000' in panel.raw_status.text()
    assert '暂停追加' in panel.filtered_status.text()
    assert '帧时间 2 ns' in panel.raw_status.text()
    panel.compare_native.setChecked(False)
    assert panel.raw_container.isHidden()
    assert len(panel.poses)==1
    panel.compare_native.setChecked(True)
    assert not panel.raw_container.isHidden()
    panel.close()


def test_raw_loss_breaks_line_and_reset_does_not_join_worlds():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.begin_live()
    panel.update_live(message(1,0))
    lost=message(2,0,False);lost.update(raw_pose_valid=False,raw_matrix=None)
    panel.update_live(lost)
    assert panel._raw_current is None
    panel.update_live(message(3,1))
    assert panel._raw_breaks==[1]
    panel.update_live(message(4,12,False,epoch=2))
    assert len(panel._raw_poses)==1 and not panel._raw_breaks
    panel.update_live(dict(type='vio_reset',message='新会话'))
    assert not panel._raw_poses and not panel.poses
    panel.close()


def test_comparison_shares_camera_and_never_uses_body_prediction():
    app=QApplication.instance() or QApplication([])
    panel=VioPanel();panel.compare_native.setChecked(True)
    panel.preview.opts['distance']=17
    panel._sync_comparison_camera()
    assert panel.raw_preview.opts['distance']==17
    panel.update_live(dict(type='pose',timestamp_ns=1,map_epoch=1,pose_valid=False,
        raw_pose_valid=False,matrix=None,raw_matrix=None,tracking_state='LOST',
        recovery_preaccept={'loss_prediction':{'status':'predicted','T_world_from_body':np.eye(4).tolist()}}))
    assert not panel._raw_poses
    panel.close()
