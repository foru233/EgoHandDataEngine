"""History survives reference transitions without becoming current localization."""
import numpy as np
import pyqtgraph.opengl as gl

from egohand.gui.vio_panel import VioPanel


def pose(frame, x, *, epoch=3, generation=1, valid=True):
    from egohand.vio.map_identity import decode_map_epoch
    matrix=np.eye(4);matrix[0,3]=x
    result=dict(type='pose',frame_id=frame,timestamp_ns=frame*33_333_333,
                map_epoch=epoch,world_frame_revision=0,reference_generation=generation,
                tracking_state='OK' if valid else 'RECENTLY_LOST',imu_initialized=True,
                pose_valid=valid,matrix=matrix.tolist() if valid else None)
    result.update(**decode_map_epoch(epoch),world_frame_id=epoch,map_change_index=0,
                  snapshot_available=True,snapshot_consistent=True)
    report={k:result[k] for k in ('frame_id','map_id','reset_id','map_revision','map_change_index','timestamp_ns')}
    report.update(schema_version=1,generation=generation,operation_sequence=generation,
                  atlas_generation=2,geometry_sequence=frame,snapshot_consistent=True,
                  frame_bound=True,basis='successful_initializer_commit',coordinate_continuity_verified=False)
    result['reference_frame']=report
    return result


def test_short_loss_keeps_current_history_and_breaks_line(qtbot):
    panel=VioPanel();qtbot.addWidget(panel);panel.begin_live()
    panel.update_live(pose(1,1));panel.update_live(pose(2,2))
    panel.update_live(pose(3,3,valid=False));panel.update_live(pose(4,4))
    assert len(panel.poses)==3 and panel._live_breaks==[2]
    assert not panel._trajectory_history and not panel.history_button.isEnabled()


def test_reference_update_archives_before_new_pose_and_does_not_overlay(qtbot):
    panel=VioPanel();qtbot.addWidget(panel);panel.begin_live()
    panel.update_live(pose(1,1));panel.update_live(pose(2,2))
    panel.update_live(pose(3,3,valid=False));panel.update_live(pose(4,4))
    panel.update_live(pose(5,50,epoch=6,generation=2,valid=False))
    assert not panel.poses
    archived,=panel._trajectory_history
    assert archived.reference==(3,0,1) and archived.breaks==(2,)
    np.testing.assert_array_equal([p.T_world_from_camera_left[0,3] for p in archived.poses],[1,2,4])
    assert not panel.preview.trajectory_items
    panel.update_live(pose(6,60,epoch=6,generation=2))
    assert len(panel.poses)==1 and len(panel._trajectory_history)==1
    assert not panel.preview.trajectory_items  # One new point; no historical line in live view.
    panel.show_trajectory_history()
    viewer=panel._history_window
    assert viewer.selection.count()==1 and viewer.slider.maximum()==2
    viewer.slider.setValue(0)
    np.testing.assert_allclose(viewer.preview.axis_items[6].pos[0],[1,0,0])
    # Seeking history changes neither the live selector, axes nor admitted path.
    np.testing.assert_allclose(panel.preview.axis_items[6].pos[0],[60,0,0])
    assert panel.poses[0].T_world_from_camera_left[0,3]==60
    assert not viewer.preview.hand_items
    with np.testing.assert_raises(ValueError):archived.poses[0].T_world_from_camera_left[0,3]=9
    viewer.close()


def test_map_switch_restart_and_session_boundaries(qtbot):
    panel=VioPanel();qtbot.addWidget(panel);panel.begin_live()
    panel.update_live(pose(1,1))
    panel.update_live(pose(2,2,epoch=1<<40,generation=0))
    assert len(panel._trajectory_history)==1
    panel.update_live(dict(type='vio_reset',message='native restart'))
    panel.update_live(dict(type='vio_reset',message='retry'))
    assert len(panel._trajectory_history)==2 and not panel.poses
    panel.show_trajectory_history()
    panel.begin_live()
    assert not panel._trajectory_history and not panel.history_button.isEnabled()
    assert not panel._history_window.isVisible()
    panel.update_live(pose(3,3));panel.update_live(pose(4,4,epoch=6,generation=2))
    assert len(panel._trajectory_history)==1
    panel.clear()
    assert not panel._trajectory_history and not panel.poses


def test_history_button_is_accessible_in_shipping_gui(qtbot):
    from client_gui import ClientMainWindow
    window=ClientMainWindow();qtbot.addWidget(window);window.show()
    window.vio_panel.begin_live()
    window.on_vio(pose(1,1));window.on_vio(pose(2,2))
    window.on_vio(pose(3,30,epoch=6,generation=2))
    button=window.vio_panel.history_button
    assert button.isVisible() and button.isEnabled() and '1' in button.text()
    button.click()
    viewer=window.vio_panel._history_window
    assert viewer.isVisible() and viewer.selection.count()==1
    old_line,=[item for item in viewer.preview.trajectory_items if isinstance(item,gl.GLLinePlotItem)]
    np.testing.assert_allclose(old_line.pos,[[1,0,0],[2,0,0]])
    assert not window.hand_overlay.has_geometry()
    assert window.vio_panel.poses[0].T_world_from_camera_left[0,3]==30
    viewer.close()
