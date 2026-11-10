"""Exercise complete mouse gestures, including a viewport moving under the cursor."""
import numpy as np
import pytest
from types import SimpleNamespace
from PySide6.QtCore import QEvent,QPoint,QPointF,Qt
from PySide6.QtGui import QMouseEvent,QVector4D
from PySide6.QtWidgets import QApplication,QWidget
from egohand.visualization.skeleton_view import SkeletonView

def test_auto_fit_uses_same_coordinates_as_hand_and_camera_axis(view):
    from egohand.gui.main_window import MainWindow
    from PySide6.QtGui import QVector3D
    joint=SimpleNamespace(xyz_m=(.2,.1,1.0),valid=True)
    result=SimpleNamespace(hands=[SimpleNamespace(joints=[joint])],diagnostics={'show3d_sequence_id':'test'})
    owner=SimpleNamespace(_hot3d_fit_key=None,skeleton=view)
    view.set_rotation_center_mode('camera')
    MainWindow.fit_hot3d_scene(owner,result)
    np.testing.assert_allclose(view.opts['center'].toTuple(),(0,0,0),atol=1e-6)
    forward=view.camera_axis.transform().map(QVector3D(0,0,1))
    np.testing.assert_allclose(forward.toTuple(),(0,1,0),atol=1e-6)
    before=view.grid.transform().map(QVector3D(0,0,0)).toTuple()
    view._place_grid()
    np.testing.assert_allclose(view.grid.transform().map(QVector3D(0,0,0)).toTuple(),before)

def test_default_camera_and_grid_stay_fixed_as_hand_moves(view,engine,source):
    result=engine.process(source.read());view.display(result)
    center=view.opts['center'].toTuple();grid=view.grid.transform().copyDataTo()
    for hand in result.hands:
        for joint in hand.joints:
            if joint.valid:joint.xyz_m+=np.array([.2,0,.15])
    view.display(result)
    np.testing.assert_allclose(view.opts['center'].toTuple(),center)
    np.testing.assert_allclose(view.grid.transform().copyDataTo(),grid)

def test_select_rotation_center_and_keep_last_on_missing_hand(view,engine,source):
    from egohand.visualization.skeleton_view import optical_to_display
    assert view.rotation_center_mode=='camera'
    view.set_rotation_center_mode('hand')
    result=engine.process(source.read());view.display(result)
    xyz=optical_to_display([j.xyz_m for h in result.hands for j in h.joints if j.valid])
    hand=(xyz.min(axis=0)+xyz.max(axis=0))/2
    assert view.rotation_center_mode=='hand'
    np.testing.assert_allclose(view.opts['center'].toTuple(),hand,atol=1e-6)
    view.set_rotation_center_mode('camera')
    np.testing.assert_allclose(view.opts['center'].toTuple(),(0,0,0),atol=1e-6)
    view.set_rotation_center_mode('range')
    bounds=np.vstack((xyz,np.zeros((1,3))))
    np.testing.assert_allclose(view.opts['center'].toTuple(),(bounds.min(axis=0)+bounds.max(axis=0))/2,atol=1e-6)
    view.set_rotation_center_mode('hand')
    previous=view.opts['center'].toTuple()
    view.display(SimpleNamespace(hands=[],diagnostics={}))
    np.testing.assert_allclose(view.opts['center'].toTuple(),previous,atol=1e-6)

def mouse(view,event_type,local,button=Qt.NoButton,buttons=Qt.NoButton,modifiers=Qt.NoModifier,global_pos=None):
    local=QPointF(*local)
    if global_pos is None:global_pos=QPointF(view.mapToGlobal(local.toPoint()))
    event=QMouseEvent(event_type,local,QPointF(global_pos),button,buttons,modifiers)
    QApplication.sendEvent(view,event)

def drag(view,start,end,button=Qt.LeftButton,modifiers=Qt.NoModifier):
    mouse(view,QEvent.MouseButtonPress,start,button,button,modifiers)
    mouse(view,QEvent.MouseMove,end,buttons=button,modifiers=modifiers)
    mouse(view,QEvent.MouseButtonRelease,end,button,modifiers=modifiers)

@pytest.fixture
def view(qtbot):
    widget=SkeletonView();widget.resize(640,480);qtbot.addWidget(widget);widget.show()
    widget.setCameraPosition(elevation=20,azimuth=-90)
    qtbot.waitExposed(widget)
    return widget

def test_successive_drags_use_the_new_press_position(view,engine,source):
    view.display(engine.process(source.read()))
    drag(view,(70,80),(95,90))
    before=(view.opts["azimuth"],view.opts["elevation"])
    mouse(view,QEvent.MouseButtonPress,(400,280),Qt.LeftButton,Qt.LeftButton)
    assert (view.opts["azimuth"],view.opts["elevation"])==before
    mouse(view,QEvent.MouseMove,(402,283),buttons=Qt.LeftButton)
    assert view.opts["azimuth"]==pytest.approx(before[0]-2)
    assert view.opts["elevation"]==pytest.approx(before[1]+3)
    mouse(view,QEvent.MouseButtonRelease,(402,283),Qt.LeftButton)

@pytest.mark.parametrize("button,modifiers",[(Qt.LeftButton,Qt.NoModifier),(Qt.MiddleButton,Qt.NoModifier),(Qt.LeftButton,Qt.ControlModifier),(Qt.MiddleButton,Qt.ControlModifier)])
def test_moving_viewport_does_not_add_pointer_delta(qtbot,button,modifiers):
    parent=QWidget();parent.resize(900,700);qtbot.addWidget(parent)
    view=SkeletonView(parent);view.setGeometry(10,10,640,480);parent.show();qtbot.waitExposed(parent)
    view.setCameraPosition(elevation=20,azimuth=-90)
    # Compare to the exact same movement in an unmoved viewport.
    drag(view,(100,100),(102,103),button,modifiers)
    expected=(view.opts["azimuth"],view.opts["elevation"],np.array(view.opts["center"].toTuple()))
    view.reset_camera();view.setCameraPosition(elevation=20,azimuth=-90)
    press_global=QPointF(view.mapToGlobal(QPoint(100,100)))
    mouse(view,QEvent.MouseButtonPress,(100,100),button,button,modifiers,press_global)
    view.move(90,50)
    mouse(view,QEvent.MouseMove,(22,63),buttons=button,modifiers=modifiers,global_pos=press_global+QPointF(2,3))
    assert view.opts["azimuth"]==pytest.approx(expected[0])
    assert view.opts["elevation"]==pytest.approx(expected[1])
    np.testing.assert_allclose(view.opts["center"].toTuple(),expected[2],atol=1e-7)
    mouse(view,QEvent.MouseButtonRelease,(22,63),button,modifiers=modifiers,global_pos=press_global+QPointF(2,3))

def test_mouse_move_after_release_cannot_resume_old_drag(view):
    drag(view,(60,60),(80,65))
    before=(view.opts["azimuth"],view.opts["elevation"])
    # A move without a new press (e.g. entering while a button is held) is inert.
    mouse(view,QEvent.MouseMove,(400,300),buttons=Qt.LeftButton)
    assert (view.opts["azimuth"],view.opts["elevation"])==before

def test_focus_loss_cancels_the_gesture(view):
    mouse(view,QEvent.MouseButtonPress,(60,60),Qt.LeftButton,Qt.LeftButton)
    QApplication.sendEvent(view,QEvent(QEvent.FocusOut))
    before=(view.opts["azimuth"],view.opts["elevation"])
    mouse(view,QEvent.MouseMove,(400,300),buttons=Qt.LeftButton)
    assert (view.opts["azimuth"],view.opts["elevation"])==before
    drag(view,(400,300),(402,303))
    assert view.opts["azimuth"]==pytest.approx(before[0]-2)
    assert view.opts["elevation"]==pytest.approx(before[1]+3)

def test_joint_pick_occurs_on_click_release_not_drag(view,engine,source,qtbot):
    result=engine.process(source.read());view.display(result)
    matrix=view.projectionMatrix()*view.viewMatrix()
    point=view.points[0][0]
    clip=matrix.map(QVector4D(*map(float,point),1.))
    p=((clip.x()/clip.w()+1)*view.width()/2,(1-clip.y()/clip.w())*view.height()/2)
    picks=[];view.joint_selected.connect(picks.append)
    mouse(view,QEvent.MouseButtonPress,p,Qt.LeftButton,Qt.LeftButton)
    assert not picks
    mouse(view,QEvent.MouseButtonRelease,p,Qt.LeftButton)
    assert len(picks)==1
    mouse(view,QEvent.MouseButtonPress,p,Qt.LeftButton,Qt.LeftButton)
    mouse(view,QEvent.MouseMove,(p[0]+30,p[1]+15),buttons=Qt.LeftButton)
    mouse(view,QEvent.MouseButtonRelease,(p[0]+30,p[1]+15),Qt.LeftButton)
    assert len(picks)==1

def test_joint_readout_does_not_reflow_the_viewport(qtbot):
    from egohand.gui.main_window import MainWindow
    w=MainWindow();qtbot.addWidget(w);w.resize(1100,720);w.show();qtbot.wait(100)
    before=(w.skeleton.mapToGlobal(w.skeleton.rect().topLeft()),w.skeleton.size())
    w.show_joint("track 12345 · 20 PINKY_TIP · X -0.12345  Y +0.23456  Z +0.98765 m · STEREO_OBSERVED")
    qtbot.wait(100)
    after=(w.skeleton.mapToGlobal(w.skeleton.rect().topLeft()),w.skeleton.size())
    assert after==before
