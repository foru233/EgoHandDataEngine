"""Exercise draw-time freshness and vetoes before queued UI status delivery."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest
from conftest import calibration
from test_live_localization_hand_join import live_context
from egohand.gui.shared.hand_presentation import HandPresentationSnapshot
from client_gui import ClientMainWindow
import client_gui


@pytest.fixture
def presentation(live_context, monkeypatch):
    c=live_context
    frame,result,pose=c.sample()
    c.join.offer(result,frame,calibration=c.calibration)
    [(saved,_)]=c.join.add_pose(pose)
    snapshot=HandPresentationSnapshot()
    snapshot.publish_pose(pose);snapshot.publish_pair(saved)
    worker=SimpleNamespace(live_vio=SimpleNamespace(source_session_id='e'*32),hand_presentation=snapshot)
    overlay=SimpleNamespace(items=[],clear=lambda:overlay.items.clear(),update=lambda h,t:overlay.items.append(t.copy()),has_geometry=lambda:bool(overlay.items))
    footer=SimpleNamespace(value='')
    footer.text=lambda:footer.value
    footer.setText=lambda text:setattr(footer,'value',text)
    display=SimpleNamespace(_paint_latest_pair=True,paused=False,_live_hand_result=None,
        _live_hand_pose=None,_live_hand_age_ms=None,_last_vio_message=copy.deepcopy(pose),
        session=SimpleNamespace(worker=worker),hand_overlay=overlay,
        footer=footer)
    display._live_hand_transform=lambda:ClientMainWindow._live_hand_transform(display)
    display._show_hand_overlay=lambda *a,**k:ClientMainWindow._show_hand_overlay(display,*a,**k)
    tick=[saved.timestamp_ns+100_000_000]
    monkeypatch.setattr(client_gui.time,'perf_counter_ns',lambda:tick[0])
    return SimpleNamespace(c=c,saved=saved,pose=pose,display=display,snapshot=snapshot,tick=tick,overlay=overlay)


def test_pair_can_paint_before_queued_callback_and_expires_at_draw(presentation):
    p=presentation
    assert p.display._live_hand_result is None
    ClientMainWindow._prepare_live_hand_paint(p.display)
    assert p.display._live_hand_result.frame_id==p.saved.frame_id
    np.testing.assert_allclose(p.display._live_hand_pose,np.eye(4),atol=1e-12)
    assert p.overlay.items
    assert '100 ms' in p.display.footer.text()
    ClientMainWindow._prepare_live_hand_paint(p.display)
    assert len(p.overlay.items)==1  # an unchanged paint cannot schedule redraws
    # No new callbacks: a later draw must still remove expired geometry.
    p.tick[0]=p.saved.timestamp_ns+251_000_000
    ClientMainWindow._prepare_live_hand_paint(p.display)
    assert p.display._live_hand_pose is None and not p.overlay.items
    assert p.saved.world_hands['status']=='available'  # writer data unchanged


def test_geometry_work_cannot_carry_result_past_deadline(presentation):
    p=presentation
    p.tick[0]=p.saved.timestamp_ns+249_000_000
    original=p.overlay.update
    def expensive_update(hands,transform):
        original(hands,transform)
        p.tick[0]+=3_000_000
    p.overlay.update=expensive_update
    ClientMainWindow._prepare_live_hand_paint(p.display)
    assert p.display._live_hand_pose is None and not p.overlay.items
    assert '等待定位' in p.display.footer.text()


@pytest.mark.parametrize('fault',['lost','reset','error','new_segment','new_source','quarantine'])
def test_newest_runtime_veto_applies_before_queued_gui_status(presentation,fault):
    p=presentation
    ClientMainWindow._prepare_live_hand_paint(p.display)
    assert p.overlay.items
    status=copy.deepcopy(p.pose)
    if fault in ('reset','error'):status={'type':'vio_reset' if fault=='reset' else 'error'}
    elif fault=='lost':status['pose_valid']=False
    elif fault=='new_source':status['live_capture']['source_session_id']='f'*32
    elif fault=='new_segment':status['local_motion']['segment_id']='new-independent'
    else:status['recovery_preaccept']['await_visual_requalification']=True
    p.snapshot.publish_pose(status)
    # UI still has the preceding valid status: cannot override producer veto.
    assert p.display._last_vio_message['pose_valid'] is True
    ClientMainWindow._prepare_live_hand_paint(p.display)
    assert p.display._live_hand_pose is None and not p.overlay.items


def test_pause_and_worker_replacement_do_not_leak_old_pair(presentation):
    p=presentation;p.display.paused=True
    ClientMainWindow._prepare_live_hand_paint(p.display)
    assert p.display._live_hand_result is None
    p.display.paused=False
    p.display.session.worker=SimpleNamespace(live_vio=SimpleNamespace(source_session_id='f'*32),
        hand_presentation=HandPresentationSnapshot())
    ClientMainWindow._prepare_live_hand_paint(p.display)
    assert p.display._live_hand_result is None and not p.overlay.items


def test_snapshot_owns_data_and_does_not_replace_latest_with_old_pair(presentation):
    p=presentation
    old=copy.deepcopy(p.saved);old.timestamp_ns-=1
    p.snapshot.publish_pair(old)
    p.saved.hands[0].joints[0].xyz_m[:]=99
    p.pose['pose_valid']=False
    result,status=p.snapshot.read()
    assert result.timestamp_ns>old.timestamp_ns and status['pose_valid'] is True
    np.testing.assert_allclose(result.hands[0].joints[0].xyz_m,[.1,.2,1.])
