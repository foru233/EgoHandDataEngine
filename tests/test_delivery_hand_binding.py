"""A trusted same-capture live pose must reach segment output and both consumers."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from conftest import calibration
from test_live_localization_hand_join import live_context,worker_context
from egohand.vio.hand_pose_binding import consumer_result_display_pose
from egohand.vio.local_segments import segment_hands_for_result

APP=Path(__file__).resolve().parents[1]


def gui_binding(result):
    spec=importlib.util.spec_from_file_location('isolated_live_capture_client',APP/'userrr/client_gui.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    display=SimpleNamespace(_hand_poses={},_world_pose=None)
    module.ClientMainWindow._bind_displayed_hands(display,result.hands,result)
    return display._world_pose


@pytest.mark.parametrize('pose_first',[False,True])
def test_two_clock_exact_capture_reaches_segment_gui_and_preserves_pose(live_context,pose_first):
    c=live_context;frame,result,pose=c.sample()
    original=copy.deepcopy(pose)
    assert result.timestamp_ns!=pose['timestamp_ns']
    if pose_first:c.join.add_pose(pose)
    ready=c.join.offer(result,frame,calibration=c.calibration)
    if not pose_first:ready=c.join.add_pose(pose)
    [(saved,_)]=ready
    assert saved.world_hands['status']==saved.segment_hands['status']=='available'
    assert saved.timestamp_ns==result.timestamp_ns
    assert saved.segment_hands['pose_timestamp_ns']==pose['timestamp_ns']
    assert saved.segment_hands['segment_id']==pose['local_motion']['segment_id']
    np.testing.assert_allclose(saved.segment_hands['hands'][0]['xyz_segment_m'][0],[.1,.2,1.],atol=1e-12)
    np.testing.assert_allclose(consumer_result_display_pose(saved),np.eye(4),atol=1e-12)
    np.testing.assert_allclose(gui_binding(saved),np.eye(4),atol=1e-12)
    np.testing.assert_array_equal(saved.hands[0].joints[0].xyz_m,[.1,.2,1.])
    assert saved.diagnostics['live_vio_pose']==original and pose==original
    assert saved.segment_hands['cross_segment_merge_permitted'] is False


@pytest.mark.parametrize('target,key,value',[
    ('capture','source_session_id','f'*32),('capture','sdk_sequence',999),
    ('capture','source_frame_id',999),('capture','hand_timestamp_ns',999),
    ('capture','sdk_device_timestamp_raw',999),('capture','hand_timestamp_kind','sdk'),
    ('capture','transport_pairing','nearest_timestamp'),('capture','exposure_synchrony_verified',True),
    ('join','recovery_frame_sequence',999),('join','camera_geometry_sha256','f'*64),
    ('config','fingerprint','f'*64),('config','calibration_file_sha256',None),
    ('world','pose_timestamp_ns',999),('world','calibration_id','other'),
    ('report','stage','native'),('pose','pose_valid',False),
])
def test_stale_wrong_capture_or_uncommitted_pose_never_reaches_segment_or_gui(live_context,target,key,value):
    c=live_context;frame,result,pose=c.sample()
    c.join.offer(result,frame,calibration=c.calibration)
    [(saved,_)]=c.join.add_pose(pose)
    damaged=copy.deepcopy(saved);bound=damaged.diagnostics['live_vio_pose']
    targets=dict(capture=bound['live_capture'],join=damaged.world_hands['join'],config=bound['live_config'],
        world=damaged.world_hands,report=bound['recovery_preaccept'],pose=bound)
    targets[target][key]=value
    assert segment_hands_for_result(damaged,bound,damaged.world_hands)['status']=='unavailable'
    assert consumer_result_display_pose(damaged) is None and gui_binding(damaged) is None
    np.testing.assert_array_equal(damaged.hands[0].joints[0].xyz_m,[.1,.2,1.])


def test_async_finalization_updates_preview_segment_and_local_pose_together(live_context):
    import time
    c=live_context;worker=worker_context(c,None)
    frame,_,pose=c.sample()
    preview=worker._process_frame(frame,time.perf_counter())
    assert preview.world_hands['status']=='pending'
    worker._vio_status(pose);worker._drain_live_world_join()
    assert preview.world_hands['status']==preview.segment_hands['status']=='available'
    assert preview.local_motion['pose_valid'] is True
    np.testing.assert_allclose(gui_binding(preview),np.eye(4),atol=1e-12)
    assert worker.output[-1][1] is preview


@pytest.mark.parametrize('pose_first',[False,True])
def test_completed_pair_reaches_display_while_earlier_recorder_result_waits(live_context,pose_first):
    c=live_context;seen=[];c.join.on_exact_pair=seen.append
    first=c.sample(1);second=c.sample(2)
    assert c.join.offer(first[1],first[0],calibration=c.calibration)==[]
    if pose_first:assert c.join.add_pose(second[2])==[]
    ready=c.join.offer(second[1],second[0],calibration=c.calibration)
    if not pose_first:ready=c.join.add_pose(second[2])
    assert ready==[]  # the existing recorder remains ordered
    assert [r.frame_id for r in seen]==[2]
    assert seen[0].world_hands['status']==seen[0].segment_hands['status']=='available'
    assert c.join.add_pose(second[2])==[] and len(seen)==1
    seen[0].hands[0].joints[0].xyz_m[:]=99
    c.tick[0]=2.1
    ready=c.join.drain()
    assert [r.frame_id for r,_ in ready]==[1,2]
    assert ready[0][0].world_hands['status']=='unavailable'
    np.testing.assert_array_equal(ready[1][0].hands[0].joints[0].xyz_m,[.1,.2,1.])


@pytest.mark.parametrize('fault',['expired','future','different_session','different_segment','lost'])
def test_live_world_overlay_expires_and_never_uses_wrong_reference(live_context,monkeypatch,fault):
    c=live_context;frame,result,pose=c.sample()
    c.join.offer(result,frame,calibration=c.calibration)
    [(saved,_)]=c.join.add_pose(pose)
    spec=importlib.util.spec_from_file_location('isolated_delivery_age_guard',APP/'userrr/client_gui.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    display=SimpleNamespace(_live_hand_result=saved,_last_vio_message=copy.deepcopy(pose),_live_hand_age_ms=None,
        session=SimpleNamespace(worker=SimpleNamespace(live_vio=SimpleNamespace(source_session_id='e'*32))))
    tick=[result.timestamp_ns+100_000_000]
    monkeypatch.setattr(module.time,'perf_counter_ns',lambda:tick[0])
    assert module.ClientMainWindow._live_hand_transform(display) is not None
    assert display._live_hand_age_ms==100
    if fault=='expired':tick[0]=result.timestamp_ns+251_000_000
    elif fault=='future':tick[0]=result.timestamp_ns-1
    elif fault=='different_session':display.session.worker.live_vio.source_session_id='f'*32
    elif fault=='different_segment':display._last_vio_message['local_motion']['segment_id']='other'
    else:display._last_vio_message['pose_valid']=False
    assert module.ClientMainWindow._live_hand_transform(display) is None
    assert display._live_hand_age_ms is None


def test_paused_gui_does_not_replace_displayed_hands():
    spec=importlib.util.spec_from_file_location('paused_delivery',APP/'userrr/client_gui.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    previous=object();display=SimpleNamespace(paused=True,_live_hand_result=previous)
    module.ClientMainWindow.on_localized_hands(display,object())
    assert display._live_hand_result is previous
