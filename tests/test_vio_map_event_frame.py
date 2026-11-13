import ctypes
from types import SimpleNamespace

import pytest

from egohand.vio.native_bridge import NativeVioBridge, _CMapEventFrame


def bridge_with_events(*, bound=True, stamp=123, count=1, coverage=1):
    bridge = NativeVioBridge.__new__(NativeVioBridge)
    bridge._closed = False; bridge._handle = 1
    def fill(handle, pointer, size):
        value = ctypes.cast(pointer, ctypes.POINTER(_CMapEventFrame)).contents
        value.schema_version=1; value.struct_size=size
        value.frame_id=7; value.map_id=2; value.reset_id=4; value.frame_timestamp_ns=stamp
        value.snapshot_consistent=1; value.frame_bound=bound; value.coverage=coverage
        value.event_count=count; value.overwritten_count=2
        event=value.events[0]; event.sequence=23; event.map_id=2
        event.type=1; event.scale=1; event.rotation[:]=[1,0,0,0,1,0,0,0,1]
        event.translation[:]=[0,0,0]; event.map_update_lock_matches_caller=1
        return 0
    bridge._dll=SimpleNamespace(egohand_vio_map_event_frame_size=lambda:ctypes.sizeof(_CMapEventFrame),
        egohand_vio_get_map_event_frame=fill)
    return bridge


def snapshot():
    return {'frame_id':7,'timestamp_ns':123,'map_id':2,'reset_id':4,'snapshot_consistent':True}


def test_partial_events_are_bound_but_never_prove_continuity():
    result=bridge_with_events().get_map_event_frame(snapshot(),123)
    assert result['frame_bound'] and result['coverage']=='partial'
    assert result['overwritten_count']==2 and result['events'][0]['sequence']==23
    assert not result['events'][0]['reset_generation_known']
    assert result['reset_generation_verified'] is False
    assert result['events'][0]['commit_timestamp_ns'] is None
    assert result['coordinate_continuity_verified'] is False


@pytest.mark.parametrize('changes', [{'frame_id':8},{'timestamp_ns':124},{'map_id':3},
                                     {'reset_id':5},{'snapshot_consistent':False}])
def test_mismatched_frame_never_exposes_events_as_bound(changes):
    snap=snapshot();snap.update(changes)
    result=bridge_with_events().get_map_event_frame(snap,123)
    assert result['coverage']=='unknown' and not result['frame_bound'] and result['events']==[]


def test_old_dll_has_no_fabricated_event_report():
    bridge=bridge_with_events();del bridge._dll.egohand_vio_get_map_event_frame
    assert bridge.get_map_event_frame(snapshot(),123) is None


def test_bad_abi_and_oversized_ring_are_rejected():
    bridge=bridge_with_events();bridge._dll.egohand_vio_map_event_frame_size=lambda:1
    with pytest.raises(RuntimeError, match='ABI'):bridge.get_map_event_frame(snapshot(),123)
    with pytest.raises(RuntimeError, match='数量'):bridge_with_events(count=17).get_map_event_frame(snapshot(),123)
