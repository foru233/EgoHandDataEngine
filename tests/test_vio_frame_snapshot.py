"""Frame snapshots must not be assembled from asynchronous body getters."""
import ctypes
import json
import socket
import struct
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from egohand.vio.native_bridge import NativeVioBridge, _CFrameSnapshot
from egohand.vio.live_subprocess import HEADER, serve


def bridge_with_snapshot(*, initialized=True, body_valid=True, consistent=True):
    bridge = NativeVioBridge.__new__(NativeVioBridge)
    bridge._closed = False
    bridge._handle = 1
    def fill(handle, pointer, size):
        value = ctypes.cast(pointer, ctypes.POINTER(_CFrameSnapshot)).contents
        value.schema_version = 1; value.struct_size = size
        value.snapshot_consistent = consistent; value.body_valid = body_valid
        value.frame_id = 7; value.timestamp_ns = 100; value.world_frame_id = 9
        value.imu_initialized = initialized; value.ba2_complete = initialized
        value.T_world_from_body[:] = np.eye(4).ravel()
        value.velocity_world_mps[:] = [1,2,3]
        value.accel_gyro_bias[:] = range(6)
        return 0
    bridge._dll = SimpleNamespace(egohand_vio_snapshot_size=lambda:ctypes.sizeof(_CFrameSnapshot),
                                 egohand_vio_get_frame_snapshot=fill)
    return bridge


def test_body_and_metadata_are_from_same_snapshot():
    value = bridge_with_snapshot().get_frame_snapshot()
    body = value['body_state']
    assert body['frame_id'] == value['frame_id'] == 7
    assert body['timestamp_ns'] == value['timestamp_ns'] == 100
    assert body['world_frame_id'] == value['world_frame_id'] == 9
    np.testing.assert_array_equal(body['gyro_bias_rps'], [3,4,5])


def test_no_init_frame_snapshot_has_metadata_but_no_body():
    value = bridge_with_snapshot(initialized=False, body_valid=False).get_frame_snapshot()
    assert value['snapshot_available'] and value['snapshot_consistent']
    assert not value['ba2_complete'] and value['body_state'] is None


def test_inconsistent_snapshot_never_supplies_candidate_body():
    value = bridge_with_snapshot(consistent=False).get_frame_snapshot()
    assert not value['snapshot_consistent'] and value['body_state'] is None


def test_extension_abi_mismatch_rejected_before_read():
    bridge = bridge_with_snapshot()
    bridge._dll.egohand_vio_snapshot_size = lambda:1
    with pytest.raises(RuntimeError, match='ABI'):
        bridge.get_frame_snapshot()


def test_legacy_library_consistency_remains_unknown():
    bridge = bridge_with_snapshot(); del bridge._dll.egohand_vio_get_frame_snapshot
    assert bridge.get_frame_snapshot() is None


def test_serve_uses_cached_body_without_second_native_read():
    class Bridge:
        map_mode = None
        def get_candidate_body_state(self):
            raise AssertionError('Asynchronous getter must not be used')
        def track_stereo(self, left, right, stamp, imu):
            snapshot = bridge_with_snapshot().get_frame_snapshot()
            snapshot['timestamp_ns'] = stamp
            return SimpleNamespace(tracking_state_name='OK', pose_valid=True, imu_initialized=True,
                inertial_scale_finalized=True, map_epoch=9,
                T_world_from_rectified_left_camera=np.eye(4), frame_snapshot=snapshot)
    a,b = socket.socketpair(); b.settimeout(3)
    failures = []
    def run():
        try:serve(a,Bridge(),recovery_enabled=False)
        except Exception as exc:failures.append(exc)
    thread = threading.Thread(target=run); thread.start()
    def read(n):
        data=b''
        while len(data)<n:data+=b.recv(n-len(data))
        return data
    def message():return json.loads(read(struct.unpack('<I',read(4))[0]))
    try:
        assert message()['type']=='ready'
        b.sendall(HEADER.pack(100,2,2,0)+bytes(8)); result=message()
        assert result['snapshot_consistent'] and result['frame_id']==7
        assert result['body_state_candidate']['frame_id']==7
        assert result['body_state_candidate']['velocity_world_mps']==[1,2,3]
    finally:
        b.close(); thread.join(timeout=3); a.close()
    assert not thread.is_alive() and not failures
