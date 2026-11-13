import json
import socket
import struct
import threading
from types import SimpleNamespace

import numpy as np

from egohand.vio import live_subprocess


def test_recovery_opt_out_is_explicit(monkeypatch):
    monkeypatch.delenv('EGOHAND_VIO_DISABLE_RECOVERY', raising=False)
    assert live_subprocess.recovery_policy_enabled()
    monkeypatch.setenv('EGOHAND_VIO_DISABLE_RECOVERY', '1')
    assert not live_subprocess.recovery_policy_enabled()


def test_library_override_reaches_native_constructor(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(live_subprocess, 'NativeVioBridge', lambda *a, **kw: calls.append((a, kw)))
    path = tmp_path / 'pinned.dll'
    monkeypatch.setenv('EGOHAND_VIO_LIBRARY', str(path))
    live_subprocess.open_bridge()
    assert calls[0][1]['library_path'] == path.resolve()


def test_disabled_policy_does_not_read_config_or_register_callback(monkeypatch, tmp_path):
    monkeypatch.setattr(live_subprocess, 'RECOVERY_CONFIG', tmp_path / 'nonexistent.yaml')
    class Bridge:
        map_mode = None
        def track_stereo(self, left, right, stamp, imu):
            return SimpleNamespace(tracking_state_name='OK', pose_valid=True, imu_initialized=False,
                inertial_scale_finalized=False, map_epoch=1, T_world_from_rectified_left_camera=np.eye(4))
        def set_recovery_callback(self, callback):
            raise AssertionError('Recovery callback must not be installed or cleared in disabled mode')
    a, b = socket.socketpair()
    b.settimeout(3)
    failures = []
    def run():
        try:
            live_subprocess.serve(a, Bridge(), recovery_enabled=False)
        except Exception as exc:
            failures.append(exc)
    thread = threading.Thread(target=run)
    thread.start()
    def read(n):
        data = b''
        while len(data) < n:
            data += b.recv(n-len(data))
        return data
    def message():
        return json.loads(read(struct.unpack('<I', read(4))[0]))
    try:
        assert message()['type'] == 'ready'
        b.sendall(live_subprocess.HEADER.pack(1, 2, 2, 0) + bytes(8))
        result = message()
        assert result['pose_valid']
        assert 'recovery_preaccept' not in result and 'automatic_new_map' not in result
    finally:
        b.close(); thread.join(timeout=3); a.close()
    assert not thread.is_alive() and not failures
