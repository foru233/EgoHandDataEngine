"""Offline product routing must negotiate a single owner and keep parent defaults."""
import os
from types import SimpleNamespace

import pytest

from egohand.vio import replay_runtime as runtime
from egohand.vio.recovery_contract import RecoveryProtocolError


@pytest.mark.parametrize('protocol,owner,accepted',[(2,'RecoveryCoordinator',True),(0,None,False),(2,'other',False)])
def test_explicit_stereo_child_route_and_handshake(tmp_path,monkeypatch,protocol,owner,accepted):
    library=tmp_path/'isolated.dll';library.write_bytes(b'candidate')
    captured={}
    class Socket:
        def bind(self,*a):pass
        def listen(self,*a):pass
        def settimeout(self,*a):pass
        def setsockopt(self,*a):pass
        def getsockname(self):return ('127.0.0.1',12345)
        def accept(self):return Socket(),None
        def shutdown(self,*a):pass
        def close(self):pass
    def popen(*args,**kwargs):
        captured.update(kwargs)
        return SimpleNamespace(wait=lambda **kw:0)
    monkeypatch.setenv('EGOHAND_VIO_DISABLE_RECOVERY','1')
    monkeypatch.setattr(runtime.socket,'socket',Socket)
    monkeypatch.setattr(runtime.subprocess,'Popen',popen)
    bridge=runtime.ReplayBridge(tmp_path/'worker.log',mode='stereo',library_path=library,coordinated_stereo=True)
    monkeypatch.setattr(bridge,'_read_message',lambda:{'type':'ready','recovery_protocol':protocol,'recovery_owner':owner})
    try:
        if accepted:
            bridge.start();assert bridge.recovery_protocol==2
        else:
            with pytest.raises(RecoveryProtocolError):bridge.start()
        assert captured['env']['EGOHAND_VIO_DISABLE_RECOVERY']=='0'
        assert captured['env']['EGOHAND_VIO_LIBRARY']==str(library.resolve())
        assert os.environ['EGOHAND_VIO_DISABLE_RECOVERY']=='1'
    finally:bridge.close()


def test_explicit_stereo_cannot_reconfigure_shared_worker(tmp_path):
    with pytest.raises(ValueError,match='独立离线Stereo'):
        runtime.ReplayBridge(tmp_path/'log',mode='stereo',worker=object(),coordinated_stereo=True)
    with pytest.raises(ValueError,match='独立离线Stereo'):
        runtime.ReplayBridge(tmp_path/'log',mode='stereo_inertial',coordinated_stereo=True)
