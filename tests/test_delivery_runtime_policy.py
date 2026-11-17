"""The selected delivery owns its child policy; shell diagnostics cannot leak."""
from types import SimpleNamespace


def test_explicit_live_profile_isolates_native_switches_and_protocol(tmp_path, monkeypatch):
    from egohand.vio import live_runtime as m
    settings=tmp_path/'settings.yaml';settings.write_text('Camera.fps: 30\n')
    library=tmp_path/'release.dll';library.write_bytes(b'release')
    monkeypatch.setattr(m,'vio_subprocess_environment',lambda **kw:{
        'PATH':'native-runtime','EGOHAND_VIO_DISABLE_RECOVERY':'1',
        'EGOHAND_VIO_EXACT_GRAPH_DIR':'stale','EGOHAND_TEST_FAULT':'1'})
    captured=[]
    class Proc:
        def __init__(self):
            self.stdin=SimpleNamespace(write=lambda b:None,flush=lambda:None,close=lambda:None)
        def poll(self):return None
        def wait(self,timeout=None):return 0
        def kill(self):return None
    monkeypatch.setattr(m.subprocess,'Popen',lambda *a,**kw:captured.append(kw['env']) or Proc())
    worker=m.LiveVioWorker()
    try:
        worker.request_session(1234,settings,'delivery',library_path=library,
            environment={'EGOHAND_VIO_DISABLE_RECOVERY':'0','EGOHAND_VIO_SHORT_RECOVERY':'1'})
        env=captured[-1]
        assert env['EGOHAND_VIO_DISABLE_RECOVERY']=='0'
        assert env['EGOHAND_VIO_RECOVERY_PROTOCOL']=='2'
        assert env['EGOHAND_VIO_STEREO_ONLY']=='0'
        assert env['PATH'].endswith('native-runtime')
        assert 'EGOHAND_VIO_EXACT_GRAPH_DIR' not in env and 'EGOHAND_TEST_FAULT' not in env
    finally:worker.close()
