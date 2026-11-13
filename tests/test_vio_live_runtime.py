import struct
from io import StringIO
import pytest
from egohand.vio.live_runtime import CandidateImuBuffer, LiveVio
from egohand.vio.native_bridge import NativeVioBridge


def test_live_queue_measures_frame_residence_instead_of_idle_consumer_wait(monkeypatch):
    from egohand.vio import live_runtime
    tick=[0]
    monkeypatch.setattr(live_runtime.time,'perf_counter_ns',lambda:tick[0])
    vio=LiveVio(object(),lambda _:None)
    frame=object();get=vio.frames.get
    def arrive_after_idle(*,timeout):
        assert timeout==.1
        tick[0]=9_000_000_000;vio.submit(frame)  # Consumer waited nine seconds for an input.
        tick[0]=9_025_000_000
        return get(timeout=0)
    monkeypatch.setattr(vio.frames,'get',arrive_after_idle)
    received,submitted,dequeued=vio._take_frame(timeout=.1)
    assert received is frame and dequeued-submitted==25_000_000
    assert vio.frame_queue_stats()==dict(submitted=1,deferred=0,replaced=0,dequeued=1,restart_discarded=0)


def test_live_latest_queue_reports_replacements_and_deferred_frames():
    vio=LiveVio(object(),lambda _:None)
    vio.defer_imu=True;vio.submit('before-ready')
    assert vio.frames.empty() and vio.frame_queue_stats()['deferred']==1
    vio.ready=True
    vio.submit('old');vio.submit('new')
    frame,_,_=vio._take_frame(timeout=0)
    assert frame=='new'
    counters=vio.frame_queue_stats()
    assert counters==dict(submitted=2,deferred=1,replaced=1,dequeued=1,restart_discarded=0)
    counters['submitted']=1000
    assert vio.frame_queue_stats()['submitted']==2


def test_live_latest_queue_keeps_counter_conservation_under_concurrent_producers():
    import threading
    vio=LiveVio(object(),lambda _:None)
    threads=[threading.Thread(target=lambda side=side:[vio.submit((side,i)) for i in range(100)])
             for side in range(3)]
    for thread in threads:thread.start()
    for thread in threads:thread.join()
    frame,_,_=vio._take_frame(timeout=0)
    assert frame[1]==99 and vio.frames.empty()
    counts=vio.frame_queue_stats()
    assert counts['submitted']==300 and counts['replaced']==299 and counts['dequeued']==1


def test_candidate_imu_interpolates_and_rejects_gaps():
    b=CandidateImuBuffer()
    payload=b''.join(struct.pack('<QQ6f',t,t,1,2,3,t/1e9,0,0) for t in (1_000_000,3_000_000,5_000_000,7_000_000))
    b.add(payload)
    rows=b.interval(1_000_000,6_000_000)
    assert [r[0] for r in rows]==[3_000_000,5_000_000]
    assert rows[0][4]==pytest.approx(.003)
    with pytest.raises(ValueError):b.interval(7_000_000,100_000_000)
    with pytest.raises(ValueError):b.add(payload)


def test_imu_timestamp_shift_direction():
    b=CandidateImuBuffer(camera_to_imu_lag_ns=-85_000_000)
    b.add(struct.pack('<QQ6f',1_000_000_000,1_002_000_000,1,2,3,4,5,6))
    assert b.last_a==1_085_000_000 and b.last_b==1_087_000_000



def test_imu_raw_archive_header_is_versioned_and_preserves_sdk_times():
    from egohand.vio.live_runtime import IMU_RAW_HEADER
    header = IMU_RAW_HEADER.pack(b"IMUR", 1, IMU_RAW_HEADER.size,
                                 111, 222, 333, 40)
    assert IMU_RAW_HEADER.size == 36
    assert IMU_RAW_HEADER.unpack(header) == (b"IMUR", 1, 36, 111, 222, 333, 40)


def test_live_vio_wait_ready_requires_successful_child_handshake():
    vio = LiveVio(object(), lambda _message: None)
    assert not vio.wait_ready(timeout=0)
    assert "初始化超时" in vio.startup_error
    vio.ready = True
    vio.thread = type('AliveThread', (), {'is_alive': lambda self: True})()
    vio.ready_event.set()
    assert vio.wait_ready(timeout=0)


def test_live_vio_failed_startup_unblocks_waiter_without_claiming_ready():
    vio = LiveVio(object(), lambda _message: None)
    vio.startup_error = "missing native bridge"
    vio.ready_event.set()
    assert not vio.wait_ready(timeout=0)
    assert vio.startup_error == "missing native bridge"


def test_live_vio_worker_crash_starts_new_bounded_session(monkeypatch):
    messages=[]
    class Source:
        calls=0
        def enable_imu_capture(self, *, allow_running=False):
            assert allow_running
            self.calls+=1
    source=Source()
    vio=LiveVio(source,messages.append)
    outcomes=iter([True,False])
    monkeypatch.setattr(vio,'_run_session',lambda:next(outcomes))
    monkeypatch.setattr(vio.stop_event,'wait',lambda _:False)
    vio.frames.put('old frame')
    vio._run()
    assert source.calls==1 and vio.frames.empty()
    assert vio.frame_queue_stats()['restart_discarded']==1
    assert [m['type'] for m in messages]==['vio_reset']
    assert '新建地图' in messages[0]['message']


def test_live_vio_worker_restarts_are_bounded(monkeypatch):
    messages=[]
    class Source:
        def enable_imu_capture(self, *, allow_running=False):pass
    vio=LiveVio(Source(),messages.append)
    monkeypatch.setattr(vio,'_run_session',lambda:True)
    monkeypatch.setattr(vio.stop_event,'wait',lambda _:False)
    vio._run()
    assert [m['type'] for m in messages]==['vio_reset','vio_reset','error']


def test_native_vio_cleanup_preserves_tracking_error(monkeypatch):
    bridge = NativeVioBridge.__new__(NativeVioBridge)

    def fail_cleanup():
        raise OSError("shutdown error")

    monkeypatch.setattr(bridge, "close", fail_cleanup)

    with pytest.raises(OSError, match="access violation") as error:
        with bridge:
            raise OSError("access violation")
    assert error.value.__notes__ == ["VIO cleanup also failed: shutdown error"]

    with pytest.raises(OSError, match="shutdown error"):
        with bridge:
            pass


def test_live_subprocess_reuses_one_bridge_across_sessions(monkeypatch):
    from contextlib import nullcontext
    from egohand.vio import live_subprocess

    events = []

    class Bridge:
        def reset(self):
            events.append("reset")

    def open_bridge():
        events.append("load")
        return nullcontext(Bridge())

    def serve(port, bridge):
        events.append(port)
        if port == 1234:
            raise ConnectionAbortedError()

    monkeypatch.setattr(live_subprocess.sys, "stdin", StringIO('[1234,"first"]\n[2345,"second"]\n'))
    monkeypatch.setattr(live_subprocess, "connect", nullcontext)
    monkeypatch.setattr(live_subprocess, "open_bridge", open_bridge)
    monkeypatch.setattr(live_subprocess, "serve", serve)
    live_subprocess.serve_live_sessions()
    assert events == ["load", 1234, "reset", 2345]
def test_vio_worker_uses_its_own_conda_runtime(tmp_path, monkeypatch):
    from egohand.vio.live_runtime import vio_subprocess_environment
    prefix=tmp_path/'vio-env'
    (prefix/'conda-meta').mkdir(parents=True)
    executable=prefix/'python.exe'
    monkeypatch.setenv('CONDA_PREFIX','unrelated-parent-environment')
    env=vio_subprocess_environment(executable)
    assert env['CONDA_PREFIX']==str(prefix)
    assert env['PATH'].split(__import__('os').pathsep)[0]==str(prefix)
    assert __import__('os').environ['CONDA_PREFIX']=='unrelated-parent-environment'


def test_vio_environment_identity_tracks_recovery_policy_content(tmp_path):
    from egohand.vio.live_runtime import _environment_identity

    recovery = tmp_path / 'recovery.yaml'
    recovery.write_text('short_recovery: true\n', encoding='utf8')
    env = {
        'EGOHAND_VIO_CONFIG': str(tmp_path / 'settings.yaml'),
        'EGOHAND_VIO_RECOVERY_CONFIG': str(recovery),
        'EGOHAND_VIO_RECOVERY_PROTOCOL': '2',
    }
    first = _environment_identity(env)
    recovery.write_text('short_recovery: false\n', encoding='utf8')
    second = _environment_identity(env)
    assert first != second


def test_vio_environment_identity_includes_mode_policy():
    from egohand.vio.live_runtime import _environment_identity

    assert _environment_identity({'EGOHAND_VIO_DISABLE_RECOVERY': '0'}) != _environment_identity(
        {'EGOHAND_VIO_DISABLE_RECOVERY': '1'})


def test_live_worker_rebuilds_child_when_recovery_file_changes(tmp_path, monkeypatch):
    from egohand.vio import live_runtime

    settings = tmp_path / 'settings.yaml'
    settings.write_text('Camera.fps: 30\n', encoding='utf8')
    library = tmp_path / 'candidate.dll'
    library.write_bytes(b'candidate')
    recovery = tmp_path / 'recovery.yaml'
    recovery.write_text('short_recovery: true\n', encoding='utf8')
    monkeypatch.setattr(live_runtime, 'PYTHON', tmp_path / 'python.exe')
    monkeypatch.setattr(live_runtime, 'apply_mode_environment', lambda env, _mode: env)
    monkeypatch.setattr(live_runtime, 'vio_subprocess_environment', lambda **_kwargs: {
        'EGOHAND_VIO_RECOVERY_CONFIG': str(recovery),
        'EGOHAND_VIO_RECOVERY_PROTOCOL': '2',
    })
    from egohand.vio import library_manifest
    monkeypatch.setattr(library_manifest, 'select_library', lambda _root: library)

    class Stdin:
        def __init__(self): self.closed = False
        def write(self, _payload): return None
        def flush(self): return None
        def close(self): self.closed = True

    class Proc:
        def __init__(self): self.stdin = Stdin(); self.closed = False
        def poll(self): return None
        def wait(self, timeout=None): self.closed = True
        def kill(self): self.closed = True

    created = []
    monkeypatch.setattr(live_runtime.subprocess, 'Popen', lambda *args, **kwargs: created.append(Proc()) or created[-1])
    worker = live_runtime.LiveVioWorker()
    worker.request_session(1234, settings, 'first')
    assert len(created) == 1
    recovery.write_text('short_recovery: false\n', encoding='utf8')
    _, reused, _ = worker.request_session(1234, settings, 'second')
    assert not reused and len(created) == 2 and created[0].closed
    worker.close()
