"""Exercise actual negotiated worker framing, bytes, resize and cancellation."""
import json
import socket
import threading
from multiprocessing.shared_memory import SharedMemory
from types import SimpleNamespace
import numpy as np
import pytest
from egohand.vio import live_subprocess as child
from egohand.vio.replay_runtime import ReplayBridge
from egohand.vio.recovery_contract import PROTOCOL
from egohand.vio.stereo_transport import StereoPublisher, StereoReader, negotiate, CAPABILITY


def missing(name):
    try:
        memory=SharedMemory(name=name,create=False)
    except FileNotFoundError:return True
    memory.close();return False


@pytest.mark.parametrize('shared',[False,True])
@pytest.mark.parametrize('cancel_before_ack',[False,True])
def test_real_worker_receives_exact_stereo_and_closes_storage(tmp_path,monkeypatch,shared,cancel_before_ack):
    config=tmp_path/'recovery.yaml';config.write_text('timeout_action: report_only\n')
    monkeypatch.setattr(child,'RECOVERY_CONFIG',config)
    monkeypatch.delenv('EGOHAND_VIO_RECOVERY_CONFIG',raising=False)
    expected=[];seen=[];errors=[];names=[]
    class Bridge:
        map_mode=None
        callback=None
        def set_recovery_callback(self,value):self.callback=value
        def track_stereo(self,left,right,stamp,imu):
            a,b,t=expected[len(seen)]
            assert np.array_equal(left,a) and np.array_equal(right,b) and stamp==t
            assert left.flags.c_contiguous and right.flags.c_contiguous
            seen.append(stamp)
            return SimpleNamespace(tracking_state_name='NOT_INITIALIZED',pose_valid=False,
                imu_initialized=False,inertial_scale_finalized=False,map_epoch=0,
                frame_snapshot=dict(frame_id=stamp,world_frame_id=0,map_id=0,reset_id=0,map_revision=0,
                    snapshot_available=True,snapshot_consistent=True),T_world_from_rectified_left_camera=None)
    server,client=socket.socketpair();client.settimeout(3);bridge=Bridge()
    def serve():
        try:child.serve(server,bridge,recovery_protocol=PROTOCOL,recovery_enabled=True)
        except EOFError:
            if not cancel_before_ack:errors.append('unexpected EOF')
        except Exception as exc:errors.append(repr(exc))
        finally:server.close()
    thread=threading.Thread(target=serve);thread.start()
    host=ReplayBridge(tmp_path/'unused.log');host.sock=client;host.recovery_protocol=PROTOCOL
    try:
        ready=host._read_message()
        assert CAPABILITY in ready['stereo_transports']
        host.shared_transport=negotiate(ready,PROTOCOL,requested=shared)
        for i,(h,w) in enumerate(((4,8),(7,5),(7,5))):
            a=np.arange(h*w,dtype=np.uint8).reshape(h,w)[::-1]
            if not shared:a=np.ascontiguousarray(a)
            b=(255-a).copy();stamp=1_000_000_000+i*33_333_333
            expected.append((a.copy(),b.copy(),stamp))
            row=host.track(stamp,a,b,[],reset_imu_alignment=i==1)
            assert row['image_transport']==('shared_memory_v1' if shared else 'tcp')
            if shared:names.append(host.shared_transport.memory.name)
            if cancel_before_ack:break
            row['quality_gate']={'passed':False};row['pose_valid']=False
            committed=host.commit_recovery(row)
            assert committed['stage']=='committed' and not committed['application_pose_admitted']
    finally:
        host.close();thread.join(timeout=3)
    assert not thread.is_alive() and not errors and bridge.callback is None
    assert len(seen)==(1 if cancel_before_ack else 3)
    assert all(missing(name) for name in names)


def test_legacy_and_unrequested_peers_keep_tcp():
    assert negotiate({},2,requested=True) is None
    assert negotiate({'stereo_transports':[CAPABILITY]},1,requested=True) is None
    assert negotiate({'stereo_transports':[CAPABILITY]},2,requested=False) is None


@pytest.mark.parametrize('change',[
    {'shape':[3,4]}, {'nbytes':1}, {'dtype':'float32'}, {'version':2}, {'name':''}])
def test_descriptor_mismatch_never_reads_a_different_frame(change):
    producer=StereoPublisher();reader=StereoReader()
    try:
        descriptor=json.loads(producer.publish(np.zeros((4,3),np.uint8),np.ones((4,3),np.uint8)))
        descriptor.update(change)
        with pytest.raises(ValueError):reader.read(json.dumps(descriptor).encode(),3,4)
    finally:reader.close();producer.close()
