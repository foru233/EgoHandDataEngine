"""VS030 分包及安全门控：模拟包不伪造真实相机同步。"""
import struct
import threading
import time
import numpy as np
import pytest

pytest.importorskip('av')
from egohand.sources.vendor_placeholder import MAGIC,VendorBundleDecoder,VendorStereoSource
from egohand.core.pipeline import EgoHandEngine
from egohand.core.types import StereoFrame

class FakeFrame:
    def __init__(self,value):self.value=value
    def to_ndarray(self,format):return np.full((3,4,3),self.value,dtype=np.uint8)

class FakeDecoder:
    def __init__(self,value):self.value=value
    def decode(self,packet):return [FakeFrame(self.value)]

def start(size):return MAGIC+struct.pack('<II',1,size)+bytes(36)
def channel(number,size):return MAGIC+struct.pack('<III',4,number,size)
def end():return MAGIC+struct.pack('<I',2)
def decoder():return VendorBundleDecoder({4:FakeDecoder(4),5:FakeDecoder(5)})

def test_complete_bundle_preserves_channels_and_packet_fragments():
    d=decoder();assert d.feed(start(7)) is None
    assert d.feed(channel(4,4)) is None
    assert d.feed(b'ab') is None and d.feed(b'cd') is None
    assert d.feed(channel(5,3)) is None and d.feed(b'xyz') is None
    a,b=d.feed(end());assert np.all(a==4) and np.all(b==5)

def test_incomplete_duplicate_or_wrong_length_bundle_not_emitted():
    d=decoder();d.feed(start(7));d.feed(channel(4,4));d.feed(b'ab');assert d.feed(end()) is None
    d.feed(start(8));d.feed(channel(4,4));d.feed(b'abcd');d.feed(channel(4,4));d.feed(b'efgh');assert d.feed(end()) is None
    d.feed(start(9));d.feed(channel(4,4));d.feed(b'abcd');d.feed(channel(5,3));d.feed(b'xyz');assert d.feed(end()) is None

def test_camera_metadata_and_shutdown_do_not_claim_sensor_sync():
    s=VendorStereoSource();meta=s.get_metadata()
    assert meta.real_stereo and not meta.synchronized and not meta.metric_capable
    assert meta.extra['channels']==[4,5]
    assert meta.extra['left_right_mapping_evidence']=='user_confirmed_matches_visionviewer'
    s.running=True;result=[]
    thread=threading.Thread(target=lambda:result.append(s.read()));thread.start();time.sleep(.05);s.interrupt_read();thread.join(1)
    assert not thread.is_alive() and result==[None]

def test_unsynchronized_camera_never_publishes_3d(engine,source):
    from dataclasses import replace
    engine.start(source);image=np.zeros((480,640,3),np.uint8)
    # 仅供门控测试的虚构标定，不能用于真实设备。
    engine.set_calibration(replace(engine.calibration,synthetic=False,calibration_source='test fixture'))
    frame=StereoFrame(0,1,image,image,None,None,engine.calibration.calibration_id,'vendor_vs030',False,'raw',
                      {'real_stereo':True,'synthetic':False,'timestamp_kind':'host_bundle_decode_completed_monotonic_ns'})
    result=engine.process(frame)
    assert result.diagnostics['reason']=='Synchronization Unverified'
    assert result.reconstruction_status=='2D Only' and not any(j.valid for h in result.hands for j in h.joints)

def test_camera_detector_requires_rectification():
    from dataclasses import replace
    from egohand.calibration.schema import synthetic_calibration
    class Detector:
        def __init__(self):self.images=[]
        def detect(self,image,timestamp):self.images.append(image.shape);return []
    detectors=[Detector(),Detector()];engine=EgoHandEngine(detectors=detectors)
    image=np.zeros((480,640,3),np.uint8)
    frame=StereoFrame(0,1,image,image,None,None,None,'vendor_vs030',False,'raw',{'real_stereo':True,'synthetic':False})
    result=engine.process(frame)
    assert not any(d.images for d in detectors)
    assert not result.diagnostics['detector_rectification_ready']
    # 虚构标定只验证流程；传感器同步仍未知，因此即使二维可运行也不能输出三维。
    engine.set_calibration(replace(synthetic_calibration(),synthetic=False,calibration_source='test fixture'))
    result=engine.process(replace(frame,timestamp_ns=2,calibration_id=engine.calibration.calibration_id))
    assert all(len(d.images)==1 for d in detectors)
    assert result.diagnostics['detector_rectification_ready']
    assert result.diagnostics['reason']=='Synchronization Unverified'
    assert result.reconstruction_status=='2D Only'

def test_calibrated_camera_preview_rejects_unrectified_error_frame():
    from types import SimpleNamespace
    from egohand.gui.workers import _publish_frame
    raw=SimpleNamespace(image_geometry='raw')
    rectified=SimpleNamespace(image_geometry='rectified')
    assert not _publish_frame('vendor_vs030',object(),raw)
    assert _publish_frame('vendor_vs030',object(),rectified)
    # 没有标定时仍允许清楚标识的原图预览，其他数据源保持原行为。
    assert _publish_frame('vendor_vs030',None,raw)
    assert _publish_frame('show3d',object(),raw)

def test_camera_control_protocol_and_response_validation():
    class FakeSocket:
        def __init__(self,replies):self.replies=list(replies);self.sent=[]
        def send(self,value):self.sent.append(value)
        def recv(self,size):return self.replies.pop(0)
    s=VendorStereoSource()
    reply=lambda cmd,status,payload:struct.pack('<8I',0x12345678,0xcafebabe,1,1,status,cmd,len(payload),0)+payload
    s.control=FakeSocket([reply(5,0,b'background heartbeat'),reply(7,0,b'ok')])
    assert s._command(6,b'1234')==b'ok'
    assert s.control.sent==[struct.pack('<8I',0x12345678,0xcafebabe,0,0,0,6,4,0)+b'1234']
    s.control=FakeSocket([reply(7,1,b'')])
    with pytest.raises(RuntimeError,match='返回错误'):s._command(6)

@pytest.mark.parametrize('pairing', ['same_udp_bundle_not_sensor_sync', 'sdk_sequence_and_equal_device_timestamp'])
def test_user_confirmed_bundle_reconstructs_without_inventing_timestamps(engine,source,pairing):
    from dataclasses import replace
    engine.set_calibration(replace(engine.calibration,synthetic=False,calibration_source='test fixture'))
    engine.config['vs030_user_confirmed_sync']=True
    f=source.read()
    f=replace(f,source_type='vendor_vs030',synchronized=False,left_timestamp_ns=None,right_timestamp_ns=None,
        metadata={'real_stereo':True,'synthetic':False,'transport_pairing':pairing},
        left_detections=[replace(d,origin='fixture') for d in f.left_detections],
        right_detections=[replace(d,origin='fixture') for d in f.right_detections])
    r=engine.process(f)
    assert r.reconstruction_status=='User-confirmed Stereo 3D'
    assert sum(h.valid_joint_count for h in r.hands)>0
    assert r.diagnostics['sync_quality']=='USER_CONFIRMED_SAME_BUNDLE'
    assert not r.diagnostics['hardware_sync_precision_verified'] and 'sync_error_ms' not in r.diagnostics
    engine.config['vs030_user_confirmed_sync']=False
    r=engine.process(replace(f,timestamp_ns=f.timestamp_ns+1))
    assert r.reconstruction_status=='2D Only' and r.diagnostics['reason']=='Synchronization Unverified'

def test_tracking_disabled_does_not_use_history_or_assign_persistent_ids(engine,source):
    engine.config['enable_3d_tracking']=False
    first=engine.process(source.read());second=engine.process(source.read())
    for result in (first,second):
        assert not result.diagnostics['tracking_3d_enabled']
        assert all(h.track_id==-1 and h.tracking_status=='frame_only' for h in result.hands if h.valid_joint_count)
        assert sum(h.valid_joint_count for h in result.hands)>0
    assert not engine.tracks.tracks
