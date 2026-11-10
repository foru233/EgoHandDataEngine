import pytest
from egohand.vio.device_timing import DeviceTimeGate,SensorFrameTime,TimedImu
from egohand.vio.native_bridge import ImuMeasurement

def frame(t,clock='device',origin='device_exposure_midpoint',evidence='SDK'):
    return SensorFrameTime(t,t+1,clock,origin,evidence)

def imu(t,clock='device',origin='device_sensor_sample'):
    return TimedImu(ImuMeasurement(t,(0.,0.,9.8),(0.,0.,0.)),clock,origin,'SDK')

def test_host_timestamps_and_unknown_sources_cannot_feed_vio():
    gate=DeviceTimeGate(max_stereo_delta_ns=5,max_imu_gap_ns=10)
    with pytest.raises(ValueError,match='provenance'):
        gate.frame_interval(frame(100,origin='host_decode_complete'))
    with pytest.raises(ValueError,match='provenance'):
        gate.push_imu(imu(95,origin='host_received'))
    gate.push_imu(imu(95))
    with pytest.raises(ValueError,match='clock domain'):
        gate.frame_interval(frame(100,clock='host'))
    assert gate.last_frame is None and len(gate.samples)==1

def test_frame_intervals_keep_raw_time_and_reject_missing_samples():
    gate=DeviceTimeGate(max_stereo_delta_ns=5,max_imu_gap_ns=10)
    gate.push_imu(imu(95));first=gate.frame_interval(frame(100))
    assert first.frame_ns==100 and first.discarded_before_first_frame==1 and not first.imu_samples
    with pytest.raises(ValueError,match='no IMU'):
        gate.frame_interval(frame(120))
    gate.push_imu(imu(105));gate.push_imu(imu(115))
    interval=gate.frame_interval(frame(120))
    assert [s.timestamp_ns for s in interval.imu_samples]==[105,115]
    assert gate.last_frame==120
    with pytest.raises(ValueError,match='strictly increasing'):gate.frame_interval(frame(120))

def test_stereo_mismatch_gaps_and_queue_overflow_are_explicit():
    gate=DeviceTimeGate(max_stereo_delta_ns=5,max_imu_gap_ns=5,max_imu_buffer=2)
    with pytest.raises(ValueError,match='stereo exposure'):
        gate.frame_interval(SensorFrameTime(100,110,'device','device_exposure_start','SDK'))
    gate.frame_interval(frame(100));gate.push_imu(imu(105));gate.push_imu(imu(115))
    with pytest.raises(OverflowError):gate.push_imu(imu(116))
    with pytest.raises(ValueError,match='gap'):gate.frame_interval(frame(120))
    assert gate.last_frame==100 and len(gate.samples)==2
