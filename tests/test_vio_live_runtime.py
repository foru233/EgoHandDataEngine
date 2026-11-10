import struct
import pytest
from egohand.vio.live_runtime import CandidateImuBuffer, LiveVio


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
