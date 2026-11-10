import struct
import pytest

from egohand.sources.vendor_imu import (
    RECORD, GYRO_SCALE, ACCEL_SCALE, VendorIMUSource
)


def make_record(x, y, z, timestamp):
    # IIO scan layout: three int16 values, 2-byte alignment padding, int64 timestamp.
    return struct.pack("<hhh2xq", x, y, z, timestamp)


def test_iio_record_layout_and_si_scaling():
    assert RECORD.size == 16
    source = VendorIMUSource()
    gyro = source._parse_record("gyro", make_record(100, -200, 300, 1_000_000))
    accel = source._parse_record("accel", make_record(100, -200, 300, 1_000_000))
    assert gyro.timestamp_ns == accel.timestamp_ns == 1_000_000
    assert gyro.xyz_si == pytest.approx((100*GYRO_SCALE, -200*GYRO_SCALE, 300*GYRO_SCALE))
    assert accel.xyz_si == pytest.approx((100*ACCEL_SCALE, -200*ACCEL_SCALE, 300*ACCEL_SCALE))
    assert gyro.clock_domain == "vs030_iio_realtime_unverified"


def test_iio_non_monotonic_timestamp_is_rejected_per_sensor():
    source = VendorIMUSource()
    assert source._parse_record("gyro", make_record(0, 0, 0, 100)) is not None
    assert source._parse_record("gyro", make_record(0, 0, 0, 99)) is None
    assert source.stats["non_monotonic"] == 1
    assert source._parse_record("accel", make_record(0, 0, 0, 99)) is not None


def test_iio_invalid_record_is_rejected():
    source = VendorIMUSource()
    with pytest.raises(ValueError):
        source._parse_record("gyro", b"short")
    with pytest.raises(ValueError):
        source._parse_record("mag", make_record(0, 0, 0, 100))

def test_configure_rejects_unknown_scan_layout_before_changing_device(monkeypatch):
    from types import SimpleNamespace
    from egohand.sources import vendor_imu
    monkeypatch.setattr(vendor_imu.ADB.__class__,'is_file',lambda self:True)
    source=VendorIMUSource();calls=[]
    def adb(*args,**kwargs):
        calls.append(args)
        return SimpleNamespace(stdout=b'0.001\nrealtime\n0\nle:s32/32>>0\n1\nle:s16/16>>0\n2\nle:s16/16>>0\n3\nle:s64/64>>0\n')
    monkeypatch.setattr(source,'_adb',adb)
    with pytest.raises(RuntimeError,match='scan layout'):source._configure()
    assert len(calls)==1 and source.restore_script is None
