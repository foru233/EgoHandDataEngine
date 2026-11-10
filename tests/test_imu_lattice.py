import struct

import pytest

from egohand.vio.imu_lattice import LatticeImuBuffer, PacketLatticeImuBuffer, packet_lattice_phase_ns


PERIOD_NS = 1_000_000_000 // 480  # 2_083_333
START_NS = 1_000_000_000  # 设备时间戳不会为 0（解析器会拒绝）


def payload(start_ns, count, period_ns=PERIOD_NS):
    rows = []
    for i in range(count):
        t = start_ns + i * period_ns
        rows.append(struct.pack('<QQ6f', t, t + 4_146_780, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0))
    return b''.join(rows)


def test_lattice_consumes_expected_count_and_stays_inside_interval():
    buffer = LatticeImuBuffer(sample_rate_hz=480.0, camera_to_imu_drift_ppm=0.0)
    buffer.add(payload(START_NS, 480))
    start, end = START_NS, START_NS + 200_000_000
    rows = buffer.consume(start, end)  # 200 ms -> 96 samples
    assert len(rows) == 96
    stamps = [r[0] for r in rows]
    assert all(start < s < end for s in stamps)
    assert stamps == sorted(stamps)
    assert buffer.pending() == 384


def test_lattice_rate_correction_uses_drift_ppm():
    buffer = LatticeImuBuffer(sample_rate_hz=480.0, camera_to_imu_drift_ppm=657.0)
    # 10 s 内应消费 round(480/1.000657*10) = 4796 条，而不是 4800
    assert buffer.expected(START_NS, START_NS + 10_000_000_000) == 4796


def test_lattice_reports_insufficient_samples_instead_of_guessing():
    buffer = LatticeImuBuffer(sample_rate_hz=480.0, camera_to_imu_drift_ppm=0.0)
    buffer.add(payload(START_NS, 10))
    assert buffer.consume(START_NS, START_NS + 100_000_000) is None
    assert buffer.pending() == 10


def test_lattice_skip_span_drops_gap_samples():
    buffer = LatticeImuBuffer(sample_rate_hz=480.0, camera_to_imu_drift_ppm=0.0)
    buffer.add(payload(START_NS, 480))
    dropped = buffer.skip_span(100_000_000)
    assert dropped == 48
    assert buffer.pending() == 432


def test_lattice_trim_before_removes_old_samples():
    buffer = LatticeImuBuffer(sample_rate_hz=480.0, camera_to_imu_drift_ppm=0.0)
    buffer.add(payload(START_NS, 480))
    buffer.trim_before(START_NS + 48 * PERIOD_NS)
    assert buffer.pending() == 432


def test_lattice_detects_large_driver_gap():
    buffer = LatticeImuBuffer(sample_rate_hz=480.0, camera_to_imu_drift_ppm=0.0)
    buffer.add(payload(START_NS, 10))
    buffer.add(payload(START_NS + 10 * PERIOD_NS + 400_000_000, 10))
    assert buffer.gap_events == 1


def test_lattice_rejects_non_monotonic_payload():
    buffer = LatticeImuBuffer(sample_rate_hz=480.0, camera_to_imu_drift_ppm=0.0)
    buffer.add(payload(START_NS, 10))
    with pytest.raises(ValueError):
        buffer.add(payload(START_NS, 10))


def test_packet_lattice_uses_packet_header_despite_seconds_of_driver_offset():
    buffer = PacketLatticeImuBuffer(sample_rate_hz=480, camera_to_imu_drift_ppm=0,
                                    phase_ns=packet_lattice_phase_ns({'lattice_phase_ms': 97}))
    header = 6_000_000
    start = header * 1000 + 97_000_000
    buffer.add(payload(START_NS, 16), header)
    buffer.add(payload(START_NS + 33_200_000, 16), header + 33_333)
    first = buffer.interval(start, start + 33_333_000)
    second = buffer.interval(start + 33_333_000, start + 66_667_000)
    assert len(first) == len(second) == 16
    assert first[0][0] == start + 1_041_667
    assert second[0][0] - first[0][0] == 33_333_000


def test_packet_lattice_short_packet_does_not_shift_next_packet_or_fill_gap():
    buffer = PacketLatticeImuBuffer(sample_rate_hz=480, camera_to_imu_drift_ppm=0, phase_ns=0)
    buffer.add(payload(START_NS, 15), START_NS // 1000)
    buffer.add(payload(START_NS + 33_333_000, 16), START_NS // 1000 + 33_333)
    assert len(buffer.interval(START_NS, START_NS + 33_333_000)) == 15
    second = buffer.interval(START_NS + 33_333_000, START_NS + 66_667_000)
    assert len(second) == 16 and second[0][0] == START_NS + 34_374_667
    assert buffer.gap_events == 1


def test_packet_lattice_reports_missing_packet_instead_of_bridging_the_gap():
    buffer = PacketLatticeImuBuffer(sample_rate_hz=480, camera_to_imu_drift_ppm=0, phase_ns=0)
    buffer.add(payload(START_NS, 16), START_NS // 1000)
    buffer.add(payload(START_NS + 66_667_000, 16), START_NS // 1000 + 66_667)
    with pytest.raises(ValueError, match='采样缺口'):
        buffer.interval(START_NS, START_NS + 100_000_000)
