import json
import struct
from types import SimpleNamespace

import numpy as np

from egohand.storage.recorder import (
    AsyncRecorder, SDK_RECORD_HEADER, SDK_RECORD_MAGIC, SDK_RECORD_VERSION,
)
from egohand.core.types import SourceMetadata


def test_sdk_records_preserve_ipc_v2_payload_and_all_clocks(tmp_path):
    path = tmp_path / "session"
    metadata = SourceMetadata("vendor_vs030", real_stereo=True, contains_imu=True)
    recorder = AsyncRecorder(path, metadata, max_queue=8)
    packet = (7, 12, 345, 111, 222, 678, 901, b"\x00\xff\x01")
    assert recorder.submit("sdk", packet)
    recorder.close(capture_stats={"sdk_packets_received": 1})

    manifest = json.loads((path / "manifest.json").read_text(encoding="utf8"))
    assert manifest["recording_status"] == "transport_unverified"
    assert manifest["sdk_count"] == 1
    assert manifest["sdk_count_by_sink"] == {"7": 1}
    assert manifest["capture_stats"]["sdk_packets_received"] == 1
    assert manifest["sdk_record_header_bytes"] == SDK_RECORD_HEADER.size == 64
    assert manifest["sdk_sha256"]
    assert manifest["imu_sha256"]

    raw = (path / "raw/sdk_records.bin").read_bytes()
    header = SDK_RECORD_HEADER.unpack(raw[:SDK_RECORD_HEADER.size])
    assert header == (
        SDK_RECORD_MAGIC, SDK_RECORD_VERSION, 64, 7, 3,
        12, 345, 111, 222, 678, 901,
    )
    assert raw[SDK_RECORD_HEADER.size:] == b"\x00\xff\x01"

    imu = (path / "raw/imu_records.bin").read_bytes()
    assert imu == raw
    index = json.loads((path / "raw/sdk_index.jsonl").read_text(encoding="utf8").strip())
    assert index["offset"] == 0
    assert index["sink"] == 7
    assert index["imu_copy_offset"] == 0
    assert index["sdk_capture_timestamp_ns"] == 111
    assert index["sdk_arrival_timestamp_ns"] == 222


def test_sdk_camera_and_imu_records_share_indexed_container(tmp_path):
    path = tmp_path / "session"
    recorder = AsyncRecorder(path, SourceMetadata("vendor_vs030"), max_queue=8)
    assert recorder.submit("sdk", (4, 1, 10, 11, 12, 13, 14, b"left"))
    assert recorder.submit("sdk", (5, 1, 10, 21, 22, 23, 24, b"right"))
    assert recorder.submit("sdk", (7, 2, 11, 31, 32, 33, 34, b"imu"))
    recorder.close()

    manifest = json.loads((path / "manifest.json").read_text(encoding="utf8"))
    assert manifest["sdk_count_by_sink"] == {"4": 1, "5": 1, "7": 1}
    assert manifest["sdk_stream_stats"]["4"]["payload_bytes"] == 4
    assert manifest["sdk_stream_stats"]["7"]["payload_bytes"] == 3

    records = (path / "raw/sdk_records.bin").read_bytes()
    offsets = []
    pos = 0
    while pos < len(records):
        values = SDK_RECORD_HEADER.unpack(records[pos:pos + SDK_RECORD_HEADER.size])
        size = values[4]
        offsets.append((values[3], pos, records[pos + SDK_RECORD_HEADER.size:pos + SDK_RECORD_HEADER.size + size]))
        pos += SDK_RECORD_HEADER.size + size
    assert [(sink, payload) for sink, _, payload in offsets] == [
        (4, b"left"), (5, b"right"), (7, b"imu")
    ]
    assert (path / "raw/imu_records.bin").stat().st_size == SDK_RECORD_HEADER.size + 3


def test_vs030_frame_index_excludes_raw_bytes_but_keeps_timing(tmp_path):
    path = tmp_path / "session"
    recorder = AsyncRecorder(path, SourceMetadata("vendor_vs030"), max_queue=8)
    frame = SimpleNamespace(
        frame_id=9,
        timestamp_ns=123456,
        source_type="vendor_vs030",
        calibration_id="calib",
        image_geometry="raw",
        left_image=np.zeros((2, 3, 3), dtype=np.uint8),
        right_image=np.zeros((2, 3, 3), dtype=np.uint8),
        metadata={
            "sdk_sequence": 99,
            "sdk_imu_packets_raw": ((1, 2, 3, b"raw imu bytes"),),
            "sdk_imu_frame_timing": ((1, 2, 3),),
            "sdk_capture_pair_delta_ns": 10,
        },
    )
    assert recorder.submit("frame_index", frame)
    recorder.close()
    row = json.loads((path / "raw/camera_frames.jsonl").read_text(encoding="utf8").strip())
    assert row["frame_id"] == 9
    assert row["metadata"]["sdk_imu_packet_count"] == 1
    assert row["metadata"]["sdk_imu_frame_timing"] == [[1, 2, 3]]
    assert "sdk_imu_packets_raw" not in row["metadata"]


def test_sdk_recording_failure_is_not_complete(tmp_path):
    path = tmp_path / "session"
    recorder = AsyncRecorder(path, SourceMetadata("vendor_vs030"))
    recorder.error = "simulated loss"
    recorder.close()
    manifest = json.loads((path / "manifest.json").read_text(encoding="utf8"))
    assert manifest["recording_status"] == "incomplete"
    assert manifest["incomplete_reason"] == "simulated loss"
