"""sdk_recording 解析/解码/校验测试：用 PyAV libx265 合成 HEVC 包，不需要相机。

覆盖：
- IPC v2 记录读取、IMU <QQ6f 解析与平均速率；
- 双目按 sequence + 相等 device_ts 配对、跳过早到的单侧包、解码 0/2 帧跳过计数；
- sdk_index.jsonl 尾部截断容错；
- build_replay_check 的“可回放/不可回放”两种结论与 recorder.close() 自动集成；
- 视频导出脚本（左右 MP4 + CSV + export.json，拒绝覆盖）。
"""
import json
import math
import importlib.util
from fractions import Fraction
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest

from egohand.core.types import SourceMetadata
from egohand.storage import recorder as recorder_module
from egohand.storage.recorder import Recorder
from egohand.storage.sdk_recording import (
    IMU_PAYLOAD_ROW,
    IMU_PAYLOAD_ROW_BYTES,
    SDK_RECORD_HEADER,
    SDK_RECORD_MAGIC,
    SDK_RECORD_VERSION,
    RecordingFormatError,
    build_replay_check,
    iter_imu_rows,
    iter_index_rows,
    iter_paired_frames,
    iter_sdk_records,
)

ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 64, 48
IMU_STEP_NS = 2_083_333  # 约 480 Hz
IMU_ROWS_PER_PACKET = 16


def _metadata():
    return SourceMetadata("vendor_vs030", real_stereo=True, contains_imu=True)


@lru_cache(maxsize=1)
def _hevc_packets():
    """编码 8 帧全 IDR（重复参数集）H.265，每包可独立解码；返回与帧颜色一一对应的包列表。"""
    import av

    encoder = av.CodecContext.create("libx265", "w")
    encoder.width = WIDTH
    encoder.height = HEIGHT
    encoder.pix_fmt = "yuv420p"
    encoder.framerate = Fraction(30, 1)
    encoder.time_base = Fraction(1, 30)
    encoder.options = {
        "x265-params": "keyint=1:min-keyint=1:repeat-headers=1:log-level=none"
    }
    packets = []
    for index in range(8):
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        image[:, :] = _color(index)
        video_frame = av.VideoFrame.from_ndarray(image, format="bgr24")
        video_frame.pts = index
        video_frame.time_base = Fraction(1, 30)
        packets.extend(bytes(packet) for packet in encoder.encode(video_frame))
    assert len(packets) == 8, f"libx265 未按帧输出：{len(packets)}"
    return packets


def _color(index):
    return (index * 20, 40 + index, 120)


def _imu_payload(first_time_ns, rows=IMU_ROWS_PER_PACKET):
    chunks = []
    for index in range(rows):
        time_a = first_time_ns + index * IMU_STEP_NS
        chunks.append(IMU_PAYLOAD_ROW.pack(
            time_a, time_a + 12_345, 0.1, 0.2, 9.8, 0.01, 0.02, 0.03,
        ))
    return b"".join(chunks)


def _sdk_record_bytes(sink, sequence, device_ts, payload, capture_ts=0, arrival_ts=0):
    return SDK_RECORD_HEADER.pack(
        SDK_RECORD_MAGIC, SDK_RECORD_VERSION, SDK_RECORD_HEADER.size, sink,
        len(payload), sequence, device_ts, capture_ts, arrival_ts, 1, 2,
    ) + payload


def _build_recording(tmp_path, records, imu_payloads=(), name="session", close=True):
    """records: [(sink, sequence, device_ts, capture_ts, arrival_ts, payload), ...]。"""
    recorder = Recorder(tmp_path / name, _metadata())
    host = 1_000
    for record in records:
        sink, sequence, device_ts, capture_ts, arrival_ts, payload = record
        recorder.write_sdk((sink, sequence, device_ts, capture_ts, arrival_ts, host, host + 1, payload))
        host += 10
    for payload in imu_payloads:
        recorder.write_sdk((7, 0, 0, 0, 0, host, host + 1, payload))
        host += 10
    if close:
        recorder.close()
    return recorder.path


def _camera_pair(sequence, left_index, right_index, device_ts, capture_ts=0, arrival_ts=0):
    packets = _hevc_packets()
    return [
        (4, sequence, device_ts, capture_ts, arrival_ts, packets[left_index]),
        (5, sequence, device_ts, capture_ts + 1, arrival_ts + 1, packets[right_index]),
    ]


def _assert_color(frame, expected):
    mean = frame.mean(axis=(0, 1))
    assert np.allclose(mean, expected, atol=8), f"颜色不符：{mean} != {expected}"


# ---------------------------------------------------------------- 原始记录 / IMU

def test_iter_sdk_records_and_imu_rows_roundtrip(tmp_path):
    packets = _hevc_packets()
    records = [
        (4, 1, 100, 11, 12, packets[0]),
        (7, 2, 101, 13, 14, _imu_payload(1_000_000)),
        (5, 1, 100, 21, 22, packets[0]),
    ]
    root = _build_recording(tmp_path, records)

    parsed = list(iter_sdk_records(root / "raw/sdk_records.bin"))
    assert [record.sink for record in parsed] == [4, 7, 5]
    assert [record.sequence for record in parsed] == [1, 2, 1]
    assert parsed[0].device_ts == 100 and parsed[0].capture_ts_ns == 11
    assert parsed[0].arrival_ts_ns == 12
    assert parsed[0].host_monotonic_ns == 1_000 and parsed[0].host_unix_ns == 1_001
    assert parsed[0].payload == packets[0]
    assert parsed[1].payload == _imu_payload(1_000_000)
    assert [record.sink for record in iter_sdk_records(root / "raw/sdk_records.bin", limit=2)] == [4, 7]

    rows = list(iter_imu_rows(root))
    assert len(rows) == IMU_ROWS_PER_PACKET
    first_time, second_time, values = rows[0]
    assert first_time == 1_000_000
    assert second_time == 1_000_000 + 12_345
    assert values == pytest.approx((0.1, 0.2, 9.8, 0.01, 0.02, 0.03), abs=1e-6)
    assert rows[1][0] - rows[0][0] == IMU_STEP_NS


def test_iter_imu_rows_rejects_malformed_payloads(tmp_path):
    root = (tmp_path / "manual")
    (root / "raw").mkdir(parents=True)
    path = root / "raw/imu_records.bin"
    path.write_bytes(_sdk_record_bytes(7, 1, 10, b"\x00" * 41))
    with pytest.raises(RecordingFormatError):
        list(iter_imu_rows(root))

    row = IMU_PAYLOAD_ROW.pack(5_000_000, 5_000_100, 0.1, 0.2, 9.8, 0.0, 0.0, 0.0)
    duplicate = row + IMU_PAYLOAD_ROW.pack(5_000_000, 5_000_100, 0.1, 0.2, 9.8, 0.0, 0.0, 0.0)
    path.write_bytes(_sdk_record_bytes(7, 1, 10, duplicate))
    with pytest.raises(RecordingFormatError):
        list(iter_imu_rows(root))


# ---------------------------------------------------------------- 双目配对

def test_iter_paired_frames_pairs_sequence_and_equal_device_ts(tmp_path):
    packets = _hevc_packets()
    records = [
        (4, 0, 10, 100, 200, packets[0]),                    # 只有左侧：不配对
        (5, 1, 11, 101, 201, packets[1]),                    # 右侧先到，随后左侧
        (4, 1, 11, 102, 202, packets[1]),
        (4, 2, 12, 103, 203, packets[2]),
        (5, 2, 12, 104, 204, packets[2]),
        (4, 3, 13, 105, 205, packets[3]),                    # device_ts 不一致：丢弃
        (5, 3, 14, 106, 206, packets[3]),
        (4, 4, 15, 107, 207, packets[4]),
        (5, 4, 15, 108, 208, packets[4]),
    ]
    root = _build_recording(tmp_path, records, imu_payloads=[_imu_payload(1_000_000)])

    stats = {}
    frames = list(iter_paired_frames(root, stats=stats))
    assert [frame.sequence for frame in frames] == [1, 2, 4]
    assert [frame.index for frame in frames] == [0, 1, 2]
    assert [frame.device_ts for frame in frames] == [11, 12, 15]
    assert frames[0].capture_ts_left_ns == 102 and frames[0].capture_ts_right_ns == 101
    assert frames[0].arrival_ts_left_ns == 202 and frames[0].arrival_ts_right_ns == 201
    _assert_color(frames[0].left_bgr, _color(1))
    _assert_color(frames[0].right_bgr, _color(1))
    _assert_color(frames[2].left_bgr, _color(4))
    assert frames[0].left_bgr.shape == (HEIGHT, WIDTH, 3)
    assert stats["device_ts_mismatch"] == 1
    assert stats["pending_unpaired"] == 1
    assert stats["pending_pair_drops"] == 0
    assert stats["skipped_decode_packets"] == 0
    assert stats["decode_errors"] == 0

    assert [frame.sequence for frame in iter_paired_frames(root, limit=2)] == [1, 2]


class _FakeFrame:
    def __init__(self, value):
        self._image = np.full((HEIGHT, WIDTH, 3), int(value), dtype=np.uint8)

    def to_ndarray(self, format=None):  # noqa: A002 - 与 PyAV 接口一致
        return self._image


class _FakeDecoder:
    """按 payload 内容伪造解码结果，用于稳定覆盖“解码 0/2 帧跳过”分支。"""

    def __init__(self):
        self._map = {
            b"": [],
            b"L": [_FakeFrame(10)],
            b"R": [_FakeFrame(20)],
            b"TWO": [_FakeFrame(30), _FakeFrame(31)],
        }

    def decode(self, packet):
        return self._map.get(bytes(packet), [])


def test_iter_paired_frames_skips_zero_and_two_frame_decodes(tmp_path, monkeypatch):
    records = [
        (4, 0, 10, 0, 0, b""),        # 解码 0 帧：跳过
        (5, 0, 10, 0, 0, b""),        # 解码 0 帧：跳过
        (4, 1, 11, 0, 0, b"TWO"),     # 解码 2 帧：跳过
        (5, 1, 11, 0, 0, b"R"),       # 单侧到达，留待配对
        (4, 2, 12, 0, 0, b"L"),
        (5, 2, 12, 0, 0, b"TWO"),     # 解码 2 帧：跳过
        (4, 3, 13, 0, 0, b"L"),
        (5, 3, 13, 0, 0, b"R"),
    ]
    root = _build_recording(tmp_path, records)

    import av

    monkeypatch.setattr(av.CodecContext, "create", staticmethod(lambda *args: _FakeDecoder()))
    stats = {}
    frames = list(iter_paired_frames(root, stats=stats))
    assert [frame.sequence for frame in frames] == [3]
    assert frames[0].left_bgr.mean() == pytest.approx(10)
    assert frames[0].right_bgr.mean() == pytest.approx(20)
    assert stats["skipped_decode_packets"] == 4
    assert stats["decode_errors"] == 0
    assert stats["paired_frames"] == 1
    assert stats["pending_unpaired"] == 2

    # build_replay_check 也必须把“抽样解码跳过的 0/2 帧”计入报告，而不是崩溃。
    report = build_replay_check(root)
    assert report["frames"]["beginning_sample"]["skipped_decode_packets"] >= 1


# ---------------------------------------------------------------- 索引容错

def test_iter_index_rows_tolerates_truncated_tail(tmp_path):
    records = _camera_pair(0, 0, 0, 10) + _camera_pair(1, 1, 1, 11)
    root = _build_recording(tmp_path, records, imu_payloads=[_imu_payload(1_000_000)])
    valid_rows = list(iter_index_rows(root))
    assert len(valid_rows) == 5

    index_path = root / "raw/sdk_index.jsonl"
    with index_path.open("a", encoding="utf-8") as file_obj:
        file_obj.write('{"offset": 12345, "sink": 4')  # 故意写半行、无换行
    assert [row["sink"] for row in iter_index_rows(root)] == [row["sink"] for row in valid_rows]

    # 中间行损坏必须显式报错，不能静默跳过。
    broken = tmp_path / "broken"
    (broken / "raw").mkdir(parents=True)
    (broken / "raw/sdk_index.jsonl").write_text(
        '{"sink": 4, "sequence": 1}\nnot-json\n{"sink": 5}\n', encoding="utf-8"
    )
    with pytest.raises(RecordingFormatError):
        list(iter_index_rows(broken))


# ---------------------------------------------------------------- 回放校验

def _replay_ready_recording(tmp_path, name="session", pairs=6, imu_rows=(16, 11, 14, 15)):
    records = []
    packets = _hevc_packets()
    for sequence in range(pairs):
        device_ts = 1_000_000 + sequence * 33_333
        records.extend(_camera_pair(sequence, sequence % 8, (sequence + 2) % 8, device_ts))
    # 实测 SDK 单包行数可变（11–16 行，并非固定 16），校验不能按 16 行假设报警。
    imu_payloads = []
    first_time = 10_000_000
    for rows in imu_rows:
        imu_payloads.append(_imu_payload(first_time, rows=rows))
        first_time += rows * IMU_STEP_NS
    return _build_recording(tmp_path, records, imu_payloads=imu_payloads, name=name)


def test_build_replay_check_ready_and_recorder_close_integration(tmp_path):
    root = _replay_ready_recording(tmp_path)
    imu_row_count = 16 + 11 + 14 + 15
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    check = manifest["replay_check"]
    assert check["replay_ready"] is True
    assert check["reason"] is None
    assert check["recording_status"] == "transport_unverified"
    assert check["frames"]["index_pairs"] == 6
    assert check["frames"]["index_unpaired_packets"] == 0
    assert check["frames"]["beginning_sample"]["pairs"] >= 1
    assert check["imu"]["rows"] == imu_row_count
    assert check["imu"]["rate_hz"] == pytest.approx(480, abs=5)
    assert check["imu"]["gap_count"] == 0
    assert check["frames"]["device_ts_gap_count"] == 0
    assert check["frames"]["paired_device_ts_delta_median_ms"] == pytest.approx(33.333, abs=0.1)
    assert not any("暂停/丢包缺口" in warning for warning in check["warnings"]), check["warnings"]
    assert not any("整数倍" in warning for warning in check["warnings"]), check["warnings"]
    assert all(item["ok"] for item in check["checks"]), check["checks"]

    on_disk = json.loads((root / "replay_check.json").read_text(encoding="utf-8"))
    assert on_disk["replay_ready"] is True
    assert on_disk == check

    # 录制状态/计数等既有字段不因校验而改变。
    assert manifest["recording_status"] == "transport_unverified"
    assert manifest["sdk_count"] == 6 * 2 + 4
    assert manifest["sdk_count_by_sink"] == {"4": 6, "5": 6, "7": 4}
    assert manifest["sdk_sha256"] and manifest["imu_sha256"]


def test_build_replay_check_pause_gap_keeps_imu_rate_and_warns(tmp_path):
    """暂停录制造成 ~0.5 s 缺口：IMU 速率仍应≈480 Hz，并输出明确的缺口告警。"""
    pairs = 8
    frame_step_us = 33_333  # 约 30 fps（device_ts 单位候选为微秒）
    pause_us = 500_000
    records = []
    device_ts = 1_000_000
    for sequence in range(pairs):
        if sequence == 4:
            device_ts += pause_us  # 暂停 0.5 s：视频/IMU 记录都出现缺口
        records.extend(_camera_pair(sequence, sequence % 8, (sequence + 2) % 8, device_ts))
        device_ts += frame_step_us

    imu_payloads = []
    first_time = 10_000_000
    for packet in range(pairs):
        if packet == 4:
            first_time += 500_000_000  # IMU 侧的同一处暂停缺口（0.5 s）
        imu_payloads.append(_imu_payload(first_time))
        first_time += IMU_ROWS_PER_PACKET * IMU_STEP_NS
    root = _build_recording(tmp_path, records, imu_payloads=imu_payloads)

    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    check = manifest["replay_check"]
    # 缺口不等于不可回放，结论不受影响。
    assert check["replay_ready"] is True

    # 旧口径（总跨度/行数）会算出约 166 Hz；新口径取相邻行增量中位数，应恢复≈480 Hz。
    assert check["imu"]["rows"] == pairs * IMU_ROWS_PER_PACKET
    assert check["imu"]["rate_hz"] == pytest.approx(480, abs=2)
    assert check["imu"]["gap_count"] == 1
    normal_span_s = (pairs * IMU_ROWS_PER_PACKET - 1) * IMU_STEP_NS / 1e9
    assert check["imu"]["span_s"] == pytest.approx(normal_span_s + 0.5, abs=0.01)

    frames = check["frames"]
    assert frames["index_pairs"] == pairs
    assert frames["paired_device_ts_delta_median_ms"] == pytest.approx(frame_step_us / 1000, abs=0.1)
    assert frames["paired_device_ts_delta_max_ms"] == pytest.approx((frame_step_us + pause_us) / 1000, abs=0.1)
    assert frames["device_ts_gap_threshold_ms"] == pytest.approx(200.0, abs=0.1)
    assert frames["device_ts_gap_count"] == 1
    assert frames["device_ts_gap_max_ms"] == pytest.approx((frame_step_us + pause_us) / 1000, abs=0.1)

    warnings = check["warnings"]
    gap_warning = next(warning for warning in warnings if "暂停/丢包缺口" in warning)
    assert "最大 533 ms" in gap_warning
    assert "回放可能在缺口处报 IMU 样本不足" in gap_warning
    assert "VIO 回放建议连续录制" in gap_warning
    continuity = next(item for item in check["checks"] if item["name"] == "帧时间连续性")
    assert continuity["ok"] is True and continuity["detail"] == gap_warning
    assert all(item["ok"] for item in check["checks"]), check["checks"]


def test_build_replay_check_detects_incomplete_status(tmp_path):
    root = _replay_ready_recording(tmp_path)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["recording_status"] = "incomplete"
    manifest["incomplete_reason"] = "模拟中断"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")

    check = build_replay_check(root)
    assert check["replay_ready"] is False
    assert "录制不完整" in check["reason"] and "模拟中断" in check["reason"]
    assert check["recording_status"] == "incomplete"
    assert json.loads((root / "replay_check.json").read_text(encoding="utf-8"))["replay_ready"] is False


def test_build_replay_check_not_ready_when_sdk_records_missing_or_corrupt(tmp_path):
    root = _replay_ready_recording(tmp_path, name="missing")
    (root / "raw/sdk_records.bin").unlink()
    check = build_replay_check(root)
    assert check["replay_ready"] is False
    assert "缺少" in check["reason"]

    corrupt_root = _replay_ready_recording(tmp_path, name="corrupt")
    sdk_path = corrupt_root / "raw/sdk_records.bin"
    with sdk_path.open("r+b") as file_obj:
        file_obj.write(b"\x00\x00\x00\x00")
    check = build_replay_check(corrupt_root)
    assert check["replay_ready"] is False
    assert "magic" in check["reason"]


def test_recorder_close_replay_check_exception_never_breaks_close(tmp_path, monkeypatch):
    def boom(_root):
        raise RuntimeError("模拟校验失败")

    monkeypatch.setattr(recorder_module, "build_replay_check", boom)
    recorder = Recorder(tmp_path / "session", _metadata())
    writer = "fake-camera-packet".encode()
    recorder.write_sdk((4, 1, 10, 11, 12, 13, 14, writer))
    recorder.close(capture_stats={"sdk_packets_received": 1})

    manifest = json.loads((recorder.path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["recording_status"] == "transport_unverified"
    assert manifest["incomplete_reason"] is None
    assert manifest["capture_stats"]["sdk_packets_received"] == 1
    assert manifest["replay_check"]["error"] == "模拟校验失败"
    assert manifest["replay_check"]["replay_ready"] is False
    fallback = json.loads((recorder.path / "replay_check.json").read_text(encoding="utf-8"))
    assert fallback["error"] == "模拟校验失败"


def test_recorder_close_keeps_incomplete_reason_with_replay_check(tmp_path, monkeypatch):
    def boom(_root):
        raise RuntimeError("模拟校验失败")

    monkeypatch.setattr(recorder_module, "build_replay_check", boom)
    recorder = Recorder(tmp_path / "session", _metadata())
    recorder.write_sdk((4, 1, 10, 11, 12, 13, 14, b"payload"))
    recorder.close(incomplete_reason="模拟录制中断")

    manifest = json.loads((recorder.path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["recording_status"] == "incomplete"
    assert manifest["incomplete_reason"] == "模拟录制中断"
    assert manifest["replay_check"]["error"] == "模拟校验失败"


# ---------------------------------------------------------------- 视频导出

def _load_export_module():
    spec = importlib.util.spec_from_file_location(
        "export_recording_videos", ROOT / "scripts/export_recording_videos.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_export_recording_videos(tmp_path):
    records = []
    packets = _hevc_packets()
    for sequence in range(3):
        records.extend(_camera_pair(sequence, sequence, sequence, 1_000_000 + sequence * 33_333))
    root = _build_recording(
        tmp_path, records,
        imu_payloads=[_imu_payload(5_000_000), _imu_payload(5_000_000 + 16 * IMU_STEP_NS)],
    )

    module = _load_export_module()
    report = module.export_recording(root, scale=0.5, fps=30, limit=2)
    out = root / "export"
    assert report["status"] == "completed"
    assert report["frames"] == 2
    assert report["encoder"] in ("opencv-mp4v", "pyav-libx264")
    assert report["imu_rows"] == 2 * IMU_ROWS_PER_PACKET
    assert report["duration_s"] == pytest.approx(2 / 30, abs=1e-6)
    for name in ("left.mp4", "right.mp4", "imu.csv", "frames.csv", "export.json"):
        assert (out / name).is_file(), name
    assert report["files"]["left_mp4_bytes"] > 0
    assert json.loads((out / "export.json").read_text(encoding="utf-8")) == report

    import cv2

    for name in ("left.mp4", "right.mp4"):
        capture = cv2.VideoCapture(str(out / name))
        count = 0
        while capture.read()[0]:
            count += 1
        capture.release()
        assert count == 2
    frames_rows = (out / "frames.csv").read_text(encoding="utf-8").splitlines()
    assert len(frames_rows) == 3  # 表头 + 2 帧
    imu_rows = (out / "imu.csv").read_text(encoding="utf-8").splitlines()
    assert len(imu_rows) == 2 * IMU_ROWS_PER_PACKET + 1

    # 默认输出目录已存在：拒绝覆盖。
    with pytest.raises(FileExistsError):
        module.export_recording(root)
    with pytest.raises(FileExistsError):
        module.export_recording(root, output=out)
