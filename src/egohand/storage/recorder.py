# 原始层/结果层分开；VS030 原始 SDK 包按新 IPC v2 保存，写盘线程与采集/预览解耦。
import hashlib
import importlib.metadata
import json
import os
import queue
import shutil
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from ..core.types import serializable, SCHEMA_VERSION
from ..calibration.loader import save_calibration
from ..calibration.rectifier import StereoRectifier
# IPC v2 头/载荷常量与只读校验入口统一放在 sdk_recording，避免与本模块循环 import。
from .sdk_recording import (
    SDK_MAX_PAYLOAD,
    SDK_RECORD_HEADER,
    SDK_RECORD_MAGIC,
    SDK_RECORD_VERSION,
    build_replay_check,
)


class RecordingError(RuntimeError):
    pass


class Recorder:
    """Append-only raw/result layers. Manifest is checkpointed and atomically replaced."""

    def __init__(self, directory, metadata, calibration=None, algorithm_config=None, model_info=None):
        self.path = Path(directory)
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / "raw/left").mkdir(parents=True)
        (self.path / "raw/right").mkdir(parents=True)
        (self.path / "results").mkdir()

        self.manifest = {
            "schema_version": "1.1.0",
            "result_schema_version": SCHEMA_VERSION,
            "metadata": serializable(metadata),
            "algorithm_config": serializable(algorithm_config or {}),
            "model": serializable(model_info or {}),
            "recording_status": "recording",
            "raw_count": 0,
            "frame_index_count": 0,
            "result_count": 0,
            "sdk_count": 0,
            "sdk_payload_bytes": 0,
            "sdk_count_by_sink": {},
            "sdk_stream_stats": {},
            "calibration": "calibration.yaml" if calibration else None,
            "image_geometry": "raw",
            "raw_index": "raw/frames.jsonl",
            "result_index": "results/frames.jsonl",
            "created_time_ns": time.time_ns(),
        }
        if calibration:
            save_calibration(calibration, self.path / "calibration.yaml")
        versions = {}
        for name in ("egohand-data-engine", "mediapipe", "opencv-contrib-python", "numpy", "av"):
            try:
                versions[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                versions[name] = None
        self.manifest["software_versions"] = versions

        if calibration:
            try:
                rectifier = StereoRectifier(calibration)
                np.savez_compressed(
                    self.path / "rectification.npz",
                    R1=rectifier.R1, R2=rectifier.R2, P1=rectifier.P1,
                    P2=rectifier.P2, Q=rectifier.Q,
                    new_K_left=rectifier.new_K_left, new_K_right=rectifier.new_K_right,
                    roi_left=np.array(rectifier.roi_left if rectifier.roi_left is not None else [], dtype=int),
                    roi_right=np.array(rectifier.roi_right if rectifier.roi_right is not None else [], dtype=int),
                )
                self.manifest["rectification"] = "rectification.npz"
            except ValueError as exc:
                self.manifest["rectification_error"] = str(exc)

        self.raw = (self.path / "raw/frames.jsonl").open("a", encoding="utf-8")
        self.results = (self.path / "results/frames.jsonl").open("a", encoding="utf-8")
        self.sdk = None
        self.imu = None
        self.sdk_index = None
        self._sdk_hash = hashlib.sha256()
        self._imu_hash = hashlib.sha256()
        self._sdk_records_since_checkpoint = 0

        if getattr(metadata, "source_type", None) == "vendor_vs030":
            self.sdk = (self.path / "raw/sdk_records.bin").open("wb")
            self.imu = (self.path / "raw/imu_records.bin").open("wb")
            self.sdk_index = (self.path / "raw/sdk_index.jsonl").open("a", encoding="utf-8")
            self.manifest.update(
                sdk_records="raw/sdk_records.bin",
                sdk_record_header_bytes=SDK_RECORD_HEADER.size,
                sdk_record_format={
                    "version": SDK_RECORD_VERSION,
                    "struct": "<IHHIIQQQQQQ",
                    "fields": [
                        "magic", "version", "header_size", "sink", "payload_size",
                        "sequence", "device_timestamp_raw", "sdk_capture_timestamp_ns",
                        "sdk_arrival_timestamp_ns", "host_receive_monotonic_ns",
                        "host_receive_unix_ns",
                    ],
                    "payload": "raw SDK payload; no decode/re-encode",
                    "sink_ids": {"4": "left H265", "5": "right H265", "7": "IMU raw"},
                    "zero_timestamp": "preserved as received; semantics are not inferred",
                },
                sdk_index="raw/sdk_index.jsonl",
                imu_records="raw/imu_records.bin",
                imu_record_note="Convenience copy of sink 7 records; canonical copy also exists in sdk_records.bin",
                sdk_timestamp_semantics={
                    "device_timestamp_raw": "unit and exposure origin unverified",
                    "sdk_capture_timestamp_ns": "SDK-provided diagnostic timestamp; not independently verified as exposure time",
                    "sdk_arrival_timestamp_ns": "SDK-provided diagnostic timestamp; clock semantics unverified",
                    "host_receive_monotonic_ns": "host perf_counter_ns sampled immediately after IPC packet read",
                    "host_receive_unix_ns": "host time_ns sampled immediately after IPC packet read",
                },
                sdk_completeness="transport loss must be checked from SDK diagnostics; clean file close alone does not prove sensor transport completeness",
                frame_index="raw/camera_frames.jsonl",
                frame_index_note="decoded stereo bundle index; original H265 and IMU bytes remain in SDK record containers",
            )
            root = Path(__file__).resolve().parents[3]
            for source, name in (
                (root / "标定结果/1/data3-camchain-imucam.yaml", "kalibr_camchain.yaml"),
                (root / "标定结果/1/data3-results-imucam.txt", "kalibr_results.txt"),
                (root / "configs/vio/live_timing.yaml", "vio_timing.yaml"),
                (root / "configs/vio/live_quality.yaml", "vio_quality.yaml"),
                (root / "configs/vio/vs030_rectified_preliminary.yaml", "vio_settings.yaml"),
            ):
                if source.is_file():
                    shutil.copy2(source, self.path / name)

        self.frame_index = None
        if getattr(metadata, "source_type", None) == "vendor_vs030":
            self.frame_index = (self.path / "raw/camera_frames.jsonl").open("a", encoding="utf-8")

        self.closed = False
        self._manifest()

    def _manifest(self):
        temp = self.path / "manifest.tmp"
        temp.write_text(
            json.dumps(serializable(self.manifest), ensure_ascii=False, allow_nan=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temp, self.path / "manifest.json")

    @staticmethod
    def _flush_file(file_obj, durable=False):
        if file_obj is None:
            return
        file_obj.flush()
        if durable:
            os.fsync(file_obj.fileno())

    def write_raw(self, frame):
        key = f"{self.manifest['raw_count']:09d}_{frame.frame_id}"
        paths = []
        for side, image in (("left", frame.left_image), ("right", frame.right_image)):
            relative = f"raw/{side}/{key}.png"
            if not cv2.imwrite(str(self.path / relative), image):
                raise RecordingError("原始帧写入失败")
            paths.append(relative)
        record = {
            key: serializable(getattr(frame, key))
            for key in (
                "frame_id", "timestamp_ns", "left_timestamp_ns", "right_timestamp_ns",
                "calibration_id", "source_type", "synchronized", "image_geometry",
                "metadata", "left_detections", "right_detections",
            )
        }
        record.update(left_image=paths[0], right_image=paths[1])
        self.raw.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.raw.flush()
        self.manifest["raw_count"] += 1

    def write_frame_index(self, frame):
        """Index a decoded VS030 stereo pair without copying large raw SDK payloads into JSON."""
        if self.frame_index is None:
            raise RecordingError("当前录制不是 VS030 SDK 数据源")
        metadata = dict(frame.metadata or {})
        raw_imu = metadata.pop("sdk_imu_packets_raw", ())
        metadata["sdk_imu_packet_count"] = len(raw_imu)
        # Keep the compact timing evidence; raw IMU payload bytes are in imu_records.bin/sdk_records.bin.
        metadata["sdk_imu_frame_timing"] = serializable(metadata.get("sdk_imu_frame_timing", ()))
        row = {
            "frame_id": int(frame.frame_id),
            "timestamp_ns": int(frame.timestamp_ns),
            "source_type": frame.source_type,
            "calibration_id": frame.calibration_id,
            "image_geometry": frame.image_geometry,
            "left_shape": list(frame.left_image.shape),
            "right_shape": list(frame.right_image.shape),
            "metadata": serializable(metadata),
        }
        self.frame_index.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        self.manifest["frame_index_count"] += 1
        if self.manifest["frame_index_count"] % 100 == 0:
            self.frame_index.flush()

    def write_result(self, result):
        self.results.write(json.dumps(serializable(result), ensure_ascii=False, allow_nan=False) + "\n")
        self.results.flush()
        self.manifest["result_count"] += 1

    def write_sdk(self, record):
        # IPC v2: sink, sequence, device_ts, sdk_capture_ts, sdk_arrival_ts,
        # host_receive_monotonic_ns, host_receive_unix_ns, payload.
        # Accept the old six-field Python tuple only for compatibility with older callers.
        if len(record) == 8:
            (sink, sequence, device_ts, capture_ts, arrival_ts,
             host_monotonic, host_unix, payload) = record
        elif len(record) == 6:
            sink, sequence, device_ts, host_monotonic, host_unix, payload = record
            capture_ts = arrival_ts = 0
        else:
            raise RecordingError(f"SDK 记录字段数错误：{len(record)}")
        sink = int(sink)
        sequence = int(sequence)
        device_ts = int(device_ts)
        capture_ts = int(capture_ts or 0)
        arrival_ts = int(arrival_ts or 0)
        host_monotonic = int(host_monotonic)
        host_unix = int(host_unix)
        payload = bytes(payload)
        if not 0 <= len(payload) <= SDK_MAX_PAYLOAD:
            raise RecordingError(f"SDK payload 长度非法：{len(payload)}")

        header = SDK_RECORD_HEADER.pack(
            SDK_RECORD_MAGIC, SDK_RECORD_VERSION, SDK_RECORD_HEADER.size,
            sink, len(payload), sequence, device_ts, capture_ts, arrival_ts,
            host_monotonic, host_unix,
        )
        offset = self.sdk.tell()
        self.sdk.write(header)
        self.sdk.write(payload)
        self._sdk_hash.update(header)
        self._sdk_hash.update(payload)

        imu_offset = None
        if sink == 7:
            imu_offset = self.imu.tell()
            self.imu.write(header)
            self.imu.write(payload)
            self._imu_hash.update(header)
            self._imu_hash.update(payload)

        index_row = {
            "offset": offset,
            "record_bytes": len(header) + len(payload),
            "sink": sink,
            "sequence": sequence,
            "device_timestamp_raw": device_ts,
            "sdk_capture_timestamp_ns": capture_ts,
            "sdk_arrival_timestamp_ns": arrival_ts,
            "host_receive_monotonic_ns": host_monotonic,
            "host_receive_unix_ns": host_unix,
            "payload_bytes": len(payload),
            "imu_copy_offset": imu_offset,
        }
        self.sdk_index.write(json.dumps(index_row, ensure_ascii=False, allow_nan=False) + "\n")

        self.manifest["sdk_count"] += 1
        self.manifest["sdk_payload_bytes"] += len(payload)
        counts = self.manifest["sdk_count_by_sink"]
        counts[str(sink)] = int(counts.get(str(sink), 0)) + 1
        streams = self.manifest["sdk_stream_stats"]
        state = streams.setdefault(str(sink), {
            "count": 0, "payload_bytes": 0, "first_sequence": sequence,
            "last_sequence": sequence, "first_device_timestamp_raw": device_ts,
            "last_device_timestamp_raw": device_ts,
            "non_monotonic_sequence_count": 0, "non_monotonic_device_timestamp_count": 0,
        })
        if sequence <= state["last_sequence"] and state["count"] > 0:
            state["non_monotonic_sequence_count"] += 1
        if device_ts < state["last_device_timestamp_raw"] and state["count"] > 0:
            state["non_monotonic_device_timestamp_count"] += 1
        state["count"] += 1
        state["payload_bytes"] += len(payload)
        state["last_sequence"] = sequence
        state["last_device_timestamp_raw"] = device_ts
        self._sdk_records_since_checkpoint += 1

        if self._sdk_records_since_checkpoint >= 500:
            for file_obj in (self.sdk, self.imu, self.sdk_index):
                self._flush_file(file_obj, durable=True)
            self.manifest["sdk_sha256_checkpoint"] = self._sdk_hash.hexdigest()
            self.manifest["imu_sha256_checkpoint"] = self._imu_hash.hexdigest()
            self._manifest()
            self._sdk_records_since_checkpoint = 0

    def record_event(self, event):
        with (self.path / "events.jsonl").open("a", encoding="utf-8") as file_obj:
            file_obj.write(json.dumps(serializable(event), ensure_ascii=False, allow_nan=False) + "\n")
        self.manifest["events"] = "events.jsonl"

    def close(self, incomplete_reason=None, capture_stats=None):
        if self.closed:
            return
        for file_obj in (self.raw, self.results, self.frame_index, self.sdk_index, self.imu, self.sdk):
            if file_obj is not None:
                self._flush_file(file_obj, durable=True)
                file_obj.close()
        self.closed = True
        if capture_stats is not None:
            self.manifest["capture_stats"] = serializable(capture_stats)
        if self.sdk:
            self.manifest["sdk_sha256"] = self._sdk_hash.hexdigest()
            self.manifest["imu_sha256"] = self._imu_hash.hexdigest()
            self.manifest["sdk_file_bytes"] = (self.path / self.manifest["sdk_records"]).stat().st_size
            self.manifest["imu_file_bytes"] = (self.path / self.manifest["imu_records"]).stat().st_size
        self.manifest["recording_status"] = (
            "incomplete" if incomplete_reason else ("transport_unverified" if self.sdk else "complete")
        )
        self.manifest["incomplete_reason"] = incomplete_reason
        self.manifest["closed_time_ns"] = time.time_ns()
        # 先落盘最终状态：build_replay_check 只读 manifest 判断录制是否已结束。
        self._manifest()
        if self.sdk:
            self._write_replay_check()

    def _write_replay_check(self):
        """vendor_vs030 关闭后自动校验可回放性；校验异常只记录，不改录制状态、不阻止 close。"""
        try:
            check = build_replay_check(self.path)
        except Exception as exc:  # noqa: BLE001 - 校验器故障不得影响录制收尾
            check = {
                "replay_ready": False,
                "reason": f"回放校验异常：{exc}",
                "error": str(exc),
                "generated_time_ns": time.time_ns(),
                "checks": [],
                "warnings": [],
                "errors": [str(exc)],
            }
            try:
                (self.path / "replay_check.json").write_text(
                    json.dumps(check, ensure_ascii=False, allow_nan=False, indent=2),
                    encoding="utf-8",
                )
            except OSError:
                pass
        try:
            self.manifest["replay_check"] = check
            self._manifest()
        except Exception as exc:  # noqa: BLE001 - manifest 落盘失败同样不能阻止 close
            self.manifest["replay_check_error"] = str(exc)


class AsyncRecorder:
    """Bounded writer queue. Live overflow stops recording explicitly; it never silently drops raw data."""

    def __init__(self, *args, max_queue=32, **kwargs):
        self.writer = Recorder(*args, **kwargs)
        self.queue = queue.Queue(max_queue)
        self.error = None
        self.accepting = True
        self.paused = False
        self.written = 0
        self.bytes_written = 0
        self.started = time.perf_counter()
        self._closing = threading.Event()
        self._submit_lock = threading.Lock()
        self.capture_stats = None
        self.last_raw_submitted = None
        self.thread = threading.Thread(target=self._run, name="EgoHandWriter", daemon=False)
        self.thread.start()

    def submit(self, kind, value, offline=False):
        while True:
            with self._submit_lock:
                if self.paused or not self.accepting:
                    return False
                try:
                    self.queue.put_nowait((kind, value))
                    if kind == "raw":
                        self.last_raw_submitted = {"frame_id": value.frame_id, "timestamp_ns": value.timestamp_ns}
                    return True
                except queue.Full:
                    if not offline:
                        self.error = "RECORDING_QUEUE_OVERFLOW：原始录制已停止，记录不完整"
                        self.accepting = False
                        self._closing.set()
                        return False
            self._closing.wait(0.01)

    def set_paused(self, value):
        with self._submit_lock:
            if not self.accepting:
                return
            self.paused = bool(value)
            event = {
                "event": "pause" if value else "resume",
                "operation_time_ns": time.time_ns(),
                "last_raw_submitted": self.last_raw_submitted,
            }
            try:
                self.queue.put_nowait(("event", event))
            except queue.Full:
                self.error = "RECORDING_QUEUE_OVERFLOW：录制操作无法保存，记录不完整"
                self.accepting = False
                self._closing.set()

    def _run(self):
        try:
            while not self._closing.is_set() or not self.queue.empty():
                try:
                    kind, value = self.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                try:
                    if kind == "raw":
                        self.writer.write_raw(value)
                        self.bytes_written += value.left_image.nbytes + value.right_image.nbytes
                    elif kind == "frame_index":
                        self.writer.write_frame_index(value)
                    elif kind == "result":
                        self.writer.write_result(value)
                    elif kind == "sdk":
                        self.writer.write_sdk(value)
                        self.bytes_written += len(value[-1]) + SDK_RECORD_HEADER.size
                    elif kind == "event":
                        self.writer.record_event(value)
                    else:
                        raise RecordingError(f"未知录制任务类型：{kind}")
                    self.written += 1
                finally:
                    self.queue.task_done()
        except Exception as exc:
            self.error = f"RECORDING_WRITE_FAILED: {exc}"
            self.accepting = False
        finally:
            self.writer.close(self.error, self.capture_stats)

    def close(self, capture_stats=None):
        with self._submit_lock:
            self.accepting = False
            self.capture_stats = capture_stats
            self._closing.set()
        self.thread.join(timeout=30)
        if self.thread.is_alive():
            raise RecordingError("录制写入线程未在 30 秒内退出")
        # Overflow can make the writer thread finish before the UI asks it to close;
        # persist the final source-side diagnostics even in that path.
        if capture_stats is not None:
            self.writer.manifest["capture_stats"] = serializable(capture_stats)
            self.writer._manifest()
