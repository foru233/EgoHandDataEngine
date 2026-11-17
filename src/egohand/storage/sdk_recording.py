"""录制目录只读解析/解码公共模块：IPC v2 原始记录、IMU 载荷、双目配对与回放校验。

约束：
- 本模块不 import recorder.py，避免 recorder <-> sdk_recording 循环依赖；
  IPC v2 头与 IMU 载荷布局常量以本模块为准，recorder.py 从这里 import。
- 只读：除 build_replay_check 写 <root>/replay_check.json 外不修改录制目录。
- 摄像头原始数据是 H.265 裸包，每条 SDK 记录对应一个 HEVC access unit；
  双目按 sequence 相同且 device_ts 相等配对（与 sources/vendor_xdsdk.py 实时解码一致）。
"""
from dataclasses import dataclass
import json
import math
import os
import statistics
import struct
import time
from pathlib import Path

import numpy as np

# magic:u32, version:u16, header_size:u16, sink:u32, payload_size:u32,
# sequence/device_ts/SDK_capture_ts/SDK_arrival_ts/host_monotonic_ns/host_unix_ns:u64
SDK_RECORD_HEADER = struct.Struct("<IHHIIQQQQQQ")
SDK_RECORD_MAGIC = 0x58444652
SDK_RECORD_VERSION = 2
SDK_MAX_PAYLOAD = 8 * 1024 * 1024
# IMU 载荷候选布局：两个 u64 时间字段 + 加速度 xyz + 角速度 xyz，每行 40 B（语义未获厂商确认）。
IMU_PAYLOAD_ROW = struct.Struct("<QQ6f")
IMU_PAYLOAD_ROW_BYTES = IMU_PAYLOAD_ROW.size
CAMERA_SINKS = (4, 5)
IMU_SINK = 7
# 与实时预览一致：未配对的 pending sequence 最多保留 8 个，超限丢最旧。
_PENDING_PAIR_LIMIT = 8
_SAMPLE_BEGINNING_PACKETS = 600
_SAMPLE_MIDDLE_PACKETS = 400
# IMU 速率估计：暂停/丢包造成的 >100 ms 行间增量视为缺口，不参与中位数估计。
_IMU_RATE_MAX_DELTA_NS = 100_000_000
# 相邻配对帧间隔：超过 max(200 ms, 5×中位数) 视为暂停/丢包缺口（device_ts 按 *1000 换算 ns）。
_FRAME_GAP_MIN_NS = 200_000_000
_FRAME_GAP_MEDIAN_FACTOR = 5


class RecordingFormatError(ValueError):
    """录制文件格式错误（头部损坏、载荷截断、索引不可解析等）。"""


@dataclass(frozen=True)
class SdkRecord:
    sink: int
    sequence: int
    device_ts: int
    capture_ts_ns: int
    arrival_ts_ns: int
    host_monotonic_ns: int
    host_unix_ns: int
    payload: bytes


@dataclass
class PairedFrame:
    index: int
    sequence: int
    device_ts: int
    left_bgr: np.ndarray
    right_bgr: np.ndarray
    capture_ts_left_ns: int
    capture_ts_right_ns: int
    arrival_ts_left_ns: int
    arrival_ts_right_ns: int


# ---------------------------------------------------------------- 路径与索引

def _read_manifest(root):
    """容错读取 manifest.json；不存在或不可解析时返回空字典（调用方自行报错）。"""
    try:
        data = json.loads((Path(root) / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _recording_file(root, manifest, key, default):
    relative = manifest.get(key) if isinstance(manifest, dict) else None
    return Path(root) / str(relative or default)


def _sdk_records_path(root):
    return _recording_file(root, _read_manifest(root), "sdk_records", "raw/sdk_records.bin")


def _imu_records_path(root):
    return _recording_file(root, _read_manifest(root), "imu_records", "raw/imu_records.bin")


def _sdk_index_path(root):
    return _recording_file(root, _read_manifest(root), "sdk_index", "raw/sdk_index.jsonl")


def iter_index_rows(root):
    """逐行读取 raw/sdk_index.jsonl；只容错文件尾部的半行截断，中间损坏显式报错。"""
    path = _sdk_index_path(root)
    with path.open("r", encoding="utf-8") as file_obj:
        line_number = 0
        while True:
            line = file_obj.readline()
            if not line:
                return
            line_number += 1
            text = line.strip()
            if not text:
                continue
            try:
                row = json.loads(text)
            except json.JSONDecodeError as exc:
                # 只接受“最后一行被截断”：其后没有更多内容。
                if file_obj.read().strip():
                    raise RecordingFormatError(
                        f"sdk_index.jsonl 第 {line_number} 行无法解析：{exc}"
                    ) from exc
                return
            if not isinstance(row, dict):
                raise RecordingFormatError(f"sdk_index.jsonl 第 {line_number} 行不是 JSON 对象")
            yield row


# ---------------------------------------------------------------- 原始记录

def _iter_records_from(path, offset=0, limit=None):
    path = Path(path)
    yielded = 0
    position = int(offset)
    with path.open("rb") as file_obj:
        if position:
            file_obj.seek(position)
        while limit is None or yielded < limit:
            raw = file_obj.read(SDK_RECORD_HEADER.size)
            if not raw:
                return
            if len(raw) != SDK_RECORD_HEADER.size:
                raise RecordingFormatError(f"sdk_records.bin 头部被截断（offset {position}）")
            (
                magic, version, header_size, sink, size, sequence, device_ts,
                capture_ts_ns, arrival_ts_ns, host_monotonic_ns, host_unix_ns,
            ) = SDK_RECORD_HEADER.unpack(raw)
            if magic != SDK_RECORD_MAGIC:
                raise RecordingFormatError(
                    f"SDK 记录 magic 非法（offset {position}）：0x{magic:08X}"
                )
            if version != SDK_RECORD_VERSION:
                raise RecordingFormatError(f"SDK 记录版本不支持（offset {position}）：{version}")
            if header_size != SDK_RECORD_HEADER.size:
                raise RecordingFormatError(
                    f"SDK 记录头长度非法（offset {position}）：{header_size}"
                )
            if size > SDK_MAX_PAYLOAD:
                raise RecordingFormatError(f"SDK 记录载荷过大（offset {position}）：{size} B")
            payload = file_obj.read(size)
            if len(payload) != size:
                raise RecordingFormatError(f"sdk_records.bin 载荷被截断（offset {position}）")
            yield SdkRecord(
                sink=int(sink), sequence=int(sequence), device_ts=int(device_ts),
                capture_ts_ns=int(capture_ts_ns), arrival_ts_ns=int(arrival_ts_ns),
                host_monotonic_ns=int(host_monotonic_ns), host_unix_ns=int(host_unix_ns),
                payload=payload,
            )
            yielded += 1
            position += SDK_RECORD_HEADER.size + size


def read_record_at(path, offset):
    """读取指定偏移处的一条原始记录（越界/损坏显式报错）。"""
    try:
        return next(_iter_records_from(path, offset, limit=1))
    except StopIteration as exc:  # pragma: no cover - offset 位于 EOF 之后
        raise RecordingFormatError(f"offset {offset} 处没有 SDK 记录") from exc


def iter_sdk_records(path, limit=None):
    """顺序读取 raw/sdk_records.bin 的 IPC v2 记录。"""
    yield from _iter_records_from(path, 0, limit)


# ---------------------------------------------------------------- IMU

def iter_imu_packets(root):
    """Return raw IMU records with their SDK header, falling back to the combined stream."""
    path = _imu_records_path(root)
    if not path.is_file():
        path = _sdk_records_path(root)
    for record in iter_sdk_records(path):
        if record.sink == IMU_SINK:
            yield record


def iter_imu_payloads(root):
    """顺序读取 sink7 的原始载荷。"""
    for record in iter_imu_packets(root):
        yield record.payload


def iter_imu_rows(root):
    """解析 IMU 载荷为 (time_a_ns, time_b_ns, (ax,ay,az,gx,gy,gz))。

    非法长度、非有限数值、零时间戳或时间非递增都显式报错（与实时链路一致）。
    """
    previous = None
    for payload in iter_imu_payloads(root):
        if not payload or len(payload) % IMU_PAYLOAD_ROW_BYTES:
            raise RecordingFormatError(
                f"IMU 载荷长度 {len(payload)} B 不是 {IMU_PAYLOAD_ROW_BYTES} B 的整数倍"
            )
        for offset in range(0, len(payload), IMU_PAYLOAD_ROW_BYTES):
            time_a, time_b, *values = IMU_PAYLOAD_ROW.unpack_from(payload, offset)
            if not all(math.isfinite(value) for value in values):
                raise RecordingFormatError(f"IMU 载荷 offset {offset} 包含非有限数值")
            if not time_a or not time_b:
                raise RecordingFormatError(f"IMU 载荷 offset {offset} 包含零时间戳")
            if previous is not None and (time_a <= previous[0] or time_b <= previous[1]):
                raise RecordingFormatError(
                    f"IMU 时间非递增：({previous[0]}, {previous[1]}) -> ({time_a}, {time_b})"
                )
            previous = (time_a, time_b)
            yield (int(time_a), int(time_b), tuple(float(value) for value in values))


# ---------------------------------------------------------------- 双目配对

def _create_decoder():
    # 延迟 import：仅解码路径依赖 PyAV，recorder 只 import 常量时无需加载 av。
    import av

    return av.CodecContext.create("hevc", "r")


def _new_stats(stats):
    if stats is None:
        return {
            "camera_packets": {"4": 0, "5": 0},
            "decoded_frames": {"4": 0, "5": 0},
            "skipped_decode_packets": 0,
            "decode_errors": 0,
            "paired_frames": 0,
            "device_ts_mismatch": 0,
            "pending_pair_drops": 0,
            "pending_unpaired": 0,
        }
    return stats


def iter_paired_frames(root, limit=None, stats=None, *, stride=1, min_device_ts=None, cancelled=None):
    """Decode all HEVC packets; convert and yield every stride-th paired frame.

    Indices refer to the original paired stream, and limit counts yielded pairs.
    Skipped pairs still update the decoders and pairing statistics.
    """
    path = _sdk_records_path(root)
    if not path.is_file():
        raise FileNotFoundError(f"缺少原始 SDK 记录：{path}")
    if limit is not None:
        limit = int(limit)
        if limit <= 0:
            return
    if type(stride) is not int or stride < 1:
        raise ValueError('stride must be a positive integer')

    import av

    counters = _new_stats(stats)
    for key in ("camera_packets", "decoded_frames"):
        counters.setdefault(key, {"4": 0, "5": 0})
    for key in (
        "skipped_decode_packets", "decode_errors", "paired_frames",
        "device_ts_mismatch", "pending_pair_drops", "pending_unpaired",
    ):
        counters.setdefault(key, 0)

    decoders = {sink: _create_decoder() for sink in CAMERA_SINKS}
    pending = {}
    produced = 0
    yielded = 0
    clip = _read_manifest(root).get('sdk_clip') or {}
    clip_start = int(clip.get('start_pair_index', 0))
    clip_end = clip.get('end_pair_index')
    if clip_start < 0 or (clip_end is not None and int(clip_end) < clip_start):
        raise RecordingFormatError('SDK 截取范围无效')
    for record in iter_sdk_records(path):
        if cancelled is not None and cancelled():return
        if record.sink not in CAMERA_SINKS:
            continue
        counters["camera_packets"][str(record.sink)] += 1
        try:
            frames = decoders[record.sink].decode(av.Packet(record.payload))
        except av.error.FFmpegError:
            counters["decode_errors"] += 1
            continue
        if len(frames) != 1:
            # 解码出 0 帧（如 GOP 中间、空包）或 2 帧（异常包）都不参与配对。
            counters["skipped_decode_packets"] += 1
            continue
        counters["decoded_frames"][str(record.sink)] += 1
        entry = pending.setdefault(record.sequence, {})
        entry[record.sink] = (frames[0], record)
        if 4 in entry and 5 in entry:
            pair = pending.pop(record.sequence)
            left_frame, left_record = pair[4]
            right_frame, right_record = pair[5]
            if left_record.device_ts == right_record.device_ts:
                index = produced
                produced += 1
                counters["paired_frames"] = produced
                if index < clip_start:
                    while len(pending) > _PENDING_PAIR_LIMIT:
                        del pending[next(iter(pending))]
                        counters["pending_pair_drops"] += 1
                    continue
                if clip_end is not None and index > int(clip_end):
                    return
                index -= clip_start
                if min_device_ts is not None and left_record.device_ts < min_device_ts:
                    while len(pending) > _PENDING_PAIR_LIMIT:
                        del pending[next(iter(pending))]
                        counters["pending_pair_drops"] += 1
                    continue
                if index % stride:
                    while len(pending) > _PENDING_PAIR_LIMIT:
                        del pending[next(iter(pending))]
                        counters["pending_pair_drops"] += 1
                    continue
                yield PairedFrame(
                    index=index,
                    sequence=int(record.sequence),
                    device_ts=int(left_record.device_ts),
                    left_bgr=left_frame.to_ndarray(format="bgr24"),
                    right_bgr=right_frame.to_ndarray(format="bgr24"),
                    capture_ts_left_ns=int(left_record.capture_ts_ns),
                    capture_ts_right_ns=int(right_record.capture_ts_ns),
                    arrival_ts_left_ns=int(left_record.arrival_ts_ns),
                    arrival_ts_right_ns=int(right_record.arrival_ts_ns),
                )
                yielded += 1
                if limit is not None and yielded >= limit:
                    return
            else:
                counters["device_ts_mismatch"] += 1
        while len(pending) > _PENDING_PAIR_LIMIT:
            oldest = next(iter(pending))
            del pending[oldest]
            counters["pending_pair_drops"] += 1
    counters["pending_unpaired"] = sum(len(entry) for entry in pending.values())


def _sample_decode(path, offset, *, max_camera_packets, max_pairs):
    """从指定偏移抽样解码若干双目包，只统计是否可解码/可配对（不保留图像）。"""
    import av

    result = {
        "start_offset": int(offset),
        "camera_packets": 0,
        "decoded_frames": {"4": 0, "5": 0},
        "pairs": 0,
        "skipped_decode_packets": 0,
        "decode_errors": 0,
        "error": None,
    }
    decoders = {sink: _create_decoder() for sink in CAMERA_SINKS}
    pending = {}
    try:
        for record in _iter_records_from(path, offset):
            if record.sink not in CAMERA_SINKS:
                continue
            result["camera_packets"] += 1
            if result["camera_packets"] > max_camera_packets:
                break
            try:
                frames = decoders[record.sink].decode(av.Packet(record.payload))
            except av.error.FFmpegError:
                result["decode_errors"] += 1
                continue
            if len(frames) != 1:
                result["skipped_decode_packets"] += 1
                continue
            result["decoded_frames"][str(record.sink)] += 1
            entry = pending.setdefault(record.sequence, {})
            entry[record.sink] = record
            if 4 in entry and 5 in entry:
                pair = pending.pop(record.sequence)
                if pair[4].device_ts == pair[5].device_ts:
                    result["pairs"] += 1
                    if result["pairs"] >= max_pairs:
                        break
            while len(pending) > _PENDING_PAIR_LIMIT:
                del pending[next(iter(pending))]
    except (RecordingFormatError, OSError) as exc:
        result["error"] = str(exc)
    return result


# ---------------------------------------------------------------- 回放校验

def _write_json_atomic(path, data):
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(
        json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2),
        encoding="utf-8",
    )
    os.replace(temp, path)


def _empty_report(root):
    return {
        "replay_ready": False,
        "reason": None,
        "error": None,
        "generated_time_ns": time.time_ns(),
        "elapsed_s": 0.0,
        "recording": str(root),
        "recording_status": None,
        "frames": {},
        "imu": {},
        "checks": [],
        "warnings": [],
        "errors": [],
    }


def build_replay_check(root):
    """只读快检录制目录能否离线回放；写 <root>/replay_check.json 并返回摘要。

    不整段解码：以 sdk_index 做双目配对/时间戳校验，抽取开头与中部样本解码。
    任何校验异常只记录在 error/errors 字段，不抛出（供 recorder.close 集成）。
    """
    root = Path(root)
    started = time.perf_counter()
    report = _empty_report(root)
    try:
        _fill_replay_check(root, report)
    except Exception as exc:  # noqa: BLE001 - 校验器自身故障不能影响录制关闭
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["errors"].append(f"校验过程异常：{exc}")
    report["elapsed_s"] = round(time.perf_counter() - started, 3)
    if report["errors"]:
        report["replay_ready"] = False
        report["reason"] = report["reason"] or report["errors"][0]
    else:
        report["replay_ready"] = True
        report["reason"] = None
    try:
        _write_json_atomic(root / "replay_check.json", report)
    except OSError as exc:
        report["write_error"] = str(exc)
    return report


def _fill_replay_check(root, report):
    checks = report["checks"]
    warnings = report["warnings"]
    errors = report["errors"]

    def check(name, ok, detail):
        checks.append({"name": name, "ok": bool(ok), "detail": str(detail)})
        if not ok:
            errors.append(f"{name}：{detail}")
        return ok

    def warn(message):
        warnings.append(str(message))

    manifest_path = root / "manifest.json"
    if not manifest_path.is_file():
        check("manifest", False, "缺少 manifest.json，无法确认录制状态")
        return
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        check("manifest", False, f"manifest.json 无法解析：{exc}")
        return
    if not isinstance(manifest, dict):
        check("manifest", False, "manifest.json 不是 JSON 对象")
        return

    status = manifest.get("recording_status")
    report["recording_status"] = status
    if status == "recording":
        check("录制状态", False, "录制尚未结束（recording_status=recording），停止后再校验")
    elif status == "incomplete":
        check("录制状态", False, f"录制不完整：{manifest.get('incomplete_reason') or '未给出原因'}")
    elif status in ("complete", "transport_unverified"):
        check("录制状态", True, f"recording_status={status}")
    else:
        warn(f"未知 recording_status：{status!r}")

    sdk_path = _recording_file(root, manifest, "sdk_records", "raw/sdk_records.bin")
    imu_path = _recording_file(root, manifest, "imu_records", "raw/imu_records.bin")
    index_path = _recording_file(root, manifest, "sdk_index", "raw/sdk_index.jsonl")

    sdk_size = 0
    sdk_ok = False
    if not sdk_path.is_file():
        check("原始 SDK 记录", False, f"缺少 {sdk_path.name}")
    elif sdk_path.stat().st_size == 0:
        check("原始 SDK 记录", False, "sdk_records.bin 为空")
    else:
        sdk_ok = True
        sdk_size = sdk_path.stat().st_size
        expected_size = manifest.get("sdk_file_bytes")
        if expected_size is not None and int(expected_size) != sdk_size:
            check(
                "SDK 记录文件大小", False,
                f"manifest 记录 {int(expected_size)} B，实际 {sdk_size} B",
            )
        else:
            check("原始 SDK 记录", True, f"{sdk_size} B")

    # --- 索引：4/5 配对与时间戳（不整段解码）
    camera_rows = []
    index_counts = {}
    total_rows = 0
    if not index_path.is_file():
        check("SDK 索引", False, f"缺少 {index_path.name}")
    else:
        try:
            for row in iter_index_rows(root):
                total_rows += 1
                try:
                    sink = int(row.get("sink"))
                except (TypeError, ValueError) as exc:
                    raise RecordingFormatError(f"索引第 {total_rows} 行缺少合法 sink") from exc
                key = str(sink)
                index_counts[key] = index_counts.get(key, 0) + 1
                if sink in CAMERA_SINKS:
                    try:
                        camera_rows.append({
                            "sink": sink,
                            "offset": int(row["offset"]),
                            "record_bytes": int(row.get("record_bytes", 0)),
                            "sequence": int(row["sequence"]),
                            "device_ts": int(row.get("device_timestamp_raw", 0)),
                        })
                    except (KeyError, TypeError, ValueError) as exc:
                        raise RecordingFormatError(f"索引第 {total_rows} 行字段非法") from exc
            check("SDK 索引", True, f"{total_rows} 行")
        except (RecordingFormatError, OSError) as exc:
            check("SDK 索引", False, f"sdk_index.jsonl 无法解析：{exc}")

        # 容错：索引尾部可能被截断（崩溃/强杀），回放本身用 sdk_records.bin，仍可尝试。
        manifest_counts = manifest.get("sdk_count_by_sink") or {}
        if isinstance(manifest_counts, dict):
            for sink_key, expected in manifest_counts.items():
                actual = index_counts.get(str(sink_key), 0)
                try:
                    expected = int(expected)
                except (TypeError, ValueError):
                    continue
                if actual != expected:
                    warn(
                        f"索引 sink {sink_key} 行数 {actual} 与 manifest {expected} 不一致"
                        "（索引尾部可能被截断）"
                    )
        manifest_total = manifest.get("sdk_count")
        if manifest_total is not None:
            try:
                if total_rows != int(manifest_total):
                    warn(f"索引总行数 {total_rows} 与 manifest sdk_count {int(manifest_total)} 不一致")
            except (TypeError, ValueError):
                pass

    frames_report = {
        "index_camera_records": len(camera_rows),
        "index_pairs": 0,
        "index_unpaired_packets": 0,
        "index_device_ts_mismatch": 0,
        "paired_device_ts_first": None,
        "paired_device_ts_last": None,
        "paired_device_ts_delta_median_ms": None,
        "paired_device_ts_delta_max_ms": None,
        "device_ts_gap_count": 0,
        "device_ts_gap_threshold_ms": None,
        "device_ts_gap_max_ms": None,
        "beginning_sample": None,
        "middle_sample": None,
    }
    report["frames"] = frames_report

    paired_sequences = []
    pair_state = {}
    device_ts_mismatch = 0
    for row in camera_rows:
        sequence = row["sequence"]
        entry = pair_state.setdefault(sequence, {})
        entry[row["sink"]] = row
        if 4 in entry and 5 in entry:
            pair = pair_state.pop(sequence)
            if pair[4]["device_ts"] == pair[5]["device_ts"]:
                paired_sequences.append((sequence, pair[4]["device_ts"]))
            else:
                device_ts_mismatch += 1
    unpaired_packets = sum(len(entry) for entry in pair_state.values())
    frames_report["index_pairs"] = len(paired_sequences)
    frames_report["index_unpaired_packets"] = unpaired_packets
    frames_report["index_device_ts_mismatch"] = device_ts_mismatch

    if not camera_rows:
        check("双目配对", False, "索引中没有 sink4/5 双目记录")
    elif not paired_sequences:
        check("双目配对", False, "索引中没有可配对的双目记录（左右 sequence/device_ts 未对齐）")
    elif device_ts_mismatch:
        check(
            "双目配对", False,
            f"{device_ts_mismatch} 个 sequence 的左右 device_ts 不一致",
        )
    else:
        check("双目配对", True, f"{len(paired_sequences)} 对")
    if unpaired_packets:
        warn(f"{unpaired_packets} 个双目包未配对（常见于录制开始/结束边界，回放会自动跳过）")

    if paired_sequences:
        timestamps = [device_ts for _, device_ts in paired_sequences]
        frames_report["paired_device_ts_first"] = timestamps[0]
        frames_report["paired_device_ts_last"] = timestamps[-1]
        if timestamps[0] <= 0:
            check("配对帧 device_ts", False, f"配对帧 device_ts 非正：{timestamps[0]}")
        non_monotonic = sum(
            1 for previous, current in zip(timestamps, timestamps[1:]) if current <= previous
        )
        if non_monotonic:
            check("配对帧 device_ts", False, f"{non_monotonic} 处 device_ts 非递增")
        elif timestamps[0] > 0:
            check(
                "配对帧 device_ts", True,
                f"{timestamps[0]} → {timestamps[-1]}（{len(timestamps)} 对，严格递增）",
            )

        # 相邻配对帧间隔（device_ts 按回放约定 ×1000 换算 ns），识别暂停/丢包缺口。
        # 缺口不影响 replay_ready，但回放在缺口处可能因 IMU 样本不足而丢帧，必须让用户知情。
        deltas_ns = [
            (int(current) - int(previous)) * 1000
            for previous, current in zip(timestamps, timestamps[1:])
        ]
        median_delta_ns = statistics.median(deltas_ns) if deltas_ns else None
        max_delta_ns = max(deltas_ns) if deltas_ns else None
        threshold_ns = (
            max(_FRAME_GAP_MIN_NS, _FRAME_GAP_MEDIAN_FACTOR * median_delta_ns)
            if median_delta_ns is not None else None
        )
        gap_deltas_ns = [
            delta for delta in deltas_ns
            if threshold_ns is not None and delta > threshold_ns
        ]
        frames_report["paired_device_ts_delta_median_ms"] = (
            round(median_delta_ns / 1e6, 3) if median_delta_ns is not None else None
        )
        frames_report["paired_device_ts_delta_max_ms"] = (
            round(max_delta_ns / 1e6, 3) if max_delta_ns is not None else None
        )
        frames_report["device_ts_gap_count"] = len(gap_deltas_ns)
        frames_report["device_ts_gap_threshold_ms"] = (
            round(threshold_ns / 1e6, 3) if threshold_ns is not None else None
        )
        frames_report["device_ts_gap_max_ms"] = (
            round(max(gap_deltas_ns) / 1e6, 3) if gap_deltas_ns else None
        )
        if non_monotonic or timestamps[0] <= 0:
            pass  # 时间戳已被判为异常，不再输出误导性的连续性结论
        elif gap_deltas_ns:
            gap_warning = (
                f"录制包含暂停/丢包缺口（{len(gap_deltas_ns)} 处，最大 "
                f"{max(gap_deltas_ns) / 1e6:.0f} ms），回放可能在缺口处报 IMU 样本不足；"
                "VIO 回放建议连续录制"
            )
            warn(gap_warning)
            checks.append({"name": "帧时间连续性", "ok": True, "detail": gap_warning})
        elif deltas_ns:
            checks.append({
                "name": "帧时间连续性",
                "ok": True,
                "detail": (
                    f"相邻帧间隔中位数 {median_delta_ns / 1e6:.1f} ms，"
                    f"无超过 {threshold_ns / 1e6:.0f} ms 的缺口"
                ),
            })

    if sdk_ok and camera_rows:
        max_end = max(row["offset"] + row["record_bytes"] for row in camera_rows)
        if max_end > sdk_size:
            check("索引偏移", False, f"索引记录结束于 {max_end} B，超出文件大小 {sdk_size} B")
        else:
            check("索引偏移", True, f"索引记录均位于 {sdk_size} B 文件内")

    # --- IMU：行数、采样率（相邻行增量中位数）、sidecar 与索引一致性
    imu_report = {
        "packets": None,
        "rows": 0,
        "rate_hz": None,
        "rate_method": "median_adjacent_delta_excluding_gaps>100ms",
        "gap_count": 0,
        "first_time_a_ns": None,
        "last_time_a_ns": None,
        "span_s": None,
    }
    report["imu"] = imu_report
    index_imu_packets = index_counts.get(str(IMU_SINK))
    manifest_imu_packets = (manifest.get("sdk_count_by_sink") or {}).get(str(IMU_SINK))
    if index_imu_packets is not None:
        imu_report["packets"] = index_imu_packets
    elif manifest_imu_packets is not None:
        try:
            imu_report["packets"] = int(manifest_imu_packets)
        except (TypeError, ValueError):
            pass

    if not imu_path.is_file():
        check("IMU 记录", False, f"缺少 {imu_path.name}")
    elif imu_path.stat().st_size == 0:
        check("IMU 记录", False, "imu_records.bin 为空")
    else:
        try:
            row_count = 0
            first_time = last_time = None
            imu_deltas_ns = []
            for time_a, _time_b, _values in iter_imu_rows(root):
                row_count += 1
                if first_time is None:
                    first_time = time_a
                if last_time is not None:
                    delta = int(time_a) - int(last_time)
                    if delta <= 0:
                        raise RecordingFormatError("IMU 时间非递增")
                    imu_deltas_ns.append(delta)
                last_time = time_a
        except (RecordingFormatError, OSError) as exc:
            check("IMU 载荷", False, f"IMU 载荷解析失败：{exc}")
        else:
            imu_report["rows"] = row_count
            imu_report["first_time_a_ns"] = first_time
            imu_report["last_time_a_ns"] = last_time
            if not row_count:
                check("IMU 载荷", False, "IMU 记录中没有数据行")
            else:
                # SDK 单包行数可变（实测 11–16 行），这里只要求每行 40 B 对齐；
                # iter_imu_rows 已对非 40 B 倍数/非递增显式报错，不能再按 16 行假设告警。
                check(
                    "IMU 载荷", True,
                    f"{row_count} 行，全部载荷按 {IMU_PAYLOAD_ROW_BYTES} B 对齐",
                )
                if row_count < 2 or first_time is None or last_time is None:
                    check("IMU 速率", False, f"IMU 样本不足（{row_count} 行），无法估计采样率")
                else:
                    span_ns = int(last_time) - int(first_time)
                    imu_report["span_s"] = round(span_ns / 1e9, 6)
                    # 暂停期间 AsyncRecorder 不接收记录，会留下 >100 ms 的行间增量；
                    # 总跨度/行数口径会把缺口算进速率（实测 452.6 Hz 但真实约 480 Hz），
                    # 因此用剔除缺口后的相邻行增量中位数估计采样率。
                    plausible = [
                        delta for delta in imu_deltas_ns
                        if 0 < delta <= _IMU_RATE_MAX_DELTA_NS
                    ]
                    imu_report["gap_count"] = len(imu_deltas_ns) - len(plausible)
                    used = plausible or [delta for delta in imu_deltas_ns if delta > 0]
                    if not used:
                        check("IMU 速率", False, "IMU 行间增量为非正，无法估计采样率")
                    else:
                        rate = 1e9 / statistics.median(used)
                        imu_report["rate_hz"] = round(rate, 3)
                        gap_note = (
                            f"；已剔除 {imu_report['gap_count']} 处 >100 ms 缺口"
                            if imu_report["gap_count"] else ""
                        )
                        check(
                            "IMU 速率", True,
                            f"{row_count} 行，约 {rate:.1f} Hz（相邻行增量中位数{gap_note}）",
                        )
                        if not (100.0 <= rate <= 2000.0):
                            warn(f"IMU 估计采样率 {rate:.1f} Hz 偏离常见 480 Hz，请核对载荷语义")
        if index_imu_packets is not None:
            try:
                sidecar_records = sum(1 for _ in iter_imu_payloads(root))
            except (RecordingFormatError, OSError) as exc:
                warn(f"IMU sidecar 记录统计失败：{exc}")
            else:
                imu_report["sidecar_records"] = sidecar_records
                if sidecar_records != int(index_imu_packets):
                    warn(
                        f"IMU sidecar 记录数 {sidecar_records} 与索引 sink7 行数 "
                        f"{int(index_imu_packets)} 不一致"
                    )

    # --- 抽样解码：开头 + 中部各一小段，不整段解码
    if sdk_ok and camera_rows:
        beginning_row = camera_rows[0]
        try:
            beginning_record = read_record_at(sdk_path, beginning_row["offset"])
        except (RecordingFormatError, OSError) as exc:
            check("开头样本解码", False, f"索引首条记录读取失败：{exc}")
        else:
            if (
                beginning_record.sink != beginning_row["sink"]
                or beginning_record.sequence != beginning_row["sequence"]
                or beginning_record.device_ts != beginning_row["device_ts"]
            ):
                check("开头样本解码", False, "索引首条记录与 sdk_records.bin 偏移处内容不一致")
            else:
                beginning = _sample_decode(
                    sdk_path, beginning_row["offset"],
                    max_camera_packets=_SAMPLE_BEGINNING_PACKETS, max_pairs=3,
                )
                frames_report["beginning_sample"] = beginning
                if beginning["error"]:
                    check("开头样本解码", False, f"抽样解码失败：{beginning['error']}")
                elif beginning["pairs"]:
                    check("开头样本解码", True, f"解码出 {beginning['pairs']} 对双目帧")
                else:
                    check(
                        "开头样本解码", False,
                        "抽样未能解码出成对双目帧"
                        f"（0/2 帧跳过 {beginning['skipped_decode_packets']}，"
                        f"解码错误 {beginning['decode_errors']}）",
                    )

        middle_row = camera_rows[len(camera_rows) // 2]
        try:
            middle_record = read_record_at(sdk_path, middle_row["offset"])
        except (RecordingFormatError, OSError) as exc:
            warn(f"中部样本索引读取失败：{exc}")
        else:
            if (
                middle_record.sink != middle_row["sink"]
                or middle_record.sequence != middle_row["sequence"]
                or middle_record.device_ts != middle_row["device_ts"]
            ):
                warn("中部样本索引记录与 sdk_records.bin 偏移处内容不一致")
            else:
                middle = _sample_decode(
                    sdk_path, middle_row["offset"],
                    max_camera_packets=_SAMPLE_MIDDLE_PACKETS, max_pairs=2,
                )
                frames_report["middle_sample"] = middle
                if middle["error"]:
                    warn(f"中部抽样解码失败：{middle['error']}")
                elif middle["pairs"]:
                    check("中部样本解码", True, f"解码出 {middle['pairs']} 对双目帧")
                else:
                    warn("中部抽样未解码出成对帧（可能从 GOP 中间开始），不作为失败依据")

    checks.append({
        "name": "总体结论",
        "ok": not errors,
        "detail": "可回放" if not errors else (errors[0] if errors else ""),
    })
