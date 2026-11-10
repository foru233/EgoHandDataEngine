"""只读检查 VS030 SDK v2 原始录制、双路 H.265 解码和 IMU sidecar 完整性。"""
import argparse
import hashlib
import json
import struct
from collections import Counter
from pathlib import Path

import av

HEADER = struct.Struct("<IHHIIQQQQQQ")
MAGIC = 0x58444652
VERSION = 2
MAX_PAYLOAD = 8 * 1024 * 1024


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as file_obj:
        for chunk in iter(lambda: file_obj.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def scan_records(path, *, decode_video=False):
    counts = Counter()
    decoded = Counter()
    payload_bytes = Counter()
    errors = []
    decoders = {4: av.CodecContext.create("hevc", "r"), 5: av.CodecContext.create("hevc", "r")} if decode_video else {}
    previous_sequence = {}
    previous_device_ts = {}
    offsets = []
    with Path(path).open("rb") as file_obj:
        while True:
            offset = file_obj.tell()
            raw_header = file_obj.read(HEADER.size)
            if not raw_header:
                break
            if len(raw_header) != HEADER.size:
                errors.append(f"truncated header at offset {offset}")
                break
            magic, version, header_size, sink, size, seq, device_ts, capture_ts, arrival_ts, host_mono, host_unix = HEADER.unpack(raw_header)
            if magic != MAGIC or version != VERSION or header_size != HEADER.size or size > MAX_PAYLOAD:
                errors.append(f"invalid header at offset {offset}")
                break
            payload = file_obj.read(size)
            if len(payload) != size:
                errors.append(f"truncated payload at offset {offset}")
                break
            counts[sink] += 1
            payload_bytes[sink] += size
            offsets.append({
                "offset": offset, "sink": sink, "sequence": seq,
                "device_timestamp_raw": device_ts, "payload_bytes": size,
            })
            if sink in previous_sequence and seq <= previous_sequence[sink]:
                errors.append(f"non-monotonic sequence sink {sink}")
            if sink in previous_device_ts and device_ts < previous_device_ts[sink]:
                errors.append(f"non-monotonic device timestamp sink {sink}")
            previous_sequence[sink] = seq
            previous_device_ts[sink] = device_ts
            if sink in decoders:
                try:
                    decoded[sink] += len(decoders[sink].decode(av.Packet(payload)))
                except av.error.FFmpegError:
                    # A session may begin mid-GOP; this is a decode diagnostic, not a transport verdict.
                    pass
    return {
        "counts": dict(counts),
        "decoded_frames": dict(decoded),
        "payload_bytes_by_sink": dict(payload_bytes),
        "records": offsets,
        "errors": errors,
    }


def validate(directory):
    directory = Path(directory)
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf8"))
    sdk_path = directory / manifest["sdk_records"]
    scanned = scan_records(sdk_path, decode_video=True)
    errors = list(scanned["errors"])

    if sum(scanned["counts"].values()) != manifest.get("sdk_count"):
        errors.append("record count mismatch")
    if {str(k): v for k, v in scanned["counts"].items()} != manifest.get("sdk_count_by_sink", {}):
        errors.append("per-sink count mismatch")
    if sdk_path.stat().st_size != manifest.get("sdk_file_bytes"):
        errors.append("SDK file size mismatch")
    if sha256(sdk_path) != manifest.get("sdk_sha256"):
        errors.append("SDK SHA256 mismatch")

    index_path = directory / manifest["sdk_index"]
    index_rows = [json.loads(line) for line in index_path.read_text(encoding="utf8").splitlines() if line.strip()]
    if len(index_rows) != sum(scanned["counts"].values()):
        errors.append("SDK index row count mismatch")
    else:
        for row, scanned_row in zip(index_rows, scanned["records"]):
            if row["offset"] != scanned_row["offset"] or row["sink"] != scanned_row["sink"]:
                errors.append(f"SDK index mismatch at offset {scanned_row['offset']}")
                break

    imu_path = directory / manifest["imu_records"]
    imu_scanned = scan_records(imu_path, decode_video=False)
    if imu_scanned["errors"]:
        errors.extend("IMU sidecar: " + e for e in imu_scanned["errors"])
    if imu_scanned["counts"].get(7, 0) != scanned["counts"].get(7, 0):
        errors.append("IMU sidecar record count mismatch")
    if sha256(imu_path) != manifest.get("imu_sha256"):
        errors.append("IMU sidecar SHA256 mismatch")

    return {
        "manifest_status": manifest.get("recording_status"),
        "sdk_records_by_sink": scanned["counts"],
        "decoded_video_frames_by_sink": scanned["decoded_frames"],
        "sdk_payload_bytes_by_sink": scanned["payload_bytes_by_sink"],
        "imu_sidecar_records": imu_scanned["counts"].get(7, 0),
        "frame_index_count": manifest.get("frame_index_count"),
        "errors": errors,
        "capture_stats": manifest.get("capture_stats"),
        "note": (
            "本工具验证文件结构、索引、哈希及可解码性；不能单凭文件证明 USB/SDK 传输零丢包，"
            "也不把设备时间戳或 SDK capture/arrival 时间戳解释为已验证的曝光时间。"
        ),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("directory")
    args = parser.parse_args()
    result = validate(args.directory)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result["errors"]:
        raise SystemExit(1)
