"""Read-only audit of VS030 transport, stored clocks and VIO configuration snapshots.

Does not claim SDK timestamps are exposure times or infer hardware synchronization.
Writes only the explicitly selected report outside the source recording.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from egohand.storage.sdk_recording import iter_sdk_records, IMU_PAYLOAD_ROW
from egohand.vio.imu_lattice import PacketLatticeImuBuffer, packet_lattice_phase_ns
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def describe(values):
    array = np.asarray(values, dtype=float)
    if not array.size:
        return None
    return {"count": len(array), "min": float(array.min()), "p50": float(np.median(array)),
            "p95": float(np.percentile(array, 95)), "max": float(array.max())}


def audit(recording):
    recording = Path(recording).resolve()
    manifest = json.loads((recording / "manifest.json").read_text(encoding="utf8"))
    timing = yaml.safe_load((recording / "vio_timing.yaml").read_text(encoding="utf8"))
    streams = {sink: [] for sink in (4, 5, 7)}
    raw_imu = []
    lattice_times = []
    imu_sizes = Counter()
    buffer = PacketLatticeImuBuffer(sample_rate_hz=timing["imu_rate_hz"],
        camera_to_imu_drift_ppm=timing.get("camera_to_imu_drift_ppm", 0),
        phase_ns=packet_lattice_phase_ns(timing))
    # Read every original record, checking all binary headers and payload lengths.
    for record in iter_sdk_records(recording / manifest.get("sdk_records", "raw/sdk_records.bin")):
        if record.sink in streams:
            streams[record.sink].append({"sequence": record.sequence, "device_ts": record.device_ts,
                "capture": record.capture_ts_ns, "arrival": record.arrival_ts_ns,
                "host": record.host_monotonic_ns})
        if record.sink == 7:
            if len(record.payload) % IMU_PAYLOAD_ROW.size:
                raise ValueError("IMU payload is not aligned")
            imu_sizes[len(record.payload) // IMU_PAYLOAD_ROW.size] += 1
            raw_imu.extend(IMU_PAYLOAD_ROW.unpack_from(record.payload, offset)
                for offset in range(0, len(record.payload), IMU_PAYLOAD_ROW.size))
            buffer.add(record.payload, record.device_ts)
            lattice_times.extend(row[0] for row in buffer.rows)
            buffer.rows.clear()
    camera = {sink: {row["sequence"]: row for row in streams[sink]} for sink in (4, 5)}
    keys = sorted(camera[4].keys() & camera[5].keys())
    pairs = [(camera[4][key], camera[5][key]) for key in keys]
    valid_pairs = [(a, b) for a, b in pairs if a["device_ts"] == b["device_ts"]]
    stamps = np.array([a["device_ts"] * 1000 for a, _ in valid_pairs], dtype=np.int64)
    raw = np.asarray(raw_imu)
    report = {"schema_version": 1, "recording": str(recording),
        "scope": "All SDK headers and payloads read; no full image decode or hardware truth",
        "recording_status": manifest.get("recording_status"),
        "hashes": {}, "streams": {}, "imu": {}, "snapshot_geometry": {}}
    for name in ["manifest.json", "calibration.yaml", "kalibr_camchain.yaml", "vio_settings.yaml",
                 "vio_quality.yaml", "vio_timing.yaml", "raw/sdk_records.bin", "raw/imu_records.bin"]:
        path = recording / name
        if path.is_file():
            report["hashes"][name] = {"sha256": sha256(path), "bytes": path.stat().st_size}
    for sink, rows in streams.items():
        seq = np.array([row["sequence"] for row in rows], dtype=np.int64)
        ts = np.array([row["device_ts"] for row in rows], dtype=np.int64)
        report["streams"][str(sink)] = {"packets": len(rows),
            "duplicate_sequence_count": len(seq) - len(set(seq)),
            "nonincreasing_sequence_count": int(np.sum(np.diff(seq) <= 0)),
            "sequence_steps": dict(Counter(map(int, np.diff(seq)))),
            "nonincreasing_device_ts_count": int(np.sum(np.diff(ts) <= 0)),
            "device_ts_interval_raw": describe(np.diff(ts))}
    report["camera"] = {"paired_sequences": len(pairs), "equal_device_ts_pairs": len(valid_pairs),
        "unpaired_packets": len(streams[4]) + len(streams[5]) - 2 * len(pairs),
        "frame_interval_ms_assuming_device_us": describe(np.diff(stamps) / 1e6),
        "duration_s_assuming_device_us": float((stamps[-1] - stamps[0]) / 1e9),
        "capture_right_minus_left_ms": describe([(b["capture"] - a["capture"]) / 1e6 for a, b in valid_pairs]),
        "arrival_right_minus_left_ms": describe([(b["arrival"] - a["arrival"]) / 1e6 for a, b in valid_pairs]),
        "warning": "Equal sequence/device tags are not proof of equal exposure time. SDK capture/arrival clocks are unverified."}
    lattice_times = np.asarray(lattice_times, dtype=np.int64)
    counts = np.diff(np.searchsorted(lattice_times, stamps, side="right"))
    covered = (stamps[:-1] >= lattice_times[0]) & (stamps[1:] <= buffer.covered_until_ns)
    report["imu"] = {"packets": len(streams[7]), "rows": len(raw),
        "samples_per_packet": dict(imu_sizes), "finite_values": bool(np.isfinite(raw).all()),
        "raw_time_a_delta_ms": describe(np.diff(raw[:, 0]) / 1e6),
        "raw_time_b_delta_ms": describe(np.diff(raw[:, 1]) / 1e6),
        "raw_time_a_nonincreasing": int(np.sum(np.diff(raw[:, 0]) <= 0)),
        "raw_time_b_nonincreasing": int(np.sum(np.diff(raw[:, 1]) <= 0)),
        "raw_time_a_gaps_over_30ms": int(np.sum(np.diff(raw[:, 0]) > 30e6)),
        "lattice_sample_interval_ms": describe(np.diff(lattice_times) / 1e6),
        "lattice_gap_events": buffer.gap_events,
        "covered_camera_intervals": int(np.sum(covered)),
        "leading_camera_frames_before_first_lattice_sample": int(np.sum(stamps < lattice_times[0])),
        "trailing_camera_frames_after_lattice_coverage": int(np.sum(stamps > buffer.covered_until_ns)),
        "samples_per_covered_camera_interval": dict(Counter(map(int, counts[covered]))),
        "accel_norm_candidate_mps2": describe(np.linalg.norm(raw[:, 2:5], axis=1)),
        "gyro_norm_candidate_rps": describe(np.linalg.norm(raw[:, 5:8], axis=1)),
        "timing": timing, "legacy_lag_applied_in_lattice": False}
    geometry = prepare_vs030_vio_geometry(recording / "kalibr_camchain.yaml")
    scale = float(timing["image_scale"])
    fs = cv2.FileStorage(str(recording / "vio_settings.yaml"), cv2.FILE_STORAGE_READ)
    try:
        observed = {key: float(fs.getNode(key).real()) for key in
            ["Camera1.fx", "Camera1.fy", "Camera1.cx", "Camera1.cy", "Camera.width", "Camera.height", "Stereo.b"]}
        T = fs.getNode("IMU.T_b_c1").mat()
    finally:
        fs.release()
    expected = {key: float(geometry.rectifier.P1[i, j] * scale) for key, i, j in
        [("Camera1.fx", 0, 0), ("Camera1.fy", 1, 1), ("Camera1.cx", 0, 2), ("Camera1.cy", 1, 2)]}
    report["snapshot_geometry"] = {"orb_left": "device channel5/cam1", "gui_left": "device channel4/cam0",
        "image_scale": scale, "expected_scaled_intrinsics": expected, "orb_intrinsics": observed,
        "max_intrinsic_error_px": max(abs(expected[k] - observed[k]) for k in expected),
        "imu_extrinsic_max_error": float(np.max(np.abs(T - geometry.T_imu_from_orb_left))),
        "warning": "Internal consistency only; not independent calibration accuracy."}
    report["manifest_hash_match"] = {"sdk": report["hashes"]["raw/sdk_records.bin"]["sha256"] == manifest.get("sdk_sha256"),
        "imu": report["hashes"]["raw/imu_records.bin"]["sha256"] == manifest.get("imu_sha256")}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.recording.resolve()):
        parser.error("Report must be outside the original recording")
    report = audit(args.recording)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf8")
    print(json.dumps({"output": str(args.output), "camera": report["camera"],
        "geometry": report["snapshot_geometry"], "hash_match": report["manifest_hash_match"]}, ensure_ascii=True))


if __name__ == "__main__":
    main()
