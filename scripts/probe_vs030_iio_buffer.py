"""短时验证 VS030 LSM6DSV16X IIO buffered IMU 流；结束时恢复原配置。"""
from __future__ import annotations
import json, struct, subprocess, time
from pathlib import Path
import numpy as np

ROOT = Path(__import__('pathlib').Path(__file__).resolve().parents[1])
ADB = ROOT / "tools/android/platform-tools/adb.exe"
SERIAL = "<device-serial>"
OUT = ROOT / "results/camera/vs030_iio_buffer_probe.json"
DEVICES = {
    "gyro": {
        "index": 1, "channels": ("in_anglvel_x", "in_anglvel_y", "in_anglvel_z"),
        "raw": ("in_anglvel_x_raw", "in_anglvel_y_raw", "in_anglvel_z_raw"),
        "scale": "in_anglvel_x_scale", "unit": "rad_s",
    },
    "accel": {
        "index": 2, "channels": ("in_accel_x", "in_accel_y", "in_accel_z"),
        "raw": ("in_accel_x_raw", "in_accel_y_raw", "in_accel_z_raw"),
        "scale": "in_accel_x_scale", "unit": "m_s2",
    },
}

def adb(*args, check=True, binary=False):
    p = subprocess.run([str(ADB), "-s", SERIAL, *map(str, args)],
                       capture_output=True, text=not binary, timeout=8)
    if check and p.returncode:
        raise RuntimeError(f"ADB failed: {args!r}: {p.stderr if not binary else p.returncode}")
    return p.stdout if binary else p.stdout.strip()

def shell(command):
    return adb("shell", command)

def read_text(path):
    return shell(f"cat {path}")

def write_text(path, value):
    shell(f"echo {value} > {path}")

def parse_records(data):
    # IIO scan layout: s16 x/y/z, 2-byte alignment, then signed 64-bit timestamp.
    record_size = 16
    n = len(data) // record_size
    rows = []
    for i in range(n):
        x, y, z, ts = struct.unpack_from("<hhhxxq", data, i * record_size)
        rows.append((ts, x, y, z))
    return rows, len(data) % record_size

def main():
    if not ADB.is_file():
        raise FileNotFoundError(ADB)
    adb("get-state")
    saved = {}
    report = {"device": "VS030 / LSM6DSV16X", "requested_odr_hz": 480,
              "timestamp_clock": {}, "streams": {}, "pairing": {}, "restored": False}
    try:
        # Record every touched sysfs value so the probe can restore the camera.
        for name, spec in DEVICES.items():
            idx = spec["index"]
            base = f"/sys/bus/iio/devices/iio:device{idx}"
            saved[name] = {
                "sampling_frequency": read_text(f"{base}/sampling_frequency"),
                "buffer_enable": read_text(f"{base}/buffer0/enable"),
                "buffer_length": read_text(f"{base}/buffer0/length"),
                "buffer_watermark": read_text(f"{base}/buffer0/watermark"),
                "scan": {ch: read_text(f"{base}/scan_elements/{ch}_en") for ch in spec["channels"]},
                "timestamp_scan": read_text(f"{base}/scan_elements/in_timestamp_en"),
            }
            report["timestamp_clock"][name] = read_text(f"{base}/current_timestamp_clock")
            if saved[name]["buffer_enable"] != "0":
                raise RuntimeError(f"{name} IIO buffer was already enabled; refusing to change it")

        # Configure identical rates and timestamped XYZ records.
        for name, spec in DEVICES.items():
            idx = spec["index"]
            base = f"/sys/bus/iio/devices/iio:device{idx}"
            write_text(f"{base}/sampling_frequency", 480)
            write_text(f"{base}/buffer0/length", 1024)
            write_text(f"{base}/buffer0/watermark", 1)
            for ch in spec["channels"]:
                write_text(f"{base}/scan_elements/{ch}_en", 1)
            write_text(f"{base}/scan_elements/in_timestamp_en", 1)

        # Enable both streams before starting readers.
        for spec in DEVICES.values():
            write_text(f"/sys/bus/iio/devices/iio:device{spec['index']}/buffer0/enable", 1)

        procs = {}
        for name, spec in DEVICES.items():
            idx = spec["index"]
            procs[name] = subprocess.Popen(
                [str(ADB), "-s", SERIAL, "exec-out", "timeout", "2", "cat", f"/dev/iio:device{idx}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        captured = {}
        for name, proc in procs.items():
            try:
                stdout, stderr = proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                stdout, stderr = proc.communicate()
            captured[name] = stdout or b""
            report["streams"][name] = {"return_code": proc.returncode,
                                       "bytes": len(captured[name]),
                                       "stderr": (stderr or b"").decode("utf-8", "replace")[-500:]}

        for name, spec in DEVICES.items():
            rows, tail = parse_records(captured[name])
            scale = float(read_text(f"/sys/bus/iio/devices/iio:device{spec['index']}/{spec['scale']}"))
            if not rows:
                report["streams"][name].update({"samples": 0, "tail_bytes": tail, "status": "NO_SAMPLES"})
                continue
            a = np.asarray(rows, dtype=np.int64)
            ts = a[:, 0]
            xyz = a[:, 1:4].astype(float) * scale
            dt = np.diff(ts).astype(float) / 1e6
            report["streams"][name].update({
                "samples": int(len(rows)), "tail_bytes": tail, "record_size_bytes": 16,
                "scale_si_per_lsb": scale, "timestamp_first": int(ts[0]), "timestamp_last": int(ts[-1]),
                "duration_s": float((ts[-1] - ts[0]) / 1e9),
                "dt_ms_median": float(np.median(dt)) if len(dt) else None,
                "dt_ms_p95": float(np.percentile(dt, 95)) if len(dt) else None,
                "strictly_increasing": bool(np.all(np.diff(ts) > 0)),
                "xyz_si_mean": xyz.mean(axis=0).tolist(),
                "xyz_si_std": xyz.std(axis=0).tolist(),
                "xyz_si_min": xyz.min(axis=0).tolist(),
                "xyz_si_max": xyz.max(axis=0).tolist(),
            })

        if all(report["streams"].get(k, {}).get("samples", 0) for k in DEVICES):
            g = np.asarray([r[0] for r in parse_records(captured["gyro"])[0]], dtype=np.int64)
            a = np.asarray([r[0] for r in parse_records(captured["accel"])[0]], dtype=np.int64)
            nearest = np.searchsorted(a, g)
            diffs = []
            for t, j in zip(g, nearest):
                candidates = [k for k in (j - 1, j) if 0 <= k < len(a)]
                if candidates:
                    diffs.append(int(a[min(candidates, key=lambda k: abs(int(a[k]) - int(t)))] - t))
            report["pairing"] = {
                "nearest_timestamp_delta_ns_median": int(np.median(diffs)) if diffs else None,
                "nearest_timestamp_delta_ns_p95_abs": int(np.percentile(np.abs(diffs), 95)) if diffs else None,
                "clock_domain": "device IIO CLOCK_REALTIME (system clock is not correctly epoch-synchronized)",
            }
    finally:
        # Always disable the temporary buffers first, then restore the original settings.
        for name, spec in DEVICES.items():
            idx = spec["index"]
            base = f"/sys/bus/iio/devices/iio:device{idx}"
            try: write_text(f"{base}/buffer0/enable", 0)
            except Exception: pass
        for name, spec in DEVICES.items():
            if name not in saved:
                continue
            idx = spec["index"]
            base = f"/sys/bus/iio/devices/iio:device{idx}"
            vals = saved[name]
            for ch, value in vals["scan"].items():
                try: write_text(f"{base}/scan_elements/{ch}_en", value)
                except Exception: pass
            try: write_text(f"{base}/scan_elements/in_timestamp_en", vals["timestamp_scan"])
            except Exception: pass
            for key, rel in (("buffer_length", "buffer0/length"), ("buffer_watermark", "buffer0/watermark"),
                             ("sampling_frequency", "sampling_frequency")):
                try: write_text(f"{base}/{rel}", vals[key])
                except Exception: pass
        report["restored"] = True
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print("REPORT", OUT)

if __name__ == "__main__":
    main()
