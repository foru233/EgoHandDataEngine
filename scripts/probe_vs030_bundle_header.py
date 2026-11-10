"""短时直接采集 VS030 UDP bundle 头部；不保存图像，只保存每包前 90 字节。"""
from __future__ import annotations
import json, struct, subprocess, time
from pathlib import Path
from egohand.sources.vendor_placeholder import ADB, CAMERA_SERIAL, VendorStereoSource

ROOT = Path(__import__('pathlib').Path(__file__).resolve().parents[1])
OUT = ROOT / "results/camera/vs030_bundle_header_probe.json"
PCAP = ROOT / "results/camera/vs030_bundle_headers_only.pcap"

def parse_pcap(data: bytes):
    if len(data) < 24 or data[:4] != b"\xd4\xc3\xb2\xa1":
        raise RuntimeError("tcpdump did not return a valid little-endian pcap")
    pos = 24
    rows = []
    while pos + 16 <= len(data):
        sec, usec, caplen, wirelen = struct.unpack_from("<IIII", data, pos)
        pos += 16
        if pos + caplen > len(data):
            break
        packet = data[pos:pos + caplen]
        pos += caplen
        if len(packet) < 42:
            continue
        ip = packet[14:]
        ihl = (ip[0] & 15) * 4
        if len(ip) < ihl + 8:
            continue
        payload = ip[ihl + 8:]
        if len(payload) < 8 or payload[:4] != b"\x90\x80\x55\xff":
            continue
        kind = struct.unpack_from("<I", payload, 4)[0]
        row = {"pcap_time": sec + usec / 1e6, "kind": kind, "payload_len_captured": len(payload)}
        if kind == 1 and len(payload) >= 48:
            row["header_hex"] = payload[:48].hex()
            row["u32"] = {str(off): struct.unpack_from("<I", payload, off)[0] for off in range(8, 48, 4)}
            row["u64"] = {str(off): struct.unpack_from("<Q", payload, off)[0] for off in range(8, 41, 4)}
        elif kind == 4 and len(payload) >= 16:
            row["channel"] = struct.unpack_from("<I", payload, 8)[0]
            row["declared_channel_bytes"] = struct.unpack_from("<I", payload, 12)[0]
            row["header_hex"] = payload[:16].hex()
        rows.append(row)
    return rows

def monotonic_candidates(headers):
    result = {}
    if len(headers) < 3:
        return result
    for off in range(8, 48, 4):
        vals = [struct.unpack_from("<I", bytes.fromhex(h["header_hex"]), off)[0] for h in headers]
        delta = [b - a for a, b in zip(vals, vals[1:])]
        if all(d > 0 for d in delta):
            result[f"u32@{off}"] = {"first": vals[0], "last": vals[-1],
                "median_delta": sorted(delta)[len(delta)//2], "unit_unknown": True}
    for off in (8, 16, 24, 32):
        vals = [struct.unpack_from("<Q", bytes.fromhex(h["header_hex"]), off)[0] for h in headers]
        delta = [b - a for a, b in zip(vals, vals[1:])]
        if all(d > 0 for d in delta):
            result[f"u64@{off}"] = {"first": vals[0], "last": vals[-1],
                "median_delta": sorted(delta)[len(delta)//2], "unit_unknown": True}
    return result

def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    if not ADB.is_file():
        raise FileNotFoundError(ADB)
    command = "timeout 8 tcpdump -i usb0 -U -s 90 -c 500 -w - udp src port 45679 and dst port 45680 2>/dev/null"
    pcap_file = PCAP.open("wb")
    capture = subprocess.Popen([str(ADB), "-s", CAMERA_SERIAL, "exec-out", command],
                               stdout=pcap_file, stderr=subprocess.PIPE)
    source = VendorStereoSource()
    frames = []
    error = None
    try:
        time.sleep(0.35)
        source.open()
        source.start()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and len(frames) < 45:
            frame = source.read()
            if frame is None:
                break
            frames.append({"index": frame.frame_id, "host_decode_completed_ns": frame.timestamp_ns,
                           "shape": list(frame.left_image.shape)})
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            source.close()
        except Exception as exc:
            error = error or f"source_close: {exc}"
        pcap_file.close()
    try:
        capture.wait(timeout=10)
    except subprocess.TimeoutExpired:
        capture.terminate()
        try: capture.wait(timeout=2)
        except subprocess.TimeoutExpired: capture.kill(); capture.wait()
    stderr = capture.stderr.read().decode("utf-8", "replace")[-1000:] if capture.stderr else ""
    raw = PCAP.read_bytes() if PCAP.exists() else b""
    packets = parse_pcap(raw) if raw else []
    headers = [x for x in packets if x["kind"] == 1]
    channels = [x for x in packets if x["kind"] == 4]
    report = {
        "device": "VS030 / RV1126B",
        "direct_source": "VendorStereoSource via ADB + USB UDP",
        "visionviewer_used": False,
        "frames_received": len(frames),
        "frame_shapes": sorted({tuple(x["shape"]) for x in frames}),
        "source_error": error,
        "tcpdump_return_code": capture.returncode,
        "pcap_bytes": len(raw),
        "captured_protocol_packets": len(packets),
        "bundle_start_count": len(headers),
        "channel_header_count": len(channels),
        "channel_ids_observed": sorted({x["channel"] for x in channels if "channel" in x}),
        "first_bundle_headers": headers[:12],
        "monotonic_field_candidates": monotonic_candidates(headers),
        "pcap_stderr": stderr,
        "pcap_path": str(PCAP),
        "important_limit": "Only the first 90 bytes of each UDP packet were captured; no encoded image payload was saved.",
    }
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("REPORT", OUT)

if __name__ == "__main__":
    main()
