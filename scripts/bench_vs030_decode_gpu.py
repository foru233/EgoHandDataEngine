"""比较 VS030 H.265 的软件解码与 CUDA/NVDEC 硬件解码。"""
import struct
import sys
import time
from pathlib import Path

import av
import numpy as np
from av.codec.hwaccel import HWAccel

ROOT = Path(__file__).resolve().parents[1]
REC = struct.Struct('<IHHIIQQQQQQ')


def load_packets(recording, limit=200):
    packets = {4: [], 5: []}
    with open(Path(recording) / 'raw/sdk_records.bin', 'rb') as stream:
        while len(packets[4]) < limit or len(packets[5]) < limit:
            raw = stream.read(REC.size)
            if not raw:
                break
            magic, version, header_size, sink, size, seq, dev, cap, arr, h1, h2 = REC.unpack(raw)
            if magic != 0x58444652:
                break
            payload = stream.read(size)
            if sink in packets and len(packets[sink]) < limit:
                packets[sink].append(payload)
    return packets


def decode_all(payloads, hwaccel=None):
    kwargs = {'hwaccel': HWAccel(hwaccel)} if hwaccel else {}
    ctx = av.CodecContext.create('hevc', 'r', **kwargs)
    frames = 0
    started = time.perf_counter()
    first_shape = None
    for payload in payloads:
        for frame in ctx.decode(av.Packet(payload)):
            array = frame.to_ndarray(format='gray')
            if first_shape is None:
                first_shape = array.shape
            frames += 1
    elapsed = time.perf_counter() - started
    return frames, elapsed, first_shape


def main():
    recording = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / 'data/imu/session_20261110_112020_689246900')
    packets = load_packets(recording, 200)
    print('packets', len(packets[4]), len(packets[5]), flush=True)
    for name, hw in (('software', None), ('cuda', 'cuda'), ('d3d11va', 'd3d11va')):
        try:
            frames, elapsed, shape = decode_all(packets[5], hw)
            print(f'{name}: frames={frames} elapsed={elapsed:.2f}s per_frame={elapsed / max(frames, 1) * 1e3:.2f}ms shape={shape}', flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f'{name}: FAILED {type(exc).__name__}: {exc}', flush=True)


if __name__ == '__main__':
    main()
