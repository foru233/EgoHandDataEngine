"""对比相机 device_ts 与 IMU 时间戳是否同一时钟域（只打印数值）。"""
import queue
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.sources.vendor_xdsdk import XdSdkStereoSource  # noqa: E402

REC = struct.Struct('<QQ6f')


def main():
    source = XdSdkStereoSource()
    source.enable_imu_capture()
    rows = []
    printed = 0
    try:
        source.open()
        source.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and printed < 5:
            frame = source.read()
            if frame is None:
                break
            while True:
                try:
                    item = source.imu_vio_queue.get_nowait()
                except queue.Empty:
                    break
                payload = item[-1]
                for off in range(0, len(payload), REC.size):
                    rows.append(REC.unpack_from(payload, off))
            md = frame.metadata
            cam_us = int(md['sdk_device_timestamp_raw'])
            cam_ns = cam_us * 1000
            last_imu = rows[-1][0] if rows else None
            print('frame', printed,
                  'cam_device_ts_raw', cam_us,
                  'cam*1000_ns', cam_ns,
                  'imu_count', len(rows),
                  'imu_last_a', last_imu,
                  'cam-imu_last_ns', (cam_ns - last_imu) if last_imu is not None else None,
                  'sdk_capture_ch4', md.get('sdk_capture_ts_ns_channel_4'),
                  'host_now_ns', time.monotonic_ns())
            printed += 1
    finally:
        source.close()


if __name__ == '__main__':
    main()
