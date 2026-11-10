"""一次性探测：抓取通道 7 原始 IMU 包，验证 40 字节布局的字段顺序与量纲。

只打印数值，不保存图像；不修改设备配置（SDK 启动/停止会自行恢复预览配置）。
"""
import queue
import statistics
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
    packets = []
    frames = 0
    error = None
    try:
        source.open()
        source.start()
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and frames < 20:
            frame = source.read()
            if frame is None:
                break
            frames += 1
            while True:
                try:
                    packets.append(source.imu_vio_queue.get_nowait())
                except queue.Empty:
                    break
    except Exception as exc:  # noqa: BLE001
        error = repr(exc)
    finally:
        source.close()

    print('frames', frames, 'packets', len(packets), 'error', error)
    print('sizes', sorted({len(p[3]) for p in packets if len(p) == 4}))
    if not packets:
        return
    rows = []
    for item in packets:
        payload = item[-1]
        for off in range(0, len(payload), REC.size):
            rows.append(REC.unpack_from(payload, off))
    print('records', len(rows))
    a = [r[0] for r in rows]
    b = [r[1] for r in rows]
    first = [r[2:5] for r in rows]
    second = [r[5:8] for r in rows]
    print('ts_a first/last', a[0], a[-1], 'median_dt_ns',
          statistics.median(x - y for x, y in zip(a[1:], a[:-1])) if len(a) > 1 else None)
    print('ts_b first/last', b[0], b[-1], 'median_dt_ns',
          statistics.median(x - y for x, y in zip(b[1:], b[:-1])) if len(b) > 1 else None)
    diffs = [y - x for x, y in zip(a, b)]
    print('ts_b-ts_a ns', 'min', min(diffs), 'median', statistics.median(diffs),
          'max', max(diffs), 'stdev', round(statistics.pstdev(diffs), 1))
    dts = [y - x for x, y in zip(a, a[1:])]
    print('ts_a dt ns', 'min', min(dts), 'median', statistics.median(dts), 'max', max(dts))
    for name, vals in (('triplet1', first), ('triplet2', second)):
        means = [statistics.fmean(v[i] for v in vals) for i in range(3)]
        stds = [statistics.pstdev(v[i] for v in vals) for i in range(3)]
        print(name, 'mean', [round(x, 6) for x in means], 'std', [round(x, 6) for x in stds],
              'norm_mean', round(sum(x * x for x in means) ** .5, 6))
    for i in range(min(3, len(rows))):
        print('row', i, rows[i])


if __name__ == '__main__':
    main()
