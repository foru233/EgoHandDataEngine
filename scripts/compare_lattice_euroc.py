"""EuRoC 回放：比较原始 IMU 时间戳与格点重采样对 ORB-SLAM3 的影响。

用途：在没有 VS030 运动数据时，验证格点路径在真实图像/IMU 上不引入回归。
"""
import csv
import json
import struct
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.vio.imu_lattice import LatticeImuBuffer  # noqa: E402
from egohand.vio.native_bridge import ImuMeasurement, NativeVioBridge  # noqa: E402

REC = struct.Struct('<QQ6f')


def load_dataset():
    root = ROOT / 'samples/date/VIO/converted/EuRoC/MH_01_easy'
    stamps = [int(x) for x in (root / 'timestamps_smoke_160.txt').read_text().split()]
    rows = []
    with (root / 'mav0/imu0/data.csv').open() as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            fields = next(csv.reader([line]))
            rows.append((int(fields[0]), [float(x) for x in fields[1:4]], [float(x) for x in fields[4:7]]))
    return root, stamps, rows


def run_replay(root, stamps, imu_rows, mode, rate_hz=200.0):
    vocabulary = ROOT / 'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt'
    config = root / 'calibration/EuRoC.yaml'
    results = []
    previous = None
    if mode == 'lattice':
        buffer = LatticeImuBuffer(sample_rate_hz=rate_hz, camera_to_imu_drift_ppm=0.0, capacity=100000)
        payload = b''.join(REC.pack(ts, ts + 1, *acc, *gyro) for ts, gyro, acc in imu_rows)
        buffer.add(payload)
        buffer.trim_before(stamps[0])
    else:
        index = 0
    started = time.perf_counter()
    with NativeVioBridge(vocabulary, config) as bridge:
        for stamp in stamps:
            images = [np.asarray(Image.open(root / f'mav0/cam{k}/data/{stamp}.png').convert('L')) for k in (0, 1)]
            if mode == 'lattice':
                batch = [] if previous is None else buffer.consume(previous, stamp)
                if batch is None:
                    raise RuntimeError('格点样本不足')
                measurements = [ImuMeasurement(int(r[0]), tuple(float(x) for x in r[1:4]),
                                               tuple(float(x) for x in r[4:7])) for r in batch]
            else:
                measurements = []
                while index < len(imu_rows) and imu_rows[index][0] <= stamp:
                    ts, gyro, acc = imu_rows[index]
                    if previous is not None and ts > previous:
                        measurements.append(ImuMeasurement(ts, tuple(acc), tuple(gyro)))
                    index += 1
            results.append(bridge.track_stereo(images[0], images[1], stamp, measurements))
            previous = stamp
    poses = [r.T_world_from_rectified_left_camera for r in results if r.pose_valid]
    return {
        'mode': mode,
        'frames': len(results),
        'seconds': round(time.perf_counter() - started, 1),
        'states': dict(Counter(r.tracking_state_name for r in results)),
        'pose_valid': len(poses),
        'imu_initialized_frames': sum(1 for r in results if r.imu_initialized),
        'poses': poses,
    }


def main():
    root, stamps, imu_rows = load_dataset()
    report = {'dataset': 'EuRoC MH_01_easy 160 frames', 'imu_rows': len(imu_rows)}
    runs = {}
    for mode in ('timestamp', 'lattice'):
        result = run_replay(root, stamps, imu_rows, mode)
        runs[mode] = result['poses']
        report[mode] = {k: v for k, v in result.items() if k != 'poses'}
    common = min(len(runs['timestamp']), len(runs['lattice']))
    if common:
        deltas = [float(np.linalg.norm(np.asarray(runs['timestamp'][i])[:3, 3] - np.asarray(runs['lattice'][i])[:3, 3]))
                  for i in range(common)]
        report['pose_position_delta_m'] = {'max': max(deltas), 'median': float(np.median(deltas))}
    out = ROOT / 'results/vio/vs030_lattice_euroc_compare.json'
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('SAVED', out)


if __name__ == '__main__':
    main()
