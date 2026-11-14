"""Verify room3 sensor/GT integrity, without running or evaluating VIO."""
from pathlib import Path
import json
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'samples10-15gb/VIO/TUM-VI/room3_holdout'
TOPICS = ('/cam0/image_raw', '/cam1/image_raw', '/imu0', '/vrpn_client/raw_transform')


def main():
    times = {name: [] for name in TOPICS}
    bad_image = bad_numeric = 0
    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(FOLDER / 'room3.bag') as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        if len(conns) != len(TOPICS) or {c.topic for c in conns} != set(TOPICS):
            raise ValueError('expected four unique calibrated sensor topics')
        for conn, _, data in reader.messages(connections=conns):
            msg = store.deserialize_ros1(data, conn.msgtype)
            t = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            times[conn.topic].append(t)
            if 'image_raw' in conn.topic:
                bad_image += not (msg.encoding == 'mono16' and msg.width == 512 and msg.height == 512
                                  and len(msg.data) == msg.height * msg.step)
            elif conn.topic == '/imu0':
                a, w = msg.linear_acceleration, msg.angular_velocity
                bad_numeric += not np.isfinite([a.x, a.y, a.z, w.x, w.y, w.z]).all()
            else:
                p, q = msg.transform.translation, msg.transform.rotation
                values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
                bad_numeric += not (np.isfinite(values).all() and abs(np.linalg.norm(values[3:]) - 1) < 1e-5)
    arrays = {k: np.array(v, dtype=np.int64) for k, v in times.items()}
    if any(len(a) < 2 or np.any(np.diff(a) <= 0) for a in arrays.values()):
        raise ValueError('empty or nonmonotonic sensor stream')
    left, right, imu, gt = [arrays[k] for k in TOPICS]
    gaps = np.diff(gt)
    report = {'scope': 'sensor and GT structural integrity only; no VIO or trajectory scoring',
              'counts': {k: len(v) for k, v in arrays.items()},
              'stereo_exact_timestamps': bool(np.array_equal(left, right)),
              'camera_resolution': [512, 512], 'image_encoding': 'mono16',
              'imu_median_interval_ns': int(np.median(np.diff(imu))),
              'imu_covers_camera_count': int(np.count_nonzero((left >= imu[0]) & (left <= imu[-1]))),
              'bad_images': int(bad_image), 'bad_numeric': int(bad_numeric),
              'gt_gaps_over_50ms': [{'before_ns': int(gt[i]), 'after_ns': int(gt[i + 1])}
                                    for i in np.flatnonzero(gaps > 50_000_000)],
              'ground_truth_frame': 'calibrated bag IMU frame; source documentation still applies',
              'evaluation_performed': False}
    if not report['stereo_exact_timestamps'] or bad_image or bad_numeric:
        raise ValueError('integrity checks failed; inspect bag')
    with (FOLDER / 'integrity_validation.json').open('x', encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
