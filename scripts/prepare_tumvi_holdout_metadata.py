"""Extract timestamps, IMU and GT metadata from a TUM-VI holdout bag."""
from decimal import Decimal
import csv
import json
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'samples10-15gb/VIO/TUM-VI/room3_holdout'
BAG = FOLDER / 'room3.bag'


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main():
    outputs = [FOLDER / name for name in ('timestamps_inertial.txt', 'imu.csv', 'groundtruth_from_bag.tum')]
    if any(path.exists() for path in outputs):
        raise FileExistsError('holdout metadata already exists; refusing overwrite')
    store = get_typestore(Stores.ROS1_NOETIC)
    cameras, imu_rows, gt_rows = [], [], []
    with Reader(BAG) as reader:
        connections = [c for c in reader.connections if c.topic in (
            '/cam0/image_raw', '/cam1/image_raw', '/imu0', '/vrpn_client/raw_transform')]
        for conn, _, raw in reader.messages(connections=connections):
            msg = store.deserialize_ros1(raw, conn.msgtype)
            stamp = ns(msg.header.stamp)
            if conn.topic == '/cam0/image_raw':
                cameras.append((stamp, 0))
            elif conn.topic == '/cam1/image_raw':
                cameras.append((stamp, 1))
            elif conn.topic == '/imu0':
                g, a = msg.angular_velocity, msg.linear_acceleration
                imu_rows.append((stamp, g.x, g.y, g.z, a.x, a.y, a.z))
            else:
                t, q = msg.transform.translation, msg.transform.rotation
                gt_rows.append((stamp, t.x, t.y, t.z, q.x, q.y, q.z, q.w))
    left = sorted(stamp for stamp, side in cameras if side == 0)
    right = sorted(stamp for stamp, side in cameras if side == 1)
    if left != right or not left or any(b <= a for a, b in zip(left, left[1:])):
        raise ValueError('holdout camera timestamps are not an exact monotonic stereo pair')
    imu_rows.sort(); gt_rows.sort()
    if not imu_rows or not gt_rows or any(b[0] <= a[0] for a, b in zip(imu_rows, imu_rows[1:])):
        raise ValueError('holdout IMU stream is empty or nonmonotonic')
    if any(b[0] <= a[0] for a, b in zip(gt_rows, gt_rows[1:])):
        raise ValueError('holdout GT timestamps are nonmonotonic')
    FOLDER.mkdir(parents=True, exist_ok=True)
    (FOLDER / 'timestamps_inertial.txt').write_text(''.join(f'{stamp}\n' for stamp in left), encoding='utf-8')
    with (FOLDER / 'imu.csv').open('x', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y', 'w_RS_S_z',
                         'a_RS_S_x [m s^-2]', 'a_RS_S_y', 'a_RS_S_z'])
        writer.writerows(imu_rows)
    (FOLDER / 'groundtruth_from_bag.tum').write_text(
        ''.join(str(Decimal(row[0]) / Decimal(1_000_000_000)) + ' ' +
                ' '.join(f'{value:.12g}' for value in row[1:]) + '\n' for row in gt_rows),
        encoding='utf-8')
    report = {'scope': 'metadata only; images remain in the immutable ROS1 bag',
              'bag': str(BAG), 'camera_pairs': len(left), 'imu_rows': len(imu_rows),
              'groundtruth_rows': len(gt_rows), 'first_camera_ns': left[0], 'last_camera_ns': left[-1],
              'imu_covers_all_cameras': imu_rows[0][0] <= left[0] <= left[-1] <= imu_rows[-1][0],
              'ground_truth_frame': 'calibrated bag IMU frame; source documentation still applies'}
    (FOLDER / 'metadata_extraction.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
