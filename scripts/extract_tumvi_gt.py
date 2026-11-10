"""从已校正 TUM-VI bag 提取 IMU 坐标系动捕真值，仅用于事后评测。"""
from decimal import Decimal
from pathlib import Path
from rosbags.highlevel import AnyReader

ROOT = Path(__file__).resolve().parents[1]
BAG = ROOT / 'samples/date/VIO/TUM-VI/room4/dataset-room4_512_16.bag'
OUTPUT = ROOT / 'results/vio/tumvi_room4_gt_from_bag.tum'

def main():
    rows = []
    with AnyReader([BAG]) as reader:
        connections = [c for c in reader.connections if c.topic == '/vrpn_client/raw_transform']
        if len(connections) != 1:
            raise ValueError('标定动捕话题缺失或不唯一')
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            timestamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            t, q = msg.transform.translation, msg.transform.rotation
            if rows and timestamp <= rows[-1][0]:
                raise ValueError('动捕时间戳不单调')
            rows.append((timestamp, t.x, t.y, t.z, q.x, q.y, q.z, q.w))
    if not rows:
        raise ValueError('动捕轨迹为空')
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(''.join(str(Decimal(row[0])/Decimal(1_000_000_000)) + ' ' + ' '.join(f'{x:.12g}' for x in row[1:]) + '\n' for row in rows), encoding='utf8')
    print(f'{len(rows)} poses -> {OUTPUT}')

if __name__ == '__main__':
    main()
