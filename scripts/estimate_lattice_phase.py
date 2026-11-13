"""用录制数据估计 IMU 包格点的最佳相位（视觉旋转 vs 陀螺）。

方法：解码录制中的双目 H265，隔帧计算单应旋转角速度；把 IMU 包按
`packet_device_ts*1000 + phase + (i+0.5)*period` 铺成时间序列，扫相位，
找与视觉旋转相关最大的相位。只打印数值，不保存图像。
"""
import argparse
import struct
import sys
from pathlib import Path

import av
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry  # noqa: E402

REC = struct.Struct('<IHHIIQQQQQQ')
MAGIC = 0x58444652
PERIOD_NS = 1e9 / 480.0
FRAME_STEP = 4


def load(recording):
    packets = {4: [], 5: [], 7: []}
    path = Path(recording) / 'raw/sdk_records.bin'
    with open(path, 'rb') as stream:
        while True:
            raw = stream.read(REC.size)
            if not raw:
                break
            magic, version, hs, sink, size, seq, dev, cap, arr, h1, h2 = REC.unpack(raw)
            payload = stream.read(size)
            if magic != MAGIC:
                raise RuntimeError('SDK 记录格式异常')
            if sink in packets:
                packets[sink].append((dev, payload))
    return packets


def visual_rotations(packets, rectifier, scale=0.5):
    maps = tuple(cv2.resize(m, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR) for m in rectifier.maps_left)
    K = rectifier.P1[:3, :3].copy(); K[:2, :] *= scale
    decoder = av.CodecContext.create('hevc', 'r')
    previous = None
    rows = []
    for index, (dev, payload) in enumerate(packets[5]):
        frames = decoder.decode(av.Packet(payload))
        if not frames:
            continue
        if index % FRAME_STEP:
            continue
        image = frames[-1].to_ndarray(format='gray')
        image = cv2.remap(image, *maps, cv2.INTER_LINEAR)
        stamp = dev * 1000
        if previous is not None:
            last_stamp, last_image, last_points = previous
            dt = (stamp - last_stamp) * 1e-9
            if 0.03 < dt < 0.5:
                points, status, _ = cv2.calcOpticalFlowPyrLK(last_image, image, last_points, None)
                if points is not None and status is not None:
                    valid = status.reshape(-1).astype(bool)
                    p0 = last_points.reshape(-1, 2)[valid]; p1 = points.reshape(-1, 2)[valid]
                    if len(p0) >= 30:
                        Hm, inliers = cv2.findHomography(p0, p1, cv2.RANSAC, 2.0)
                        if Hm is not None and inliers is not None and int(inliers.sum()) >= 25:
                            M = np.linalg.inv(K) @ Hm @ K
                            if np.linalg.det(M) > 0:
                                U, _, V = np.linalg.svd(M); R = U @ V
                                if np.linalg.det(R) > 0:
                                    rv, _ = cv2.Rodrigues(R)
                                    rows.append(((last_stamp + stamp) // 2, dt, (-rv[:, 0] / dt).tolist()))
        corners = cv2.goodFeaturesToTrack(image, 400, .01, 12)
        previous = (stamp, image, corners) if corners is not None and len(corners) >= 30 else None
    return rows


def imu_series(packets, phase_ns):
    stamps = []; values = []
    for dev, payload in packets[7]:
        count = len(payload) // 40
        start = dev * 1000 + phase_ns
        for index in range(count):
            row = struct.unpack_from('<QQ6f', payload, index * 40)
            stamps.append(int(start + (index + .5) * PERIOD_NS))
            values.append(list(row[2:]))
    return np.array(stamps, dtype=np.int64), np.array(values, dtype=float)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recording')
    args = parser.parse_args()
    session = Path(args.recording)
    packets = load(session)
    geometry = prepare_vs030_vio_geometry(session / 'kalibr_camchain.yaml')
    visual = visual_rotations(packets, geometry.rectifier)
    print('visual samples', len(visual), flush=True)
    if len(visual) < 30:
        raise SystemExit('视觉样本不足')
    vt = np.array([v[0] for v in visual], dtype=np.int64)
    vr = np.array([v[2] for v in visual], dtype=float)
    rotate = geometry.T_imu_from_orb_left[:3, :3].T
    best = None
    for phase_ms in range(-250, 251, 5):
        stamps, values = imu_series(packets, round(phase_ms * 1e6))
        gyro = values[:, 3:] @ rotate.T
        valid = (vt >= stamps[0]) & (vt <= stamps[-1])
        if valid.sum() < 30:
            continue
        g = np.stack([np.interp(vt[valid], stamps, gyro[:, axis]) for axis in range(3)], axis=1)
        a = vr[valid] - vr[valid].mean(0); g = g - g.mean(0)
        denom = np.linalg.norm(a) * np.linalg.norm(g)
        corr = float(np.sum(a * g) / denom) if denom > 1e-12 else -9
        if best is None or corr > best[1]:
            best = (phase_ms, corr)
        if phase_ms % 25 == 0:
            print('phase %+4d ms corr %.4f' % (phase_ms, corr), flush=True)
    print('BEST phase %+d ms corr %.4f' % best, flush=True)


if __name__ == '__main__':
    main()
