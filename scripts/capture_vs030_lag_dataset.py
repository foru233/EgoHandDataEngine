"""采集 VS030 视觉旋转 + 原始 IMU，用于离线估计相机-IMU 时延（含漂移）。

只保存数值，不保存图像。输出 results/vio/vs030_lag_capture_<时间戳>.npz 与同名 .json。
用法：python scripts/capture_vs030_lag_dataset.py [时长秒]
采集期间请缓慢、连续地转动相机（左右/上下各若干次），并保留一段静止。
"""
import json
import queue
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.sources.vendor_xdsdk import XdSdkStereoSource  # noqa: E402
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry  # noqa: E402

IMU = np.dtype([('ts_a', '<u8'), ('ts_b', '<u8'), ('accel', '<f4', (3,)), ('gyro', '<f4', (3,))])
FRAME_INTERVAL_S = 0.12
MAX_CORNERS = 500
SCALE = 0.5


def decode_rows(payload):
    if len(payload) % 40:
        raise ValueError('IMU payload length is not a multiple of 40')
    out = np.frombuffer(payload, dtype=np.dtype([('ts_a', '<u8'), ('ts_b', '<u8'),
                                                 ('accel', '<f4', (3,)), ('gyro', '<f4', (3,))]))
    return out


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 420.0
    geo = prepare_vs030_vio_geometry(ROOT / '标定结果/1/data3-camchain-imucam.yaml')
    rect = geo.rectifier
    maps = tuple(cv2.resize(m, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_LINEAR)
                 for m in rect.maps_left)
    K = rect.P1[:3, :3].copy()
    K[:2, :] *= SCALE
    source = XdSdkStereoSource()
    source.enable_imu_capture(max_packets=8192)
    imu_chunks = []
    visual = []
    error = None
    previous = None
    last_visual = 0.0
    start = None
    try:
        source.open()
        source.start()
        start = time.monotonic()
        end = start + duration
        next_report = start + 30
        while time.monotonic() < end:
            frame = source.read()
            if frame is None:
                break
            while True:
                try:
                    item = source.imu_vio_queue.get_nowait()
                except queue.Empty:
                    break
                imu_chunks.append(decode_rows(item[-1]))
            now = time.monotonic()
            if now < last_visual + FRAME_INTERVAL_S:
                continue
            md = frame.metadata
            ts = int(md['sdk_device_timestamp_raw']) * 1000
            gray = cv2.cvtColor(frame.right_image, cv2.COLOR_BGR2GRAY)
            gray = cv2.remap(gray, *maps, cv2.INTER_LINEAR)
            if previous is not None:
                last_ts, last_image, last_points = previous
                dt = (ts - last_ts) * 1e-9
                points, status, _ = cv2.calcOpticalFlowPyrLK(last_image, gray, last_points, None)
                if points is not None and status is not None and 0.03 < dt < 0.5:
                    valid = status.reshape(-1).astype(bool)
                    p0 = last_points.reshape(-1, 2)[valid]
                    p1 = points.reshape(-1, 2)[valid]
                    if len(p0) >= 30:
                        H, inliers = cv2.findHomography(p0, p1, cv2.RANSAC, 2.0)
                        if H is not None and inliers is not None and int(inliers.sum()) >= 25:
                            M = np.linalg.inv(K) @ H @ K
                            if np.linalg.det(M) > 0:
                                U, _, V = np.linalg.svd(M)
                                R = U @ V
                                if np.linalg.det(R) > 0:
                                    rv, _ = cv2.Rodrigues(R)
                                    visual.append(((last_ts + ts) // 2, dt, int(inliers.sum()),
                                                   (-rv[:, 0] / dt).tolist()))
            corners = cv2.goodFeaturesToTrack(gray, MAX_CORNERS, .01, 12)
            previous = (ts, gray, corners) if corners is not None and len(corners) >= 30 else None
            last_visual = now
            if time.monotonic() >= next_report:
                total = sum(len(c) for c in imu_chunks)
                print('progress', round(time.monotonic() - start, 1), 's frames', len(visual),
                      'imu_rows', total, 'stats', source.get_capture_stats(), flush=True)
                next_report += 30
    except Exception as exc:  # noqa: BLE001
        error = repr(exc)
    finally:
        try:
            source.close()
        except Exception:  # noqa: BLE001
            pass

    imu = np.concatenate(imu_chunks) if imu_chunks else np.zeros(0, dtype=IMU)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    out = ROOT / 'results/vio' / f'vs030_lag_capture_{stamp}.npz'
    np.savez_compressed(out, imu=imu,
                        visual_ts=np.array([v[0] for v in visual], dtype=np.int64),
                        visual_dt=np.array([v[1] for v in visual], dtype=np.float64),
                        visual_inliers=np.array([v[2] for v in visual], dtype=np.int32),
                        visual_rot=np.array([v[3] for v in visual], dtype=np.float64))
    meta = {'duration_s': duration, 'imu_rows': int(len(imu)), 'visual_rows': len(visual),
            'error': error, 'stats': source.get_capture_stats() if hasattr(source, 'get_capture_stats') else None,
            'note': '数值采集；不含图像。visrot 为单应旋转估计的角速度（rad/s），与 IMU gyro 的轴向关系沿用 diagnose_vs030_gyro.py。'}
    (out.with_suffix('.json')).write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf8')
    print('SAVED', out, json.dumps(meta, ensure_ascii=True), flush=True)


if __name__ == '__main__':
    main()
