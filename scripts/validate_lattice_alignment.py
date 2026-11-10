"""用已采集的 8 分钟数据离线验证格点重采样是否恢复视觉-陀螺对齐。"""
import glob
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.vio.imu_lattice import LatticeImuBuffer  # noqa: E402
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry  # noqa: E402


def corr(a, b):
    a = a - a.mean(0)
    b = b - b.mean(0)
    denom = np.linalg.norm(a) * np.linalg.norm(b)
    return float(np.sum(a * b) / denom) if denom > 1e-12 else float('nan')


def main():
    path = sorted(glob.glob(str(ROOT / 'results/vio/vs030_lag_capture_*.npz')))[-1]
    data = np.load(path)
    imu = data['imu']
    vt = data['visual_ts'].astype(np.int64)
    vdt = data['visual_dt'].astype(np.float64)
    vr = data['visual_rot'].astype(np.float64)
    geo = prepare_vs030_vio_geometry(ROOT / '标定结果/1/data3-camchain-imucam.yaml')
    rot = geo.T_imu_from_orb_left[:3, :3].T
    gyro_cam = imu['gyro'].astype(np.float64) @ rot.T
    driver_ts = imu['ts_b'].astype(np.int64)

    # ---- 原始时间戳：按 driver 时间范围取均值 ----
    raw_means = []
    for t, dt in zip(vt, vdt):
        start, end = int(t - dt * 5e8), int(t + dt * 5e8)
        lo, hi = np.searchsorted(driver_ts, start), np.searchsorted(driver_ts, end)
        raw_means.append(gyro_cam[lo:hi].mean(0) if hi > lo else np.zeros(3))
    raw_means = np.array(raw_means)

    # ---- 格点：按相机时间计数消费 ----
    buffer = LatticeImuBuffer(sample_rate_hz=480.0, camera_to_imu_drift_ppm=3000.0,
                              capacity=400000)
    buffer.add(imu.tobytes())
    buffer.trim_before(int(vt[0] - vdt[0] * 5e8))
    lattice_means = []
    skipped = 0
    last_end = int(vt[0] - vdt[0] * 5e8)
    for t, dt in zip(vt, vdt):
        start, end = int(t - dt * 5e8), int(t + dt * 5e8)
        if start > last_end:
            buffer.skip_span(start - last_end)
        rows = buffer.consume(start, end)
        last_end = end
        if rows is None:
            skipped += 1
            lattice_means.append(gyro_cam.mean(0))
            continue
        lattice_means.append((np.mean([r[4:7] for r in rows], axis=0) @ rot.T) if rows else np.zeros(3))
    lattice_means = np.array(lattice_means)

    report = {
        'source': path,
        'intervals': len(vt),
        'pending_left': buffer.pending(),
        'lattice_skipped': skipped,
        'raw_corr': corr(vr, raw_means),
        'lattice_corr': corr(vr, lattice_means),
        'raw_rot_std': float(vr.std()),
    }
    out = ROOT / 'results/vio/vs030_lattice_validation.json'
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('SAVED', out)


if __name__ == '__main__':
    main()
