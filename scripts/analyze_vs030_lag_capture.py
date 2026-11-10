"""分析 capture_vs030_lag_dataset.py 的数据：时延是否随时间漂移（常数 vs 仿射）。

输出：results/vio/vs030_lag_capture_analysis.json 与控制台摘要。
不修改采集数据；不声称精度。
"""
import glob
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry  # noqa: E402

LAGS_MS = np.arange(-300, 301, 5)
GLOBAL_LAGS_MS = np.arange(-400, 401, 2)


def correlate(visual_ts, visual_rot, gyro_ts, gyro_cam, lags_ms):
    rows = []
    for lag in lags_ms:
        target = visual_ts + int(lag * 1e6)
        valid = (target >= gyro_ts[0]) & (target <= gyro_ts[-1])
        if valid.sum() < 15:
            continue
        g = np.stack([np.interp(target[valid], gyro_ts, gyro_cam[:, ax]) for ax in range(3)], axis=1)
        a = visual_rot[valid]
        a = a - a.mean(0)
        g = g - g.mean(0)
        denom = np.linalg.norm(a) * np.linalg.norm(g)
        if denom > 1e-12:
            rows.append((float(lag), float(np.sum(a * g) / denom), int(valid.sum())))
    return rows


def best(rows):
    return max(rows, key=lambda r: r[1]) if rows else None


def main():
    paths = sorted(glob.glob(str(ROOT / 'results/vio/vs030_lag_capture_*.npz')))
    if not paths:
        raise SystemExit('没有采集数据')
    path = Path(paths[-1])
    data = np.load(path)
    imu = data['imu']
    vt = data['visual_ts'].astype(np.int64)
    vr = data['visual_rot'].astype(np.float64)
    geo = prepare_vs030_vio_geometry(ROOT / '标定结果/1/data3-camchain-imucam.yaml')
    gyro_to_camera = geo.T_imu_from_orb_left[:3, :3].T
    gts = imu['ts_b'].astype(np.int64)
    gcam = imu['gyro'].astype(np.float64) @ gyro_to_camera.T

    report = {'source': str(path), 'imu_rows': int(len(imu)), 'visual_rows': int(len(vr)),
              'imu_rate_hz': float(1e9 / np.median(np.diff(imu['ts_a'].astype(np.int64))))}

    # 全局单点
    rows = correlate(vt, vr, gts, gcam, GLOBAL_LAGS_MS)
    b = best(rows)
    report['global_best'] = {'lag_ms': b[0], 'correlation': b[1], 'samples': b[2]} if b else None

    # 分窗：时延漂移检测
    window_s = 10.0
    hop_s = 5.0
    span_s = (vt[-1] - vt[0]) / 1e9
    windows = []
    t0 = 0.0
    while t0 + window_s <= span_s + 1e-9:
        sel = (vt >= vt[0] + int(t0 * 1e9)) & (vt < vt[0] + int((t0 + window_s) * 1e9))
        if sel.sum() >= 12 and vr[sel].std() > 0.005:
            wrows = correlate(vt[sel], vr[sel], gts, gcam, LAGS_MS)
            wb = best(wrows)
            if wb:
                windows.append({'start_s': round(t0, 1), 'samples': int(sel.sum()),
                                'motion_std': round(float(vr[sel].std()), 4),
                                'lag_ms': wb[0], 'correlation': round(wb[1], 4)})
        t0 += hop_s
    report['windows'] = windows
    if len(windows) >= 3:
        clean = [w for w in windows if w['correlation'] > 0.85 and w['motion_std'] > 0.2
                 and -290 < w['lag_ms'] < 60]
        x = np.array([w['start_s'] + window_s / 2 for w in clean])
        y = np.array([w['lag_ms'] for w in clean])
        if len(clean) >= 3:
            slope, intercept = np.polyfit(x, y, 1)
            pred = slope * x + intercept
            report['drift_model'] = {
                'windows': len(clean), 'slope_ms_per_s': float(slope),
                'intercept_ms': float(intercept),
                'rms_residual_ms': float(np.sqrt(np.mean((y - pred) ** 2))),
                'constant_lag_residual_ms': float(np.std(y)),
                'drift_ppm': float(slope * 1e-3 * 1e6),
                'lag_min_ms': float(y.min()), 'lag_max_ms': float(y.max())}
        report['affine_fit_all_windows'] = {'note': '含静止/边界窗口，仅作参考'}
        report['constant_lag_rms_ms'] = float(np.std([w['lag_ms'] for w in windows]))
    # 静止段噪声
    still = np.abs(vr).max(axis=1) < 0.02
    if still.sum() >= 10:
        report['static'] = {'samples': int(still.sum()),
                            'visual_rot_std': float(vr[still].std()),
                            'gyro_std': np.round(gcam.std(0), 6).tolist()}
    out = ROOT / 'results/vio/vs030_lag_capture_analysis.json'
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps({k: v for k, v in report.items() if k != 'windows'}, ensure_ascii=False, indent=2))
    print('windows:', json.dumps(windows, ensure_ascii=False))
    print('SAVED', out)


if __name__ == '__main__':
    main()
