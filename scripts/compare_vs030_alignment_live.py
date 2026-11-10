"""一次运行内依次测试 lattice 与 timestamp 两种 IMU 对齐方式（各自等待运动触发）。

用法：python scripts/compare_vs030_alignment_live.py --seconds 60
用户只需在两段各自开始前转动相机；每段检测到运动后计时。
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.sources.vendor_xdsdk import XdSdkStereoSource  # noqa: E402
from egohand.vio.live_runtime import LiveVio  # noqa: E402


def summarize(records, seconds, wall, frames, timing, errors, motion_samples):
    epochs = sorted({r.get('map_epoch') for r in records if r.get('map_epoch') is not None})
    states = {}
    quality = {}
    longest = current = 0
    first_init = None
    for index, record in enumerate(records):
        state = record.get('tracking_state')
        states[state] = states.get(state, 0) + 1
        quality[record.get('quality_status')] = quality.get(record.get('quality_status'), 0) + 1
        if record.get('imu_initialized'):
            if first_init is None:
                first_init = index
            current = current + 1 if state == 'OK' else 0
            longest = max(longest, current)
    return {
        'seconds': seconds,
        'wall_seconds': round(wall, 1),
        'frames_submitted': frames,
        'pose_records': len(records),
        'states': states,
        'quality': quality,
        'imu_initialized': sum(1 for r in records if r.get('imu_initialized')),
        'first_imu_initialized_index': first_init,
        'raw_pose_valid': sum(1 for r in records if r.get('raw_pose_valid')),
        'accepted_pose': sum(1 for r in records if r.get('pose_valid')),
        'map_epochs': len(epochs),
        'map_epoch_values': epochs[:20],
        'longest_ok_streak_after_init': longest,
        'motion_samples': motion_samples,
        'timing': timing,
        'errors': errors,
    }


def run_mode(alignment, seconds, lag_ms, motion_timeout, drift_ppm, rate_hz):
    source = XdSdkStereoSource()
    records = []
    errors = []
    timing = {'imu_alignment': alignment, 'camera_to_imu_lag_ms': lag_ms,
              'camera_to_imu_drift_ppm': drift_ppm, 'imu_rate_hz': rate_hz}

    def on_status(message):
        if message.get('type') == 'error':
            errors.append(message.get('message'))
        elif message.get('type') == 'pose':
            records.append(message)

    vio = LiveVio(source, on_status, timing=timing)
    started = time.monotonic()
    frames = 0
    print(f'MODE {alignment} starting', flush=True)
    try:
        source.open()
        vio.start()
        if not vio.wait_ready(120):
            raise RuntimeError(vio.startup_error or 'ORB-SLAM3 未就绪')
        source.start()
        print(f'MODE {alignment} WAITING_MOTION', flush=True)
        deadline = None
        record_start = 0
        stream_start = time.monotonic()
        while True:
            frame = source.read()
            if frame is None:
                break
            frames += 1
            vio.submit(frame)
            if deadline is None:
                if vio.motion_event.is_set():
                    print(f'MODE {alignment} MOTION_DETECTED', flush=True)
                    record_start = len(records)
                    deadline = time.monotonic() + seconds
                elif time.monotonic() - stream_start > motion_timeout:
                    raise RuntimeError('等待相机运动超时')
            elif time.monotonic() >= deadline:
                break
        records = records[record_start:]
    except Exception as exc:  # noqa: BLE001
        errors.append(repr(exc))
    finally:
        try:
            vio.close()
        finally:
            try:
                source.close()
            except Exception:  # noqa: BLE001
                pass
    return summarize(records, seconds, time.monotonic() - started, frames, timing, errors,
                     vio.motion_samples)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--lag-ms', type=float, default=-85.0)
    parser.add_argument('--drift-ppm', type=float, default=657.0)
    parser.add_argument('--rate-hz', type=float, default=480.0)
    parser.add_argument('--motion-timeout', type=float, default=600.0)
    parser.add_argument('--out', default='vs030_alignment_ab.json')
    args = parser.parse_args()
    results = {}
    for alignment in ('lattice', 'timestamp'):
        results[alignment] = run_mode(alignment, args.seconds, args.lag_ms, args.motion_timeout,
                                      args.drift_ppm, args.rate_hz)
        print(f'MODE {alignment} DONE', json.dumps(results[alignment], ensure_ascii=True), flush=True)
    path = ROOT / 'results/vio' / args.out
    path.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf8')
    print('SAVED', path, flush=True)


if __name__ == '__main__':
    main()
