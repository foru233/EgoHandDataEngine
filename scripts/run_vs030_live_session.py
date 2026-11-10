"""无 GUI 运行一次 VS030 实时 VIO 会话，输出汇总 JSON（用于 A/B 对比）。

用法：python scripts/run_vs030_live_session.py --seconds 90 --out vs030_live_lattice.json
运行顺序：打开相机 -> 启动 ORB 子进程（加载词典）-> 就绪后开始收流 -> 提交帧。
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import yaml
from egohand.sources.vendor_xdsdk import XdSdkStereoSource  # noqa: E402
from egohand.vio.live_runtime import LiveVio  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=90.0)
    parser.add_argument('--out', default='vs030_live_session.json')
    parser.add_argument('--ready-timeout', type=float, default=120.0)
    parser.add_argument('--alignment', choices=['lattice', 'timestamp'])
    parser.add_argument('--lag-ms', type=float)
    parser.add_argument('--drift-ppm', type=float)
    parser.add_argument('--rate-hz', type=float)
    parser.add_argument('--wait-motion', action='store_true')
    parser.add_argument('--motion-timeout', type=float, default=300.0)
    args = parser.parse_args()
    timing = yaml.safe_load((ROOT / 'configs/vio/live_timing.yaml').read_text(encoding='utf8'))
    if args.alignment:
        timing['imu_alignment'] = args.alignment
    if args.lag_ms is not None:
        timing['camera_to_imu_lag_ms'] = args.lag_ms
    if args.drift_ppm is not None:
        timing['camera_to_imu_drift_ppm'] = args.drift_ppm
    if args.rate_hz is not None:
        timing['imu_rate_hz'] = args.rate_hz

    source = XdSdkStereoSource()
    records = []
    errors = []

    def on_status(message):
        if message.get('type') == 'error':
            errors.append(message.get('message'))
        elif message.get('type') == 'pose':
            records.append(message)

    vio = LiveVio(source, on_status, timing=timing)
    started = time.monotonic()
    try:
        source.open()
        vio.start()
        if not vio.wait_ready(args.ready_timeout):
            raise RuntimeError(vio.startup_error or 'ORB-SLAM3 未就绪')
        source.start()
        if args.wait_motion:
            print('WAITING_MOTION', flush=True)
        stream_start = time.monotonic()
        deadline = None
        record_start_index = 0
        frames = 0
        while True:
            frame = source.read()
            if frame is None:
                break
            frames += 1
            vio.submit(frame)
            if deadline is None:
                if not args.wait_motion:
                    deadline = time.monotonic() + args.seconds
                elif vio.motion_event.is_set():
                    print('MOTION_DETECTED', flush=True)
                    record_start_index = len(records)
                    deadline = time.monotonic() + args.seconds
                elif time.monotonic() - stream_start > args.motion_timeout:
                    raise RuntimeError('等待相机运动超时；未开始计时')
            elif time.monotonic() >= deadline:
                break
        records = records[record_start_index:]
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

    epochs = sorted({r.get('map_epoch') for r in records if r.get('map_epoch') is not None})
    states = {}
    quality = {}
    longest_ok = current_ok = 0
    first_init = None
    for index, record in enumerate(records):
        state = record.get('tracking_state')
        states[state] = states.get(state, 0) + 1
        quality[record.get('quality_status')] = quality.get(record.get('quality_status'), 0) + 1
        if record.get('imu_initialized'):
            if first_init is None:
                first_init = index
            current_ok = current_ok + 1 if state == 'OK' else 0
            longest_ok = max(longest_ok, current_ok)
    summary = {
        'seconds_requested': args.seconds,
        'wall_seconds': round(time.monotonic() - started, 1),
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
        'longest_ok_streak_after_init': longest_ok,
        'imu_alignment': records[-1].get('imu_alignment') if records else None,
        'timing': {k: timing.get(k) for k in ('imu_alignment', 'camera_to_imu_lag_ms', 'imu_rate_hz', 'camera_to_imu_drift_ppm')},
        'imu_lattice': records[-1].get('imu_lattice') if records else None,
        'errors': errors,
    }
    out = ROOT / 'results/vio' / args.out
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf8')
    print('RESULT', json.dumps(summary, ensure_ascii=True), flush=True)
    print('SAVED', out, flush=True)


if __name__ == '__main__':
    main()
