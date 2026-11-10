"""同一段实机采集，离线重放多种 IMU 对齐参数（运动完全相同，消除逐次差异）。

流程：
1) 等运动触发，抓取 60 s 已校正 mono8 双目的 JPEG（10 Hz）与原始 IMU 包；
2) 依次用 lattice(多个速率) / timestamp(-85 ms) 重放给同一个 ORB-SLAM3 桥；
3) 输出各模式的跟踪状态、IMU 初始化、epoch 数。

注意：重放比实时快，且图像经过 JPEG 有损压缩；结论只用于相对比较。
"""
import argparse
import json
import os
import queue
import socket
import struct
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.sources.vendor_xdsdk import XdSdkStereoSource  # noqa: E402
from egohand.vio.live_runtime import CandidateImuBuffer, HEADER, IMU, PYTHON, _recv  # noqa: E402
from egohand.vio.imu_lattice import LatticeImuBuffer  # noqa: E402
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry  # noqa: E402
from egohand.vio.vs030_imu_candidate import parse_candidate_payload  # noqa: E402

JPEG_QUALITY = 95
FRAME_INTERVAL_NS = 100_000_000


def read_message(sock):
    size = struct.unpack('<I', _recv(sock, 4))[0]
    if size > 100000:
        raise ValueError('结果长度异常')
    return json.loads(_recv(sock, size).decode('utf8'))


def capture(seconds, motion_timeout):
    source = XdSdkStereoSource()
    source.enable_imu_capture()
    geometry = prepare_vs030_vio_geometry(ROOT / '标定结果/1/data3-camchain-imucam.yaml')
    rect = geometry.rectifier
    frames = []
    packets = []
    error = None
    last = 0
    started = None
    motion = 0
    try:
        source.open()
        source.start()
        deadline = time.monotonic() + motion_timeout
        print('CAPTURE_WAITING_MOTION', flush=True)
        while time.monotonic() < deadline:
            frame = source.read()
            if frame is None:
                break
            while True:
                try:
                    item = source.imu_vio_queue.get_nowait()
                except queue.Empty:
                    break
                payload = item[-1]
                if started is None:
                    for row in parse_candidate_payload(payload):
                        if max(abs(row.values[3]), abs(row.values[4]), abs(row.values[5])) > 0.15:
                            motion += 1
                    packets.append(payload)
                else:
                    packets.append(payload)
            if started is None:
                if motion >= 8:
                    started = time.monotonic()
                    print('CAPTURE_MOTION_DETECTED', flush=True)
                continue
            t = int(frame.metadata['sdk_device_timestamp_raw']) * 1000
            if t - last < FRAME_INTERVAL_NS:
                continue
            left = cv2.remap(frame.right_image, *rect.maps_left, cv2.INTER_LINEAR)
            right = cv2.remap(frame.left_image, *rect.maps_right, cv2.INTER_LINEAR)
            gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            ok_l, enc_l = cv2.imencode('.jpg', gray_l, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            ok_r, enc_r = cv2.imencode('.jpg', gray_r, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not (ok_l and ok_r):
                raise RuntimeError('内存图像压缩失败')
            frames.append((t, enc_l, enc_r))
            last = t
            if time.monotonic() - started >= seconds:
                break
    except Exception as exc:  # noqa: BLE001
        error = repr(exc)
    finally:
        source.close()
    return frames, packets, error, source.sdk_diagnostics


def replay(frames, packets, mode, rate_hz=480.0, drift_ppm=657.0, lag_ms=-85.0):
    if mode == 'lattice':
        buffer = LatticeImuBuffer(sample_rate_hz=rate_hz, camera_to_imu_drift_ppm=drift_ppm, capacity=200000)
    else:
        buffer = CandidateImuBuffer(camera_to_imu_lag_ns=round(lag_ms * 1e6))
    for packet in packets:
        buffer.add(packet)
    if mode == 'lattice':
        buffer.trim_before(frames[0][0])
    listener = socket.socket(); listener.bind(('127.0.0.1', 0)); listener.listen(1); listener.settimeout(30)
    env = os.environ.copy(); env['PYTHONPATH'] = str(ROOT / 'src') + os.pathsep + env.get('PYTHONPATH', '')
    log_path = ROOT / 'logs' / f'vio_replay_{mode}_{rate_hz:.1f}_{time.time_ns()}.log'
    count = Counter(); epochs = set(); error = None
    with log_path.open('w', encoding='utf8') as log:
        proc = subprocess.Popen([str(PYTHON), '-m', 'egohand.vio.live_subprocess', str(listener.getsockname()[1])],
                                stdout=log, stderr=subprocess.STDOUT, env=env,
                                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        conn = None
        try:
            conn, _ = listener.accept(); conn.settimeout(60)
            message = read_message(conn)
            if message.get('type') != 'ready':
                raise RuntimeError(str(message))
            print('REPLAY_READY', mode, rate_hz, flush=True)
            previous = None
            for t, left, right in frames:
                if previous is None:
                    measures = []
                elif mode == 'lattice':
                    measures = buffer.consume(previous, t)
                    if measures is None:
                        raise RuntimeError('格点样本不足')
                else:
                    measures = buffer.interval(previous, t)
                l = cv2.imdecode(left, cv2.IMREAD_GRAYSCALE); r = cv2.imdecode(right, cv2.IMREAD_GRAYSCALE)
                if l is None or r is None:
                    raise RuntimeError('内存图像解码失败')
                conn.sendall(HEADER.pack(t, l.shape[1], l.shape[0], len(measures)) +
                             l.tobytes() + r.tobytes() + b''.join(IMU.pack(*m) for m in measures))
                response = read_message(conn)
                if response.get('type') == 'error':
                    raise RuntimeError(response.get('message'))
                count[response.get('tracking_state', 'UNKNOWN')] += 1
                count['imu_initialized'] += bool(response.get('imu_initialized'))
                count['pose_valid'] += bool(response.get('pose_valid'))
                epochs.add(response.get('map_epoch'))
                previous = t
        except Exception as exc:  # noqa: BLE001
            error = str(exc)
        finally:
            if conn:
                conn.close()
            listener.close()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.terminate(); proc.wait(timeout=5)
    result = {'mode': mode, 'rate_hz': rate_hz, 'drift_ppm': drift_ppm, 'lag_ms': lag_ms,
              'counts': dict(count), 'map_epochs': len(epochs), 'error': error, 'log': str(log_path)}
    print('REPLAY_DONE', json.dumps(result, ensure_ascii=True), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--motion-timeout', type=float, default=600.0)
    parser.add_argument('--out', default='vs030_alignment_replay.json')
    args = parser.parse_args()
    frames, packets, error, sdk = capture(args.seconds, args.motion_timeout)
    print('CAPTURE_DONE', len(frames), len(packets), ascii(error), flush=True)
    if error or len(frames) < 40:
        raise SystemExit(f'采集不足：{error}')
    modes = [
        ('lattice', 480.0, 0.0, -85.0),
        ('lattice', 480.0, 657.0, -85.0),
        ('lattice', 480.0, -1700.0, -85.0),
        ('timestamp', 480.0, 657.0, -85.0),
    ]
    results = []
    for mode, rate, drift, lag in modes:
        results.append(replay(frames, packets, mode, rate, drift, lag))
    report = {'frames': len(frames), 'imu_packets': len(packets), 'sdk': sdk,
              'note': '同一段内存数据重放；JPEG 有损、速度非实时，仅作相对比较。',
              'results': results}
    path = ROOT / 'results/vio' / args.out
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print('SAVED', path, flush=True)


if __name__ == '__main__':
    main()
