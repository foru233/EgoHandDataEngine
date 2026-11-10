"""用录制的 sdk_records.bin（原始 H265 + IMU）离线对比 IMU 对齐方式。

同一份数据分别以 lattice(480Hz) 与 timestamp(-85ms) 喂给 ORB-SLAM3，
比较跟踪状态、IMU 初始化与 map epoch。输出 results/vio/vs030_recording_vio_compare.json。
"""
import argparse
import json
import os
import socket
import struct
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

import av
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.vio.live_runtime import CandidateImuBuffer, HEADER, IMU, PYTHON, _recv  # noqa: E402
from egohand.vio.imu_lattice import LatticeImuBuffer  # noqa: E402
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry  # noqa: E402

REC = struct.Struct('<IHHIIQQQQQQ')
MAGIC = 0x58444652


def read_message(sock):
    size = struct.unpack('<I', _recv(sock, 4))[0]
    if size > 100000:
        raise ValueError('结果长度异常')
    return json.loads(_recv(sock, size).decode('utf8'))


def load_records(path):
    pairs = {}
    imu_payloads = []
    with open(path, 'rb') as stream:
        while True:
            raw = stream.read(REC.size)
            if not raw:
                break
            if len(raw) != REC.size:
                raise RuntimeError('SDK 记录头不完整')
            magic, version, header_size, sink, size, sequence, device_ts, cap, arr, h1, h2 = REC.unpack(raw)
            if magic != MAGIC or version != 2:
                raise RuntimeError(f'SDK 记录格式异常 magic={magic:#x} version={version}')
            payload = stream.read(size)
            if len(payload) != size:
                raise RuntimeError('SDK 记录载荷不完整')
            if sink == 7:
                imu_payloads.append(payload)
            elif sink in (4, 5):
                entry = pairs.setdefault(sequence, {})
                entry[sink] = (device_ts, payload)
    ordered = []
    for sequence in sorted(pairs):
        entry = pairs[sequence]
        if 4 in entry and 5 in entry and entry[4][0] == entry[5][0]:
            ordered.append((sequence, entry[4][0], entry[4][1], entry[5][1]))
    return ordered, imu_payloads


def replay(pairs, imu_payloads, geometry, mode, rate_hz=480.0, drift_ppm=0.0, lag_ms=-85.0, scale=1.0, config=None):
    if mode == 'lattice':
        buffer = LatticeImuBuffer(sample_rate_hz=rate_hz, camera_to_imu_drift_ppm=drift_ppm, capacity=200000)
    else:
        buffer = CandidateImuBuffer(camera_to_imu_lag_ns=round(lag_ms * 1e6))
    for payload in imu_payloads:
        buffer.add(payload)
    # 录制版：数据已按到达顺序完整预载，直接从第一条样本开始按计数消费；
    # 不按时间戳裁剪（相机/IMU 标签域可能相差数秒），也不清空。
    rect = geometry.rectifier
    maps_left, maps_right = rect.maps_left, rect.maps_right
    listener = socket.socket(); listener.bind(('127.0.0.1', 0)); listener.listen(1); listener.settimeout(30)
    env = os.environ.copy(); env['PYTHONPATH'] = str(ROOT / 'src') + os.pathsep + env.get('PYTHONPATH', '')
    log_path = ROOT / 'logs' / f'vio_recording_{mode}_{int(time.time())}.log'
    counts = Counter(); epochs = set(); error = None; durations = []
    decoders = {4: av.CodecContext.create('hevc', 'r'), 5: av.CodecContext.create('hevc', 'r')}
    with log_path.open('w', encoding='utf8') as log:
        proc = subprocess.Popen([str(PYTHON), '-m', 'egohand.vio.live_subprocess', str(listener.getsockname()[1])],
                                stdout=log, stderr=subprocess.STDOUT, env=env,
                                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        conn = None
        try:
            conn, _ = listener.accept(); conn.settimeout(120)
            message = read_message(conn)
            if message.get('type') != 'ready':
                raise RuntimeError(str(message))
            print('REPLAY_READY', mode, flush=True)
            previous = None
            for sequence, device_ts, payload4, payload5 in pairs:
                frames5 = decoders[5].decode(av.Packet(payload5))
                frames4 = decoders[4].decode(av.Packet(payload4))
                if len(frames5) != 1 or len(frames4) != 1:
                    continue
                t = device_ts * 1000
                if previous is None:
                    measures = []
                elif mode == 'lattice':
                    measures = buffer.consume(previous, t)
                    if measures is None:
                        counts['skipped_no_imu'] += 1
                        previous = t
                        continue
                else:
                    measures = buffer.interval(previous, t)
                left = cv2.remap(frames5[0].to_ndarray(format='bgr24'), *maps_left, cv2.INTER_LINEAR)
                right = cv2.remap(frames4[0].to_ndarray(format='bgr24'), *maps_right, cv2.INTER_LINEAR)
                left = np.ascontiguousarray(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY))
                right = np.ascontiguousarray(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY))
                if scale != 1.0:
                    left = cv2.resize(left, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                    right = cv2.resize(right, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                started = time.perf_counter()
                conn.sendall(HEADER.pack(t, left.shape[1], left.shape[0], len(measures)) +
                             left.tobytes() + right.tobytes() + b''.join(IMU.pack(*m) for m in measures))
                response = read_message(conn)
                durations.append((time.perf_counter() - started) * 1000.0)
                if response.get('type') == 'error':
                    raise RuntimeError(response.get('message'))
                counts[response.get('tracking_state', 'UNKNOWN')] += 1
                counts['imu_initialized'] += bool(response.get('imu_initialized'))
                counts['pose_valid'] += bool(response.get('pose_valid'))
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
    import numpy as _np
    result = {'mode': mode, 'rate_hz': rate_hz, 'drift_ppm': drift_ppm, 'lag_ms': lag_ms,
              'counts': dict(counts), 'map_epochs': len(epochs), 'error': error, 'log': str(log_path),
              'frame_ms_median': round(float(_np.median(durations)), 1) if durations else None,
              'frame_ms_p90': round(float(_np.percentile(durations, 90)), 1) if durations else None,
              'frames_replayed': len(durations)}
    print('REPLAY_DONE', json.dumps(result, ensure_ascii=True), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recording', help='recordings/session_... 目录')
    parser.add_argument('--out', default='vs030_recording_vio_compare.json')
    parser.add_argument('--mode', choices=['lattice', 'timestamp', 'both'], default='both')
    parser.add_argument('--scale', type=float, default=1.0)
    parser.add_argument('--config', default=None, help='ORB 配置路径（默认 vs030_rectified_preliminary.yaml）')
    args = parser.parse_args()
    session = Path(args.recording)
    pairs, imu_payloads = load_records(session / 'raw/sdk_records.bin')
    print('pairs', len(pairs), 'imu_payloads', len(imu_payloads), flush=True)
    geometry = prepare_vs030_vio_geometry(session / 'kalibr_camchain.yaml')
    # 若指定 --config，用该配置替换 live_subprocess 的默认配置（通过环境变量传递）
    if args.config:
        os.environ['EGOHAND_VIO_CONFIG'] = str(Path(args.config).resolve())
    results = []
    if args.mode in ('lattice', 'both'):
        results.append(replay(pairs, imu_payloads, geometry, 'lattice', 480.0, 0.0, scale=args.scale, config=args.config))
    if args.mode in ('timestamp', 'both'):
        results.append(replay(pairs, imu_payloads, geometry, 'timestamp', 480.0, 0.0, -85.0))
    report = {'recording': str(session), 'pairs': len(pairs), 'imu_packets': len(imu_payloads),
              'note': '同一份录制数据重放；重建时按原始 H265 解码并校正，无 JPEG 有损环节。',
              'results': results}
    path = ROOT / 'results/vio' / args.out
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print('SAVED', path, flush=True)


if __name__ == '__main__':
    main()
