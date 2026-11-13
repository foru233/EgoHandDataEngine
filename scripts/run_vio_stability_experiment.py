"""Isolated, serial VS030 experiments with explicit immutable inputs and telemetry."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'map_creat'))


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--worker', type=int)
    p.add_argument('--recording', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--calibration', type=Path)
    p.add_argument('--settings', type=Path)
    p.add_argument('--frames', type=int)
    p.add_argument('--phase-ms', type=float)
    # 显式分辨率覆盖：默认沿用录制 vio_timing.yaml 的 image_scale；
    # 受控对照用它分别喂 0.5(半) 与 1.0(全)，不改录制与正式配置。
    p.add_argument('--image-scale', type=float)
    p.add_argument('--disable-recovery', action='store_true')
    p.add_argument('--stereo-only', action='store_true')
    p.add_argument('--gauge-fix', action='store_true')
    p.add_argument('--gauge-constraint', action='store_true')
    p.add_argument('--cheirality-guard', action='store_true')
    p.add_argument('--diagnostics', action='store_true')
    p.add_argument('--black-from', type=int)
    p.add_argument('--black-count', type=int, default=0)
    p.add_argument('--start-stamp-ns', type=int)
    p.add_argument('--end-stamp-ns', type=int)
    return p


def worker(args):
    # This branch runs inside the isolated egohand-vio environment, without PyAV.
    from egohand.vio.native_bridge import NativeVioBridge
    from egohand.vio.live_subprocess import serve
    with socket.create_connection(('127.0.0.1', args.worker), timeout=120) as sock:
        sock.settimeout(None)
        try:
            with NativeVioBridge(ROOT / 'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                                 args.settings, library_path=args.library) as bridge:
                serve(sock, bridge, recovery_enabled=not args.disable_recovery)
        except Exception as exc:
            from egohand.vio.live_subprocess import report_error
            report_error(sock, exc)
            raise


def resolve_experiment_config(recording, calibration, settings, phase_ms=None, image_scale=None):
    """Validate the recording and distinguish verified timing from diagnostic overrides."""
    from egohand.vio.config_snapshot import load_config_snapshot, resolve_recording_config
    rec = Path(recording).resolve()
    # Check the recording descriptor and its original sidecar even when selecting
    # a different calibration/settings pair for an experiment.
    recorded = resolve_recording_config(rec)
    selected = load_config_snapshot(calibration=calibration, settings=settings,
        timing=rec/'vio_timing.yaml', quality=rec/'vio_quality.yaml')
    timing = dict(selected.timing)
    timing.pop('session_phase_verified', None)
    if image_scale is not None:
        if not math.isfinite(image_scale) or not 0 < image_scale <= 1:
            raise ValueError('--image-scale 须在 (0,1] 内')
        timing['image_scale'] = float(image_scale)
    model_path = rec/'vio_session_phase.json'
    evidence = {'status': 'unverified_manual', 'diagnostic_only': True,
                'model_path': None, 'model_sha256': None}
    if phase_ms is not None:
        if not math.isfinite(phase_ms) or not -150 <= phase_ms <= 200:
            raise ValueError('--phase-ms 超出有效范围')
        # An explicit override is an experiment, not a verified session model.
        timing.update(lattice_phase_mode='manual', lattice_phase_ms=float(phase_ms))
        evidence.update(status='explicit_fixed_phase_diagnostic', diagnostic_only=True)
    elif model_path.is_file():
        try:
            data = json.loads(model_path.read_text(encoding='utf8'))
            models = data['models']
            if data.get('schema_version') != 1 or len(models) != 1:
                raise ValueError('录制时序模型数量或版本无效')
            model = models[0]
            hashes = selected.describe()['hashes']
            verification = model['verification']
            if (model.get('schema_version') != 1 or verification.get('verified') is not True
                    or any(model['config_hashes'].get(k) != hashes[k] for k in ('calibration', 'settings'))):
                raise ValueError('录制时序模型与实验标定/ORB 配置不匹配')
            phase = float(verification['phase_ms'])
            if not math.isfinite(phase) or not -150 <= phase <= 200:
                raise ValueError('录制时序模型相位无效')
        except (OSError, json.JSONDecodeError, KeyError, TypeError, IndexError, AttributeError) as exc:
            raise ValueError('录制时序模型无法验证') from exc
        timing.update(lattice_phase_mode='session_auto', lattice_phase_ms=phase,
                      session_phase_verified=True)
        evidence.update(status='verified_matching_recording_model', diagnostic_only=False)
    elif timing.get('lattice_phase_mode') == 'session_auto':
        raise ValueError('录制缺少匹配的已核验时序模型；仅可显式指定诊断相位')
    if model_path.is_file():
        evidence.update(model_path=str(model_path),
                        model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest())
    evidence['recording_snapshot'] = recorded.describe()
    return selected, timing, evidence


def raw_input_manifest(recording):
    """Hash the exact recording files present under raw, without following links."""
    root = Path(recording).resolve() / 'raw'
    if not root.is_dir() or root.is_symlink():
        raise ValueError('录制缺少可信 raw 目录')
    files = {}
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise ValueError(f'raw 不允许符号链接：{path}')
        if not path.is_file():
            continue
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        files[path.relative_to(root).as_posix()] = {
            'size_bytes': path.stat().st_size, 'sha256': digest.hexdigest()}
    if not files:
        raise ValueError('录制 raw 目录为空')
    return files


def run(args):
    import cv2
    import numpy as np
    import psutil
    import yaml
    from atlas_replay import recording_frames
    from egohand.vio.live_runtime import PYTHON, _send_tracking_frame, _recv, vio_subprocess_environment
    from egohand.vio.pose_quality import PoseQualityGate, apply_recovery_quarantine
    from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry

    out = args.output.resolve()
    if out.exists():
        raise FileExistsError(out)
    rec = args.recording.resolve()
    cal = (args.calibration or rec / 'kalibr_camchain.yaml').resolve()
    settings = (args.settings or rec / 'vio_settings.yaml').resolve()
    config, timing, timing_evidence = resolve_experiment_config(
        rec, cal, settings, args.phase_ms, args.image_scale)
    geometry = prepare_vs030_vio_geometry(cal)
    quality_path = rec / 'vio_quality.yaml'
    from dataclasses import replace
    replace(config, timing=timing).validate_geometry(geometry)
    raw_before = raw_input_manifest(rec)
    out.mkdir(parents=True, exist_ok=False)
    options = yaml.safe_load(quality_path.read_text(encoding='utf8'))
    for k in list(options):
        if k == 'pose_continuity' or k.startswith('continuity_'):
            options.pop(k)
    gate = PoseQualityGate(**options, require_imu=not args.stereo_only)
    files = [cal, settings, rec / 'vio_timing.yaml', quality_path, args.library.resolve()]
    provenance = {
        'recording': str(rec), 'timing': timing, 'timing_evidence': timing_evidence,
        'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'raw_input_files_before': raw_before,
        'files': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in files],
        'python_sources': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in sorted((ROOT/'src/egohand/vio').glob('*.py'))},
        'gauge_fix': args.gauge_fix, 'recovery_enabled': not (args.disable_recovery or args.stereo_only),
        'gauge_constraint': args.gauge_constraint,
        'cheirality_guard': args.cheirality_guard,
        'sensor': 'stereo' if args.stereo_only else 'stereo_inertial',
        'synthetic_black_frames': {'from': args.black_from, 'count': args.black_count},
        'image_range_ns': [args.start_stamp_ns,args.end_stamp_ns],
        'coordinates': 'GUI cam0 raw camera; T_world_from_camera; metres from stereo baseline',
    }
    (out / 'inputs.json').write_text(json.dumps(provenance, indent=2), encoding='utf8')
    env = vio_subprocess_environment()
    env['EGOHAND_VIO_GAUGE_FIX'] = '1' if args.gauge_fix else '0'
    env['EGOHAND_VIO_GAUGE_CONSTRAINT'] = '1' if args.gauge_constraint else '0'
    env['EGOHAND_VIO_CHEIRALITY_GUARD'] = '1' if args.cheirality_guard else '0'
    env['EGOHAND_VIO_STEREO_ONLY'] = '1' if args.stereo_only else '0'
    if args.diagnostics:
        (out / 'native_diagnostics').mkdir()
        env['EGOHAND_VIO_DIAGNOSTICS'] = str(out / 'native_diagnostics')
    else:
        env.pop('EGOHAND_VIO_DIAGNOSTICS', None)
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0)); listener.listen(1); listener.settimeout(1)
    command = [str(PYTHON), str(Path(__file__).resolve()), '--worker', str(listener.getsockname()[1]),
               '--output', str(out), '--library', str(args.library.resolve()), '--settings', str(settings)]
    if args.disable_recovery or args.stereo_only:
        command.append('--disable-recovery')
    log = (out / 'run.log').open('w', encoding='utf8')
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env,
                            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    monitor = psutil.Process(proc.pid)
    conn = None; count = 0; started = time.perf_counter(); latencies = []; error = None

    def read_message():
        n = struct.unpack('<I', _recv(conn, 4))[0]
        if n > 1_000_000:
            raise ValueError('Invalid worker response length')
        r = json.loads(_recv(conn, n))
        if r.get('type') == 'error':
            raise RuntimeError(r.get('message'))
        return r

    try:
        deadline = time.monotonic() + 120
        while conn is None:
            try:
                conn, _ = listener.accept()
            except socket.timeout:
                if proc.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Worker startup failed; inspect run.log')
        conn.settimeout(120)
        read_message()
        with (out / 'poses.jsonl').open('w', encoding='utf8', buffering=1) as poses:
            for stamp, left, right, aligned_imu in recording_frames(rec, geometry, timing,
                                                                   use_imu=True):
                if args.start_stamp_ns is not None and stamp < args.start_stamp_ns:
                    continue
                if args.end_stamp_ns is not None and stamp > args.end_stamp_ns:
                    break
                if args.frames is not None and count >= args.frames:
                    break
                # Select exactly the same image timestamps in both sensor modes.
                imu = [] if args.stereo_only else aligned_imu
                if count == 0:
                    imu = []
                if args.black_from is not None and args.black_from <= count < args.black_from + args.black_count:
                    left = np.zeros_like(left)
                    right = np.zeros_like(right)
                tick = time.perf_counter()
                _send_tracking_frame(conn, stamp, left, right, imu)
                result = read_message()
                latency = (time.perf_counter() - tick) * 1000
                latencies.append(latency)
                if result.get('matrix') is not None:
                    result['matrix'] = (np.asarray(result['matrix']) @ geometry.T_orb_left_from_gui_left).tolist()
                result = apply_recovery_quarantine(gate.check(result))
                result['replay'] = {'frame_index': count, 'imu_samples_in_interval': len(imu)}
                result['timing'] = {'track_ms': latency, 'elapsed_s': time.perf_counter() - started}
                result['imu_interval'] = {'first_ns': imu[0][0] if imu else None,
                                          'last_ns': imu[-1][0] if imu else None,
                                          'max_gap_ns': max((b[0] - a[0] for a, b in zip(imu, imu[1:])), default=None)}
                if count % 25 == 0:
                    mem = monitor.memory_info()
                    result['resources'] = {'rss_bytes': mem.rss, 'vms_bytes': mem.vms,
                                            'threads': monitor.num_threads(),
                                            'cpu_seconds': sum(monitor.cpu_times()[:2])}
                result['image_diagnostics'] = {
                    'width': int(left.shape[1]), 'height': int(left.shape[0]),
                    'left_mean': float(left.mean()), 'right_mean': float(right.mean()),
                    'left_laplacian_variance': float(cv2.Laplacian(left, cv2.CV_32F).var()),
                    'right_laplacian_variance': float(cv2.Laplacian(right, cv2.CV_32F).var()),
                }
                poses.write(json.dumps(result, allow_nan=False) + '\n')
                count += 1
                if count % 100 == 0:
                    print(json.dumps({'frames': count, 'elapsed_s': round(time.perf_counter()-started, 1)}), flush=True)
    except Exception as exc:
        error = repr(exc)
    finally:
        if conn is not None:
            conn.close()
        listener.close()
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.terminate(); proc.wait(timeout=10)
        log.close()
        try:
            raw_after = raw_input_manifest(rec)
            raw_integrity = {'status': 'unchanged' if raw_after == raw_before else 'changed',
                             'files_after': raw_after}
            if raw_after != raw_before and error is None:
                error = 'raw_input_changed_during_experiment'
        except (OSError, ValueError) as exc:
            raw_integrity = {'status': 'unavailable', 'reason': str(exc)}
            if error is None:
                error = 'raw_input_integrity_unavailable'
        report = {'frames': count, 'elapsed_s': time.perf_counter()-started,
                  'error': error, 'worker_exit_code': proc.returncode, 'raw_input_integrity': raw_integrity,
                  'track_ms_p50_p95': np.percentile(latencies, [50, 95]).tolist() if latencies else None}
        (out / 'experiment.json').write_text(json.dumps(report, indent=2), encoding='utf8')
        print(json.dumps(report), flush=True)
    return int(error is not None or proc.returncode != 0)


if __name__ == '__main__':
    args = parser().parse_args()
    if args.worker:
        worker(args)
    else:
        raise SystemExit(run(args))
