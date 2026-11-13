"""确定性离线 VIO 回放：录制目录 → 与实时链路一致的桥/几何/门控 → poses.jsonl / replay.json。

设计依据：`handoff/GUI录制与离线回放_设计_20261110.md` 第 4.4 节。约束与实时链路保持一致：

- 帧与 IMU 都由 `storage.sdk_recording` 的只读生成器流式读取，不在内存中保留整段录制；
- 帧序 = 录制顺序（确定性）；首帧必须提交 0 条 IMU（native bridge / ORB-SLAM3 约束）；
- 相机时间 `stamp = device_ts * 1000`（沿用实时候选假设，未获厂商确认）；
- IMU 对齐支持 `timestamp` 与 `lattice`，与 `LiveVio` 相同；`stride` 跳帧时 lattice 用
  `skip_span` 丢弃被跳帧区间的样本并保持计数相位，timestamp 模式直接跨区间取样本；
- 位姿矩阵先乘 `T_orb_left_from_gui_left`，再过 `PoseQualityGate`，字段与实时审计一致；
- 录制 IMU 若比视频先结束（实测尾部约 50 ms 的空档），不插值/不外推：与实时链路
  “样本不足跳过该帧”一致地结束回放，在 `replay.json.imu_coverage_exhausted` 与
  `warnings` 中显式记录已处理的帧数；
- 子进程、socket、文件在成功/失败/取消路径上都会清理；`replay.json` 始终落盘。

本模块只写 `<output_dir>`，不写实时日志目录。
"""
from __future__ import annotations

from dataclasses import replace
import json
import os
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from .imu_lattice import LatticeImuBuffer, PacketLatticeImuBuffer, packet_lattice_phase_ns
from .vs030_imu_candidate import parse_candidate_payload
from .live_runtime import CALIBRATION, CandidateImuBuffer, PYTHON, ROOT, _recv, _send_tracking_frame, vio_subprocess_environment
from .recovery_contract import (PROTOCOL, RecoveryProtocolError, application_feedback,
                                send_message, negotiate_recovery)
from .hybrid_quality import make_quality_gate
from .localization_state import LocalizationPipeline
from .vs030_geometry import prepare_vs030_vio_geometry
from .config_snapshot import resolve_recording_config
from .modes import DEFAULT_MODE, STEREO, HYBRID, validate_mode, apply_mode_environment

TIMING_CONFIG = ROOT / 'configs/vio/live_timing.yaml'
QUALITY_CONFIG = ROOT / 'configs/vio/live_quality.yaml'
HANDSHAKE_TIMEOUT_S = 60.0
TRACK_TIMEOUT_S = 120.0
IMU_LATTICE_CAPACITY = 20000
MAX_CAMERA_TO_IMU_LAG_MS = 1000.0
MAX_IMU_GAP_NS = 30_000_000
PROGRESS_EVERY_FRAMES = 25
PENDING_FIRST_FRAME_LIMIT = 5000


class ReplayCancelled(Exception):
    """回放被调用方取消（不是失败）。"""


class ImuCoverageEnd(Exception):
    """录制 IMU 数据比视频先结束，无法再为后续相机帧提供惯性样本。"""


def sdk_recording_module():
    """延迟 import `storage.sdk_recording`，缺失时给出中文原因而不是导入期崩溃。"""
    try:
        from ..storage import sdk_recording
    except Exception as exc:  # noqa: BLE001 - 任何导入失败都要转成可读错误
        raise RuntimeError(f'录制解析模块 storage.sdk_recording 不可用：{exc}') from exc
    for name in ('iter_paired_frames', 'iter_imu_rows'):
        if not hasattr(sdk_recording, name):
            raise RuntimeError(f'storage.sdk_recording 缺少回放所需接口：{name}')
    return sdk_recording


def resolve_timing(recording_dir, override=None):
    """解析 IMU 对齐参数：显式参数 > 录制内 vio_timing.yaml > 仓库配置。

    录制目录里的 `vio_timing.yaml` 是录制时 `Recorder` 复制的参数快照，优先使用它可以
    让回放复现录制当时的口径；显式 `override`（CLI/GUI 选择）逐字段覆盖。
    """
    recording_dir = Path(recording_dir)
    base = {}
    source = None
    for path in (recording_dir / 'vio_timing.yaml', TIMING_CONFIG):
        if path.is_file():
            try:
                data = yaml.safe_load(path.read_text(encoding='utf8')) or {}
            except (OSError, yaml.YAMLError) as exc:
                raise ValueError(f'无法读取时间配置 {path}：{exc}') from exc
            if not isinstance(data, dict):
                raise ValueError(f'时间配置格式无效：{path}')
            base = dict(data)
            source = str(path)
            break
    if not base:
        raise FileNotFoundError('未找到 VIO 时间配置（录制内 vio_timing.yaml 或 configs/vio/live_timing.yaml）')
    if override:
        base.update({key: value for key, value in dict(override).items() if value is not None})
        source = f'{source}+参数覆盖'
    lag_ms = float(base.get('camera_to_imu_lag_ms', 0.0))
    if not np.isfinite(lag_ms) or abs(lag_ms) > MAX_CAMERA_TO_IMU_LAG_MS:
        raise ValueError('相机—IMU 时间偏移无效')
    alignment = str(base.get('imu_alignment', 'timestamp')).strip().lower()
    if alignment not in ('timestamp', 'lattice'):
        raise ValueError(f'未知的 IMU 对齐方式：{alignment}')
    rate_hz = float(base.get('imu_rate_hz', 480.0))
    if not np.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError('IMU 采样率无效')
    drift_ppm = float(base.get('camera_to_imu_drift_ppm', 0.0))
    if not np.isfinite(drift_ppm) or abs(drift_ppm) >= 100_000:
        raise ValueError('IMU 漂移参数异常')
    lattice_reference = str(base.get('lattice_reference', 'packet'))
    if lattice_reference not in ('packet', 'count'):
        raise ValueError('未知的格点参考方式')
    return {
        'camera_to_imu_lag_ms': lag_ms,
        'imu_alignment': alignment,
        'imu_rate_hz': rate_hz,
        'camera_to_imu_drift_ppm': drift_ppm,
        'lattice_reference': lattice_reference,
        'lattice_phase_ms': packet_lattice_phase_ns(base) / 1e6,
        'source': source,
    }


class ReplayBridge:
    """本机 TCP + `egohand.vio.live_subprocess` 子进程，协议与实时链路完全相同。"""

    def __init__(self, log_path, *, handshake_timeout=HANDSHAKE_TIMEOUT_S, timeout=TRACK_TIMEOUT_S,
                 settings_path=None, mode=DEFAULT_MODE, worker=None, library_path=None, coordinated_stereo=False):
        self.mode=validate_mode(mode)
        self.library_path = None if library_path is None else Path(library_path).resolve(strict=True)
        if self.library_path is not None and worker is not None:
            raise ValueError('显式离线DLL须使用独立worker，不能复用GUI估计器')
        self.coordinated_stereo=bool(coordinated_stereo)
        if self.coordinated_stereo and (self.mode != STEREO or worker is not None):
            raise ValueError('显式双目单所有者协议只支持独立离线Stereo会话')
        # A shared worker keeps one ORB process (and therefore the already-loaded vocabulary)
        # alive for the whole GUI lifetime, so a replay must NOT spawn its own one-shot
        # `live_subprocess <port>`: that re-reads ORBvoc.txt (~145 MB) on every replay.
        self.worker = worker
        self.log_path = Path(log_path)
        self.handshake_timeout = float(handshake_timeout)
        self.timeout = float(timeout)
        self.listener = None
        self.proc = None
        self.exit_code = None
        self.sock = None
        self.log = None
        # True when this replay reused the GUI's already-loaded ORB process (no vocabulary read).
        self.reused_worker = False
        self._closed = False
        self._lock = threading.Lock()
        self.settings_path = None if settings_path is None else Path(settings_path).resolve()
        self.recovery_protocol = 1
        if self.mode=='hybrid':
            from .hybrid_settings import visual_bootstrap_settings
            if self.settings_path is None:
                from .config_snapshot import resolve_live_config
                self.settings_path=resolve_live_config().settings
            self.settings_path=visual_bootstrap_settings(self.settings_path)

    def start(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(0.5)
        self.listener = listener
        reuse = self.worker is not None
        if reuse and self.settings_path is None:
            # request_session 需要 settings 才能算出进程身份；缺省时回退到实时配置，
            # 而不是让 None 传下去在 read_bytes() 上炸掉。
            from .config_snapshot import resolve_live_config
            self.settings_path = resolve_live_config().settings
        if not reuse:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log = self.log_path.open('w', encoding='utf8')
            env = apply_mode_environment(vio_subprocess_environment(recovery_protocol=PROTOCOL), self.mode)
            if self.library_path is not None:
                env['EGOHAND_VIO_LIBRARY'] = str(self.library_path)
            if self.coordinated_stereo:
                env['EGOHAND_VIO_DISABLE_RECOVERY'] = '0'
            if self.settings_path is not None:
                env['EGOHAND_VIO_CONFIG'] = str(self.settings_path)
            self.proc = subprocess.Popen(
                [str(PYTHON), '-m', 'egohand.vio.live_subprocess', str(listener.getsockname()[1])],
                stdout=self.log, stderr=subprocess.STDOUT, env=env,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0),
            )
        else:
            # Ask the persistent `--live-sessions` worker for a session on our port. Its stdout
            # belongs to the GUI lifetime, so this replay must not truncate or own a log file.
            try:
                _proc, self.reused_worker, _log = self.worker.request_session(
                    listener.getsockname()[1], self.settings_path,
                    self.log_path.parent.name, mode=self.mode)
            except Exception:
                self._shutdown_socket()
                raise
        deadline = time.monotonic() + self.handshake_timeout
        while True:
            try:
                self.sock, _ = listener.accept()
                break
            except socket.timeout:
                if self._closed:
                    raise ReplayCancelled() from None
                if self.proc is not None and self.proc.poll() is not None:
                    raise RuntimeError(f'VIO 子进程启动失败，详见 {self.log_path}')
                if time.monotonic() > deadline:
                    raise TimeoutError(f'VIO 子进程连接超时（{self.handshake_timeout:.0f}s）')
            except OSError:
                if self._closed:
                    raise ReplayCancelled() from None
                raise
        if self._closed:
            raise ReplayCancelled()
        self.sock.settimeout(self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        message = self._read_message()
        if message.get('type') != 'ready':
            raise RuntimeError(message.get('message', 'VIO 初始化失败'))
        self.recovery_protocol=negotiate_recovery(message,
            requested=int(os.environ.get('EGOHAND_VIO_RECOVERY_PROTOCOL',str(PROTOCOL))),
            enabled=self.coordinated_stereo or (self.mode!='stereo' and os.environ.get('EGOHAND_VIO_DISABLE_RECOVERY')!='1'))

    def _read_message(self):
        length = struct.unpack('<I', _recv(self.sock, 4))[0]
        if length > 100000:
            raise ValueError('VIO 响应过大')
        return json.loads(_recv(self.sock, length).decode('utf8'))

    def track(self, stamp, left_gray, right_gray, measurements, *, reset_imu_alignment=False):
        if self.sock is None:
            raise ReplayCancelled()
        _send_tracking_frame(self.sock, stamp, left_gray, right_gray, measurements,
                             recovery_protocol=self.recovery_protocol,
                             reset_imu_alignment=reset_imu_alignment)
        result = self._read_message()
        if result.get('type') == 'error':
            raise RuntimeError(result.get('message') or 'VIO 子进程返回错误')
        return result

    def commit_recovery(self,result):
        if self.recovery_protocol!=PROTOCOL:
            raise RecoveryProtocolError('worker does not support committed recovery')
        send_message(self.sock,application_feedback(result))
        reply=self._read_message()
        if reply.get('type')!='recovery_committed':
            raise RecoveryProtocolError(reply.get('message','recovery commit missing'))
        return reply.get('recovery_preaccept')

    def abort(self):
        """取消时只断开 socket（不阻塞等待子进程），由 run() 的收尾统一回收。"""
        self._closed = True
        self._shutdown_socket()

    def close(self):
        self._closed = True
        self._shutdown_socket()
        with self._lock:
            self._reap()

    def _shutdown_socket(self):
        sock, self.sock = self.sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        listener, self.listener = self.listener, None
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass

    def _reap(self):
        proc, self.proc = self.proc, None
        if proc is not None:
            try:
                self.exit_code=proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    self.exit_code=proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    self.exit_code=proc.wait(timeout=3)
        log, self.log = self.log, None
        if log is not None:
            try:
                log.flush()
                log.close()
            except OSError:
                pass


class _ImuFeed:
    """把 `iter_imu_rows` 的行流按需送进对齐缓冲；记录读取统计供报告使用。"""

    def __init__(self, rows, buffer, alignment):
        self._rows = iter(rows)
        self._buffer = buffer
        self._alignment = alignment
        self._exhausted = False
        self.rows_read = 0
        self.first_time_ns = None
        self.last_time_ns = None

    def _next_row(self):
        if self._exhausted:
            return None
        try:
            time_a, time_b, values = next(self._rows)
        except StopIteration:
            self._exhausted = True
            return None
        time_a, time_b = int(time_a), int(time_b)
        values = tuple(float(value) for value in values)
        if len(values) != 6:
            raise ValueError('IMU 行必须包含 6 个分量（加速度 xyz + 角速度 xyz）')
        if time_a <= 0 or time_b <= 0:
            raise ValueError('IMU 行时间必须为正整数纳秒')
        if self.last_time_ns is not None and time_a <= self.last_time_ns:
            raise ValueError('IMU 时间戳非严格递增')
        return time_a, time_b, values

    def _account(self, time_a):
        self.rows_read += 1
        if self.first_time_ns is None:
            self.first_time_ns = time_a
        self.last_time_ns = time_a

    def _pull(self):
        row = self._next_row()
        if row is None:
            return False
        time_a, time_b, values = row
        try:
            self._buffer.add(struct.pack('<QQ6f', time_a, time_b, *values))
        except OverflowError:
            raise RuntimeError('IMU 缓冲超限：录制 IMU 数据或相机帧间隔异常') from None
        self._account(time_a)
        return True

    @property
    def pending(self):
        return len(self._buffer.rows)

    def ensure_until_time(self, time_ns):
        """timestamp 模式：读行直到缓冲覆盖到该相机时间。"""
        while self._buffer.last_b < time_ns and self._pull():
            pass
        if self._buffer.last_b < time_ns:
            if self._exhausted:
                raise ImuCoverageEnd(
                    f'IMU 数据在相机时间 {int(time_ns)} 之前结束'
                    f'（最后一行 {self.last_time_ns}）'
                )
            raise RuntimeError(f'IMU 数据不足：录制 IMU 未覆盖到相机帧时间 {int(time_ns)}')

    def ensure_pending(self, needed):
        """lattice 模式：读行直到缓冲里有足够的格点样本。"""
        needed = int(needed)
        while self.pending < needed and self._pull():
            pass
        if self.pending < needed:
            if self._exhausted:
                raise ImuCoverageEnd(
                    f'IMU 数据已结束，剩余 {self.pending} 个样本，少于本帧需要的 {needed} 个'
                )
            raise RuntimeError(f'IMU 样本不足：需要 {needed}，可用 {self.pending}')

    def ensure_initial(self, stamp_ns):
        """首帧前读入并丢弃早于首帧的样本，让 lattice 相位从首帧开始。"""
        stamp_ns = int(stamp_ns)
        while self._buffer.last_a < stamp_ns:
            if not self._pull():
                break
            if self.pending > PENDING_FIRST_FRAME_LIMIT:
                self._buffer.trim_before(stamp_ns)
        self._buffer.trim_before(stamp_ns)

    def drain(self):
        """回放结束后读完剩余行，仅用于统计整段录制的 IMU 行数/速率。"""
        while True:
            row = self._next_row()
            if row is None:
                break
            self._account(row[0])

    def summary(self, alignment, timing):
        measured_rate = None
        if self.rows_read >= 2 and self.last_time_ns and self.last_time_ns > self.first_time_ns:
            measured_rate = (self.rows_read - 1) * 1e9 / (self.last_time_ns - self.first_time_ns)
        data = {
            'rows': self.rows_read,
            'first_time_ns': self.first_time_ns,
            'last_time_ns': self.last_time_ns,
            'measured_rate_hz': None if measured_rate is None else round(measured_rate, 3),
            'configured_rate_hz': timing.get('imu_rate_hz'),
            'alignment': alignment,
        }
        if alignment == 'lattice':
            data['lattice_consumed'] = int(getattr(self._buffer, 'consumed', 0))
            data['lattice_gap_events'] = int(getattr(self._buffer, 'gap_events', 0))
            data['camera_to_imu_drift_ppm'] = timing.get('camera_to_imu_drift_ppm')
        else:
            data['camera_to_imu_lag_ms'] = timing.get('camera_to_imu_lag_ms')
        return data


class _PacketImuFeed(_ImuFeed):
    def _pull(self):
        packet = next(self._rows, None)
        if packet is None:
            self._exhausted = True
            return False
        for row in self._buffer.add(packet.payload, packet.device_ts):
            self._account(row.time_a_raw)
        return True

    def ensure_until_time(self, time_ns):
        while self._buffer.covered_until_ns < time_ns and self._pull():
            pass
        if self._buffer.covered_until_ns < time_ns:
            raise ImuCoverageEnd(f'最后的 IMU 包格点覆盖到 {self._buffer.covered_until_ns} ns，'
                                 f'当前相机帧为 {time_ns} ns')

    def drain(self):
        for packet in self._rows:
            for row in parse_candidate_payload(packet.payload):
                self._account(row.time_a_raw)

    def summary(self, alignment, timing):
        data = super().summary(alignment, timing)
        data['lattice_reference'] = 'packet'
        data['lattice_phase_ms'] = self._buffer.phase_ns / 1e6
        return data


def _verified_phase_policy_for_hybrid_recording(config, *, mode, explicit_timing=False, explicit_config=False):
    """Prefer an independently verified IMU phase for legacy hybrid recordings.

    Never change the original recording snapshot. An explicit reproducibility
    override remains manual, and already-verified session models are retained.
    """
    if (mode != HYBRID or explicit_timing or explicit_config
            or config.timing.get('imu_alignment') != 'lattice'
            or config.timing.get('lattice_reference', 'packet') != 'packet'
            or config.timing.get('session_phase_verified')):
        return config, False
    if config.timing.get('lattice_phase_mode') == 'session_auto':
        return config, False
    return replace(config, timing={**config.timing, 'lattice_phase_mode': 'session_auto'}), True


class ReplayVio:
    """离线回放一个 VS030 录制目录；`run()` 同步执行并返回 `replay.json` 同内容报告。

    参数：
    - `recording_dir`：录制目录（`raw/sdk_records.bin` 等）；
    - `output_dir`：输出目录；默认 `<recording>/replay_vio/replay_<unixns>`；
    - `timing`：IMU 对齐参数覆盖（`camera_to_imu_lag_ms` / `imu_alignment` / `imu_rate_hz` /
      `camera_to_imu_drift_ppm`），缺省读录制内 `vio_timing.yaml` 或仓库配置；
    - `stride`：每 N 帧处理 1 帧（跳过的帧不送桥，lattice 用 skip_span 保持相位）；
    - `max_frames`：最多处理多少帧（None 表示全部）；
    - `start_stamp_ns` / `end_stamp_ns`：相机绝对时间的闭区间；从区间起点以空 IMU
      首帧启动估计器，保持原始时间戳和录制帧索引，stride 仍以整段录制为基准；
    - `on_status`：状态回调，消息形如 `{"type": "status"|"error"|"progress", "message": ...}`，
      与实时链路语义一致（progress 额外带计数字段）。
    """

    def __init__(self, recording_dir, *, output_dir=None, timing=None, stride=1,
                 max_frames=None, on_status=None, config_snapshot=None, mode=DEFAULT_MODE, on_frame=None,
                 worker=None, start_stamp_ns=None, end_stamp_ns=None, library_path=None, coordinated_stereo=False,
                 continue_on_localization_error=False):
        self.mode = validate_mode(mode)
        self.use_imu = self.mode != STEREO
        self.on_frame = on_frame
        # When the caller owns a long-lived VIO worker (one GUI), reuse its ORB process so the
        # vocabulary is loaded once per GUI instead of once per replay.
        self.worker = worker
        self.library_path = library_path
        self.coordinated_stereo=bool(coordinated_stereo)
        self.continue_on_localization_error=bool(continue_on_localization_error)
        self.recording_dir = Path(recording_dir).resolve()
        self.output_dir = None if output_dir is None else Path(output_dir).resolve()
        self.timing_override = dict(timing) if timing is not None else None
        self.config_override = config_snapshot
        self.stride = int(stride)
        if self.stride < 1:
            raise ValueError('stride 必须 >= 1')
        self.max_frames = None if max_frames is None else int(max_frames)
        if self.max_frames is not None and self.max_frames < 1:
            raise ValueError('max_frames 必须 >= 1')
        for name, value in (('start_stamp_ns', start_stamp_ns), ('end_stamp_ns', end_stamp_ns)):
            if value is not None and (type(value) is not int or not 0 < value < 2**63):
                raise ValueError(f'{name} 必须为正整数纳秒')
        if start_stamp_ns is not None and end_stamp_ns is not None and start_stamp_ns > end_stamp_ns:
            raise ValueError('start_stamp_ns 必须 <= end_stamp_ns')
        self.start_stamp_ns = start_stamp_ns
        self.end_stamp_ns = end_stamp_ns
        self.on_status = on_status
        self._cancel = threading.Event()
        self._bridge = None
        self._epochs = []

    def cancel(self):
        """请求取消（可从其他线程调用）；已写入的 poses.jsonl 与最终 replay.json 保留。"""
        self._cancel.set()
        bridge = self._bridge
        if bridge is not None:
            try:
                bridge.abort()
            except Exception:  # noqa: BLE001 - 取消路径不抛出
                pass

    def run(self):
        started = time.perf_counter()
        report = {
            'status': 'failed',
            'mode': self.mode,
            'recording': {'root': str(self.recording_dir)},
            'timing': {},
            'frames': {'decoded': 0, 'processed': 0, 'skipped': 0,
                       'ok': 0, 'imu_initialized': 0, 'valid_poses': 0},
            'map_epochs': [],
            'reset_events': {},
            'imu': {},
            'warnings': [],
            'imu_coverage_exhausted': False,
            'stride': self.stride,
            'max_frames': self.max_frames,
            'image_range': {'start_stamp_ns': self.start_stamp_ns, 'end_stamp_ns': self.end_stamp_ns,
                            'first_processed_stamp_ns': None, 'last_processed_stamp_ns': None,
                            'frames_before_start': 0, 'end_bound_reached': False},
            'elapsed_s': 0.0,
            'error': None,
            'output': None,
        }
        self._cancel.clear()
        self._epochs = []
        out_dir = None
        bridge = None
        feed = None
        frames_iter = None
        poses_file = None
        timing = {}
        try:
            if not self.recording_dir.is_dir():
                raise FileNotFoundError(f'录制目录不存在：{self.recording_dir}')
            if not (self.recording_dir / 'raw' / 'sdk_records.bin').is_file():
                raise FileNotFoundError(
                    f'不是 VS030 SDK 录制：缺少 {self.recording_dir / "raw" / "sdk_records.bin"}'
                )
            self._recording_summary(report)
            out_dir = self._prepare_output()
            report['output'] = {
                'dir': str(out_dir),
                'poses': str(out_dir / 'poses.jsonl'),
                'run_log': str(out_dir / 'run.log'),
                'replay': str(out_dir / 'replay.json'),
            }
            config = resolve_recording_config(self.recording_dir, override=self.config_override,
                                              timing_override=self.timing_override)
            config, auto_phase = _verified_phase_policy_for_hybrid_recording(
                config, mode=self.mode, explicit_timing=self.timing_override is not None,
                explicit_config=self.config_override is not None)
            if auto_phase:
                self._status({'type': 'status', 'message':
                              '混合模式：历史录制相位未经核验，先独立核验双目/IMU时序，'
                              '不会直接使用旧固定相位'})
            from .session_phase import prepare_recording_phase
            if self.use_imu:
                config = prepare_recording_phase(self.recording_dir, config, out_dir,
                    cancelled=self._cancel,
                    on_progress=lambda text: self._status({'type':'status','message':text}))
                report['phase_verification'] = {
                    'auto_prepared': auto_phase,
                    'verified': bool(config.timing.get('session_phase_verified')),
                    'effective_phase_ms': config.timing.get('lattice_phase_ms'),
                }
            # No phase verification is meaningful when the native system consumes no IMU.
            timing_input = config.timing if self.use_imu else {**config.timing, 'lattice_phase_mode':'manual'}
            timing = resolve_timing(self.recording_dir, timing_input)
            timing['image_scale'] = config.image_scale
            report['timing'] = dict(timing)
            report['config_snapshot'] = config.describe()
            from ..calibration.loader import load_calibration
            from .hand_world import calibration_geometry_fingerprint
            report['replay_config'] = {
                'schema_version': 1,
                'fingerprint': report['config_snapshot']['fingerprint'],
                'calibration_file_sha256': report['config_snapshot']['hashes']['calibration'],
                'camera_geometry_sha256': calibration_geometry_fingerprint(
                    load_calibration(config.calibration, kalibr_left_camera='cam0')),
                'camera_frame': 'raw_cam0_channel4',
            }
            sdk = sdk_recording_module()
            self._status({'type': 'status',
                          'message': '正在启动 ORB-SLAM3；首次加载词典可能需要约 20–30 秒，'
                                     '回放将在就绪后开始…'})
            geometry = prepare_vs030_vio_geometry(config.calibration)
            config.validate_geometry(geometry)
            quality_options = dict(config.quality)
            # 显示重锚选项不属于质量门控；回放保留原地图坐标。
            for key in ('pose_continuity', 'continuity_max_speed_mps', 'continuity_min_jump_m'):
                quality_options.pop(key, None)
            quality = make_quality_gate(self.mode, **quality_options)
            alignment = timing['imu_alignment']
            packet_lattice = (alignment == 'lattice' and timing['lattice_reference'] == 'packet'
                              and hasattr(sdk, 'iter_imu_packets'))
            buffer = None
            mode_text = '纯双目视觉'
            if self.use_imu:
                if alignment == 'lattice':
                    options = dict(
                        sample_rate_hz=timing['imu_rate_hz'],
                        camera_to_imu_drift_ppm=timing['camera_to_imu_drift_ppm'],
                        capacity=IMU_LATTICE_CAPACITY,
                    )
                    buffer = (PacketLatticeImuBuffer(phase_ns=packet_lattice_phase_ns(timing), **options)
                              if packet_lattice else LatticeImuBuffer(**options))
                    mode_text = (f"帧格点 {timing['imu_rate_hz']:.1f} Hz "
                                 f"{timing['camera_to_imu_drift_ppm']:+.0f} ppm")
                else:
                    buffer = CandidateImuBuffer(
                        camera_to_imu_lag_ns=round(timing['camera_to_imu_lag_ms'] * 1e6)
                    )
                    mode_text = f"时间戳偏移 {timing['camera_to_imu_lag_ms']:+.1f} ms"
                if packet_lattice:
                    feed = _PacketImuFeed(sdk.iter_imu_packets(self.recording_dir), buffer, alignment)
                else:
                    feed = _ImuFeed(sdk.iter_imu_rows(self.recording_dir), buffer, alignment)
            frames_iter = sdk.iter_paired_frames(self.recording_dir)
            poses_file = (out_dir / 'poses.jsonl').open('w', encoding='utf8', buffering=1)
            library_options = {'library_path': self.library_path} if self.library_path is not None else {}
            if self.coordinated_stereo:library_options['coordinated_stereo']=True
            bridge = ReplayBridge(out_dir / 'run.log', settings_path=config.settings, mode=self.mode,
                                  worker=self.worker, **library_options)
            report['coordinated_stereo']=self.coordinated_stereo
            if self.library_path is not None:
                import hashlib
                library = Path(self.library_path).resolve()
                report['explicit_library'] = {'path':str(library),
                    'sha256':hashlib.sha256(library.read_bytes()).hexdigest(),
                    'deployment_changed':False}
            if self.mode=='hybrid':
                import hashlib
                report['hybrid_runtime']={'instances':1,'sensor':'IMU_STEREO',
                    'visual_bootstrap':True,'imu_initialization_forced':False,
                    'effective_settings':str(bridge.settings_path),
                    'effective_settings_sha256':hashlib.sha256(bridge.settings_path.read_bytes()).hexdigest()}
            self._bridge = bridge
            bridge.start()
            # FakeBridge 等替身没有该属性；缺失时按"新加载"描述，不影响行为。
            reused = bool(getattr(bridge, 'reused_worker', False))
            self._status({'type': 'status',
                          'message': ('复用已加载词典，开始离线回放' if reused
                                      else 'ORB-SLAM3 已就绪，开始离线回放')
                                     + f' · {mode_text}（实验）'})
            self._play(frames_iter, feed, buffer, bridge, quality, geometry,
                       poses_file, timing, report)
            if report['frames']['processed'] == 0:
                raise RuntimeError('录制中没有可回放的双目配对帧')
            if feed is not None:
                feed.drain()
            report['status'] = 'completed_with_localization_failure' if report.get('localization_failure') else 'completed'
        except ReplayCancelled:
            report['status'] = 'cancelled'
            report['error'] = '回放已取消'
        except Exception as exc:  # noqa: BLE001 - 失败必须收敛成报告
            if self._cancel.is_set():
                report['status'] = 'cancelled'
                report['error'] = '回放已取消'
            else:
                report['status'] = 'failed'
                report['error'] = str(exc)
                self._status({'type': 'error', 'message': str(exc)})
        finally:
            if frames_iter is not None and hasattr(frames_iter, 'close'):
                try:
                    frames_iter.close()
                except Exception:  # noqa: BLE001
                    pass
            if poses_file is not None:
                try:
                    poses_file.close()
                except OSError:
                    pass
            self._bridge = None
            if bridge is not None:
                try:
                    bridge.close()
                except Exception:  # noqa: BLE001 - 清理失败不能掩盖原始错误
                    pass
                report['native_process_exit_code']=getattr(bridge,'exit_code',None)
            if feed is not None:
                try:
                    feed.drain()  # 失败时也读完剩余 IMU 行，让报告的行数/速率反映整段录制
                except Exception:  # noqa: BLE001
                    pass
                try:
                    summary = feed.summary(timing.get('imu_alignment', ''), timing)
                    # 包数：录制侧索引/manifest 的 sink7 记录数（IMU 行流本身不带包边界）。
                    counts = (report.get('recording') or {}).get('sdk_count_by_sink') or {}
                    try:
                        summary['packets'] = int(counts['7']) if counts.get('7') is not None else None
                    except (KeyError, TypeError, ValueError):
                        summary['packets'] = None
                    summary.setdefault('rate_hz', summary.get('measured_rate_hz'))
                    report['imu'] = summary
                except Exception as exc:  # noqa: BLE001
                    report['imu'] = {'error': str(exc)}
            report['map_epochs'] = list(self._epochs)
            report['reset_events'] = self._scan_reset_events(out_dir, report)
            report['elapsed_s'] = round(time.perf_counter() - started, 3)
            if out_dir is not None:
                try:
                    self._write_report(out_dir, report)
                except Exception as exc:  # noqa: BLE001
                    report['error'] = f"{report['error']}；写入 replay.json 失败：{exc}" \
                        if report['error'] else f'写入 replay.json 失败：{exc}'
                    report['status'] = 'failed'
            if report['status'] == 'completed':
                stats = report['frames']
                self._status({'type': 'status',
                              'message': f"回放完成：处理 {stats['processed']} 帧，"
                                         f"跟踪 OK {stats['ok']} 帧，"
                                         f"位姿通过门控 {stats['valid_poses']} 帧"})
        return report

    # ------------------------------------------------------------------ 回放主循环

    def _play(self, frames_iter, feed, buffer, bridge, quality, geometry, poses_file,
              timing, report):
        localization = LocalizationPipeline(self.mode, quality,
            recovery_commit=bridge.commit_recovery if getattr(bridge,'recovery_protocol',1)==PROTOCOL else None)
        alignment = timing['imu_alignment']
        packet_lattice = isinstance(buffer, PacketLatticeImuBuffer)
        rect = geometry.rectifier
        stats = report['frames']
        max_frames = self.max_frames
        stride = self.stride
        position = 0
        prev_selected_stamp = None
        prev_stream_stamp = None
        localization_failure=None
        for paired in frames_iter:
            if self._cancel.is_set():
                raise ReplayCancelled()
            position += 1
            stamp = int(paired.device_ts) * 1000
            if prev_stream_stamp is not None and stamp <= prev_stream_stamp:
                raise ValueError(
                    '相机设备时间在回放中非严格递增：'
                    f'前一帧 {prev_stream_stamp}，当前 {stamp}（device_ts={paired.device_ts}）'
                )
            stats['decoded'] += 1
            if self.end_stamp_ns is not None and stamp > self.end_stamp_ns:
                stats['skipped'] += 1  # This decoded boundary frame is not sent to SLAM.
                report['image_range']['end_bound_reached'] = True
                break
            selected = (position - 1) % stride == 0
            try:
                if self.start_stamp_ns is not None and stamp < self.start_stamp_ns:
                    # Advance the original sensor stream without warming up SLAM or
                    # accumulating an entire prefix in a bounded IMU buffer.
                    if self.use_imu:
                        if packet_lattice:
                            feed.ensure_until_time(stamp)
                            while len(buffer.rows) > 1 and buffer.rows[1][0] <= stamp:
                                buffer.rows.popleft()
                        elif alignment == 'lattice':
                            if prev_stream_stamp is None:
                                feed.ensure_initial(stamp)
                            else:
                                feed.ensure_pending(buffer.expected(prev_stream_stamp, stamp))
                                buffer.skip_span(stamp - prev_stream_stamp)
                        else:
                            feed.ensure_until_time(stamp)
                            # Keep two preceding samples for gyro interpolation.
                            while len(buffer.rows) > 2 and buffer.rows[2].time_a_raw <= stamp:
                                buffer.rows.popleft()
                    stats['skipped'] += 1
                    report['image_range']['frames_before_start'] += 1
                    prev_stream_stamp = stamp
                    continue
                if not selected:
                    if self.use_imu and alignment == 'lattice' and not packet_lattice and prev_stream_stamp is not None:
                        span = stamp - prev_stream_stamp
                        feed.ensure_pending(buffer.expected(prev_stream_stamp, stamp))
                        buffer.skip_span(span)
                    stats['skipped'] += 1
                    prev_stream_stamp = stamp
                    continue

                measurements = []
                if not self.use_imu or localization_failure is not None:
                    pass
                elif packet_lattice:
                    feed.ensure_until_time(stamp)
                    if prev_selected_stamp is None and buffer.rows[0][0] > stamp:
                        stats['skipped'] += 1
                        prev_stream_stamp = stamp
                        continue
                    if prev_selected_stamp is not None:
                        measurements = buffer.interval(prev_selected_stamp, stamp)
                elif prev_selected_stamp is None:
                    # native bridge / ORB-SLAM3 要求首帧没有 IMU 样本。
                    if alignment == 'lattice':
                        feed.ensure_initial(stamp)
                        buffer.trim_before(stamp)
                elif alignment == 'lattice':
                    need = buffer.expected(prev_stream_stamp, stamp)
                    feed.ensure_pending(need)
                    measurements = buffer.consume(prev_stream_stamp, stamp)
                    if measurements is None:
                        raise RuntimeError(
                            f'IMU 格点样本不足：帧间需要 {need}，可用 {buffer.pending()}'
                        )
                else:
                    feed.ensure_until_time(stamp)
                    try:
                        measurements = buffer.interval(prev_selected_stamp, stamp)
                    except ValueError as exc:
                        raise RuntimeError(
                            f'IMU 时间戳对齐失败（timestamp，'
                            f'lag {timing["camera_to_imu_lag_ms"]:+.1f} ms）：{exc}；'
                            '可尝试 --lag-ms 0 或 --alignment lattice 对照'
                        ) from exc
            except ImuCoverageEnd as exc:
                # 与实时链路“IMU 样本不足跳过该帧”的语义一致：录制 IMU 比视频先结束，
                # 后面所有帧都无法获得惯性样本，显式记录并结束回放（仍算跑完，报告保留证据）。
                stats['skipped'] += 1
                report['imu_coverage_exhausted'] = True
                report['warnings'].append(f'IMU 覆盖不足，回放在第 {position} 个解码帧处结束：{exc}')
                self._status({'type': 'status',
                              'message': f'IMU 数据覆盖不足，回放提前结束（已处理 '
                                         f"{stats['processed']} 帧）：{exc}"})
                break

            left = self._gray(cv2.remap(paired.right_bgr, *rect.maps_left, cv2.INTER_LINEAR))
            right = self._gray(cv2.remap(paired.left_bgr, *rect.maps_right, cv2.INTER_LINEAR))
            scale = float(timing.get('image_scale', 1.0))
            if scale != 1.0:
                left = cv2.resize(left, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                right = cv2.resize(right, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            if localization_failure is None:
                try:
                    result = bridge.track(stamp, left, right, measurements)
                    matrix = result.get('matrix')
                    if matrix is not None:
                        result['matrix'] = (
                            np.asarray(matrix, dtype=float) @ geometry.T_orb_left_from_gui_left
                        ).tolist()
                    result = localization.process(result, measurements)
                except (ConnectionError,OSError,RuntimeError) as exc:
                    if isinstance(exc,ReplayCancelled) or self._cancel.is_set() or not self.continue_on_localization_error:
                        raise
                    localization_failure={'timestamp_ns':stamp,'frame_index':int(getattr(paired,'index',position-1)),
                        'error':str(exc),'action':'continue_camera_hand_processing_without_localization',
                        'estimator_restarted':False,'old_pose_reused':False}
                    report['localization_failure']=localization_failure
                    report['error']=str(exc)
                    report['warnings'].append('定位进程异常；剩余双目/手部继续处理，相机位姿不可用：'+str(exc))
                    self._status({'type':'error','message':report['warnings'][-1]})
                    try:bridge.close()
                    except Exception as cleanup_error:
                        report['warnings'].append('定位子进程清理异常：'+str(cleanup_error))
            if localization_failure is not None:
                from .local_segments import LocalTrajectorySegments
                result={'type':'pose','mode':self.mode,'timestamp_ns':stamp,
                    'frame_id':int(getattr(paired,'index',position-1)),
                    'tracking_state':'LOCALIZATION_UNAVAILABLE','pose_valid':False,'matrix':None,
                    'map_epoch':None,'world_frame_revision':None,'imu_initialized':False,
                    'quality_status':'localization_worker_failed','localization_failure':dict(localization_failure),
                    'quality_gate':{'passed':False,'reason_code':'localization_worker_failed'}}
                result['local_motion']=LocalTrajectorySegments().process(result)
                result['local_motion']['reason']='localization_worker_failed'
                report['frames']['localization_unavailable']=report['frames'].get('localization_unavailable',0)+1
            result['imu_alignment'] = alignment if self.use_imu else 'disabled'
            if self.use_imu and alignment == 'lattice':
                result['imu_lattice'] = {
                    'pending': buffer.pending(),
                    'consumed': buffer.consumed,
                    'gap_events': buffer.gap_events,
                    'rate_hz': buffer.sample_rate_hz,
                    'drift_ppm': timing['camera_to_imu_drift_ppm'],
                }
                if packet_lattice:
                    result['imu_lattice'].update(reference='packet', phase_ms=buffer.phase_ns / 1e6)
            elif self.use_imu:
                result['camera_to_imu_lag_ms'] = timing['camera_to_imu_lag_ms']
            result['sdk_timestamp_audit'] = self._audit(paired)
            result['replay_config'] = dict(report['replay_config'])
            result['replay'] = {
                'source': 'offline_replay',
                'recording': str(self.recording_dir),
                'order': position,
                'frame_index': int(getattr(paired, 'index', position - 1)),
                'sequence': int(paired.sequence),
                'device_ts_raw': int(paired.device_ts),
                'stride': stride,
                'imu_samples_in_interval': len(measurements),
            }
            poses_file.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + '\n')

            stats['processed'] += 1
            if report['image_range']['first_processed_stamp_ns'] is None:
                report['image_range']['first_processed_stamp_ns'] = stamp
            report['image_range']['last_processed_stamp_ns'] = stamp
            if result.get('tracking_state') == 'OK':
                stats['ok'] += 1
            if result.get('imu_initialized'):
                stats['imu_initialized'] += 1
            if result.get('pose_valid'):
                stats['valid_poses'] += 1
            epoch = result.get('map_epoch')
            if epoch is not None and (not self._epochs or self._epochs[-1] != epoch):
                self._epochs.append(epoch)
            display_result = dict(result)
            display_result.pop('raw_matrix', None)
            if self.on_frame is not None:
                self.on_frame(paired, display_result)
            self._status(display_result)

            if stats['processed'] % PROGRESS_EVERY_FRAMES == 0:
                self._status({
                    'type': 'progress',
                    'message': f"回放进度：已解码 {stats['decoded']} 帧，"
                               f"已处理 {stats['processed']} 帧，跳帧 {stats['skipped']}",
                    'decoded': stats['decoded'],
                    'processed': stats['processed'],
                    'skipped': stats['skipped'],
                    'ok': stats['ok'],
                    'valid_poses': stats['valid_poses'],
                })
            prev_selected_stamp = stamp
            prev_stream_stamp = stamp
            if max_frames is not None and stats['processed'] >= max_frames:
                break

    # ------------------------------------------------------------------ 辅助

    @staticmethod
    def _gray(image):
        if image is None:
            raise ValueError('回放帧图像缺失')
        array = np.asarray(image)
        if array.ndim == 2:
            return np.ascontiguousarray(array)
        if array.ndim == 3 and array.shape[2] == 3:
            return np.ascontiguousarray(cv2.cvtColor(array, cv2.COLOR_BGR2GRAY))
        raise ValueError(f'回放帧图像通道数不支持：{array.shape}')

    def _audit(self, paired):
        def value(name):
            try:
                return int(getattr(paired, name, 0) or 0)
            except (TypeError, ValueError):
                return 0
        capture_left = value('capture_ts_left_ns')
        capture_right = value('capture_ts_right_ns')
        arrival_left = value('arrival_ts_left_ns')
        arrival_right = value('arrival_ts_right_ns')
        return {
            'camera_device_ts_raw': int(paired.device_ts),
            'camera_device_ts_to_ns_candidate': 'raw * 1000 (microseconds assumption; unverified)',
            'sdk_clock_domain': 'sdk_internal_monotonic_ns',
            'camera_capture_ts_ns_channel_4': capture_left,
            'camera_capture_ts_ns_channel_5': capture_right,
            'camera_arrival_ts_ns_channel_4': arrival_left,
            'camera_arrival_ts_ns_channel_5': arrival_right,
            'camera_capture_pair_delta_ns': abs(capture_left - capture_right),
            'camera_arrival_pair_delta_ns': abs(arrival_left - arrival_right),
            'warning': 'SDK monotonic timestamps are diagnostic only; device_ts unit/exposure '
                       'semantics and IMU payload layout remain unverified.',
        }

    def _prepare_output(self):
        if self.output_dir is not None:
            out_dir = self.output_dir
        else:
            out_dir = self.recording_dir / 'replay_vio' / f'replay_{time.time_ns()}'
        out_dir.mkdir(parents=True, exist_ok=True)
        return out_dir

    def _recording_summary(self, report):
        summary = report['recording']
        manifest_path = self.recording_dir / 'manifest.json'
        if not manifest_path.is_file():
            summary['warning'] = '录制目录缺少 manifest.json；只按 raw/ 文件回放'
            return
        try:
            manifest = json.loads(manifest_path.read_text(encoding='utf8'))
        except (OSError, ValueError) as exc:
            summary['warning'] = f'manifest.json 不可读：{exc}'
            return
        for key in ('recording_status', 'frame_index_count', 'sdk_count_by_sink',
                    'sdk_count', 'closed_time_ns'):
            if key in manifest:
                summary[key] = manifest[key]

    @staticmethod
    def _scan_reset_events(out_dir, report):
        events = {
            'system_reset': 0,
            'motion_insufficient': 0,
            'map_epoch_changes': max(0, len(report.get('map_epochs', [])) - 1),
        }
        log_path = (out_dir / 'run.log') if out_dir is not None else None
        if log_path is not None and log_path.is_file():
            try:
                text = log_path.read_text(encoding='utf8', errors='replace')
            except OSError:
                return events
            events['system_reset'] = text.count('System Reseting')
            events['motion_insufficient'] = text.count('Not enough motion')
        return events

    @staticmethod
    def _write_report(out_dir, report):
        path = out_dir / 'replay.json'
        temp = out_dir / 'replay.json.tmp'
        temp.write_text(
            json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2),
            encoding='utf8',
        )
        os.replace(temp, path)

    def _status(self, message):
        callback = self.on_status
        if callback is None:
            return
        try:
            callback(message)
        except Exception:  # noqa: BLE001 - UI 回调异常不能打断回放
            pass
