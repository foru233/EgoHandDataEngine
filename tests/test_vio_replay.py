"""离线 VIO 回放的单元测试与集成测试。

- 单元测试用假 `storage.sdk_recording` 生成器 + 假桥，覆盖 IMU 选取、stride 相位、
  时间戳严格递增、报告字段、桥/子进程失败清理、取消语义与 CLI 退出码；
- 集成测试合成一段 HEVC 双目 + 原始 IMU 的 VS030 录制，启动真实 egohand-vio 桥跑完
  完整回放；环境缺失（Python 环境/DLL/词典/词袋/录制模块）时 pytest.skip，不用相机。
"""
import importlib.util
import json
import struct
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from egohand.vio import replay_runtime
from egohand.vio.replay_runtime import ReplayVio, resolve_timing

ROOT = Path(__file__).resolve().parents[1]
BASE_NS = 1_700_000_000_000_000
FRAME_STEP_NS = 100_000_000  # 单元测试用 10 Hz，便于按整数格点断言
TEXTURE_SIZE = (1200, 1600)


# ---------------------------------------------------------------- 假数据/假桥夹具

class FakeBridge:
    """替身桥：记录每帧收到的 IMU 样本，返回固定 OK 位姿。"""

    instances = []

    def __init__(self, log_path, **kwargs):
        self.log_path = Path(log_path)
        self.calls = []
        self.started = False
        self.closed = False
        self.aborted = False
        FakeBridge.instances.append(self)

    def start(self):
        self.started = True
        self.log_path.write_text('fake bridge ready\n', encoding='utf8')

    def track(self, stamp, left_gray, right_gray, measurements):
        self.calls.append({
            'stamp': int(stamp),
            'measurements': [tuple(float(value) for value in row) for row in measurements],
            'left_shape': tuple(left_gray.shape),
            'right_shape': tuple(right_gray.shape),
        })
        return {
            'type': 'pose', 'timestamp_ns': int(stamp), 'tracking_state': 'OK',
            'pose_valid': True, 'imu_initialized': True, 'map_epoch': 0,
            'matrix': np.eye(4).tolist(),
        }

    def abort(self):
        self.aborted = True

    def close(self):
        self.closed = True


class FailingBridge(FakeBridge):
    def track(self, stamp, left_gray, right_gray, measurements):
        raise RuntimeError('模拟桥故障')


class SlowBridge(FakeBridge):
    def track(self, stamp, left_gray, right_gray, measurements):
        result = super().track(stamp, left_gray, right_gray, measurements)
        self.first_call.set()
        time.sleep(0.05)
        return result

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.first_call = threading.Event()


class _IdentityRectifier:
    maps_left = (np.zeros((6, 8), np.float32), np.zeros((6, 8), np.float32))
    maps_right = (np.zeros((6, 8), np.float32), np.zeros((6, 8), np.float32))


class _IdentityGeometry:
    rectifier = _IdentityRectifier()
    T_orb_left_from_gui_left = np.eye(4)


def make_frames(count, *, start_ns=BASE_NS, step_ns=FRAME_STEP_NS):
    frames = []
    for index in range(count):
        device_ts = (start_ns + index * step_ns) // 1000
        frames.append(SimpleNamespace(
            index=index,
            sequence=1000 + index,
            device_ts=device_ts,
            left_bgr=np.full((6, 8, 3), 10 + index, np.uint8),
            right_bgr=np.full((6, 8, 3), 20 + index, np.uint8),
            capture_ts_left_ns=start_ns + index * step_ns + 11,
            capture_ts_right_ns=start_ns + index * step_ns + 12,
            arrival_ts_left_ns=start_ns + index * step_ns + 21,
            arrival_ts_right_ns=start_ns + index * step_ns + 22,
        ))
    return frames


def make_imu_rows(start_ns, end_ns, rate_hz=480.0):
    rows = []
    count = int((end_ns - start_ns) * rate_hz / 1e9)
    for index in range(count + 1):
        stamp = int(start_ns + index * 1e9 / rate_hz)
        rows.append((stamp, stamp, (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)))
    return rows


def make_fake_sdk(frames, imu_rows):
    class FakeSdk:
        @staticmethod
        def iter_paired_frames(root, limit=None, stats=None):
            items = frames if limit is None else frames[:limit]
            for frame in items:
                yield frame

        @staticmethod
        def iter_imu_rows(root):
            for row in imu_rows:
                yield row

    return FakeSdk


def make_recording_dir(tmp_path, name='rec'):
    recording = tmp_path / name
    (recording / 'raw').mkdir(parents=True)
    (recording / 'raw' / 'sdk_records.bin').write_bytes(b'')
    return recording


@pytest.fixture(autouse=True)
def _isolate_fake_instances():
    FakeBridge.instances = []
    yield
    FakeBridge.instances = []


@pytest.fixture
def fake_environment(monkeypatch):
    """假 SDK 模块 + 恒等几何 + 假桥，测试不依赖真实录制解码与 ORB-SLAM3。"""
    def configure(frames, imu_rows, bridge_class=FakeBridge):
        monkeypatch.setattr(replay_runtime, 'sdk_recording_module',
                            lambda: make_fake_sdk(frames, imu_rows))
        monkeypatch.setattr(replay_runtime, 'prepare_vs030_vio_geometry',
                            lambda _path: _IdentityGeometry())
        monkeypatch.setattr(replay_runtime, 'ReplayBridge', bridge_class)
    return configure


def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text(encoding='utf8').splitlines() if line]


# ---------------------------------------------------------------- 纯函数

def test_resolve_timing_prefers_recording_snapshot_and_overrides_fields(tmp_path):
    recording = make_recording_dir(tmp_path)
    (recording / 'vio_timing.yaml').write_text(
        'imu_alignment: lattice\n'
        'camera_to_imu_lag_ms: -85.0\n'
        'imu_rate_hz: 480.0\n'
        'camera_to_imu_drift_ppm: 657.0\n',
        encoding='utf8',
    )
    timing = resolve_timing(recording, {'camera_to_imu_lag_ms': 0.0, 'imu_alignment': 'timestamp'})
    assert timing['imu_alignment'] == 'timestamp'
    assert timing['camera_to_imu_lag_ms'] == 0.0
    assert timing['imu_rate_hz'] == 480.0
    assert timing['camera_to_imu_drift_ppm'] == 657.0
    assert timing['source'].endswith('vio_timing.yaml+参数覆盖')

    with pytest.raises(ValueError):
        resolve_timing(recording, {'imu_alignment': 'nonsense'})
    with pytest.raises(ValueError):
        resolve_timing(recording, {'camera_to_imu_lag_ms': 5000.0})


def test_replay_rejects_missing_recording(tmp_path, monkeypatch):
    monkeypatch.setattr(replay_runtime, 'sdk_recording_module', lambda: SimpleNamespace(
        iter_paired_frames=lambda *a, **k: iter(()), iter_imu_rows=lambda *a, **k: iter(()),
    ))
    vio = ReplayVio(tmp_path / 'missing', output_dir=tmp_path / 'out')
    report = vio.run()
    assert report['status'] == 'failed'
    assert '录制目录不存在' in report['error']


# ---------------------------------------------------------------- timestamp 对齐

def test_replay_timestamp_alignment_reports_and_imu_intervals(tmp_path, fake_environment):
    count = 20
    frames = make_frames(count)
    imu_rows = make_imu_rows(BASE_NS - 200_000_000, BASE_NS + count * FRAME_STEP_NS + 200_000_000)
    fake_environment(frames, imu_rows)
    recording = make_recording_dir(tmp_path)
    statuses = []
    vio = ReplayVio(
        recording,
        timing={'imu_alignment': 'timestamp', 'camera_to_imu_lag_ms': 0.0},
        on_status=statuses.append,
    )
    report = vio.run()

    assert report['status'] == 'completed'
    assert report['output']['dir'].startswith(str(recording / 'replay_vio' / 'replay_'))
    assert report['frames']['decoded'] == count
    assert report['frames']['processed'] == count
    assert report['frames']['skipped'] == 0
    assert report['frames']['ok'] == count
    assert report['frames']['imu_initialized'] == count
    assert report['timing']['imu_alignment'] == 'timestamp'
    assert report['reset_events'] == {'system_reset': 0, 'motion_insufficient': 0,
                                      'map_epoch_changes': 0}
    assert report['imu']['rows'] == len(imu_rows)
    assert report['imu']['measured_rate_hz'] == pytest.approx(480.0, abs=1.0)

    bridge = FakeBridge.instances[-1]
    assert bridge.started and bridge.closed
    assert len(bridge.calls) == count
    assert bridge.calls[0]['measurements'] == []  # 首帧必须 0 条 IMU
    for previous, current in zip(bridge.calls, bridge.calls[1:]):
        assert current['stamp'] > previous['stamp']
        stamps = [row[0] for row in current['measurements']]
        assert stamps == sorted(stamps)
        assert all(previous['stamp'] < stamp <= current['stamp'] for stamp in stamps)
        assert 40 <= len(stamps) <= 50  # 100 ms × 480 Hz ≈ 48，允许边界
    assert bridge.calls[1]['left_shape'] == (6, 8)

    poses_path = Path(report['output']['poses'])
    rows = read_jsonl(poses_path)
    assert len(rows) == count
    assert [row['replay']['sequence'] for row in rows] == [frame.sequence for frame in frames]
    assert [row['replay']['device_ts_raw'] for row in rows] == [frame.device_ts for frame in frames]
    for row in rows:
        assert row['replay']['source'] == 'offline_replay'
        assert 'sdk_timestamp_audit' in row
    assert report['frames']['valid_poses'] == sum(1 for row in rows if row['pose_valid'])
    assert report['frames']['valid_poses'] >= 1  # 20 帧 × 100 ms 超过 1.5 s 稳定门控
    assert Path(report['output']['replay']).is_file()
    assert json.loads(Path(report['output']['replay']).read_text(encoding='utf8')) == report
    assert Path(report['output']['run_log']).is_file()
    assert any(message.get('type') == 'status' and '已就绪' in message.get('message', '')
               for message in statuses)
    assert any(message.get('type') == 'pose' for message in statuses)
    assert any(message.get('type') == 'status' and '回放完成' in message.get('message', '')
               for message in statuses)


# ---------------------------------------------------------------- lattice + stride 相位

def test_replay_lattice_stride_skips_keep_phase(tmp_path, fake_environment):
    count = 13
    stride = 3
    frames = make_frames(count)
    imu_rows = make_imu_rows(BASE_NS - 200_000_000, BASE_NS + count * FRAME_STEP_NS + 200_000_000)
    fake_environment(frames, imu_rows)
    recording = make_recording_dir(tmp_path)
    report = ReplayVio(
        recording,
        output_dir=tmp_path / 'out',
        timing={'imu_alignment': 'lattice', 'imu_rate_hz': 480.0,
                'camera_to_imu_drift_ppm': 0.0},
        stride=stride,
    ).run()

    assert report['status'] == 'completed'
    assert report['frames']['decoded'] == count
    assert report['frames']['skipped'] == count - 5  # 第 1/4/7/10/13 帧被处理
    assert report['frames']['processed'] == 5
    assert report['timing']['imu_alignment'] == 'lattice'
    assert report['imu']['lattice_consumed'] == 4 * 48  # 每帧间隔(100 ms)×480 Hz
    assert report['imu']['lattice_gap_events'] == 0

    bridge = FakeBridge.instances[-1]
    assert [len(call['measurements']) for call in bridge.calls] == [0, 48, 48, 48, 48]
    # 跳帧区间用 skip_span 丢弃，最后一个 1/stride 间隔仍按格点均匀铺开。
    processed_stamps = [call['stamp'] for call in bridge.calls]
    assert processed_stamps == [int(frame.device_ts) * 1000 for frame in frames[0::stride]]
    for previous_stamp, call in zip(processed_stamps, bridge.calls[1:]):
        stamps = [row[0] for row in call['measurements']]
        assert stamps == sorted(stamps)
        assert all(previous_stamp < stamp <= call['stamp'] for stamp in stamps)


def test_replay_timestamp_stride_spans_skipped_intervals(tmp_path, fake_environment):
    count = 6
    stride = 2
    frames = make_frames(count)
    imu_rows = make_imu_rows(BASE_NS - 200_000_000, BASE_NS + count * FRAME_STEP_NS + 200_000_000)
    fake_environment(frames, imu_rows)
    recording = make_recording_dir(tmp_path)
    report = ReplayVio(
        recording,
        output_dir=tmp_path / 'out',
        timing={'imu_alignment': 'timestamp', 'camera_to_imu_lag_ms': 0.0},
        stride=stride,
        max_frames=3,
    ).run()

    assert report['status'] == 'completed'
    assert report['frames']['decoded'] == 5  # 处理到第 3 个选中帧（第 5 帧）即达到 max_frames
    assert report['frames']['processed'] == 3
    assert report['frames']['skipped'] == 2
    bridge = FakeBridge.instances[-1]
    assert [call['stamp'] for call in bridge.calls] == [
        int(frame.device_ts) * 1000 for frame in frames[0::stride]
    ]
    # timestamp 模式跳过帧不消费 IMU：第 2 帧跨越 200 ms 区间，约 96 个样本。
    assert 90 <= len(bridge.calls[1]['measurements']) <= 100


# ---------------------------------------------------------------- 失败/取消清理

def test_replay_rejects_non_increasing_device_timestamps(tmp_path, fake_environment):
    frames = make_frames(6)
    frames[4].device_ts = frames[3].device_ts
    imu_rows = make_imu_rows(BASE_NS - 200_000_000, BASE_NS + 8 * FRAME_STEP_NS)
    fake_environment(frames, imu_rows)
    recording = make_recording_dir(tmp_path)
    report = ReplayVio(
        recording, output_dir=tmp_path / 'out',
        timing={'imu_alignment': 'timestamp', 'camera_to_imu_lag_ms': 0.0},
    ).run()

    assert report['status'] == 'failed'
    assert '非严格递增' in report['error']
    bridge = FakeBridge.instances[-1]
    assert bridge.closed
    assert len(bridge.calls) == 4  # 异常帧之前的 4 帧已提交
    assert len(read_jsonl(report['output']['poses'])) == 4
    replays = read_jsonl(report['output']['poses'])
    stamps = [row['timestamp_ns'] for row in replays]
    assert stamps == sorted(stamps) and len(set(stamps)) == len(stamps)


def test_replay_bridge_failure_cleans_up_and_writes_report(tmp_path, fake_environment):
    frames = make_frames(4)
    imu_rows = make_imu_rows(BASE_NS - 100_000_000, BASE_NS + 4 * FRAME_STEP_NS)
    fake_environment(frames, imu_rows, bridge_class=FailingBridge)
    recording = make_recording_dir(tmp_path)
    statuses = []
    report = ReplayVio(
        recording, output_dir=tmp_path / 'out',
        timing={'imu_alignment': 'timestamp', 'camera_to_imu_lag_ms': 0.0},
        on_status=statuses.append,
    ).run()

    assert report['status'] == 'failed'
    assert '模拟桥故障' in report['error']
    bridge = FailingBridge.instances[-1]
    assert bridge.started and bridge.closed
    assert Path(report['output']['replay']).is_file()
    assert Path(report['output']['poses']).is_file()
    saved = json.loads(Path(report['output']['replay']).read_text(encoding='utf8'))
    assert saved['status'] == 'failed'
    assert any(message.get('type') == 'error' and '模拟桥故障' in message.get('message', '')
               for message in statuses)


def test_replay_real_bridge_start_failure_is_cleaned_up(tmp_path, fake_environment, monkeypatch):
    frames = make_frames(3)
    imu_rows = make_imu_rows(BASE_NS - 100_000_000, BASE_NS + 3 * FRAME_STEP_NS)
    fake_environment(frames, imu_rows, bridge_class=replay_runtime.ReplayBridge)
    monkeypatch.setattr(replay_runtime, 'PYTHON', tmp_path / 'missing_python.exe')
    recording = make_recording_dir(tmp_path)
    vio = ReplayVio(
        recording, output_dir=tmp_path / 'out',
        timing={'imu_alignment': 'timestamp', 'camera_to_imu_lag_ms': 0.0},
    )
    report = vio.run()

    assert report['status'] == 'failed'
    assert report['error']  # 启动失败（本地化 WinError 文案随系统语言变化，不硬编码）
    assert 'WinError' in report['error'] or 'missing_python' in report['error']
    assert vio._bridge is None  # 启动失败后不残留桥引用
    assert Path(report['output']['replay']).is_file()
    assert Path(report['output']['run_log']).is_file()


def test_replay_cancel_stops_run_and_reports_cancelled(tmp_path, fake_environment):
    frames = make_frames(60)
    imu_rows = make_imu_rows(BASE_NS - 100_000_000, BASE_NS + 60 * FRAME_STEP_NS + 100_000_000)
    fake_environment(frames, imu_rows, bridge_class=SlowBridge)
    recording = make_recording_dir(tmp_path)
    vio = ReplayVio(
        recording, output_dir=tmp_path / 'out',
        timing={'imu_alignment': 'timestamp', 'camera_to_imu_lag_ms': 0.0},
    )
    results = []
    thread = threading.Thread(target=lambda: results.append(vio.run()), daemon=True)
    thread.start()
    bridge = None
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if SlowBridge.instances:
            bridge = SlowBridge.instances[-1]
            if bridge.first_call.wait(0.05):
                break
    assert bridge is not None
    vio.cancel()
    thread.join(timeout=10)
    assert not thread.is_alive()
    assert results and results[0]['status'] == 'cancelled'
    assert bridge.aborted
    assert Path(results[0]['output']['replay']).is_file()


def test_replay_ends_cleanly_when_imu_coverage_ends_before_video(tmp_path, fake_environment):
    count = 10
    frames = make_frames(count)
    # IMU 只覆盖前 ~5 帧（模拟实测录制 IMU sidecar 比视频先结束的数据特性）。
    imu_rows = make_imu_rows(BASE_NS - 100_000_000, BASE_NS + 5 * FRAME_STEP_NS)
    fake_environment(frames, imu_rows)
    recording = make_recording_dir(tmp_path)
    report = ReplayVio(
        recording, output_dir=tmp_path / 'out',
        timing={'imu_alignment': 'lattice', 'imu_rate_hz': 480.0,
                'camera_to_imu_drift_ppm': 0.0},
    ).run()

    assert report['status'] == 'completed'  # 不是崩溃：显式记为 IMU 覆盖边界
    assert report['imu_coverage_exhausted'] is True
    assert report['warnings']
    assert 0 < report['frames']['processed'] < count
    assert report['frames']['decoded'] == report['frames']['processed'] + report['frames']['skipped']
    bridge = FakeBridge.instances[-1]
    assert len(bridge.calls) == report['frames']['processed']
    assert bridge.closed
    assert Path(report['output']['replay']).is_file()
    assert 'imu_coverage_exhausted' in json.loads(
        Path(report['output']['replay']).read_text(encoding='utf8')
    )


# ---------------------------------------------------------------- CLI

def _load_cli_module():
    path = ROOT / 'scripts' / 'replay_recording_vio.py'
    spec = importlib.util.spec_from_file_location('replay_recording_vio_cli', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _cli_report(status='completed', error=None, output_dir=None):
    output_dir = Path(output_dir or ROOT / 'results')
    return {
        'status': status,
        'output': {'dir': str(output_dir), 'poses': str(output_dir / 'poses.jsonl'),
                   'run_log': str(output_dir / 'run.log'),
                   'replay': str(output_dir / 'replay.json')},
        'frames': {'decoded': 4, 'processed': 4, 'skipped': 0, 'ok': 4,
                   'imu_initialized': 4, 'valid_poses': 0},
        'map_epochs': [0], 'reset_events': {'system_reset': 0, 'motion_insufficient': 0,
                                            'map_epoch_changes': 0},
        'imu': {'rows': 100}, 'elapsed_s': 1.5, 'error': error,
        'timing': {'imu_alignment': 'lattice'},
    }


def test_cli_passes_options_prints_replay_path_and_exit_codes(tmp_path, monkeypatch, capsys):
    import egohand.vio.replay_runtime as runtime
    recording = make_recording_dir(tmp_path)
    calls = {}

    class DummyVio:
        report = _cli_report(output_dir=tmp_path / 'out')

        def __init__(self, recording, **kwargs):
            calls['recording'] = recording
            calls['kwargs'] = kwargs

        def run(self):
            return DummyVio.report

    monkeypatch.setattr(runtime, 'ReplayVio', DummyVio)
    cli = _load_cli_module()
    code = cli.main([
        '--recording', str(recording), '--output', str(tmp_path / 'out'),
        '--lag-ms', '-85', '--alignment', 'lattice', '--stride', '2',
        '--max-frames', '10', '--quiet',
    ])
    assert code == 0
    summary = json.loads(capsys.readouterr().out)
    assert summary['status'] == 'completed'
    assert summary['replay_json'] == DummyVio.report['output']['replay']
    assert calls['kwargs']['timing'] == {'camera_to_imu_lag_ms': -85.0,
                                         'imu_alignment': 'lattice'}
    assert calls['kwargs']['stride'] == 2
    assert calls['kwargs']['max_frames'] == 10
    assert calls['kwargs']['timing'] is not None

    DummyVio.report = _cli_report(status='failed', error='模拟失败', output_dir=tmp_path / 'out')
    assert cli.main(['--recording', str(recording), '--quiet']) == 1

    with pytest.raises(SystemExit):
        cli.main(['--recording', str(recording), '--stride', '0'])
    with pytest.raises(SystemExit):
        cli.main(['--recording', str(tmp_path / 'missing'), '--quiet'])


# ---------------------------------------------------------------- 合成录制 + 真桥集成

def _texture_frame(index, width=1600, height=1200):
    """确定性纹理图像：足够 ORB 特征，且逐帧有轻微平移；灰度值写入 Y 平面。"""
    import cv2

    rng = np.random.default_rng(20261110)
    base = cv2.GaussianBlur(rng.integers(0, 256, (height, width), dtype=np.uint8), (7, 7), 0)
    canvas = cv2.addWeighted(base, 0.55, np.full((height, width), 70, np.uint8), 0.45, 0)
    for y in range(120, height - 120, 160):
        for x in range(120, width - 120, 160):
            cv2.circle(canvas, (x + index * 2, y), 26, 255, -1)
            cv2.rectangle(canvas, (x - 48, y - 48), (x + 48, y + 48), 0, 4)
    return canvas


def _encode_hevc_packets(images):
    """libx265 无 B 帧编码；保证每个输入帧解码时恰好产出 1 帧（配对逻辑可即时成对）。"""
    import av

    encoder = av.CodecContext.create('libx265', 'w')
    encoder.width = images[0].shape[1]
    encoder.height = images[0].shape[0]
    encoder.pix_fmt = 'yuv420p'
    encoder.options = {'preset': 'ultrafast',
                       'x265-params': 'bframes=0:keyint=10:log-level=error'}
    encoder.open()
    packets = []
    chroma_shape = (encoder.height // 2, encoder.width // 2)
    for image in images:
        frame = av.VideoFrame(encoder.width, encoder.height, 'yuv420p')
        frame.planes[0].update(image)
        frame.planes[1].update(np.full(chroma_shape, 128, np.uint8))
        frame.planes[2].update(np.full(chroma_shape, 128, np.uint8))
        packets.extend(bytes(packet) for packet in encoder.encode(frame))
    packets.extend(bytes(packet) for packet in encoder.encode(None))
    return packets


def _write_synthetic_recording(root, frame_count=20):
    """写一份与 Recorder 输出逐字节兼容的 VS030 录制（sdk_records/imu_records/index/manifest）。"""
    from egohand.storage.recorder import (
        SDK_RECORD_HEADER, SDK_RECORD_MAGIC, SDK_RECORD_VERSION,
    )

    root = Path(root)
    (root / 'raw').mkdir(parents=True, exist_ok=True)
    size = TEXTURE_SIZE
    left_packets = _encode_hevc_packets([_texture_frame(i, size[1], size[0])
                                         for i in range(frame_count)])
    right_packets = _encode_hevc_packets([_texture_frame(i + 1, size[1], size[0])
                                          for i in range(frame_count)])
    assert len(left_packets) == frame_count and len(right_packets) == frame_count

    frame_step_ns = 33_333_000  # 30 Hz
    base_ns = BASE_NS
    imu_rows = make_imu_rows(base_ns - 200_000_000,
                             base_ns + frame_count * frame_step_ns + 200_000_000)
    imu_rows = imu_rows[: len(imu_rows) - (len(imu_rows) % 16)]  # 每条 IMU 包 16 行 × 40 B
    imu_payloads = [
        b''.join(struct.pack('<QQ6f', time_a, time_b, *values)
                 for time_a, time_b, values in imu_rows[offset:offset + 16])
        for offset in range(0, len(imu_rows), 16)
    ]

    sdk_records = bytearray()
    index_lines = []
    total = 0

    def append_record(sink, sequence, device_ts, capture_ns, arrival_ns, payload):
        nonlocal total
        offset = len(sdk_records)
        header = SDK_RECORD_HEADER.pack(
            SDK_RECORD_MAGIC, SDK_RECORD_VERSION, SDK_RECORD_HEADER.size,
            sink, len(payload), sequence, device_ts, capture_ns, arrival_ns,
            capture_ns + 1_000_000, capture_ns + 2_000_000,
        )
        sdk_records.extend(header)
        sdk_records.extend(payload)
        index_lines.append(json.dumps({
            'offset': offset,
            'record_bytes': len(header) + len(payload),
            'sink': sink,
            'sequence': sequence,
            'device_timestamp_raw': device_ts,
            'sdk_capture_timestamp_ns': capture_ns,
            'sdk_arrival_timestamp_ns': arrival_ns,
            'host_receive_monotonic_ns': capture_ns + 1_000_000,
            'host_receive_unix_ns': capture_ns + 2_000_000,
            'payload_bytes': len(payload),
            'imu_copy_offset': None,
        }) + '\n')
        total += 1

    imu_sidecar = bytearray()
    for packet_index, payload in enumerate(imu_payloads):
        imu_device_ts = imu_rows[packet_index * 16][0] // 1000
        imu_offset = len(imu_sidecar)
        header = SDK_RECORD_HEADER.pack(
            SDK_RECORD_MAGIC, SDK_RECORD_VERSION, SDK_RECORD_HEADER.size,
            7, len(payload), 10_000 + packet_index, imu_device_ts,
            imu_rows[packet_index * 16][0], imu_rows[packet_index * 16][0],
            imu_rows[packet_index * 16][0] + 1_000_000,
            imu_rows[packet_index * 16][0] + 2_000_000,
        )
        imu_sidecar.extend(header)
        imu_sidecar.extend(payload)
        append_record(7, 10_000 + packet_index, imu_device_ts,
                      imu_rows[packet_index * 16][0], imu_rows[packet_index * 16][0], payload)
        index_lines[-1] = json.dumps({
            **json.loads(index_lines[-1]), 'imu_copy_offset': imu_offset,
        }) + '\n'

    for index in range(frame_count):
        device_ts = (base_ns + index * frame_step_ns) // 1000
        capture_ns = base_ns + index * frame_step_ns
        append_record(4, index, device_ts, capture_ns, capture_ns + 1_000,
                      left_packets[index])
        append_record(5, index, device_ts, capture_ns + 1, capture_ns + 1_001,
                      right_packets[index])

    (root / 'raw' / 'sdk_records.bin').write_bytes(bytes(sdk_records))
    (root / 'raw' / 'imu_records.bin').write_bytes(bytes(imu_sidecar))
    (root / 'raw' / 'sdk_index.jsonl').write_text(''.join(index_lines), encoding='utf8')
    (root / 'manifest.json').write_text(json.dumps({
        'recording_status': 'transport_unverified',
        'sdk_records': 'raw/sdk_records.bin',
        'imu_records': 'raw/imu_records.bin',
        'sdk_index': 'raw/sdk_index.jsonl',
        'sdk_count': total,
        'sdk_count_by_sink': {'4': frame_count, '5': frame_count,
                              '7': len(imu_payloads)},
        'frame_index_count': frame_count,
        'sdk_record_header_bytes': SDK_RECORD_HEADER.size,
    }, ensure_ascii=False), encoding='utf8')
    (root / 'vio_timing.yaml').write_text(
        'imu_alignment: lattice\n'
        'camera_to_imu_lag_ms: -85.0\n'
        'imu_rate_hz: 480.0\n'
        'camera_to_imu_drift_ppm: 0.0\n',
        encoding='utf8',
    )
    return frame_count, len(imu_rows), len(imu_payloads)


def _integration_environment_ready():
    from egohand.vio.live_runtime import CALIBRATION, PYTHON

    required = (
        PYTHON,
        CALIBRATION,
        ROOT / 'third_party' / 'ORB_SLAM3' / 'Vocabulary' / 'ORBvoc.txt',
        ROOT / 'configs' / 'vio' / 'vs030_rectified_preliminary.yaml',
        ROOT / 'src' / 'egohand' / 'vio' / 'native' / 'bin' / 'egohand_vio_bridge.dll',
    )
    if not all(Path(item).is_file() for item in required):
        return False
    try:
        import av  # noqa: F401
        import egohand.storage.sdk_recording  # noqa: F401
    except Exception:  # noqa: BLE001
        return False
    return True


INTEGRATION_READY = _integration_environment_ready()
INTEGRATION_SKIP = pytest.mark.skipif(
    not INTEGRATION_READY,
    reason='egohand-vio 环境/DLL/词袋/录制解析模块不可用，跳过真实桥集成测试',
)


@INTEGRATION_SKIP
def test_synthetic_hevc_recording_replays_through_real_bridge(tmp_path):
    """合成 HEVC 双目 + 原始 IMU 录制 → 完整离线回放（真实 live_subprocess 桥）。"""
    recording = tmp_path / 'session_synthetic'
    frame_count, imu_row_count, imu_packet_count = _write_synthetic_recording(recording, 20)

    statuses = []
    report = ReplayVio(recording, on_status=statuses.append).run()

    assert report['status'] == 'completed', report.get('error')
    assert report['timing']['imu_alignment'] == 'lattice'
    assert report['frames']['decoded'] == frame_count
    assert report['frames']['processed'] == frame_count
    assert report['frames']['skipped'] == 0
    assert report['imu']['rows'] == imu_row_count
    assert report['imu']['lattice_consumed'] > 0
    assert report['reset_events']['map_epoch_changes'] >= 0

    output = report['output']
    assert Path(output['run_log']).is_file() and Path(output['run_log']).stat().st_size > 0
    assert Path(output['replay']).is_file()
    assert json.loads(Path(output['replay']).read_text(encoding='utf8')) == report
    poses = read_jsonl(output['poses'])
    assert len(poses) == frame_count
    timestamps = [row['timestamp_ns'] for row in poses]
    assert timestamps == sorted(timestamps) and len(set(timestamps)) == len(timestamps)
    for row in poses:
        assert row['replay']['source'] == 'offline_replay'
        assert row['sdk_timestamp_audit']['camera_device_ts_raw'] * 1000 == row['timestamp_ns']
        assert row['imu_alignment'] == 'lattice'
    assert any(message.get('type') == 'status' and '回放完成' in message.get('message', '')
               for message in statuses)
    assert report['elapsed_s'] > 0
