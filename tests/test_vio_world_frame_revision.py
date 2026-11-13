"""世界系修订号的桥接/日志管道测试（不加载原生 DLL）。

覆盖三件事：
1. 质量门控不丢弃 `world_frame_revision`（poses.jsonl 的字段来源）；
2. `live_subprocess.serve` 把修订号与拟合详情放进每帧响应；
3. `scripts/check_world_frame_revision.py` 能在日志里定位递增点，并对旧库
   （没有该字段）明确报告"不支持"而不是"没有重锚"。
"""
import importlib.util
import json
from pathlib import Path
import socket
import struct

import numpy as np

from egohand.vio.pose_quality import PoseQualityGate

ROOT = Path(__file__).resolve().parents[1]
HEADER = struct.Struct('<qIII')


def _load_check_script():
    path = ROOT / 'scripts' / 'check_world_frame_revision.py'
    spec = importlib.util.spec_from_file_location('_world_frame_revision_check', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_pose_quality_gate_keeps_world_frame_revision():
    gate = PoseQualityGate(stable_seconds=0.0)
    matrix = np.eye(4)
    row = {'timestamp_ns': 1, 'tracking_state': 'OK', 'imu_initialized': True,
           'pose_valid': True, 'matrix': matrix.tolist(), 'map_epoch': 9,
           'world_frame_revision': 1, 'world_frame': {'verdict': 'reanchor'}}
    checked = gate.check(row)
    assert checked['world_frame_revision'] == 1
    assert checked['world_frame']['verdict'] == 'reanchor'


def _serve_one_frame(bridge):
    from egohand.vio.live_subprocess import serve

    parent, child = socket.socketpair()
    try:
        payload = HEADER.pack(10, 4, 3, 0) + bytes(4 * 3) + bytes(4 * 3)
        parent.sendall(payload)
        parent.shutdown(socket.SHUT_WR)
        serve(child, bridge, recovery_enabled=False)
        length = struct.unpack('<I', parent.recv(4))[0]
        ready = json.loads(parent.recv(length))
        assert ready['type'] == 'ready'
        length = struct.unpack('<I', parent.recv(4))[0]
        return json.loads(parent.recv(length))
    finally:
        parent.close()
        child.close()


class _Result:
    tracking_state_name = 'OK'
    tracking_state = 2
    pose_valid = True
    imu_initialized = True
    inertial_scale_finalized = True
    map_epoch = 9
    T_world_from_rectified_left_camera = np.eye(4)
    frame_snapshot = None
    world_frame_revision = 1
    world_frame_info = {'revision': 1, 'verdict': 'reanchor',
                        'last_translation_m': 22.82, 'last_residual_p95_m': 0.04,
                        'last_keyframes': 75}


class _Bridge:
    map_mode = None
    _recovery_callback_error = None

    def __init__(self, result):
        self.result = result

    def track_stereo(self, left, right, stamp, imu):
        return self.result

    def get_candidate_body_state(self):
        # 旧库路径：没有原子快照时 serve() 会回退到这个 getter。
        return None


def test_live_subprocess_reports_world_frame_revision_per_frame():
    response = _serve_one_frame(_Bridge(_Result()))
    assert response['world_frame_revision'] == 1
    # 详情不带 revision（避免同一字段两种来源），其余拟合量照常透传。
    assert response['world_frame']['verdict'] == 'reanchor'
    assert response['world_frame']['last_keyframes'] == 75
    assert 'revision' not in response['world_frame']


def test_live_subprocess_defaults_to_zero_for_legacy_bridge():
    class LegacyResult:
        """没有 world_frame_* 属性，等价于不认识该扩展的旧桥接库。"""

        tracking_state_name = 'OK'
        tracking_state = 2
        pose_valid = True
        imu_initialized = True
        inertial_scale_finalized = True
        map_epoch = 9
        T_world_from_rectified_left_camera = np.eye(4)
        frame_snapshot = None

    response = _serve_one_frame(_Bridge(LegacyResult()))
    assert response['world_frame_revision'] == 0
    assert 'world_frame' not in response


def _write_poses(path, rows):
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf8')


def _pose_row(stamp, *, revision=None, epoch=9, step=None, jump=False):
    row = {'type': 'pose', 'timestamp_ns': stamp, 'tracking_state': 'OK',
           'pose_valid': True, 'map_epoch': epoch, 'map_change_index': 440,
           'replay': {'frame_index': 0}}
    if revision is not None:
        row['world_frame_revision'] = revision
        row['world_frame'] = {'verdict': 'reanchor', 'last_keyframes': 75,
                              'last_residual_p95_m': 0.04}
    if step is not None:
        row['translation_step_m'] = step
        row['jump_detected'] = jump
    return row


def test_check_script_locates_increment_despite_unchanged_epoch(tmp_path, capsys):
    module = _load_check_script()
    poses = tmp_path / 'poses.jsonl'
    _write_poses(poses, [_pose_row(1, revision=0),
                         _pose_row(2, revision=0, step=0.047),
                         _pose_row(3, revision=1, step=27.307, jump=True),
                         _pose_row(4, revision=1)])
    output = tmp_path / 'check.json'
    assert module.main(['--poses', str(poses), '--output', str(output)]) == 0
    report = json.loads(output.read_text(encoding='utf8'))
    assert report['world_frame_revision_supported'] is True
    assert report['reanchor_count'] == 1
    assert report['world_frame_revision_final'] == 1
    event = report['increments'][0]
    assert event['line'] == 3 and (event['revision_before'], event['revision_after']) == (0, 1)
    assert event['translation_step_m'] == 27.307 and event['jump_detected'] is True
    assert report['increment_epoch_relations'][0]['same_epoch'] is True
    assert report['map_epoch_distinct'] == [9]
    assert [item['line'] for item in report['steps_over_jump_m']] == [3]
    assert 'world_frame_revision final=1' in capsys.readouterr().out


def test_check_script_marks_legacy_log_without_revision(tmp_path, capsys):
    module = _load_check_script()
    poses = tmp_path / 'poses.jsonl'
    _write_poses(poses, [_pose_row(1), _pose_row(2, step=27.307, jump=True)])
    assert module.main(['--poses', str(poses)]) == 2
    assert 'NOT PRESENT' in capsys.readouterr().out
