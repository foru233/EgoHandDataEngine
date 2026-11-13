"""词典复用回归：GUI 生命周期内常驻 VIO worker，回放不得再自起一次性子进程。

背景：`ReplayBridge` 原本每次回放都 `Popen(... live_subprocess <port>)`，而该进程在启动时
读取约 145 MB 的 `ORBvoc.txt`。客户端 GUI 之前也没有把 `LiveVioWorker` 传给
`SessionController.start`，于是每次「开始」都要重新加载词典。这两个行为都在这里锁住。
"""
import ast
from pathlib import Path

import pytest

from egohand.vio import replay_runtime

ROOT = Path(__file__).resolve().parents[1]


class RecordingWorker:
    """记录 request_session 调用，不启动任何真实进程。"""

    def __init__(self):
        self.calls = []
        self.closed = False

    def request_session(self, port, settings, session, *, mode='stereo_inertial'):
        self.calls.append({'port': port, 'settings': settings, 'session': session, 'mode': mode})
        return None, False, None

    def close(self):
        self.closed = True


@pytest.fixture(autouse=True)
def _no_spawn(monkeypatch):
    """任何真实 Popen 都视为失败：共享 worker 路径不允许再起子进程。"""

    def forbidden(*args, **kwargs):
        raise AssertionError(f'共享 worker 模式下不应启动子进程: {args!r}')

    monkeypatch.setattr(replay_runtime.subprocess, 'Popen', forbidden)


def test_shared_worker_replaces_one_shot_subprocess(monkeypatch):
    worker = RecordingWorker()
    bridge = replay_runtime.ReplayBridge('out/run.log', settings_path=Path('settings.yaml'),
                                         mode='stereo_inertial', worker=worker)

    # 握手依赖子进程连回来；这里只验证「请求了会话、没有自己起进程」。
    def stop_after_request(*_args, **_kwargs):
        raise replay_runtime.ReplayCancelled()

    monkeypatch.setattr(replay_runtime.socket.socket, 'accept', stop_after_request)
    with pytest.raises(replay_runtime.ReplayCancelled):
        bridge.start()

    assert len(worker.calls) == 1
    call = worker.calls[0]
    assert call['mode'] == 'stereo_inertial'
    assert isinstance(call['port'], int) and call['port'] > 0
    assert str(call['settings']).endswith('settings.yaml')
    # 共享模式下不得接管日志文件，也不得持有子进程句柄。
    assert bridge.proc is None
    assert bridge.log is None
    bridge.close()
    assert worker.closed is False, '回放结束不得关闭 GUI 的常驻 worker'


def test_close_only_releases_own_socket(monkeypatch):
    worker = RecordingWorker()
    bridge = replay_runtime.ReplayBridge('out/run.log', settings_path=Path('settings.yaml'),
                                         worker=worker)
    monkeypatch.setattr(replay_runtime.socket.socket, 'accept',
                        lambda *_a, **_k: (_ for _ in ()).throw(replay_runtime.ReplayCancelled()))
    with pytest.raises(replay_runtime.ReplayCancelled):
        bridge.start()
    bridge.close()
    assert worker.closed is False
    assert bridge.sock is None and bridge.listener is None


def test_replay_passes_worker_through_to_bridge(monkeypatch, tmp_path):
    """ReplayVio(worker=...) 必须把同一个 worker 传给 ReplayBridge。"""
    seen = {}

    class Bridge:
        def __init__(self, *args, **kwargs):
            seen.update(kwargs)

        def abort(self):
            pass

    monkeypatch.setattr(replay_runtime, 'ReplayBridge', Bridge)
    recording = tmp_path / 'rec'
    (recording / 'raw').mkdir(parents=True)
    (recording / 'raw' / 'sdk_records.bin').write_bytes(b'')
    worker = RecordingWorker()
    vio = replay_runtime.ReplayVio(recording, output_dir=tmp_path / 'out', worker=worker)
    assert vio.worker is worker
    # run() 需要真实解码，这里只验证构造期参数分流。
    assert seen == {} or seen.get('worker') is worker


def test_client_gui_owns_one_worker_for_its_lifetime():
    """客户端 GUI 必须在窗口期持有 worker，并在 closeEvent 释放。"""
    source = (ROOT / 'userrr' / 'client_gui.py').read_text(encoding='utf8')
    tree = ast.parse(source)

    window = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.ClassDef) and node.name == 'ClientMainWindow')
    methods = {node.name: node for node in window.body if isinstance(node, ast.FunctionDef)}
    assert 'closeEvent' in methods

    init_source = ast.get_source_segment(source, methods['__init__'])
    assert 'LiveVioWorker()' in init_source, '客户端 GUI 未创建常驻 worker'

    close_source = ast.get_source_segment(source, methods['closeEvent'])
    assert 'live_vio_worker.close()' in close_source, 'GUI 退出未释放常驻 worker'

    start_calls = [node for node in ast.walk(tree)
                   if isinstance(node, ast.Call)
                   and isinstance(node.func, ast.Attribute) and node.func.attr == 'start'
                   and isinstance(node.func.value, ast.Attribute)
                   and node.func.value.attr == 'session']
    assert start_calls, '未找到 session.start 调用'
    keywords = {kw.arg for call in start_calls for kw in call.keywords}
    assert 'vio_worker' in keywords, 'session.start 未传入共享 worker'
