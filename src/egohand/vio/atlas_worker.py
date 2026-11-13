"""Isolated ORB Atlas sessions for scanning and read-only localization."""
import json
import os
import select
import socket
import struct
import subprocess
import time
from contextlib import contextmanager
from pathlib import Path

from .live_subprocess import receive

ROOT = Path(__file__).resolve().parents[3]


def reply(connection, expected):
    size, = struct.unpack("<I", receive(connection, 4))
    if size > 65_536:
        raise RuntimeError("VIO 消息长度异常")
    message = json.loads(receive(connection, size))
    if message.get("type") != expected:
        raise RuntimeError(message.get("message") or f"VIO 返回意外消息 {message}")
    return message


@contextmanager
def atlas_worker(workspace, config_path, atlas=None, *, cancelled=None):
    settings = workspace / "orb.yaml"
    config = Path(config_path).read_text(encoding="utf-8-sig")
    if any(key in config for key in ("System.LoadAtlasFromFile", "System.SaveAtlasToFile")):
        raise ValueError("ORB 基础配置包含地图会话专用字段")
    if "loopClosing:" not in config:
        config += "\nloopClosing: 0\n"
    if atlas:
        if atlas.suffix != ".osa" or not atlas.is_file():
            raise ValueError("请选择可读的 .osa 定位地图")
        config += f'System.LoadAtlasFromFile: "{atlas.with_suffix("").as_posix()}"\n'
    settings.write_text(config, encoding="utf8")

    python = os.environ.get("EGOHAND_VIO_PYTHON") or str(
        Path.home() / "miniconda3/envs/egohand-vio/python.exe"
    )
    from .live_runtime import vio_subprocess_environment
    env = vio_subprocess_environment(python, role='atlas')
    env.update(PYTHONPATH=str(ROOT / "src"), EGOHAND_VIO_CONFIG=str(settings),
               EGOHAND_VIO_MAP_MODE="localize" if atlas else "build",
               EGOHAND_VIO_MAP_OUTPUT=str(workspace))
    with socket.socket() as listener, (workspace / "orb.log").open("wb") as log:
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(.25)
        proc = subprocess.Popen(
            [python, "-m", "egohand.vio.live_subprocess", str(listener.getsockname()[1])],
            stdout=log, stderr=subprocess.STDOUT, env=env,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        try:
            deadline = time.monotonic() + 30
            while True:
                if cancelled is not None and cancelled.is_set():
                    raise InterruptedError("地图会话已取消")
                try:
                    connection, _ = listener.accept()
                    break
                except socket.timeout:
                    if proc.poll() is not None or time.monotonic() >= deadline:
                        raise TimeoutError("VIO 地图进程未连接")
            with connection:
                connection.settimeout(120)
                deadline = time.monotonic() + 120
                while not select.select([connection], [], [], .25)[0]:
                    if cancelled is not None and cancelled.is_set():
                        raise InterruptedError("地图会话已取消")
                    if proc.poll() is not None or time.monotonic() >= deadline:
                        raise TimeoutError("VIO 地图进程未就绪")
                reply(connection, "ready")
                yield connection
        except (EOFError, OSError) as exc:
            raise RuntimeError(f"VIO 地图进程中断，请查看 {workspace / 'orb.log'}") from exc
        finally:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


