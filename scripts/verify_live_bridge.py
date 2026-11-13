"""实机预检：报告实时链路会选中哪个桥接库、它的能力/ABI/计数器是否正常。

只读，不修改任何配置、不启动相机、不写日志目录。

用法：
    .\\.venv\\Scripts\\python.exe scripts\\verify_live_bridge.py
    .\\.venv\\Scripts\\python.exe scripts\\verify_live_bridge.py --library src\\egohand\\vio\\native\\bin\\egohand_vio_safe_exp.dll
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

# 由 egohand-vio 环境加载，因为桥接库依赖该环境里的 OpenCV/TBB 运行库。
PROBE = r'''
import ctypes, json, os, sys
path = sys.argv[1]
lib = ctypes.CDLL(path)
out = {"path": path}

def call(name, restype):
    fn = getattr(lib, name, None)
    if fn is None:
        return None
    fn.restype = restype
    fn.argtypes = []
    try:
        return fn()
    except Exception:
        return None

caps = getattr(lib, "egohand_vio_capabilities", None)
if caps is not None:
    caps.restype = ctypes.c_char_p
    raw = caps()
    out["capabilities"] = raw.decode("utf8", "replace").split(";") if raw else []
else:
    out["capabilities"] = []

size = call("egohand_vio_world_frame_info_size", ctypes.c_int32)
out["world_frame_info_size"] = int(size) if size is not None else None
api = getattr(lib, "egohand_vio_api_version", None)
if api is not None:
    api.restype = ctypes.c_char_p
    out["api_version"] = (api() or b"").decode("utf8", "replace")
out["has_result_size"] = call("egohand_vio_result_size", ctypes.c_int32)
out['test_hooks_present'] = hasattr(lib, 'egohand_vio_test_inject_numerical_fault')
for sym in ("egohand_vio_point_clamp_count", "egohand_vio_depth_gate_rejected", "egohand_vio_numerical_fault_count",
            "egohand_vio_entry_pruned", "egohand_vio_invalid_depth_checks",
            "egohand_vio_invalid_depth_removed", "egohand_vio_invalid_depth_points"):
    out[sym] = call(sym, ctypes.c_uint64)
print(json.dumps(out))
'''


def live_environment_python() -> Path:
    """与 live_runtime 一致地解析 egohand-vio 的解释器。"""
    import os
    explicit = os.environ.get("EGOHAND_VIO_PYTHON")
    if explicit:
        return Path(explicit)
    candidate = Path.home() / "miniconda3" / "envs" / "egohand-vio" / "python.exe"
    return candidate if candidate.is_file() else Path("python.exe")


def main(argv=None):
    parser = argparse.ArgumentParser(description="实时 VIO 桥接库预检（只读）")
    parser.add_argument("--library", help="显式检查某个 DLL；默认走 library_manifest 的选择逻辑")
    args = parser.parse_args(argv)

    from egohand.vio.library_manifest import select_library

    native_dir = ROOT / "src" / "egohand" / "vio" / "native"
    if args.library:
        library = Path(args.library)
        if not library.is_file():
            raise SystemExit(f"指定的库不存在：{library}")
        source = "命令行 --library"
    else:
        library = select_library(native_dir)
        registry = native_dir / "bin" / "active_bridges.json"
        source = (f"active_bridges.json（{registry.name}）" if registry.is_file()
                  else "硬编码回退（无 active_bridges.json → live=v9 / atlas=atlas_bridge）")

    print(f"实时链路将加载：{library}")
    print(f"选择依据       ：{source}")
    print(f"文件时间       ：{library.stat().st_mtime_ns and __import__('datetime').datetime.fromtimestamp(library.stat().st_mtime)}")

    python = live_environment_python()
    if not python.is_file():
        print(f"⚠ 未找到 egohand-vio 解释器（{python}），跳过能力探测")
        return 0
    proc = subprocess.run([str(python), "-c", PROBE, str(library)],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"⚠ 能力探测失败（{proc.returncode}）：{proc.stderr.strip()[:400]}")
        return 1

    info = json.loads(proc.stdout.strip().splitlines()[-1])
    print(f"api_version   ：{info.get('api_version') or '—'}")
    print(f"capabilities  ：{', '.join(info.get('capabilities') or []) or '—'}")
    print(f"world_frame_info_size：{info.get('world_frame_info_size')}"
          f"（应与其他库一致，当前基线 144）")
    print(f"result_size   ：{info.get('has_result_size')}")
    print(f"故障注入测试接口：{info.get('test_hooks_present')}（正式库必须为 False）")
    if info.get('test_hooks_present'):
        print('拒绝将故障注入测试库用于正式采集。')
        return 1
    counters = {k: v for k, v in info.items() if k.startswith("egohand_vio_") and k != "egohand_vio_result_size"}
    if counters:
        print(f"可选计数器    ：{counters}")

    # 与在用配置的期望能力对齐检查
    timing = ROOT / "configs" / "vio" / "live_timing.yaml"
    if timing.is_file():
        text = timing.read_text(encoding="utf8")
        for line in text.splitlines():
            if line.startswith("orb_config:"):
                print(f"在用 ORB 配置 ：{line.split(':', 1)[1].strip()}")
        print("提示：切换库后必须重启 GUI（或至少重启 live worker），运行中的进程不会热加载 DLL。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
