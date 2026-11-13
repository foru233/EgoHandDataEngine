"""Explicit bridge roles and hash validation; no version-name search chain."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path


def select_library(native_dir: Path, *, map_mode=None) -> Path:
    native_dir = Path(native_dir)
    explicit = os.environ.get('EGOHAND_VIO_LIBRARY')
    if explicit:
        library = Path(explicit).resolve()
        if not library.is_file():
            raise FileNotFoundError(f'指定的 VIO 桥接库不存在：{library}')
        return library
    registry = native_dir / 'bin' / 'active_bridges.json'
    role = 'atlas' if map_mode else 'live'
    if not registry.is_file():
        # One pinned deployment baseline per role. Historical DLLs are retained
        # for rollback, but are never selected by probing version numbers.
        filename = 'egohand_vio_atlas_bridge.dll' if map_mode else 'egohand_vio_recovery_bridge_v9.dll'
        return native_dir / 'bin' / filename
    data = json.loads(registry.read_text(encoding='utf8'))
    if data.get('schema_version') != 1:
        raise ValueError('不支持的 VIO 桥接库清单版本')
    entry = data['roles'][role]
    library = (registry.parent / entry['file']).resolve()
    if library.parent != registry.parent.resolve():
        raise ValueError('VIO 桥接库清单路径必须位于 bin 目录')
    actual = hashlib.sha256(library.read_bytes()).hexdigest()
    if actual != entry['sha256']:
        raise ValueError(f'VIO 桥接库校验失败：{library.name}，请重新构建或恢复部署清单')
    required = {'atlas_build', 'atlas_localize'} if map_mode else {'stereo_inertial'}
    if not required.issubset(entry.get('capabilities', [])):
        raise ValueError(f'VIO 桥接库不支持 {role} 所需能力')
    return library
