"""Portable VS030 map package storage."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from uuid import uuid4

import numpy as np

from .map_depth import voxel_centers

FILES = ("atlas.osa", "cloud.ply", "space.npz", "calibration.yaml", "orb.yaml")


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_package(folder, occupied, free, voxel_size_m, atlas_map_id, processing, *, previous=None):
    if not occupied:
        raise ValueError("没有从多个视角确认的表面，请补拍有纹理的区域")
    points = voxel_centers(occupied, voxel_size_m)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n"
    )
    with (folder / "cloud.ply").open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(points.astype("<f4").tobytes())
    np.savez_compressed(
        folder / "space.npz", occupied=np.asarray(sorted(occupied), dtype=np.int32).reshape(-1, 3),
        free=np.asarray(sorted(free), dtype=np.int32).reshape(-1, 3),
    )
    manifest = {
        "format": "egohand-vs030-map",
        "schema": 1,
        "map_id": previous["map_id"] if previous else str(uuid4()),
        "version": previous["version"] + 1 if previous else 1,
        "atlas_map_id": atlas_map_id,
        "units": "metres",
        "coordinate_frame": "orb_atlas_world",
        "voxel_size_m": voxel_size_m,
        "processing": processing,
        "sha256": {name: digest(folder / name) for name in FILES},
    }
    if previous:
        manifest["parent_atlas_sha256"] = previous["sha256"]["atlas.osa"]
        manifest["parent_version"] = previous["version"]
    (folder / "map.json").write_text(json.dumps(manifest, indent=2), encoding="utf8")
    return manifest


def read_package(folder):
    folder = Path(folder)
    manifest = json.loads((folder / "map.json").read_text(encoding="utf8"))
    if (manifest.get("format"), manifest.get("schema"), manifest.get("units"),
        manifest.get("coordinate_frame")) != ("egohand-vs030-map", 1, "metres", "orb_atlas_world"):
        raise ValueError("地图包格式或坐标系不兼容")
    for name in FILES:
        if digest(folder / name) != manifest["sha256"][name]:
            raise ValueError(f"地图包文件校验失败：{name}")
    with np.load(folder / "space.npz", allow_pickle=False) as space:
        occupied, free = space["occupied"], space["free"]
        if any(keys.ndim != 2 or keys.shape[1] != 3 or keys.dtype.kind != "i"
               for keys in (occupied, free)):
            raise ValueError("地图体素数据格式错误")
    size = float(manifest["voxel_size_m"])
    if not np.isfinite(size) or size <= 0:
        raise ValueError("地图体素尺寸无效")
    return manifest, (occupied.astype(np.float32) + .5) * size

