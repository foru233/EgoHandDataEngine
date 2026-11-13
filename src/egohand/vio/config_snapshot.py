"""Versioned VIO inputs shared by live capture, replay and mapping.

Recording replay is strict: missing snapshots need an explicit complete override.
The legacy lag value is retained for timestamp experiments, never added to lattice phase.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import shutil
import math
from dataclasses import replace

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
SNAPSHOT_FILES = {"calibration": "kalibr_camchain.yaml", "settings": "vio_settings.yaml",
                  "timing": "vio_timing.yaml", "quality": "vio_quality.yaml"}


def _yaml(path):
    try:
        value = yaml.safe_load(Path(path).read_text(encoding="utf-8-sig")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"无法读取 VIO 配置 {path}：{exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"VIO 配置必须是字典：{path}")
    return value


@dataclass(frozen=True)
class VioConfigSnapshot:
    calibration: Path
    settings: Path
    timing_path: Path
    quality_path: Path
    timing: dict
    quality: dict
    source: str

    @property
    def image_scale(self):
        return float(self.timing.get("image_scale", 1.0))

    def describe(self):
        paths = {"calibration": self.calibration, "settings": self.settings,
                 "timing": self.timing_path, "quality": self.quality_path}
        hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in paths.items()}
        identity = {"schema_version": 1, "hashes": hashes, "effective_timing": self.timing}
        fingerprint = hashlib.sha256(json.dumps(identity, sort_keys=True, ensure_ascii=True).encode()).hexdigest()
        return {**identity, "fingerprint": fingerprint, "source": self.source,
                "paths": {name: str(path) for name, path in paths.items()},
                "coordinates": {"units": "m", "gui_left": "cam0/channel4", "orb_left": "cam1/channel5"}}

    def validate_geometry(self, geometry=None):
        """Check rectification/config consistency; does not prove calibration accuracy."""
        if geometry is None:
            from .vs030_geometry import prepare_vs030_vio_geometry
            geometry = prepare_vs030_vio_geometry(self.calibration)
        scale = self.image_scale
        if not np.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError("VIO 图像缩放须在 (0,1] 内")
        fs = cv2.FileStorage(str(self.settings), cv2.FILE_STORAGE_READ)
        try:
            if not fs.isOpened():
                raise ValueError(f"无法读取 ORB 配置：{self.settings}")
            for key, i, j in [("Camera1.fx", 0, 0), ("Camera1.fy", 1, 1),
                              ("Camera1.cx", 0, 2), ("Camera1.cy", 1, 2)]:
                actual = fs.getNode(key).real()
                expected = geometry.rectifier.P1[i, j] * scale
                if fs.getNode(key).empty() or not np.isclose(actual, expected, rtol=1e-6, atol=1e-5):
                    raise ValueError(f"标定与 ORB 配置不兼容：{key}={actual}，校正缩放后应为 {expected}")
            height, width = geometry.rectifier.maps_left[0].shape
            for key, expected in [("Camera.width", round(width * scale)),
                                  ("Camera.height", round(height * scale))]:
                if fs.getNode(key).real() != expected:
                    raise ValueError(f"标定与 ORB 图像尺寸不兼容：{key} 应为 {expected}")
            baseline = -geometry.rectifier.P2[0, 3] / geometry.rectifier.P2[0, 0]
            if not np.isclose(fs.getNode("Stereo.b").real(), baseline, rtol=1e-5, atol=1e-7):
                raise ValueError("标定与 ORB 双目基线不兼容")
            transform = fs.getNode("IMU.T_b_c1").mat()
            if transform is None or not np.allclose(transform, geometry.T_imu_from_orb_left, atol=1e-6):
                raise ValueError("标定与 ORB 相机—IMU 外参不兼容")
        finally:
            fs.release()
        return geometry

    def write(self, directory):
        """Copy a complete snapshot to a new result directory, never alter source files."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        for key, source in [("calibration", self.calibration), ("settings", self.settings),
                            ("quality", self.quality_path)]:
            destination = directory / SNAPSHOT_FILES[key]
            if destination.resolve() != source.resolve():
                shutil.copyfile(source, destination)
        (directory / SNAPSHOT_FILES["timing"]).write_text(
            yaml.safe_dump(self.timing, allow_unicode=True, sort_keys=True), encoding="utf8")
        written = load_config_snapshot(**{key: directory / name for key, name in SNAPSHOT_FILES.items()},
                                       source="recorded_snapshot")
        descriptor = written.describe()
        descriptor["source_snapshot"] = self.describe()
        (directory / "vio_config_snapshot.json").write_text(
            json.dumps(descriptor, indent=2, ensure_ascii=False), encoding="utf8")


def load_config_snapshot(*, calibration, settings, timing, quality, source="explicit", timing_override=None):
    paths = [Path(path).resolve() for path in (calibration, settings, timing, quality)]
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("VIO 配置快照不完整，须显式指定完整配置：" + ", ".join(missing))
    options = _yaml(paths[2])
    if timing_override:
        options.update({key: value for key, value in timing_override.items() if value is not None})
    return VioConfigSnapshot(paths[0], paths[1], paths[2], paths[3], options, _yaml(paths[3]), source)


def resolve_recording_config(recording, *, override=None, timing_override=None):
    if override is not None:
        if not isinstance(override, VioConfigSnapshot):
            raise TypeError("回放配置覆盖必须是完整 VioConfigSnapshot")
        return load_config_snapshot(calibration=override.calibration, settings=override.settings,
            timing=override.timing_path, quality=override.quality_path,
            source=f"explicit_override:{override.source}",
            timing_override={**override.timing, **(timing_override or {})})
    root = Path(recording)
    config = load_config_snapshot(**{key: root / value for key, value in SNAPSHOT_FILES.items()},
        source=f"recording:{root.resolve()}", timing_override=timing_override)
    descriptor_path = root / "vio_config_snapshot.json"
    if descriptor_path.is_file():
        try:
            descriptor = json.loads(descriptor_path.read_text(encoding="utf8"))
            if descriptor.get("schema_version") != 1:
                raise ValueError("不支持的 VIO 配置快照版本")
            if descriptor.get("hashes") != config.describe()["hashes"]:
                raise ValueError("VIO 配置快照校验失败，文件已发生变化；请显式选择完整覆盖配置")
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"VIO 配置快照描述无法读取：{descriptor_path}") from exc
    model_path = root / 'vio_session_phase.json'
    if config.timing.get('lattice_phase_mode')=='session_auto' and model_path.is_file():
        try:
            data=json.loads(model_path.read_text(encoding='utf8'))
        except (OSError,json.JSONDecodeError) as exc:
            raise ValueError(f'无法读取录制时序模型：{model_path}') from exc
        models=data.get('models') if isinstance(data,dict) else None
        if (not isinstance(models,list) or len(models)!=1 or data.get('schema_version')!=1
                or not isinstance(models[0],dict)):
            raise ValueError('录制包含多个时序会话或模型缺失，请按会话分段核验后回放')
        model=models[0]
        verified=model.get('verification',{})
        hashes=config.describe()['hashes']
        if (not isinstance(verified,dict) or not isinstance(model.get('config_hashes'),dict)
                or model.get('schema_version')!=1 or verified.get('verified') is not True
                or any(model.get('config_hashes',{}).get(k)!=hashes[k] for k in ('calibration','settings'))):
            raise ValueError('录制时序模型未通过核验或与标定/ORB 配置不匹配')
        try:phase=float(verified['phase_ms'])
        except (KeyError,TypeError,ValueError) as exc:raise ValueError('录制时序模型相位无效') from exc
        if not math.isfinite(phase) or not -150<=phase<=200:
            raise ValueError('录制时序模型相位无效')
        if timing_override and 'lattice_phase_ms' in timing_override:
            raise ValueError('覆盖已确认的时序模型须显式提供完整配置')
        config=replace(config,timing={**config.timing,'lattice_phase_ms':phase,
                                      'session_phase_verified':True})
    return config


def resolve_live_config():
    from ..sources.vendor_placeholder import CALIBRATION_PATH
    timing = ROOT / "configs/vio/live_timing.yaml"
    settings = Path(str(_yaml(timing).get("orb_config", "")))
    if not settings.is_absolute():
        settings = ROOT / settings
    return load_config_snapshot(calibration=CALIBRATION_PATH, settings=settings, timing=timing,
        quality=ROOT / "configs/vio/live_quality.yaml", source="live")
