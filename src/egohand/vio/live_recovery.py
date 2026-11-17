"""Explicit VS030 live recovery candidate profile.

The profile is opt-in and is passed to the isolated VIO child.  It never
changes the production deployment manifest or the parent GUI environment.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from ..calibration.loader import load_calibration
from .config_snapshot import load_config_snapshot

ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "userrr/runtime_configs/live_recovery_candidate.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class LiveRecoveryProfile:
    """Validate and describe the v4 live candidate without selecting it silently."""

    label = "双目＋IMU · v4 实时恢复实验版 · 精度待验收"
    mode = "stereo_inertial"

    def __init__(self, manifest=PROFILE):
        self.manifest = Path(manifest).resolve()
        try:
            self.data = json.loads(self.manifest.read_text(encoding="utf8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"无法读取 v4 实时候选 Profile：{self.manifest}") from exc
        if self.data.get("schema_version") != 1 or self.data.get("profile") != "live_recovery_candidate_v4":
            raise ValueError("不支持的 v4 实时候选 Profile 版本")
        self.label = self.data.get("label", self.label)
        self.paths = {
            key: (ROOT / value["path"]).resolve()
            for key, value in self.data.get("files", {}).items()
        }
        required = {"calibration", "settings", "timing", "quality", "library", "recovery"}
        missing = sorted(required - set(self.paths))
        if missing:
            raise ValueError("v4 实时候选 Profile 缺少文件：" + ", ".join(missing))
        self.verify_files()
        self._verify_abi()
        self.config_snapshot = load_config_snapshot(
            calibration=self.paths["calibration"], settings=self.paths["settings"],
            timing=self.paths["timing"], quality=self.paths["quality"],
            source="live_recovery_candidate_v4",
        )
        self.geometry = self.config_snapshot.validate_geometry()
        self.calibration = load_calibration(self.paths["calibration"], kalibr_left_camera="cam0")

    def verify_files(self):
        for key, path in self.paths.items():
            if not path.is_file():
                raise FileNotFoundError(f"v4 实时候选文件不存在：{path}")
            expected = self.data["files"][key].get("sha256")
            actual = sha256(path)
            if expected != actual:
                raise ValueError(f"v4 实时候选文件校验失败：{path}")

    def _verify_abi(self):
        """The short-recovery symbol is the ABI marker for this candidate DLL."""
        # Loading a Windows DLL from the GUI environment can fail only because
        # its native dependency directory is not on PATH.  The exported symbol
        # is still present in the PE export/string table, so inspect the binary
        # marker here and let the isolated child load it with its own PATH.
        marker = b"egohand_vio_short_recovery_report"
        if marker not in self.paths["library"].read_bytes():
            raise ValueError("v4 候选 DLL 不支持短时恢复 ABI，已拒绝启动")
        recovery = self.data.get("recovery", {})
        if recovery.get("short_recovery") is not True:
            raise ValueError("v4 候选 Profile 未启用 short_recovery")

    @property
    def environment(self):
        environment = dict(self.data.get("environment", {}))
        environment.update({
            "EGOHAND_VIO_LIBRARY": str(self.paths["library"]),
            "EGOHAND_VIO_RECOVERY_CONFIG": str(self.paths["recovery"]),
            "EGOHAND_VIO_SHORT_RECOVERY": "1",
        })
        return environment

    @property
    def provenance(self):
        return {
            "schema_version": 1,
            "profile": self.data.get("profile"),
            "label": self.label,
            "manifest": str(self.manifest),
            "manifest_sha256": sha256(self.manifest),
            "files": {
                key: {"path": str(path), "sha256": sha256(path)}
                for key, path in self.paths.items()
            },
            "environment": self.environment,
            "config": self.config_snapshot.describe(),
            "calibration_id": self.calibration.calibration_id,
            "production_deployed": bool(self.data.get("delivery_selected", False)),
            "release_id": self.data.get("release_id"),
        }

    def worker_options(self):
        """Arguments shared by customer and research live workers."""
        self.verify_files()
        return {
            "vio_config_snapshot": self.config_snapshot,
            "vio_library_path": self.paths["library"],
            "vio_environment": self.environment,
            "calibration": self.calibration,
        }

    def recording_options(self, recording):
        """Bind the same candidate to a recording's own timing snapshot."""
        self.verify_files()
        config = load_config_snapshot(calibration=self.paths['calibration'],
            settings=self.paths['settings'], timing=Path(recording)/'vio_timing.yaml',
            quality=self.paths['quality'], source='offline_recovery_candidate_v4',
            timing_override={'image_scale':1.0,'lattice_phase_mode':'session_auto',
                             'session_phase_verified':False})
        config.validate_geometry()
        return {'config_snapshot':config,'library_path':self.paths['library'],
                'environment':self.environment}
