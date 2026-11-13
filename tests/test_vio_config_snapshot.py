"""Configuration provenance and geometry checks across VIO consumers."""
from dataclasses import replace
from pathlib import Path

import pytest

from egohand.vio.config_snapshot import (
    resolve_live_config, resolve_recording_config,
)


def test_recording_requires_complete_snapshot_and_explicit_override(tmp_path):
    with pytest.raises(FileNotFoundError, match='显式指定完整配置'):
        resolve_recording_config(tmp_path)
    live = resolve_live_config()
    override = resolve_recording_config(tmp_path, override=live)
    assert override.source.startswith('explicit_override:')
    assert override.settings == live.settings


def test_recording_snapshot_freezes_settings_and_hash_identity(tmp_path):
    live = resolve_live_config()
    live.write(tmp_path)
    config = resolve_recording_config(tmp_path)
    assert config.settings == tmp_path / 'vio_settings.yaml'
    assert config.calibration == tmp_path / 'kalibr_camchain.yaml'
    assert config.describe()['hashes']['settings'] == live.describe()['hashes']['settings']
    # A stale path embedded in timing does not redirect the stored ORB settings.
    timing = tmp_path / 'vio_timing.yaml'
    timing.write_text(timing.read_text(encoding='utf8') + '\norb_config: DOES_NOT_EXIST.yaml\n', encoding='utf8')
    with pytest.raises(ValueError, match='快照校验失败'):
        resolve_recording_config(tmp_path)
    # Legacy four-file snapshots lack the new descriptor but still resolve their local settings.
    (tmp_path / 'vio_config_snapshot.json').unlink()
    changed = resolve_recording_config(tmp_path)
    assert changed.settings == config.settings
    assert changed.describe()['fingerprint'] != config.describe()['fingerprint']


def test_live_and_snapshot_geometry_agree_and_bad_scale_is_rejected(tmp_path):
    live = resolve_live_config()
    geometry = live.validate_geometry()
    live.write(tmp_path)
    recorded = resolve_recording_config(tmp_path)
    recorded.validate_geometry(geometry)
    wrong = replace(recorded, timing={**recorded.timing, 'image_scale': 1.0})
    with pytest.raises(ValueError, match='不兼容'):
        wrong.validate_geometry(geometry)


def test_lattice_phase_override_does_not_add_legacy_lag(tmp_path):
    resolve_live_config().write(tmp_path)
    base = resolve_recording_config(tmp_path)
    modified = resolve_recording_config(tmp_path, timing_override={'lattice_phase_ms': 110.0})
    assert modified.timing['camera_to_imu_lag_ms'] == base.timing['camera_to_imu_lag_ms']
    assert modified.timing['lattice_phase_ms'] == 110.0
    assert modified.describe()['fingerprint'] != base.describe()['fingerprint']


def test_snapshot_missing_quality_never_falls_back_to_repository(tmp_path):
    resolve_live_config().write(tmp_path)
    (tmp_path / 'vio_quality.yaml').unlink()
    with pytest.raises(FileNotFoundError, match='vio_quality.yaml'):
        resolve_recording_config(tmp_path)


def test_written_snapshot_hashes_cover_actual_saved_normalized_timing(tmp_path):
    import json
    resolve_live_config().write(tmp_path)
    config = resolve_recording_config(tmp_path)
    descriptor = json.loads((tmp_path / 'vio_config_snapshot.json').read_text(encoding='utf8'))
    assert descriptor['hashes'] == config.describe()['hashes']
    assert descriptor['fingerprint'] == config.describe()['fingerprint']
