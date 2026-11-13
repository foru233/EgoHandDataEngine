import importlib.util
import json
from dataclasses import replace
from pathlib import Path

import pytest

from egohand.vio.config_snapshot import resolve_live_config, resolve_recording_config


spec = importlib.util.spec_from_file_location(
    'vio_experiment_runner', Path(__file__).resolve().parents[1] / 'scripts/run_vio_stability_experiment.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def recording(tmp_path):
    config = resolve_live_config()
    replace(config, timing={**config.timing, 'lattice_phase_mode': 'session_auto'}).write(tmp_path)
    original = resolve_recording_config(tmp_path)
    model = {'schema_version': 1, 'config_hashes': original.describe()['hashes'],
             'verification': {'verified': True, 'phase_ms': 34}}
    (tmp_path/'vio_session_phase.json').write_text(json.dumps({'schema_version': 1, 'models': [model]}), encoding='utf8')
    return tmp_path/'kalibr_camchain.yaml', tmp_path/'vio_settings.yaml'


def test_matching_model_is_verified_but_alternate_settings_are_rejected(tmp_path):
    cal, settings = recording(tmp_path)
    _, timing, evidence = runner.resolve_experiment_config(tmp_path, cal, settings)
    assert timing['session_phase_verified'] is True
    assert timing['lattice_phase_ms'] == 34
    assert evidence['status'] == 'verified_matching_recording_model'
    alternate = tmp_path/'alternate.yaml'
    alternate.write_bytes(settings.read_bytes() + b'\n# modified\n')
    with pytest.raises(ValueError, match='不匹配'):
        runner.resolve_experiment_config(tmp_path, cal, alternate)


def test_explicit_phase_is_diagnostic_and_never_inherits_verification(tmp_path):
    cal, settings = recording(tmp_path)
    alternate = tmp_path/'alternate.yaml'
    alternate.write_bytes(settings.read_bytes() + b'\n# modified\n')
    _, timing, evidence = runner.resolve_experiment_config(tmp_path, cal, alternate, 105, 1.0)
    assert timing['lattice_phase_mode'] == 'manual'
    assert 'session_phase_verified' not in timing
    assert evidence['diagnostic_only'] is True
    assert evidence['status'] == 'explicit_fixed_phase_diagnostic'


def test_tampered_recording_snapshot_fails_even_with_explicit_phase(tmp_path):
    cal, settings = recording(tmp_path)
    settings.write_bytes(settings.read_bytes() + b'\n# tamper\n')
    with pytest.raises(ValueError, match='快照校验失败'):
        runner.resolve_experiment_config(tmp_path, cal, settings, 105)


def test_auto_phase_requires_model_unless_explicit_diagnostic(tmp_path):
    cal, settings = recording(tmp_path)
    (tmp_path/'vio_session_phase.json').unlink()
    with pytest.raises(ValueError, match='缺少匹配'):
        runner.resolve_experiment_config(tmp_path, cal, settings)
    _, timing, evidence = runner.resolve_experiment_config(tmp_path, cal, settings, 105)
    assert timing['lattice_phase_mode'] == 'manual'
    assert evidence['diagnostic_only']


def test_raw_manifest_detects_changes_and_rejects_empty_recording(tmp_path):
    raw = tmp_path/'raw'
    raw.mkdir()
    with pytest.raises(ValueError, match='为空'):
        runner.raw_input_manifest(tmp_path)
    (raw/'samples.bin').write_bytes(b'original')
    before = runner.raw_input_manifest(tmp_path)
    assert before['samples.bin']['size_bytes'] == 8
    assert before == runner.raw_input_manifest(tmp_path)
    (raw/'samples.bin').write_bytes(b'modified')  # same size, changed content
    assert runner.raw_input_manifest(tmp_path) != before
