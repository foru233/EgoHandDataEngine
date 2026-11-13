"""Candidate binding and hardware exclusion at the real customer GUI boundary."""
import json
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'userrr'))
from client_gui import ClientMainWindow
from offline_recovery import OfflineRecoveryProfile

RECORDING = ROOT/'data/session_20261112_184714_636910800'


@pytest.fixture
def profile(monkeypatch):
    # configure_process deliberately changes only this test's environment.
    for key in list(os.environ):
        if key.startswith(('EGOHAND_VIO_', 'EGOHAND_TEST_')):
            monkeypatch.delenv(key)
    p = OfflineRecoveryProfile()
    for key, value in p.environment.items():monkeypatch.setenv(key, value)
    return p


def test_candidate_binds_complete_geometry_and_fresh_native_worker(qtbot, monkeypatch, tmp_path, profile):
    import client_gui
    w = ClientMainWindow(recovery_profile=profile, replay_output_dir=tmp_path/'run')
    qtbot.addWidget(w)
    called = {}
    monkeypatch.setattr(w.session, 'start', lambda source, **kw: called.update(kw))
    monkeypatch.setattr(client_gui, 'create_source', lambda request: called.update(request=request))
    try:
        w.select_recording(RECORDING);w.start_pipeline()
        assert w.last_error is None
        assert called['request'].kind == 'recorded'
        assert called['offline'] and called['live_vio']
        assert called['vio_worker'] is None
        assert called['vio_mode'] == 'stereo_inertial'
        assert called['vio_library_path'] == profile.paths['library']
        c = called['vio_config_snapshot']
        assert c.image_scale == 1 and not c.timing['session_phase_verified']
        assert c.timing['lattice_phase_mode'] == 'session_auto'
        assert called['calibration'].calibration_id.endswith('cam0-left')
        d = json.loads((tmp_path/'run/gui_launch.json').read_text(encoding='utf8'))
        assert d['files']['library']['sha256'] == profile.data['files']['library']['sha256']
        assert d['production_deployed'] is False
        w.sync_controls()
        assert not w.source.isEnabled() and not w.mode.isEnabled()
        assert '候选' in w.windowTitle() and '全分辨率' in w.profile_label.text()
    finally:w.close()


def test_candidate_rejects_hardware_before_source_creation(qtbot, monkeypatch, profile):
    import client_gui
    w = ClientMainWindow(recovery_profile=profile);qtbot.addWidget(w)
    monkeypatch.setattr(client_gui, 'create_source', lambda _: pytest.fail('hardware must never be constructed'))
    try:
        w.source.addItem('invalid hardware request', 'vendor');w.source.setCurrentIndex(1)
        w.start_pipeline()
        assert '仅支持' in w.last_error and w.session.worker is None
    finally:w.close()


def test_candidate_requires_policy_and_preserves_recording(profile, monkeypatch, tmp_path):
    monkeypatch.delenv('EGOHAND_VIO_SHORT_RECOVERY')
    with pytest.raises(ValueError, match='专用启动'):
        profile.worker_options(RECORDING, tmp_path/'run')
    monkeypatch.setenv('EGOHAND_VIO_SHORT_RECOVERY', '1')
    with pytest.raises(ValueError, match='原始录制之外'):
        profile.worker_options(RECORDING, RECORDING/'candidate_results')


def test_profile_rejects_changed_library_manifest(tmp_path):
    from offline_recovery import PROFILE
    d = json.loads(PROFILE.read_text(encoding='utf8'))
    d['files']['library']['sha256'] = '0'*64
    p = tmp_path/'bad.json';p.write_text(json.dumps(d), encoding='utf8')
    with pytest.raises(ValueError, match='校验失败'):OfflineRecoveryProfile(p)


def test_launcher_does_not_inherit_disabled_recovery_or_test_overrides(monkeypatch):
    p = OfflineRecoveryProfile()
    # Register all affected environment entries for fixture restoration.
    for key in list(os.environ):
        if key.startswith(('EGOHAND_VIO_', 'EGOHAND_TEST_')):monkeypatch.delenv(key)
    for key, value in p.environment.items():monkeypatch.setenv(key, value)
    monkeypatch.setenv('EGOHAND_VIO_DISABLE_RECOVERY', '1')
    monkeypatch.setenv('EGOHAND_TEST_OLD_DIAGNOSTIC', '1')
    p.configure_process()
    from egohand.vio.live_runtime import vio_subprocess_environment
    env = vio_subprocess_environment(recovery_protocol=2)
    assert 'EGOHAND_VIO_DISABLE_RECOVERY' not in env
    assert 'EGOHAND_TEST_OLD_DIAGNOSTIC' not in env
    assert env['EGOHAND_VIO_SHORT_RECOVERY'] == '1'
    assert env['EGOHAND_VIO_LIBRARY'] == str(p.paths['library'])
    assert all(env[k] == v for k, v in p.environment.items())


def test_normal_gui_keeps_production_modes_and_worker(qtbot, monkeypatch):
    w = ClientMainWindow();qtbot.addWidget(w)
    try:
        assert w.recovery_profile is None and w.source.findData('vendor') >= 0
        assert w.mode.count() == 5 and '正式 DLL' in w.mode.itemText(0)
        assert w.mode.currentData() == 'live_recovery_v4'
        assert w.source.isEnabled() and w.mode.isEnabled()
        called = {}
        monkeypatch.setattr(w.session, 'start', lambda source, **kw: called.update(kw))
        w.source.setCurrentIndex(w.source.findData('synthetic'));w.start_pipeline()
        assert called['vio_worker'] is w.live_vio_worker
        assert 'vio_library_path' not in called and 'vio_config_snapshot' not in called
    finally:w.close()


def test_normal_gui_recording_selects_isolated_candidate(qtbot, monkeypatch, tmp_path):
    import client_gui
    w=ClientMainWindow(replay_output_dir=tmp_path/'run');qtbot.addWidget(w)
    called={}
    monkeypatch.setattr(w.session,'start',lambda source,**kw: called.update(kw))
    monkeypatch.setattr(client_gui,'create_source',lambda request: called.update(request=request))
    try:
        w.select_recording(RECORDING)
        assert w.mode.currentData()=='offline_recovery_v4'
        w.start_pipeline()
        assert w.last_error is None and called['request'].kind=='recorded'
        assert called['vio_mode']=='stereo_inertial' and called['vio_worker'] is None
        assert called['vio_environment']['EGOHAND_VIO_SHORT_RECOVERY']=='1'
        assert called['vio_library_path'].name=='egohand_vio_short_recovery_candidate.dll'
    finally:w.close()


def test_candidate_host_quality_gate_uses_explicit_policy_without_global_environment(monkeypatch):
    from egohand.vio.hybrid_quality import make_quality_gate
    monkeypatch.delenv('EGOHAND_VIO_RECOVERY_CONFIG',raising=False)
    profile=OfflineRecoveryProfile()
    gate=make_quality_gate('stereo_inertial',recovery_config=profile.paths['recovery'])
    assert gate.allow_verified_stereo_bootstrap is True
