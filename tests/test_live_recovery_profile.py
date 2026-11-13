import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'userrr'))


def test_live_candidate_profile_is_explicit_and_geometry_bound():
    from egohand.vio.live_recovery import LiveRecoveryProfile

    profile = LiveRecoveryProfile()
    options = profile.worker_options()
    assert options['vio_library_path'] == profile.paths['library']
    assert options['vio_environment']['EGOHAND_VIO_LIBRARY'] == str(profile.paths['library'])
    assert options['vio_environment']['EGOHAND_VIO_RECOVERY_CONFIG'] == str(profile.paths['recovery'])
    assert options['vio_config_snapshot'].image_scale == 1.0
    assert profile.data['files']['library']['sha256'] != '0' * 64
    assert profile.provenance['production_deployed'] is False


def test_live_candidate_profile_rejects_changed_timing(tmp_path):
    from egohand.vio.live_recovery import PROFILE, LiveRecoveryProfile

    data = json.loads(PROFILE.read_text(encoding='utf8'))
    data['files']['timing']['sha256'] = '0' * 64
    bad = tmp_path / 'bad.json'
    bad.write_text(json.dumps(data), encoding='utf8')
    with pytest.raises(ValueError, match='校验失败'):
        LiveRecoveryProfile(bad)


def test_customer_live_candidate_is_bound_to_vs030_and_profile(qtbot, monkeypatch):
    from client_gui import ClientMainWindow
    from egohand.vio.live_recovery import LiveRecoveryProfile
    import client_gui

    profile = LiveRecoveryProfile()
    window = ClientMainWindow(live_recovery_profile=profile)
    qtbot.addWidget(window)
    called = {}
    monkeypatch.setattr(window.session, 'start', lambda source, **options: called.update(source=source, options=options))
    monkeypatch.setattr(client_gui, 'create_source', lambda request: called.update(request=request) or object())
    try:
        window.start_pipeline()
        assert called['request'].kind == 'vendor'
        assert called['options']['live_vio'] is True
        assert called['options']['vio_library_path'] == profile.paths['library']
        assert called['options']['vio_environment']['EGOHAND_VIO_SHORT_RECOVERY'] == '1'
        assert called['options']['calibration'].calibration_id == profile.calibration.calibration_id
        assert 'v4 实时恢复' in window.mode.currentText()
    finally:
        window.close()


def test_research_gui_exposes_same_v4_profile_choice(qtbot):
    from egohand.gui.main_window import MainWindow

    window = MainWindow()
    qtbot.addWidget(window)
    try:
        assert window.vio_panel.live_profile.findData('v4') >= 0
        assert window.vio_panel.live_profile.itemText(window.vio_panel.live_profile.findData('v4')) == 'v4 实时恢复实验版'
    finally:
        window.close()
