import json
import pytest
from egohand.vio import live_runtime


def registry(tmp_path, monkeypatch, roles):
    monkeypatch.setattr(live_runtime,'ROOT',tmp_path)
    monkeypatch.delenv('EGOHAND_VIO_LIBRARY',raising=False)
    folder=tmp_path/'src/egohand/vio/native/bin';folder.mkdir(parents=True)
    (folder/'active_bridges.json').write_text(json.dumps({'roles':roles}))


def test_atlas_and_live_consume_their_own_deployment_flags(tmp_path,monkeypatch):
    registry(tmp_path,monkeypatch,{
        'live':{'environment':{'EGOHAND_VIO_GAUGE_CONSTRAINT':'0'}},
        'atlas':{'environment':{'EGOHAND_VIO_GAUGE_CONSTRAINT':'1'}}})
    assert live_runtime.vio_subprocess_environment('python.exe')['EGOHAND_VIO_GAUGE_CONSTRAINT']=='0'
    assert live_runtime.vio_subprocess_environment('python.exe',role='atlas')['EGOHAND_VIO_GAUGE_CONSTRAINT']=='1'


def test_explicit_experiment_library_does_not_inherit_deployment_flags(tmp_path,monkeypatch):
    registry(tmp_path,monkeypatch,{'live':{'environment':{'EGOHAND_VIO_GAUGE_CONSTRAINT':'1'}}})
    monkeypatch.setenv('EGOHAND_VIO_LIBRARY','explicit.dll')
    monkeypatch.setenv('EGOHAND_VIO_GAUGE_CONSTRAINT','0')
    assert live_runtime.vio_subprocess_environment('python.exe')['EGOHAND_VIO_GAUGE_CONSTRAINT']=='0'


def test_deployment_cannot_enable_test_hooks(tmp_path,monkeypatch):
    registry(tmp_path,monkeypatch,{'live':{'environment':{'EGOHAND_VIO_ALLOW_TEST_HOOKS':'1'}}})
    with pytest.raises(ValueError,match='功能开关'):
        live_runtime.vio_subprocess_environment('python.exe')
