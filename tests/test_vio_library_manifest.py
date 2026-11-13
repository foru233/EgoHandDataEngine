import hashlib
import json
import pytest
from egohand.vio.library_manifest import select_library


def test_manifest_does_not_guess_newer_dll(tmp_path, monkeypatch):
    monkeypatch.delenv('EGOHAND_VIO_LIBRARY', raising=False)
    binary = tmp_path / 'bin'
    binary.mkdir()
    (binary / 'egohand_vio_recovery_bridge_v999.dll').write_bytes(b'not selected')
    assert select_library(tmp_path).name == 'egohand_vio_recovery_bridge_v9.dll'


def test_manifest_verifies_hash_and_capabilities(tmp_path, monkeypatch):
    monkeypatch.delenv('EGOHAND_VIO_LIBRARY', raising=False)
    binary = tmp_path / 'bin'
    binary.mkdir()
    library = binary / 'verified.dll'
    library.write_bytes(b'valid library')
    manifest = {'schema_version': 1, 'roles': {'live': {'file': library.name,
                'sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
                'capabilities': ['stereo_inertial']}}}
    (binary / 'active_bridges.json').write_text(json.dumps(manifest))
    assert select_library(tmp_path) == library
    library.write_bytes(b'mismatched build')
    with pytest.raises(ValueError, match='校验失败'):
        select_library(tmp_path)


def test_explicit_missing_library_fails_without_fallback(tmp_path, monkeypatch):
    monkeypatch.setenv('EGOHAND_VIO_LIBRARY', str(tmp_path / 'missing.dll'))
    with pytest.raises(FileNotFoundError):
        select_library(tmp_path)
