from types import SimpleNamespace

import numpy as np
import pytest

from egohand.vio.map_comparison import MapComparison
from egohand.vio.map_package import FILES, digest, read_package, write_package


def test_save_revision_preserves_original_and_atlas_coordinates(tmp_path):
    original = tmp_path / "original"
    original.mkdir()
    for name in ("atlas.osa", "calibration.yaml", "orb.yaml"):
        (original / name).write_bytes(name.encode())
    occupied = {(0, 0, 1), (0, 0, 2)}
    free = {(0, 0, 0), (1, 0, 0)}
    manifest = write_package(original, occupied, free, .1, 3, {})
    before = {name: digest(original / name) for name in (*FILES, "map.json")}
    rectifier = SimpleNamespace(P1=np.eye(3, 4), P2=np.eye(3, 4))
    rectifier.P2[0, 3] = -.065
    comparison = MapComparison(original, manifest, rectifier, 1)
    revision = tmp_path / "revision"
    comparison.save_revision(revision, ({(1, 0, 0)}, {(20, 20, 20)}, {(0, 0, 1)}))

    revised_manifest, points = read_package(revision)
    assert revised_manifest["version"] == 2
    assert revised_manifest["atlas_map_id"] == 3
    assert revised_manifest["sha256"]["atlas.osa"] == before["atlas.osa"]
    assert {name: digest(original / name) for name in before} == before
    np.testing.assert_allclose(points, [[.05, .05, .25], [.15, .05, .05]])
    with pytest.raises(FileExistsError):
        comparison.save_revision(revision, ({(1, 0, 0)}, set(), {(0, 0, 1)}))
