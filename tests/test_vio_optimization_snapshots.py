"""Geometry evidence must distinguish gauge motion, scale and map deformation."""
import importlib.util
import csv
from pathlib import Path

import numpy as np


_SCRIPT = Path(__file__).resolve().parents[1] / "scripts/compare_vio_optimization_snapshots.py"
_SPEC = importlib.util.spec_from_file_location("optimization_snapshots", _SCRIPT)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def _points():
    return {index: point for index, point in enumerate(np.random.default_rng(731).normal(size=(70, 3)))}


def test_whole_map_translation_and_yaw_are_rigid_not_scale():
    before = _points()
    angle = 0.83
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    translation = np.array([-128, 153, -196])
    after = {key: rotation @ value + translation for key, value in before.items()}
    fit = _MODULE.fit_same_ids(before, after)
    assert fit["se3"]["rmse"] < 1e-12
    assert fit["pairwise"]["absolute_change_max"] < 1e-12
    assert abs(fit["sim3"]["scale"] - 1) < 1e-12
    np.testing.assert_allclose(fit["se3"]["translation"], translation)


def test_scale_change_is_visible_separately_from_rigid_fit():
    before = _points()
    after = {key: 1.3 * value + [3, 2, 1] for key, value in before.items()}
    fit = _MODULE.fit_same_ids(before, after)
    assert fit["se3"]["rmse"] > 0.3
    assert fit["sim3"]["rmse"] < 1e-12
    assert abs(fit["sim3"]["scale"] - 1.3) < 1e-12


def test_local_deformation_cannot_be_explained_by_rigid_or_similarity_fit():
    before = _points()
    after = {key: value.copy() for key, value in before.items()}
    after[4] += [4, -8, 3]
    fit = _MODULE.fit_same_ids(before, after)
    assert fit["se3"]["max_residual"] > 8
    assert fit["sim3"]["max_residual"] > 8
    assert fit["pairwise"]["absolute_change_max"] > 7


def test_missing_and_nonfinite_ids_do_not_create_fake_correspondences():
    before = {1: np.zeros(3), 2: np.ones(3), 3: np.array([np.nan, 2, 3])}
    after = {1: np.ones(3), 4: np.zeros(3), 3: np.ones(3)}
    assert _MODULE.fit_same_ids(before, after) == {"same_ids": 1, "status": "insufficient_points"}


def test_pose_orientations_constrain_rotation_when_positions_do_not_move(tmp_path):
    columns = ["id", "x", "y", "z"] + [f"r{i}{j}" for i in range(3) for j in range(3)]
    angle = 0.7
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    for name, position, value in (("before", [0, 0, 0], np.eye(3)), ("after", [128, -153, 196], rotation)):
        with (tmp_path / f"{name}.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(columns)
            writer.writerow([4, *position, *value.ravel()])
    fit = _MODULE.pose_alignment(tmp_path / "before.csv", tmp_path / "after.csv")
    assert fit["status"] == "ok"
    np.testing.assert_allclose(fit["rotation"], rotation, atol=1e-12)
    np.testing.assert_allclose(fit["translation"], [128, -153, 196])
    assert fit["position_residual_p50_p95_max"][-1] < 1e-12
