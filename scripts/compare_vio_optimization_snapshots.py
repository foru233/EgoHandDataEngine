"""Compare same-ID native optimization snapshots without treating origin motion as drift.

Usage: python scripts/compare_vio_optimization_snapshots.py DIAGNOSTIC_DIRECTORY
Outputs fit_report.json. No input or native state is changed.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_points(path: Path) -> dict[int, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as stream:
        return {
            int(row["id"]): np.array([float(row[key]) for key in ("x", "y", "z")])
            for row in csv.DictReader(stream)
        }


def pose_alignment(before_path: Path, after_path: Path) -> dict:
    """Use body orientations when positional motion has insufficient spatial spread."""
    def read(path):
        with path.open(encoding="utf-8", newline="") as stream:
            return {int(row["id"]): row for row in csv.DictReader(stream)}
    before, after = read(before_path), read(after_path)
    ids = sorted(before.keys() & after.keys())
    if not ids:
        return {"status": "no_correspondences"}
    rotation_columns = [f"r{i}{j}" for i in range(3) for j in range(3)]
    r0 = np.array([[float(before[key][col]) for col in rotation_columns] for key in ids]).reshape(-1, 3, 3)
    r1 = np.array([[float(after[key][col]) for col in rotation_columns] for key in ids]).reshape(-1, 3, 3)
    p0 = np.array([[float(before[key][col]) for col in ("x", "y", "z")] for key in ids])
    p1 = np.array([[float(after[key][col]) for col in ("x", "y", "z")] for key in ids])
    valid = np.isfinite(r0).all(axis=(1, 2)) & np.isfinite(r1).all(axis=(1, 2)) & np.isfinite(p0).all(axis=1) & np.isfinite(p1).all(axis=1)
    r0, r1, p0, p1 = r0[valid], r1[valid], p0[valid], p1[valid]
    if not len(p0):
        return {"status": "no_finite_correspondences"}
    u, _, vt = np.linalg.svd(np.einsum("nij,nkj->ik", r1, r0))
    sign = np.eye(3)
    sign[-1, -1] = np.linalg.det(u @ vt)
    rotation = u @ sign @ vt
    translation = np.median(p1 - (rotation @ p0.T).T, axis=0)
    error = np.linalg.norm(p1 - ((rotation @ p0.T).T + translation), axis=1)
    individual = np.einsum("nij,nkj->nik", r1, r0)
    residual_rotations = np.einsum("ij,njk->nik", rotation.T, individual)
    angles = np.degrees(np.arccos(np.clip((np.trace(residual_rotations, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    return {"status": "ok", "same_ids": int(valid.sum()), "rotation": rotation.tolist(), "translation": translation.tolist(),
            "translation_norm": float(np.linalg.norm(translation)),
            "rotation_degrees": float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))),
            "position_residual_p50_p95_max": np.percentile(error, [50, 95, 100]).tolist(),
            "orientation_residual_degrees_p50_p95_max": np.percentile(angles, [50, 95, 100]).tolist()}


def fit_same_ids(before: dict, after: dict) -> dict:
    ids = sorted(before.keys() & after.keys())
    ids = [key for key in ids if np.isfinite(before[key]).all() and np.isfinite(after[key]).all()]
    if len(ids) < 3:
        return {"same_ids": len(ids), "status": "insufficient_points"}
    source, target = np.array([before[key] for key in ids]), np.array([after[key] for key in ids])
    a, b = source.mean(axis=0), target.mean(axis=0)
    centered_a, centered_b = source - a, target - b
    u, singular, vt = np.linalg.svd(centered_b.T @ centered_a / len(ids))
    sign = np.eye(3)
    sign[-1, -1] = np.linalg.det(u @ vt)
    rotation = u @ sign @ vt
    variance = float(np.square(centered_a).sum() / len(ids))
    if variance < 1e-16:
        return {"same_ids": len(ids), "status": "degenerate_source"}
    scale = float(np.trace(np.diag(singular) @ sign) / variance)
    result = {"same_ids": len(ids), "status": "ok", "source_rank": int(np.linalg.matrix_rank(centered_a)),
              "cross_covariance_singular_values": singular.tolist()}
    for name, factor in (("se3", 1.0), ("sim3", scale)):
        translation = b - factor * rotation @ a
        residual = np.linalg.norm((factor * (rotation @ source.T)).T + translation - target, axis=1)
        result[name] = {"scale": factor, "rotation": rotation.tolist(), "translation": translation.tolist(),
                        "translation_norm": float(np.linalg.norm(translation)),
                        "rotation_degrees": float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))),
                        "rmse": float(np.sqrt(np.mean(residual ** 2))),
                        "residual_p95": float(np.percentile(residual, 95)),
                        "max_residual": float(residual.max())}
    # Pairwise changes independently detect geometry changes that an origin shift cannot explain.
    sample = np.linspace(0, len(ids) - 1, min(128, len(ids)), dtype=int)
    first, second = source[sample], target[sample]
    i, j = np.triu_indices(len(sample), 1)
    distance_before = np.linalg.norm(first[i] - first[j], axis=1)
    distance_after = np.linalg.norm(second[i] - second[j], axis=1)
    change = np.abs(distance_after - distance_before)
    valid = distance_before > 1e-6
    result["pairwise"] = {"deterministic_sample_ids": [ids[k] for k in sample],
                          "pairs": len(change), "absolute_change_p95": float(np.percentile(change, 95)),
                          "absolute_change_max": float(change.max()),
                          "distance_ratio_median": float(np.median(distance_after[valid] / distance_before[valid])) if valid.any() else None}
    return result


def compare_directory(directory: Path) -> dict:
    reports = []
    stages = (("fullba_before", "fullba_solved"), ("fullba_solved", "fullba_normalized"),
              ("initialize_before", "after_commit"))
    for before_stage, after_stage in stages:
        for before in sorted(directory.glob(f"*_{before_stage}_kf.csv")):
            prefix = before.name.removesuffix(f"_{before_stage}_kf.csv")
            after = directory / f"{prefix}_{after_stage}_kf.csv"
            if not after.is_file():
                continue
            case = {"event": prefix, "before": before_stage, "after": after_stage,
                    "keyframes": fit_same_ids(read_points(before), read_points(after)),
                    "keyframe_pose_alignment": pose_alignment(before, after)}
            points_before = directory / f"{prefix}_{before_stage}_points.csv"
            points_after = directory / f"{prefix}_{after_stage}_points.csv"
            if points_before.is_file() and points_after.is_file():
                first, second = read_points(points_before), read_points(points_after)
                case["map_points"] = fit_same_ids(first, second)
                pose_fit = case["keyframe_pose_alignment"]
                if pose_fit["status"] == "ok":
                    ids = [key for key in sorted(first.keys() & second.keys()) if np.isfinite(first[key]).all() and np.isfinite(second[key]).all()]
                    if ids:
                        a, b = np.array([first[key] for key in ids]), np.array([second[key] for key in ids])
                        rotation, translation = np.array(pose_fit["rotation"]), np.array(pose_fit["translation"])
                        residual = np.linalg.norm(b - ((rotation @ a.T).T + translation), axis=1)
                        case["map_points_under_keyframe_pose_alignment"] = {
                            "same_ids": len(ids), "residual_p50_p90_p95_p99_max": np.percentile(residual, [50, 90, 95, 99, 100]).tolist(),
                            "above_0_1_m": int((residual > 0.1).sum()), "above_1_m": int((residual > 1).sum()),
                            "before_radius_p50_p95_max": np.percentile(np.linalg.norm(a, axis=1), [50, 95, 100]).tolist(),
                            "after_radius_p50_p95_max": np.percentile(np.linalg.norm(b, axis=1), [50, 95, 100]).tolist()}
            for stage in (before_stage, after_stage):
                solver = directory / f"{prefix}_{stage}_solver.csv"
                if solver.is_file():
                    with solver.open(encoding="utf-8", newline="") as stream:
                        case[stage + "_solver"] = list(csv.DictReader(stream))
            reports.append(case)
    return {"schema_version": 1, "directory": str(directory.resolve()), "comparisons": reports,
            "limitations": ["Low SE3 residual supports a whole-map rigid change; it does not prove absolute accuracy.",
                            "Collinear/small keyframe sets cannot independently constrain rotation.",
                            "Least-squares point-cloud fits are sensitive to outliers; pose-informed residuals are reported separately.",
                            "Point snapshots include only graph vertices retained by the optimizer."]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    report = compare_directory(args.directory)
    destination = args.directory / "fit_report.json"
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(f"{len(report['comparisons'])} comparisons saved to {destination}")
