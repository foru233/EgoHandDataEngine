"""Explain per-point geometric excursions using actual optimizer edge diagnostics.

Predicted disparity is computed from the optimizer state; it is NOT an observed
pixel disparity or an independent depth measurement. Inputs remain read-only.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path

import cv2
import numpy as np

from compare_vio_optimization_snapshots import pose_alignment


def read_rows(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return {int(row["id"]): row for row in csv.DictReader(stream)}


def position(row):
    return np.array([float(row[key]) for key in ("x", "y", "z")])


def rotation(row):
    return np.array([float(row[f"r{i}{j}"]) for i in range(3) for j in range(3)]).reshape(3, 3)


def edges(path, offset):
    result = defaultdict(list)
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if "EdgeMono" not in row["type"] and "EdgeStereo" not in row["type"]:
                continue
            vertices = [int(value) for value in row["vertices"].split(";") if value]
            result[vertices[0] - offset].append({"stereo": "EdgeStereo" in row["type"], "kf": vertices[1], "chi2": float(row["chi2"])})
    return result


def edge_residual_groups(path):
    groups = defaultdict(list)
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            groups[row["type"].split("::")[-1]].append(float(row["chi2"]))
    return {key: {"edges": len(value), "sum_chi2": float(np.sum(value)),
                  "p50_p95_max": np.percentile(value, [50, 95, 100]).tolist()}
            for key, value in groups.items()}


def analyze(directory, settings):
    storage = cv2.FileStorage(str(settings), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError(f"Cannot read settings {settings}")
    tbc = storage.getNode("IMU.T_b_c1").mat()
    bf = storage.getNode("Camera1.fx").real() * storage.getNode("Stereo.b").real()
    storage.release()
    if tbc is None or tbc.shape != (4, 4) or bf <= 0:
        raise ValueError("Expected rectified stereo IMU.T_b_c1, Camera1.fx and Stereo.b")
    cases = []
    for first in sorted(directory.glob("*_fullba_before_points.csv")):
        prefix = first.name.removesuffix("_fullba_before_points.csv")
        second = directory / f"{prefix}_fullba_solved_points.csv"
        if not second.is_file():
            continue
        before, after = read_rows(first), read_rows(second)
        kf0_path, kf1_path = (directory / f"{prefix}_{stage}_kf.csv" for stage in ("fullba_before", "fullba_solved"))
        kf0, kf1 = read_rows(kf0_path), read_rows(kf1_path)
        fit = pose_alignment(kf0_path, kf1_path)
        if fit["status"] != "ok":
            continue
        transform, shift = np.array(fit["rotation"]), np.array(fit["translation"])
        first_edges_path = directory / f"{prefix}_fullba_before_edges.csv"
        with first_edges_path.open(encoding="utf-8", newline="") as stream:
            visual_vertices = [int(row["vertices"].split(";")[0]) for row in csv.DictReader(stream) if "EdgeMono" in row["type"] or "EdgeStereo" in row["type"]]
        offset = min(visual_vertices) - min(before)
        edge0, edge1 = edges(first_edges_path, offset), edges(directory / f"{prefix}_fullba_solved_edges.csv", offset)
        if any(key not in before for key in edge0):
            raise ValueError(f"Point vertex offset could not be validated for {prefix}")

        def disparities(point, observations, keyframes):
            values = []
            negative_depths = 0
            for edge in observations:
                if not edge["stereo"] or edge["kf"] not in keyframes:
                    continue
                row = keyframes[edge["kf"]]
                rwb, twb = rotation(row), position(row)
                camera = tbc[:3, :3].T @ (rwb.T @ (point - twb) - tbc[:3, 3])
                if camera[2] <= 0:
                    negative_depths += 1
                else:
                    values.append(float(bf / camera[2]))
            return values, negative_depths

        points = []
        for key in sorted(before.keys() & after.keys()):
            a, b = position(before[key]), position(after[key])
            dp0, negative0 = disparities(a, edge0[key], kf0)
            dp1, negative1 = disparities(b, edge1[key], kf1)
            points.append({"id": key, "aligned_displacement_m": float(np.linalg.norm(b - (transform @ a + shift))),
                           "stereo_edges": sum(edge["stereo"] for edge in edge0[key]),
                           "mono_edges": sum(not edge["stereo"] for edge in edge0[key]),
                           "chi2_before": sum(edge["chi2"] for edge in edge0[key]),
                           "chi2_after": sum(edge["chi2"] for edge in edge1[key]),
                           "predicted_disparities_before_px": dp0, "predicted_disparities_after_px": dp1,
                           "negative_depth_before": negative0, "negative_depth_after": negative1,
                           "before_radius_m": float(np.linalg.norm(a)), "after_radius_m": float(np.linalg.norm(b))})
        groups = {}
        for name, selected in (("has_stereo", [point for point in points if point["stereo_edges"]]),
                               ("mono_only", [point for point in points if not point["stereo_edges"]])):
            if selected:
                residuals = np.array([point["aligned_displacement_m"] for point in selected])
                groups[name] = {"points": len(selected), "residual_p50_p95_max": np.percentile(residuals, [50, 95, 100]).tolist(),
                                "above_10m": int((residuals > 10).sum())}
        cases.append({"event": prefix, "vertex_offset": offset, "keyframe_pose_alignment": fit,
                      "edge_groups_before": edge_residual_groups(first_edges_path),
                      "edge_groups_after": edge_residual_groups(directory / f"{prefix}_fullba_solved_edges.csv"),
                      "groups": groups, "top_50_points": sorted(points, key=lambda point: point["aligned_displacement_m"], reverse=True)[:50]})
        destination = directory / f"{prefix}_point_diagnostics.json"
        destination.write_text(json.dumps(points, indent=2, allow_nan=False), encoding="utf-8")
    return {"schema_version": 1, "settings": str(settings.resolve()), "cases": cases,
            "limitations": ["Predicted disparities come from the before/after model, not measured feature coordinates.",
                            "Edge chi2 totals are unrobust, have differing dimensions and correlations, and do not validate noise units.",
                            "Low chi-square does not prove correspondence correctness or metric depth observability.",
                            "Large Euclidean changes in distant points need not produce large pixel reprojection changes."]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--settings", required=True, type=Path)
    args = parser.parse_args()
    report = analyze(args.directory, args.settings)
    target = args.directory / "point_analysis.json"
    target.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(f"{len(report['cases'])} point analyses saved to {target}")
