"""Group map points by their FullInertialBA displacement and compare information-matrix spectra.

Reads only native diagnostic files written by VioDiagnostics::PointInformationMetrics and
GraphSnapshot (<prefix>_<stage>_point_condition.csv / _points.csv / _kf.csv). Nothing is
recomputed from the map and no native state is touched.

Displacement is reported twice:
  raw        |p_solved - p_before|
  non_rigid  the same difference after removing the rigid keyframe motion fitted between the
             fullba_before and fullba_solved keyframe snapshots (whole-map re-anchors are not
             the object of the point-question)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from compare_vio_optimization_snapshots import pose_alignment

GROUPS = (("<1m", 0.0, 1.0), ("1-100m", 1.0, 100.0), (">100m", 100.0, math.inf))


def read_condition(path: Path) -> dict[int, dict]:
    rows = {}
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            rows[int(row["id"])] = {
                "observations": int(float(row["observations"])),
                "edges": int(float(row["edges"])),
                "sigma_min": float(row["sigma_min"]),
                "sigma_max": float(row["sigma_max"]),
                "condition_number": float(row["condition_number"]),
                "trace": float(row["trace"]),
                "valid": int(float(row["valid"])),
            }
    return rows


def read_positions(path: Path) -> dict[int, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as stream:
        return {int(row["id"]): np.array([float(row[key]) for key in ("x", "y", "z")])
                for row in csv.DictReader(stream)}


def summary(values) -> dict:
    array = np.asarray([value for value in values if np.isfinite(value)], dtype=float)
    if not array.size:
        return {"n": 0}
    return {"n": int(array.size), "p50": float(np.percentile(array, 50)),
            "p95": float(np.percentile(array, 95)), "max": float(array.max())}


def rank(values: np.ndarray) -> np.ndarray:
    order = values.argsort()
    ranks = np.empty(len(values), dtype=float)
    ranks[order] = np.arange(len(values), dtype=float)
    # average ties so that constant inputs do not pretend to correlate
    for value in np.unique(values):
        mask = values == value
        if mask.sum() > 1:
            ranks[mask] = ranks[mask].mean()
    return ranks


def spearman(a: np.ndarray, b: np.ndarray):
    if len(a) < 3:
        return None
    ra, rb = rank(a) - len(a) / 2.0, rank(b) - len(b) / 2.0
    denominator = math.sqrt(float((ra ** 2).sum()) * float((rb ** 2).sum()))
    return float((ra * rb).sum() / denominator) if denominator else None


def groups_of(displacement: dict[int, float]) -> dict[str, list[int]]:
    result = {}
    for name, low, high in GROUPS:
        result[name] = [key for key, value in displacement.items() if low <= value < high]
    return result


def analyse_event(directory: Path, prefix: str) -> dict:
    stages = ("fullba_before", "fullba_solved")
    conditions = {}
    for stage in stages:
        path = directory / f"{prefix}_{stage}_point_condition.csv"
        if not path.is_file():
            return {"event": prefix, "status": "missing_stage", "missing": path.name}
        conditions[stage] = read_condition(path)
    before = read_positions(directory / f"{prefix}_fullba_before_points.csv")
    solved = read_positions(directory / f"{prefix}_fullba_solved_points.csv")
    shared = sorted(set(before) & set(solved) & set(conditions["fullba_before"]) & set(conditions["fullba_solved"]))
    if not shared:
        return {"event": prefix, "status": "no_shared_points"}
    raw = {key: float(np.linalg.norm(solved[key] - before[key])) for key in shared}
    alignment = pose_alignment(directory / f"{prefix}_fullba_before_kf.csv", directory / f"{prefix}_fullba_solved_kf.csv")
    non_rigid = None
    if alignment.get("status") == "ok":
        rotation, translation = np.array(alignment["rotation"]), np.array(alignment["translation"])
        non_rigid = {key: float(np.linalg.norm(solved[key] - (rotation @ before[key] + translation))) for key in shared}
    case = {"event": prefix, "status": "ok", "shared_points": len(shared),
            "keyframe_pose_alignment": {key: alignment.get(key) for key in
                                        ("status", "same_ids", "translation_norm", "rotation_degrees",
                                         "position_residual_p50_p95_max")},
            "groups": {}, "stage_metrics": {}, "top_displaced": []}
    for label, displacement in (("raw", raw), ("non_rigid", non_rigid)):
        if displacement is None:
            continue
        case["groups"][label] = {name: len(keys) for name, keys in groups_of(displacement).items()}
        case["stage_metrics"][label] = {}
        for stage in stages:
            rows = conditions[stage]
            case["stage_metrics"][label][stage] = {}
            for name, keys in groups_of(displacement).items():
                valid = [key for key in keys if rows[key]["valid"]]
                case["stage_metrics"][label][stage][name] = {
                    "points": len(keys), "valid": len(valid),
                    "condition_number": summary([rows[key]["condition_number"] for key in valid]),
                    "sigma_min": summary([rows[key]["sigma_min"] for key in valid]),
                    "sigma_max": summary([rows[key]["sigma_max"] for key in valid]),
                    "edges": summary([float(rows[key]["edges"]) for key in valid]),
                    "observations": summary([float(rows[key]["observations"]) for key in valid]),
                }
        truth = non_rigid if label == "non_rigid" else raw
        for stage in stages:
            values = np.array([conditions[stage][key]["condition_number"] for key in shared])
            finite = np.isfinite(values)
            if finite.sum() >= 3:
                case["stage_metrics"][label][stage]["spearman_condition_vs_log_displacement"] = spearman(
                    values[finite], np.log10(np.maximum(np.array([truth[key] for key in shared])[finite], 1e-6)))
    worst = sorted(shared, key=lambda key: raw[key], reverse=True)[:12]
    for key in worst:
        case["top_displaced"].append({
            "id": key, "raw_displacement_m": raw[key],
            "non_rigid_displacement_m": None if non_rigid is None else non_rigid[key],
            "observations": conditions["fullba_before"][key]["observations"],
            "edges_before": conditions["fullba_before"][key]["edges"],
            "edges_solved": conditions["fullba_solved"][key]["edges"],
            "sigma_min_before": conditions["fullba_before"][key]["sigma_min"],
            "sigma_min_solved": conditions["fullba_solved"][key]["sigma_min"],
            "condition_before": conditions["fullba_before"][key]["condition_number"],
            "condition_solved": conditions["fullba_solved"][key]["condition_number"],
        })
    case["discrimination"] = discrimination(shared, non_rigid or raw, conditions, before)
    return case


def auc(scores: np.ndarray, labels: np.ndarray):
    """Probability that a 'true' point scores higher than a 'false' one (rank/Mann-Whitney)."""
    positives = int(labels.sum())
    negatives = len(labels) - positives
    if not positives or not negatives:
        return None
    ranks = rank(scores) + 1.0
    return float((ranks[labels].sum() - positives * (positives + 1) / 2.0) / (positives * negatives))


def discrimination(shared, displacement, conditions, positions) -> dict:
    """Is the condition number separating the pushed points, or only their depth?"""
    keys = [key for key in shared
            if math.isfinite(displacement[key])
            and math.isfinite(conditions["fullba_before"][key]["condition_number"])
            and math.isfinite(conditions["fullba_solved"][key]["condition_number"])]
    if len(keys) < 30:
        return {"status": "too_few_points"}
    pushes = np.array([displacement[key] for key in keys])
    radius = np.array([float(np.linalg.norm(positions[key])) for key in keys])
    pushed = pushes > 100.0
    result = {"status": "ok", "points": len(keys), "over_100m": int(pushed.sum())}
    for label, stage in (("condition_before", "fullba_before"), ("condition_solved", "fullba_solved")):
        values = np.array([conditions[stage][key]["condition_number"] for key in keys])
        result["auc_" + label] = auc(values, pushed)
    for label in ("sigma_min", "sigma_max", "edges", "observations"):
        stage = "fullba_before"
        values = np.array([float(conditions[stage][key][label]) for key in keys])
        result["auc_" + label + "_before"] = auc(-values if label == "sigma_min" else values, pushed)
    result["auc_radius_before"] = auc(radius, pushed)
    result["spearman_log_condition_before_vs_log_radius"] = spearman(
        np.log(np.maximum(np.array([conditions["fullba_before"][key]["condition_number"] for key in keys]), 1e-30)),
        np.log(np.maximum(radius, 1e-9)))
    result["spearman_log_sigma_max_before_vs_log_radius"] = spearman(
        np.log(np.maximum(np.array([conditions["fullba_before"][key]["sigma_max"] for key in keys]), 1e-30)),
        np.log(np.maximum(radius, 1e-9)))
    for name, low, high in GROUPS:
        selected = (pushes >= low) & (pushes < high)
        if selected.sum():
            result["radius_before_" + name] = summary(radius[selected])
            result["displacement_" + name] = summary(pushes[selected])
    return result


def run(directory: Path) -> dict:
    reports = []
    for first in sorted(directory.glob("*_fullba_before_point_condition.csv")):
        prefix = first.name.removesuffix("_fullba_before_point_condition.csv")
        reports.append(analyse_event(directory, prefix))
    return {"schema_version": 1, "directory": str(directory.resolve()), "events": reports,
            "limitations": [
                "H = sum J^T Omega J uses the user-set information only; the Huber robust weight of a "
                "linearization is not applied.",
                "Only edges whose linearizeOplus() produced finite entries are summed; a point whose "
                "edges were all rejected reports valid=0.",
                "sigma_min/sigma_max depend on the Jacobian scale (image scale/depth), so condition "
                "numbers of points at very different depths are not directly comparable; the ratio the "
                "condition number encodes is scale-free.",
                "A large condition number is not by itself evidence of the solving step that moved a "
                "point; it only says the point's normal equations are ill-conditioned in one direction.",
            ]}


def sanitize(value):
    """Replace non-finite floats by None so the JSON stays strict."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: sanitize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize(item) for item in value]
    return value


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    report = sanitize(run(args.directory))
    target = args.directory / "point_condition_analysis.json"
    target.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(f"{len(report['events'])} events analysed, saved to {target}")
