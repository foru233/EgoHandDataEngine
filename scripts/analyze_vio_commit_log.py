"""Describe paired native local-BA geometry and same-frame jump context.

Observation only: reference-pose transport is a counterfactual, never a recovered
pose, world-transform certificate, or input to the production quality gates.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from compare_vio_optimization_snapshots import fit_same_ids


def matrix(value):
    try:
        m = np.asarray(value, dtype=float).reshape(4, 4)
    except (ValueError, TypeError):
        return None
    if (not np.isfinite(m).all() or not np.allclose(m[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=2e-5)
            or not np.isclose(np.linalg.det(m[:3, :3]), 1, atol=2e-5)):
        return None
    return m


def motion(before, after):
    relative = before[:3, :3].T @ after[:3, :3]
    return {"translation_m": float(np.linalg.norm(after[:3, 3] - before[:3, 3])),
            "rotation_deg": float(np.degrees(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1))))}


def pose_fit(before, after):
    ids = sorted(before.keys() & after.keys())
    if not ids:
        return {"status": "no_finite_same_id_poses", "same_ids": 0}
    r0, r1 = np.array([before[k][:3, :3] for k in ids]), np.array([after[k][:3, :3] for k in ids])
    p0, p1 = np.array([before[k][:3, 3] for k in ids]), np.array([after[k][:3, 3] for k in ids])
    individual = r1 @ r0.transpose(0, 2, 1)
    u, _, vt = np.linalg.svd(individual.sum(axis=0))
    sign = np.eye(3); sign[-1, -1] = np.linalg.det(u @ vt)
    rotation = u @ sign @ vt
    translation = np.median(p1 - p0 @ rotation.T, axis=0)
    position_errors = np.linalg.norm(p1 - (p0 @ rotation.T + translation), axis=1)
    residual = rotation.T @ individual
    angles = np.degrees(np.arccos(np.clip((np.trace(residual, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    return {"status": "ok", "same_ids": len(ids), "same_keyframe_ids": ids,
            "rotation": rotation.tolist(), "translation": translation.tolist(),
            "translation_norm_m": float(np.linalg.norm(translation)),
            "rotation_deg": float(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1)))),
            "position_residual_p50_p95_max_m": np.percentile(position_errors, [50, 95, 100]).tolist(),
            "orientation_residual_p50_p95_max_deg": np.percentile(angles, [50, 95, 100]).tolist()}


def keyframes(record):
    result = {}
    for row in record.get("keyframes", []):
        m = matrix(row.get("world_from_camera"))
        if m is not None:
            if row["id"] in result:
                raise ValueError("Duplicate keyframe ID")
            result[row["id"]] = m
    return result


def points(record):
    result = {}
    for row in record.get("sampled_points", []):
        p = np.asarray(row.get("position"), dtype=float)
        if not row.get("bad") and p.shape == (3,) and np.isfinite(p).all():
            if row["id"] in result:
                raise ValueError("Duplicate point ID")
            result[row["id"]] = p
    return result


def compare_commit(before, after):
    k0, k1, p0, p1 = keyframes(before), keyframes(after), points(before), points(after)
    pose = pose_fit(k0, k1)
    result = {"event": before["event"], "kind": before["kind"], "map_id": before["map_id"],
              "before_sequence": before["sequence"], "after_sequence": after["sequence"],
              "before_change_index": before["map_change_index"], "after_change_index": after["map_change_index"],
              "before_revision": before["revision"], "after_revision": after["revision"],
              "keyframe_pose_fit": pose,
              "keyframe_position_fit": fit_same_ids({k: v[:3, 3] for k, v in k0.items()},
                                                      {k: v[:3, 3] for k, v in k1.items()}),
              "sampled_point_fit": fit_same_ids(p0, p1)}
    ids = sorted(p0.keys() & p1.keys())
    if ids and pose["status"] == "ok":
        a, b = np.array([p0[k] for k in ids]), np.array([p1[k] for k in ids])
        error = np.linalg.norm(b - (a @ np.array(pose["rotation"]).T + pose["translation"]), axis=1)
        result["sampled_points_under_keyframe_pose_fit"] = {
            "same_ids": len(ids), "residual_p50_p95_max_m": np.percentile(error, [50, 95, 100]).tolist()}
    return result


def analyze(records, poses=()):
    errors, pairs, pending, native_frames, bridges, rebases = [], [], {}, {}, {}, []
    for index, r in enumerate(records, 1):
        if r.get("schema_version") != 1 or r.get("sequence") != index:
            errors.append({"sequence": index, "reason": "schema_or_sequence_gap"})
        if r.get("type") == "local_ba":
            event = r["event"]
            if r["phase"] == "before" and event not in pending:
                pending[event] = r
            elif r["phase"] == "after" and event in pending:
                before = pending.pop(event)
                if (r["kind"] != before["kind"] or r["map_id"] != before["map_id"]
                        or r["map_change_index"] != before["map_change_index"] + 1):
                    errors.append({"event": event, "reason": "incoherent_local_commit_pair"})
                else:
                    pairs.append(compare_commit(before, r))
            else:
                errors.append({"event": event, "reason": "unpaired_or_duplicate_commit"})
        elif r.get("type") == "frame":
            key = (r["frame_id"], round(r["timestamp_sec"]*1e9), r["reset_generation"])
            if key in native_frames:
                errors.append({"frame_id": key[0], "reason": "duplicate_frame_identity"})
            native_frames[key] = r
        elif r.get("type") == "bridge_frame":
            key = (r["frame_id"], r["timestamp_ns"])
            if key in bridges:
                errors.append({"frame_id": key[0], "reason": "duplicate_bridge_identity"})
            if abs(r["epoch_ns"] + round(r["native_timestamp_sec"]*1e9) - r["timestamp_ns"]) > 1:
                errors.append({"frame_id": key[0], "reason": "native_input_clock_binding_mismatch"})
            bridges[key] = r
        elif r.get("type") == "last_frame_rebase":
            rebases.append(r)
    errors.extend({"event": event, "reason": "missing_commit_after"} for event in pending)
    jumps, joined, missing, getter_invalidated = [], 0, [], 0
    # Link by actual native identity, never by recording index or temporal proximity.
    previous = None
    for row in poses:
        key = (row.get("frame_id"), row["timestamp_ns"])
        bridge = bridges.get(key)
        if bridge is None:
            if row.get("snapshot_consistent"):
                missing.append(key)
            previous = None
            continue
        joined += 1
        fields = {"map_id": "map_id", "reset_generation": "reset_id", "revision": "map_revision",
                  "map_change_index": "map_change_index", "consistent": "snapshot_consistent"}
        if any(bridge[a] != row.get(b) for a, b in fields.items()):
            errors.append({"frame_id": key[0], "reason": "published_frame_identity_mismatch"})
        current = native_frames.get((key[0], round(bridge["native_timestamp_sec"]*1e9), bridge["reset_generation"]))
        if current is None:
            if bridge["consistent"]:
                errors.append({"frame_id": key[0], "reason": "consistent_bridge_missing_tracking_capture"})
            previous = None
            continue
        if any(current[k] != bridge[k] for k in ("map_id", "reset_generation", "revision", "map_change_index")):
            errors.append({"frame_id": key[0], "reason": "capture_bridge_identity_mismatch"})
        if current["consistent"] and not bridge["consistent"]:
            getter_invalidated += 1
        if not bridge["consistent"]:
            previous = None
            continue
        if row.get("jump_detected"):
            case = {"frame_id": key[0], "timestamp_ns": key[1], "source_frame_index": row.get("replay", {}).get("frame_index"),
                    "change_index": current["map_change_index"], "reference_kf_id": current["reference_kf_id"],
                    "production_translation_step_m": row.get("translation_step_m"),
                    "production_rotation_step_deg": row.get("rotation_step_deg"),
                    "coordinate_continuity_verified": False}
            if previous is not None:
                a, b = matrix(previous["world_from_camera"]), matrix(current["world_from_camera"])
                same_map = all(previous[k] == current[k] for k in ("map_id", "reset_generation", "atlas_generation"))
                case["same_map_lifetime"] = same_map
                if same_map and a is not None and b is not None and previous["pose_valid"] and current["pose_valid"]:
                    case["native_consecutive_pose_step"] = motion(a, b)
                    c0, c1 = matrix(previous.get("reference_world_from_camera")), matrix(current.get("reference_world_from_camera"))
                    if (previous["reference_kf_id"] is not None and previous["reference_kf_id"] == current["reference_kf_id"]
                            and c0 is not None and c1 is not None):
                        transported = c1 @ np.linalg.inv(c0) @ a
                        case["same_reference_counterfactual"] = {
                            "reference_pose_change": motion(c0, c1), "residual_pose_step": motion(transported, b),
                            "interpretation": "reference transport only; not a certified whole-map transform or accepted pose"}
                case["local_commits_between_frame_publications"] = [p["event"] for p in pairs
                    if previous["sequence"] < p["before_sequence"] < p["after_sequence"] < current["sequence"]
                    and p["map_id"] == current["map_id"]]
                case["counter_events_between_frame_publications"] = [r for r in records
                    if r.get("type") == "counter" and previous["sequence"] < r["sequence"] < current["sequence"]
                    and r["map_id"] == current["map_id"]]
                case["last_frame_rebases"] = [r for r in rebases
                    if previous["sequence"] < r["sequence"] < current["sequence"] and r["frame_id"] == current["frame_id"]]
            jumps.append(case)
        previous = current
    if missing:
        errors.append({"reason": "consistent_published_frames_missing_native_observation", "identities": missing})
    return {"schema_version": 1, "records": len(records), "record_types": dict(Counter(r.get("type") for r in records)),
            "integrity_errors": errors, "paired_local_commits": pairs,
            "published_frames_joined": joined, "getter_invalidated_captures": getter_invalidated, "jump_contexts": jumps,
            "limitations": ["Only successful local visual/inertial BA commits have paired geometry; other writers remain unclassified.",
                            "Only the first 256 point IDs are sampled; samples cannot certify all map geometry.",
                            "Logging changes scheduling and is not a real-time benchmark or deterministic replay proof.",
                            "Reference-pose transport is diagnostic only and never authorizes recovery or Odom.",
                            "Counters do not certify map object lifetime or complete writer synchronization."]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--poses", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.log.open(encoding="utf8")]
    poses = [json.loads(line) for line in args.poses.open(encoding="utf8")] if args.poses else []
    report = analyze(records, poses)
    with args.output.open("x", encoding="utf8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps({"records": report["records"], "commits": len(report["paired_local_commits"]),
                      "jumps": len(report["jump_contexts"]), "errors": report["integrity_errors"]}))
    if report["integrity_errors"]:
        raise SystemExit(1)
