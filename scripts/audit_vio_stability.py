"""Read-only VIO log audit; missing observations never imply an instantaneous jump.

Usage: python scripts/audit_vio_stability.py --poses logs/*_poses.jsonl
  --worker logs/vio_vs030_worker_*.log --output results/vio/stability_baseline

Thresholds here select cases for investigation, not acceptance or trajectory filtering.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np


def distribution(values):
    a = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    if not len(a):
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None}
    return {"count": len(a), "min": float(a.min()), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def map_identity(epoch):
    """Decode the local System::GetMapEpoch layout, retain null/legacy unknowns."""
    if epoch is None:
        return {"map_id": None, "reset_revision": None, "big_change_revision": None}
    try:
        value = int(epoch)
    except (TypeError, ValueError):
        return {"map_id": None, "reset_revision": None, "big_change_revision": None}
    return {"map_id": value >> 40, "reset_revision": (value >> 20) & ((1 << 20) - 1),
            "big_change_revision": value & ((1 << 20) - 1)}


def raw_matrix(row):
    # A gated matrix may be null while raw_matrix still contains the estimate.
    if not row.get("raw_pose_valid", row.get("pose_valid", False)):
        return None
    value = row.get("raw_matrix") if "raw_matrix" in row else row.get("matrix")
    try:
        a = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    return a if a.shape == (4, 4) and np.isfinite(a).all() else None


def pose_pair(previous, current, max_gap_s, jump_m):
    pi, pr, pm = previous
    ci, cr, cm = current
    dt = (int(cr["timestamp_ns"]) - int(pr["timestamp_ns"])) * 1e-9
    delta = float(np.linalg.norm(cm[:3, 3] - pm[:3, 3]))
    relative = pm[:3, :3].T @ cm[:3, :3]
    angle = math.degrees(math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1))))
    ep, ec = pr.get("map_epoch"), cr.get("map_epoch")
    ip, ic = map_identity(ep), map_identity(ec)
    same_epoch = ep is not None and ec is not None and ep == ec
    same_map = ip["map_id"] is not None and ip["map_id"] == ic["map_id"]
    short_gap = 0 < dt <= max_gap_s
    return {"previous_line": pi, "current_line": ci, "dt_s": dt,
            "record_adjacent": ci == pi + 1, "short_gap": short_gap,
            "same_epoch": same_epoch, "same_map_id": same_map,
            "same_generation": same_map and ip["reset_revision"] == ic["reset_revision"],
            "previous_epoch": ep, "current_epoch": ec,
            "translation_m": delta, "rotation_deg": angle,
            "speed_mps": delta / dt if dt > 0 else None,
            "angular_speed_deg_s": angle / dt if dt > 0 else None,
            "investigate_jump": short_gap and delta > jump_m}


def summarize_pairs(pairs):
    return {"count": len(pairs), "translation_m": distribution(p["translation_m"] for p in pairs),
            "rotation_deg": distribution(p["rotation_deg"] for p in pairs),
            "speed_mps": distribution(p["speed_mps"] for p in pairs),
            "angular_speed_deg_s": distribution(p["angular_speed_deg_s"] for p in pairs),
            "dt_s": distribution(p["dt_s"] for p in pairs),
            "investigate_jump_count": sum(p["investigate_jump"] for p in pairs)}


def audit_rows(rows, *, max_gap_s=0.2, jump_m=0.5):
    valid_pairs, series, events = [], [], []
    previous_valid = None
    states, reasons = Counter(), Counter()
    raw_count = matrix_count = gated_count = 0
    timestamps, timing, resources, wall_times = [], {}, [], []
    for i, row in enumerate(rows, 1):
        if "timestamp_ns" not in row:
            continue
        stamp = int(row["timestamp_ns"])
        timestamps.append(stamp)
        frame_timing = row.get('timing') or {}
        if isinstance(frame_timing.get('track_ms'), (float, int)):
            timing.setdefault('track', []).append(frame_timing['track_ms'])
        if isinstance(frame_timing.get('elapsed_s'), (float, int)):
            wall_times.append(frame_timing['elapsed_s'])
        if row.get('resources'):
            resources.append({'elapsed_s':frame_timing.get('elapsed_s'), **row['resources']})
        states[str(row.get("tracking_state", "missing"))] += 1
        native_valid = bool(row.get("raw_pose_valid", row.get("pose_valid", False)))
        raw_count += native_valid
        gated_count += bool(row.get("pose_valid"))
        if not row.get("pose_valid"):
            reasons[str(row.get("quality_status", "missing"))] += 1
        m = raw_matrix(row)
        matrix_count += m is not None
        item = {"line": i, "timestamp_ns": stamp, **map_identity(row.get("map_epoch")),
                "map_epoch": row.get("map_epoch"), "tracking_state": row.get("tracking_state"),
                "frame_id": row.get("frame_id"), "map_change_index": row.get("map_change_index"),
                "snapshot_consistent": row.get("snapshot_consistent"),
                "raw_pose_valid": native_valid, "raw_matrix_available": m is not None,
                "pose_valid": bool(row.get("pose_valid")),
                "imu_initialized": row.get("imu_initialized"),
                "ba1_complete": row.get("ba1_complete"),
                "ba2_complete": row.get("ba2_complete", row.get("inertial_scale_finalized")),
                "quality_status": row.get("quality_status"),
                "jump_detected": bool(row.get("jump_detected")),
                "x": float(m[0, 3]) if m is not None else None,
                "y": float(m[1, 3]) if m is not None else None,
                "z": float(m[2, 3]) if m is not None else None}
        series.append(item)
        if len(series) > 1:
            for key in ("tracking_state", "imu_initialized", "ba1_complete", "ba2_complete",
                        "map_id", "reset_revision", "big_change_revision", "map_change_index"):
                before = series[-2].get(key)
                if item.get(key) != before:
                    events.append({"line": i, "timestamp_ns": stamp, "event": key,
                                   "before": before, "after": item.get(key)})
        if row.get("automatic_new_map"):
            events.append({"line": i, "timestamp_ns": stamp, "event": "automatic_new_map",
                           "reason": row.get("recovery_preaccept")})
        for key, value in ((row.get("sdk_timestamp_audit") or {}).get("timing_ms") or {}).items():
            if isinstance(value, (float, int)):
                timing.setdefault(key, []).append(value)
        if m is not None:
            current = (i, row, m)
            if previous_valid is not None:
                valid_pairs.append(pose_pair(previous_valid, current, max_gap_s, jump_m))
            previous_valid = current
    adjacent = [p for p in valid_pairs if p["record_adjacent"]]
    short = [p for p in valid_pairs if p["short_gap"]]
    same = [p for p in short if p["same_epoch"]]
    changed = [p for p in short if not p["same_epoch"] and p["previous_epoch"] is not None
               and p["current_epoch"] is not None]
    dt = np.diff(np.asarray(timestamps, dtype=np.int64)) * 1e-9
    duration = (timestamps[-1] - timestamps[0]) * 1e-9 if timestamps else 0
    report = {"records": len(rows), "pose_records": len(series), "raw_valid_count": raw_count,
              "raw_matrix_available_count": matrix_count, "gated_valid_count": gated_count,
              "raw_valid_rate": raw_count / len(series) if series else None,
              "gated_valid_rate": gated_count / len(series) if series else None,
              "duration_image_clock_s": duration,
              "processed_frames_per_image_second": (len(series) - 1) / duration if duration > 0 else None,
              "wall_throughput_fps": None, "frame_interval_s": distribution(dt),
              "non_monotonic_frame_intervals": int(np.sum(dt <= 0)),
              "long_frame_intervals": int(np.sum(dt > max_gap_s)),
              "tracking_states": dict(states), "gate_rejection_reasons": dict(reasons),
              "pairs": {"consecutive_records_with_raw_matrix": summarize_pairs(adjacent),
                        "consecutive_available_raw_poses_all_gaps": summarize_pairs(valid_pairs),
                        "consecutive_available_raw_poses_short_gap": summarize_pairs(short),
                        "same_exact_epoch_consecutive_records_short_gap": summarize_pairs([p for p in same if p["record_adjacent"]]),
                        "same_exact_epoch_short_gap": summarize_pairs(same),
                        "same_map_id_short_gap": summarize_pairs([p for p in short if p["same_map_id"]]),
                        "changed_epoch_short_gap": summarize_pairs(changed),
                        "same_generation_changed_epoch_short_gap": summarize_pairs([p for p in changed if p['same_generation']]),
                        "changed_map_or_generation_short_gap": summarize_pairs([p for p in changed if not p['same_generation']])},
              "jump_cases": [p for p in short if p["investigate_jump"]],
              "events": events, "timing_ms": {k: distribution(v) for k, v in timing.items()},
              "resource_samples":len(resources),
              "resources":{key:distribution(r.get(key) for r in resources)
                           for key in ('rss_bytes','vms_bytes','threads','cpu_seconds')},
              "thresholds": {"max_gap_s": max_gap_s, "investigation_translation_m": jump_m},
              "missing_metrics": ["BA1 per-frame state (legacy logs)", "independent motion ground truth",
                                  "IMU sample timestamps/counts actually passed per image interval",
                                  "worker exit code/stack unless present in supplied worker log",
                                  "RSS/commit memory/CPU/threads unless separately sampled"],
              "notes": ["Epoch decoder is specific to this checkout's System::GetMapEpoch layout.",
                        "A short-gap jump is an investigation candidate, not proof of an erroneous pose.",
                        "Image-clock rate is not processing wall throughput.",
                        "No gap interpolation, matrix alteration or quality threshold changes applied."]}
    if len(wall_times)>1 and wall_times[-1]>wall_times[0]:
        report['wall_throughput_fps']=(len(wall_times)-1)/(wall_times[-1]-wall_times[0])
        report['wall_throughput_scope']='response stream first-to-last observed elapsed_s; startup excluded'
    rss=[r for r in resources if isinstance(r.get('rss_bytes'),(int,float)) and isinstance(r.get('elapsed_s'),(int,float))]
    if len(rss)>1 and rss[-1]['elapsed_s']>rss[0]['elapsed_s']:
        report['rss_observed_trend']={
            'first_bytes':rss[0]['rss_bytes'],'last_bytes':rss[-1]['rss_bytes'],
            'delta_bytes':rss[-1]['rss_bytes']-rss[0]['rss_bytes'],
            'linear_slope_bytes_per_wall_second':float(np.polyfit([r['elapsed_s'] for r in rss],[r['rss_bytes'] for r in rss],1)[0]),
            'note':'A finite-run slope includes map growth and does not establish unbounded memory growth.'}
    return report, series, valid_pairs


WORKER_EVENTS = {
    "session": r"Live VIO session:", "relocalized": r"Relocalized!!",
    "ba1_start": r"start VIBA 1", "ba1_end": r"end VIBA 1",
    "ba2_start": r"start VIBA 2", "ba2_end": r"end VIBA 2",
    "ba2_incomplete": r"VIBA 2 incomplete", "new_map": r"New Map created",
    "recovery_timeout": r"Recovery timeout:", "reset_request": r"LM: (?:Active )?[Mm]ap reset recieved",
    "nan_failure": r"SO3::exp failed|SO3::exp.*NaN", "allocation_failure": r"bad_alloc|MemoryError|bad allocation",
    "fatal": r"Traceback \(most recent call last\)|terminate called|Assertion.*failed|Fatal Python error",
    "loop": r"\*Loop detected|\*Merge detected|Merge finished!"}


def audit_worker(path):
    lines = path.read_text(encoding="utf8", errors="replace").splitlines()
    events, session = [], None
    for number, line in enumerate(lines, 1):
        if "Live VIO session:" in line:
            session = line.split("Live VIO session:", 1)[1].strip()
        for name, pattern in WORKER_EVENTS.items():
            if re.search(pattern, line, flags=re.IGNORECASE):
                events.append({"line": number, "event": name, "session": session,
                               "timestamp_ns": None, "text": line,
                               "context": lines[max(0, number - 3):min(len(lines), number + 5)]})
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "event_counts": dict(Counter(e["event"] for e in events)), "events": events,
            "native_abnormal_exit_count": None,
            "note": "Legacy native logs have no frame timestamps or exit codes; event order is retained, not time-aligned or counted as proven crashes."}


def write_csv(path, rows):
    if not rows:
        path.write_text("", encoding="utf8")
        return
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, ensure_ascii=False) if isinstance(v, (dict, list)) else v
                             for k, v in row.items()})


def plot_series(path, series, pairs, events):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(15, 12), constrained_layout=True)
    t0 = series[0]["timestamp_ns"]
    times = [(r["timestamp_ns"] - t0) * 1e-9 for r in series]
    values = lambda key: [r.get(key) if r.get(key) is not None else np.nan for r in series]
    axes[0, 0].plot(values("x"), values("y"), linewidth=.6)
    axes[0, 0].set(xlabel="world X (m)", ylabel="world Y (m)", title="Raw XY (epoch transitions included)")
    for key in ("x", "y", "z"):
        axes[0, 1].plot(times, values(key), label=key, linewidth=.7)
    axes[0, 1].legend(); axes[0, 1].set(title="Raw position", ylabel="m")
    by_line = {r["line"]: (r["timestamp_ns"] - t0) * 1e-9 for r in series}
    for ax, key, title in ((axes[1, 0], "speed_mps", "Raw translation speed (m/s)"),
                           (axes[1, 1], "angular_speed_deg_s", "Raw angular speed (deg/s)")):
        usable = [p for p in pairs if p["short_gap"]]
        ax.plot([by_line[p["current_line"]] for p in usable], [p[key] for p in usable], linewidth=.7)
        ax.set_title(title)
        ax.set_yscale("symlog", linthresh=.1)
    for key in ("imu_initialized", "ba2_complete", "pose_valid"):
        axes[2, 0].step(times, values(key), where="post", label=key, linewidth=.7)
    axes[2, 0].legend(); axes[2, 0].set(title="Reported initialization / accepted output")
    for key in ("map_id", "reset_revision", "big_change_revision"):
        axes[2, 1].step(times, values(key), where="post", label=key)
    axes[2, 1].legend(); axes[2, 1].set(title="Decoded map identity / revisions")
    for ax in [axes[0, 1], *axes[1], *axes[2]]:
        for e in events:
            if e["event"] in ("ba2_complete", "map_id", "reset_revision", "big_change_revision"):
                ax.axvline((e["timestamp_ns"] - t0) * 1e-9, color="red", alpha=.15, linewidth=.7)
        ax.set_xlabel("image time since first result (s)"); ax.grid(alpha=.2)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", type=Path, nargs="+", required=True)
    parser.add_argument("--worker", type=Path, nargs="*", default=[])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-gap-s", type=float, default=.2)
    parser.add_argument("--jump-m", type=float, default=.5)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    if args.max_gap_s <= 0 or args.jump_m <= 0:
        parser.error("Audit thresholds must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    summaries = []
    for path in args.poses:
        rows, malformed = [], []
        for line_no, line in enumerate(path.read_text(encoding="utf8").splitlines(), 1):
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                malformed.append({"line": line_no, "error": str(exc)})
                # Keep a placeholder so adjacency refers to actual log line positions.
                rows.append({})
        report, series, pairs = audit_rows(rows, max_gap_s=args.max_gap_s, jump_m=args.jump_m)
        report.update(input_path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      malformed_lines=malformed)
        dest = args.output / path.stem
        dest.mkdir(exist_ok=True)
        write_csv(dest / "series.csv", series)
        write_csv(dest / "pairs.csv", pairs)
        write_csv(dest / "events.csv", report["events"])
        if not args.no_plots and series:
            try:
                plot_series(dest / "timeline.png", series, pairs, report["events"])
            except ImportError as exc:
                report["plot_error"] = str(exc)
        (dest / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf8")
        summaries.append({"input": str(path), "report": str(dest / "report.json"),
                          "records": report["records"], "raw_valid_count": report["raw_valid_count"],
                          "gated_valid_count": report["gated_valid_count"],
                          "same_epoch": report["pairs"]["same_exact_epoch_short_gap"],
                          "changed_epoch": report["pairs"]["changed_epoch_short_gap"]})
    workers = [audit_worker(p) for p in args.worker]
    result = {"schema_version": 1, "poses": summaries, "workers": workers}
    (args.output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf8")
    print(json.dumps({"output": str(args.output.resolve()), "pose_logs": len(summaries),
                      "worker_logs": len(workers)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
