"""世界系修订号核查：把 `poses.jsonl` 里的 `world_frame_revision` 与位姿跳变对齐。

只读：不改写位姿、不重跑 SLAM。用于回答两个问题：
1. 原生侧在哪一帧把世界系修订号 +1（`world_frame_revision` 首次出现的行）；
2. 这次递增与 `map_epoch` 是否变化、与该帧 `translation_step_m` / `jump_detected`
   是什么关系 —— 即“整图重锚”而不是“跟踪误差”。

Usage:
  python scripts/check_world_frame_revision.py \
    --poses results/vio/debug_window/<run>/stereo_inertial/poses.jsonl \
    [--output results/vio/<run>/world_frame_revision_check.json] [--jump-m 1.0]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from egohand.vio.world_frame import load_pose_rows, map_identity  # noqa: E402


def figure_rows(rows):
    """收集修订号变化点；行号与 poses.jsonl 记录序号一致（1 起）。"""
    increments, previous, last_value = [], None, None
    for line, row in enumerate(rows, 1):
        if "timestamp_ns" not in row:
            continue
        value = row.get("world_frame_revision")
        if value is None:
            last_value = None
            continue
        value = int(value)
        if previous is not None and value > previous:
            detail = row.get("world_frame") or {}
            increments.append({
                "line": line,
                "frame_index": (row.get("replay") or {}).get("frame_index"),
                "timestamp_ns": int(row["timestamp_ns"]),
                "revision_before": previous,
                "revision_after": value,
                "map_epoch": row.get("map_epoch"),
                "map_identity": map_identity(row.get("map_epoch")),
                "map_change_index": row.get("map_change_index"),
                "translation_step_m": row.get("translation_step_m"),
                "rotation_step_deg": row.get("rotation_step_deg"),
                "jump_detected": row.get("jump_detected"),
                "quality_status": row.get("quality_status"),
                "tracking_state": row.get("tracking_state"),
                "raw_pose_valid": row.get("raw_pose_valid", row.get("pose_valid")),
                "world_frame_detail": detail,
            })
        previous = value
        last_value = value
    return increments, last_value


def evaluation_states(rows):
    """每次原生全 BA 拟合只上报一次状态变化：这是“判定过 vs 没判定”的分界线。"""
    states, previous = [], None
    for line, row in enumerate(rows, 1):
        if "timestamp_ns" not in row:
            continue
        detail = row.get("world_frame")
        if not detail:
            continue
        signature = (detail.get("evaluations"), detail.get("verdict"),
                     round(float(detail.get("last_translation_m") or 0.0), 6),
                     detail.get("last_keyframes"))
        if signature == previous:
            continue
        previous = signature
        states.append({
            "line": line,
            "frame_index": (row.get("replay") or {}).get("frame_index"),
            "evaluations": detail.get("evaluations"),
            # 早期构建把“尚未评估”写成 verdict=reanchor（枚举默认值 0），这里统一归一，
            # 使旧日志与新日志的读法一致；判定结论本身不受影响。
            "verdict": detail.get("verdict") if detail.get("evaluations") else "not_evaluated",
            "keyframes": detail.get("last_keyframes"),
            "fitted_translation_m": detail.get("last_translation_m"),
            "fitted_rotation_deg": detail.get("last_rotation_deg"),
            "fitted_scale": detail.get("last_scale"),
            "residual_p50_m": detail.get("last_residual_p50_m"),
            "residual_p95_m": detail.get("last_residual_p95_m"),
            "residual_max_m": detail.get("last_residual_max_m"),
            "residual_limit_m": detail.get("last_residual_limit_m"),
            "rejected_nonrigid": detail.get("rejected_nonrigid"),
            "revision": row.get("world_frame_revision"),
            "map_epoch": row.get("map_epoch"),
            "translation_step_m": row.get("translation_step_m"),
            "rotation_step_deg": row.get("rotation_step_deg"),
            "jump_detected": row.get("jump_detected"),
            "quality_status": row.get("quality_status"),
            "tracking_state": row.get("tracking_state"),
        })
    return states


def summarize(rows, increments, last_value, jump_m):
    values = [int(row["world_frame_revision"]) for row in rows
              if "timestamp_ns" in row and row.get("world_frame_revision") is not None]
    supported = bool(values)
    epochs = [row.get("map_epoch") for row in rows
              if "timestamp_ns" in row and row.get("world_frame_revision") is not None]
    epoch_before_after = []
    for event in increments:
        previous_row = rows[event["line"] - 2] if event["line"] >= 2 else {}
        epoch_before_after.append({
            "line": event["line"],
            "map_epoch_before": previous_row.get("map_epoch"),
            "map_epoch_after": event["map_epoch"],
            "same_epoch": previous_row.get("map_epoch") == event["map_epoch"],
        })
    big_steps = [{"line": line, "translation_step_m": float(row.get("translation_step_m") or 0.0)}
                 for line, row in enumerate(rows, 1)
                 if "timestamp_ns" in row and (row.get("translation_step_m") or 0.0) > jump_m]
    return {
        "records": len(rows),
        "world_frame_revision_supported": supported,
        "world_frame_revision_final": last_value,
        "world_frame_revision_distinct": sorted(set(values)),
        "reanchor_count": len(increments),
        "map_epoch_distinct": sorted({int(epoch) for epoch in epochs if epoch is not None}),
        "increment_epoch_relations": epoch_before_after,
        "jump_detected_lines": [line for line, row in enumerate(rows, 1)
                                if "timestamp_ns" in row and row.get("jump_detected")],
        "steps_over_jump_m": big_steps,
        "jump_m": jump_m,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--jump-m", type=float, default=1.0,
                        help="统计 translation_step_m 超过该值的位置（默认 1.0 m）")
    args = parser.parse_args(argv)
    rows, malformed = load_pose_rows(args.poses)
    increments, last_value = figure_rows(rows)
    states = evaluation_states(rows)
    summary = summarize(rows, increments, last_value, args.jump_m)
    report = {"schema_version": 1, "input": str(args.poses.resolve()),
              "malformed_lines": malformed, **summary,
              "evaluations": states, "increments": increments}
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf8")
    print(f"input {args.poses} ({len(rows)} records)")
    if not summary["world_frame_revision_supported"]:
        print("world_frame_revision: NOT PRESENT (legacy DLL or non-bridge poses.jsonl)")
        if args.output:
            print(f"output {args.output}")
        return 2
    print(f"world_frame_revision final={summary['world_frame_revision_final']} "
          f"reanchors={summary['reanchor_count']} "
          f"map_epoch distinct={summary['map_epoch_distinct']}")
    header = ("line", "frame", "rev", "epoch", "same_epoch", "step_m", "rot_deg",
              "jump", "kmoved", "fit_m", "res_p95_m")
    print("".join(str(name).ljust(12) for name in header))
    for event, relation in zip(increments, summary["increment_epoch_relations"]):
        detail = event["world_frame_detail"] or {}
        cells = (event["line"], event["frame_index"],
                 f"{event['revision_before']}->{event['revision_after']}",
                 f"{relation['map_epoch_before']}->{relation['map_epoch_after']}",
                 relation["same_epoch"],
                 f"{event['translation_step_m']:.3f}"
                 if event["translation_step_m"] is not None else "-",
                 f"{event['rotation_step_deg']:.2f}"
                 if event["rotation_step_deg"] is not None else "-",
                 event["jump_detected"], detail.get("last_keyframes", "-"),
                 f"{detail.get('last_translation_m'):.3f}"
                 if detail.get("last_translation_m") is not None else "-",
                 f"{detail.get('last_residual_p95_m'):.4f}"
                 if detail.get("last_residual_p95_m") is not None else "-")
        print("".join(str(cell).ljust(12) for cell in cells))
    print("native evaluations (one row per changed state):")
    header = ("line", "frame", "evals", "verdict", "kf", "fit_m", "fit_deg", "scale",
              "res_p95", "res_lim", "step_m", "jump", "epoch", "rev")
    print("".join(str(name).ljust(17) for name in header))
    for state in states:
        cells = (state["line"], state["frame_index"], state["evaluations"], state["verdict"],
                 state["keyframes"],
                 f"{state['fitted_translation_m']:.3f}"
                 if state["fitted_translation_m"] is not None else "-",
                 f"{state['fitted_rotation_deg']:.2f}"
                 if state["fitted_rotation_deg"] is not None else "-",
                 f"{state['fitted_scale']:.3f}" if state["fitted_scale"] is not None else "-",
                 f"{state['residual_p95_m']:.4f}" if state["residual_p95_m"] is not None else "-",
                 f"{state['residual_limit_m']:.4f}"
                 if state["residual_limit_m"] is not None else "-",
                 f"{state['translation_step_m']:.3f}"
                 if state["translation_step_m"] is not None else "-",
                 state["jump_detected"], state["map_epoch"], state["revision"])
        print("".join(str(cell).ljust(17) for cell in cells))
    if summary["steps_over_jump_m"]:
        print("steps over jump_m: " + ", ".join(
            f"line {item['line']}={item['translation_step_m']:.3f} m"
            for item in summary["steps_over_jump_m"]))
    if args.output:
        print(f"output {args.output}")
    return 0 if summary["reanchor_count"] else 1


if __name__ == "__main__":
    sys.exit(main())
