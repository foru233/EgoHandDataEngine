"""世界系变更审计：统计与大位移同 epoch 的整图重锚，输出 world_frame_changes.json。

Usage: python scripts/audit_world_frame_changes.py \
  --poses results/vio/debug_window/pair2_20261111_171149/stereo_inertial/poses.jsonl \
  --output results/vio/world_frame_audit_20261111

只读：不启动 SLAM、不改写位姿、不插值；阈值只用于筛选候选，不是验收结论。
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from egohand.vio.world_frame import (detect_world_frame_changes,  # noqa: E402
                                     load_pose_rows, summarize)

COLUMNS = (("lines", 12), ("timestamp_ns", 15), ("dt_s", 8), ("trans_m", 9),
           ("rot_deg", 9), ("epoch", 10), ("same_epoch", 12), ("kind", 20))


def print_table(events):
    """打印事件表；行号与 poses.jsonl 记录序号一致。"""
    print("".join(name.ljust(width) for name, width in COLUMNS))
    for event in events:
        cells = (f"{event['previous_line']}->{event['current_line']}",
                 str(event["timestamp_ns"]), f"{event['dt_s']:.3f}",
                 f"{event['translation_m']:.3f}", f"{event['rotation_deg']:.2f}",
                 f"{event['map_epoch_before']}->{event['map_epoch_after']}",
                 str(event["same_epoch"]), event["kind"])
        print("".join(text.ljust(width) for text, (_, width) in zip(cells, COLUMNS)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", type=Path, required=True, help="位姿日志 poses.jsonl")
    parser.add_argument("--output", type=Path, default=ROOT / "results/vio/world_frame_audit",
                        help="输出目录（默认 results/vio/world_frame_audit）")
    parser.add_argument("--jump-m", type=float, default=1.0, help="位移阈值（米，默认 1.0）")
    parser.add_argument("--max-gap-s", type=float, default=0.2, help="短间隔上限（秒，默认 0.2）")
    args = parser.parse_args(argv)
    if args.jump_m <= 0 or args.max_gap_s <= 0:
        parser.error("阈值必须为正")
    rows, malformed = load_pose_rows(args.poses)
    events = detect_world_frame_changes(rows, jump_m=args.jump_m, max_gap_s=args.max_gap_s)
    summary = summarize(events, jump_m=args.jump_m, max_gap_s=args.max_gap_s)
    report = {"schema_version": 1, "input": str(args.poses.resolve()),
              "sha256": hashlib.sha256(args.poses.read_bytes()).hexdigest(),
              "records": len(rows), "malformed_lines": malformed, **summary,
              "event_fields": ["previous_line", "current_line", "timestamp_ns", "dt_s",
                               "translation_m", "rotation_deg", "map_epoch_before",
                               "map_epoch_after", "same_epoch", "kind", "reason"],
              "events": events}
    args.output.mkdir(parents=True, exist_ok=True)
    dest = args.output / "world_frame_changes.json"
    dest.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf8")
    print(f"input {args.poses} ({len(rows)} records, {len(malformed)} malformed)")
    print(f"thresholds jump_m={args.jump_m} m, max_gap_s={args.max_gap_s} s")
    if events:
        print_table(events)
    else:
        print("no adjacent native-valid pose step exceeds jump_m")
    print(json.dumps({name: summary[name] for name in
                      ("event_count", "world_frame_change_count", "map_switch_count",
                       "epoch_unknown_count", "max_translation_m", "max_translation_lines")},
                     ensure_ascii=False))
    print(f"output {dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
