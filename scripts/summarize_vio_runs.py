"""汇总多组实验的稳定性指标（与 docs/VIO_ACCEPTANCE_REPORT.md 同一口径）。

用法：
    python scripts/summarize_vio_runs.py results/vio/gauge_constraint_ab_20261111/off_run1 ...
输出：Markdown 表格 + JSON。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import audit_vio_stability as audit  # noqa: E402 - 复用验收同一口径

JUMP_M = 0.5
MAX_GAP_S = 0.2


def summarize(run_dir: Path):
    poses = run_dir / "poses.jsonl"
    rows = [json.loads(line) for line in poses.read_text(encoding="utf8").splitlines() if line.strip()]
    report, _series, _pairs = audit.audit_rows(rows, max_gap_s=MAX_GAP_S, jump_m=JUMP_M)
    pairs = report.get("pairs", {})
    same = pairs.get("same_exact_epoch_short_gap", {})
    changed = pairs.get("changed_epoch_short_gap", {})

    # 整图重锚线索：jump_detected 且步长 > 1 m
    big = [(i, float(r.get("translation_step_m") or 0)) for i, r in enumerate(rows, 1)
           if r.get("jump_detected") and float(r.get("translation_step_m") or 0) > 1.0]

    inputs = run_dir / "inputs.json"
    provenance = {}
    if inputs.is_file():
        data = json.loads(inputs.read_text(encoding="utf8"))
        provenance = {"sensor": data.get("sensor"), "gauge_constraint": data.get("gauge_constraint"),
                      "gauge_fix": data.get("gauge_fix")}

    return {
        "run": run_dir.name,
        **provenance,
        "frames": report.get("pose_records"),
        "raw_valid": report.get("raw_valid_count"),
        "gated_valid": report.get("gated_valid_count"),
        "same_jumps": same.get("investigate_jump_count"),
        "same_max_m": (same.get("translation_m") or {}).get("max"),
        "same_p95_m": (same.get("translation_m") or {}).get("p95"),
        "changed_jumps": changed.get("investigate_jump_count"),
        "changed_max_m": (changed.get("translation_m") or {}).get("max"),
        "big_jumps": big,
        "tracking_states": report.get("tracking_states"),
    }


def main(argv):
    rows = [summarize(Path(p)) for p in argv]
    header = ("| 组 | 传感器 | 规范约束 | 帧 | 原生有效 | 门控后 | 同段>0.5m | 同段最大 [m] "
              "| 跨段最大 [m] | >1m 重锚 |")
    print(header)
    print("| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for r in rows:
        print("| {run} | {sensor} | {gauge_constraint} | {frames} | {raw_valid} | {gated_valid} "
              "| {same_jumps} | {same_max} | {changed_max} | {big} |".format(
                  run=r["run"], sensor=r.get("sensor"), gauge_constraint=r.get("gauge_constraint"),
                  frames=r["frames"], raw_valid=r["raw_valid"], gated_valid=r["gated_valid"],
                  same_jumps=r["same_jumps"],
                  same_max="—" if r["same_max_m"] is None else f"{r['same_max_m']:.3f}",
                  changed_max="—" if r["changed_max_m"] is None else f"{r['changed_max_m']:.3f}",
                  big=len(r["big_jumps"])))
    out = Path(argv[0]).resolve().parent / "summary.json"
    out.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf8")
    print(f"\nJSON: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
