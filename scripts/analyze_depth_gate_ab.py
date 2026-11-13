"""量化远点门限控：从 poses.jsonl 读 optimization_counters，从诊断 CSV 读点云深度分布。

用法：
    python scripts/analyze_depth_gate_ab.py results/vio/depth_gate_multirun
输出：<root>/depth_gate_ab.json + 控制台表格
"""
from __future__ import annotations

import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

RUN = re.compile(r"^(off|on)_run(\d+)$")


def counters(poses: Path):
    rows = [json.loads(line) for line in poses.read_text(encoding="utf8").splitlines() if line.strip()]
    seen = [r.get("optimization_counters") for r in rows if r.get("optimization_counters")]
    if not seen:
        return None
    last = seen[-1]
    return {"depth_gate_rejected": last.get("depth_gate_rejected"),
            "point_clamp_rejected": last.get("point_clamp_rejected"),
            "frames": len(rows)}


def depth_stats(diag: Path):
    """事件 2/4/6 的 fullba_solved 阶段点云深度分布（远点是主指标）。"""
    out = {}
    for event in ("event2", "event4", "event6"):
        path = diag / f"map0_{event}_fullba_solved_points.csv"
        if not path.is_file():
            continue
        with path.open(encoding="utf8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        xyz = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
        depth = np.linalg.norm(xyz, axis=1)
        out[event] = {"n": len(depth), "p50": float(np.median(depth)),
                      "p95": float(np.percentile(depth, 95)), "max": float(depth.max()),
                      "over_50m": int((depth > 50).sum())}
    return out


def main(argv):
    root = Path(argv[0])
    table = {}
    for run in sorted(p for p in root.iterdir() if p.is_dir() and RUN.match(p.name)):
        arm, index = RUN.match(run.name).groups()
        poses = run / "poses.jsonl"
        if not poses.is_file():
            continue
        table[run.name] = {"arm": arm, "index": int(index),
                           "counters": counters(poses),
                           "depth": depth_stats(run / "native_diagnostics")}

    (root / "depth_gate_ab.json").write_text(json.dumps(table, ensure_ascii=False, indent=2),
                                             encoding="utf8")

    print("| 运行 | 臂 | 帧 | 门限拒绝 | 止血丢弃 | event6 点数 | event6 深度p95 | >50m |")
    print("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for name, item in table.items():
        c = item["counters"] or {}
        e6 = item["depth"].get("event6") or {}
        print("| {n} | {a} | {f} | {d} | {p} | {n6} | {p95} | {o} |".format(
            n=name, a=item["arm"], f=c.get("frames"), d=c.get("depth_gate_rejected"),
            p=c.get("point_clamp_rejected"), n6=e6.get("n"),
            p95=f"{e6['p95']:.1f}" if e6 else "—", o=e6.get("over_50m")))

    print()
    for arm in ("off", "on"):
        items = [v for v in table.values() if v["arm"] == arm]
        if not items:
            continue
        rejected = [v["counters"]["depth_gate_rejected"] for v in items
                    if v["counters"] and v["counters"]["depth_gate_rejected"] is not None]
        e6 = [v["depth"]["event6"] for v in items if "event6" in v["depth"]]
        print(f"{arm.upper()}: n={len(items)}"
              f" | 门限拒绝 {rejected} (中位 {np.median(rejected) if rejected else '—'})"
              f" | event6 出现 {len(e6)}/{len(items)} 次"
              + (f" | event6 p95 中位 {np.median([x['p95'] for x in e6]):.1f} m" if e6 else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
