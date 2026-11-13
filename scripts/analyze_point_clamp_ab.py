"""止血 A/B 汇总：安全指标 + 主指标。

与 analyze_depth_gate_ab.py 的区别：这里额外统计"门控后有效位姿率"和 world_frame_revision，
用来确认止血没有把地图砍稀、也没有破坏重锚检测。

用法：python scripts/analyze_point_clamp_ab.py results/vio/point_clamp_multirun2
"""
from __future__ import annotations

import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

RUN = re.compile(r"^(off|on)_run(\d+)$")


def poses_summary(poses: Path):
    rows = [json.loads(line) for line in poses.read_text(encoding="utf8").splitlines() if line.strip()]
    counters = None
    for row in rows:
        if row.get("optimization_counters"):
            counters = row["optimization_counters"]
    raw = sum(1 for r in rows if r.get("raw_pose_valid"))
    gated = sum(1 for r in rows if r.get("pose_valid"))
    revision = max((r.get("world_frame_revision") or 0) for r in rows) if rows else 0
    return {"frames": len(rows), "raw_valid": raw, "gated_valid": gated,
            "gated_rate": gated / len(rows) if rows else None,
            "world_frame_revision": revision, "counters": counters}


def depth_summary(diag: Path, event="event6", stage="fullba_solved"):
    path = diag / f"map0_{event}_{stage}_points.csv"
    if not path.is_file():
        return None
    with path.open(encoding="utf8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    xyz = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    depth = np.linalg.norm(xyz, axis=1)
    return {"n": len(depth), "p50": float(np.median(depth)),
            "p95": float(np.percentile(depth, 95)), "max": float(depth.max()),
            "over_50m": int((depth > 50).sum()), "over_100m": int((depth > 100).sum())}


def main(argv):
    root = Path(argv[0])
    table = {}
    for run in sorted(p for p in root.iterdir() if p.is_dir() and RUN.match(p.name)):
        arm, index = RUN.match(run.name).groups()
        poses = run / "poses.jsonl"
        if not poses.is_file():
            continue
        table[run.name] = {"arm": arm, "index": int(index),
                           "poses": poses_summary(poses),
                           "event6": depth_summary(run / "native_diagnostics")}
    (root / "point_clamp_ab.json").write_text(json.dumps(table, ensure_ascii=False, indent=2), encoding="utf8")

    print("| 运行 | 臂 | 门控后有效 | 有效率 | 止血丢弃 | event6 点数 | >50m | >100m | p95 | 重锚 rev |")
    print("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for name, item in table.items():
        p, e = item["poses"], item["event6"] or {}
        c = p["counters"] or {}
        print("| {n} | {a} | {g} | {r} | {d} | {n6} | {o} | {o1} | {p95} | {rev} |".format(
            n=name, a=item["arm"], g=p["gated_valid"],
            r=f"{p['gated_rate']*100:.0f}%" if p["gated_rate"] else "—",
            d=c.get("point_clamp_rejected"), n6=e.get("n"), o=e.get("over_50m"),
            o1=e.get("over_100m"), p95=f"{e['p95']:.1f}" if e else "—",
            rev=p["world_frame_revision"]))

    print()
    for arm in ("off", "on"):
        items = [v for v in table.values() if v["arm"] == arm]
        if not items:
            continue
        rej = [v["poses"]["counters"]["point_clamp_rejected"] for v in items
               if v["poses"]["counters"] and v["poses"]["counters"].get("point_clamp_rejected") is not None]
        gated = [v["poses"]["gated_valid"] for v in items]
        far = [v["event6"]["over_50m"] for v in items if v["event6"]]
        p95 = [v["event6"]["p95"] for v in items if v["event6"]]
        print(f"{arm.upper()}: n={len(items)}"
              f" | 止血丢弃 {rej} (中位 {np.median(rej) if rej else '—'})"
              f" | 门控后有效 中位 {np.median(gated):.0f}"
              f" | event6 出现 {len(far)}/{len(items)}"
              + (f" | >50m 中位 {np.median(far):.0f} (范围 {min(far)}-{max(far)})" if far else "")
              + (f" | p95 中位 {np.median(p95):.1f}" if p95 else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
