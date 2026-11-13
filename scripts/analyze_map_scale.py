"""整图尺度指标：把"远点多"与"整图被放大"分开。

指标定义（都对整体平移不变，因此不会被整图重锚污染）：
- `kf_spread_p50/p95`：同阶段关键帧到其质心的距离分位（米）
- `pt_spread_p50/p95`：同阶段地图点到其质心的距离分位（米）
- `pt_depth_p50`：地图点到坐标原点的距离中位数（受整图平移影响，仅作参照）

同一事件内比较 `fullba_before` 与 `fullba_solved` 的 spread 变化，即可判断
"这一次求解把整图放大了多少"，而不是只看有多少点超过某个绝对阈值。

用法：
    python scripts/analyze_map_scale.py results/vio/depth_gate_multirun
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np


def read_points(path: Path):
    if not path.is_file():
        return None
    with path.open(encoding="utf8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    return np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])


def spread(xyz: np.ndarray):
    """到质心的距离分位；对整体平移不变。"""
    if xyz is None or len(xyz) < 2:
        return None
    centre = xyz.mean(axis=0)
    radius = np.linalg.norm(xyz - centre, axis=1)
    return {"n": int(len(xyz)),
            "p50": float(np.median(radius)),
            "p95": float(np.percentile(radius, 95)),
            "max": float(radius.max())}


def depth_from_origin(xyz: np.ndarray):
    if xyz is None or len(xyz) == 0:
        return None
    return float(np.median(np.linalg.norm(xyz, axis=1)))


def event_scale(diag: Path, event: str):
    out = {}
    for stage in ("fullba_before", "fullba_solved"):
        pts = read_points(diag / f"map0_{event}_{stage}_points.csv")
        kfs = read_points(diag / f"map0_{event}_{stage}_kf.csv")
        out[stage] = {"points": spread(pts), "keyframes": spread(kfs),
                      "point_depth_p50": depth_from_origin(pts)}
    before, solved = out.get("fullba_before"), out.get("fullba_solved")
    if before and solved and before.get("points") and solved.get("points"):
        ratio_src = solved["points"]["p50"] / max(before["points"]["p50"], 1e-9)
        out["growth"] = {"point_spread_ratio": float(ratio_src),
                         "point_p95_ratio": float(solved["points"]["p95"] /
                                                  max(before["points"]["p95"], 1e-9))}
    return out


def run_scale(run: Path):
    diag = run / "native_diagnostics"
    if not diag.is_dir():
        return None
    return {event: event_scale(diag, event) for event in ("event2", "event4", "event6")
            if (diag / f"map0_{event}_fullba_solved_points.csv").is_file()}


def main(argv):
    root = Path(argv[0])
    table = {}
    for run in sorted(p for p in root.iterdir() if p.is_dir()):
        scale = run_scale(run)
        if scale:
            table[run.name] = scale
    (root / "map_scale.json").write_text(json.dumps(table, ensure_ascii=False, indent=2),
                                         encoding="utf8")

    print("| 运行 | event | 点 spread p50 前→后 | 比值 | 点 spread p95 前→后 | 关键帧 spread p50 前→后 |")
    print("| --- | --- | --- | ---: | --- | --- |")
    for name, events in table.items():
        for event, data in events.items():
            before = (data.get("fullba_before") or {}).get("points")
            solved = (data.get("fullba_solved") or {}).get("points")
            kfb = (data.get("fullba_before") or {}).get("keyframes")
            kfs = (data.get("fullba_solved") or {}).get("keyframes")
            if not before or not solved:
                continue
            print("| {n} | {e} | {a:.2f} → {b:.2f} | {r:.2f}× | {a9:.2f} → {b9:.2f} | {ka} → {kb} |".format(
                n=name, e=event, a=before["p50"], b=solved["p50"],
                r=solved["p50"] / max(before["p50"], 1e-9),
                a9=before["p95"], b9=solved["p95"],
                ka=f"{kfb['p50']:.2f}" if kfb else "—",
                kb=f"{kfs['p50']:.2f}" if kfs else "—"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
