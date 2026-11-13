"""深度倍数止血的倍率扫描汇总：1.5 / 2.0 / 3.0。

用法：python scripts/analyze_clamp_ratio_scan.py results/vio/depthratio_scan results/vio/depthratio_ab
（第二个参数是 2.0 档所在目录，可省略）
"""
from __future__ import annotations

import json
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

PATTERN_TAGS = ("r1.5", "r2.0", "r3.0", "on", "off")


def tag_of(name: str) -> str:
    for tag in PATTERN_TAGS:
        if name.startswith(tag):
            return tag
    return "other"


def load(root: Path):
    out = {}
    scale_path = root / "map_scale.json"
    if scale_path.is_file():
        table = json.loads(scale_path.read_text(encoding="utf8"))
    else:
        # 目录下没有汇总时，逐运行调用 analyze_map_scale 的逻辑不在此重复实现；
        # 由调用方先跑 analyze_map_scale.py 生成 map_scale.json。
        table = {}
    for run in sorted(p for p in root.iterdir() if p.is_dir()):
        poses = run / "poses.jsonl"
        if not poses.is_file():
            continue
        rows = [json.loads(line) for line in poses.read_text(encoding="utf8").splitlines() if line.strip()]
        counters = None
        for row in rows:
            if row.get("optimization_counters"):
                counters = row["optimization_counters"]
        growth = ((table.get(run.name) or {}).get("event6") or {}).get("growth") or {}
        out[run.name] = {
            "tag": tag_of(run.name),
            "clamped": (counters or {}).get("point_clamp_rejected"),
            "gated": sum(1 for r in rows if r.get("pose_valid")),
            "raw": sum(1 for r in rows if r.get("raw_pose_valid")),
            "growth": growth.get("point_spread_ratio"),
        }
    return out


def permutation_p(control, treated):
    """单侧精确置换检验：treated 的中位是否显著低于 control。"""
    control = [v for v in control if v is not None]
    treated = [v for v in treated if v is not None]
    if not control or not treated:
        return None
    allv = control + treated
    n_treated = len(treated)
    observed = float(np.median(treated))
    extreme = total = 0
    for combo in combinations(range(len(allv)), n_treated):
        group = [allv[i] for i in combo]
        if np.median(group) <= observed:
            extreme += 1
        total += 1
    return extreme / total


def main(argv):
    roots = [Path(p) for p in argv]
    data = {}
    for root in roots:
        data.update(load(root))

    by_tag = {}
    for name, item in data.items():
        by_tag.setdefault(item["tag"], []).append(item)

    print("| 组 | n | 丢弃数中位 | 原生有效中位 | 门控后中位 | event6 放大倍数 | 中位 | 范围 |")
    print("| --- | ---: | ---: | ---: | ---: | --- | ---: | --- |")
    for tag in sorted(by_tag):
        items = by_tag[tag]
        growths = [i["growth"] for i in items if i["growth"] is not None]
        clamped = [i["clamped"] for i in items if i["clamped"] is not None]
        print("| {t} | {n} | {c} | {r} | {g} | {v} | {m} | {lo}–{hi} |".format(
            t=tag, n=len(items),
            c=int(np.median(clamped)) if clamped else "—",
            r=int(np.median([i["raw"] for i in items])),
            g=int(np.median([i["gated"] for i in items])),
            v=" / ".join(f"{x:.0f}×" for x in growths) if growths else "—",
            m=f"{np.median(growths):.1f}×" if growths else "—",
            lo=f"{min(growths):.0f}" if growths else "—",
            hi=f"{max(growths):.0f}" if growths else "—"))

    control = [i["growth"] for i in by_tag.get("off", [])]
    print()
    for tag in sorted(t for t in by_tag if t != "off"):
        p = permutation_p(control, [i["growth"] for i in by_tag[tag]])
        if p is not None:
            print(f"vs off: {tag} 单侧置换检验 p = {p:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
