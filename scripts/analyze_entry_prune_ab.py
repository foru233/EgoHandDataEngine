"""入口剪枝 A/B 汇总：与 map_scale 指标联用。

用法：
    python scripts/analyze_entry_prune_ab.py results/vio/entry_prune_ab
要求先跑 analyze_map_scale.py 生成该目录下的 map_scale.json。
"""
from __future__ import annotations

import json
import sys
from itertools import combinations
from math import comb
from pathlib import Path

import numpy as np

ARM = ("off", "on")
JUMP_THRESHOLD = 50.0


def growth_by_arm(root: Path):
    scale = json.loads((root / "map_scale.json").read_text(encoding="utf8"))
    out = {a: [] for a in ARM}
    pruned = {a: [] for a in ARM}
    gated = {a: [] for a in ARM}
    for run in sorted(p for p in root.iterdir() if p.is_dir()):
        arm = next((a for a in ARM if run.name.startswith(a)), None)
        if arm is None:
            continue
        item = scale.get(run.name) or {}
        g = ((item.get("event6") or {}).get("growth") or {}).get("point_spread_ratio")
        if g is not None:
            out[arm].append(g)
        poses = run / "poses.jsonl"
        if poses.is_file():
            rows = [json.loads(x) for x in poses.read_text(encoding="utf8").splitlines() if x.strip()]
            counters = None
            for r in rows:
                if r.get("optimization_counters"):
                    counters = r["optimization_counters"]
            if counters:
                pruned[arm].append(counters.get("entry_pruned"))
            gated[arm].append(sum(1 for r in rows if r.get("pose_valid")))
    return out, pruned, gated


def fisher_one_sided(control_events, control_n, treated_events, treated_n):
    total = control_n + treated_n
    p = 0.0
    for x in range(0, treated_events + 1):
        if x <= control_events and (treated_n - x) <= (total - control_events):
            p += comb(control_events, x) * comb(total - control_events, treated_n - x) / comb(total, treated_n)
    return p


def perm_median_p(control, treated):
    if not control or not treated:
        return None
    allv = list(control) + list(treated)
    observed = float(np.median(treated))
    n = len(treated)
    extreme = total = 0
    for combo in combinations(range(len(allv)), n):
        if np.median([allv[i] for i in combo]) <= observed:
            extreme += 1
        total += 1
    return extreme / total


def main(argv):
    root = Path(argv[0])
    growth, pruned, gated = growth_by_arm(root)

    print("| 臂 | n | entry_pruned 中位 | 门控后中位 | event6 放大倍数 | 中位 | 范围 |")
    print("| --- | ---: | ---: | ---: | --- | ---: | --- |")
    for arm in ARM:
        g = growth[arm]
        if not g:
            print(f"| {arm} | 0 | — | — | — | — | — |")
            continue
        p = [x for x in pruned[arm] if x is not None]
        print("| {a} | {n} | {pr} | {ga} | {vals} | {med:.1f}× | {lo:.1f}–{hi:.1f} |".format(
            a=arm, n=len(g), pr=int(np.median(p)) if p else "—",
            ga=int(np.median(gated[arm])) if gated[arm] else "—",
            vals=" / ".join(f"{x:.0f}×" for x in g),
            med=float(np.median(g)), lo=min(g), hi=max(g)))

    off, on = growth["off"], growth["on"]
    if off and on:
        off_e = sum(1 for x in off if x > JUMP_THRESHOLD)
        on_e = sum(1 for x in on if x > JUMP_THRESHOLD)
        print()
        print(f">50× 二分类：off {off_e}/{len(off)} vs on {on_e}/{len(on)}"
              f"  Fisher 单侧 p = {fisher_one_sided(off_e, len(off), on_e, len(on)):.4f}")
        p = perm_median_p(off, on)
        if p is not None:
            print(f"中位数置换检验 p = {p:.4f}")
        print(f"on 是否全部低于 off 中位：{all(x < np.median(off) for x in on)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
