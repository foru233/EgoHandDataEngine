"""世界系整体重锚检测：在已录制位姿流上区分"同一 epoch 内整图搬移"与"地图/代切换"。

`map_epoch` 编码为 `map_id<<40 | reset_revision<<20 | big_change_revision`。
`FullInertialBA(bFixLocal=false)` 缺少世界平移与重力轴 yaw 约束时，一次求解即可
把整张图刚体搬移而**完全不改动该编码**（取证：`docs/VIO_27m跳变取证_20261111.md`，
75 个关键帧整体平移 22.82 m，位姿流上表现为相邻帧 27.3 m 位移）。因此
"epoch 变了才断轨迹"的判断会把这类段当成真实运动，需要单独标识。

本模块只读位姿流：不插值、不改写矩阵、不调整质量门控；阈值只用于筛选候选，
不构成验收结论。epoch 解码与原生位姿校验复用 `scripts/audit_vio_stability.py`。
"""
from __future__ import annotations

from collections import Counter
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

EVENT_WORLD_FRAME_CHANGE = "world_frame_change"
EVENT_MAP_SWITCH = "map_switch"
EVENT_RECORDED_REVISION = "recorded_revision"
EVENT_EPOCH_UNKNOWN = "epoch_unknown"
EVENT_NORMAL = "normal"


def _load_audit_module():
    """按文件路径加载 `scripts/audit_vio_stability.py`，复用其 epoch 解码与矩阵校验。

    该脚本不保证随包分发；加载失败时由本文件的同名函数按同一语义兜底。
    """
    path = Path(__file__).resolve().parents[3] / "scripts" / "audit_vio_stability.py"
    if not path.is_file():
        return None
    try:
        spec = importlib.util.spec_from_file_location("_egohand_audit_vio_stability", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:  # 退化到本地实现，审计本身仍可运行
        return None


_AUDIT = _load_audit_module()


def map_identity(epoch):
    """解码 epoch 三要素；None/非法值一律返回三个 None（与 audit 脚本一致）。"""
    if _AUDIT is not None:
        return _AUDIT.map_identity(epoch)
    # 来源 scripts/audit_vio_stability.map_identity，语义保持一致：本 checkout 的
    # System::GetMapEpoch 布局 map24 | reset20 | big_change20，未知保持 None。
    if epoch is None:
        return {"map_id": None, "reset_revision": None, "big_change_revision": None}
    try:
        value = int(epoch)
    except (TypeError, ValueError):
        return {"map_id": None, "reset_revision": None, "big_change_revision": None}
    return {"map_id": value >> 40, "reset_revision": (value >> 20) & ((1 << 20) - 1),
            "big_change_revision": value & ((1 << 20) - 1)}


def raw_matrix(row):
    """取原生位姿矩阵；门控后 matrix 可为 null，此时仍以 raw_matrix 为准。"""
    if _AUDIT is not None:
        return _AUDIT.raw_matrix(row)
    # 来源 scripts/audit_vio_stability.raw_matrix，语义保持一致。
    if not row.get("raw_pose_valid", row.get("pose_valid", False)):
        return None
    value = row.get("raw_matrix") if "raw_matrix" in row else row.get("matrix")
    try:
        a = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    return a if a.shape == (4, 4) and np.isfinite(a).all() else None


def classify_step(previous, current, *, jump_m=1.0, max_gap_s=0.2):
    """判定相邻两个原生有效位姿之间的世界系关系。

    `previous` / `current` 为 `(line, row, matrix)`，即 `detect_world_frame_changes`
    内部累积的同一结构，便于 GUI 或审计逐帧复用。返回几何量与分类，不改动输入。
    """
    previous_line, previous_row, previous_matrix = previous
    current_line, current_row, current_matrix = current
    dt = (int(current_row["timestamp_ns"]) - int(previous_row["timestamp_ns"])) * 1e-9
    delta = float(np.linalg.norm(current_matrix[:3, 3] - previous_matrix[:3, 3]))
    relative = previous_matrix[:3, :3].T @ current_matrix[:3, :3]
    angle = math.degrees(math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1))))
    before_epoch, after_epoch = previous_row.get("map_epoch"), current_row.get("map_epoch")
    before, after = map_identity(before_epoch), map_identity(after_epoch)
    # 三要素全部相等才算同一世界系；epoch 缺失/非法时不做同段断言。
    same_epoch = bool(before["map_id"] is not None and before == after)
    epoch_known = before["map_id"] is not None and after["map_id"] is not None
    short_gap = 0 < dt <= max_gap_s
    over_threshold = short_gap and delta > jump_m
    if not over_threshold:
        kind = EVENT_NORMAL
        reason = (f"位移 {delta:.3f} m 未超阈值 {jump_m:.3f} m 或间隔 {dt:.3f} s 不属于短间隔："
                  "普通相邻帧，不构成世界系变更")
    elif not epoch_known:
        kind = EVENT_EPOCH_UNKNOWN
        reason = (f"短间隔内位移 {delta:.3f} m > 阈值 {jump_m:.3f} m，但缺少可解码的 map_epoch："
                  "无法判断世界系是否切换，只能作为待查候选")
    elif same_epoch:
        kind = EVENT_WORLD_FRAME_CHANGE
        reason = (f"短间隔内位移 {delta:.3f} m > 阈值 {jump_m:.3f} m，而 map_id/reset_revision/"
                  f"big_change_revision 均未变（epoch {before_epoch} → {after_epoch}）："
                  "同一 epoch 编码内的整图重锚，不是帧间抖动或真实运动")
    elif (before["map_id"] == after["map_id"]
          and before["reset_revision"] == after["reset_revision"]):
        # 只有 big_change_revision 递增：同一个 map、同一次 reset 内的"大修订"（InformNewBigChange，
        # 通常是回环或全局 BA）。系统**自己记录了**这次修订，因此不属于"未记录的重锚"。
        # 取证依据：六个跳变案例中 frame_00610(3→6)、frame_01529(…12→…14) 的 same_epoch=False
        # 只是 big_change 递增，map_id=0、reset 未变，不是地图重建。
        kind = EVENT_RECORDED_REVISION
        reason = (f"短间隔内位移 {delta:.3f} m > 阈值 {jump_m:.3f} m，map_id/reset_revision 未变但 "
                  f"big_change_revision {before['big_change_revision']} → "
                  f"{after['big_change_revision']}：同一地图内的已记录大修订（回环/全局 BA），"
                  "不属于未记录的世界系重锚，但仍是同一世界系内的全局位移")
    else:
        kind = EVENT_MAP_SWITCH
        reason = (f"短间隔内位移 {delta:.3f} m > 阈值 {jump_m:.3f} m，且 map_id/reset_revision 变化"
                  f"（{before} → {after}）：地图/代切换，不属于同一世界系的连续轨迹")
    return {"previous_line": previous_line, "current_line": current_line,
            "timestamp_ns": int(current_row["timestamp_ns"]), "dt_s": dt,
            "translation_m": delta, "rotation_deg": angle,
            "map_epoch_before": before_epoch, "map_epoch_after": after_epoch,
            "map_id_before": before["map_id"], "map_id_after": after["map_id"],
            "reset_revision_before": before["reset_revision"],
            "reset_revision_after": after["reset_revision"],
            "big_change_revision_before": before["big_change_revision"],
            "big_change_revision_after": after["big_change_revision"],
            "same_epoch": same_epoch, "record_adjacent": current_line == previous_line + 1,
            "speed_mps": delta / dt if dt > 0 else None,
            "native_jump_detected": bool(current_row.get("jump_detected")),
            "quality_status": current_row.get("quality_status"),
            "kind": kind, "reason": reason}


def detect_world_frame_changes(rows, *, jump_m=1.0, max_gap_s=0.2):
    """返回位移超过 `jump_m` 的原生有效位姿事件，按 `current_line` 递增。

    行号与 `audit_vio_stability.audit_rows` 一致（1 起、按 jsonl 记录计）。无效帧
    （原生无效/矩阵缺失/含非有限值）被跳过，既不产生事件，也不会制造跨缺测的假跳变。
    """
    if jump_m <= 0 or max_gap_s <= 0:
        raise ValueError("阈值必须为正：jump_m>0、max_gap_s>0")
    events, previous = [], None
    for line, row in enumerate(rows, 1):
        if "timestamp_ns" not in row:  # 空行/畸形记录的占位
            continue
        matrix = raw_matrix(row)
        if matrix is None:
            continue
        current = (line, row, matrix)
        if previous is not None:
            event = classify_step(previous, current, jump_m=jump_m, max_gap_s=max_gap_s)
            if event["kind"] != EVENT_NORMAL:
                events.append(event)
        previous = current
    return events


def summarize(events, *, jump_m=1.0, max_gap_s=0.2):
    """按类别汇总事件；`normal` 不进入事件列表，因此只统计四类候选。"""
    counts = Counter(event["kind"] for event in events)
    largest = max(events, key=lambda e: e["translation_m"], default=None)
    kinds = (EVENT_WORLD_FRAME_CHANGE, EVENT_RECORDED_REVISION, EVENT_MAP_SWITCH, EVENT_EPOCH_UNKNOWN)
    return {"event_count": len(events),
            "world_frame_change_count": counts.get(EVENT_WORLD_FRAME_CHANGE, 0),
            "recorded_revision_count": counts.get(EVENT_RECORDED_REVISION, 0),
            "map_switch_count": counts.get(EVENT_MAP_SWITCH, 0),
            "epoch_unknown_count": counts.get(EVENT_EPOCH_UNKNOWN, 0),
            "counts": {kind: counts.get(kind, 0) for kind in kinds},
            "max_translation_m": largest["translation_m"] if largest else None,
            "max_translation_lines": [largest["previous_line"], largest["current_line"]] if largest else None,
            "thresholds": {"jump_m": jump_m, "max_gap_s": max_gap_s},
            "notes": ["阈值只用于筛选候选，不是验收结论。",
                      "epoch 编码不变而位移超阈值即判为整图重锚；仅凭位姿流无法区分"
                      "纯刚体重锚与重锚叠加地图点畸变。",
                      "同段/跨段的进一步统计仍以 audit_vio_stability.audit_rows 为准。"]}


def load_pose_rows(path):
    """读取 poses.jsonl；畸形行保留空占位以便行号与日志记录位置一致。"""
    rows, malformed = [], []
    for line_no, line in enumerate(Path(path).read_text(encoding="utf8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as exc:
            malformed.append({"line": line_no, "error": str(exc)})
            rows.append({})
    return rows, malformed
