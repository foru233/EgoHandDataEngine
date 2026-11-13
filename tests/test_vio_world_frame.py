"""世界系变更检测：同 epoch 内整图重锚必须与地图切换、帧间抖动分开计数。"""
import importlib.util
from pathlib import Path

import numpy as np

from egohand.vio import world_frame as wf

spec = importlib.util.spec_from_file_location(
    "audit_vio_stability", Path(__file__).parents[1] / "scripts/audit_vio_stability.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def pose(t, x, epoch=1, valid=True, rotation_deg=0.0, jump=False):
    """构造一条原生位姿记录；rotation_deg 绕 z 轴旋转，epoch 为原始编码值。"""
    theta = np.radians(rotation_deg)
    m = np.eye(4)
    m[:3, :3] = [[np.cos(theta), -np.sin(theta), 0.0],
                 [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 1.0]]
    m[:3, 3] = [x, 0.0, 0.0]
    return {"timestamp_ns": int(t * 1e9), "raw_pose_valid": valid,
            "raw_matrix": m.tolist() if valid else None, "pose_valid": False,
            "map_epoch": epoch, "tracking_state": "OK" if valid else "LOST",
            "jump_detected": jump}


def test_same_epoch_large_step_is_world_frame_change():
    """① epoch 三要素不变而位移 27 m：整图重锚，不是帧间抖动。"""
    epoch = (0 << 40) | (0 << 20) | 9
    events = wf.detect_world_frame_changes([pose(0.0, 0.0, epoch), pose(0.033, 27.3, epoch, jump=True)])
    assert len(events) == 1
    event = events[0]
    assert event["kind"] == wf.EVENT_WORLD_FRAME_CHANGE
    assert (event["previous_line"], event["current_line"]) == (1, 2)
    assert event["same_epoch"] is True
    assert abs(event["translation_m"] - 27.3) < 1e-6
    assert event["map_epoch_before"] == event["map_epoch_after"] == epoch
    assert event["native_jump_detected"] is True
    assert "重锚" in event["reason"]


def test_big_change_only_is_recorded_revision():
    """② 只有 big_change_revision 递增：同一 map 内的已记录大修订，不是地图重建。

    依据：六个跳变案例里 frame_00610(3→6)、frame_01529(…12→…14) 的 same_epoch=False
    只是 big_change 递增，map_id=0、reset 未变。
    """
    before, after = (0 << 40) | (0 << 20) | 2, (0 << 40) | (0 << 20) | 3
    events = wf.detect_world_frame_changes([pose(0.0, 0.0, before), pose(0.033, 4.0, after)])
    assert [e["kind"] for e in events] == [wf.EVENT_RECORDED_REVISION]
    assert events[0]["same_epoch"] is False and events[0]["big_change_revision_after"] == 3
    assert events[0]["map_id_before"] == events[0]["map_id_after"] == 0
    assert "已记录" in events[0]["reason"]


def test_reset_or_map_change_is_map_switch():
    """②b map_id 或 reset_revision 变化才算地图/代切换。"""
    switched = wf.detect_world_frame_changes([pose(0.0, 0.0, 1 << 40), pose(0.033, 4.0, 0)])
    assert [e["kind"] for e in switched] == [wf.EVENT_MAP_SWITCH]
    # reset_revision 递增（同 map_id）
    a, b = (0 << 40) | (1 << 20) | 0, (0 << 40) | (2 << 20) | 0
    reset = wf.detect_world_frame_changes([pose(0.0, 0.0, a), pose(0.033, 4.0, b)])
    assert [e["kind"] for e in reset] == [wf.EVENT_MAP_SWITCH]


def test_small_step_produces_no_event():
    """③ 正常跟踪步长（厘米级）不产生任何事件。"""
    epoch = 5
    rows = [pose(i * 0.033, 0.017 * i, epoch) for i in range(20)]
    assert wf.detect_world_frame_changes(rows) == []
    assert wf.summarize([])["world_frame_change_count"] == 0


def test_invalid_frames_are_skipped_without_false_jump():
    """④ 无效帧被跳过：既不误判，也不跨缺测拼出假位移。"""
    epoch = 9
    rows = [pose(0.0, 0.0, epoch), pose(0.033, 30.0, epoch, valid=False),
            pose(0.066, 0.01, epoch), pose(0.099, 0.02, epoch)]
    assert wf.detect_world_frame_changes(rows) == []
    # 长间隔的大位移同样不判为世界系变更（间隔超过 max_gap_s）
    assert wf.detect_world_frame_changes([pose(0.0, 0.0, epoch), pose(4.0, 30.0, epoch)]) == []
    # epoch 缺失时无法断言同一世界系：单列为待查候选
    unknown = wf.detect_world_frame_changes([pose(0.0, 0.0, None), pose(0.033, 5.0, None)])
    assert [e["kind"] for e in unknown] == [wf.EVENT_EPOCH_UNKNOWN]


def test_counts_align_with_stability_audit_investigation_scope():
    """⑤ 与 audit_vio_stability.audit_rows 的跳变口径对齐，但不断言计数相等。

    对齐点：同阈值、短间隔、原生有效位姿对，两者的候选集合一致（本模块事件必落在
    audit 的 jump_cases 内）。差异：audit 的 jump_m 默认 0.5 m 且按同段/跨段聚合，
    只看"是否超阈值"；本模块默认 1.0 m 并额外要求 epoch 解码后才给出类别，因此
    audit 的跳变数 ≥ 本模块事件数，二者不是同一个量。
    """
    epoch = 9
    rows = [pose(0.0, 0.0, epoch), pose(0.033, 0.6, epoch), pose(0.066, 1.2, epoch),
            pose(0.099, 1.21, epoch), pose(0.132, 27.3, epoch)]
    report, _, _ = audit.audit_rows(rows, max_gap_s=0.2, jump_m=1.0)
    audit_lines = {(p["previous_line"], p["current_line"]) for p in report["jump_cases"]}
    events = wf.detect_world_frame_changes(rows, jump_m=1.0, max_gap_s=0.2)
    assert {(e["previous_line"], e["current_line"]) for e in events} <= audit_lines
    assert [e["kind"] for e in events] == [wf.EVENT_WORLD_FRAME_CHANGE] and len(audit_lines) == 1
    # 阈值不同即口径不同：audit 默认 0.5 m 会把 0.6 m 的步长也算作候选
    loose = audit.audit_rows(rows, max_gap_s=0.2, jump_m=0.5)[0]["jump_cases"]
    assert len(loose) > len(audit_lines)
    assert len(wf.detect_world_frame_changes(rows, jump_m=0.5, max_gap_s=0.2)) == len(loose)
