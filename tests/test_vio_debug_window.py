"""调试窗口的回归测试：不依赖 OpenGL 与显示器。

覆盖：
- 二维回落视图能接收断点数据并重绘（不抛异常）
- 统计口径与 audit_vio_stability 一致（同 epoch >0.5 m 计数）
- 窗口在 --no-gl 模式下可用合成数据构建，并把位姿送进四个面板
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

PySide6 = pytest.importorskip("PySide6")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))


def _pose(stamp, x, epoch=1):
    m = np.eye(4)
    m[0, 3] = x
    return {"timestamp_ns": stamp, "pose_valid": True, "raw_pose_valid": True,
            "matrix": m.tolist(), "raw_matrix": m.tolist(), "map_epoch": epoch,
            "tracking_state": "OK", "quality_status": "ok"}


def _write_poses(tmp_path: Path, rows):
    run = tmp_path / "run"
    run.mkdir(parents=True, exist_ok=True)
    (run / "poses.jsonl").write_text("\n".join(json.dumps(r) for r in rows), encoding="utf8")
    return run


@pytest.fixture(scope="module")
def app():
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_series_breaks_at_gap_and_epoch_change(app):
    import vio_debug_window as win  # noqa: PLC0415 - 依赖 QApplication

    rows = [_pose(0, 0.0), _pose(33_333_333, 0.1), _pose(66_666_666, 0.2, epoch=2),
            _pose(100_000_000, 0.3, epoch=2)]
    points, breaks, _stamps, _epochs = win.series_for(rows, gated=False)
    assert len(points) == 4
    assert breaks == [2], "epoch 变化处必须断开"


def test_stats_match_audit_module(app):
    import vio_debug_window as win
    import audit_vio_stability as audit

    rows = [_pose(i * 33_333_333, i * 0.01) for i in range(10)]
    rows.append(_pose(9 * 33_333_333 + 33_333_333, 5.0))  # 4.9 m 跳变，同 epoch
    stats = win.stats_for(rows)
    report, _series, _pairs = audit.audit_rows(rows, max_gap_s=0.2, jump_m=0.5)
    assert stats["same_jumps"] == report["pairs"]["same_exact_epoch_short_gap"]["investigate_jump_count"]
    assert stats["same_jumps"] == 1
    assert stats["raw_valid"] == report["raw_valid_count"] == len(rows)


def test_window_loads_without_gl(app, tmp_path):
    import vio_debug_window as win

    rows = [_pose(i * 33_333_333, i * 0.02) for i in range(12)]
    run = _write_poses(tmp_path, rows)
    window = win.VioDebugWindow(str(run), str(run), auto=False)
    try:
        panel = window.panels["stereo_inertial_raw"]
        assert len(panel._points) == len(rows)
        assert panel.is_gl is not None
        assert "帧" in panel.stats.text()
    finally:
        window.close()


def test_plane_view_draws_offscreen(app):
    import vio_debug_window as win

    view = win.PlaneView((0.2, 0.87, 0.74, 1.0))
    view.resize(320, 240)
    pts = np.column_stack((np.linspace(0, 1, 20), np.sin(np.linspace(0, 3, 20)), np.zeros(20)))
    view.draw(pts, estimate_breaks=[10])
    image = view.grab()
    assert not image.isNull()
    assert image.width() == 320
