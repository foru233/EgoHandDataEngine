"""录制面板（RecordingPanel）离屏测试：目录默认值/浏览、状态切换、回放摘要与按钮启用条件。"""
from pathlib import Path

import yaml

from egohand.core.pipeline import ROOT
from egohand.gui import recording_panel as panel_module
from egohand.gui.recording_panel import RecordingPanel


def make_panel(qtbot):
    panel = RecordingPanel()
    qtbot.addWidget(panel)
    panel.show()
    return panel


def test_default_directory_uses_recording_root(qtbot):
    config = yaml.safe_load((ROOT / "configs/app.yaml").read_text(encoding="utf-8")) or {}
    expected = Path(config.get("recording_root") or "recordings")
    expected = expected if expected.is_absolute() else ROOT / expected
    panel = make_panel(qtbot)
    assert Path(panel.get_directory()) == expected
    assert panel.directory.isReadOnly()


def test_recording_root_falls_back_when_config_missing(tmp_path, monkeypatch):
    (tmp_path / "configs").mkdir()
    (tmp_path / "configs/app.yaml").write_text("theme: dark\n", encoding="utf-8")
    monkeypatch.setattr(panel_module, "ROOT", tmp_path)
    assert panel_module.default_recording_root() == tmp_path / "recordings"
    (tmp_path / "configs/app.yaml").write_text("recording_root: data/sessions\n", encoding="utf-8")
    assert panel_module.default_recording_root() == tmp_path / "data/sessions"


def test_browse_sets_directory(qtbot, monkeypatch, tmp_path):
    panel = make_panel(qtbot)
    chosen = tmp_path / "chosen"
    monkeypatch.setattr(
        panel_module.QFileDialog, "getExistingDirectory", lambda *args, **kwargs: str(chosen)
    )
    panel.browse_button.click()
    assert panel.get_directory() == str(chosen)
    # 浏览取消时保留原目录。
    monkeypatch.setattr(panel_module.QFileDialog, "getExistingDirectory", lambda *args, **kwargs: "")
    panel.browse_button.click()
    assert panel.get_directory() == str(chosen)


def test_state_transitions_buttons_and_progress(qtbot):
    panel = make_panel(qtbot)
    # 数据源未运行时不能录制。
    assert not panel.start_button.isEnabled()
    panel.set_available(True)
    assert panel.start_button.isEnabled()
    assert not panel.pause_button.isEnabled() and not panel.stop_button.isEnabled()
    assert "未录制" in panel.status.text()

    panel.set_state(False, False, True)  # 启动中
    assert "操作中" in panel.status.text() and not panel.start_button.isEnabled()

    panel.set_state(True, False, False)  # 录制中
    assert panel.status.text().splitlines()[0] == "录制中"
    assert panel.pause_button.isEnabled() and panel.stop_button.isEnabled()
    assert not panel.start_button.isEnabled() and not panel.browse_button.isEnabled()

    panel.set_progress({"frames": 120, "video_packets": 240, "left_packets": 120,
                        "right_packets": 120, "imu_packets": 9600, "elapsed_s": 65.0, "mb": 12.5})
    text = panel.status.text()
    for expected in ("帧 120", "视频包 240", "IMU 包 9600", "01:05", "12.5 MB"):
        assert expected in text

    panel.set_state(True, True, False)  # 已暂停
    assert "已暂停" in panel.status.text()
    assert panel.pause_button.text() == "继续"
    assert "帧 120" in panel.status.text()  # 暂停后保留最后计数

    panel.set_state(False, False, False)
    assert "未录制" in panel.status.text()
    assert panel.start_button.isEnabled()


def test_signals(qtbot, tmp_path):
    panel = make_panel(qtbot)
    panel.set_available(True)
    panel.set_directory(tmp_path)
    events = []
    panel.start_requested.connect(lambda: events.append("start"))
    panel.pause_requested.connect(lambda paused: events.append(("pause", paused)))
    panel.stop_requested.connect(lambda: events.append("stop"))
    panel.start_button.click()
    assert events == ["start"]
    panel.set_state(True, False, False)
    panel.pause_button.click()
    assert events[-1] == ("pause", True)
    panel.set_state(True, True, False)
    panel.pause_button.click()
    assert events[-1] == ("pause", False)
    panel.stop_button.click()
    assert events[-1] == "stop"
    panel.set_state(False, False, False)
    assert not panel.pause_button.isEnabled()


def test_replay_button_requires_ready_session(qtbot, tmp_path):
    panel = make_panel(qtbot)
    panel.set_available(True)
    session = tmp_path / "session"
    session.mkdir()
    panel.set_last_session(session, None)
    assert not panel.replay_button.isEnabled()
    assert "未提供回放校验信息" in panel.summary.text()

    panel.set_last_session(session, {"replay_ready": False, "reason": "索引中没有 sink4/5 双目记录",
                                     "frames": {"index_pairs": 0}, "imu": {"rows": 0}})
    assert not panel.replay_button.isEnabled()
    assert "不可回放" in panel.summary.text() and "双目" in panel.summary.text()

    panel.set_last_session(session, {"replay_ready": True, "reason": None,
                                     "frames": {"index_pairs": 1860},
                                     "imu": {"rows": 86400, "rate_hz": 479.7},
                                     "warnings": ["1 个双目包未配对"]})
    summary = panel.summary.text()
    assert panel.replay_ready and panel.replay_button.isEnabled()
    assert "可回放" in summary and "帧数 1860" in summary and "IMU 86400 条" in summary
    assert "479.7 Hz" in summary
    assert "已停止" in panel.status.text()

    # 录制中/操作中/回放运行中都不能提交。
    panel.set_state(True, False, False)
    assert not panel.replay_button.isEnabled()
    panel.set_state(False, False, True)
    assert not panel.replay_button.isEnabled()
    panel.set_state(False, False, False)
    panel.set_replay_running(True)
    assert not panel.replay_button.isEnabled()
    panel.set_replay_running(False)
    assert panel.replay_button.isEnabled()


def test_replay_request_uses_timing_defaults(qtbot, tmp_path):
    panel = make_panel(qtbot)
    panel.set_available(True)
    defaults = panel_module.load_timing_defaults()
    assert panel.alignment.currentData() == defaults["alignment"]
    assert abs(panel.lag.value() - defaults["lag_ms"]) < 1e-6

    session = tmp_path / "session"
    session.mkdir()
    panel.set_last_session(session, {"replay_ready": True, "frames": {"index_pairs": 10},
                                     "imu": {"rows": 480}})
    requests = []
    panel.replay_requested.connect(requests.append)
    panel.alignment.setCurrentIndex(panel.alignment.findData("timestamp"))
    panel.lag.setValue(-85.0)
    panel.stride.setValue(2)
    panel.replay_button.click()
    assert len(requests) == 1
    request = requests[0]
    assert request["recording"] == str(session)
    assert request["alignment"] == "timestamp"
    assert abs(request["lag_ms"] + 85.0) < 1e-6
    assert request["stride"] == 2
    # 提交后禁用，避免同一会话重复启动回放；主窗口结束回放后恢复。
    assert not panel.replay_button.isEnabled()
    panel.set_replay_running(False)
    assert panel.replay_button.isEnabled()


def test_narrow_inspector_layout_does_not_overflow(qtbot, tmp_path):
    """录制面板在右栏窄宽度（约 280 px）下不能出现横向截断：长路径只显示末段。"""
    panel = make_panel(qtbot)
    long_session = tmp_path / ("session_20261110_103800_" + "x" * 40)
    long_session.mkdir()
    panel.set_last_session(long_session, {
        "replay_ready": True,
        "frames": {"index_pairs": 12345},
        "imu": {"rows": 999999, "rate_hz": 479.7},
        "warnings": ["录制包含暂停/丢包缺口（1 处，最大 633 ms），回放可能在缺口处报 IMU 样本不足；VIO 回放建议连续录制"],
    })
    panel.resize(268, 760)
    panel.show()
    qtbot.wait(20)
    assert panel.minimumSizeHint().width() <= 280
    summary = panel.summary.text()
    assert str(long_session) not in summary          # 完整长路径不再进入可见文本
    status = panel.status.text()
    assert "\u200b" in status                        # 长路径带零宽断点，窄栏可换行
    assert "session_20261110_103800_" in status.replace("\u200b", "")  # 仍能看到会话名
    assert str(long_session) in panel.summary.toolTip()  # 完整路径在 tooltip
    assert "可回放" in summary
    assert "IMU 999999 条" in summary
    assert panel.replay_status.isHidden() is False or panel.replay_status.text() == ""


def test_replay_parameters_collapsible(qtbot):
    """回放参数默认收起，避免把录制面板撑高一屏；点击“参数”可展开。"""
    panel = make_panel(qtbot)
    assert panel.replay_options.isHidden()
    panel.parameters_toggle.setChecked(True)
    assert panel.replay_options.isVisible()
    panel.parameters_toggle.setChecked(False)
    assert panel.replay_options.isHidden()
