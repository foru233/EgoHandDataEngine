"""录制会话面板：保存位置、录制/暂停/停止状态、回放校验摘要与离线 VIO 回放入口。

面板只负责展示与发信号，真正的写盘/回放由 MainWindow 调度（见设计 4.6）。目录默认取
`configs/app.yaml` 的 `recording_root`（相对 ROOT 解析），离线回放参数默认取
`configs/vio/live_timing.yaml`。

布局约束：面板位于右栏“录制”页签，宽度只有约 280–340 px。为避免横向挤压/截断，
- 长文本（会话路径、报告路径）只显示末段，完整路径放 tooltip；
- 可换行标签使用 Ignored 水平策略，不允许撑宽面板；
- 按钮用 2×2 网格，回放参数用“标签在上、控件在下”的两列网格。
"""
from pathlib import Path

import yaml
from PySide6.QtCore import Qt, Signal, QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QGroupBox, QHBoxLayout,
    QLineEdit, QSizePolicy, QSpinBox, QToolButton, QVBoxLayout, QWidget,
)
from ..core.pipeline import ROOT
from .components import label, button

DEFAULT_RECORDING_ROOT = "recordings"
TIMING_CONFIG = ROOT / "configs/vio/live_timing.yaml"


def default_recording_root():
    """configs/app.yaml 的 recording_root；相对路径按 ROOT 解析，缺省 ROOT/recordings。"""
    value = DEFAULT_RECORDING_ROOT
    try:
        config = yaml.safe_load((ROOT / "configs/app.yaml").read_text(encoding="utf-8")) or {}
        if isinstance(config, dict):
            value = str(config.get("recording_root") or DEFAULT_RECORDING_ROOT)
    except (OSError, yaml.YAMLError):
        value = DEFAULT_RECORDING_ROOT
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def load_timing_defaults():
    """离线回放参数的默认值；配置缺失时退回实时链路的保守默认。"""
    timing = {}
    try:
        data = yaml.safe_load(TIMING_CONFIG.read_text(encoding="utf-8")) or {}
        if isinstance(data, dict):
            timing = data
    except (OSError, yaml.YAMLError):
        timing = {}
    try:
        lag_ms = float(timing.get("camera_to_imu_lag_ms", 0.0))
    except (TypeError, ValueError):
        lag_ms = 0.0
    alignment = str(timing.get("imu_alignment", "lattice") or "lattice").strip().lower()
    if alignment not in ("lattice", "timestamp"):
        alignment = "lattice"
    return {"lag_ms": lag_ms, "alignment": alignment, "stride": 1}


def _first_int(value, keys):
    """从 int / float / dict 报告字段里取第一个计数值（兼容不同校验/回放报告）。"""
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return int(value)
    if isinstance(value, dict):
        for key in keys:
            item = value.get(key)
            if isinstance(item, bool):
                continue
            if isinstance(item, (int, float)):
                return int(item)
    return None


def _first_float(value, keys):
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, dict):
        for key in keys:
            item = value.get(key)
            if isinstance(item, bool):
                continue
            if isinstance(item, (int, float)):
                return float(item)
    return None


def _breakable(text):
    """给长路径/长下划线名字插入零宽断点，让 QLabel 在窄栏里可以换行而不是被截断。"""
    text = str(text)
    for character in ("/", "\\", "_", "-", "."):
        text = text.replace(character, character + "\u200b")
    return text


def _short_path(path):
    """窄面板只显示最后两级路径，完整路径放 tooltip；返回带零宽断点的可换行文本。"""
    try:
        path = Path(path)
        parts = [part for part in path.parts if part not in (path.anchor, "\\", "/")]
        if len(parts) >= 2:
            return _breakable(".../" + "/".join(parts[-2:]))
        return _breakable(path)
    except (TypeError, ValueError):
        return _breakable(path)


def _wrapping(widget):
    """允许在窄栏内换行/收缩，避免长文本把整块面板撑宽。"""
    widget.setWordWrap(True)
    widget.setMinimumWidth(0)
    widget.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
    widget.setTextInteractionFlags(Qt.TextSelectableByMouse)
    return widget


def _field(title, widget):
    """标签在上、控件在下的紧凑字段，用于“离线回放参数”。"""
    column = QVBoxLayout()
    column.setSpacing(2)
    column.addWidget(label(title, "hint"))
    widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    column.addWidget(widget)
    return column


class RecordingPanel(QGroupBox):
    start_requested = Signal()
    pause_requested = Signal(bool)   # True=暂停，False=继续
    stop_requested = Signal()
    replay_requested = Signal(dict)  # {"recording","lag_ms","alignment","stride"}

    def __init__(self, parent=None):
        super().__init__("录制会话", parent)
        self._available = False
        self._recording = False
        self._paused = False
        self._pending = False
        self._progress = {}
        self._last_session = None
        self._last_check = None
        self._replay_ready = False
        self._replay_running = False
        self._replay_output = None
        self._summary_text = "停止录制后显示回放校验摘要"

        self.setMinimumWidth(0)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 8, 0, 0)
        layout.setSpacing(6)

        layout.addWidget(label("保存位置", "muted"))
        directory_row = QHBoxLayout()
        directory_row.setSpacing(6)
        self.directory = QLineEdit(str(default_recording_root()))
        self.directory.setReadOnly(True)
        self.directory.setMinimumWidth(60)
        self.directory.setAccessibleName("录制保存位置")
        self.directory.setToolTip("每次录制在此目录下新建会话子目录；只读，点击“选择…”更改")
        self.browse_button = button("选择…", self.browse, "folder", "ghost", "选择录制保存目录")
        self.browse_button.setFixedWidth(62)
        directory_row.addWidget(self.directory, 1)
        directory_row.addWidget(self.browse_button)
        layout.addLayout(directory_row)

        self.status = _wrapping(label("未录制\n选择保存位置后点击开始录制", "mono"))
        layout.addWidget(self.status)

        # 一行三个短按钮 + tooltip：窄栏（约 240 px）下也不会被截断。
        buttons = QHBoxLayout()
        buttons.setSpacing(6)
        self.start_button = button("开始", self._request_start, "record", "primary", "开始录制视频 + IMU")
        self.pause_button = button("暂停", self._request_pause, tooltip="暂停 / 继续写入录制数据")
        self.stop_button = button("停止", self._request_stop, "stop", tooltip="停止录制并生成回放校验")
        for widget in (self.start_button, self.pause_button, self.stop_button):
            widget.setMinimumHeight(32)
            widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.pause_button)
        buttons.addWidget(self.stop_button)
        layout.addLayout(buttons)

        self.summary = _wrapping(label(self._summary_text, "hint"))
        layout.addWidget(self.summary)

        # 离线回放：主按钮 + “参数”折叠开关；参数默认收起，保证面板在右栏一屏内放得下。
        replay_row = QHBoxLayout()
        replay_row.setSpacing(6)
        self.replay_button = button(
            "离线 VIO 回放…", self._request_replay, "play",
            tooltip="在后台用录制的双目 H.265 + IMU 重跑确定性 VIO；不占用相机",
        )
        self.replay_button.setMinimumHeight(34)
        self.replay_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.parameters_toggle = QToolButton()
        self.parameters_toggle.setText("参数")
        self.parameters_toggle.setCheckable(True)
        self.parameters_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.parameters_toggle.setArrowType(Qt.RightArrow)
        self.parameters_toggle.setToolTip("展开/收起离线回放参数（对齐方式 / 偏移 / 步长）")
        self.parameters_toggle.setAccessibleName("离线回放参数")
        self.parameters_toggle.setCursor(Qt.PointingHandCursor)
        self.parameters_toggle.setMinimumHeight(34)
        self.parameters_toggle.toggled.connect(self._toggle_parameters)
        self.output_button = button("打开结果", self._open_output, "folder", "ghost")
        self.output_button.hide()
        replay_row.addWidget(self.replay_button, 1)
        replay_row.addWidget(self.parameters_toggle)
        replay_row.addWidget(self.output_button)
        layout.addLayout(replay_row)

        self.replay_options = QWidget()
        options = QGridLayout(self.replay_options)
        options.setContentsMargins(0, 2, 0, 0)
        options.setSpacing(6)
        self.alignment = QComboBox()
        self.alignment.addItem("帧格点", "lattice")
        self.alignment.addItem("时间戳", "timestamp")
        self.alignment.setAccessibleName("IMU 对齐方式")
        self.alignment.setMinimumWidth(70)
        self.lag = QDoubleSpinBox()
        self.lag.setRange(-1000.0, 1000.0)
        self.lag.setDecimals(1)
        self.lag.setSuffix(" ms")
        self.lag.setAccessibleName("相机—IMU 时间偏移")
        self.lag.setMinimumWidth(80)
        self.stride = QSpinBox()
        self.stride.setRange(1, 60)
        self.stride.setAccessibleName("回放步长")
        self.stride.setMinimumWidth(50)
        options.addLayout(_field("对齐", self.alignment), 0, 0)
        options.addLayout(_field("偏移", self.lag), 0, 1)
        options.addLayout(_field("步长", self.stride), 1, 0)
        options.setColumnStretch(0, 1)
        options.setColumnStretch(1, 1)
        self.replay_options.hide()
        layout.addWidget(self.replay_options)

        self.replay_status = _wrapping(label("", "hint"))
        self.replay_status.hide()
        layout.addWidget(self.replay_status)

        self._apply_timing_defaults()
        self._refresh()

    # ---------------------------------------------------------------- 对外 API
    def get_directory(self):
        return self.directory.text().strip()

    def set_directory(self, directory):
        text = str(directory or "").strip()
        if not text:
            return
        self.directory.setText(text)
        self.directory.setToolTip(text)

    def set_available(self, available):
        """数据源未运行时禁用录制操作；由 sync_controls 调用。"""
        self._available = bool(available)
        self._refresh()

    def set_state(self, recording, paused, pending):
        self._recording = bool(recording)
        self._paused = bool(paused) if self._recording else False
        self._pending = bool(pending)
        self._refresh()

    def set_progress(self, progress):
        self._progress = dict(progress or {})
        self._refresh()

    def set_last_session(self, path, check=None):
        """展示停止后的会话与回放校验摘要；check 缺失时明确说明不可回放。"""
        self._last_session = Path(path) if path else None
        self._last_check = dict(check) if isinstance(check, dict) else None
        self._replay_output = None
        self.output_button.hide()
        self.set_replay_status("")
        self._update_summary()
        self._refresh()

    def set_replay_running(self, running):
        self._replay_running = bool(running)
        self._refresh()

    def set_replay_status(self, text, tooltip=None):
        text = str(text or "")
        self.replay_status.setText(text)
        self.replay_status.setToolTip(str(tooltip) if tooltip else text)
        # 空闲时整块隐藏，把纵向空间留给保存位置/校验摘要；回放开始后再出现。
        self.replay_status.setVisible(bool(text))

    def set_replay_output(self, path):
        self._replay_output = Path(path) if path else None
        self.output_button.setVisible(self._replay_output is not None)
        if self._replay_output is not None:
            self.replay_status.setToolTip(str(self._replay_output))

    def replay_request(self):
        return {
            "recording": str(self._last_session) if self._last_session else "",
            "lag_ms": float(self.lag.value()),
            "alignment": self.alignment.currentData(),
            "stride": int(self.stride.value()),
        }

    @property
    def last_session(self):
        return self._last_session

    @property
    def replay_ready(self):
        return self._replay_ready

    @property
    def recording(self):
        return self._recording

    @property
    def paused(self):
        return self._paused

    @property
    def progress(self):
        return dict(self._progress)

    # ---------------------------------------------------------------- 内部
    def _toggle_parameters(self, checked):
        """折叠/展开回放参数；收起后面板在右栏一屏内放得下。"""
        self.replay_options.setVisible(bool(checked))
        self.parameters_toggle.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)
        self.parameters_toggle.setToolTip(
            "收起离线回放参数" if checked else "展开离线回放参数（对齐方式 / 偏移 / 步长）"
        )

    def _apply_timing_defaults(self):
        defaults = load_timing_defaults()
        index = self.alignment.findData(defaults["alignment"])
        self.alignment.setCurrentIndex(index if index >= 0 else 0)
        self.lag.setValue(max(self.lag.minimum(), min(self.lag.maximum(), defaults["lag_ms"])))
        self.stride.setValue(defaults["stride"])

    def browse(self):
        start = self.get_directory() or str(default_recording_root())
        directory = QFileDialog.getExistingDirectory(self, "选择录制保存目录", start)
        if directory:
            self.set_directory(directory)

    def _request_start(self):
        if not self._available or self._recording or self._pending:
            return
        if not self.get_directory():
            self.browse()
            if not self.get_directory():
                return
        self.start_requested.emit()

    def _request_pause(self):
        if not self._available or not self._recording or self._pending:
            return
        self.pause_requested.emit(not self._paused)

    def _request_stop(self):
        if not self._available or not self._recording or self._pending:
            return
        self.stop_requested.emit()

    def _request_replay(self):
        if not self._replay_ready or self._last_session is None:
            return
        if self._recording or self._pending or self._replay_running:
            return
        self._replay_running = True
        self.set_replay_status("离线 VIO 回放已提交，正在后台启动…")
        self._refresh()
        self.replay_requested.emit(self.replay_request())

    def _open_output(self):
        if self._replay_output is None:
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self._replay_output)))

    def _state_text(self):
        if self._pending:
            return "操作中…"
        if self._recording and self._paused:
            return "已暂停"
        if self._recording:
            return "录制中"
        if self._last_session is not None:
            return "已停止"
        return "未录制"

    def _format_elapsed(self, seconds):
        total = max(0, int(round(float(seconds))))
        return f"{total // 60:02d}:{total % 60:02d}"

    def _detail_text(self):
        if self._recording:
            progress = self._progress
            parts = []
            frames = progress.get("frames")
            if frames is not None:
                parts.append(f"帧 {int(frames)}")
            video = progress.get("video_packets")
            if video is not None:
                left = progress.get("left_packets")
                right = progress.get("right_packets")
                if left is not None and right is not None:
                    parts.append(f"视频包 {int(video)} (L{int(left)}/R{int(right)})")
                else:
                    parts.append(f"视频包 {int(video)}")
            imu = progress.get("imu_packets")
            if imu is not None:
                parts.append(f"IMU 包 {int(imu)}")
            elapsed = progress.get("elapsed_s")
            if elapsed is not None:
                parts.append("耗时 " + self._format_elapsed(elapsed))
            megabytes = progress.get("mb")
            if megabytes is not None:
                parts.append(f"写入 {float(megabytes):.1f} MB")
            return " · ".join(parts) if parts else "等待首批数据…"
        if self._last_session is not None:
            # 只显示末段路径，避免长路径在窄栏里占满整块面板；完整路径见 tooltip。
            return "会话 " + _short_path(self._last_session)
        return "选择保存位置后点击开始录制"

    def _update_summary(self):
        if self._last_session is None:
            self._replay_ready = False
            self._summary_text = "停止录制后显示回放校验摘要"
            self.summary.setToolTip("")
            return
        session = self._last_session
        check = self._last_check
        self.summary.setToolTip(str(session))
        if not isinstance(check, dict):
            self._replay_ready = False
            self._summary_text = "未提供回放校验信息（manifest.json.replay_check 缺失）\n会话 " + _short_path(session)
            return
        ready = bool(check.get("replay_ready"))
        self._replay_ready = ready
        reason = str(check.get("reason") or "").strip()
        lines = ["可回放 · 已通过回放校验" if ready else "不可回放" + ("：" + reason if reason else "")]
        frames = _first_int(check.get("frames"), ("index_pairs", "paired", "paired_frames", "decoded", "frames"))
        imu_rows = _first_int(check.get("imu"), ("rows", "count", "samples"))
        rate = _first_float(check.get("imu"), ("rate_hz",))
        counts = []
        if frames is not None:
            counts.append(f"帧数 {frames}")
        if imu_rows is not None:
            counts.append(f"IMU {imu_rows} 条")
        if rate is not None:
            counts.append(f"IMU 速率 {rate:.1f} Hz")
        if counts:
            lines.append(" · ".join(counts))
        warnings = check.get("warnings") or []
        if ready and warnings:
            # 面板只放一行短提示，完整告警进 tooltip；避免长告警把窄栏撑高。
            warning = str(warnings[0])
            short = "提示：录制含时间缺口，回放建议连续录制（详情见悬停）" if "缺口" in warning else warning[:28] + ("…" if len(warning) > 28 else "")
            lines.append(short)
            self.summary.setToolTip(str(session) + "\n" + "\n".join(str(item) for item in warnings))
        self._summary_text = "\n".join(lines)

    def _refresh(self):
        recording, pending = self._recording, self._pending
        self.browse_button.setEnabled(not recording and not pending)
        self.start_button.setEnabled(self._available and not recording and not pending)
        self.pause_button.setEnabled(self._available and recording and not pending)
        self.stop_button.setEnabled(self._available and recording and not pending)
        self.pause_button.setText("继续" if self._paused else "暂停")
        self.replay_options.setEnabled(not recording and not pending)
        self.parameters_toggle.setEnabled(not recording and not pending)
        self.replay_button.setEnabled(
            self._replay_ready and self._last_session is not None
            and not recording and not pending and not self._replay_running
        )
        self.status.setText(self._state_text() + "\n" + self._detail_text())
        self.summary.setText(self._summary_text)
