"""VIO 对比调试窗口：纯视觉 / 视觉+IMU × 门控前(原生) / 门控后(GUI 展示) 四联视图。

复用现有实现，不重写几何/门控/回放：
- 三维视图：`egohand.gui.vio_panel.TrajectoryView`（与 GUI 主界面同一控件、同一配色）
- 统计口径：`scripts/audit_vio_stability.py` 的 `audit_rows`（0.5 m / 0.2 s 阈值与验收报告一致）
- 重新跑数据：`scripts/run_vio_stability_experiment.py`（--stereo-only 切换纯视觉）

用法：
    .\\.venv\\Scripts\\python.exe scripts\\vio_debug_window.py --auto
    .\\.venv\\Scripts\\python.exe scripts\\vio_debug_window.py ^
        --stereo-only results/vio/stability_baseline/snapshot_matrix/stereo_only ^
        --stereo-inertial results/vio/stability_baseline/snapshot_matrix/snapshot_default
    .\\.venv\\Scripts\\python.exe scripts\\vio_debug_window.py --auto ^
        --recording data/imu/session_20261111_105436_609132400 ^
        --library src/egohand/vio/native/bin/egohand_vio_recovery_bridge_v9.dll --run

四张图分别对应：
    左列 门控前（原生 ORB 输出，未经 PoseQualityGate）
    右列 门控后（GUI 实际展示的位姿）
    上行 纯视觉（Stereo，无 IMU 约束）
    下行 视觉+IMU（Stereo-Inertial）
"""
from __future__ import annotations

import sys
import json
import subprocess
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QApplication, QCheckBox, QFileDialog, QGridLayout, QHBoxLayout,
                               QLabel, QMainWindow, QPushButton, QScrollArea, QSizePolicy,
                               QVBoxLayout, QWidget)

import audit_vio_stability as audit  # 复用验收同一套统计口径

JUMP_M = 0.5
MAX_GAP_S = 0.2


# --------------------------------------------------------------------------- 数据

def load_rows(run_dir: Path):
    poses = Path(run_dir) / "poses.jsonl"
    if poses.is_file():
        rows = [json.loads(line) for line in poses.read_text(encoding="utf8").splitlines() if line.strip()]
        return rows, poses
    if Path(run_dir).is_file() and Path(run_dir).name.endswith(".jsonl"):
        rows = [json.loads(line) for line in Path(run_dir).read_text(encoding="utf8").splitlines() if line.strip()]
        return rows, Path(run_dir)
    raise FileNotFoundError(f"找不到 poses.jsonl：{run_dir}")


def find_newest_run(pattern: str, *, require_stereo_only: bool | None = None):
    """在 results/vio 下按修改时间找最新的一组实验输出。"""
    candidates = []
    for path in (ROOT / "results" / "vio").rglob("poses.jsonl"):
        run_dir = path.parent
        if pattern and pattern not in str(run_dir).replace("\\", "/"):
            continue
        inputs = run_dir / "inputs.json"
        if require_stereo_only is not None and inputs.is_file():
            try:
                sensor = json.loads(inputs.read_text(encoding="utf8")).get("sensor")
            except (OSError, json.JSONDecodeError):
                sensor = None
            is_stereo_only = sensor == "stereo"
            if is_stereo_only != require_stereo_only:
                continue
        candidates.append((path.stat().st_mtime, run_dir))
    if not candidates:
        return None
    return max(candidates)[1]


def series_for(rows, *, gated: bool):
    """返回 (位置序列, 断点索引列表, 时间戳, 元信息)。

    - 门控前：用 raw_matrix（原生结果）
    - 门控后：用 matrix（GUI 展示的位姿）
    断点：无效帧造成的空缺、以及 map_epoch 变化（不同地图坐标系不能连线）。
    断点索引按 `TrajectoryView.draw` 的约定给出：`connected[b-1] = False`，
    即 b 是紧凑点列里"新段第一个点"的下标。
    """
    points, stamps, epochs = [], [], []
    for row in rows:
        if gated:
            matrix = row.get("matrix") if row.get("pose_valid") else None
        else:
            matrix = audit.raw_matrix(row)
        stamps.append(row.get("timestamp_ns"))
        epochs.append(row.get("map_epoch"))
        if matrix is None:
            points.append(None)
            continue
        a = np.asarray(matrix, dtype=float)
        points.append(a[:3, 3])

    compact, breaks = [], []
    previous_present = False
    previous_epoch = None
    for point, epoch in zip(points, epochs):
        if point is None:
            previous_present = False
            continue
        new_segment = (not previous_present) or (previous_epoch != epoch)
        if new_segment and compact:  # 段起点（紧凑下标）
            breaks.append(len(compact))
        compact.append(point)
        previous_present = True
        previous_epoch = epoch
    valid = np.asarray(compact, dtype=float).reshape(-1, 3)
    breaks = [b for b in breaks if 0 < b < len(valid)]
    return valid, breaks, stamps, epochs


def stats_for(rows):
    """统计口径直接复用 audit_vio_stability.audit_rows（0.5 m / 0.2 s 与验收报告一致）。"""
    report, _series, _pairs = audit.audit_rows(rows, max_gap_s=MAX_GAP_S, jump_m=JUMP_M)
    pairs = report.get("pairs", {})
    same = pairs.get("same_exact_epoch_short_gap", {}) or {}
    changed = pairs.get("changed_epoch_short_gap", {}) or {}
    return {
        "frames": report.get("pose_records", 0),
        "raw_valid": report.get("raw_valid_count", 0),
        "gated_valid": report.get("gated_valid_count", 0),
        "same_jumps": same.get("investigate_jump_count", 0),
        "same_max": (same.get("translation_m") or {}).get("max"),
        "same_p95": (same.get("translation_m") or {}).get("p95"),
        "changed_jumps": changed.get("investigate_jump_count", 0),
        "changed_max": (changed.get("translation_m") or {}).get("max"),
        "duration_s": report.get("duration_image_clock_s"),
        "states": report.get("tracking_states", {}),
    }


# --------------------------------------------------------------------------- 面板

class PlaneView(QWidget):
    """无 OpenGL 时的二维顶视图（X-Y，Z 用颜色深浅），接口与 TrajectoryView 对齐。

    只在无法创建 GL 上下文（offscreen / 远程桌面 / 无显卡驱动）时启用，
    保证调试窗口在这些环境也能出图，而不是四块黑屏。
    """

    def __init__(self, color=(0.2, 0.87, 0.74, 1.0), parent=None):
        super().__init__(parent)
        self.setMinimumHeight(240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.color = color
        self.points = np.empty((0, 3))
        self.breaks: list[int] = []
        self.setAutoFillBackground(True)
        self.setStyleSheet("background:#0E141D;")

    def draw(self, estimated, truth=None, estimate_on=True, truth_on=False, index=None,
             fit=False, *, estimate_breaks=()):
        self.points = np.asarray(estimated, dtype=float).reshape(-1, 3)
        self.breaks = [int(b) for b in (estimate_breaks or ())]
        self.update()

    def setCameraPosition(self, **kwargs):  # 兼容统一尺度调用
        pass

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update()

    def paintEvent(self, event):
        from PySide6.QtGui import QColor, QPainter, QPen, QBrush
        from PySide6.QtCore import QPointF, Qt as QtNS

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#0E141D"))
        painter.setPen(QPen(QColor("#2A3644"), 1))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))

        if len(self.points) < 2:
            painter.setPen(QPen(QColor("#6B7B8C"), 1))
            painter.drawText(self.rect(), QtNS.AlignCenter, "无有效位姿")
            painter.end()
            return

        xy = self.points[:, :2]
        lo, hi = xy.min(0), xy.max(0)
        span = np.maximum(hi - lo, 1e-6)
        margin_px = 18.0
        width = max(10.0, self.width() - 2 * margin_px)
        height = max(10.0, self.height() - 2 * margin_px)
        scale = min(width / span[0], height / span[1])
        centre_px = np.array([self.width() / 2, self.height() / 2])
        centre_m = (lo + hi) / 2

        def to_px(p):
            q = (np.asarray(p, dtype=float) - centre_m) * scale
            return QPointF(float(centre_px[0] + q[0]), float(centre_px[1] - q[1]))

        # 度量参考网格
        painter.setPen(QPen(QColor("#1E2A38"), 1))
        step = 1.0 if max(span) > 4 else (0.5 if max(span) > 1 else 0.1)
        for k in range(-20, 21):
            x = centre_m[0] + k * step
            p1, p2 = to_px((x, centre_m[1] - span[1])), to_px((x, centre_m[1] + span[1]))
            painter.drawLine(p1, p2)
            y = centre_m[1] + k * step
            p1, p2 = to_px((centre_m[0] - span[0], y)), to_px((centre_m[0] + span[0], y))
            painter.drawLine(p1, p2)

        break_set = set(self.breaks)
        zs = self.points[:, 2]
        z_lo, z_hi = float(zs.min()), float(zs.max())
        z_span = max(z_hi - z_lo, 1e-6)
        base = QColor.fromRgbF(*self.color)
        painter.setPen(QPen(base, 2))
        for i in range(1, len(self.points)):
            if i in break_set:
                continue
            shade = 0.45 + 0.55 * (1.0 - (zs[i] - z_lo) / z_span)
            colour = QColor(base)
            colour = colour.darker(int(100 / max(shade, 0.2)))
            painter.setPen(QPen(colour, 2))
            painter.drawLine(to_px(xy[i - 1]), to_px(xy[i]))
            # 相邻点位移 > 0.5 m 的异常段标红
            step_m = float(np.linalg.norm(self.points[i] - self.points[i - 1]))
            if step_m > JUMP_M:
                painter.setPen(QPen(QColor("#FF5555"), 3))
                painter.drawLine(to_px(xy[i - 1]), to_px(xy[i]))
                painter.setBrush(QBrush(QColor("#FF5555")))
                painter.setPen(QtNS.NoPen)
                painter.drawEllipse(to_px(xy[i]), 4, 4)

        painter.setPen(QPen(QColor("#C8D6E5"), 1))
        painter.drawText(6, self.height() - 6,
                         f"2D 顶视图 X-Y · 跨度 {span[0]:.2f}×{span[1]:.2f} m · Z {z_lo:+.2f}…{z_hi:+.2f} m")
        painter.end()


def make_view(color, *, allow_gl=True):
    """默认必须用 OpenGL（与 GUI 主界面同一个 TrajectoryView）。

    `--no-gl` / `EGOHAND_DEBUG_NO_GL=1` 才退回二维顶视图，用于完全没有 GL 的环境。
    注意：`QT_QPA_PLATFORM=offscreen` 无法创建 GL 上下文，那种情况下必须显式 --no-gl，
    否则直接报错而不是静默黑屏。
    """
    import os
    if not allow_gl or os.environ.get("EGOHAND_DEBUG_NO_GL") == "1":
        return PlaneView(color), False
    from egohand.gui.vio_panel import TrajectoryView
    view = TrajectoryView()
    view.setMinimumHeight(240)
    return view, True


class TrajectoryPanel(QWidget):
    """一个"图"：标题 + 统计 + 三维轨迹（OpenGL，复用 GUI 的 TrajectoryView）。"""

    def __init__(self, title: str, subtitle: str, color, parent=None, *, allow_gl=True):
        super().__init__(parent)

        self.title = QLabel(title)
        self.title.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        self.subtitle = QLabel(subtitle)
        self.subtitle.setStyleSheet("color:#9FB0C0;")
        self.stats = QLabel("等待数据…")
        self.stats.setStyleSheet("color:#C8D6E5; font-family:Consolas,'Microsoft YaHei'; font-size:11px;")
        self.stats.setWordWrap(True)
        self.view, self.is_gl = make_view(color, allow_gl=allow_gl)
        self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.color = color
        self._points = np.empty((0, 3))
        self._breaks: list[int] = []

        head = QVBoxLayout()
        head.setSpacing(1)
        head.addWidget(self.title)
        head.addWidget(self.subtitle)
        head.addWidget(self.stats)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 6)
        layout.addLayout(head)
        layout.addWidget(self.view, 1)

    def set_data(self, points: np.ndarray, breaks, stats: dict, *, fit=True):
        self._points = np.asarray(points, dtype=float).reshape(-1, 3)
        self._breaks = list(breaks or [])
        self.stats.setText(self._format(stats))
        self.redraw(fit=fit)

    def jump_indices(self):
        """同段相邻点位移 > 0.5 m 的位置（用于在三维图里标红）。"""
        if len(self._points) < 2:
            return []
        out = []
        break_set = set(self._breaks)
        for i in range(1, len(self._points)):
            if i in break_set:
                continue
            if float(np.linalg.norm(self._points[i] - self._points[i - 1])) > JUMP_M:
                out.append(i)
        return out

    def redraw(self, *, fit=True):
        if not self.is_gl:
            if not len(self._points):
                self.view.draw(np.empty((0, 3)), np.empty((0, 3)), True, False, None, fit=False)
                return
            self.view.draw(self._points, np.empty((0, 3)), True, False, None, fit=fit,
                           estimate_breaks=self._breaks)
            return
        # 先清掉上一轮的标记
        for item in getattr(self, "_markers", []):
            try:
                self.view.removeItem(item)
            except Exception:  # noqa: BLE001
                pass
        self._markers = []
        truth = np.empty((0, 3))
        self.view.draw(self._points, truth, True, False, None, fit=fit,
                       estimate_breaks=self._breaks)
        if not len(self._points):
            return
        import pyqtgraph.opengl as gl
        from PySide6.QtGui import QColor

        jumps = self.jump_indices()
        if jumps:
            item = gl.GLScatterPlotItem(pos=self._points[jumps], color=QColor("#FF5555"),
                                        size=10, pxMode=True, glOptions="translucent")
            self.view.addItem(item)
            self._markers.append(item)
        if self._breaks:
            item = gl.GLScatterPlotItem(pos=self._points[self._breaks], color=QColor("#FFD166"),
                                        size=8, pxMode=True, glOptions="translucent")
            self.view.addItem(item)
            self._markers.append(item)
        self.view.update()

    def _format(self, s: dict) -> str:
        def m(value):
            return "—" if value is None else f"{value:.2f} m"

        same_max = s.get("same_max")
        changed_max = s.get("changed_max")
        lines = [
            f"帧 {s['frames']} · 原生有效 {s['raw_valid']} · 门控后 {s['gated_valid']}"
            f" · 时长 {0 if s['duration_s'] is None else round(s['duration_s'], 1)} s",
            f"同地图最大步长 {m(same_max)} · >{JUMP_M} m {s['same_jumps']} 次"
            f" | 跨地图最大跳变 {m(changed_max)} · >{JUMP_M} m {s['changed_jumps']} 次",
        ]
        if self._points.size:
            extent = self._points.max(0) - self._points.min(0)
            lines.append(f"轨迹跨度 X/Y/Z {extent[0]:.2f} / {extent[1]:.2f} / {extent[2]:.2f} m")
        return "\n".join(lines)


class RunWorker(QThread):
    """在后台用现有脚本重新跑两组实验（纯视觉 / 视觉+IMU）。"""

    message = Signal(str)
    finished_ok = Signal(object)

    def __init__(self, recording: Path, library: Path, out_root: Path, parent=None):
        super().__init__(parent)
        self.recording = Path(recording)
        self.library = Path(library)
        self.out_root = Path(out_root)

    def _run(self, name: str, extra: list[str]):
        out = self.out_root / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}"
        cmd = [str(ROOT / ".venv/Scripts/python.exe"), str(ROOT / "scripts/run_vio_stability_experiment.py"),
               "--recording", str(self.recording), "--output", str(out),
               "--library", str(self.library)] + extra
        self.message.emit(f"运行 {name} …\n{' '.join(cmd)}")
        proc = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
        if proc.returncode != 0:
            raise RuntimeError(f"{name} 失败：{proc.stderr[-800:] or proc.stdout[-800:]}")
        return out

    def run(self):
        try:
            stereo = self._run("stereo_only", ["--stereo-only"])
            inertial = self._run("stereo_inertial", [])
            self.finished_ok.emit({"stereo_only": str(stereo), "stereo_inertial": str(inertial)})
        except Exception as exc:  # noqa: BLE001 - 界面必须显示原因
            self.message.emit(f"失败：{exc}")
            self.finished_ok.emit(None)


# --------------------------------------------------------------------------- 窗口

class VioDebugWindow(QMainWindow):
    def __init__(self, stereo_only, stereo_inertial, *, recording=None, library=None, auto=False):
        super().__init__()
        self.setWindowTitle("VIO 对比调试：纯视觉 / 视觉+IMU × 门控前/后")
        self.resize(1360, 900)
        self.recording = recording
        self.library = library
        self.auto = auto
        self.panels: dict[str, TrajectoryPanel] = {}
        self.runs = {"stereo_only": stereo_only, "stereo_inertial": stereo_inertial}
        self.worker: RunWorker | None = None

        central = QWidget()
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(8, 6, 8, 8)

        bar = QHBoxLayout()
        self.status = QLabel("就绪")
        self.status.setStyleSheet("color:#9FB0C0;")
        bar.addWidget(self.status, 1)
        self.unified = QCheckBox("统一坐标尺度（四图同比例）")
        self.unified.setChecked(False)
        self.unified.stateChanged.connect(lambda *_: self.redraw_all())
        bar.addWidget(self.unified)
        self.sync_view = QCheckBox("同步视角")
        self.sync_view.setChecked(True)
        self.sync_view.stateChanged.connect(self._apply_sync_mode)
        bar.addWidget(self.sync_view)
        reload_btn = QPushButton("重新载入")
        reload_btn.clicked.connect(self.load_all)
        bar.addWidget(reload_btn)
        run_btn = QPushButton("重新跑两组（复用 run_vio_stability_experiment.py）")
        run_btn.setEnabled(bool(recording and library))
        run_btn.clicked.connect(self.rerun)
        bar.addWidget(run_btn)
        shot = QPushButton("保存截图")
        shot.clicked.connect(self.screenshot)
        bar.addWidget(shot)
        root_layout.addLayout(bar)

        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(6)
        spec = [
            ("stereo_only_raw", "① 纯视觉 · 门控前（原生）", "Stereo / 无 IMU（门控前）", (0.55, 0.75, 1.0, 1.0)),
            ("stereo_only_gated", "② 纯视觉 · 门控后（GUI 展示）", "Stereo / 无 IMU（门控后）", (0.20, 0.87, 0.74, 1.0)),
            ("stereo_inertial_raw", "③ 视觉+IMU · 门控前（原生）", "Stereo-Inertial（门控前）", (1.0, 0.68, 0.45, 1.0)),
            ("stereo_inertial_gated", "④ 视觉+IMU · 门控后（GUI 展示）", "Stereo-Inertial（门控后）", (0.85, 0.45, 0.95, 1.0)),
        ]
        for key, title, subtitle, color in spec:
            panel = TrajectoryPanel(title, subtitle, color)
            self.panels[key] = panel
        grid.addWidget(self.panels["stereo_only_raw"], 0, 0)
        grid.addWidget(self.panels["stereo_only_gated"], 0, 1)
        grid.addWidget(self.panels["stereo_inertial_raw"], 1, 0)
        grid.addWidget(self.panels["stereo_inertial_gated"], 1, 1)
        for i in (0, 1):
            grid.setRowStretch(i, 1)
            grid.setColumnStretch(i, 1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(grid_host)
        root_layout.addWidget(scroll, 1)
        self.setCentralWidget(central)

        self.resize_timer = QTimer(self)
        self.resize_timer.setSingleShot(True)
        self.resize_timer.timeout.connect(self._refit)
        self._apply_sync_mode()
        self.load_all()

    # -- 视角同步 ---------------------------------------------------------
    def _apply_sync_mode(self):
        """勾选时：任意一张图的鼠标操作（旋转/平移/缩放）同步到其余三张。"""
        if not hasattr(self, "_sync_installed"):
            self._sync_installed = True
            self._syncing = False
        for panel in self.panels.values():
            if not panel.is_gl:
                continue
            view = panel.view
            if getattr(view, "_vio_sync_hooked", False):
                continue
            view._vio_sync_hooked = True
            for handler in ("mouseReleaseEvent", "wheelEvent"):
                original = getattr(type(view), handler, None)
                if original is None:
                    continue
                hook = self._make_hook(handler, original)
                setattr(view, handler, hook.__get__(view, type(view)))
        return

    def _make_hook(self, handler_name, original):
        owner = self

        def hook(view_self, event):
            original(view_self, event)
            if not owner.sync_view.isChecked() or getattr(owner, "_syncing", False):
                return
            owner._syncing = True
            try:
                source = view_self.opts
                for panel in owner.panels.values():
                    other = panel.view
                    if not panel.is_gl or other is view_self:
                        continue
                    other.opts["center"] = source["center"]
                    other.opts["elevation"] = source["elevation"]
                    other.opts["azimuth"] = source["azimuth"]
                    other.opts["distance"] = source["distance"]
                    other.update()
            finally:
                owner._syncing = False

        return hook

    # -- 载入 -------------------------------------------------------------
    def resolve_runs(self):
        if self.auto or self.runs["stereo_only"] is None or self.runs["stereo_inertial"] is None:
            newest_stereo = find_newest_run("", require_stereo_only=True)
            newest_inertial = find_newest_run("", require_stereo_only=False)
            if self.runs["stereo_only"] is None and newest_stereo is not None:
                self.runs["stereo_only"] = newest_stereo
            if self.runs["stereo_inertial"] is None and newest_inertial is not None:
                self.runs["stereo_inertial"] = newest_inertial

    def load_all(self):
        self.resolve_runs()
        ok = []
        for mode in ("stereo_only", "stereo_inertial"):
            run = self.runs.get(mode)
            label = "纯视觉" if mode == "stereo_only" else "视觉+IMU"
            if not run:
                self.panels[f"{mode}_raw"].stats.setText("未找到该组数据")
                self.panels[f"{mode}_gated"].stats.setText("未找到该组数据")
                continue
            try:
                rows, path = load_rows(Path(run))
            except (OSError, FileNotFoundError, json.JSONDecodeError) as exc:
                self.status.setText(f"{label} 载入失败：{exc}")
                continue
            st = stats_for(rows)
            raw_pts, raw_breaks, _s, _e = series_for(rows, gated=False)
            gated_pts, gated_breaks, _s, _e = series_for(rows, gated=True)
            self.panels[f"{mode}_raw"].set_data(raw_pts, raw_breaks, st)
            self.panels[f"{mode}_gated"].set_data(gated_pts, gated_breaks, st)
            ok.append(f"{label}←{path.parent.name}")
        self.status.setText("已载入：" + " | ".join(ok) if ok else "未载入任何数据（用 --stereo-only/--stereo-inertial 指定）")
        self.redraw_all()

    def redraw_all(self):
        fit = not self.unified.isChecked()
        if self.unified.isChecked():
            self._fit_unified()
        else:
            for panel in self.panels.values():
                panel.redraw(fit=True)

    def _fit_unified(self):
        """统一尺度：用四张图里最大的跨度设同一相机距离，便于横向比较。"""
        radius = 0.2
        for panel in self.panels.values():
            if panel._points.size:
                centre = (panel._points.min(0) + panel._points.max(0)) / 2
                radius = max(radius, float(np.linalg.norm(panel._points - centre, axis=1).max()))
        for panel in self.panels.values():
            panel.redraw(fit=True)
            if panel._points.size:
                panel.view.setCameraPosition(distance=radius * 3.4, elevation=35, azimuth=-55)

    def _refit(self):
        if not self.unified.isChecked():
            for panel in self.panels.values():
                panel.redraw(fit=True)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "resize_timer"):
            self.resize_timer.start(120)

    # -- 动作 -------------------------------------------------------------
    def rerun(self):
        if not (self.recording and self.library):
            self.status.setText("需要 --recording 与 --library 才能重新跑")
            return
        self.worker = RunWorker(Path(self.recording), Path(self.library),
                                ROOT / "results" / "vio" / "debug_window")
        self.worker.message.connect(self.status.setText)
        self.worker.finished_ok.connect(self._rerun_done)
        self.status.setText("正在运行两组实验（约 6 分钟）…")
        self.worker.start()

    def _rerun_done(self, result):
        if not result:
            self.status.setText("运行失败，详见控制台输出")
            return
        self.runs = result
        self.load_all()

    def screenshot(self):
        out = ROOT / "results" / "vio" / "debug_window"
        out.mkdir(parents=True, exist_ok=True)
        path = out / f"vio_compare_{time.strftime('%Y%m%d_%H%M%S')}.png"
        self.grab().save(str(path))
        self.status.setText(f"截图已保存：{path}")


def parse_args(argv=None):
    import argparse
    p = argparse.ArgumentParser(description="VIO 四联对比调试窗口")
    p.add_argument("--stereo-only", help="纯视觉实验输出目录（含 poses.jsonl）")
    p.add_argument("--stereo-inertial", help="视觉+IMU 实验输出目录（含 poses.jsonl）")
    p.add_argument("--auto", action="store_true", help="自动挑选 results/vio 下最新的一组")
    p.add_argument("--recording", help="重新跑时使用的录制目录")
    p.add_argument("--library", help="重新跑时使用的桥接 DLL")
    p.add_argument("--screenshot", help="不显示窗口，直接截图到该路径（用于无显示器环境验证）")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = QApplication.instance() or QApplication(sys.argv)
    window = VioDebugWindow(args.stereo_only, args.stereo_inertial,
                            recording=args.recording, library=args.library, auto=args.auto)
    if args.screenshot:
        window.show()
        app.processEvents()
        for panel in window.panels.values():
            panel.redraw(fit=True)
        app.processEvents()
        Path(args.screenshot).parent.mkdir(parents=True, exist_ok=True)
        window.grab().save(args.screenshot)
        print(f"screenshot: {args.screenshot}")
        return 0
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
