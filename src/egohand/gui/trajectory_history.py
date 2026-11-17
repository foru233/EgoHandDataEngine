"""Read-only history in separate coordinate references, never a live pose source."""
from dataclasses import dataclass, replace

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QComboBox, QDialog, QLabel, QSlider, QVBoxLayout


@dataclass(frozen=True)
class TrajectorySnapshot:
    reference: object
    poses: tuple
    breaks: tuple
    reason: str

    @classmethod
    def capture(cls, reference, poses, breaks, reason):
        frozen = []
        for pose in poses:
            matrix = pose.T_world_from_camera_left.copy()
            matrix.setflags(write=False)
            frozen.append(replace(pose, T_world_from_camera_left=matrix))
        return cls(reference, tuple(frozen), tuple(breaks), reason)


class TrajectoryHistoryDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('applicationRoot')
        # Local import avoids importing the live panel from its own module load.
        from .vio_panel import TrajectoryView

        self.setWindowTitle('历史轨迹 · 独立参考回看')
        self.resize(800, 600)
        self.entries = ()
        layout = QVBoxLayout(self)
        self.notice = QLabel('仅回看已保存的显示轨迹；各段坐标独立，不与当前定位或手部叠加。')
        self.notice.setWordWrap(True)
        layout.addWidget(self.notice)
        self.selection = QComboBox()
        self.selection.setAccessibleName('历史轨迹段')
        layout.addWidget(self.selection)
        self.preview = TrajectoryView()
        layout.addWidget(self.preview, 1)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setAccessibleName('历史轨迹位姿')
        layout.addWidget(self.slider)
        self.details = QLabel()
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        self.selection.currentIndexChanged.connect(self.select_entry)
        self.slider.valueChanged.connect(self.draw_pose)

    def set_entries(self, entries):
        previous = self.selection.currentIndex()
        self.entries = tuple(entries)
        self.selection.blockSignals(True)
        self.selection.clear()
        for index, entry in enumerate(self.entries):
            self.selection.addItem(f'历史 {index+1} · {entry.reason} · {len(entry.poses)} 个位姿')
        self.selection.setCurrentIndex(max(0, previous))
        self.selection.blockSignals(False)
        if not self.entries:
            self.preview.draw([], [], False, False, None)
            self.preview.set_pose_axes(None, show_world=False)
            self.details.clear()
            self.slider.setRange(0, 0)
            return
        self.select_entry(self.selection.currentIndex())

    def select_entry(self, index):
        if not 0 <= index < len(self.entries):
            return
        entry = self.entries[index]
        self.slider.blockSignals(True)
        self.slider.setRange(0, len(entry.poses)-1)
        self.slider.setValue(len(entry.poses)-1)
        self.slider.blockSignals(False)
        self.draw_pose(fit=True)

    def draw_pose(self, *_, fit=False):
        index = self.selection.currentIndex()
        if not 0 <= index < len(self.entries):
            return
        entry = self.entries[index]
        frame = self.slider.value()
        points = np.asarray([p.T_world_from_camera_left[:3, 3] for p in entry.poses])
        self.preview.draw(points, [], True, False, frame, fit=fit, estimate_breaks=entry.breaks)
        self.preview.set_pose_axes(entry.poses[frame].T_world_from_camera_left, show_world=True)
        xyz = points[frame]
        self.details.setText(f'历史 {index+1} · 位姿 {frame+1}/{len(entry.poses)} · '
                             f'XYZ {xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f} m\n'
                             '历史坐标；不代表当前相机位置。')
