from bisect import bisect_right
from collections import defaultdict

from PySide6.QtCore import QSignalBlocker, QTimer

from egohand.gui.vio_panel import live_camera_pose_text
from egohand.vio.hand_pose_binding import consumer_result_display_pose
from egohand.vio.local_segments import consumer_display_pose, display_reference_key
from egohand.vio.trajectory import OfflinePose


def recorded_message(result):
    return result.diagnostics.get('vio_pose') or result.diagnostics.get('live_vio_pose')


class RecordedTrajectory:
    def __init__(self, panel):
        self.panel = panel
        self.native_show = panel.show_recorded
        self.sequence = None
        self.segments = {}

    def prepare(self, results, transforms=None):
        if self.sequence is results:
            return
        segments = defaultdict(lambda: ([], [], []))
        gaps = set()
        for index, result in enumerate(results):
            message = recorded_message(result)
            if not isinstance(message, dict):
                continue
            key = display_reference_key(message)
            transform = transforms[index] if transforms is not None else (
                         consumer_result_display_pose(result) if 'live_vio_pose' in result.diagnostics
                         else consumer_display_pose(message, expected_timestamp_ns=(
                             result.timestamp_ns if 'local_motion' in message else None)))
            if transform is None:
                gaps.add(key)
                continue
            indices, poses, breaks = segments[key]
            if key in gaps and poses:
                breaks.append(len(poses))
            gaps.discard(key)
            indices.append(index)
            poses.append(OfflinePose(message['timestamp_ns'], transform,
                                     getattr(result, 'calibration_id', None) or 'vs030',
                                     pose_source='recorded_admitted_local',
                                     tracking_status=message.get('tracking_state')))
        self.segments = segments
        self.sequence = results

    def show(self, results, index, *, show_hands=True, incremental=False):
        panel = self.panel
        self.prepare(results)
        current = recorded_message(results[index])
        if (incremental and panel._recorded_sequence is results
                and index == panel._recorded_index + 1 and isinstance(current, dict)
                and current.get('timestamp_ns') == results[index].timestamp_ns
                and current.get('type') == 'pose'):
            return self.native_show(results, index, show_hands=show_hands, incremental=True)
        panel.clear()
        panel.begin_live()
        panel._recorded_sequence = results
        panel._recorded_index = index
        if not isinstance(current, dict):
            panel.preview.set_hands([], None)
            return
        indices, poses, breaks = self.segments.get(display_reference_key(current), ((), (), ()))
        count = bisect_right(indices, index)
        panel.poses = poses[:count]
        panel._live_breaks = [point for point in breaks if point < count]
        with QSignalBlocker(panel.slider):
            panel.slider.setRange(0, max(0, count - 1))
            panel.slider.setValue(max(0, count - 1))
        panel._update_view(fit=False)
        if not count or indices[count - 1] != index:
            panel.preview.set_pose_axes(None)
        panel.live_pose_readout.emit(live_camera_pose_text(current))
        motion = current.get('local_motion') or {}
        panel._recorded_details = (
            f'已有结果 · {motion.get("pose_source") or current.get("mode") or "来源未知"}'
            f' · 局部段 {motion.get("segment_index", "未知")} · {count} 位姿\n'
            '按已保存的分段身份显示；不连接不兼容坐标')
        panel.details.setText(panel._recorded_details)
        if show_hands:
            panel.set_hand_result(results[index])
            QTimer.singleShot(0, panel.preview.fit_geometry)
