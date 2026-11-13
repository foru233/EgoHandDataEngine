"""Frame presentation policy shared by the research and delivery interfaces."""
from dataclasses import dataclass


@dataclass(frozen=True)
class SessionState:
    running: bool
    stopping: bool
    active: bool
    text: str


def session_state(worker, *, paused=False, has_result=False):
    running = bool(worker and worker.isRunning())
    stopping = running and worker.stop_event.is_set()
    active = running and not stopping
    text = ('正在停止' if stopping else '已暂停' if active and paused else
            '运行中' if active else '已停止' if has_result else '就绪')
    return SessionState(running, stopping, active, text)


def accepted_pairs(result):
    diagnostics = result.diagnostics
    if diagnostics.get('result_kind') in ('official_ground_truth', 'ground_truth_unavailable') or diagnostics.get('synthetic'):
        return None
    return diagnostics.get('association_quality', {}).get('accepted_pairs', [])


def display_stereo(frame, result, left_view, right_view, *, numbers=False,
                   epilines=False, epipolar_axis=1):
    """Overlay only accepted observations on the matching processed pixels."""
    pairs = accepted_pairs(result)
    for side, view, image, detections in (
        ('left', left_view, frame.left_image, result.left_detections),
        ('right', right_view, frame.right_image, result.right_detections),
    ):
        view.display(image, detections, result.hands, numbers, epilines,
                     side, epipolar_axis, accepted_pairs=pairs)


def clear_stereo(*views):
    """A missing frame must never keep showing pixels from a previous frame."""
    for view in views:
        view.set_roi_selection(False)
        view.pix = None
        view.last_bgr = None
        view._image_size = None
        view.clear()
        view.update()
