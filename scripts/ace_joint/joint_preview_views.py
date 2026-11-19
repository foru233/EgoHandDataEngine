from dataclasses import replace

import numpy as np
from egohand.core.types import StereoFrame
from egohand.gui.shared.presentation import display_stereo


def scaled_result(result, scale):
    def point(uv):
        return None if uv is None else tuple((np.asarray(uv) + .5) * scale - .5)

    def detections(items):
        return [replace(item, pixel_landmarks=(item.pixel_landmarks + .5) * scale - .5)
                for item in items]

    hands = [replace(hand, joints=[replace(joint, left_uv_px=point(joint.left_uv_px),
                                          right_uv_px=point(joint.right_uv_px))
                                  for joint in hand.joints]) for hand in result.hands]
    return replace(result, hands=hands, left_detections=detections(result.left_detections),
                   right_detections=detections(result.right_detections))


def show_preview(window, replay, images, caption):
    raw_left, raw_right, left, right, depth = images
    result = window.latest
    calibration = replay.calibration
    scale = np.array((left.shape[1] / calibration.image_width, left.shape[0] / calibration.image_height))
    frame = StereoFrame(result.frame_id, result.timestamp_ns, left, right)
    display_stereo(frame, scaled_result(result, scale), window.left_view, window.right_view)
    window.raw_left_view.display(raw_left)
    window.raw_right_view.display(raw_right)
    geometry = '校正' if result.diagnostics.get('output_geometry') == 'rectified' else '原始'
    window.left_tile.set_caption(f'{geometry} · 左')
    window.right_tile.set_caption(f'{geometry} · 右')
    window.raw_left_tile.set_caption('原始 · 左')
    window.raw_right_tile.set_caption('原始 · 右')
    window.on_depth(window.generation, depth, caption)
    window._saved_depth_loaded = True
    window._sync_aspect(frame)
    status = '预计算双目与保存结果同帧'
    window.source_label.setToolTip(status)
    window.footer.setToolTip(window.footer.toolTip().replace('原始图像不可用 · 三维结果可查看', status))
