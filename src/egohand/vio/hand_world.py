"""显式绑定相同标定和时间域，将现有手部结果另存为世界坐标。"""
import numpy as np

from .trajectory import transform_hand_points


def world_hands_for_result(result, provider, *, left_image_timestamp_ns, result_clock_domain, pose_clock_domain, pose_session_id):
    """不修改原始结果；缺失位姿、标定或时钟域时不输出世界关节。"""
    response = {'frame_id': result.frame_id, 'timestamp_ns': result.timestamp_ns,
                'session_id': result.session_id, 'coordinate_frame': 'vio_local_world',
                'unit': 'm', 'status': 'unavailable', 'hands': []}
    if not result.session_id or not pose_session_id or result.session_id != pose_session_id:
        response['reason'] = 'session_mismatch'
        return response
    if not result_clock_domain or result_clock_domain != pose_clock_domain:
        response['reason'] = 'clock_domain_mismatch'
        return response
    if left_image_timestamp_ns != result.timestamp_ns:
        response['reason'] = 'left_image_timestamp_mismatch'
        return response
    if result.coordinate_frame != 'camera_left_optical' or not result.calibration_id:
        response['reason'] = 'invalid_camera_frame_or_calibration'
        return response
    pose = provider.pose_at(result.timestamp_ns)
    if pose is None:
        response['reason'] = 'no_synchronized_pose'
        return response
    if pose.calibration_id != result.calibration_id:
        response['reason'] = 'calibration_mismatch'
        return response
    for hand in result.hands:
        xyz = np.full((len(hand.joints), 3), np.nan)
        valid = np.array([joint.valid and np.isfinite(joint.xyz_m).all() for joint in hand.joints], dtype=bool)
        if valid.any():
            points = np.array([joint.xyz_m for joint in hand.joints], dtype=float)
            xyz[valid] = transform_hand_points(points[valid], pose.T_world_from_camera_left)
        response['hands'].append({'track_id': hand.track_id, 'handedness': hand.handedness,
                                  'xyz_world_m': xyz, 'valid': valid})
    has_valid = any(hand['valid'].any() for hand in response['hands'])
    response.update(status='available' if has_valid else 'no_valid_hand_joints', pose_source=pose.pose_source,
                    tracking_status=pose.tracking_status,
                    pose_timestamp_ns=pose.timestamp_ns,
                    calibration_id=pose.calibration_id)
    return response
