"""显式绑定相同标定和时间域，将现有手部结果另存为世界坐标。"""
import hashlib
import json
from pathlib import Path

import numpy as np

from .trajectory import transform_hand_points


def conservative_world_reference(reference=None):
    """Normalize recorded identity while never claiming continuity or release."""
    reference = reference if isinstance(reference, dict) else {}
    def uint(name, bits):
        value = reference.get(name)
        return value if type(value) is int and 0 <= value < (1 << bits) else None

    normalized = {'schema_version': 1,
            'map_epoch': uint('map_epoch', 64),
            'world_frame_revision': uint('world_frame_revision', 64),
            'identity_status': 'unverified',
            'coordinate_continuity': 'unknown',
            'cross_frame_merge_permitted': False}
    if 'reference_generation' in reference:
        normalized['reference_generation'] = uint('reference_generation',64)
    return normalized


def _world_reference(pose=None):
    """Record available identity without asserting reference continuity.

    A map epoch or revision alone is not an atomic map-event proof. Existing
    EuRoC trajectory poses do not carry either value, so both stay unknown.
    """
    return conservative_world_reference({
        'map_epoch': getattr(pose, 'map_epoch', None),
        'world_frame_revision': getattr(pose, 'world_frame_revision', None),
    })


def _transform_hands(result, transform):
    hands = []
    for hand in result.hands:
        xyz = np.full((len(hand.joints), 3), np.nan)
        valid = np.array([joint.valid and np.isfinite(joint.xyz_m).all()
                          for joint in hand.joints], dtype=bool)
        if valid.any():
            points = np.array([joint.xyz_m for joint in hand.joints], dtype=float)
            xyz[valid] = transform_hand_points(points[valid], transform)
        hands.append({'track_id': hand.track_id, 'handedness': hand.handedness,
                      'xyz_world_m': xyz, 'valid': valid})
    return hands


def calibration_geometry_fingerprint(calibration):
    """Hash numeric camera geometry, units and model, not a file's display name.

    A saved JSON calibration and its original Kalibr file have different bytes
    and IDs. Their geometry must still agree before camera points can be joined.
    This describes supplied calibration; it does not certify physical accuracy.
    """
    if calibration is None:
        raise ValueError('missing calibration')
    values = {}
    for name in ('K_left', 'D_left', 'K_right', 'D_right', 'R_right_left', 't_right_left'):
        value = np.asarray(getattr(calibration, name), dtype=np.float64)
        if not np.isfinite(value).all():
            raise ValueError('nonfinite calibration')
        # Preserve dimensions; do not allow reshaping/rounding to hide changes.
        values[name] = value.tolist()
    for name in ('image_width', 'image_height', 'distortion_model', 'translation_unit', 'synthetic'):
        values[name] = getattr(calibration, name)
    if values['translation_unit'] != 'm':
        raise ValueError('camera geometry must use metres')
    return hashlib.sha256(json.dumps(values, sort_keys=True, allow_nan=False).encode()).hexdigest()


class RecordedLocalizationHandJoin:
    """Join the recorded pipeline's exact SDK pair to committed Map admission.

    This owns only join provenance. LocalizationPipeline and RecoveryCoordinator
    still own admission and recovery. No interpolation, pose cache or continuity
    estimate is added. Each result retains unknown cross-frame continuity.
    """
    def __init__(self, recording_dir, calibration, *, result_session_id,
                 vio_calibration, vio_config_fingerprint, vio_calibration_sha256=None):
        self.recording_dir = Path(recording_dir).resolve()
        self.calibration_id = getattr(calibration, 'calibration_id', None)
        self.calibration_fingerprint = calibration_geometry_fingerprint(calibration)
        self.vio_calibration_fingerprint = calibration_geometry_fingerprint(vio_calibration)
        self.result_session_id = result_session_id
        self.vio_config_fingerprint = vio_config_fingerprint
        self.vio_calibration_sha256 = vio_calibration_sha256
        self.recovery_session_id = None
        self.last_timestamp_ns = None

    @classmethod
    def from_recording(cls, recording_dir, calibration, *, result_session_id, config_snapshot=None):
        from ..calibration.loader import load_calibration
        from .config_snapshot import resolve_recording_config
        config = resolve_recording_config(recording_dir, override=config_snapshot)
        # Replay converts ORB's cam1 pose into this raw GUI cam0 frame.
        vio_calibration = load_calibration(config.calibration, kalibr_left_camera='cam0')
        return cls(recording_dir, calibration, result_session_id=result_session_id,
                   vio_calibration=vio_calibration,
                   # Timing may be independently prepared before Replay starts.
                   # Bind its actual descriptor at the first coherent callback.
                   vio_config_fingerprint=None,
                   vio_calibration_sha256=config.describe()['hashes']['calibration'])

    def apply(self, result, frame, pose, *, calibration):
        from .localization_state import consumer_map_pose
        from .recovery_contract import validate_report, RecoveryProtocolError

        response = {'frame_id': result.frame_id, 'timestamp_ns': result.timestamp_ns,
                    'session_id': result.session_id, 'coordinate_frame': 'vio_local_world',
                    'unit': 'm', 'status': 'unavailable', 'hands': [],
                    'world_reference': conservative_world_reference(),
                    'join': {'schema_version': 1, 'source': 'recorded_localization',
                             'recording': str(self.recording_dir),
                             'vio_config_fingerprint': self.vio_config_fingerprint,
                             'camera_geometry_sha256': self.calibration_fingerprint,
                             'timestamp_basis': 'sdk_device_raw_us_to_ns_candidate',
                             'exposure_synchrony_verified': False}}

        def reject(reason):
            response['reason'] = reason
            return response

        if (not self.result_session_id or result.session_id != self.result_session_id):
            return reject('session_mismatch')
        if (result.coordinate_frame != 'camera_left_optical' or result.unit != 'm'
                or not self.calibration_id or result.calibration_id != self.calibration_id
                or frame.calibration_id != self.calibration_id):
            return reject('invalid_camera_frame_or_calibration')
        try:
            current_calibration = calibration_geometry_fingerprint(calibration)
        except (TypeError, ValueError, AttributeError):
            return reject('calibration_mismatch')
        if (current_calibration != self.calibration_fingerprint
                or current_calibration != self.vio_calibration_fingerprint):
            return reject('calibration_mismatch')
        metadata = frame.metadata
        if (frame.source_type != 'vendor_vs030' or metadata.get('clock_domain') != 'sdk_device'
                or metadata.get('timestamp_kind') != 'sdk_device_timestamp_raw_us_to_ns'):
            return reject('clock_domain_mismatch')
        stamp = result.timestamp_ns
        if (type(stamp) is not int or stamp < 0 or type(frame.timestamp_ns) is not int
                or stamp != frame.timestamp_ns or result.frame_id != frame.frame_id):
            return reject('source_frame_mismatch')
        if self.last_timestamp_ns is not None and stamp <= self.last_timestamp_ns:
            return reject('non_monotonic_join')
        if not isinstance(pose, dict):
            return reject('missing_localization')
        replay = pose.get('replay')
        if not isinstance(replay, dict) or replay.get('source') != 'offline_replay':
            return reject('missing_replay_binding')
        try:
            paths = (metadata.get('recording_dir'), replay.get('recording'))
            if any(not isinstance(p, str) or Path(p).resolve() != self.recording_dir for p in paths):
                return reject('recording_mismatch')
        except (OSError, ValueError):
            return reject('recording_mismatch')
        if (type(replay.get('frame_index')) is not int or replay['frame_index'] != frame.frame_id
                or type(pose.get('timestamp_ns')) is not int or pose['timestamp_ns'] != stamp
                or type(replay.get('device_ts_raw')) is not int
                or replay['device_ts_raw'] * 1000 != stamp
                or replay['device_ts_raw'] != metadata.get('sdk_device_timestamp_raw')
                or type(replay.get('sequence')) is not int
                or replay['sequence'] != metadata.get('sdk_sequence')):
            return reject('source_frame_mismatch')
        config = pose.get('replay_config')
        def digest(value):
            return (isinstance(value, str) and len(value) == 64
                    and all(c in '0123456789abcdef' for c in value))
        if (not isinstance(config, dict) or type(config.get('schema_version')) is not int
                or config['schema_version'] != 1 or config.get('camera_frame') != 'raw_cam0_channel4'
                or not digest(config.get('fingerprint')) or not digest(config.get('calibration_file_sha256'))
                or config.get('camera_geometry_sha256') != self.vio_calibration_fingerprint
                or (self.vio_calibration_sha256 is not None
                    and config['calibration_file_sha256'] != self.vio_calibration_sha256)
                or (self.vio_config_fingerprint is not None
                    and config['fingerprint'] != self.vio_config_fingerprint)):
            return reject('localization_config_mismatch')
        evidence = pose.get('localization')
        if (not isinstance(evidence, dict) or evidence.get('schema_version') != 1
                or evidence.get('contract_version') != 2):
            return reject('localization_contract_required')
        try:
            report = validate_report(pose.get('recovery_preaccept'), pose, stage='committed',
                                     session_id=self.recovery_session_id)
        except RecoveryProtocolError:
            return reject('recovery_binding_invalid')
        # Latch only after source/calibration/transport coherence is established.
        self.recovery_session_id = report['session_id']
        self.vio_config_fingerprint = config['fingerprint']
        self.last_timestamp_ns = stamp
        response['join'].update(recovery_session_id=report['session_id'],
                                vio_config_fingerprint=config['fingerprint'],
                                calibration_file_sha256=config['calibration_file_sha256'],
                                recovery_frame_sequence=report['frame_sequence'],
                                source_frame_index=frame.frame_id,
                                native_frame_id=pose.get('frame_id'))
        response['world_reference'] = conservative_world_reference(pose)
        transform = consumer_map_pose(pose, world_hands=True, expected_timestamp_ns=stamp)
        if transform is None:
            return reject('localization_not_admitted')
        response['hands'] = _transform_hands(result, transform)
        has_valid = any(hand['valid'].any() for hand in response['hands'])
        response.update(status='available' if has_valid else 'no_valid_hand_joints',
                        pose_source='orb_slam3_committed_map', tracking_status=pose.get('tracking_state'),
                        pose_timestamp_ns=stamp, calibration_id=result.calibration_id)
        return response


def world_hands_for_result(result, provider, *, left_image_timestamp_ns, result_clock_domain, pose_clock_domain, pose_session_id):
    """不修改原始结果；缺失位姿、标定或时钟域时不输出世界关节。"""
    response = {'frame_id': result.frame_id, 'timestamp_ns': result.timestamp_ns,
                'session_id': result.session_id, 'coordinate_frame': 'vio_local_world',
                'unit': 'm', 'status': 'unavailable', 'hands': [],
                'world_reference': _world_reference()}
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
    response['world_reference'] = _world_reference(pose)
    response['hands'] = _transform_hands(result, pose.T_world_from_camera_left)
    has_valid = any(hand['valid'].any() for hand in response['hands'])
    response.update(status='available' if has_valid else 'no_valid_hand_joints', pose_source=pose.pose_source,
                    tracking_status=pose.tracking_status,
                    pose_timestamp_ns=pose.timestamp_ns,
                    calibration_id=pose.calibration_id)
    return response
