"""Best effort camera pose selection for GUI display only.

This module deliberately does not grant localization admission.  It selects a
pose for visualization from the already admitted pose, a bounded IMU-only
prediction, a validated native visual candidate, or the last displayed pose.
"""
from dataclasses import dataclass
from typing import Optional
import numpy as np

from .trajectory import rigid_transform
from .local_segments import consumer_display_pose
from .map_identity import world_identity


@dataclass(frozen=True)
class BestEffortCameraPose:
    timestamp_ns: Optional[int]
    source_frame_id: object
    source_session_id: object
    source: str
    status: str
    T_reference_from_camera: Optional[np.ndarray]
    reference_id: object
    segment_id: object
    coordinate_frame: str
    trusted: bool
    prediction_duration_s: Optional[float]
    age_ms: Optional[float]
    stale: bool
    reason: str

    @property
    def matrix(self):
        return self.T_reference_from_camera


def _valid_stamp(value):
    return type(value) is int and value >= 0


def _valid_matrix(value):
    try:
        matrix = rigid_transform(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not np.isfinite(matrix).all():
        return None
    return matrix


def _identity(message):
    """Return the map/segment identity carried by the current message."""
    try:
        # Keep the map identity stable while local_motion is temporarily
        # unavailable during loss.  Segment changes are checked separately.
        return _normalize_identity(world_identity(message))
    except (AttributeError, KeyError, TypeError, ValueError):
        return None


def _normalize_identity(value):
    if isinstance(value, list):
        return tuple(_normalize_identity(item) for item in value)
    if isinstance(value, tuple):
        return tuple(_normalize_identity(item) for item in value)
    return value


def _contract_extrinsic(message, explicit):
    value = explicit
    if value is None:
        contract = message.get('display_pose_contract') or {}
        value = contract.get('T_body_from_camera')
    return _valid_matrix(value)


def _candidate(message):
    candidate = message.get('display_visual_candidate')
    if not isinstance(candidate, dict):
        # Kept for recorded logs made before the additive display field.
        candidate = None
        if message.get('raw_pose_valid') and message.get('raw_matrix') is not None:
            candidate = {'matrix': message.get('raw_matrix'),
                         'timestamp_ns': message.get('timestamp_ns'),
                         'frame_id': message.get('frame_id')}
    if not isinstance(candidate, dict):
        return None, 'visual_candidate_missing'
    stamp = candidate.get('timestamp_ns', message.get('timestamp_ns'))
    if stamp != message.get('timestamp_ns') or not _valid_stamp(stamp):
        return None, 'visual_candidate_timestamp_mismatch'
    matrix = _valid_matrix(candidate.get('matrix'))
    if matrix is None:
        return None, 'visual_candidate_invalid_matrix'
    candidate_identity = _normalize_identity(candidate.get('reference_id'))
    current_identity = _identity(message)
    if candidate_identity is not None and current_identity is not None and candidate_identity != current_identity:
        return None, 'visual_candidate_reference_mismatch'
    if candidate.get('frame_id') is not None and candidate.get('frame_id') != message.get('frame_id'):
        return None, 'visual_candidate_frame_mismatch'
    if candidate.get('session_id') is not None and candidate.get('session_id') != message.get('session_id'):
        return None, 'visual_candidate_session_mismatch'
    return matrix, None


class BestEffortPoseSelector:
    """Stateful display selector with bounded prediction and freeze behavior."""

    def __init__(self, *, max_prediction_s=2.0):
        self.max_prediction_s = float(max_prediction_s)
        self.last: Optional[BestEffortCameraPose] = None
        self.last_identity = None
        self.last_segment_from_map = None
        self.last_segment_id = None
        self.last_extrinsic = None
        self.has_trusted_pose = False

    def reset(self):
        self.last = None
        self.last_identity = None
        self.last_segment_from_map = None
        self.last_segment_id = None
        self.last_extrinsic = None
        self.has_trusted_pose = False

    def update(self, message, *, now_timestamp_ns=None):
        stamp = message.get('timestamp_ns')
        identity = _identity(message)
        if identity is not None and self.last_identity is not None and identity != self.last_identity:
            # A new map/segment cannot inherit a pose from the previous frame.
            self.last = None
            self.last_segment_from_map = None
            self.last_segment_id = None
        if identity is not None:
            self.last_identity = identity

        contract = message.get('display_pose_contract') or {}
        extrinsic = _contract_extrinsic(message, self.last_extrinsic)
        if extrinsic is not None:
            self.last_extrinsic = extrinsic

        motion = message.get('local_motion')
        if isinstance(motion, dict) and motion.get('status') == 'estimated':
            segment = _valid_matrix(motion.get('T_segment_from_map'))
            segment_id = motion.get('segment_id')
            if (segment_id is not None and self.last_segment_id is not None
                    and segment_id != self.last_segment_id):
                self.last = None
                self.last_segment_from_map = None
            if segment is not None:
                self.last_segment_from_map = segment
                self.last_segment_id = segment_id

        trusted = None
        try:
            trusted = consumer_display_pose(message)
        except (TypeError, ValueError, OverflowError, KeyError):
            trusted = None
        if trusted is not None and _valid_stamp(stamp):
            self.has_trusted_pose = True
            pose = self._make(message, trusted, 'trusted', 'trusted_pose', True, identity,
                              prediction_duration_s=None, now_timestamp_ns=now_timestamp_ns)
            self.last = pose
            return pose

        prediction = (message.get('recovery_preaccept') or {}).get('loss_prediction') or {}
        predicted = prediction.get('status') == 'predicted'
        duration = prediction.get('duration_s')
        valid_duration = (isinstance(duration, (int, float)) and not isinstance(duration, bool)
                          and np.isfinite(duration) and 0 <= float(duration) <= self.max_prediction_s)
        body = _valid_matrix(prediction.get('T_world_from_body')) if predicted and valid_duration else None
        if body is not None and extrinsic is not None and _valid_stamp(stamp):
            # Prediction is T_reference_from_body.  Convert to the GUI left
            # camera with the calibrated body-from-camera extrinsic.
            camera = body @ extrinsic
            if identity is not None and self.last_segment_from_map is not None and self.last_identity == identity:
                camera = self.last_segment_from_map @ camera
                frame = 'segment'
            else:
                frame = str(contract.get('reference_frame') or 'map')
            pose = self._make(message, camera, 'imu_prediction', 'prediction', False, identity,
                              prediction_duration_s=float(duration), now_timestamp_ns=now_timestamp_ns,
                              coordinate_frame=frame, reason='bounded_imu_prediction')
            self.last = pose
            return pose

        candidate, reason = _candidate(message)
        if candidate is not None and _valid_stamp(stamp):
            pose = self._make(message, candidate, 'visual_candidate', 'candidate', False, identity,
                              prediction_duration_s=None, now_timestamp_ns=now_timestamp_ns,
                              reason='visual_candidate_not_admitted')
            self.last = pose
            return pose

        if self.last is not None and self.last.matrix is not None:
            age = self._age_ms(self.last.timestamp_ns, stamp, now_timestamp_ns)
            frozen = BestEffortCameraPose(
                # Keep the timestamp of the last *new* estimate. Updating it
                # on every frozen frame would make the displayed age reset to
                # one frame forever.
                timestamp_ns=self.last.timestamp_ns,
                source_frame_id=message.get('frame_id', self.last.source_frame_id),
                source_session_id=message.get('session_id', self.last.source_session_id),
                source='frozen', status='frozen', T_reference_from_camera=self.last.matrix.copy(),
                reference_id=self.last.reference_id, segment_id=self.last.segment_id,
                coordinate_frame=self.last.coordinate_frame, trusted=False,
                prediction_duration_s=None, age_ms=age, stale=True,
                reason=reason or 'no_new_valid_estimate')
            self.last = frozen
            return frozen

        return BestEffortCameraPose(
            timestamp_ns=stamp if _valid_stamp(stamp) else None,
            source_frame_id=message.get('frame_id'), source_session_id=message.get('session_id'),
            source='origin', status='origin', T_reference_from_camera=np.eye(4),
            reference_id=identity, segment_id=None, coordinate_frame='local_origin', trusted=False,
            prediction_duration_s=None, age_ms=None, stale=True,
            reason=reason or 'no_pose_initialized')

    def _make(self, message, matrix, source, status, trusted, identity, *,
              prediction_duration_s, now_timestamp_ns, coordinate_frame=None, reason=None):
        motion = message.get('local_motion') or {}
        frame = coordinate_frame or ('segment' if motion.get('segment_id') else 'map')
        age = self._age_ms(message.get('timestamp_ns'), message.get('timestamp_ns'), now_timestamp_ns)
        return BestEffortCameraPose(
            timestamp_ns=message.get('timestamp_ns'), source_frame_id=message.get('frame_id'),
            source_session_id=(motion.get('session_id') or message.get('session_id')),
            source=source, status=status, T_reference_from_camera=matrix.copy(),
            reference_id=identity, segment_id=motion.get('segment_id') or self.last_segment_id,
            coordinate_frame=frame, trusted=trusted, prediction_duration_s=prediction_duration_s,
            age_ms=age, stale=False, reason=reason or status)

    @staticmethod
    def _age_ms(pose_stamp, current_stamp, now_timestamp_ns):
        if now_timestamp_ns is not None and _valid_stamp(pose_stamp) and _valid_stamp(now_timestamp_ns):
            return max(0., (int(now_timestamp_ns) - int(pose_stamp)) / 1e6)
        if _valid_stamp(pose_stamp) and _valid_stamp(current_stamp):
            return max(0., (int(current_stamp) - int(pose_stamp)) / 1e6)
        return 0.


def choose_best_effort_pose(message, *, selector=None, now_timestamp_ns=None):
    selector = selector or BestEffortPoseSelector()
    return selector.update(message, now_timestamp_ns=now_timestamp_ns)

