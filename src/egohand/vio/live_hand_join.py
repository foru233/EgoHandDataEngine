"""Bounded exact-capture join; localization admission remains worker owned.

Hands use host decode timestamps, while VIO uses a candidate SDK device clock.
The explicit capture tuple relates these records without equating the clocks.
Only metadata and completed hand results are retained, never source images.
"""
from collections import OrderedDict
import copy
from dataclasses import replace
import time

from .hand_world import (calibration_geometry_fingerprint, conservative_world_reference,
                         _transform_hands)
from .localization_state import consumer_map_pose
from .recovery_contract import validate_report, RecoveryProtocolError


def _uint(value):
    return type(value) is int and 0 <= value < 2**64


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def live_capture_binding(frame, source_session_id):
    """Values copied from the exact source frame, before any rectification."""
    md = frame.metadata
    return {'schema_version': 1, 'source': 'live_capture', 'source_session_id': source_session_id,
            'source_frame_id': frame.frame_id, 'hand_timestamp_ns': frame.timestamp_ns,
            'hand_timestamp_kind': md.get('timestamp_kind'),
            'sdk_sequence': md.get('sdk_sequence'),
            'sdk_device_timestamp_raw': md.get('sdk_device_timestamp_raw'),
            'transport_pairing': md.get('transport_pairing'),
            'source_type': frame.source_type,
            'pose_timestamp_basis': 'sdk_device_raw_us_to_ns_candidate',
            'exposure_synchrony_verified': False}


class LiveLocalizationHandJoin:
    """Single processing-thread owner, with ordered and exactly-once release.

    Pose callbacks enter through PipelineWorker's bounded mailbox. Unmatched
    camera results are released explicitly unavailable on timeout/capacity/stop.
    A recorder attachment is opaque and is returned with its original result;
    callers must flush before closing or pausing that recorder.
    """
    def __init__(self, calibration, *, source_session_id, max_pending=32, max_wait_s=2., clock=time.monotonic):
        if (not isinstance(source_session_id, str) or len(source_session_id) != 32
                or any(c not in '0123456789abcdef' for c in source_session_id)):
            raise ValueError('invalid live source session')
        if type(max_pending) is not int or max_pending < 1 or not 0 < max_wait_s <= 30:
            raise ValueError('invalid live join bounds')
        self.geometry = calibration_geometry_fingerprint(calibration)
        self.calibration_id = calibration.calibration_id
        self.source_session_id = source_session_id
        self.max_pending, self.max_wait_s, self.clock = max_pending, max_wait_s, clock
        self.pending, self.poses = OrderedDict(), OrderedDict()
        self.last_hand_frame = -1
        self.config_fingerprint = None
        self.stats = {'matched': 0, 'unavailable': 0, 'pose_rejected': 0, 'pose_evicted': 0}

    def _key(self, binding):
        if not isinstance(binding, dict):
            return None
        if (type(binding.get('schema_version')) is not int or binding['schema_version'] != 1
                or binding.get('source') != 'live_capture'
                or binding.get('source_session_id') != self.source_session_id
                or binding.get('source_type') != 'vendor_vs030'
                or binding.get('hand_timestamp_kind') != 'host_bundle_decode_completed_monotonic_ns'
                or binding.get('transport_pairing') != 'sdk_sequence_and_equal_device_timestamp'
                or binding.get('pose_timestamp_basis') != 'sdk_device_raw_us_to_ns_candidate'
                or binding.get('exposure_synchrony_verified') is not False):
            return None
        names = ('source_frame_id', 'hand_timestamp_ns', 'sdk_sequence', 'sdk_device_timestamp_raw')
        values = tuple(binding.get(n) for n in names)
        return values if all(_uint(v) for v in values) else None

    def _world(self, result, binding, pose=None, reason=None):
        response = {'frame_id': result.frame_id, 'timestamp_ns': result.timestamp_ns,
                    'session_id': result.session_id, 'coordinate_frame': 'vio_local_world',
                    'unit': 'm', 'status': 'unavailable', 'hands': [],
                    'world_reference': conservative_world_reference(),
                    'join': {**binding, 'source': 'live_localization',
                             'camera_geometry_sha256': self.geometry}}
        if reason is None:
            try:
                report = validate_report(pose.get('recovery_preaccept'), pose, stage='committed')
            except (AttributeError, RecoveryProtocolError):
                reason = 'recovery_binding_invalid'
            else:
                config = pose['live_config']
                response['join'].update(vio_config_fingerprint=config['fingerprint'],
                    calibration_file_sha256=config['calibration_file_sha256'],
                    recovery_session_id=report['session_id'], recovery_frame_sequence=report['frame_sequence'],
                    native_frame_id=pose.get('frame_id'))
                response['world_reference'] = conservative_world_reference(pose)
                evidence = pose.get('localization')
                if (not isinstance(evidence, dict) or type(evidence.get('contract_version')) is not int
                        or evidence['contract_version'] != 2):
                    reason = 'localization_contract_required'
                else:
                    transform = consumer_map_pose(pose, world_hands=True, expected_timestamp_ns=pose['timestamp_ns'])
                    if transform is None:
                        reason = 'localization_not_admitted'
                    else:
                        response['hands'] = _transform_hands(result, transform)
                        response.update(status='available' if any(h['valid'].any() for h in response['hands'])
                                        else 'no_valid_hand_joints',
                            pose_source='orb_slam3_committed_map', tracking_status=pose.get('tracking_state'),
                            pose_timestamp_ns=pose['timestamp_ns'], calibration_id=result.calibration_id)
        if reason is not None:
            response['reason'] = reason
        return response

    def _release(self, *, force_reason=None):
        output = []
        now = self.clock()
        while self.pending:
            key, item = next(iter(self.pending.items()))
            result, binding, attachment, offered = item
            pose = self.poses.get(key)
            reason = force_reason
            if pose is None and reason is None:
                if now - offered < self.max_wait_s:
                    break
                reason = 'localization_join_timeout'
            self.pending.pop(key)
            self.poses.pop(key, None)
            # An already paired frame is valid even when another pending frame
            # forces a flush. No result is mutated after submission to the writer.
            world = self._world(result, binding, pose, reason=None if pose is not None else reason)
            finalized = replace(result, world_hands=world,
                                diagnostics={**result.diagnostics, 'live_world_join_finalized': True})
            from .local_segments import segment_hands_for_result
            finalized.local_motion=copy.deepcopy(pose.get('local_motion')) if pose is not None else None
            finalized.segment_hands=segment_hands_for_result(finalized,pose,world)
            self.stats['matched' if world['status'] in ('available', 'no_valid_hand_joints') else 'unavailable'] += 1
            output.append((finalized, attachment))
        return output

    def add_pose(self, pose):
        binding = pose.get('live_capture') if isinstance(pose, dict) else None
        key = self._key(binding)
        config = pose.get('live_config') if isinstance(pose, dict) else None
        valid = key is not None and isinstance(config, dict)
        if valid:
            valid = (type(config.get('schema_version')) is int and config['schema_version'] == 1
                and config.get('camera_frame') == 'raw_cam0_channel4'
                and config.get('camera_geometry_sha256') == self.geometry
                and _digest(config.get('fingerprint')) and _digest(config.get('calibration_file_sha256'))
                and (self.config_fingerprint is None or config['fingerprint'] == self.config_fingerprint)
                and type(pose.get('timestamp_ns')) is int and pose['timestamp_ns'] == key[3] * 1000)
            try:
                validate_report(pose.get('recovery_preaccept'), pose, stage='committed')
            except RecoveryProtocolError:
                valid = False
        if not valid:
            self.stats['pose_rejected'] += 1
            return self._release()
        # A previously finalized hand frame cannot be rewritten by a late pose.
        if key[0] <= self.last_hand_frame and key not in self.pending:
            return self._release()
        if key in self.poses:
            # Duplicate callback cannot replace geometry already buffered.
            self.stats['pose_rejected'] += 1
            return self._release()
        self.config_fingerprint = config['fingerprint']
        self.poses[key] = copy.deepcopy(pose)
        while len(self.poses) > self.max_pending:
            self.poses.popitem(last=False)
            self.stats['pose_evicted'] += 1
        return self._release()

    def offer(self, result, frame, *, calibration, attachment=None):
        binding = live_capture_binding(frame, self.source_session_id)
        key = self._key(binding)
        reason = None
        if (key is None or type(result.frame_id) is not int or result.frame_id != frame.frame_id
                or type(result.timestamp_ns) is not int or result.timestamp_ns != frame.timestamp_ns):
            reason = 'source_frame_mismatch'
        elif result.frame_id <= self.last_hand_frame:
            reason = 'non_monotonic_join'
        elif (not isinstance(result.session_id,str) or not result.session_id
                or result.source_type != 'vendor_vs030'
                or result.coordinate_frame != 'camera_left_optical' or result.unit != 'm'
                or result.calibration_id != self.calibration_id or frame.calibration_id != self.calibration_id):
            reason = 'invalid_camera_frame_or_calibration'
        else:
            try:
                if calibration_geometry_fingerprint(calibration) != self.geometry:
                    reason = 'calibration_mismatch'
            except (ValueError, TypeError, AttributeError):
                reason = 'calibration_mismatch'
        output = self._release()
        if reason is not None:
            # Preserve acquisition order before releasing an immediate failure.
            output += self._release(force_reason='localization_join_discontinuity')
            output.append((replace(result, world_hands=self._world(result, binding, reason=reason)), attachment))
            self.stats['unavailable'] += 1
            return output
        self.last_hand_frame = result.frame_id
        if len(self.pending) >= self.max_pending:
            # Release only the oldest result; capacity does not authorize a pose.
            oldest = next(iter(self.pending))
            item = self.pending.pop(oldest)
            old_result, old_binding, old_attachment, _ = item
            self.poses.pop(oldest, None)
            output.append((replace(old_result, world_hands=self._world(old_result, old_binding,
                                  reason='localization_join_capacity')), old_attachment))
            self.stats['unavailable'] += 1
        self.pending[key] = (copy.deepcopy(result), binding, attachment, self.clock())
        return output + self._release()

    def drain(self):
        return self._release()

    def flush(self, reason='localization_join_closed', *, reset=False):
        output = self._release(force_reason=reason)
        self.poses.clear()
        if reset:
            self.config_fingerprint = None
        return output
