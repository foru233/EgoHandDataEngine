"""Conservative per-frame localization evidence; not an independent odometer.

Transforms use T_A_from_B: p_A = T_A_from_B @ p_B.  No independent
front-end odometry or complete native map-event journal is currently exposed.
Accordingly odom and map-from-odom remain explicitly unavailable.
"""
import numpy as np

from .trajectory import rigid_transform
from .map_identity import (snapshot_body_matches_frame, snapshot_identity_matches_epoch,
                           invalid_world_revision_transition, invalid_reference_frame)
from .pose_quality import recovery_quarantine_required
from .recovery_contract import (PROTOCOL, RecoveryProtocolError, validate_report)


def consumer_map_pose(result, *, world_hands=False, expected_timestamp_ns=None):
    """Read admitted Map coordinates, with an adapter for pre-contract payloads.

    A present v2 contract is authoritative; malformed/unknown contracts never
    fall back to a legacy valid bit. This validates transport coherence, not
    pose accuracy, reference continuity, or permission to merge frames.
    """
    if not isinstance(result, dict) or result.get('pose_valid') is not True:
        return None
    if expected_timestamp_ns is not None and (
            type(expected_timestamp_ns) is not int
            or type(result.get('timestamp_ns')) is not int
            or result['timestamp_ns'] != expected_timestamp_ns):
        return None
    if (recovery_quarantine_required(result) or result.get('snapshot_consistent') is False
            or invalid_reference_frame(None,result)):
        return None
    report=result.get('recovery_preaccept')
    if isinstance(report,dict) and 'schema_version' in report:
        try:validate_report(report,result,stage='committed')
        except RecoveryProtocolError:return None
    try:
        legacy = rigid_transform(result.get('matrix'))
        if 'localization' not in result:
            return legacy.copy()
        evidence = result['localization']
        if not isinstance(evidence, dict):
            return None
        if (type(evidence.get('schema_version')) is not int or evidence['schema_version'] != 1
                or type(evidence.get('contract_version')) is not int or evidence['contract_version'] != 2):
            return None
        admission = evidence.get('admission')
        lifecycle = evidence.get('lifecycle')
        map_state = evidence.get('map')
        if not all(isinstance(value, dict) for value in (admission, lifecycle, map_state)):
            return None
        if (admission.get('map_pose_available') is not True
                or admission.get('legacy_pose_available') is not True
                or (world_hands and admission.get('world_hands_permitted') is not True)
                or evidence.get('quarantined') is not False
                or evidence.get('native_snapshot_consistent') is not True
                or lifecycle.get('timestamp_monotonic') is not True
                or map_state.get('status') != 'estimated'
                or map_state.get('camera_frame') != 'gui_left_camera'):
            return None
        for field in ('timestamp_ns', 'map_epoch', 'world_frame_revision', 'reference_generation'):
            if (type(evidence.get(field)) is not type(result.get(field))
                    or evidence.get(field) != result.get(field)):
                return None
        stamp = evidence.get('timestamp_ns'); epoch = evidence.get('map_epoch')
        if type(stamp) is not int or stamp < 0 or type(epoch) is not int or not 0 <= epoch < 2**64:
            return None
        revision = evidence.get('world_frame_revision')
        if revision is not None and (type(revision) is not int or not 0 <= revision < 2**64):
            return None
        transform = rigid_transform(map_state.get('T_map_from_camera'))
        if not np.array_equal(transform, legacy):
            return None
        return transform.copy()
    except (TypeError, ValueError, OverflowError):
        return None


def recovery_evidence(result):
    """Describe the two recovery sources without changing their decisions.

    Unknown inputs stay unknown.  A disagreement means the two independent
    sources report different booleans; it does not prove either is wrong.
    This is advisory evidence, not a visual verification or release signal.
    """
    preaccept = result.get('recovery_preaccept')
    preaccept = preaccept if isinstance(preaccept, dict) else {}
    canonical = preaccept.get('schema_version') == PROTOCOL
    if canonical:
        try:
            validate_report(preaccept,result,stage='committed')
        except RecoveryProtocolError:
            return {'schema_version':2,'status':'unknown','owner':'RecoveryCoordinator',
                    'reason':'recovery_contract_invalid','advisory_only':True}

    def boolean_or_none(value):
        return value if isinstance(value, bool) else None

    coordinator = boolean_or_none(preaccept.get('await_visual_requalification'))
    monitor = boolean_or_none(result.get('recovery_monitor_awaiting_visual_requalification'))
    if canonical:
        combined = 'quarantined' if coordinator else 'not_quarantined_reported'
    elif coordinator is True or monitor is True:
        combined = 'quarantined'
    elif coordinator is False and monitor is False:
        combined = 'not_quarantined_reported'
    else:
        combined = 'unknown'
    elapsed = preaccept.get('quarantine_elapsed_s')
    try:
        valid_elapsed = (not isinstance(elapsed, bool) and isinstance(elapsed, (int, float))
                         and np.isfinite(elapsed) and elapsed >= 0)
    except (TypeError, ValueError, OverflowError):
        valid_elapsed = False
    if not valid_elapsed:
        elapsed = None

    def string_or_none(value):
        return value if isinstance(value, str) and value else None

    return {
        'schema_version': 2 if canonical else 1,
        'owner': 'RecoveryCoordinator' if canonical else 'legacy_multiple_sources',
        'frame_committed': canonical,
        'status': combined,
        'source_disagreement': (coordinator != monitor
                                if coordinator is not None and monitor is not None else None),
        'coordinator': {
            'await_visual_requalification': coordinator,
            'cause': string_or_none(preaccept.get('quarantine_cause')),
            'evidence_status': string_or_none(preaccept.get('evidence_status')),
            'elapsed_s': elapsed,
            'timed_out': boolean_or_none(preaccept.get('quarantine_timed_out')),
            'timeout_action': string_or_none(preaccept.get('timeout_action')),
            'anchor_available': boolean_or_none(preaccept.get('anchor_available')),
            'request_new_map': boolean_or_none(preaccept.get('request_new_map')),
        },
        'monitor': {'await_visual_requalification': monitor,
                    'active':False if canonical else True},
        'independent_visual_verification': 'not_assessed',
        'advisory_only': True,
    }


def _body_snapshot_consistent(result):
    body = result.get('body_state_candidate')
    return (snapshot_body_matches_frame(result, body)
            and body.get('ba2_complete') is True)


def localization_evidence(result):
    """Describe one already-gated camera frame without changing its pose.

    A native frame snapshot provides coherence, not an accuracy guarantee.
    The returned state is an advisory view of current evidence, not a new
    recovery policy; missing inputs are never converted into success.
    """
    tracking = result.get('tracking_state')
    quarantine = recovery_quarantine_required(result)
    snapshot = (snapshot_identity_matches_epoch(result)
                and not invalid_world_revision_transition(None, result.get('world_frame_revision'))
                and not invalid_reference_frame(None,result))
    raw_ok = bool(result.get('raw_pose_valid', result.get('pose_valid'))) and tracking == 'OK'
    if raw_ok:
        try:
            rigid_transform(result.get('raw_matrix', result.get('matrix')))
        except (TypeError, ValueError, OverflowError):
            raw_ok = False
    map_pose = None
    stamp=result.get('timestamp_ns'); epoch=result.get('map_epoch')
    identity_valid=(isinstance(stamp,int) and not isinstance(stamp,bool) and stamp>=0
                    and isinstance(epoch,int) and not isinstance(epoch,bool) and 0<=epoch<2**64)
    if (result.get('pose_valid') and raw_ok and not quarantine and snapshot and identity_valid
            and result.get('jump_detected') is not True
            and result.get('display_map_coordinates_preserved') is not False):
        try:
            map_pose = rigid_transform(result.get('matrix')).tolist()
        except (TypeError, ValueError, OverflowError):
            pass
    imu_initialized = result.get('imu_initialized') is True
    inertial_refined = (imu_initialized and result.get('ba2_complete') is True
                        and result.get('inertial_scale_finalized') is True)
    body_consistent = snapshot and _body_snapshot_consistent(result)
    if quarantine:
        state = 'RELOCALIZING'
    elif tracking == 'LOST':
        state = 'LOST'
    elif tracking == 'RECENTLY_LOST':
        state = 'RELOCALIZING'
    elif tracking in ('NOT_INITIALIZED', 'NO_IMAGES_YET'):
        state = 'VISUAL_INITIALIZING'
    elif not raw_ok or not snapshot or result.get('jump_detected') is True:
        state = 'DEGRADED'
    elif result.get('mode') == 'stereo':
        state = 'VISUAL_TRACKING' if map_pose is not None else 'DEGRADED'
    elif result.get('mode') == 'stereo_inertial' and result.get('pose_source') == 'stereo_bootstrap':
        state = 'VISUAL_TRACKING' if map_pose is not None else 'INERTIAL_INITIALIZING'
    elif result.get('mode') == 'hybrid' and result.get('pose_source') == 'visual_fallback':
        state = 'VISUAL_TRACKING' if map_pose is not None else 'DEGRADED'
    elif (result.get('mode') not in ('stereo_inertial', 'hybrid')
          or (result.get('mode') == 'hybrid' and
              (result.get('pose_source') != 'vio' or result.get('inertial_display_ready') is not True))):
        state = 'DEGRADED'
    # Inertial initialization/refinement describes the native estimator phase.
    # A display gate can suppress the pose while the underlying tracking is OK.
    # Do not mislabel that normal waiting period as a sensor/estimator fault.
    elif not imu_initialized:
        state = 'INERTIAL_INITIALIZING'
    elif not inertial_refined:
        state = 'INERTIAL_REFINING'
    elif map_pose is None or not body_consistent:
        state = 'DEGRADED'
    else:
        state = 'VIO_TRACKING'
    prediction = (result.get('recovery_preaccept') or {}).get('loss_prediction') or {}
    predicted = (prediction.get('status') == 'predicted'
                 and prediction.get('mode') == 'imu_only_extrapolation_not_visual_pose')
    return {
        'schema_version': 1,
        'state': state,
        'timestamp_ns': result.get('timestamp_ns'),
        'tracking_state': tracking,
        'raw_tracking_pose_available': raw_ok,
        'pose_output_available': map_pose is not None,
        'pose_gate_status': result.get('quality_status'),
        'native_snapshot_consistent': snapshot,
        # Reset request checks exist in a candidate; deployed/complete native
        # map/atlas reset capability has not yet been certified.
        'reset_generation_verified': False,
        'map_epoch': result.get('map_epoch'),
        'world_frame_revision': result.get('world_frame_revision'),
        'reference_generation': result.get('reference_generation'),
        'reference_frame': result.get('reference_frame'),
        'map': {'status': 'estimated' if map_pose is not None else 'unavailable',
                'T_map_from_camera': map_pose,
                'camera_frame': 'gui_left_camera',
                'reason': None if map_pose is not None else 'gated_or_snapshot_unverified'},
        'odom': {'status': 'unavailable', 'T_odom_from_camera': None,
                 'segment_id': None, 'reason': 'independent_frontend_state_unavailable'},
        'T_map_from_odom': None,
        'imu_initialized': imu_initialized,
        'inertial_refinement_reported': inertial_refined,
        'body_snapshot_consistent': body_consistent,
        'prediction': {'status': 'predicted' if predicted else 'unavailable',
                       'source': 'imu_only_not_visual_pose' if predicted else None},
        'quarantined': quarantine,
        # This additive evidence view does not participate in admission and
        # cannot prove requalification or a continuous world reference.
        'recovery_evidence': recovery_evidence(result),
        'coordinate_continuity_verified': False,
    }


class LocalizationPipeline:
    """One application admission path for live and recorded camera results.

    Native pre-acceptance remains in the worker. This adapter owns the existing
    application gate, revision audit and lifecycle journal. Protocol v2 commits
    quality evidence to the worker's sole recovery coordinator; older protocols
    retain an explicitly labelled diagnostic monitor for compatibility.
    it does not authorize same-map recovery or independent odometry. Optional
    display continuity is recorded separately from the map pose contract.
    """

    def __init__(self, mode, quality, *, continuity=None, recovery_commit=None):
        from .modes import validate_mode
        from .recovery_monitor import RecoveryMonitor
        from .map_revision_audit import MapRevisionAudit

        self.mode = validate_mode(mode)
        self.quality = quality
        self.continuity = continuity
        self._recovery_commit = recovery_commit
        self.recovery_monitor = RecoveryMonitor() if recovery_commit is None else None
        self._recovery_session = None
        self._recovery_sequence = 0
        self._imu_reset_pending = False
        self.revision_audit = MapRevisionAudit()
        self._state = 'WAITING_FOR_SENSORS'
        self._entered_ns = None
        self._transition_index = 0
        self._last_timestamp_ns = None
        from .local_segments import LocalTrajectorySegments
        self.local_segments = LocalTrajectorySegments()

    def reset_imu_alignment(self):
        """Discard prediction samples without erasing outstanding quarantine.

        Resetting a sensor buffer is not geometric requalification. A pending
        loss also has to remain unresolved after its IMU history is discarded.
        """
        if self.recovery_monitor is not None:
            self.recovery_monitor.reset_imu_alignment()
        else:
            self._imu_reset_pending = True

    def take_imu_alignment_reset(self):
        pending=self._imu_reset_pending
        self._imu_reset_pending=False
        return pending

    def process(self, result, measurements):
        from .pose_quality import apply_recovery_quarantine
        from .recovery_monitor import observe_recovery

        out = dict(result)
        out['mode'] = self.mode
        stamp = out.get('timestamp_ns')
        monotonic = (type(stamp) is int and stamp >= 0
                     and (self._last_timestamp_ns is None or stamp > self._last_timestamp_ns))
        if not monotonic:
            if self._recovery_commit is not None:
                raise RecoveryProtocolError('nonmonotonic frame in recovery transaction')
            # Reject before touching quality windows, recovery anchors or map
            # history; a stale response cannot replace the newest evidence.
            out.update(raw_pose_valid=bool(out.get('pose_valid')), raw_matrix=out.get('matrix'),
                       pose_valid=False, matrix=None, jump_detected=False,
                       quality_status='位姿时间非递增或无效')
            out['quality_gate'] = {'schema_version': 1, 'gate_id': 'VIO.TIMESTAMP',
                'passed': False, 'reason_code': 'timestamp_invalid_or_nonmonotonic',
                'owner': 'LocalizationPipeline', 'effect': 'suppress_pose'}
            out['recovery_monitor_awaiting_visual_requalification'] = (
                self.recovery_monitor.await_visual_requalification)
            return self._publish(out, monotonic=False)
        if self._recovery_commit is not None:
            native=validate_report(out.get('recovery_preaccept'),out,stage='native',
                session_id=self._recovery_session,sequence=self._recovery_sequence+1)
        elif isinstance(out.get('recovery_preaccept'),dict) and 'schema_version' in out['recovery_preaccept']:
            raise RecoveryProtocolError('committed recovery requires its transaction transport')
        out = self.quality.check(out)
        out['mode'] = self.mode
        # Previously the quarantine was applied before and after observation.
        # Excluding quarantined anchors preserves that ordering with one final
        # admission application. Raw tracking is still observed during loss.
        if self._recovery_commit is not None:
            committed=validate_report(self._recovery_commit(out),out,stage='committed',
                session_id=native['session_id'],sequence=native['frame_sequence'])
            if (committed.get('application_quality_passed') is not out['quality_gate']['passed']
                    or committed.get('application_pose_admitted') is not
                       (out.get('pose_valid') is True and not committed['await_visual_requalification'])):
                raise RecoveryProtocolError('worker commit contradicts application admission')
            self._recovery_session=native['session_id'];self._recovery_sequence=native['frame_sequence']
            out['recovery_preaccept']=committed
            if committed.get('direct_recovery') is not None:
                out['recovery_check']=committed['direct_recovery']
            out['recovery_monitor_awaiting_visual_requalification']=None
            out['recovery_ownership']={'schema_version':1,'owner':'RecoveryCoordinator',
                'protocol_version':PROTOCOL,'session_id':self._recovery_session,
                'frame_sequence':self._recovery_sequence,'monitor_active':False}
        else:
            recovery = observe_recovery(self.recovery_monitor, out, measurements,
                                       allow_anchor=not recovery_quarantine_required(out))
            if recovery is not None:
                out['recovery_check'] = recovery
            out['recovery_monitor_awaiting_visual_requalification'] = (
                self.recovery_monitor.await_visual_requalification)
            out['recovery_ownership']={'schema_version':1,'owner':'legacy_multiple_sources',
                                      'protocol_version':1,'monitor_active':True}
        out = apply_recovery_quarantine(out)
        if self.continuity is not None:
            out = self.continuity.process(out)
        revision = self.revision_audit.observe(out)
        if revision is not None:
            out['map_revision_decision'] = revision
        return self._publish(out, monotonic=True)

    def _publish(self, out, *, monotonic):
        evidence = localization_evidence(out)
        evidence['contract_version'] = 2
        stamp = out.get('timestamp_ns')
        valid_time = type(stamp) is int and stamp >= 0
        transition = None
        state = evidence['state'] if monotonic else 'DEGRADED'
        if state != self._state:
            self._transition_index += 1
            transition = {'from': self._state, 'to': state,
                          'timestamp_ns': stamp if valid_time else None,
                          'map_epoch': out.get('map_epoch'),
                          'world_frame_revision': out.get('world_frame_revision'),
                          'reason_code': (out.get('recovery_gate') or
                                          out.get('quality_gate') or {}).get('reason_code')}
            self._state = state
            self._entered_ns = stamp if monotonic else None
        if monotonic:
            self._last_timestamp_ns = stamp
        evidence['state'] = state
        evidence['lifecycle'] = {'schema_version': 1, 'owner': 'LocalizationPipeline',
            'state': state, 'entered_timestamp_ns': self._entered_ns,
            'transition_index': self._transition_index, 'transition': transition,
            'timestamp_monotonic': monotonic,
            'native_preacceptance_owner': 'RecoveryCoordinator'}
        evidence['admission'] = {
            'legacy_pose_available': bool(out.get('pose_valid')),
            'map_pose_available': evidence['pose_output_available'],
            'world_hands_permitted': evidence['pose_output_available'],
            'cross_frame_merge_permitted': False,
            'odom_available': False,
            'quality_gate': out.get('quality_gate'),
            'recovery_gate': out.get('recovery_gate'),
        }
        out['localization'] = evidence
        if self._recovery_commit is not None:
            out['local_motion'] = self.local_segments.process(out)
        return out
