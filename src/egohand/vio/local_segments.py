"""Segment admitted Map poses for local recording output; never release a gate.

This is a coordinate adapter owned by LocalizationPipeline, not an estimator or
independent Odom. Unchanged reported reference identities are runtime evidence,
not a certificate that all native map writers are atomic or the poses accurate.
"""
import numpy as np

from .trajectory import rigid_transform


class LocalTrajectorySegments:
    def __init__(self):
        self.index = -1
        self.anchor = None
        self.identity = None
        self.last_frame = None
        self.last_sequence = None
        self.session_id = None
        self.pending_reason = 'first_admitted_pose'

    def process(self, pose):
        from .localization_state import consumer_map_pose
        from .recovery_contract import validate_report, RecoveryProtocolError

        out = {'schema_version':1, 'status':'unavailable', 'session_id':None,
            'segment_id':None, 'timestamp_ns':pose.get('timestamp_ns'), 'unit':'m',
            'camera_frame':'camera_left_optical', 'pose_valid':False,
            'pose_source':pose.get('pose_source') or pose.get('mode'),
            'T_segment_from_camera':None, 'T_session_from_camera':None,
            'T_session_from_segment':None, 'T_segment_from_map':None,
            'reference_continuity_status':'unavailable',
            'coordinate_continuity_verified':False,
            'independent_odom':False, 'cross_segment_merge_permitted':False,
            'quality_status':pose.get('quality_status'),
            'quality_gate':pose.get('quality_gate'),
            'map_id':pose.get('map_id'), 'reset_id':pose.get('reset_id'),
            'map_revision':pose.get('map_revision'),
            'world_frame_revision':pose.get('world_frame_revision'),
            'reference_generation':pose.get('reference_generation')}
        reason = 'localization_not_admitted'
        transform = consumer_map_pose(pose)
        try:
            report = validate_report(pose.get('recovery_preaccept'),pose,stage='committed')
        except RecoveryProtocolError:
            report = None
            reason = 'committed_recovery_contract_required'
        reference = pose.get('reference_frame') or {}
        atlas = reference.get('atlas_generation')
        revision = pose.get('world_frame_revision')
        if type(revision) is not int or not 0 <= revision < 2**64:
            transform = None
            reason = 'world_revision_required'
        if report is None or transform is None:
            self.anchor = None
            self.pending_reason = 'admission_gap'
            out['reason'] = reason
            return out
        session = report['session_id']
        identity = (session, pose['map_epoch'],revision,
                    pose.get('reference_generation'),atlas,out['pose_source'])
        frame = pose['frame_id']; sequence = report['frame_sequence']
        contiguous = (self.last_frame is not None and frame == self.last_frame+1
                      and sequence == self.last_sequence+1)
        if self.session_id is not None and session != self.session_id:
            self.index = -1
            self.anchor = None
            self.pending_reason = 'localization_session_changed'
        if self.anchor is None or identity != self.identity or not contiguous:
            reason = (self.pending_reason if self.anchor is None else
                      'reported_reference_or_source_changed' if identity != self.identity else
                      'frame_sequence_gap')
            self.index += 1
            self.anchor = np.linalg.inv(transform)
            self.identity = identity
            self.anchor_stamp = pose['timestamp_ns']
        else:
            reason = None
        self.session_id = session
        self.last_frame, self.last_sequence = frame, sequence
        local = self.anchor @ transform
        # Session coordinates are defined by the FIRST segment only. Other
        # segments have no proven relationship to it and remain null.
        session_from_segment = np.eye(4) if self.index == 0 else None
        out.update(status='estimated', session_id=session,
            segment_id=f'{session}:{self.index}', segment_index=self.index,
            pose_valid=True, segment_start_reason=reason,
            segment_origin_timestamp_ns=self.anchor_stamp,
            T_segment_from_map=self.anchor.tolist(),
            T_segment_from_camera=local.tolist(),
            T_session_from_camera=local.tolist() if self.index == 0 else None,
            T_session_from_segment=session_from_segment.tolist() if self.index == 0 else None,
            reference_continuity_status='estimated_within_unchanged_reported_reference',
            uncertainty='native_map_writer_coverage_and_absolute_accuracy_unverified',
            reason=None)
        return out


def consumer_segment_pose(pose, *, expected_timestamp_ns=None, world_hands=False):
    """Validate additive local coordinates against the admitted Map transform."""
    from .localization_state import consumer_map_pose
    transform = consumer_map_pose(pose, expected_timestamp_ns=expected_timestamp_ns,world_hands=world_hands)
    motion = pose.get('local_motion') if isinstance(pose,dict) else None
    if transform is None or not isinstance(motion,dict):
        return None
    if (type(motion.get('schema_version')) is not int or motion['schema_version'] != 1
            or motion.get('status') != 'estimated' or motion.get('pose_valid') is not True
            or motion.get('timestamp_ns') != pose.get('timestamp_ns')
            or motion.get('unit') != 'm' or motion.get('camera_frame') != 'camera_left_optical'
            or not isinstance(motion.get('segment_id'),str)):
        return None
    report = pose.get('recovery_preaccept') or {}
    if motion.get('session_id') != report.get('session_id'):
        return None
    for key in ('map_id','reset_id','map_revision','world_frame_revision','reference_generation'):
        if motion.get(key) != pose.get(key):
            return None
    try:
        local = rigid_transform(motion.get('T_segment_from_camera'))
        anchor = rigid_transform(motion.get('T_segment_from_map'))
        if not np.allclose(local, anchor @ transform, rtol=0,atol=1e-8):
            return None
        return local.copy()
    except (TypeError,ValueError,OverflowError):
        return None


def consumer_display_pose(pose, *, world_hands=False, expected_timestamp_ns=None):
    """Local output is authoritative when present; retain legacy Map viewers."""
    if isinstance(pose,dict) and 'local_motion' in pose and pose.get('preset_map_id') is None:
        return consumer_segment_pose(pose,expected_timestamp_ns=expected_timestamp_ns,world_hands=world_hands)
    from .localization_state import consumer_map_pose
    return consumer_map_pose(pose,world_hands=world_hands,expected_timestamp_ns=expected_timestamp_ns)


def display_reference_key(pose):
    if 'local_motion' in pose and pose.get('preset_map_id') is None:
        motion=pose.get('local_motion') or {}
        return ('segment',motion.get('session_id'),motion.get('segment_id'))
    from .map_identity import world_identity
    return world_identity(pose)


def segment_hands_for_result(result, pose, world):
    """Use an already exact-frame, calibration-checked world join as provenance."""
    from .hand_world import _transform_hands
    out = {'schema_version':1,'status':'unavailable','reason':'exact_frame_join_required',
           'frame_id':result.frame_id,'timestamp_ns':result.timestamp_ns,
           'result_session_id':result.session_id,'unit':'m','hands':[],
           'coordinate_frame':'segment','segment_id':None,'cross_segment_merge_permitted':False}
    if not isinstance(world,dict) or world.get('status') not in ('available','no_valid_hand_joints'):
        return out
    from .hand_pose_binding import result_pose_timestamp
    stamp = result_pose_timestamp(result, pose, world)
    if stamp is None:
        out['reason']='exact_frame_join_required'
        return out
    local = consumer_segment_pose(pose,expected_timestamp_ns=stamp,world_hands=True)
    if local is None:
        out['reason'] = 'local_segment_unavailable'
        return out
    out.update(status=world['status'],reason=None,segment_id=pose['local_motion']['segment_id'],
        localization_session_id=pose['local_motion']['session_id'],
        pose_timestamp_ns=stamp,calibration_id=result.calibration_id,
        reference_continuity_status=pose['local_motion']['reference_continuity_status'],
        coordinate_continuity_verified=False,join=world['join'])
    out['hands'] = [{**{k:v for k,v in h.items() if k!='xyz_world_m'},
                     'xyz_segment_m':h['xyz_world_m']} for h in _transform_hands(result,local)]
    return out
