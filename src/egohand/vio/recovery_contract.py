"""Frame-bound recovery IPC contract; no geometry or recovery policy here."""
import json
import struct

PROTOCOL = 2
FRAME_CONTEXT = struct.Struct('<I')
RESET_IMU_ALIGNMENT = 1
MAX_MESSAGE = 100_000
IDENTITY_FIELDS = ('timestamp_ns', 'frame_id', 'map_epoch', 'world_frame_revision')
OPTIONAL_IDENTITY_FIELDS = ('reference_generation',)


def identity_fields(*frames):
    """Optional identities are bound whenever either side supplies them."""
    return (*IDENTITY_FIELDS, *(key for key in OPTIONAL_IDENTITY_FIELDS if any(key in f for f in frames)))


class RecoveryProtocolError(ConnectionError):
    """A recovery transaction cannot be associated with its pending frame."""


def send_message(sock, value):
    data = json.dumps(value, allow_nan=False).encode('utf8')
    if len(data) > MAX_MESSAGE:
        raise RecoveryProtocolError('recovery message too large')
    sock.sendall(struct.pack('<I', len(data))+data)


def negotiate_recovery(ready, *, requested=PROTOCOL, enabled=True):
    protocol=ready.get('recovery_protocol',1)
    if type(protocol) is not int or protocol not in (0,1,PROTOCOL):
        raise RecoveryProtocolError('unsupported worker recovery protocol')
    if enabled and (protocol!=requested or
                    (protocol==PROTOCOL and ready.get('recovery_owner')!='RecoveryCoordinator')):
        raise RecoveryProtocolError('worker recovery ownership does not match requested protocol')
    return protocol


def validate_report(report, frame, *, stage, session_id=None, sequence=None):
    if not isinstance(report, dict):
        raise RecoveryProtocolError('recovery report missing')
    if (type(report.get('schema_version')) is not int or report['schema_version'] != PROTOCOL
            or report.get('owner') != 'RecoveryCoordinator' or report.get('stage') != stage
            or type(report.get('await_visual_requalification')) is not bool
            or type(report.get('frame_sequence')) is not int or report['frame_sequence'] <= 0
            or not isinstance(report.get('session_id'), str) or len(report['session_id']) != 32
            or any(c not in '0123456789abcdef' for c in report['session_id'])):
        raise RecoveryProtocolError('invalid recovery report contract')
    if session_id is not None and report['session_id'] != session_id:
        raise RecoveryProtocolError('recovery session changed')
    if sequence is not None and report['frame_sequence'] != sequence:
        raise RecoveryProtocolError('recovery frame sequence mismatch')
    for name in identity_fields(report, frame):
        if name in OPTIONAL_IDENTITY_FIELDS and (name in report) != (name in frame):
            raise RecoveryProtocolError('recovery optional identity missing: '+name)
        if name not in report:
            raise RecoveryProtocolError('recovery identity field missing: '+name)
        value = report.get(name)
        expected = frame.get(name)
        if type(value) is not type(expected) or value != expected:
            raise RecoveryProtocolError('recovery frame identity mismatch: '+name)
        # Missing legacy native frame/world fields stay explicitly unknown.
        if value is not None and (type(value) is not int or not 0<=value<2**64):
            raise RecoveryProtocolError('invalid recovery identity: '+name)
    if type(report.get('timestamp_ns')) is not int or type(report.get('map_epoch')) is not int:
        raise RecoveryProtocolError('recovery timestamp or map identity missing')
    return report


def application_feedback(result):
    report = validate_report(result.get('recovery_preaccept'), result, stage='native')
    quality = result.get('quality_gate')
    if not isinstance(quality, dict) or type(quality.get('passed')) is not bool:
        raise RecoveryProtocolError('application quality decision missing')
    return {'type': 'application_admission', 'schema_version': PROTOCOL,
        'session_id': report['session_id'], 'frame_sequence': report['frame_sequence'],
        **{name: result.get(name) for name in identity_fields(result)},
        'quality_passed': quality['passed'], 'jump_detected': result.get('jump_detected') is True,
        'pose_admitted': result.get('pose_valid') is True,
        'inertial_anchor_admitted': (result.get('pose_valid') is True and
            ((result.get('mode') == 'stereo_inertial' and result.get('pose_source') != 'stereo_bootstrap') or
             (result.get('mode') == 'hybrid' and result.get('pose_source') == 'vio'
              and result.get('inertial_display_ready') is True)))}


def validate_feedback(feedback, pending):
    if not isinstance(feedback, dict) or feedback.get('type') != 'application_admission':
        raise RecoveryProtocolError('application admission acknowledgement missing')
    if type(feedback.get('schema_version')) is not int or feedback['schema_version'] != PROTOCOL:
        raise RecoveryProtocolError('unknown application admission contract')
    for name in ('session_id', 'frame_sequence', *identity_fields(feedback, pending)):
        if name in OPTIONAL_IDENTITY_FIELDS and (name in feedback) != (name in pending):
            raise RecoveryProtocolError('application optional identity missing: '+name)
        if name not in feedback:raise RecoveryProtocolError('application acknowledgement field missing: '+name)
        if type(feedback.get(name)) is not type(pending.get(name)) or feedback.get(name) != pending.get(name):
            raise RecoveryProtocolError('application acknowledgement mismatch: '+name)
    for name in ('quality_passed', 'jump_detected', 'pose_admitted', 'inertial_anchor_admitted'):
        if type(feedback.get(name)) is not bool:
            raise RecoveryProtocolError('invalid application admission flag: '+name)
    if feedback['pose_admitted'] and (not feedback['quality_passed'] or feedback['jump_detected']):
        raise RecoveryProtocolError('contradictory application admission')
    if feedback['inertial_anchor_admitted'] and not feedback['pose_admitted']:
        raise RecoveryProtocolError('anchor without admitted pose')
    return feedback
