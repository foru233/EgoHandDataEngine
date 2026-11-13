"""Read-only Atlas identity checks for the current native epoch encoding."""


def world_identity(result):
    """Keep legacy epochs compatible; include native world revisions when known."""
    epoch = result.get('map_epoch')
    revision = result.get('world_frame_revision')
    if 'reference_generation' in result:
        return (epoch, revision, result['reference_generation'])
    return (epoch, revision) if revision is not None else epoch


def invalid_reference_frame(previous_generation, result, previous_epoch=None):
    """Validate the optional new-reference identity, never map rigidity or recovery.

    A legacy stream with no reference extension remains supported. Once a
    reference is known, dropping it or regressing within one map fails closed.
    """
    if 'reference_generation' not in result and 'reference_frame' not in result:
        return previous_generation is not None
    generation = result.get('reference_generation');report = result.get('reference_frame')
    if (type(generation) is not int or not 0 <= generation < 2**64 or not isinstance(report, dict)
            or type(report.get('schema_version')) is not int or report['schema_version'] != 1
            or type(report.get('generation')) is not int or report.get('frame_bound') is not True
            or report.get('snapshot_consistent') is not True or report.get('generation') != generation
            or report.get('basis') != 'successful_initializer_commit'
            or report.get('coordinate_continuity_verified') is not False
            or not snapshot_identity_matches_epoch(result)):
        return True
    for key in ('frame_id', 'map_id', 'reset_id', 'map_revision', 'map_change_index', 'timestamp_ns'):
        if type(report.get(key)) is not int or type(result.get(key)) is not int or report[key] != result[key]:
            return True
    for key in ('operation_sequence', 'atlas_generation', 'geometry_sequence'):
        if type(report.get(key)) is not int or not 0 <= report[key] < 2**64:return True
    if (generation == 0) != (report['operation_sequence'] == 0):return True
    if previous_generation is not None and previous_epoch is not None:
        try:
            old, new = decode_map_epoch(previous_epoch), decode_map_epoch(result['map_epoch'])
        except (ValueError, TypeError, OverflowError):return True
        if (old['map_id'], old['reset_id']) == (new['map_id'], new['reset_id']) and generation < previous_generation:
            return True
    return False


def decode_map_epoch(epoch):
    """Mirror System::GetMapEpoch: map24 | reset20 | big_change20."""
    value = int(epoch)
    if value < 0 or value >= 1 << 64:
        raise ValueError("地图版本不是有效的 uint64")
    return {"map_id": value >> 40, "reset_id": (value >> 20) & ((1 << 20) - 1),
            "map_revision": value & ((1 << 20) - 1)}


def snapshot_identity_matches_epoch(result):
    """Check explicit same-frame identity fields; not proof of atomic reset generation."""
    if result.get('snapshot_available') is not True or result.get('snapshot_consistent') is not True:
        return False
    epoch = result.get('map_epoch')
    if isinstance(epoch, bool) or not isinstance(epoch, int):
        return False
    try:
        identity = decode_map_epoch(epoch)
        if any(isinstance(result.get(k), bool) or not isinstance(result.get(k), int)
               or result[k] != value for k, value in identity.items()):
            return False
        frame = result.get('frame_id')
        stamp = result.get('timestamp_ns')
        return (isinstance(frame, int) and not isinstance(frame, bool) and frame >= 0
                and isinstance(stamp, int) and not isinstance(stamp, bool) and stamp >= 0
                and type(result.get('world_frame_id')) is int and result['world_frame_id'] == epoch)
    except (ValueError, TypeError, OverflowError):
        return False


def invalid_world_revision_transition(previous, current):
    """Reject malformed revisions and losing an already-known revision.

    Legacy streams with neither value remain unknown, not verified.
    """
    if current is None:
        return previous is not None
    return type(current) is not int or not 0 <= current < (1 << 64)


def snapshot_body_matches_frame(result, body):
    """Validate body/camera association, without certifying map transactions.

    BA readiness is a caller policy, separate from this shared geometry and
    identity check. Legacy independently queried body states fail closed.
    """
    import numpy as np
    from .trajectory import rigid_transform

    if not isinstance(body, dict) or not snapshot_identity_matches_epoch(result):
        return False
    if body.get('snapshot_consistent') is not True:
        return False
    if any(type(body.get(key)) is not int or body[key] != expected for key, expected in (
        ('timestamp_ns', result['timestamp_ns']), ('frame_id', result['frame_id']),
        ('world_frame_id', result['map_epoch']),
    )):
        return False
    try:
        rigid_transform(body.get('T_world_from_body'))
        for name in ('velocity_world_mps', 'accel_bias_mps2', 'gyro_bias_rps'):
            vector = np.asarray(body.get(name), dtype=float)
            if vector.shape != (3,) or not np.isfinite(vector).all():
                return False
    except (TypeError, ValueError, OverflowError):
        return False
    return True


class PresetMapIdentity:
    """Pin the first located world revision; never join a later world implicitly."""

    def __init__(self, map_id):
        self.map_id = int(map_id)
        self.located_epoch = None

    def rejection(self, result):
        if result.get("snapshot_consistent") is False:
            return "定位状态跨地图版本，地图叠加已暂停"
        try:
            epoch = int(result["map_epoch"])
            identity = decode_map_epoch(epoch)
        except (KeyError, TypeError, ValueError):
            return "缺少有效地图身份，地图叠加已暂停"
        if identity["map_id"] != self.map_id:
            return "当前位姿未定位到所选 Atlas 地图"
        world = world_identity(result)
        if self.located_epoch is not None and world != self.located_epoch:
            return "Atlas 世界坐标版本发生变化，地图叠加已暂停，请重新载入地图"
        if result.get("pose_valid") and result.get("tracking_state") == "OK":
            self.located_epoch = world
        return None
