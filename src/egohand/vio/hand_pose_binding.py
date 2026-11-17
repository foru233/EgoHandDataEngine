"""Resolve a hand result's exact pose timestamp without equating sensor clocks."""


def _uint(value):
    return type(value) is int and 0 <= value < 2**64


def result_pose_timestamp(result, pose, world):
    """An existing exact live join relates host time to a committed sensor pose.

    This consumes the producer's binding; it neither finds nearest poses nor
    changes localization admission. Recorded same-clock callers stay unchanged.
    """
    if not isinstance(pose, dict) or not isinstance(world, dict):
        return None
    stamp = world.get('pose_timestamp_ns')
    if not _uint(stamp) or stamp != pose.get('timestamp_ns'):
        return None
    capture = pose.get('live_capture')
    if capture is None:
        return stamp if stamp == result.timestamp_ns else None
    if not isinstance(capture, dict):
        return None
    join = world.get('join')
    config = pose.get('live_config')
    recovery = pose.get('recovery_preaccept')
    reference = world.get('world_reference')
    if not all(isinstance(v, dict) for v in (join, config, recovery, reference)):
        return None
    if (type(capture.get('schema_version')) is not int or capture['schema_version'] != 1
            or not _uint(result.frame_id) or not _uint(result.timestamp_ns)
            or capture.get('source') != 'live_capture'
            or join.get('source') != 'live_localization'
            or capture.get('source_type') != 'vendor_vs030' or result.source_type != 'vendor_vs030'
            or result.coordinate_frame != 'camera_left_optical' or result.unit != 'm'
            or capture.get('hand_timestamp_kind') != 'host_bundle_decode_completed_monotonic_ns'
            or capture.get('transport_pairing') != 'sdk_sequence_and_equal_device_timestamp'
            or capture.get('pose_timestamp_basis') != 'sdk_device_raw_us_to_ns_candidate'
            or capture.get('exposure_synchrony_verified') is not False):
        return None
    fields = ('source_session_id', 'source_frame_id', 'hand_timestamp_ns', 'sdk_sequence',
              'sdk_device_timestamp_raw', 'hand_timestamp_kind', 'transport_pairing',
              'pose_timestamp_basis', 'exposure_synchrony_verified', 'source_type')
    if any(join.get(k) != capture.get(k) for k in fields):
        return None
    for key in ('source_frame_id', 'hand_timestamp_ns', 'sdk_sequence', 'sdk_device_timestamp_raw'):
        if not _uint(capture.get(key)):
            return None
    session = capture.get('source_session_id')
    if not isinstance(session, str) or len(session) != 32 or any(c not in '0123456789abcdef' for c in session):
        return None
    for key in ('camera_geometry_sha256', 'fingerprint', 'calibration_file_sha256'):
        value = config.get(key)
        if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
            return None
    if (capture['source_frame_id'] != result.frame_id or capture['hand_timestamp_ns'] != result.timestamp_ns
            or stamp != capture['sdk_device_timestamp_raw'] * 1000
            or world.get('frame_id') != result.frame_id or world.get('timestamp_ns') != result.timestamp_ns
            or world.get('calibration_id') != result.calibration_id
            or world.get('pose_source') != 'orb_slam3_committed_map'
            or config.get('camera_frame') != 'raw_cam0_channel4'
            or join.get('camera_geometry_sha256') != config.get('camera_geometry_sha256')
            or join.get('vio_config_fingerprint') != config.get('fingerprint')
            or join.get('calibration_file_sha256') != config.get('calibration_file_sha256')
            or join.get('recovery_session_id') != recovery.get('session_id')
            or join.get('recovery_frame_sequence') != recovery.get('frame_sequence')
            or reference.get('cross_frame_merge_permitted') is not False):
        return None
    if any(reference.get(k) != pose.get(k) for k in ('map_epoch', 'world_frame_revision', 'reference_generation')):
        return None
    return stamp


def consumer_result_display_pose(result):
    """Consume the pose for this completed hand frame, preserving rejection."""
    from .local_segments import consumer_display_pose
    diagnostics = result.diagnostics
    if 'live_vio_pose' not in diagnostics:
        return consumer_display_pose(diagnostics.get('vio_pose'), world_hands=True,
                                     expected_timestamp_ns=result.timestamp_ns)
    pose = diagnostics['live_vio_pose']
    world = result.world_hands
    if not isinstance(world, dict) or world.get('status') not in ('available', 'no_valid_hand_joints'):
        return None
    stamp = result_pose_timestamp(result, pose, world)
    if stamp is None:
        return None
    return consumer_display_pose(pose, world_hands=True, expected_timestamp_ns=stamp)
