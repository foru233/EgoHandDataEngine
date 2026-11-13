"""Record observed identity changes and the actual gate decision, without approval."""
from .map_identity import decode_map_epoch


class MapRevisionAudit:
    def __init__(self):
        self._previous = None

    def observe(self, result):
        current = {'timestamp_ns': result.get('timestamp_ns'), 'map_epoch': result.get('map_epoch'),
                   'world_frame_revision': result.get('world_frame_revision'),
                   'reference_generation': result.get('reference_generation')}
        previous = self._previous
        self._previous = current
        if previous is None or (previous['map_epoch'] == current['map_epoch']
                                and previous['world_frame_revision'] == current['world_frame_revision']
                                and previous['reference_generation'] == current['reference_generation']):
            return None
        try:
            old = decode_map_epoch(previous['map_epoch'])
            new = decode_map_epoch(current['map_epoch'])
            if old['map_id'] != new['map_id'] or old['reset_id'] != new['reset_id']:
                event = 'map_or_reset_changed'
            elif previous['world_frame_revision'] != current['world_frame_revision']:
                event = 'world_revision_changed'
            elif previous['reference_generation'] != current['reference_generation']:
                event = 'inertial_reference_changed'
            else:
                event = 'map_revision_changed'
        except (ValueError, TypeError, OverflowError):
            event = 'identity_unknown'
        return {'event': event, 'before': previous, 'after': current,
                'gate_action': 'accepted' if result.get('pose_valid') else 'rejected',
                'quality_status': result.get('quality_status'),
                'jump_detected': bool(result.get('jump_detected')),
                'coordinate_continuity': 'unverified',
                'reason': 'partial_native_events_do_not_cover_all_map_updates'}
