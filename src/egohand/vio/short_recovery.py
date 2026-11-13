"""Validate frame-bound native stereo geometry; no recovery state is kept here."""
import math
import numpy as np
from .map_identity import (snapshot_identity_matches_epoch,invalid_reference_frame,
                           decode_map_epoch,invalid_world_revision_transition)
from .trajectory import rigid_transform


def visual_reference(frame):
    """An exact existing reference; this never derives a transform between maps."""
    ref=frame.get('reference_frame') or {}
    return tuple(frame.get(k) for k in ('map_id','reset_id','map_revision',
        'world_frame_revision','reference_generation'))+(ref.get('atlas_generation'),)


def geometry_candidate(observation, *, timestamp_ns, map_epoch, reference_generation,
                       candidate_body=None):
    """Check native candidate evidence before provisional tracker acceptance."""
    if not isinstance(observation,dict):return False
    o=observation
    if (type(o.get('schema_version')) is not int or o['schema_version']!=1 or
        o.get('source')!='native_stereo_map_geometry_v1' or o.get('enabled') is not True or
        o.get('available') is not True or o.get('qualified') is not True):return False
    integers=('frame_id','timestamp_ns','map_id','reset_id','map_revision',
              'atlas_generation','reference_generation','geometry_sequence',
              'geometric_inliers','stereo_inliers','occupied_cells','associated')
    if any(type(o.get(k)) is not int or o[k]<0 for k in integers):return False
    if type(map_epoch) is not int or not 0<=map_epoch<2**64:return False
    try:epoch=decode_map_epoch(map_epoch)
    except (TypeError,ValueError,OverflowError):return False
    if (type(timestamp_ns) is not int or o['timestamp_ns']!=timestamp_ns or
        any(o[k]!=epoch[k] for k in ('map_id','reset_id','map_revision')) or
        type(reference_generation) is not int or o['reference_generation']!=reference_generation):return False
    if (o['geometric_inliers']<50 or o['stereo_inliers']<30 or
        not 6<=o['occupied_cells']<=12 or o['geometric_inliers']*2<o['associated'] or
        o['stereo_inliers']>o['geometric_inliers'] or o['geometric_inliers']>o['associated']):return False
    p=o.get('stereo_p95_normalized')
    if type(p) not in (int,float) or not math.isfinite(p) or not 0<=p<=3:return False
    if o.get('phase') not in ('local_map','short_imu_search','relocalization'):return False
    if candidate_body is not None:
        try:
            body=rigid_transform(np.asarray(o['T_world_from_body']).reshape(4,4))
            candidate=rigid_transform(candidate_body)
            if not np.allclose(body,candidate,atol=2e-5,rtol=1e-5):return False
        except (KeyError,TypeError,ValueError,OverflowError):return False
    return True


def geometry_matches_frame(frame,observation,expected_reference):
    """Final qualification requires the returned pose and native snapshot identity."""
    if (not isinstance(expected_reference,tuple) or visual_reference(frame)!=expected_reference or
        not isinstance(observation,dict) or observation.get('pose_binding_verified') is not True or
        not snapshot_identity_matches_epoch(frame) or invalid_reference_frame(None,frame) or
        invalid_world_revision_transition(None,frame.get('world_frame_revision')) or
        frame.get('tracking_state')!='OK' or frame.get('pose_valid') is not True):return False
    if not geometry_candidate(observation,timestamp_ns=frame.get('timestamp_ns'),
        map_epoch=frame.get('map_epoch'),reference_generation=frame.get('reference_generation')):return False
    o=observation;ref=frame.get('reference_frame') or {}
    if any(o[k]!=frame.get(k) for k in ('frame_id','map_id','reset_id','map_revision','reference_generation')):return False
    if (o['atlas_generation']!=ref.get('atlas_generation') or
        o['geometry_sequence']!=ref.get('geometry_sequence')):return False
    return True
