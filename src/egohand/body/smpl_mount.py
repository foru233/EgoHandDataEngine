"""Explicit geometric camera mount on the SMPL forehead surface."""
import numpy as np

HEAD_ANCHOR_VERSION='forehead_surface_v2'


def forehead_vertex(vertices,head_weights):
    """Choose the upper face centerline, not a fixed offset from head joint.

    SMPL's head joint lies near the skull base. Adding 8 cm selected the
    nasal bridge. Use the upper quarter of the actual head surface instead:
    approximately 6 cm below the crown, above the brow/eye band. Geometry
    remains a mounting assumption, not an observed anatomical landmark.
    """
    v=np.asarray(vertices,float);ids=np.flatnonzero(np.asarray(head_weights)>.5)
    if len(ids)<20:raise ValueError('SMPL head surface missing')
    lo=v[ids].min(0);hi=v[ids].max(0);center=(lo+hi)/2
    target_y=hi[1]-.25*(hi[1]-lo[1])
    candidates=ids[(np.abs(v[ids,0]-center[0])<.02)&
        (v[ids,1]>hi[1]-.35*(hi[1]-lo[1]))&(v[ids,2]>center[2])]
    if not len(candidates):raise ValueError('SMPL forehead center surface missing')
    score=(v[candidates,0]-center[0])**2+(v[candidates,1]-target_y)**2
    return int(candidates[np.argmin(score)])
