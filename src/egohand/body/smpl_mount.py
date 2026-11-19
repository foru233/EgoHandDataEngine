"""Explicit geometric camera mount on the SMPL forehead surface."""
import numpy as np

HEAD_ANCHOR_VERSION='forehead_surface_v2'
DEFAULT_CAMERA_FOREHEAD_OFFSET_MM=7.5


def camera_forehead_offset_m(millimetres):
    value=float(millimetres)
    if not 0. <= value <= 100.:
        raise ValueError('相机至额头垂直距离必须在 0–100 mm 之间')
    return value / 1000.


def forehead_target(camera_pose, offset_m):
    """Move from the camera centre along its optical +Y (mount down)."""
    return camera_pose[..., :3, 3] + camera_pose[..., :3, 1] * offset_m


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
