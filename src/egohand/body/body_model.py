"""Small parametric skeleton used only for visualization.

All matrices use ``T_world_from_camera`` style naming.  The default mount
parameter is an explicit approximation: ``T_camera_from_head`` maps a point
from the head frame to the camera optical frame.  It is not a measured
calibration.
"""
from dataclasses import dataclass, field
import numpy as np


BODY_PRESET = {
    "name": "adult_male_175cm",
    "height_m": 1.75,
    "shoulder_width_m": 0.42,
    "upper_arm_length_m": 0.30,
    "forearm_length_m": 0.26,
    # Chest-to-pelvis segment; neck and upper chest are modeled separately.
    # Counting them twice made the nominal 175 cm model about 10 cm too tall.
    "torso_length_m": 0.45,
    "leg_length_m": 0.90,
}


def _matrix(value):
    out = np.asarray(value, dtype=float)
    if out.shape != (4, 4) or not np.isfinite(out).all():
        raise ValueError("transforms must be finite 4x4 matrices")
    if not np.allclose(out[3], [0, 0, 0, 1], rtol=0, atol=1e-6):
        raise ValueError("transform bottom row must be [0,0,0,1]")
    return out.copy()


@dataclass(frozen=True)
class BodyPose:
    """One displayable estimate; inferred joints are never labelled measured."""

    frame_id: int
    timestamp_ns: int
    reference_key: tuple | None
    status: str
    joints: dict[str, np.ndarray]
    measured_joints: frozenset[str] = frozenset()
    bones: tuple[tuple[str, str], ...] = ()
    diagnostics: dict = field(default_factory=dict)

    @property
    def strict_valid(self):
        return self.status == "strict_valid"

    def copy_joints(self):
        return {name: value.copy() for name, value in self.joints.items()}


class BodyModel:
    """Parametric adult skeleton dimensions and camera mount convention."""

    BONE_NAMES = (
        ("Head", "Neck"), ("Neck", "CervicalBase"), ("CervicalBase", "Chest"), ("Chest", "Pelvis"),
        ("CervicalBase", "LeftShoulder"), ("LeftShoulder", "LeftElbow"),
        ("LeftElbow", "LeftWrist"), ("CervicalBase", "RightShoulder"),
        ("RightShoulder", "RightElbow"), ("RightElbow", "RightWrist"),
        ("Pelvis", "LeftHip"), ("LeftHip", "LeftKnee"),
        ("LeftKnee", "LeftAnkle"), ("Pelvis", "RightHip"),
        ("RightHip", "RightKnee"), ("RightKnee", "RightAnkle"),
    )

    def __init__(self, preset=None, *, t_camera_from_head=None):
        values = dict(BODY_PRESET)
        if preset:
            values.update(preset)
        for key in BODY_PRESET:
            if key == "name":
                continue
            value = float(values[key])
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"invalid body parameter: {key}")
            values[key] = value
        self.preset = values
        # The body Head point is the forehead midpoint bound to the camera
        # optical origin.  It is intentionally not a head-center offset.
        if t_camera_from_head is None:
            default_mount = np.eye(4)
            t_camera_from_head = default_mount
        self.t_camera_from_head = _matrix(t_camera_from_head)

    @property
    def t_head_from_camera(self):
        return np.linalg.inv(self.t_camera_from_head)

    @property
    def bones(self):
        return self.BONE_NAMES

    def with_mount_offset(self, xyz_m):
        t = self.t_camera_from_head.copy()
        value = np.asarray(xyz_m, dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError("mount offset must be a finite 3-vector")
        t[:3, 3] = value
        return BodyModel(self.preset, t_camera_from_head=t)
