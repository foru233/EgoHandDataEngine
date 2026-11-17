"""Bounded two-bone inverse kinematics with a stable pole vector."""
import numpy as np


def _unit(value, fallback):
    value = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(value))
    if not np.isfinite(norm) or norm < 1e-9:
        return np.asarray(fallback, dtype=float)
    return value / norm


def solve_two_bone_ik(shoulder, wrist, upper_length, forearm_length, pole,
                      *, min_bend=1e-4, max_flexion_deg=150.):
    """Return elbow, wrist_target, reachability and a diagnostic dictionary.

    The target is clamped to the annulus that the two bones can reach.  The
    returned elbow lies in the plane selected by ``pole`` and remains stable
    when the target approaches a singular straight-arm configuration.
    """
    shoulder = np.asarray(shoulder, dtype=float)
    wrist = np.asarray(wrist, dtype=float)
    pole = np.asarray(pole, dtype=float)
    if any(v.shape != (3,) or not np.isfinite(v).all()
           for v in (shoulder, wrist, pole)):
        raise ValueError("IK inputs must be finite 3-vectors")
    a = float(upper_length); b = float(forearm_length)
    if not np.isfinite(a + b) or a <= 0 or b <= 0:
        raise ValueError("bone lengths must be positive")
    delta = wrist - shoulder
    distance = float(np.linalg.norm(delta))
    direction = _unit(delta, [0., 0., 1.])
    if not np.isfinite(max_flexion_deg) or not 0 < max_flexion_deg < 180:
        raise ValueError('max_flexion_deg must be between 0 and 180')
    min_reach = max(np.sqrt(a*a+b*b+2*a*b*np.cos(np.deg2rad(max_flexion_deg))),
                    abs(a - b) + min_bend, 1e-6)
    max_reach = max(a + b - min_bend, min_reach)
    clamped_distance = float(np.clip(distance, min_reach, max_reach))
    target = shoulder + direction * clamped_distance
    # Build a pole direction orthogonal to the arm.  Near collinearity uses a
    # deterministic world-independent fallback to avoid elbow flipping.
    pole_perp = pole - direction * float(np.dot(pole, direction))
    if np.linalg.norm(pole_perp) < 1e-8:
        fallback = np.array([0., 1., 0.])
        if abs(float(np.dot(fallback, direction))) > .9:
            fallback = np.array([1., 0., 0.])
        pole_perp = fallback - direction * float(np.dot(fallback, direction))
    bend = _unit(pole_perp, [0., 1., 0.])
    along = (a * a - b * b + clamped_distance * clamped_distance) / (2 * clamped_distance)
    height = float(np.sqrt(max(0., a * a - along * along)))
    elbow = shoulder + direction * along + bend * height
    return elbow, target, bool(abs(distance - clamped_distance) < 1e-6), {
        "target_distance_m": distance,
        "clamped_distance_m": clamped_distance,
        "reachable": bool(min_reach <= distance <= max_reach),
        "clamped": bool(abs(distance - clamped_distance) >= 1e-6),
    }
