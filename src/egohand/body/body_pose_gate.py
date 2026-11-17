"""Conservative gate for combining a trusted head pose and two 3D hands."""
from dataclasses import dataclass, field
import numpy as np


@dataclass(frozen=True)
class BodyGateResult:
    status: str
    reason: str
    head_transform: np.ndarray | None = None
    hands: dict = field(default_factory=dict)
    reference_key: tuple | None = None
    diagnostics: dict = field(default_factory=dict)

    @property
    def strict_valid(self):
        return self.status == "strict_valid"


def _valid_hand(hand):
    if not isinstance(hand, dict):
        return False
    xyz = np.asarray(hand.get("xyz_world_m"), dtype=float)
    valid = np.asarray(hand.get("valid"), dtype=bool)
    if xyz.shape != (21, 3) or valid.shape != (21,):
        return False
    required = [0, 5, 9, 13, 17]
    return all(bool(valid[i]) and np.isfinite(xyz[i]).all() for i in required)


def _reference_key(pose, world):
    ref = world.get("world_reference") if isinstance(world, dict) else {}
    ref = ref if isinstance(ref, dict) else {}
    values = []
    for key in ("map_epoch", "world_frame_revision", "reference_generation"):
        value = pose.get(key)
        other = ref.get(key)
        if other is not None and value is not None and other != value:
            return None, "reference_mismatch"
        values.append(value if value is not None else other)
    segment = pose.get("display_segment") or (pose.get("local_motion") or {}).get("segment_id")
    return (pose.get("map_id"), pose.get("reset_id"), *values, segment), None


def evaluate_body_gate(result, pose, *, strict=True, now_ns=None, max_age_ns=250_000_000):
    """Validate evidence before any body geometry is generated.

    ``consumer_display_pose(..., world_hands=True)`` remains the authority for
    admitted map pose.  This gate only combines already validated inputs and
    never turns a prediction, candidate or stale pose into a trusted one.
    """
    from ..vio.local_segments import consumer_display_pose
    if result is None or not isinstance(pose, dict):
        return BodyGateResult("waiting_head", "missing_head")
    stamp = getattr(result, "timestamp_ns", None)
    frame_id = getattr(result, "frame_id", None)
    if type(stamp) is not int or stamp < 0 or type(frame_id) is not int:
        return BodyGateResult("out_of_sync", "invalid_result_timestamp")
    if now_ns is not None and (type(now_ns) is not int or now_ns < stamp or now_ns - stamp > max_age_ns):
        return BodyGateResult("stale", "input_expired")
    diagnostics = getattr(result, 'diagnostics', {})
    if isinstance(diagnostics, dict) and 'live_vio_pose' in diagnostics:
        # The live hand timestamp is host-clock time; the pose timestamp is
        # sensor-clock time. The existing exact-capture binding verifies both.
        from ..vio.hand_pose_binding import consumer_result_display_pose
        transform = consumer_result_display_pose(result)
    else:
        transform = consumer_display_pose(pose, world_hands=True, expected_timestamp_ns=stamp)
    if transform is None:
        return BodyGateResult("waiting_head", "head_pose_untrusted")
    if pose.get("pose_source") in ("imu_prediction", "visual_candidate", "frozen") or pose.get("display_source") in ("imu_prediction", "visual_candidate", "frozen"):
        return BodyGateResult("waiting_head", "head_pose_not_visual_committed")
    world = getattr(result, "world_hands", None)
    if not isinstance(world, dict) or world.get("status") not in ("available", "no_valid_hand_joints"):
        reference_key, _ = _reference_key(pose, {})
        return BodyGateResult("waiting_left_hand" if strict else "estimated_preview", "hands_unavailable",
                              transform, reference_key=reference_key, diagnostics={"head_valid": True})
    if world.get("frame_id") != frame_id or world.get("timestamp_ns") != stamp:
        return BodyGateResult("out_of_sync", "frame_timestamp_mismatch", transform)
    if world.get("session_id") != getattr(result, "session_id", None):
        return BodyGateResult("out_of_sync", "session_mismatch", transform)
    reference_key, error = _reference_key(pose, world)
    if error:
        return BodyGateResult("out_of_sync", error, transform)
    hands = {}
    for hand in world.get("hands", []):
        label = hand.get("handedness") if isinstance(hand, dict) else None
        if label in ("Left", "Right") and label not in hands and _valid_hand(hand):
            hands[label] = hand
    missing = [side for side in ("Left", "Right") if side not in hands]
    if strict and missing:
        return BodyGateResult("waiting_left_hand" if missing == ["Left"] else
                              "waiting_right_hand" if missing == ["Right"] else "estimated_preview",
                              "missing_" + "_and_".join(s.lower() for s in missing), transform,
                              hands, reference_key, {"missing": missing})
    status = "strict_valid" if not missing else "estimated_preview"
    return BodyGateResult(status, "ok" if not missing else "partial_hands", transform, hands,
                          reference_key, {"missing": missing, "head_valid": True})
