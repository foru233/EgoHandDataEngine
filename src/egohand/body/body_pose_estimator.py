"""Head and two-hand driven lightweight body pose estimation."""
import numpy as np

from .body_model import BodyModel, BodyPose
from .body_pose_gate import evaluate_body_gate
from .two_bone_ik import solve_two_bone_ik



def _unit(value, fallback):
    value = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(value))
    return np.asarray(fallback, dtype=float) if not np.isfinite(norm) or norm < 1e-8 else value / norm


def _transform_point(matrix, point):
    return matrix[:3, :3] @ np.asarray(point, dtype=float) + matrix[:3, 3]


class BodyPoseEstimator:
    """Stateful display estimator; state is discarded at reference changes."""

    def __init__(self, model=None):
        self.model = model or BodyModel()
        self._reference_key = None
        self._torso_rotation = None
        self._last_pose = None
        self._ground_y = None
        self._ground_base_y = None
        self.ground_offset_m = 0.0
        self._reference_head_y = None
        self._held_arms = {}
        from ..vio.hand_identity_stabilizer import HandIdentityStabilizer
        self._identity = HandIdentityStabilizer()

    def max_ground_raise_m(self):
        """Maximum nominal rise, retaining a small hip-to-ankle clearance."""
        return max(0.0, float(self.model.preset['leg_length_m']) - .005)

    def reset(self):
        self._reference_key = None
        self._torso_rotation = None
        self._last_pose = None
        self._ground_y = None
        self._ground_base_y = None
        self._reference_head_y = None
        self._held_arms.clear()
        self._identity.reset()

    def update(self, result, pose, *, strict_gate=True, now_ns=None):
        gate = evaluate_body_gate(result, pose, strict=strict_gate, now_ns=now_ns)
        if gate.head_transform is None:
            self._last_pose = None
            return None, gate
        if gate.reference_key != self._reference_key:
            self._reference_key = gate.reference_key
            self._torso_rotation = None
            self._last_pose = None
            self._ground_y = None
            self._ground_base_y = None
            self._reference_head_y = None
            self._held_arms.clear()
        if strict_gate and not gate.strict_valid:
            self._last_pose = None
            return None, gate
        estimate = self._estimate(result, gate)
        self._last_pose = estimate
        return estimate, gate

    def _estimate(self, result, gate):
        T_wc = gate.head_transform
        # Head is the forehead midpoint: bind it directly to the measured
        # camera pose instead of placing it behind the camera as a skull
        # center.  This makes the displayed forward axis agree with the
        # image/camera direction.
        R_head = T_wc[:3, :3].copy()
        head = T_wc[:3, 3].copy()
        foot_thickness = 0.025
        if self._reference_head_y is None:
            self._reference_head_y = float(head[1])
        # Keep the body upright above the horizontal XY reference plane.
        down = np.array([0., 1., 0.])
        head_right = R_head[:, 0]
        right = head_right - down * float(np.dot(head_right, down))
        right = _unit(right, [1., 0., 0.])
        forward = _unit(np.cross(right, down), [0., 0., -1.])
        # Keep torso orientation separate from head rotation and smooth only
        # the inferred prior.  Hand targets remain current measurements.
        # A small bounded lean is an inferred prior, not a measurement of the
        # spine.  It lets chest and pelvis move with head pitch/roll without
        # letting a head turn rotate the entire body abruptly.
        head_down = R_head[:, 1]
        horizontal = head_down - down * np.dot(head_down, down)
        lean = horizontal * .30
        lean_length = np.linalg.norm(lean)
        if lean_length > .25:
            lean *= .25 / lean_length
        # The camera is handheld in the supplied recording. Do not transfer
        # its yaw/roll into the torso: an upright body must not twist around
        # the world vertical axis. Head orientation remains available to the
        # neck/head model and hands retain their measured directions.
        torso_R = np.column_stack((np.array([1., 0., 0.]),
                                   np.array([0., 1., 0.]),
                                   np.array([0., 0., 1.])))
        self._torso_rotation = torso_R
        right, down, forward = torso_R[:, 0], torso_R[:, 1], torso_R[:, 2]
        p = self.model.preset
        torso = p["torso_length_m"]
        shoulder_w = p["shoulder_width_m"]
        # Head height is only a relative anchor; it is not a floor or height measurement.
        # Camera optical +Y points toward the neck in the default mount.
        # Rotate this fixed head-local offset with the actual head orientation.
        neck = head + R_head @ np.array([0., .12, -.08])
        # Estimate a smoothly curved neck between the observed head and the
        # slower torso prior. Limit the head-driven lateral deflection rather
        # than forcing the neck base to remain vertical.
        head_neck = _unit(R_head[:, 1], down)
        mixed = _unit(.55 * head_neck + .45 * down, down)
        lateral = mixed - down * float(np.dot(mixed, down))
        lateral_norm = float(np.linalg.norm(lateral))
        if lateral_norm > .5:
            lateral *= .5 / lateral_norm
        neck_direction = _unit(down + lateral, down)
        neck_base = neck + neck_direction * .08
        chest = neck_base + down * (.08 * torso / .55)
        pelvis = chest + down * torso
        left_shoulder = neck_base - right * (shoulder_w / 2)
        right_shoulder = neck_base + right * (shoulder_w / 2)
        # Conservative adult-male hip spacing; avoid an exaggerated split
        # stance when only head and hands constrain the body.
        hip_w = shoulder_w * .72
        # Preserve the inferred pelvis tilt. Only the shared leg bending plane
        # uses a horizontal heading; this is not a level-pelvis constraint.
        world_down = np.array([0., 1., 0.])
        pelvis_right = _unit(right - world_down * np.dot(right, world_down), [1., 0., 0.])
        pelvis_forward = _unit(np.cross(pelvis_right, world_down), [0., 0., 1.])
        left_hip = pelvis - right * (hip_w / 2)
        right_hip = pelvis + right * (hip_w / 2)
        if self._ground_base_y is None:
            # The reference is established from the actual initial hip
            # estimate, avoiding a crouch caused by omitting neck lengths.
            self._ground_base_y = float((left_hip[1]+right_hip[1]) / 2 + p['leg_length_m'] + foot_thickness)
        self._ground_y = self._ground_base_y + self.ground_offset_m
        # A relaxed neutral arm hangs beside the torso, with a slight outward
        # angle.  Keep the two modeled bone lengths instead of using a short
        # wrist target, which made the IK fold the arm upwards.
        upper = p["upper_arm_length_m"]
        forearm = p["forearm_length_m"]
        left_dir = _unit(down - .08 * right, down)
        right_dir = _unit(down + .08 * right, down)
        default_left = left_shoulder + left_dir * (upper + forearm)
        default_right = right_shoulder + right_dir * (upper + forearm)
        measured = set()
        targets = {"Left": default_left, "Right": default_right}
        # Use the exact camera-frame hand object that the GUI hand overlay
        # renders, then apply the same admitted T_world_from_camera.  This
        # prevents a stale/alternate world_hands serialization from creating
        # an orange body wrist disconnected from the cyan recognized hand.
        from ..vio.hand_identity_stabilizer import StableHand
        stable_hands, _ = self._identity.update(getattr(result, "hands", ()))
        recognized = {}
        recognized_points = []
        recognized_tracks = {}
        for hand in stable_hands:
            side = getattr(hand, "handedness", None)
            joints = getattr(hand, "joints", ())
            if side in ("Left", "Right") and side not in recognized and joints:
                joint = joints[0]
                if getattr(joint, "valid", False):
                    wrist = np.asarray(getattr(joint, "xyz_m", [np.nan] * 3), dtype=float)
                    if wrist.shape == (3,) and np.isfinite(wrist).all():
                        world_wrist = _transform_point(T_wc, wrist)
                        recognized[side] = world_wrist
                        recognized_points.append((side, world_wrist))
                        track=getattr(hand, 'track_id', None)
                        if type(track) is int and track > 0:
                            recognized_tracks[track]=(side, world_wrist)
        gate_tracks=[h.get('track_id') for h in gate.hands.values()]
        for side, hand in gate.hands.items():
            reference = np.asarray(hand["xyz_world_m"], dtype=float)[0]
            wrist = recognized.get(side)
            track=hand.get('track_id')
            if type(track) is int and track > 0 and gate_tracks.count(track)==1 and track in recognized_tracks:
                side,wrist=recognized_tracks[track]
            elif wrist is None and recognized_points and np.isfinite(reference).all():
                # Gate keys can retain the detector's raw side. Match its
                # world observation to the stable hand, then use that stable
                # side for both the target and measured-joint bookkeeping.
                match_side, match_point = min(recognized_points,
                    key=lambda p: float(np.linalg.norm(p[1]-reference)))
                if np.linalg.norm(match_point-reference) <= .18:
                    side, wrist = match_side, match_point
            if wrist is None:
                wrist = np.asarray(hand["xyz_world_m"], dtype=float)[0]
            if np.isfinite(wrist).all():
                targets[side] = wrist.copy(); measured.add(side + "Wrist")
                self._held_arms[side] = (int(result.timestamp_ns), wrist.copy())
        held=[]
        for side in ('Left','Right'):
            if side+'Wrist' in measured or side not in self._held_arms:continue
            stamp, wrist = self._held_arms[side]
            age = int(result.timestamp_ns)-stamp
            # A lost hand retains its last estimated posture until a new
            # observation or a reference reset; it is not a new measurement.
            if age >= 0:
                targets[side] = wrist.copy()
                held.append(side)
        def pole_for(name, origin, fallback):
            previous = self._last_pose
            if previous is None or name not in previous.joints:
                return fallback
            vector = previous.joints[name] - origin
            return vector if np.linalg.norm(vector) > 1e-5 else fallback

        left_elbow, left_reachable_wrist, _, left_diag = solve_two_bone_ik(
            left_shoulder, targets["Left"], p["upper_arm_length_m"], p["forearm_length_m"],
            pole_for('LeftElbow', left_shoulder, down + right * .35))
        right_elbow, right_reachable_wrist, _, right_diag = solve_two_bone_ik(
            right_shoulder, targets["Right"], p["upper_arm_length_m"], p["forearm_length_m"],
            pole_for('RightElbow', right_shoulder, down - right * .35))
        if "LeftWrist" not in measured and "Left" not in held:
            left_elbow = left_shoulder + left_dir * upper
        if "RightWrist" not in measured and "Right" not in held:
            right_elbow = right_shoulder + right_dir * upper
        # The wrist is a real stereo observation whenever available.  IK may
        # clamp its internal target to obtain a stable elbow, but must never
        # replace the observed wrist with that clamped point.
        left_wrist = targets["Left"]
        right_wrist = targets["Right"]
        leg = p["leg_length_m"]
        leg_bone = leg * .5
        # Feet may move horizontally as the floor approaches hip height.
        # Keep a continuous reachable hip-to-ankle distance while respecting
        # the knee flexion limit. Y is down in the estimator's source frame.
        min_reach = leg * np.cos(np.deg2rad(135.) / 2.)
        ankle_y = self._ground_y - foot_thickness
        horizontal_forward = pelvis_forward
        # Ground adjustment owns the ankle height. When the head/torso prior
        # puts the hips outside leg reach, move the inferred pelvis as one unit
        # instead of releasing the feet from the floor or stretching the legs.
        # This preserves pelvis tilt and measured head/hand targets; the spine
        # length becomes a relaxed display prior, not a measured body segment.
        initial_verticals = [float(ankle_y-hip[1]) for hip in (left_hip,right_hip)]
        pelvis_shift = float(np.clip(0.,max(v-leg for v in initial_verticals),min(initial_verticals)))
        pelvis = pelvis + world_down * pelvis_shift
        left_hip = left_hip + world_down * pelvis_shift
        right_hip = right_hip + world_down * pelvis_shift
        # Without foot observations, keep a neutral shared foot depth. Solving
        # each ankle's forward offset independently makes pelvis roll synthesize
        # one foot in front of the other. Different hip heights are still free
        # to produce different knee flexion on the common ground plane.
        verticals = [float(ankle_y-hip[1]) for hip in (left_hip, right_hip)]
        forward_offset = max((np.sqrt(max(0., (min_reach+1e-5)**2-max(0.,v)**2))
                              for v in verticals if -1e-6 <= v <= leg+1e-6), default=0.)
        forward_offset = min(forward_offset,min(np.sqrt(max(0.,leg**2-v**2)) for v in verticals))
        def leg_pose(hip):
            vertical = float(ankle_y-hip[1])
            # Both legs share a foot depth. The lower hip's minimum knee reach
            # can conflict with the higher hip's maximum reach; in that case
            # relax the bend-angle prior, retaining ankle height and bone lengths.
            reached_vertical = vertical
            ankle = hip + horizontal_forward * forward_offset
            ankle[1] = hip[1] + reached_vertical
            feasible = True
            # Both knees bend in the current body's sagittal plane. A previous
            # knee minus a newly translated hip can point backwards, especially
            # near a straight leg. Do not use that world-space history as a pole.
            # At a nearly horizontal thigh/ankle target, choose the upper side
            # of the floor explicitly instead of generic IK's downward fallback.
            knee_pole = (horizontal_forward * reached_vertical
                         - world_down * forward_offset)
            if np.linalg.norm(knee_pole) < 1e-8:
                knee_pole = horizontal_forward
            distance = float(np.linalg.norm(ankle-hip))
            flexion_limit = 135. if distance >= min_reach else 179.999
            knee, _, _, diag = solve_two_bone_ik(
                hip, ankle, leg_bone, leg_bone,
                knee_pole, max_flexion_deg=flexion_limit, min_bend=0.)
            diag['ground_reachable'] = feasible
            diag['ground_error_m'] = float(ankle[1]-ankle_y)
            diag['flexion_prior_relaxed'] = flexion_limit > 135.
            return knee, ankle, diag
        left_knee, left_ankle, left_leg_diag = leg_pose(left_hip)
        right_knee, right_ankle, right_leg_diag = leg_pose(right_hip)
        vertical_delta = float(head[1] - self._reference_head_y)
        joints = {
            "Head": head, "Neck": neck, "CervicalBase": neck_base, "Chest": chest, "Pelvis": pelvis,
            "LeftShoulder": left_shoulder, "LeftElbow": left_elbow, "LeftWrist": left_wrist,
            "RightShoulder": right_shoulder, "RightElbow": right_elbow, "RightWrist": right_wrist,
            "LeftHip": left_hip, "LeftKnee": left_knee, "LeftAnkle": left_ankle,
            "RightHip": right_hip, "RightKnee": right_knee, "RightAnkle": right_ankle,
        }
        return BodyPose(
            frame_id=int(result.frame_id), timestamp_ns=int(result.timestamp_ns),
            reference_key=gate.reference_key, status=gate.status, joints=joints,
            measured_joints=frozenset(measured), bones=self.model.bones,
            diagnostics={"gate_reason": gate.reason, "ik": {"Left": left_diag, "Right": right_diag,
                                                               "LeftLeg": left_leg_diag, "RightLeg": right_leg_diag},
                         "reachable_wrist_targets": {"Left": left_reachable_wrist.tolist(),
                                                      "Right": right_reachable_wrist.tolist()},
                         "t_camera_from_head": self.model.t_camera_from_head.tolist(),
                         "head_rotation_world": R_head.tolist(),
                         "body_preset": dict(self.model.preset),
                         "inferred_joints": sorted(set(joints) - measured),
                         "torso_lean_source": "disabled_canonical_upright",
                         "neck_source": "bounded_head_torso_blend_prior",
                         "pelvis_source": "tilted_torso_prior_no_pelvis_observation",
                         "leg_bend_source": "shared_forward_sagittal_prior",
                         "pelvis_forward_world": pelvis_forward.tolist(),
                         "ground_contact": bool(left_leg_diag['ground_reachable'] and right_leg_diag['ground_reachable']),
                         "foot_thickness_m": foot_thickness,
                         "held_hands": held,
                         "held_hands_are_measurements": False,
                         "ground_y_source": "session_relative_default",
                         "ground_y_m": self._ground_y,
                         "ankle_ground_distance_m": foot_thickness,
                         "ground_contact_source": "enforced_display_constraint_not_observation",
                         "pelvis_ground_adjustment_m": pelvis_shift,
                         "torso_length_prior_relaxed": abs(pelvis_shift) > 1e-8,
                         "vertical_head_delta_m": vertical_delta,
                         "posture_adjusted": bool(abs(vertical_delta) > .02)},
        )
