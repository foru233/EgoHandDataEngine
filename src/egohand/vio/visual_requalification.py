"""Experimental stereo/map geometry check; never a recovery release decision.

Inputs are rectified stereo correspondences against an independently acquired,
immutable map snapshot. This module cannot establish their provenance, static
scene status, snapshot atomicity, or world-reference continuity. It is not
connected to the production recovery coordinator or monitor.
"""
from dataclasses import dataclass
import base64
import hashlib
import math

import cv2
import numpy as np

from .trajectory import rigid_transform


@dataclass(frozen=True)
class AppearanceMatchLimits:
    """Explicit research settings for an extractor independent of Tracking."""
    features: int
    scale_factor: float
    levels: int
    edge_threshold: int
    fast_threshold: int
    ratio: float
    max_hamming: float
    max_epipolar_px: float
    min_disparity_px: float
    max_disparity_px: float

    def __post_init__(self):
        for name in ('features', 'levels', 'edge_threshold', 'fast_threshold'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError('invalid appearance extractor counts')
        for name in ('scale_factor', 'ratio', 'max_hamming', 'max_epipolar_px',
                     'min_disparity_px', 'max_disparity_px'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError('invalid appearance matching limits')
        if (self.scale_factor <= 1 or not 0 < self.ratio < 1 or not 0 < self.max_hamming <= 256
                or self.max_epipolar_px <= 0 or not 0 < self.min_disparity_px < self.max_disparity_px):
            raise ValueError('invalid appearance matching limits')


_APPEARANCE_DTYPE = np.dtype([('id', '<u8'), ('position', '<f4', (3,)),
                             ('descriptor', 'u1', (32,)), ('observations', '<u4')])


def _appearance_reference_valid(value):
    if 'reference_generation' not in value and 'reference_frame' not in value:
        return True  # Historical research samples retain explicitly unknown reference.
    generation, report = value.get('reference_generation'), value.get('reference_frame')
    if (type(generation) is not int or not 0 <= generation < 2**64
            or not isinstance(report, dict) or report.get('frame_bound') is not True
            or report.get('snapshot_consistent') is not True or type(report.get('schema_version')) is not int
            or report['schema_version'] != 1
            or report.get('basis') != 'successful_initializer_commit'
            or report.get('coordinate_continuity_verified') is not False):
        return False
    for key in ('frame_id', 'timestamp_ns', 'map_id', 'reset_id', 'map_revision',
                'map_change_index', 'atlas_generation'):
        if type(report.get(key)) is not int or report[key] != value.get(key):
            return False
    for key in ('generation', 'operation_sequence', 'geometry_sequence'):
        if type(report.get(key)) is not int or not 0 <= report[key] < 2**64:
            return False
    return (report['generation'] == generation
            and (generation == 0) == (report['operation_sequence'] == 0))


def decode_map_appearance(value, *, bridge_instance_id, settings_sha256):
    """Validate a frozen numeric sample. Provenance is caller-bound, not authenticated.

    All-writer atomicity and static scene/world continuity remain unverified.
    No runtime object, address or Tracking match list crosses this interface.
    """
    if not isinstance(value, dict) or value.get('status') != 'copied':
        raise ValueError('map appearance unavailable')
    if (value.get('schema_version') != 1 or value.get('struct_size') != 136
            or value.get('point_layout') != 'u64_f32x3_u8x32_u32_le_v1'
            or value.get('point_stride') != _APPEARANCE_DTYPE.itemsize
            or value.get('source_kind') != 1 or value.get('coverage') != 'partial'
            or value.get('advisory_only') is not True or value.get('production_release_authorized') is not False
            or not bridge_instance_id or value.get('bridge_instance_id') != bridge_instance_id
            or not isinstance(settings_sha256, str) or len(settings_sha256) != 64
            or value.get('settings_sha256') != settings_sha256):
        raise ValueError('map appearance provenance or layout mismatch')
    names = ('point_count', 'sampled_count', 'capacity', 'total_map_points', 'frame_id', 'map_id',
             'reset_id', 'atlas_generation', 'map_revision', 'map_change_index', 'world_frame_revision',
             'timestamp_ns', 'input_timestamp_ns', 'image_width', 'image_height', 'flags')
    if any(isinstance(value.get(n), bool) or not isinstance(value.get(n), int) or value[n] < 0 for n in names):
        raise ValueError('invalid map appearance identity')
    if not _appearance_reference_valid(value):
        raise ValueError('invalid map appearance reference binding')
    if (not 0 <= value['point_count'] <= value['sampled_count'] <= value['capacity'] <= 4096
            or value['capacity'] < 1 or value['sampled_count'] > value['total_map_points']
            or value['flags'] not in (1, 3)
            or bool(value['flags'] & 2) != (value['total_map_points'] > value['sampled_count'])
            or value['timestamp_ns'] != value['input_timestamp_ns']
            or value['timestamp_ns'] <= 0 or min(value['image_width'], value['image_height']) <= 0
            or value.get('input_image_size') != [value['image_width'], value['image_height']]):
        raise ValueError('invalid map appearance counts, time or image size')
    scalars = [value.get(k) for k in ('fx', 'fy', 'cx', 'cy', 'bf')]
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in scalars):
        raise ValueError('invalid map appearance calibration')
    if min(value['fx'], value['fy'], value['bf']) <= 0:
        raise ValueError('invalid map appearance calibration')
    encoded = value.get('points_base64')
    if not isinstance(encoded, str) or len(encoded) > 4096 * 56 * 4 // 3 + 4:
        raise ValueError('invalid map appearance payload')
    raw = base64.b64decode(encoded, validate=True)
    if len(raw) != value['point_count'] * _APPEARANCE_DTYPE.itemsize:
        raise ValueError('map appearance payload size mismatch')
    points = np.frombuffer(raw, dtype=_APPEARANCE_DTYPE)
    if (len(np.unique(points['id'])) != len(points) or not np.isfinite(points['position']).all()
            or (points['observations'] < 2).any()):
        raise ValueError('invalid map appearance points')
    return points


def _mutual_descriptor_matches(query, train, limits):
    """Two-way ratio test rejects ties and repeated/ambiguous descriptors."""
    if query is None or train is None or len(query) < 2 or len(train) < 2:
        return {}
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    def accepted(a, b):
        return {m[0].queryIdx: m[0].trainIdx for m in matcher.knnMatch(a, b, k=2)
                if len(m) == 2 and m[0].distance <= limits.max_hamming
                and m[0].distance < limits.ratio * m[1].distance}
    forward, reverse = accepted(query, train), accepted(train, query)
    return {q: t for q, t in forward.items() if reverse.get(t) == q}


def match_map_appearance_stereo(*, appearance, left_gray, right_gray, timestamp_ns,
                                bridge_instance_id, settings_sha256, limits,
                                camera_matrix, baseline_m, frame_binding=None):
    """Extract from actual pixels and match without any candidate pose or projection.

    Same-frame mode requires byte-identical native input images. A separate
    frame_binding explicitly selects a frozen earlier map and later images;
    content revisions may differ but continuity is never inferred from this.
    Neither candidate pose nor Tracking matches are accepted as matching inputs.
    """
    if not isinstance(limits, AppearanceMatchLimits):
        raise TypeError('limits must be AppearanceMatchLimits')
    points = decode_map_appearance(appearance, bridge_instance_id=bridge_instance_id,
                                   settings_sha256=settings_sha256)
    binding = appearance if frame_binding is None else frame_binding
    if not isinstance(binding, dict):
        raise ValueError('invalid image frame binding')
    if frame_binding is not None:
        if not _appearance_reference_valid(binding):
            raise ValueError('invalid image reference binding')
        if (('reference_generation' in appearance) != ('reference_generation' in binding)
                or appearance.get('reference_generation') != binding.get('reference_generation')):
            raise ValueError('frozen map/image identity mismatch: reference_generation')
        for name in ('bridge_instance_id', 'settings_sha256', 'map_id', 'reset_id', 'atlas_generation', 'world_frame_revision'):
            if type(binding.get(name)) is not type(appearance[name]) or binding.get(name) != appearance[name]:
                raise ValueError('frozen map/image identity mismatch: '+name)
        for name in ('frame_id', 'timestamp_ns', 'map_revision', 'map_change_index'):
            if (isinstance(binding.get(name), bool) or not isinstance(binding.get(name), int)
                    or binding[name] < appearance[name]):
                raise ValueError('frozen map/image order mismatch: '+name)
        if binding['frame_id'] == appearance['frame_id'] or binding['timestamp_ns'] == appearance['timestamp_ns']:
            raise ValueError('frozen map requires a later image frame')
        if binding.get('input_image_size') != appearance['input_image_size']:
            raise ValueError('frozen map image size mismatch')
    if (isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int)
            or timestamp_ns != binding.get('timestamp_ns')
            or timestamp_ns != binding.get('input_timestamp_ns')):
        raise ValueError('map appearance image timestamp mismatch')
    width, height = appearance['image_width'], appearance['image_height']
    K = np.asarray(camera_matrix, dtype=float)
    actual_K = np.array([[appearance['fx'], 0, appearance['cx']],
                         [0, appearance['fy'], appearance['cy']], [0, 0, 1.]])
    if (K.shape != (3, 3) or not np.isfinite(K).all()
            or not np.allclose(K, actual_K, rtol=1e-6, atol=1e-5)
            or isinstance(baseline_m, bool) or not math.isfinite(baseline_m) or baseline_m <= 0
            or not math.isclose(appearance['bf']/appearance['fx'], baseline_m, rel_tol=1e-6, abs_tol=1e-7)):
        raise ValueError('map appearance calibration mismatch')
    for name, pixels in (('left', left_gray), ('right', right_gray)):
        if (not isinstance(pixels, np.ndarray) or pixels.dtype != np.uint8
                or pixels.shape != (height, width)
                or hashlib.sha256(np.ascontiguousarray(pixels).tobytes()).hexdigest() != binding.get(name+'_sha256')):
            raise ValueError('map appearance input image mismatch: '+name)
    orb = cv2.ORB_create(nfeatures=limits.features, scaleFactor=limits.scale_factor,
        nlevels=limits.levels, edgeThreshold=limits.edge_threshold,
        fastThreshold=limits.fast_threshold)
    left_keys, left_desc = orb.detectAndCompute(left_gray, None)
    right_keys, right_desc = orb.detectAndCompute(right_gray, None)
    map_left = _mutual_descriptor_matches(np.ascontiguousarray(points['descriptor']), left_desc, limits)
    left_right = _mutual_descriptor_matches(left_desc, right_desc, limits)
    matches = []
    for m, l in sorted(map_left.items()):
        r = left_right.get(l)
        if r is None:
            continue
        u, v = left_keys[l].pt
        ur, vr = right_keys[r].pt
        if (abs(v-vr) <= limits.max_epipolar_px
                and limits.min_disparity_px <= u-ur <= limits.max_disparity_px):
            matches.append((m, l, r))
    indices = [m for m, _, _ in matches]
    left = np.array([left_keys[l].pt for _, l, _ in matches], dtype=float).reshape(-1, 2)
    right = np.array([right_keys[r].pt for _, _, r in matches], dtype=float).reshape(-1, 2)
    return {'point_ids': [int(i) for i in points['id'][indices]],
        'map_points': points['position'][indices].astype(float), 'left_pixels': left, 'right_pixels': right,
        'matching_source': 'independent_opencv_orb_pixels_and_map_descriptors',
        'candidate_pose_used_for_matching': False, 'tracking_correspondences_used': False,
        'map_left_mutual_count': len(map_left), 'left_right_mutual_count': len(left_right),
        'left_features': len(left_keys), 'right_features': len(right_keys),
        'map_snapshot_frame_id': appearance['frame_id'], 'image_frame_id': binding['frame_id'],
        'map_snapshot_revision': appearance['map_revision'], 'image_map_revision': binding['map_revision'],
        'reference_generation': appearance.get('reference_generation'),
        'reference_identity_known': 'reference_generation' in appearance,
        'frozen_map_reference': frame_binding is not None, 'coordinate_continuity_verified': False,
        'advisory_only': True, 'production_release_authorized': False}


@dataclass(frozen=True)
class ExperimentalLimits:
    """Caller-specified research parameters, not validated deployment thresholds."""
    min_correspondences: int
    min_inliers: int
    min_inlier_ratio: float
    ransac_reprojection_px: float
    max_stereo_p95_px: float
    max_translation_difference_m: float
    max_rotation_difference_deg: float
    min_occupied_grid_cells: int
    min_geometry_singular_ratio: float

    def __post_init__(self):
        if (isinstance(self.min_correspondences, bool) or isinstance(self.min_inliers, bool)
                or not isinstance(self.min_correspondences, int) or not isinstance(self.min_inliers, int)
                or self.min_correspondences < 6 or not 6 <= self.min_inliers <= self.min_correspondences
                or isinstance(self.min_occupied_grid_cells, bool)
                or not isinstance(self.min_occupied_grid_cells, int)
                or not 1 <= self.min_occupied_grid_cells <= 16):
            raise ValueError('invalid experimental counts')
        for value in (self.min_inlier_ratio, self.ransac_reprojection_px, self.max_stereo_p95_px,
                      self.max_translation_difference_m, self.max_rotation_difference_deg,
                      self.min_geometry_singular_ratio):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('invalid experimental limits')
        if not (0 < self.min_inlier_ratio <= 1 and self.ransac_reprojection_px > 0
                and self.max_stereo_p95_px > 0 and self.max_translation_difference_m > 0
                and self.max_rotation_difference_deg > 0 and 0 < self.min_geometry_singular_ratio <= 1):
            raise ValueError('invalid experimental limits')


def _result(status, reason, **metrics):
    return {'schema_version': 1, 'status': status, 'reason': reason,
            'advisory_only': True, 'production_release_authorized': False,
            'independence_verified': False, 'snapshot_atomicity_verified': False,
            'coordinate_continuity_verified': False, 'metrics': metrics}


def _identity(value):
    if not isinstance(value, dict):
        return None
    keys = ('map_id', 'reset_id', 'map_revision', 'world_frame_revision')
    if any(isinstance(value.get(k), bool) or not isinstance(value.get(k), int) or value[k] < 0 for k in keys):
        return None
    optional = ('atlas_generation', 'reference_generation')
    if any(k in value and (type(value[k]) is not int or not 0 <= value[k] < 2**64) for k in optional):
        return None
    return tuple(value[k] for k in keys) + tuple(value.get(k) for k in optional)


def evaluate_stereo_map_geometry(*, map_identity, frame_identity, snapshot_consistent,
                                 timestamp_ns, point_ids, map_points, left_pixels, right_pixels,
                                 camera_matrix, baseline_m, image_size, candidate_T_map_from_camera,
                                 limits, allow_frozen_content=False):
    """Compare independent stereo PnP geometry with one candidate camera pose.

    ``map_points`` are Nx3 metres in the snapshot's map frame, and pixels are
    Nx2 at the same rectified resolution (right camera is +X by baseline and
    both rectified cameras share the same focal lengths and principal point).
    Both identities require matching map/reset/content/world revisions. An
    external trusted producer must bind every correspondence to this snapshot
    and timestamp; passing a bool or IDs here does not itself authenticate it.
    No result from this research-only function may clear quarantine.
    """
    if not isinstance(limits, ExperimentalLimits):
        raise TypeError('limits must be ExperimentalLimits')
    expected, actual = _identity(map_identity), _identity(frame_identity)
    if expected is None or actual is None:
        return _result('unavailable', 'identity_missing_or_invalid')
    if not isinstance(allow_frozen_content, bool):
        raise TypeError('allow_frozen_content must be bool')
    compatible = expected == actual
    if allow_frozen_content:
        compatible = (expected[:2] == actual[:2] and expected[3:] == actual[3:] and expected[2] <= actual[2])
    if not compatible:
        return _result('rejected', 'identity_mismatch')
    if snapshot_consistent is not True:
        return _result('unavailable', 'snapshot_not_consistent')
    if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or timestamp_ns < 0:
        return _result('unavailable', 'timestamp_invalid')
    try:
        points = np.asarray(map_points, dtype=np.float64)
        left = np.asarray(left_pixels, dtype=np.float64)
        right = np.asarray(right_pixels, dtype=np.float64)
        ids = list(point_ids)
        K = np.asarray(camera_matrix, dtype=np.float64)
        candidate = rigid_transform(candidate_T_map_from_camera)
        width, height = image_size
        baseline = float(baseline_m)
        if (isinstance(width, bool) or isinstance(height, bool) or not isinstance(width, int)
                or not isinstance(height, int) or width <= 0 or height <= 0
                or isinstance(baseline_m, bool) or not math.isfinite(baseline) or baseline <= 0):
            raise ValueError('invalid image geometry')
        n = len(points)
        if (points.shape != (n, 3) or left.shape != (n, 2) or right.shape != (n, 2)
                or len(ids) != n or any(isinstance(i, bool) or not isinstance(i, int) or i < 0 for i in ids)
                or len(set(ids)) != n or K.shape != (3, 3)):
            raise ValueError('invalid correspondence shape or duplicate map point')
        if not all(np.isfinite(x).all() for x in (points, left, right, K, candidate)):
            raise ValueError('nonfinite data')
        if (K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1])
                or not np.isclose(K[0, 1], 0) or not np.isclose(K[1, 0], 0)):
            raise ValueError('unsupported intrinsic matrix')
        if any((uv[:, 0] < 0).any() or (uv[:, 0] >= width).any()
               or (uv[:, 1] < 0).any() or (uv[:, 1] >= height).any() for uv in (left, right)):
            raise ValueError('pixel outside image')
    except (ValueError, TypeError, OverflowError, cv2.error):
        return _result('unavailable', 'invalid_geometry_input')
    if n < limits.min_correspondences:
        return _result('unavailable', 'insufficient_correspondences', correspondences=n)
    try:
        ok, rv, tv, indices = cv2.solvePnPRansac(
            points, left, K, None, iterationsCount=300,
            reprojectionError=limits.ransac_reprojection_px, confidence=0.999,
            flags=cv2.SOLVEPNP_EPNP)
        if not ok or indices is None:
            return _result('rejected', 'pnp_failed')
        indices = np.asarray(indices, dtype=int).reshape(-1)
        if (len(indices) < limits.min_inliers or len(indices) / n < limits.min_inlier_ratio
                or len(np.unique(indices)) != len(indices)):
            return _result('rejected', 'insufficient_pnp_inliers', inliers=len(indices), correspondences=n)
        p, l, r = points[indices], left[indices], right[indices]
        singular = np.linalg.svd(p - p.mean(axis=0), compute_uv=False)
        ratio = float(singular[-1] / singular[0]) if singular[0] > 0 else 0.0
        if ratio < limits.min_geometry_singular_ratio:
            return _result('rejected', 'degenerate_map_geometry', geometry_singular_ratio=ratio)
        cells = {(min(3, int(x * 4 / width)), min(3, int(y * 4 / height))) for x, y in l}
        if len(cells) < limits.min_occupied_grid_cells:
            return _result('rejected', 'insufficient_image_coverage', occupied_cells=len(cells))
        rv, tv = cv2.solvePnPRefineLM(p, l, K, None, rv, tv)
        R, _ = cv2.Rodrigues(rv)
        pc = (R @ p.T + tv.reshape(3, 1)).T
        if not np.isfinite(pc).all() or (pc[:, 2] <= 0).any():
            return _result('rejected', 'nonpositive_depth')
        projected_l = np.column_stack((K[0, 0]*pc[:, 0]/pc[:, 2]+K[0, 2], K[1, 1]*pc[:, 1]/pc[:, 2]+K[1, 2]))
        projected_r = projected_l.copy()
        projected_r[:, 0] -= K[0, 0]*baseline / pc[:, 2]
        stereo_error = np.maximum(np.linalg.norm(projected_l-l, axis=1), np.linalg.norm(projected_r-r, axis=1))
        p95 = float(np.percentile(stereo_error, 95))
        estimated_rotation = R.T
        estimated_position = -estimated_rotation @ tv.reshape(3)
        translation = float(np.linalg.norm(estimated_position-candidate[:3, 3]))
        relative = estimated_rotation.T @ candidate[:3, :3]
        angle = float(np.degrees(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))))
        metrics = {'correspondences': n, 'inliers': len(indices), 'inlier_ratio': len(indices)/n,
                   'frozen_content_comparison': allow_frozen_content,
                   'image_frame_identity': dict(frame_identity),
                   'stereo_reprojection_p95_px': p95, 'translation_difference_m': translation,
                   'rotation_difference_deg': angle, 'occupied_cells': len(cells),
                   'geometry_singular_ratio': ratio, 'timestamp_ns': timestamp_ns,
                   'map_identity': {k: map_identity[k] for k in ('map_id', 'reset_id', 'map_revision',
                       'world_frame_revision', 'atlas_generation', 'reference_generation') if k in map_identity}}
        if not all(math.isfinite(v) for v in (p95, translation, angle)):
            return _result('unavailable', 'nonfinite_result')
        if p95 > limits.max_stereo_p95_px:
            return _result('rejected', 'stereo_reprojection_inconsistent', **metrics)
        if translation > limits.max_translation_difference_m or angle > limits.max_rotation_difference_deg:
            return _result('rejected', 'candidate_pose_inconsistent', **metrics)
        return _result('geometry_consistent', 'research_check_only', **metrics)
    except (ValueError, TypeError, OverflowError, cv2.error, np.linalg.LinAlgError):
        return _result('unavailable', 'geometry_solver_failed')
