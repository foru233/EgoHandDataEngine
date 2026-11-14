"""Audit frame-bound hand publication in a saved offline worker result stream.

Checks the recorded contract and transform, not ground-truth pose or hand accuracy.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.vio.localization_state import consumer_map_pose


def independently_check_publication(row, world, pose):
    """Cross-check recorded producer evidence without the consumer validator.

    This is a second implementation of a limited set of necessary invariants,
    not an independent ground-truth measurement or full recovery-protocol proof.
    """
    errors = []
    evidence = pose.get('localization')
    recovery = pose.get('recovery_preaccept')
    if not isinstance(evidence, dict) or not isinstance(recovery, dict):
        return ['independent_contract_invalid']
    admission, map_state = evidence.get('admission'), evidence.get('map')
    if not isinstance(admission, dict) or not isinstance(map_state, dict):
        return ['independent_contract_invalid']
    if (pose.get('pose_valid') is not True or pose.get('tracking_state') != 'OK'
            or pose.get('snapshot_consistent') is not True
            or admission.get('world_hands_permitted') is not True
            or admission.get('map_pose_available') is not True
            or evidence.get('quarantined') is not False
            or map_state.get('status') != 'estimated'
            or map_state.get('camera_frame') != 'gui_left_camera'):
        errors.append('independent_pose_or_admission_invalid')
    if (recovery.get('await_visual_requalification') is not False
            or recovery.get('application_pose_admitted') is not True
            or recovery.get('stage') != 'committed'):
        errors.append('independent_recovery_not_committed_or_quarantined')
    if (pose.get('timestamp_ns') != row.get('timestamp_ns')
            or pose.get('frame_id') != row.get('frame_id')
            or recovery.get('timestamp_ns') != row.get('timestamp_ns')
            or recovery.get('frame_id') != row.get('frame_id')
            or world.get('pose_source') != 'orb_slam3_committed_map'):
        errors.append('independent_pose_frame_or_source_invalid')
    for field in ('timestamp_ns', 'map_epoch', 'world_frame_revision', 'reference_generation'):
        if evidence.get(field) != pose.get(field) or recovery.get(field) != pose.get(field):
            errors.append('independent_identity_mismatch:' + field)
    try:
        transform = np.asarray(pose['matrix'], dtype=float)
        mapped = np.asarray(map_state['T_map_from_camera'], dtype=float)
        if (transform.shape != (4, 4) or mapped.shape != (4, 4)
                or not np.isfinite(transform).all() or not np.array_equal(transform, mapped)
                or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-6, rtol=0)
                or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-4, rtol=0)
                or abs(np.linalg.det(transform[:3, :3]) - 1.0) > 1e-4):
            errors.append('independent_transform_invalid')
    except (KeyError, TypeError, ValueError):
        errors.append('independent_transform_invalid')
    return errors


def audit(rows):
    counts, errors = Counter(), []
    max_error = 0.0
    joints_checked = 0
    for row in rows:
        counts['frames'] += 1
        world = row.get('world_hands')
        if not isinstance(world, dict):
            errors.append({'frame_id': row.get('frame_id'), 'reason': 'missing_world_result'})
            continue
        status = world.get('status')
        counts['world_'+str(status)] += 1
        pose = (row.get('diagnostics') or {}).get('vio_pose') or {}
        recovery = pose.get('recovery_preaccept') or {}
        flags = {'invalid_pose': pose.get('pose_valid') is not True,
                 'quarantined': recovery.get('await_visual_requalification') is True,
                 'lost_tracking': pose.get('tracking_state') in ('LOST', 'RECENTLY_LOST')}
        for name, active in flags.items():
            if active:
                counts[name + '_frames'] += 1
                if status == 'available':
                    counts[name + '_with_world_available'] += 1
                    errors.append({'frame_id': row.get('frame_id'), 'reason': name + '_with_world_available'})
        if status != 'available':
            for hand in world.get('hands') or []:
                valid, xyz = hand.get('valid'), hand.get('xyz_world_m')
                # no_valid_hand_joints legitimately carries explicitly invalid
                # hand skeletons; it must never contain usable coordinates.
                if (not isinstance(valid, list) or any(item is not False for item in valid)
                        or not isinstance(xyz, list)
                        or any(not isinstance(point, list) or any(v is not None for v in point) for point in xyz)):
                    errors.append({'frame_id': row.get('frame_id'), 'reason': 'unavailable_contains_world_coordinates'})
            continue
        def fail(reason):
            errors.append({'frame_id': row.get('frame_id'), 'reason': reason})
        for reason in independently_check_publication(row, world, pose):
            fail(reason)
        transform = consumer_map_pose(pose, world_hands=True, expected_timestamp_ns=row.get('timestamp_ns'))
        if transform is None:
            fail('available_without_admitted_pose')
            continue
        if (world.get('timestamp_ns') != row.get('timestamp_ns') or
                world.get('frame_id') != row.get('frame_id') or
                world.get('pose_timestamp_ns') != row.get('timestamp_ns')):
            fail('world_frame_or_timestamp_mismatch')
        ref = world.get('world_reference') or {}
        if any(ref.get(k) != pose.get(k) for k in ('map_epoch', 'world_frame_revision', 'reference_generation')):
            fail('world_reference_mismatch')
        if ref.get('cross_frame_merge_permitted') is not False:
            fail('unverified_cross_frame_merge_permission')
        segment = row.get('segment_hands') or {}
        local = row.get('local_motion') or {}
        if segment.get('status') == 'available' and (
                segment.get('cross_segment_merge_permitted') is not False or
                segment.get('segment_id') != local.get('segment_id')):
            fail('segment_identity_or_merge_permission_invalid')
        report = pose.get('recovery_preaccept') or {}
        join = world.get('join') or {}
        if (join.get('recovery_session_id') != report.get('session_id') or
                join.get('recovery_frame_sequence') != report.get('frame_sequence') or
                join.get('source_frame_index') != row.get('frame_id')):
            fail('world_recovery_binding_mismatch')
        camera_hands = {(h.get('track_id'), h.get('handedness')): h for h in row.get('hands', [])}
        frame_joints_checked = 0
        for hand in world.get('hands', []):
            camera = camera_hands.get((hand.get('track_id'), hand.get('handedness')))
            xyz, valid = hand.get('xyz_world_m'), hand.get('valid')
            if camera is None or not isinstance(xyz, list) or not isinstance(valid, list) or len(xyz) != 21 or len(valid) != 21:
                fail('world_hand_shape_or_identity_invalid')
                continue
            joints = camera.get('joints') or []
            if len(joints) != 21:
                fail('camera_hand_shape_invalid')
                continue
            for index, item in enumerate(valid):
                if item is not True:
                    if item is not False or not isinstance(xyz[index], list) or any(v is not None for v in xyz[index]):
                        fail('invalid_world_joint_contains_coordinates')
                    continue
                try:
                    if joints[index].get('valid') is not True:
                        raise ValueError('world joint lacks valid camera joint')
                    point = np.asarray(joints[index]['xyz_m'], dtype=float)
                    measured = np.asarray(xyz[index], dtype=float)
                    if point.shape != (3,) or measured.shape != (3,) or not np.isfinite(point).all() or not np.isfinite(measured).all():
                        raise ValueError('invalid point')
                    error = float(np.linalg.norm(transform[:3, :3] @ point + transform[:3, 3] - measured))
                    max_error = max(max_error, error)
                    joints_checked += 1
                    frame_joints_checked += 1
                    if error > 1e-8:
                        fail('world_transform_mismatch')
                except (KeyError, TypeError, ValueError):
                    fail('world_joint_invalid')
        if not frame_joints_checked:
            fail('available_without_valid_world_joint')
    return {'counts': dict(counts), 'valid_world_joints_checked': joints_checked,
            'max_transform_error_m': max_error if joints_checked else None,
            'errors': errors,
            'limitations': ['Recorded contract and algebraic consistency only; no independent physical ground truth',
                'Cannot certify synchronization, absolute accuracy or cross-map continuity',
                'Includes an independent limited invariant check, but also uses the production consumer validator',
                'Cannot prove actual physical sensor alignment or correctness of the underlying estimated pose']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    digest = hashlib.sha256()
    with args.frames.open('rb') as source:
        def rows():
            for line in source:
                digest.update(line)
                if line.strip(): yield json.loads(line)
        report = audit(rows())
    report['frames_sha256'] = digest.hexdigest()
    report['auditor_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with args.output.open('x', encoding='utf8') as target:
        json.dump(report, target, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if report['errors'] else 0


if __name__ == '__main__':
    sys.exit(main())
