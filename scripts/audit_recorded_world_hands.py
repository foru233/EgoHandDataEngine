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
        pose = (row.get('diagnostics') or {}).get('vio_pose') or {}
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
                'Uses the same consumer admission validator as the production pipeline']}


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
    with args.output.open('x', encoding='utf8') as target:
        json.dump(report, target, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if report['errors'] else 0


if __name__ == '__main__':
    sys.exit(main())
