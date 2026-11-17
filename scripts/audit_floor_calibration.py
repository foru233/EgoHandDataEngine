"""Isolated offline test of a fixed floor estimated per VIO reference.

The floor comes from the full sequence, not from a ground measurement.  Never
replace a saved result with this experiment without visual acceptance.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from egohand.body.two_bone_ik import solve_two_bone_ik


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    if args.source.resolve() == args.destination.resolve():
        raise ValueError('input and output must differ')
    with args.source.open(encoding='utf8') as stream:
        rows = list(map(json.loads, stream))
    floor_raise = {}
    for row in rows:
        body = row.get('body')
        if body:
            key = json.dumps(body.get('reference_key'), sort_keys=True)
            shift = float(body['diagnostics'].get('pelvis_ground_adjustment_m') or 0.)
            # Leave a little knee flexion at the highest observed head pose.
            # A perfectly straight two-bone chain is singular: millimetres of
            # head movement can otherwise create centimetres of knee motion.
            floor_raise[key] = max(floor_raise.get(key, 0.), shift + .015)
    max_error = 0.
    for row in rows:
        body = row.get('body')
        if not body:
            continue
        diag, joints = body['diagnostics'], body['joints']
        shift = float(diag.get('pelvis_ground_adjustment_m') or 0.)
        raise_m = floor_raise[json.dumps(body.get('reference_key'), sort_keys=True)]
        for name in ('Pelvis', 'LeftHip', 'RightHip'):
            pos = np.asarray(joints[name], float)
            pos[1] -= shift
            joints[name] = pos.tolist()
        for side in ('Left', 'Right'):
            hip = np.asarray(joints[side + 'Hip'], float)
            knee_old = np.asarray(joints[side + 'Knee'], float)
            ankle_old = np.asarray(joints[side + 'Ankle'], float)
            hip_old = hip + [0., shift, 0.]
            upper, lower = np.linalg.norm(knee_old - hip_old), np.linalg.norm(ankle_old - knee_old)
            target = ankle_old.copy()
            target[1] -= raise_m
            vertical = float(target[1] - hip[1])
            horizontal = target - hip
            horizontal[1] = 0.
            # Keep the foot on the fixed plane while avoiding a perfectly
            # straight chain.  Horizontal foot placement is not observed.
            max_reach = (upper + lower) * .995
            horizontal_limit = np.sqrt(max(0., max_reach**2 - vertical**2))
            horizontal_norm = float(np.linalg.norm(horizontal))
            if horizontal_norm > horizontal_limit and horizontal_norm > 1e-9:
                horizontal *= horizontal_limit / horizontal_norm
                target[[0, 2]] = (hip + horizontal)[[0, 2]]
            forward = np.asarray(diag.get('pelvis_forward_world', [0., 0., 1.]), float)
            forward[1] = 0.
            norm = float(np.linalg.norm(forward))
            forward = forward / norm if norm > 1e-8 else np.array([0., 0., 1.])
            displacement = target - hip
            # Both knees bend in the same forward sagittal plane, including
            # the initially straight-leg singularity.
            pole = forward * displacement[1] - np.array([0., 1., 0.]) * float(displacement @ forward)
            knee, ankle, _, _ = solve_two_bone_ik(hip, target, upper, lower,
                                                   pole, min_bend=0.)
            joints[side + 'Knee'], joints[side + 'Ankle'] = knee.tolist(), ankle.tolist()
            max_error = max(max_error, abs(float(ankle[1] - target[1])))
        diag['ground_y_m'] -= raise_m
        diag['experimental_floor_raise_m'] = raise_m
        diag['experimental_floor_is_observed'] = False
        diag['pelvis_ground_adjustment_m'] = 0.
        diag['torso_length_prior_relaxed'] = False
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    with args.destination.open('x', encoding='utf8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'reference_floor_raises_m': list(floor_raise.values()),
                      'max_ankle_floor_error_m': max_error}))


if __name__ == '__main__':
    main()
