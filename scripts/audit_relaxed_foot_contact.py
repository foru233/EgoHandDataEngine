"""Offline experiment: preserve torso/leg lengths when floor contact is impossible.

This does not replace the customer's result. Feet are unobserved, so a relaxed
floor contact is an inferred alternative, recorded explicitly in diagnostics.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from egohand.body.two_bone_ik import solve_two_bone_ik


def transform(row):
    body = row.get('body')
    if not body:
        return 0.
    diagnostic = body.get('diagnostics') or {}
    adjustment = float(diagnostic.get('pelvis_ground_adjustment_m') or 0.)
    if adjustment <= 1e-7:
        return 0.
    joints = body['joints']
    for name in ('Pelvis', 'LeftHip', 'RightHip'):
        p = np.asarray(joints[name], dtype=float)
        p[1] -= adjustment
        joints[name] = p.tolist()
    maximum_lift = 0.
    for side in ('Left', 'Right'):
        hip = np.asarray(joints[side + 'Hip'], dtype=float)
        old_knee = np.asarray(joints[side + 'Knee'], dtype=float)
        old_ankle = np.asarray(joints[side + 'Ankle'], dtype=float)
        old_hip = hip + [0., adjustment, 0.]
        upper = np.linalg.norm(old_knee - old_hip)
        lower = np.linalg.norm(old_ankle - old_knee)
        knee, ankle, _, _ = solve_two_bone_ik(hip, old_ankle, upper, lower,
                                              old_knee - old_hip, min_bend=0.)
        joints[side + 'Knee'] = knee.tolist()
        joints[side + 'Ankle'] = ankle.tolist()
        maximum_lift = max(maximum_lift, float(old_ankle[1] - ankle[1]))
    diagnostic['experimental_foot_contact_relaxed'] = True
    diagnostic['experimental_ankle_lift_m'] = maximum_lift
    diagnostic['experimental_foot_contact_is_observed'] = False
    diagnostic['ground_contact'] = maximum_lift < 1e-4
    body['diagnostics'] = diagnostic
    return maximum_lift


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    if args.source.resolve() == args.destination.resolve():
        raise ValueError('input and output must differ')
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    maximum = 0.
    with args.source.open(encoding='utf8') as src, args.destination.open('x', encoding='utf8') as dst:
        for line in src:
            row = json.loads(line)
            lift = transform(row)
            count += int(lift > 1e-4)
            maximum = max(maximum, lift)
            dst.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'frames_with_relaxed_contact': count, 'max_lift_m': maximum}))


if __name__ == '__main__':
    main()
