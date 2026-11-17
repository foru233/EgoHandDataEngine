import copy
import importlib.util
from pathlib import Path

import numpy as np


spec = importlib.util.spec_from_file_location(
    'offline_arm_transition', Path(__file__).resolve().parents[1] / 'scripts' / 'build_offline_arm_transition.py'
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def _row(frame, wrist, measured=False, reference='a'):
    return {'frame_id': frame, 'body': {'reference_key': reference,
        'measured_joints': ['RightWrist'] if measured else [],
        'joints': {'RightShoulder': [0., 0., 0.], 'RightElbow': [0.2, 0.2, 0.],
                   'RightWrist': wrist}}}


def test_transition_only_changes_inferred_arm_and_preserves_lengths():
    rows = [_row(0, [0.4, 0., 0.]), _row(1, [0.4, 0., 0.]),
            _row(2, [0.2, 0., 0.3], measured=True)]
    original = copy.deepcopy(rows)
    result = module.apply(rows)
    assert rows == original
    assert result[2] == original[2]
    for before, after in zip(original[:2], result[:2]):
        old, new = before['body']['joints'], after['body']['joints']
        for a, b in [('RightShoulder', 'RightElbow'), ('RightElbow', 'RightWrist')]:
            assert np.isclose(np.linalg.norm(np.subtract(old[a], old[b])),
                              np.linalg.norm(np.subtract(new[a], new[b])))
        assert after['body']['diagnostics']['offline_arm_anticipation']['estimated']


def test_transition_does_not_cross_reference_or_missing_body():
    rows = [_row(0, [0.4, 0., 0.], reference='a'), {'frame_id': 1, 'body': None},
            _row(2, [0.4, 0., 0.], reference='a'),
            _row(3, [0.2, 0., 0.3], measured=True, reference='b')]
    assert module.apply(rows) == rows
